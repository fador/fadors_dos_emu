#include "HIMEM.hpp"
#include "../../utils/Logger.hpp"
#include <algorithm>
#include <cstring>

namespace fador::memory {

HIMEM::HIMEM() {
  m_freeBlocks.push_back({0, XMS_SIZE});
  // Reserve handle slot 0 (unused sentinel)
  m_handles.push_back({0, 0, false, 0});
}

// ── Pool allocator ──────────────────────────────────────────────────────────

uint32_t HIMEM::allocatePool(uint32_t size) {
  // First-fit in free block list
  for (auto it = m_freeBlocks.begin(); it != m_freeBlocks.end(); ++it) {
    if (it->size >= size) {
      uint32_t offset = it->offset;
      if (it->size == size) {
        m_freeBlocks.erase(it);
      } else {
        it->offset += size;
        it->size -= size;
      }
      return offset;
    }
  }
  return UINT32_MAX; // allocation failed
}

void HIMEM::freePool(uint32_t offset, uint32_t size) {
  // Insert in sorted order and merge adjacent
  auto it = m_freeBlocks.begin();
  while (it != m_freeBlocks.end() && it->offset < offset)
    ++it;
  it = m_freeBlocks.insert(it, {offset, size});

  // Merge with next
  auto next = it + 1;
  if (next != m_freeBlocks.end() && it->offset + it->size == next->offset) {
    it->size += next->size;
    m_freeBlocks.erase(next);
  }
  // Merge with previous
  if (it != m_freeBlocks.begin()) {
    auto prev = it - 1;
    if (prev->offset + prev->size == it->offset) {
      prev->size += it->size;
      m_freeBlocks.erase(it);
    }
  }
}

// ── A20 Gate ────────────────────────────────────────────────────────────────

bool HIMEM::enableA20() {
  m_a20Enabled = true;
  if (m_memBus)
    m_memBus->setA20(true);
  return true;
}

bool HIMEM::disableA20() {
  m_a20Enabled = false;
  if (m_memBus)
    m_memBus->setA20(false);
  return true;
}

// ── HMA ─────────────────────────────────────────────────────────────────────

bool HIMEM::requestHMA(uint16_t /*size*/) {
  if (m_hmaAllocated) {
    m_lastError = ERR_HMA_IN_USE;
    return false;
  }
  if (!m_a20Enabled) {
    m_lastError = ERR_A20_ERROR;
    return false;
  }
  m_hmaAllocated = true;
  return true;
}

bool HIMEM::releaseHMA() {
  if (!m_hmaAllocated) {
    m_lastError = ERR_HMA_NOT_ALLOC;
    return false;
  }
  m_hmaAllocated = false;
  return true;
}

// ── EMB allocation ──────────────────────────────────────────────────────────

uint16_t HIMEM::allocateEMB(uint16_t sizeKB) {
  uint32_t sizeBytes = static_cast<uint32_t>(sizeKB) * 1024;
  if (sizeBytes == 0)
    sizeBytes = 1024; // minimum 1KB

  uint32_t offset = allocatePool(sizeBytes);
  if (offset == UINT32_MAX) {
    m_lastError = ERR_OUT_OF_MEMORY;
    return 0;
  }

  // Find a free handle slot or create one
  for (size_t i = 1; i < m_handles.size(); ++i) {
    if (!m_handles[i].allocated) {
      m_handles[i] = {offset, sizeBytes, true, 0};
      return static_cast<uint16_t>(i);
    }
  }
  if (m_handles.size() >= MAX_HANDLES) {
    freePool(offset, sizeBytes);
    m_lastError = ERR_NO_FREE_HANDLES;
    return 0;
  }
  m_handles.push_back({offset, sizeBytes, true, 0});
  return static_cast<uint16_t>(m_handles.size() - 1);
}

bool HIMEM::freeEMB(uint16_t handle) {
  if (handle == 0 || handle >= m_handles.size() ||
      !m_handles[handle].allocated) {
    m_lastError = ERR_INVALID_HANDLE;
    return false;
  }
  if (m_handles[handle].lockCount > 0) {
    m_lastError = ERR_HANDLE_LOCKED;
    return false;
  }
  freePool(m_handles[handle].offset, m_handles[handle].size);
  m_handles[handle].allocated = false;
  return true;
}

bool HIMEM::lockEMB(uint16_t handle, uint32_t &linearAddr) {
  if (handle == 0 || handle >= m_handles.size() ||
      !m_handles[handle].allocated) {
    m_lastError = ERR_INVALID_HANDLE;
    return false;
  }
  if (m_handles[handle].lockCount == 255) {
    m_lastError = ERR_LOCK_OVERFLOW;
    return false;
  }
  m_handles[handle].lockCount++;
  // Return a "linear address" = 0x100000 (1MB) + offset into XMS pool.
  // This is what real HIMEM.SYS would return for protected-mode access.
  linearAddr = 0x100000 + m_handles[handle].offset;
  return true;
}

bool HIMEM::unlockEMB(uint16_t handle) {
  if (handle == 0 || handle >= m_handles.size() ||
      !m_handles[handle].allocated) {
    m_lastError = ERR_INVALID_HANDLE;
    return false;
  }
  if (m_handles[handle].lockCount == 0) {
    m_lastError = ERR_NOT_LOCKED;
    return false;
  }
  m_handles[handle].lockCount--;
  return true;
}

bool HIMEM::resizeEMB(uint16_t handle, uint16_t newSizeKB) {
  if (handle == 0 || handle >= m_handles.size() ||
      !m_handles[handle].allocated) {
    m_lastError = ERR_INVALID_HANDLE;
    return false;
  }
  if (m_handles[handle].lockCount > 0) {
    m_lastError = ERR_HANDLE_LOCKED;
    return false;
  }
  uint32_t newSize = static_cast<uint32_t>(newSizeKB) * 1024;
  if (newSize == 0)
    newSize = 1024;

  // Simple approach: free old, allocate new, copy data
  uint32_t oldOffset = m_handles[handle].offset;
  uint32_t oldSize = m_handles[handle].size;
  uint32_t copySize = std::min(oldSize, newSize);

  uint32_t newOffset = allocatePool(newSize);
  if (newOffset == UINT32_MAX) {
    m_lastError = ERR_OUT_OF_MEMORY;
    return false;
  }

  if (m_memBus) {
    uint8_t *newPtr = m_memBus->directAccess(0x100000 + newOffset);
    uint8_t *oldPtr = m_memBus->directAccess(0x100000 + oldOffset);
    if (newPtr && oldPtr) {
      std::memcpy(newPtr, oldPtr, copySize);
    }
  }

  freePool(oldOffset, oldSize);
  m_handles[handle].offset = newOffset;
  m_handles[handle].size = newSize;
  return true;
}

// ── EMB Move ────────────────────────────────────────────────────────────────

bool HIMEM::moveEMB(uint32_t length, uint16_t srcHandle, uint32_t srcOffset,
                    uint16_t dstHandle, uint32_t dstOffset, MemoryBus &memory) {
  // Handle 0 means conventional memory (srcOffset/dstOffset is seg:off linear
  // addr)
  auto getPtr = [&](uint16_t h, uint32_t off, uint32_t len,
                    bool /*isWrite*/) -> uint8_t * {
    if (h == 0) {
      // Conventional memory: off is seg:off packed as DWORD
      // High word = segment, low word = offset
      uint16_t seg = static_cast<uint16_t>(off >> 16);
      uint16_t ofs = static_cast<uint16_t>(off & 0xFFFF);
      uint32_t linear = (static_cast<uint32_t>(seg) << 4) + ofs;
      if (linear + len > MemoryBus::MEMORY_SIZE) {
        m_lastError = ERR_INVALID_LENGTH;
        return nullptr;
      }
      return memory.directAccess(linear);
    }
    if (h >= m_handles.size() || !m_handles[h].allocated) {
      m_lastError = ERR_INVALID_HANDLE;
      return nullptr;
    }
    if (off + len > m_handles[h].size) {
      m_lastError = ERR_INVALID_LENGTH;
      return nullptr;
    }
    return memory.directAccess(0x100000 + m_handles[h].offset + off);
  };

  uint8_t *src = getPtr(srcHandle, srcOffset, length, false);
  uint8_t *dst = getPtr(dstHandle, dstOffset, length, true);
  if (!src || !dst)
    return false;

  std::memmove(dst, src, length); // memmove handles overlap
  return true;
}

// ── Query ───────────────────────────────────────────────────────────────────

uint16_t HIMEM::queryFreeKB(uint16_t &largestBlockKB) const {
  uint32_t total = 0;
  uint32_t largest = 0;
  for (const auto &fb : m_freeBlocks) {
    total += fb.size;
    if (fb.size > largest)
      largest = fb.size;
  }
  largestBlockKB =
      static_cast<uint16_t>(std::min(largest / 1024, (uint32_t)64512));
  return static_cast<uint16_t>(std::min(total / 1024, (uint32_t)64512));
}

uint32_t HIMEM::getHandleSizeKB(uint16_t handle) const {
  if (handle == 0 || handle >= m_handles.size() || !m_handles[handle].allocated)
    return 0;
  return m_handles[handle].size / 1024;
}

uint8_t HIMEM::getLockCount(uint16_t handle) const {
  if (handle == 0 || handle >= m_handles.size() || !m_handles[handle].allocated)
    return 0;
  return m_handles[handle].lockCount;
}

uint8_t HIMEM::getFreeHandles() const {
  int used = 0;
  for (const auto &h : m_handles)
    if (h.allocated)
      used++;
  int freeSlots = MAX_HANDLES - used;
  return static_cast<uint8_t>(std::max(0, std::min(255, freeSlots)));
}

uint8_t *HIMEM::getBlock(uint16_t handle) {
  if (handle == 0 || handle >= m_handles.size() || !m_handles[handle].allocated)
    return nullptr;
  if (!m_memBus)
    return nullptr;
  return m_memBus->directAccess(0x100000 + m_handles[handle].offset);
}

// ── Legacy compatibility ────────────────────────────────────────────────────

uint32_t HIMEM::available() const {
  uint32_t total = 0;
  for (const auto &fb : m_freeBlocks)
    total += fb.size;
  return total;
}

uint16_t HIMEM::allocate(uint32_t size) {
  uint16_t sizeKB =
      static_cast<uint16_t>(std::min((size + 1023) / 1024, (uint32_t)0xFFFF));
  return allocateEMB(sizeKB);
}

bool HIMEM::free(uint16_t handle) { return freeEMB(handle); }

} // namespace fador::memory
