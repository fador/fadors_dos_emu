#include "HIMEM.hpp"
#include <algorithm>

namespace fador::memory {

HIMEM::HIMEM() {
    m_xms.resize(XMS_SIZE, 0);
    m_blocks.push_back({0, XMS_SIZE, false}); // One big free block
}

uint16_t HIMEM::allocate(uint32_t size) {
    for (size_t i = 0; i < m_blocks.size(); ++i) {
        if (!m_blocks[i].used && m_blocks[i].size >= size) {
            uint32_t start = m_blocks[i].start;
            m_blocks[i].used = true;
            m_blocks[i].size = size;
            // Split block if needed
            if (m_blocks[i].size > size) {
                m_blocks.insert(m_blocks.begin() + i + 1, {start + size, m_blocks[i].size - size, false});
            }
            return static_cast<uint16_t>(i + 1); // Handles start at 1
        }
    }
    return 0;
}

bool HIMEM::free(uint16_t handle) {
    if (handle == 0 || handle > m_blocks.size()) return false;
    m_blocks[handle - 1].used = false;
    // Merge adjacent free blocks
    for (size_t i = 0; i + 1 < m_blocks.size(); ) {
        if (!m_blocks[i].used && !m_blocks[i + 1].used) {
            m_blocks[i].size += m_blocks[i + 1].size;
            m_blocks.erase(m_blocks.begin() + i + 1);
        } else {
            ++i;
        }
    }
    return true;
}

uint8_t* HIMEM::getBlock(uint16_t handle) {
    if (handle == 0 || handle > m_blocks.size() || !m_blocks[handle - 1].used) return nullptr;
    return &m_xms[m_blocks[handle - 1].start];
}

uint32_t HIMEM::available() const {
    uint32_t total = 0;
    for (const auto& b : m_blocks) {
        if (!b.used) total += b.size;
    }
    return total;
}

bool HIMEM::handleXMSInterrupt(uint8_t ah, uint16_t& result) {
    // Simulate INT 2Fh/43h (XMS API)
    switch (ah) {
        case 0x00: // Query XMS driver
            result = 0x80; // XMS driver present
            return true;
        case 0x08: // Query free XMS
            result = static_cast<uint16_t>(available() / 1024); // KB
            return true;
        default:
            result = 0;
            return false;
    }
}

} // namespace fador::memory
