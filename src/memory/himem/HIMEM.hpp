#pragma once
#include <cstdint>
#include <vector>
#include "../MemoryBus.hpp"

namespace fador::memory {

class HIMEM {
public:
    HIMEM();
    ~HIMEM() = default;

    // XMS 2.0 API — called by XMS HLE dispatch (INT E0h)
    // Each returns AX in result, sets BL error code on failure.
    uint16_t getVersion() const { return 0x0200; }  // XMS 2.0
    uint16_t getDriverVersion() const { return 0x0100; }  // Internal rev 1.0

    // A20 gate control
    bool enableA20();
    bool disableA20();
    bool isA20Enabled() const { return m_a20Enabled; }

    // HMA (High Memory Area) — 64KB at 0x100000
    bool requestHMA(uint16_t size);
    bool releaseHMA();
    bool isHMAAllocated() const { return m_hmaAllocated; }

    // Extended Memory Blocks (EMB)
    uint16_t allocateEMB(uint16_t sizeKB);  // returns handle, 0 on failure
    bool freeEMB(uint16_t handle);
    bool lockEMB(uint16_t handle, uint32_t& linearAddr);
    bool unlockEMB(uint16_t handle);
    bool resizeEMB(uint16_t handle, uint16_t newSizeKB);

    // EMB Move (XMS function 0Bh)
    // The move struct is read from DOS memory by the caller.
    bool moveEMB(uint32_t length,
                 uint16_t srcHandle, uint32_t srcOffset,
                 uint16_t dstHandle, uint32_t dstOffset,
                 MemoryBus& memory);

    // Query free XMS — returns largest free block in KB and total free in KB
    uint16_t queryFreeKB(uint16_t& largestBlockKB) const;

    // Get raw pointer to EMB data (for ProgramLoader compatibility)
    uint8_t* getBlock(uint16_t handle);

    // Error codes (BL register)
    static constexpr uint8_t ERR_NOT_IMPLEMENTED  = 0x80;
    static constexpr uint8_t ERR_VDISK_DETECTED   = 0x81;
    static constexpr uint8_t ERR_A20_ERROR        = 0x82;
    static constexpr uint8_t ERR_NO_HMA           = 0x90;
    static constexpr uint8_t ERR_HMA_IN_USE       = 0x91;
    static constexpr uint8_t ERR_HMA_NOT_ALLOC    = 0x93;
    static constexpr uint8_t ERR_A20_STILL_ENABLED= 0x94;
    static constexpr uint8_t ERR_NO_FREE_HANDLES  = 0xA0;
    static constexpr uint8_t ERR_INVALID_HANDLE   = 0xA2;
    static constexpr uint8_t ERR_OUT_OF_MEMORY    = 0xA0;
    static constexpr uint8_t ERR_LOCK_OVERFLOW    = 0xAA;
    static constexpr uint8_t ERR_LOCK_FAILED      = 0xAB;
    static constexpr uint8_t ERR_NOT_LOCKED       = 0xAA;
    static constexpr uint8_t ERR_HANDLE_LOCKED    = 0xAB;
    static constexpr uint8_t ERR_INVALID_LENGTH   = 0xA7;

    uint8_t lastError() const { return m_lastError; }

    // Legacy compatibility
    uint32_t available() const;
    uint16_t allocate(uint32_t size);
    bool free(uint16_t handle);

    void setMemoryBus(MemoryBus* bus) { m_memBus = bus; }

private:
    static constexpr uint32_t XMS_SIZE = 16 * 1024 * 1024; // 16MB XMS
    static constexpr int MAX_HANDLES = 128;

    // The XMS pool lives in this host-side buffer (not mapped into the 1MB address space).
    // Handle 0 is reserved; handles 1..MAX_HANDLES are valid.
    std::vector<uint8_t> m_xms;

    struct EMB {
        uint32_t offset;   // offset into m_xms
        uint32_t size;     // bytes
        bool     allocated;
        uint8_t  lockCount;
    };
    std::vector<EMB> m_handles; // index 0 unused; [1..n] = handles

    bool m_a20Enabled = false;
    bool m_hmaAllocated = false;
    mutable uint8_t m_lastError = 0;
    MemoryBus* m_memBus = nullptr;

    // Free-list tracking for the XMS pool
    struct FreeBlock {
        uint32_t offset;
        uint32_t size;
    };
    std::vector<FreeBlock> m_freeBlocks;

    uint32_t allocatePool(uint32_t size);
    void freePool(uint32_t offset, uint32_t size);
};

} // namespace fador::memory
