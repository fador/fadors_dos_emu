#pragma once
#include <cstdint>
#include <vector>
#include "../MemoryBus.hpp"

namespace fador::memory {

class HIMEM {
public:
    HIMEM();
    ~HIMEM() = default;

    // Allocate XMS block, returns handle or 0 on failure
    uint16_t allocate(uint32_t size);
    // Free XMS block by handle
    bool free(uint16_t handle);
    // Get pointer to XMS block
    uint8_t* getBlock(uint16_t handle);
    // Query available XMS
    uint32_t available() const;

    // Simulate INT 2Fh/43h (XMS API)
    bool handleXMSInterrupt(uint8_t ah, uint16_t& result);

private:
    static constexpr uint32_t XMS_SIZE = 1024 * 1024 * 16; // 16MB XMS
    std::vector<uint8_t> m_xms;
    struct Block {
        uint32_t start;
        uint32_t size;
        bool used;
    };
    std::vector<Block> m_blocks;
};

} // namespace fador::memory
