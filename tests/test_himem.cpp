#include "test_framework.hpp"
#include "memory/himem/HIMEM.hpp"
#include "memory/MemoryBus.hpp"

using namespace fador::memory;

TEST_CASE("HIMEM: Memory Allocation", "[HIMEM][Memory]") {
    HIMEM himem;
    MemoryBus memoryBus;
    himem.setMemoryBus(&memoryBus);

    SECTION("A20 and HMA Management") {
        REQUIRE(himem.isA20Enabled() == false);
        REQUIRE(himem.isHMAAllocated() == false);

        // Should fail because A20 is not enabled
        REQUIRE(himem.requestHMA(0xFFFF) == false);
        REQUIRE(himem.lastError() == HIMEM::ERR_A20_ERROR);

        REQUIRE(himem.enableA20() == true);
        REQUIRE(himem.isA20Enabled() == true);

        // Now HMA should work
        REQUIRE(himem.requestHMA(0xFFFF) == true);
        REQUIRE(himem.isHMAAllocated() == true);

        // Should fail because HMA is already in use
        REQUIRE(himem.requestHMA(0xFFFF) == false);
        REQUIRE(himem.lastError() == HIMEM::ERR_HMA_IN_USE);

        REQUIRE(himem.releaseHMA() == true);
        REQUIRE(himem.isHMAAllocated() == false);
    }

    SECTION("EMB Allocation and Free") {
        uint16_t largestFree, initialTotalFree;
        initialTotalFree = himem.queryFreeKB(largestFree);
        REQUIRE(initialTotalFree > 0);

        // Allocate 10KB
        uint16_t handle1 = himem.allocateEMB(10);
        REQUIRE(handle1 != 0);
        REQUIRE(himem.getHandleSizeKB(handle1) == 10);

        uint16_t handle2 = himem.allocateEMB(20);
        REQUIRE(handle2 != 0);
        REQUIRE(himem.getHandleSizeKB(handle2) == 20);

        // Free handle1
        REQUIRE(himem.freeEMB(handle1) == true);

        // Cannot free twice
        REQUIRE(himem.freeEMB(handle1) == false);
        REQUIRE(himem.lastError() == HIMEM::ERR_INVALID_HANDLE);

        // Invalid handle
        REQUIRE(himem.freeEMB(999) == false);

        // Free handle2
        REQUIRE(himem.freeEMB(handle2) == true);
    }

    SECTION("EMB Lock and Unlock") {
        uint16_t handle = himem.allocateEMB(10);
        REQUIRE(handle != 0);

        uint32_t linearAddr = 0;
        REQUIRE(himem.lockEMB(handle, linearAddr) == true);
        REQUIRE(himem.getLockCount(handle) == 1);
        REQUIRE(linearAddr >= 0x100000); // Should be in extended memory space

        // Cannot free locked handle
        REQUIRE(himem.freeEMB(handle) == false);
        REQUIRE(himem.lastError() == HIMEM::ERR_HANDLE_LOCKED);

        // Multiple locks
        uint32_t linearAddr2 = 0;
        REQUIRE(himem.lockEMB(handle, linearAddr2) == true);
        REQUIRE(himem.getLockCount(handle) == 2);
        REQUIRE(linearAddr == linearAddr2); // Same linear addr

        REQUIRE(himem.unlockEMB(handle) == true);
        REQUIRE(himem.getLockCount(handle) == 1);

        REQUIRE(himem.unlockEMB(handle) == true);
        REQUIRE(himem.getLockCount(handle) == 0);

        // Cannot unlock if not locked
        REQUIRE(himem.unlockEMB(handle) == false);
        REQUIRE(himem.lastError() == HIMEM::ERR_NOT_LOCKED);

        // Now can free
        REQUIRE(himem.freeEMB(handle) == true);
    }

    SECTION("EMB Resize") {
        uint16_t handle = himem.allocateEMB(10);
        REQUIRE(handle != 0);

        // Resize larger
        REQUIRE(himem.resizeEMB(handle, 20) == true);
        REQUIRE(himem.getHandleSizeKB(handle) == 20);

        // Resize smaller
        REQUIRE(himem.resizeEMB(handle, 5) == true);
        REQUIRE(himem.getHandleSizeKB(handle) == 5);

        // Cannot resize locked handle
        uint32_t linearAddr = 0;
        REQUIRE(himem.lockEMB(handle, linearAddr) == true);
        REQUIRE(himem.resizeEMB(handle, 10) == false);
        REQUIRE(himem.lastError() == HIMEM::ERR_HANDLE_LOCKED);

        REQUIRE(himem.unlockEMB(handle) == true);
        REQUIRE(himem.freeEMB(handle) == true);
    }
}
