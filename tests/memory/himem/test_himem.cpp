#include "../../test_framework.hpp"
#include "memory/himem/HIMEM.hpp"
#include "memory/MemoryBus.hpp"

using namespace fador::memory;

TEST_CASE("HIMEM: A20 Gate Toggle", "[HIMEM][Memory]") {
    HIMEM himem;
    MemoryBus memBus;
    himem.setMemoryBus(&memBus);

    SECTION("Initial state") {
        REQUIRE(!himem.isA20Enabled());
        REQUIRE(!memBus.isA20Enabled());
    }

    SECTION("Enable A20") {
        REQUIRE(himem.enableA20());
        REQUIRE(himem.isA20Enabled());
        REQUIRE(memBus.isA20Enabled());
    }

    SECTION("Disable A20") {
        himem.enableA20();
        REQUIRE(himem.disableA20());
        REQUIRE(!himem.isA20Enabled());
        REQUIRE(!memBus.isA20Enabled());
    }

    SECTION("A20 Gate without MemoryBus") {
        HIMEM himemNoBus;
        // Should not crash when m_memBus is nullptr
        REQUIRE(himemNoBus.enableA20());
        REQUIRE(himemNoBus.isA20Enabled());

        REQUIRE(himemNoBus.disableA20());
        REQUIRE(!himemNoBus.isA20Enabled());
    }
}
