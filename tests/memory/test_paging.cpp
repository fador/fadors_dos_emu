#include "test_framework.hpp"
#include "cpu/CPU.hpp"
#include "memory/MemoryBus.hpp"

using namespace fador;

namespace {

constexpr uint32_t kPageDir = 0x10000;
constexpr uint32_t kPageTable = 0x11000;

// Identity-map the first 4 MiB and map linear 0x00400000 (second page
// directory entry) to physical 0x00200000.
void buildPageTables(memory::MemoryBus &mem) {
  for (uint32_t i = 0; i < 1024; ++i)
    mem.write32(kPageTable + i * 4, (i << 12) | 0x07);
  mem.write32(kPageDir + 0, kPageTable | 0x07);
  mem.write32(kPageDir + 4, 0x12000 | 0x07);
  mem.write32(0x12000, 0x00200000 | 0x07);     // 0x400000 -> 0x200000
  mem.write32(0x12004, 0x00300000 | 0x07);     // 0x401000 -> 0x300000
  // 0x402000 left not present
}

} // namespace

TEST_CASE("Paging: linear accesses are translated only with CR0.PG", "[Memory][Paging]") {
  memory::MemoryBus mem;
  cpu::CPU cpu;
  cpu.setMemoryBus(&mem);
  mem.setA20(true);
  buildPageTables(mem);
  mem.write32(0x200010, 0x11223344);
  mem.write32(0x400010, 0xAABBCCDD);

  SECTION("Paging off: linear == physical") {
    REQUIRE(mem.lread32(0x400010) == 0xAABBCCDD);
  }

  SECTION("Paging on through CR0/CR3") {
    cpu.setCR(3, kPageDir);
    cpu.setCR(0, 0x80000001);
    REQUIRE(mem.pagingEnabled());
    REQUIRE(mem.lread32(0x400010) == 0x11223344);
    // Physical accessors (HLE, renderers) are unaffected.
    REQUIRE(mem.read32(0x400010) == 0xAABBCCDD);
    // Identity-mapped low memory.
    mem.write8(0x5000, 0x5A);
    REQUIRE(mem.lread8(0x5000) == 0x5A);

    mem.lwrite16(0x400020, 0xBEEF);
    REQUIRE(mem.read16(0x200020) == 0xBEEF);

    // Accessed and dirty bits are set in the PTE.
    REQUIRE((mem.read32(0x12000) & 0x60) == 0x60);
    REQUIRE((mem.read32(kPageDir + 4) & 0x20) == 0x20);

    // Clearing PG returns to untranslated access.
    cpu.setCR(0, 0x00000001);
    REQUIRE(!mem.pagingEnabled());
    REQUIRE(mem.lread32(0x400010) == 0xAABBCCDD);
  }

  SECTION("Accesses that straddle a page boundary are split") {
    cpu.setCR(3, kPageDir);
    cpu.setCR(0, 0x80000001);
    mem.write8(0x200FFE, 0x01);
    mem.write8(0x200FFF, 0x02);
    mem.write8(0x300000, 0x03);
    mem.write8(0x300001, 0x04);
    REQUIRE(mem.lread32(0x400FFE) == 0x04030201);
    mem.lwrite32(0x400FFD, 0xA1B2C3D4);
    REQUIRE(mem.read8(0x200FFD) == 0xD4);
    REQUIRE(mem.read8(0x200FFE) == 0xC3);
    REQUIRE(mem.read8(0x200FFF) == 0xB2);
    REQUIRE(mem.read8(0x300000) == 0xA1);
  }

  SECTION("TLB is flushed by CR3 reload and INVLPG") {
    cpu.setCR(3, kPageDir);
    cpu.setCR(0, 0x80000001);
    REQUIRE(mem.lread32(0x400010) == 0x11223344);
    mem.write32(0x12000, 0x00300000 | 0x07); // remap behind the TLB's back
    mem.write32(0x300010, 0x55667788);
    REQUIRE(mem.lread32(0x400010) == 0x11223344); // stale entry
    mem.invalidatePage(0x400010);
    REQUIRE(mem.lread32(0x400010) == 0x55667788);
    mem.write32(0x12000, 0x00200000 | 0x07);
    cpu.setCR(3, kPageDir);
    REQUIRE(mem.lread32(0x400010) == 0x11223344);
  }

  SECTION("Not-present page reports a fault and has no effect") {
    cpu.setCR(3, kPageDir);
    cpu.setCR(0, 0x80000001);
    uint32_t linear = 0, err = 0;
    REQUIRE(!mem.takePageFault(linear, err));
    mem.lwrite32(0x402008, 0x12345678);
    REQUIRE(mem.takePageFault(linear, err));
    REQUIRE(linear == 0x402008);
    REQUIRE((err & 0x3) == 0x2); // not present, write
  }

  SECTION("CPU stack operations go through paging") {
    cpu.setCR(3, kPageDir);
    cpu.setCR(0, 0x80000001);
    cpu.setSegBase(cpu::SS, 0x400000);
    cpu.setReg32(cpu::ESP, 0x100);
    cpu.push32(0xCAFEBABE);
    REQUIRE(mem.read32(0x2000FC) == 0xCAFEBABE);
    REQUIRE(cpu.pop32() == 0xCAFEBABE);
  }
}
