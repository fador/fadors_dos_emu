// BIOS serial (INT 14h) and printer (INT 17h) services, and the port tables
// in the BIOS data area.  Like the ROM BIOS, the services program the
// UART / parallel port registers through the I/O bus at the addresses
// stored in the BDA, so programs that swap or patch those entries see the
// usual behaviour.

#include "BIOS.hpp"
#include "IOBus.hpp"
#include "../utils/Logger.hpp"

namespace fador::hw {

namespace {
// INT 14h AH=00h / 04h baud rate codes -> 115200 / baud divisors
// (110, 150, 300, 600, 1200, 2400, 4800, 9600, 19200)
constexpr uint16_t kBaudDivisors[9] = {1047, 768, 384, 192, 96, 48, 24, 12, 6};
// Equipment word bits the BIOS reports besides the ports: diskette
// present, x87, 4 RAM banks, initial video mode 80x25 colour.
constexpr uint16_t kEquipmentBase = 0x002F;
} // namespace

void BIOS::setPortAddresses(const std::array<uint16_t, 4> &com,
                            const std::array<uint16_t, 3> &lpt) {
  m_comPorts = com;
  m_lptPorts = lpt;
  writePortTables();
}

void BIOS::writePortTables() {
  // The POST lists the ports it found without gaps: the first one present
  // becomes COM1 / LPT1.
  uint16_t serial = 0, parallel = 0;
  for (uint32_t i = 0; i < 4; ++i)
    m_memory.write16(0x400 + 2 * i, 0);
  for (uint32_t i = 0; i < 3; ++i)
    m_memory.write16(0x408 + 2 * i, 0);
  for (uint16_t port : m_comPorts)
    if (port)
      m_memory.write16(0x400 + 2u * serial++, port);
  for (uint16_t port : m_lptPorts)
    if (port)
      m_memory.write16(0x408 + 2u * parallel++, port);
  uint16_t equipment = m_memory.read16(0x410);
  if (equipment == 0)
    equipment = kEquipmentBase;
  equipment = static_cast<uint16_t>((equipment & ~0xCE00u) | (serial << 9) |
                                    (parallel << 14));
  m_memory.write16(0x410, equipment);
}

int BIOS::waitForPort(uint8_t vector, uint8_t timeoutUnits,
                      const std::function<bool()> &ready) {
  if (ready()) {
    m_portWait.active = false;
    return 1;
  }
  if (!m_portTimeSource || !m_idleCallback) {
    m_portWait.active = false;
    return 0; // nothing can change while we wait
  }
  const uint64_t now = m_portTimeSource();
  const uint16_t cs = m_cpu.getSegReg(cpu::CS);
  const uint32_t eip = m_cpu.getInstructionStartEIP();
  if (!m_portWait.active || m_portWait.vector != vector ||
      m_portWait.cs != cs || m_portWait.eip != eip) {
    // A count of 0 wraps around in the ROM's DEC/JNZ loop: 256 units.
    const uint64_t units = timeoutUnits ? timeoutUnits : 256;
    m_portWait = {true, vector, cs, eip, now + units * PORT_TIMEOUT_UNIT_NS};
  }
  if (now >= m_portWait.deadlineNs) {
    m_portWait.active = false;
    return 0;
  }
  // Re-execute the INT after letting time pass (and IRQs run), like the
  // blocking INT 16h read.
  m_cpu.setEIP(eip);
  m_idleCallback();
  return -1;
}

// ── INT 14h ───────────────────────────────────────────────────────────

void BIOS::handleSerialService() {
  const uint8_t ah = m_cpu.getReg8(cpu::AH);
  const uint16_t index = m_cpu.getReg16(cpu::DX);
  if (index > 3 || !m_iobus)
    return;
  const uint16_t base = m_memory.read16(0x400 + 2u * index);
  if (base == 0)
    return; // no such port: registers are returned unchanged
  auto in = [&](uint16_t reg) { return m_iobus->read8(base + reg); };
  auto out = [&](uint16_t reg, uint8_t v) { m_iobus->write8(base + reg, v); };
  auto returnStatus = [&]() {
    m_cpu.setReg8(cpu::AH, in(5)); // line status
    m_cpu.setReg8(cpu::AL, in(6)); // modem status
  };
  auto setLine = [&](uint16_t divisor, uint8_t lcr) {
    out(3, 0x80); // DLAB
    out(0, static_cast<uint8_t>(divisor & 0xFF));
    out(1, static_cast<uint8_t>(divisor >> 8));
    out(3, lcr);
    out(1, 0x00); // interrupts off
  };
  const uint8_t timeout = m_memory.read8(0x47C + index);

  switch (ah) {
  case 0x00: { // Initialize: AL = baud(7-5) parity(4-3) stop(2) length(1-0)
    const uint8_t al = m_cpu.getReg8(cpu::AL);
    setLine(kBaudDivisors[al >> 5], al & 0x1F);
    returnStatus();
    break;
  }
  case 0x01: { // Send AL: DTR+RTS, wait for DSR, CTS and THRE
    const uint8_t ch = m_cpu.getReg8(cpu::AL);
    out(4, 0x03);
    uint8_t lsr = 0;
    const int r = waitForPort(0x14, timeout, [&] {
      if ((in(6) & 0x30) != 0x30)
        return false;
      lsr = in(5);
      return (lsr & 0x20) != 0;
    });
    if (r < 0)
      return; // retrying
    if (r == 0) {
      m_cpu.setReg8(cpu::AH, static_cast<uint8_t>(in(5) | 0x80));
      return;
    }
    out(0, ch);
    m_cpu.setReg8(cpu::AH, static_cast<uint8_t>(lsr & 0x7F));
    break;
  }
  case 0x02: { // Receive: DTR, wait for DSR and data ready
    out(4, 0x01);
    const int r = waitForPort(0x14, timeout, [&] {
      return (in(6) & 0x20) && (in(5) & 0x01);
    });
    if (r < 0)
      return;
    if (r == 0) {
      m_cpu.setReg8(cpu::AH, static_cast<uint8_t>(in(5) | 0x80));
      return;
    }
    const uint8_t lsr = in(5);
    m_cpu.setReg8(cpu::AL, in(0));
    m_cpu.setReg8(cpu::AH, lsr & 0x1E); // error bits only
    break;
  }
  case 0x03: // Status
    returnStatus();
    break;
  case 0x04: { // Extended initialize (PS/2)
    // AL break, BH parity, BL stop bits, CH word length, CL baud code
    const uint8_t brk = m_cpu.getReg8(cpu::AL);
    const uint8_t parity = m_cpu.getReg8(cpu::BH);
    const uint8_t stop = m_cpu.getReg8(cpu::BL);
    const uint8_t length = m_cpu.getReg8(cpu::CH);
    const uint8_t baud = m_cpu.getReg8(cpu::CL);
    static constexpr uint8_t kParity[5] = {0x00, 0x08, 0x18, 0x28, 0x38};
    const uint8_t lcr = static_cast<uint8_t>(
        (length & 3) | (stop ? 0x04 : 0x00) | kParity[parity < 5 ? parity : 0] |
        (brk ? 0x40 : 0x00));
    setLine(kBaudDivisors[baud < 9 ? baud : 7], lcr);
    returnStatus();
    break;
  }
  case 0x05: // Extended communication port control (PS/2)
    if (m_cpu.getReg8(cpu::AL) == 0x00)
      m_cpu.setReg8(cpu::BL, in(4)); // read MCR
    else if (m_cpu.getReg8(cpu::AL) == 0x01)
      out(4, m_cpu.getReg8(cpu::BL) & 0x1F); // write MCR
    returnStatus();
    break;
  default:
    LOG_DEBUG("BIOS INT 14h: unsupported function AH=0x", std::hex, (int)ah);
    break;
  }
}

// ── INT 17h ───────────────────────────────────────────────────────────

void BIOS::handlePrinterService() {
  const uint8_t ah = m_cpu.getReg8(cpu::AH);
  const uint16_t index = m_cpu.getReg16(cpu::DX);
  if (index > 2 || !m_iobus)
    return;
  const uint16_t base = m_memory.read16(0x408 + 2u * index);
  if (base == 0)
    return; // no such port: registers are returned unchanged
  auto in = [&](uint16_t reg) { return m_iobus->read8(base + reg); };
  auto out = [&](uint16_t reg, uint8_t v) { m_iobus->write8(base + reg, v); };
  // AH = status with ACK# and ERROR# turned active high:
  // 80 not busy, 40 acknowledge, 20 out of paper, 10 selected,
  // 08 I/O error, 01 time-out.
  auto returnStatus = [&](uint8_t timeoutBit) {
    const uint8_t st = static_cast<uint8_t>((in(1) & 0xF8) | timeoutBit);
    m_cpu.setReg8(cpu::AH, static_cast<uint8_t>(st ^ 0x48));
  };

  switch (ah) {
  case 0x00: { // Print AL: wait while busy, then strobe
    out(0, m_cpu.getReg8(cpu::AL));
    const int r = waitForPort(0x17, m_memory.read8(0x478 + index),
                              [&] { return (in(1) & 0x80) != 0; });
    if (r < 0)
      return; // retrying
    if (r == 0) {
      returnStatus(0x01);
      return;
    }
    out(2, 0x0D); // STROBE on (INIT# high, SELECT IN)
    out(2, 0x0C); // STROBE off
    returnStatus(0x00);
    break;
  }
  case 0x01: // Initialize: pulse INIT#
    out(2, 0x08);
    out(2, 0x0C);
    returnStatus(0x00);
    break;
  case 0x02: // Status
    returnStatus(0x00);
    break;
  default:
    LOG_DEBUG("BIOS INT 17h: unsupported function AH=0x", std::hex, (int)ah);
    break;
  }
}

} // namespace fador::hw
