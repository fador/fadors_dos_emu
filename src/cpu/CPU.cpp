#include "CPU.hpp"
#include "../memory/MemoryBus.hpp"
#include "../utils/Logger.hpp"

namespace fador::cpu {

namespace {
constexpr uint32_t kWatchStackOffset = 0x75E;

bool overlapsWatchedOffset(uint32_t start, uint32_t width) {
  if (width == 0)
    return false;
  uint32_t end = start + width - 1;
  return start <= kWatchStackOffset && end >= kWatchStackOffset;
}

bool shouldLogStackWatch(uint16_t cs, uint32_t eip, uint32_t start,
                         uint32_t width) {
  if (!overlapsWatchedOffset(start, width))
    return false;

  // Focus on the known corruption path and the nearby IRQ/handler window.
  if (cs == 0x16F) {
    return eip >= 0x246800 && eip <= 0x246A20;
  }
  if (cs == 0x008F) {
    return eip <= 0x0120;
  }
  return false;
}
} // namespace

CPU::CPU() { reset(); }

void CPU::reset() {
  m_regs.fill(0);
  // Real mode startup values
  m_segRegs.fill(0);
  m_segRegs[CS] = 0xF000;
  m_is32BitCode = false;
  m_is32BitStack = false;

  m_eip = 0xFFF0;
  m_eflags = 0x00000002;

  m_cr.fill(0);
  m_dr.fill(0);

  // Initialise segment bases for Real Mode
  for (int i = 0; i < 6; i++) {
    m_segBase[i] = static_cast<uint32_t>(m_segRegs[i]) << 4;
  }

  LOG_INFO("CPU Reset: CS:EIP = 0x", std::hex, m_segRegs[CS], ":0x", m_eip);
}

uint32_t CPU::getReg32(uint8_t index) const { return m_regs[index & 7]; }

void CPU::setReg32(uint8_t index, uint32_t value) {
  uint8_t idx = index & 7;
  if (idx == ESP && m_regs[ESP] != value && m_segRegs[CS] == 0x16F &&
      m_eip >= 0x246800 && m_eip <= 0x246A20) {
    static int espTraceCount = 0;
    if (espTraceCount < 400) {
      ++espTraceCount;
      uint8_t b0 = 0, b1 = 0, b2 = 0;
      if (m_memory) {
        uint32_t pc = m_segBase[CS] + m_eip;
        b0 = m_memory->read8(pc);
        b1 = m_memory->read8(pc + 1);
        b2 = m_memory->read8(pc + 2);
      }
      LOG_ERROR("ESP TRACE: CS=0x", std::hex, m_segRegs[CS],
                " EIP=0x", m_eip,
                " old=0x", m_regs[ESP],
                " new=0x", value,
                " d=", (value >= m_regs[ESP]) ? "+" : "-",
                " code32=", m_is32BitCode ? 1 : 0,
                " stack32=", m_is32BitStack ? 1 : 0,
                " op=", static_cast<uint32_t>(b0), " ",
                static_cast<uint32_t>(b1), " ",
                static_cast<uint32_t>(b2));
    }
  }
  if (idx == ESP && value == 0) {
    static bool loggedSetEspZero = false;
    if (!loggedSetEspZero) {
      loggedSetEspZero = true;
      uint8_t b0 = 0, b1 = 0, b2 = 0;
      if (m_memory) {
        uint32_t pc = m_segBase[CS] + m_eip;
        b0 = m_memory->read8(pc);
        b1 = m_memory->read8(pc + 1);
        b2 = m_memory->read8(pc + 2);
      }
      LOG_ERROR("ESP became zero via setReg32: CS=0x", std::hex,
                m_segRegs[CS], " EIP=0x", m_eip,
                " SS=0x", m_segRegs[SS],
                " oldESP=0x", m_regs[ESP],
                " op=", static_cast<uint32_t>(b0), " ",
                static_cast<uint32_t>(b1), " ",
                static_cast<uint32_t>(b2));
    }
  }
  m_regs[idx] = value;
}

uint16_t CPU::getReg16(uint8_t index) const {
  return static_cast<uint16_t>(m_regs[index & 7] & 0xFFFF);
}

void CPU::setReg16(uint8_t index, uint16_t value) {
  uint8_t idx = index & 7;
  if (idx == SP && m_segRegs[SS] == 0x177 &&
      ((m_segRegs[CS] == 0x8F && m_eip < 0x200) ||
       (m_segRegs[CS] == 0x16F && m_eip >= 0x246800 && m_eip <= 0x246A20))) {
    static int spTraceCount = 0;
    if (spTraceCount < 300) {
      ++spTraceCount;
      uint16_t oldSp = static_cast<uint16_t>(m_regs[SP] & 0xFFFF);
      uint8_t b0 = 0, b1 = 0, b2 = 0;
      if (m_memory) {
        uint32_t pc = m_segBase[CS] + m_eip;
        b0 = m_memory->read8(pc);
        b1 = m_memory->read8(pc + 1);
        b2 = m_memory->read8(pc + 2);
      }
      LOG_ERROR("SP16 TRACE: CS=0x", std::hex, m_segRegs[CS],
                " EIP=0x", m_eip,
                " old=0x", static_cast<uint32_t>(oldSp),
                " new=0x", static_cast<uint32_t>(value),
                " d=", (value >= oldSp) ? "+" : "-",
                " code32=", m_is32BitCode ? 1 : 0,
                " stack32=", m_is32BitStack ? 1 : 0,
                " op=", static_cast<uint32_t>(b0), " ",
                static_cast<uint32_t>(b1), " ",
                static_cast<uint32_t>(b2));
    }
  }
  uint32_t &reg = m_regs[idx];
  reg = (reg & 0xFFFF0000) | value;
}

uint8_t CPU::getReg8(uint8_t index) const {
  uint8_t regIndex = index & 3; // 0=AL/AH...3=BL/BH
  bool highByte = (index & 4) != 0;

  if (highByte) {
    return static_cast<uint8_t>((m_regs[regIndex] >> 8) & 0xFF);
  } else {
    return static_cast<uint8_t>(m_regs[regIndex] & 0xFF);
  }
}

void CPU::setReg8(uint8_t index, uint8_t value) {
  uint8_t regIndex = index & 3;
  bool highByte = (index & 4) != 0;

  uint32_t &reg = m_regs[regIndex];
  if (highByte) {
    reg = (reg & 0xFFFF00FF) | (static_cast<uint32_t>(value) << 8);
  } else {
    reg = (reg & 0xFFFFFF00) | value;
  }
}

void CPU::push16(uint16_t value) {
  bool use32Stack = m_is32BitStack;
  uint32_t stackBase = m_segBase[SS];
  if ((m_cr[0] & 1) && !(m_eflags & 0x00020000) && m_memory) {
    uint16_t ss = m_segRegs[SS];
    bool descValid = false;
    if (ss != 0) {
      const auto &table = (ss & 0x04) ? m_ldtr : m_gdtr;
      uint32_t index = static_cast<uint32_t>(ss & ~7);
      if (table.base != 0 && index + 7 <= table.limit) {
        uint32_t entry = table.base + index;
        uint32_t high = m_memory->read32(entry + 4);
        bool present = (high & 0x00008000u) != 0;
        bool codeOrData = (high & 0x00001000u) != 0;
        if (present && codeOrData) {
          descValid = true;
          use32Stack = (high & 0x00400000u) != 0;
        } else {
          use32Stack = false;
        }
      } else {
        use32Stack = false;
      }
    } else {
      use32Stack = false;
    }
    if (!descValid && ss != 0) {
      // Fallback for transient RM-style selector states under PE.
      stackBase = static_cast<uint32_t>(ss) << 4;
      use32Stack = false;
    }
  }
  if (use32Stack) {
    uint32_t esp = getReg32(ESP) - 2;
    setReg32(ESP, esp);
    if (m_memory) {
      m_memory->write16(stackBase + esp, value);
      static int watchLogs = 0;
        if (watchLogs < 400 &&
          shouldLogStackWatch(m_segRegs[CS], m_eip, esp, 2)) {
        ++watchLogs;
        LOG_ERROR("STACK WATCH write16: CS=0x", std::hex, m_segRegs[CS],
                  " EIP=0x", m_eip,
                  " SS=0x", m_segRegs[SS],
                  " ESP=0x", esp,
                  " val=0x", value,
                  " op32=", m_is32BitCode ? 1 : 0,
                  " stk32=", m_is32BitStack ? 1 : 0);
      }
    }
  } else {
    uint16_t sp = getReg16(SP) - 2;
    setReg16(SP, sp);
    if (m_memory) {
      m_memory->write16(stackBase + sp, value);
      static int watchLogs = 0;
        if (watchLogs < 400 &&
          shouldLogStackWatch(m_segRegs[CS], m_eip, sp, 2)) {
        ++watchLogs;
        LOG_ERROR("STACK WATCH write16: CS=0x", std::hex, m_segRegs[CS],
                  " EIP=0x", m_eip,
                  " SS=0x", m_segRegs[SS],
                  " SP=0x", static_cast<uint32_t>(sp),
                  " val=0x", value,
                  " op32=", m_is32BitCode ? 1 : 0,
                  " stk32=", m_is32BitStack ? 1 : 0);
      }
    }
  }
}

void CPU::push32(uint32_t value) {
  bool use32Stack = m_is32BitStack;
  uint32_t stackBase = m_segBase[SS];
  if ((m_cr[0] & 1) && !(m_eflags & 0x00020000) && m_memory) {
    uint16_t ss = m_segRegs[SS];
    bool descValid = false;
    if (ss != 0) {
      const auto &table = (ss & 0x04) ? m_ldtr : m_gdtr;
      uint32_t index = static_cast<uint32_t>(ss & ~7);
      if (table.base != 0 && index + 7 <= table.limit) {
        uint32_t entry = table.base + index;
        uint32_t high = m_memory->read32(entry + 4);
        bool present = (high & 0x00008000u) != 0;
        bool codeOrData = (high & 0x00001000u) != 0;
        if (present && codeOrData) {
          descValid = true;
          use32Stack = (high & 0x00400000u) != 0;
        } else {
          use32Stack = false;
        }
      } else {
        use32Stack = false;
      }
    } else {
      use32Stack = false;
    }
    if (!descValid && ss != 0) {
      // Fallback for transient RM-style selector states under PE.
      stackBase = static_cast<uint32_t>(ss) << 4;
      use32Stack = false;
    }
  }
  if (use32Stack) {
    uint32_t esp = getReg32(ESP) - 4;
    setReg32(ESP, esp);
    if (m_memory) {
      m_memory->write32(stackBase + esp, value);
      static int watchLogs = 0;
      bool suspiciousLowValue = value < 0x1000;
      if (watchLogs < 400 &&
          (overlapsWatchedOffset(esp, 4) ||
           shouldLogStackWatch(m_segRegs[CS], m_eip, esp, 4) ||
           (overlapsWatchedOffset(esp, 4) && suspiciousLowValue))) {
        ++watchLogs;
        LOG_ERROR("STACK WATCH write32: CS=0x", std::hex, m_segRegs[CS],
                  " EIP=0x", m_eip,
                  " SS=0x", m_segRegs[SS],
                  " ESP=0x", esp,
                  " val=0x", value,
                  " op32=", m_is32BitCode ? 1 : 0,
                  " stk32=", m_is32BitStack ? 1 : 0);
      }
    }
  } else {
    uint16_t sp = getReg16(SP) - 4;
    setReg16(SP, sp);
    if (m_memory) {
      m_memory->write32(stackBase + sp, value);
      static int watchLogs = 0;
      bool suspiciousLowValue = value < 0x1000;
      if (watchLogs < 400 &&
          (overlapsWatchedOffset(sp, 4) ||
           shouldLogStackWatch(m_segRegs[CS], m_eip, sp, 4) ||
           (overlapsWatchedOffset(sp, 4) && suspiciousLowValue))) {
        ++watchLogs;
        LOG_ERROR("STACK WATCH write32: CS=0x", std::hex, m_segRegs[CS],
                  " EIP=0x", m_eip,
                  " SS=0x", m_segRegs[SS],
                  " SP=0x", static_cast<uint32_t>(sp),
                  " val=0x", value,
                  " op32=", m_is32BitCode ? 1 : 0,
                  " stk32=", m_is32BitStack ? 1 : 0);
      }
    }
  }
}

uint16_t CPU::pop16() {
  uint16_t value = 0;
  bool use32Stack = m_is32BitStack;
  uint32_t stackBase = m_segBase[SS];
  if ((m_cr[0] & 1) && !(m_eflags & 0x00020000) && m_memory) {
    uint16_t ss = m_segRegs[SS];
    bool descValid = false;
    if (ss != 0) {
      const auto &table = (ss & 0x04) ? m_ldtr : m_gdtr;
      uint32_t index = static_cast<uint32_t>(ss & ~7);
      if (table.base != 0 && index + 7 <= table.limit) {
        uint32_t entry = table.base + index;
        uint32_t high = m_memory->read32(entry + 4);
        bool present = (high & 0x00008000u) != 0;
        bool codeOrData = (high & 0x00001000u) != 0;
        if (present && codeOrData) {
          descValid = true;
          use32Stack = (high & 0x00400000u) != 0;
        } else {
          use32Stack = false;
        }
      } else {
        use32Stack = false;
      }
    } else {
      use32Stack = false;
    }
    if (!descValid && ss != 0) {
      // Fallback for transient RM-style selector states under PE.
      stackBase = static_cast<uint32_t>(ss) << 4;
      use32Stack = false;
    }
  }
  if (use32Stack) {
    uint32_t esp = getReg32(ESP);
    if (m_memory) {
      value = m_memory->read16(stackBase + esp);
      static int watchLogs = 0;
        if (watchLogs < 400 &&
          shouldLogStackWatch(m_segRegs[CS], m_eip, esp, 2)) {
        ++watchLogs;
        LOG_ERROR("STACK WATCH read16: CS=0x", std::hex, m_segRegs[CS],
                  " EIP=0x", m_eip,
                  " SS=0x", m_segRegs[SS],
                  " ESP=0x", esp,
                  " val=0x", value,
                  " op32=", m_is32BitCode ? 1 : 0,
                  " stk32=", m_is32BitStack ? 1 : 0);
      }
    }
    setReg32(ESP, esp + 2);
  } else {
    uint16_t sp = getReg16(SP);
    if (m_memory) {
      value = m_memory->read16(stackBase + sp);
      static int watchLogs = 0;
        if (watchLogs < 400 &&
          shouldLogStackWatch(m_segRegs[CS], m_eip, sp, 2)) {
        ++watchLogs;
        LOG_ERROR("STACK WATCH read16: CS=0x", std::hex, m_segRegs[CS],
                  " EIP=0x", m_eip,
                  " SS=0x", m_segRegs[SS],
                  " SP=0x", static_cast<uint32_t>(sp),
                  " val=0x", value,
                  " op32=", m_is32BitCode ? 1 : 0,
                  " stk32=", m_is32BitStack ? 1 : 0);
      }
    }
    setReg16(SP, sp + 2);
  }
  return value;
}

void CPU::setEIP(uint32_t val) {
  if (m_segRegs[CS] == 0x16F && val == 0x24696F && val != (m_eip + 1)) {
    static int loggedTarget24696f = 0;
    if (loggedTarget24696f < 20) {
      ++loggedTarget24696f;
      uint8_t b0 = 0, b1 = 0, b2 = 0, b3 = 0;
      if (m_memory) {
        uint32_t pc = m_segBase[CS] + m_eip;
        b0 = m_memory->read8(pc);
        b1 = m_memory->read8(pc + 1);
        b2 = m_memory->read8(pc + 2);
        b3 = m_memory->read8(pc + 3);
      }
      LOG_ERROR("EIP CTL->24696f: oldEIP=0x", std::hex, m_eip,
                " newEIP=0x", val,
                " ESP=0x", m_regs[ESP],
                " op=", static_cast<uint32_t>(b0), " ",
                static_cast<uint32_t>(b1), " ",
                static_cast<uint32_t>(b2), " ",
                static_cast<uint32_t>(b3));
    }
  }

  static bool loggedLowEipJump = false;
  if (!loggedLowEipJump && m_segRegs[CS] == 0x16F && m_eip > 0x1000 &&
      val < 0x1000) {
    loggedLowEipJump = true;
    void *caller = __builtin_return_address(0);
    uint8_t b0 = 0, b1 = 0, b2 = 0, b3 = 0;
    uint8_t pm2 = 0, pm1 = 0, pp1 = 0, pp2 = 0;
    uint32_t stk0 = 0, stk1 = 0, stk2 = 0;
    uint32_t ssBaseDbg = 0;
    uint32_t csDescHigh = 0;
    bool csDescD = false;
    if (m_memory) {
      uint32_t pc = m_segBase[CS] + m_eip;
      b0 = m_memory->read8(pc);
      b1 = m_memory->read8(pc + 1);
      b2 = m_memory->read8(pc + 2);
      b3 = m_memory->read8(pc + 3);
      pm2 = m_memory->read8(pc - 2);
      pm1 = m_memory->read8(pc - 1);
      pp1 = m_memory->read8(pc + 4);
      pp2 = m_memory->read8(pc + 5);
      uint32_t ssBase = m_segBase[SS];
      ssBaseDbg = ssBase;
      uint32_t esp = m_regs[ESP];
      stk0 = m_memory->read32(ssBase + esp);
      stk1 = m_memory->read32(ssBase + esp + 4);
      stk2 = m_memory->read32(ssBase + esp + 8);

      uint16_t csSel = m_segRegs[CS];
      if (csSel != 0) {
        uint32_t tableBase = (csSel & 0x04) ? m_ldtr.base : m_gdtr.base;
        uint32_t entry = tableBase + (csSel & ~7);
        csDescHigh = m_memory->read32(entry + 4);
        csDescD = (csDescHigh & 0x00400000u) != 0;
      }
    }
    LOG_ERROR("EIP truncation: CS=0x", std::hex, m_segRegs[CS],
              " oldEIP=0x", m_eip,
              " newEIP=0x", val,
              " SS=0x", m_segRegs[SS],
              " ESP=0x", m_regs[ESP],
              " SP=0x", static_cast<uint16_t>(m_regs[ESP] & 0xFFFF),
              " ssBase=0x", ssBaseDbg,
              " caller=0x", reinterpret_cast<uintptr_t>(caller),
              " prev=", static_cast<uint32_t>(pm2), " ",
              static_cast<uint32_t>(pm1),
              " op=", static_cast<uint32_t>(b0), " ",
              static_cast<uint32_t>(b1), " ",
              static_cast<uint32_t>(b2), " ",
              static_cast<uint32_t>(b3),
              " next=", static_cast<uint32_t>(pp1), " ",
              static_cast<uint32_t>(pp2),
              " stk=0x", stk0, " 0x", stk1, " 0x", stk2,
              " csHigh=0x", csDescHigh,
              " csD=", csDescD ? 1 : 0);
  }
  m_eip = val;
}

uint32_t CPU::pop32() {
  uint32_t value = 0;
  bool use32Stack = m_is32BitStack;
  uint32_t stackBase = m_segBase[SS];
  if ((m_cr[0] & 1) && !(m_eflags & 0x00020000) && m_memory) {
    uint16_t ss = m_segRegs[SS];
    bool descValid = false;
    if (ss != 0) {
      const auto &table = (ss & 0x04) ? m_ldtr : m_gdtr;
      uint32_t index = static_cast<uint32_t>(ss & ~7);
      if (table.base != 0 && index + 7 <= table.limit) {
        uint32_t entry = table.base + index;
        uint32_t high = m_memory->read32(entry + 4);
        bool present = (high & 0x00008000u) != 0;
        bool codeOrData = (high & 0x00001000u) != 0;
        if (present && codeOrData) {
          descValid = true;
          use32Stack = (high & 0x00400000u) != 0;
        } else {
          use32Stack = false;
        }
      } else {
        use32Stack = false;
      }
    } else {
      use32Stack = false;
    }
    if (!descValid && ss != 0) {
      // Fallback for transient RM-style selector states under PE.
      stackBase = static_cast<uint32_t>(ss) << 4;
      use32Stack = false;
    }
  }
  if (use32Stack) {
    uint32_t esp = getReg32(ESP);
    if (m_memory) {
      if (shouldLogStackWatch(m_segRegs[CS], m_eip, esp, 4)) {
        uint32_t absAddr = stackBase + esp;
        uint8_t b0 = m_memory->read8(absAddr + 0);
        uint8_t b1 = m_memory->read8(absAddr + 1);
        uint8_t b2 = m_memory->read8(absAddr + 2);
        uint8_t b3 = m_memory->read8(absAddr + 3);
        LOG_ERROR("STACK WATCH pop32 bytes: CS=0x", std::hex, m_segRegs[CS],
                  " EIP=0x", m_eip,
                  " SS=0x", m_segRegs[SS],
                  " ESP=0x", esp,
                  " abs=0x", absAddr,
                  " b=", static_cast<uint32_t>(b0), " ",
                  static_cast<uint32_t>(b1), " ",
                  static_cast<uint32_t>(b2), " ",
                  static_cast<uint32_t>(b3));
      }
      value = m_memory->read32(stackBase + esp);
      static int watchLogs = 0;
        if (watchLogs < 400 &&
          shouldLogStackWatch(m_segRegs[CS], m_eip, esp, 4)) {
        ++watchLogs;
          uint32_t absAddr = stackBase + esp;
        LOG_ERROR("STACK WATCH read32: CS=0x", std::hex, m_segRegs[CS],
                  " EIP=0x", m_eip,
                  " SS=0x", m_segRegs[SS],
                  " ESP=0x", esp,
                    " abs=0x", absAddr,
                  " val=0x", value,
                  " op32=", m_is32BitCode ? 1 : 0,
                  " stk32=", m_is32BitStack ? 1 : 0);
      }
    }
    setReg32(ESP, esp + 4);
  } else {
    uint16_t sp = getReg16(SP);
    if (m_memory) {
      value = m_memory->read32(stackBase + sp);
      static int watchLogs = 0;
        if (watchLogs < 400 &&
          shouldLogStackWatch(m_segRegs[CS], m_eip, sp, 4)) {
        ++watchLogs;
        LOG_ERROR("STACK WATCH read32: CS=0x", std::hex, m_segRegs[CS],
                  " EIP=0x", m_eip,
                  " SS=0x", m_segRegs[SS],
                  " SP=0x", static_cast<uint32_t>(sp),
                  " val=0x", value,
                  " op32=", m_is32BitCode ? 1 : 0,
                  " stk32=", m_is32BitStack ? 1 : 0);
      }
    }
    setReg16(SP, sp + 4);
  }
  return value;
}

} // namespace fador::cpu
