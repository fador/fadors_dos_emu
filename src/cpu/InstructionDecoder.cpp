#include "InstructionDecoder.hpp"
#include "../hw/BIOS.hpp"
#include "../hw/DOS.hpp"
#include "../hw/IOBus.hpp"
#include "../utils/Logger.hpp"
#include <cstdio>

namespace fador::cpu {

InstructionDecoder::InstructionDecoder(CPU &cpu, memory::MemoryBus &memory,
                                       hw::IOBus &iobus, hw::BIOS &bios,
                                       hw::DOS &dos)
    : m_cpu(cpu), m_memory(memory), m_iobus(iobus), m_bios(bios), m_dos(dos),
      m_stepCount(0), m_hasPrefix66(false), m_hasPrefix67(false),
      m_hasRepnz(false), m_hasRepz(false), m_segmentOverride(0xFF),
      m_currentEA(0), m_currentOffset(0), m_eaResolved(false) {
  m_cpu.setMemoryBus(&m_memory);

  // Initialise segment bases for Real Mode
  for (int i = 0; i < 6; i++) {
    m_segBase[i] =
        static_cast<uint32_t>(m_cpu.getSegReg(static_cast<SegRegIndex>(i)))
        << 4;
  }
}

uint8_t InstructionDecoder::fetch8() {
  uint32_t eip = m_cpu.getEIP();
  uint8_t val = m_memory.read8(m_segBase[CS] + eip);
  m_cpu.setEIP(eip + 1);
  return val;
}

uint16_t InstructionDecoder::fetch16() {
  uint32_t eip = m_cpu.getEIP();
  uint16_t val = m_memory.read16(m_segBase[CS] + eip);
  m_cpu.setEIP(eip + 2);
  return val;
}

uint32_t InstructionDecoder::fetch32() {
  uint32_t eip = m_cpu.getEIP();
  uint32_t val = m_memory.read32(m_segBase[CS] + eip);
  m_cpu.setEIP(eip + 4);
  return val;
}

ModRM InstructionDecoder::decodeModRM(uint8_t byte) {
  return {static_cast<uint8_t>((byte >> 6) & 0x03),
          static_cast<uint8_t>((byte >> 3) & 0x07),
          static_cast<uint8_t>(byte & 0x07)};
}

SIB InstructionDecoder::decodeSIB(uint8_t byte) {
  return {static_cast<uint8_t>((byte >> 6) & 0x03),
          static_cast<uint8_t>((byte >> 3) & 0x07),
          static_cast<uint8_t>(byte & 0x07)};
}

uint32_t InstructionDecoder::getEffectiveAddress16(const ModRM &modrm) {
  if (m_eaResolved)
    return m_currentEA;

  uint16_t addr = 0;
  uint8_t defaultSeg = DS;

  if (modrm.mod == 0 && modrm.rm == 6) {
    addr = fetch16();
  } else {
    switch (modrm.rm) {
    case 0:
      addr = m_cpu.getReg16(BX) + m_cpu.getReg16(SI);
      break;
    case 1:
      addr = m_cpu.getReg16(BX) + m_cpu.getReg16(DI);
      break;
    case 2:
      addr = m_cpu.getReg16(BP) + m_cpu.getReg16(SI);
      defaultSeg = SS;
      break;
    case 3:
      addr = m_cpu.getReg16(BP) + m_cpu.getReg16(DI);
      defaultSeg = SS;
      break;
    case 4:
      addr = m_cpu.getReg16(SI);
      break;
    case 5:
      addr = m_cpu.getReg16(DI);
      break;
    case 6:
      addr = m_cpu.getReg16(BP);
      defaultSeg = SS;
      break;
    case 7:
      addr = m_cpu.getReg16(BX);
      break;
    }

    if (modrm.mod == 1) {
      addr += static_cast<int8_t>(fetch8());
    } else if (modrm.mod == 2) {
      addr += fetch16(); // 16-bit mode disp16
    }
  }

  uint8_t seg = (m_segmentOverride != 0xFF) ? m_segmentOverride : defaultSeg;
  m_currentOffset = addr;
  m_currentEA = m_segBase[seg] + m_currentOffset;
  m_eaResolved = true;
  return m_currentEA;
}

uint32_t InstructionDecoder::getEffectiveAddress32(const ModRM &modrm) {
  if (m_eaResolved)
    return m_currentEA;

  uint32_t addr = 0;
  uint8_t defaultSeg = DS;

  if (modrm.rm == 4) { // SIB byte follows
    SIB sib = decodeSIB(fetch8());
    uint32_t base = 0;
    if (sib.base == 5) {
      if (modrm.mod == 0)
        base = fetch32();
      else {
        base = m_cpu.getReg32(EBP);
        defaultSeg = SS;
      }
    } else {
      base = m_cpu.getReg32(sib.base);
      if (sib.base == ESP || sib.base == EBP)
        defaultSeg = SS;
    }

    uint32_t index = (sib.index == 4) ? 0 : m_cpu.getReg32(sib.index);
    addr = base + (index << sib.scale);
  } else {
    if (modrm.mod == 0 && modrm.rm == 5) {
      addr = fetch32();
    } else {
      addr = m_cpu.getReg32(modrm.rm);
      if (modrm.rm == ESP || modrm.rm == EBP)
        defaultSeg = SS;
    }
  }

  if (modrm.mod == 1) {
    addr += static_cast<int32_t>(static_cast<int8_t>(fetch8()));
  } else if (modrm.mod == 2) {
    addr += fetch32();
  }

  uint8_t seg = (m_segmentOverride != 0xFF) ? m_segmentOverride : defaultSeg;
  m_currentOffset = addr;
  m_currentEA = m_segBase[seg] + m_currentOffset;
  m_eaResolved = true;
  return m_currentEA;
}

uint32_t InstructionDecoder::readModRM32(const ModRM &modrm) {
  if (modrm.mod == 3)
    return m_cpu.getReg32(modrm.rm);
  return m_memory.read32(m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                       : getEffectiveAddress16(modrm));
}

uint16_t InstructionDecoder::readModRM16(const ModRM &modrm) {
  if (modrm.mod == 3)
    return m_cpu.getReg16(modrm.rm);
  return m_memory.read16(m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                       : getEffectiveAddress16(modrm));
}

uint8_t InstructionDecoder::readModRM8(const ModRM &modrm) {
  if (modrm.mod == 3)
    return m_cpu.getReg8(modrm.rm);
  return m_memory.read8(m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                      : getEffectiveAddress16(modrm));
}

void InstructionDecoder::writeModRM32(const ModRM &modrm, uint32_t value) {
  if (modrm.mod == 3)
    m_cpu.setReg32(modrm.rm, value);
  else
    m_memory.write32(m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                   : getEffectiveAddress16(modrm),
                     value);
}

void InstructionDecoder::writeModRM16(const ModRM &modrm, uint16_t value) {
  if (modrm.mod == 3)
    m_cpu.setReg16(modrm.rm, value);
  else
    m_memory.write16(m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                   : getEffectiveAddress16(modrm),
                     value);
}

void InstructionDecoder::writeModRM8(const ModRM &modrm, uint8_t value) {
  if (modrm.mod == 3)
    m_cpu.setReg8(modrm.rm, value);
  else
    m_memory.write8(m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                  : getEffectiveAddress16(modrm),
                    value);
}

uint32_t InstructionDecoder::aluOp(uint8_t op, uint32_t dest, uint32_t src,
                                   int size) {
  uint32_t res = 0;
  uint32_t mask = (size == 8) ? 0xFF : (size == 16) ? 0xFFFF : 0xFFFFFFFF;
  uint32_t msb = 1 << (size - 1);
  uint32_t flags =
      m_cpu.getEFLAGS() & ~(0x08D5); // Clear OF, SF, ZF, AF, PF, CF
  uint8_t cf = 0, of = 0, af = 0;
  uint8_t c_in = (m_cpu.getEFLAGS() & 1) ? 1 : 0;

  dest &= mask;
  src &= mask;

  switch (op) {
  case 0: // ADD
    res = dest + src;
    cf = (res & ~mask) ? 1 : 0;
    of = ((dest ^ res) & (src ^ res) & msb) ? 1 : 0;
    af = ((dest ^ src ^ res) & 0x10) ? 1 : 0;
    break;
  case 1: // OR
    res = dest | src;
    cf = 0;
    of = 0;
    af = 0;
    break;
  case 2: // ADC
    res = dest + src + c_in;
    cf = (res & ~mask) ? 1 : 0;
    of = ((dest ^ res) & (src ^ res) & msb) ? 1 : 0;
    af = ((dest ^ src ^ res) & 0x10) ? 1 : 0;
    break;
  case 3: // SBB
    res = dest - src - c_in;
    cf = (dest < src + c_in) ? 1 : 0;
    of = ((dest ^ src) & (dest ^ res) & msb) ? 1 : 0;
    af = ((dest ^ src ^ res) & 0x10) ? 1 : 0;
    break;
  case 4: // AND
    res = dest & src;
    cf = 0;
    of = 0;
    af = 0;
    break;
  case 5: // SUB
  case 7: // CMP
    res = dest - src;
    cf = (dest < src) ? 1 : 0;
    of = ((dest ^ src) & (dest ^ res) & msb) ? 1 : 0;
    af = ((dest ^ src ^ res) & 0x10) ? 1 : 0;
    break;
  case 6: // XOR
    res = dest ^ src;
    cf = 0;
    of = 0;
    af = 0;
    break;
  }

  res &= mask;

  if (cf)
    flags |= 0x0001; // CF
  if (res == 0)
    flags |= 0x0040; // ZF
  if (res & msb)
    flags |= 0x0080; // SF
  if (of)
    flags |= 0x0800; // OF
  if (af)
    flags |= 0x0010; // AF

  uint8_t p = res & 0xFF; // PF
  p ^= p >> 4;
  p ^= p >> 2;
  p ^= p >> 1;
  if (!(p & 1))
    flags |= 0x0004;

  m_cpu.setEFLAGS(flags);
  return res;
}

uint32_t InstructionDecoder::incOp(uint32_t val, int size) {
  uint32_t oldCF = m_cpu.getEFLAGS() & 0x0001;
  uint32_t result = aluOp(0, val, 1, size);
  m_cpu.setEFLAGS((m_cpu.getEFLAGS() & ~0x0001U) | oldCF);
  return result;
}

uint32_t InstructionDecoder::decOp(uint32_t val, int size) {
  uint32_t oldCF = m_cpu.getEFLAGS() & 0x0001;
  uint32_t result = aluOp(5, val, 1, size);
  m_cpu.setEFLAGS((m_cpu.getEFLAGS() & ~0x0001U) | oldCF);
  return result;
}

bool InstructionDecoder::checkCondition(uint8_t cond) {
  uint32_t flags = m_cpu.getEFLAGS();
  bool cf = flags & 0x0001, pf = flags & 0x0004;
  bool zf = flags & 0x0040, sf = flags & 0x0080;
  bool of = flags & 0x0800;

  switch (cond & 0x0F) {
  case 0x0:
    return of; // JO
  case 0x1:
    return !of; // JNO
  case 0x2:
    return cf; // JB / JC
  case 0x3:
    return !cf; // JNB / JNC
  case 0x4:
    return zf; // JZ / JE
  case 0x5:
    return !zf; // JNZ / JNE
  case 0x6:
    return cf || zf; // JBE / JNA
  case 0x7:
    return !cf && !zf; // JA / JNBE
  case 0x8:
    return sf; // JS
  case 0x9:
    return !sf; // JNS
  case 0xA:
    return pf; // JP / JPE
  case 0xB:
    return !pf; // JNP / JPO
  case 0xC:
    return sf != of; // JL / JNGE
  case 0xD:
    return sf == of; // JGE / JNL
  case 0xE:
    return zf || (sf != of); // JLE / JNG
  case 0xF:
    return !zf && (sf == of); // JG / JNLE
  default:
    return false;
  }
}

void InstructionDecoder::step() {
  m_stepCount++;

  m_hasPrefix66 = m_cpu.is32BitCode();
  m_hasPrefix67 = m_cpu.is32BitCode();
  m_hasRepnz = false;
  m_hasRepz = false;
  m_segmentOverride = 0xFF;
  m_eaResolved = false;

  m_instrStartEIP = m_cpu.getEIP();
  uint8_t opcode = fetch8();

  while (true) {
    switch (opcode) {
    case 0x66:
      m_hasPrefix66 = !m_hasPrefix66;
      opcode = fetch8();
      continue;
    case 0x67:
      m_hasPrefix67 = !m_hasPrefix67;
      opcode = fetch8();
      continue;
    case 0xF0:
      opcode = fetch8();
      continue; // LOCK prefix - no-op in our emulator
    case 0xF2:
      m_hasRepnz = true;
      opcode = fetch8();
      continue;
    case 0xF3:
      m_hasRepz = true;
      opcode = fetch8();
      continue;
    case 0x2E:
      m_segmentOverride = CS;
      opcode = fetch8();
      continue;
    case 0x36:
      m_segmentOverride = SS;
      opcode = fetch8();
      continue;
    case 0x3E:
      m_segmentOverride = DS;
      opcode = fetch8();
      continue;
    case 0x26:
      m_segmentOverride = ES;
      opcode = fetch8();
      continue;
    case 0x64:
      m_segmentOverride = FS;
      opcode = fetch8();
      continue;
    case 0x65:
      m_segmentOverride = GS;
      opcode = fetch8();
      continue;
    default:
      break;
    }
    break;
  }

  m_trace_cs[m_trace_idx % 32] = m_cpu.getSegReg(CS);
  m_trace_eip[m_trace_idx % 32] = m_cpu.getEIP() - 1;
  m_trace_op[m_trace_idx % 32] = opcode;
  m_trace_idx++;

  // TEMP: trace first 50 PM instructions after DPMI entry
  {
    static int pmTraceCount = 0;
    if (pmTraceCount > 0) {
      pmTraceCount--;
      LOG_WARN("PMTRACE[", pmTraceCount, "] CS:EIP=", std::hex,
               m_cpu.getSegReg(CS), ":", m_cpu.getEIP() - 1,
               " op=", (int)opcode,
               " AX=", m_cpu.getReg32(cpu::EAX),
               " BX=", m_cpu.getReg32(cpu::EBX),
               " CX=", m_cpu.getReg32(cpu::ECX),
               " DX=", m_cpu.getReg32(cpu::EDX),
               " SP=", m_cpu.getReg32(cpu::ESP),
               " BP=", m_cpu.getReg32(cpu::EBP),
               " SI=", m_cpu.getReg32(cpu::ESI),
               " DI=", m_cpu.getReg32(cpu::EDI),
               " DS=", m_cpu.getSegReg(DS),
               " ES=", m_cpu.getSegReg(ES),
               " SS=", m_cpu.getSegReg(SS),
               " FL=", m_cpu.getEFLAGS(),
               " csBase=", m_segBase[CS]);
    }
    // Trigger on DPMI entry completion (CS changes from real-mode to 0x0F)
    if ((m_cpu.getCR(0) & 1) && m_cpu.getSegReg(CS) == 0x0F && pmTraceCount == 0) {
      static bool triggered = false;
      if (!triggered) {
        triggered = true;
        pmTraceCount = 200;
        LOG_WARN("PMTRACE: PM entry detected, tracing 200 instructions");
      }
    }
  }

  if (opcode == 0x0F) {
    executeOpcode0F(fetch8());
  } else {
    // String operations repeat handling
    if (m_hasRepz || m_hasRepnz) {
      uint8_t stringOps[] = {0xA4, 0xA5, 0xA6, 0xA7, 0xAA, 0xAB, 0xAC,
                             0xAD, 0xAE, 0xAF, 0x6C, 0x6D, 0x6E, 0x6F};
      bool isStringOp = false;
      for (uint8_t op : stringOps)
        if (op == opcode) {
          isStringOp = true;
          break;
        }

      if (isStringOp) {
        // Execute up to a batch of iterations per step() call.
        // This prevents large REP operations (e.g. rep movsb cx=65535)
        // from blocking the main loop for seconds — the main loop
        // needs to run periodically for SDL event polling, rendering,
        // and audio generation.
        constexpr int MAX_REP_BATCH = 4096;
        int batchCount = 0;
        while (true) {
          uint32_t cx =
              m_hasPrefix67 ? m_cpu.getReg32(ECX) : m_cpu.getReg16(CX);
          if (cx == 0)
            break;

          executeOpcode(opcode);

          cx--;
          if (m_hasPrefix67)
            m_cpu.setReg32(ECX, cx);
          else
            m_cpu.setReg16(CX, static_cast<uint16_t>(cx));

          bool isCompare = (opcode == 0xA6 || opcode == 0xA7 ||
                            opcode == 0xAE || opcode == 0xAF);
          if (isCompare) {
            bool zf = (m_cpu.getEFLAGS() & 0x0040);
            if (m_hasRepz && !zf)
              break;
            if (m_hasRepnz && zf)
              break;
          }

          if (cx == 0)
            break;

          batchCount++;
          if (batchCount >= MAX_REP_BATCH) {
            // Yield back to the main loop; rewind IP to the start
            // of the REP instruction so it re-executes next step().
            m_cpu.setEIP(m_instrStartEIP);
            break;
          }
        }
        return;
      }
    }
    executeOpcode(opcode);
  }
}

void InstructionDecoder::executeOpcode(uint8_t opcode) {
  switch (opcode) {
  case 0x90: // NOP
    break;

  case 0xCC: // INT3
    triggerInterrupt(3);
    break;

  case 0xCD: { // INT imm8
    uint8_t vec = fetch8();
    triggerInterrupt(vec);
    break;
  }
  case 0xCE: { // INTO
    if (m_cpu.getEFLAGS() & 0x0800)
      triggerInterrupt(4);
    break;
  }

  case 0xCF: { // IRET/IRETD
    // TEMP: trace HLE pops from IRET in PM
    if ((m_cpu.getCR(0) & 1) && m_cpu.hleStackSize() > 0) {
      LOG_WARN("IRET-POP CS=0x", std::hex, m_cpu.getSegReg(CS),
               " EIP=0x", m_instrStartEIP,
               " hleDepth=", m_cpu.hleStackSize(),
               " is32=", m_hasPrefix66,
               " SP=0x", m_cpu.getReg16(SP),
               " ESP=0x", m_cpu.getReg32(ESP));
    }
    if (m_hasPrefix66) {
      uint32_t esp = m_cpu.getReg32(ESP);
      uint32_t newEip = m_memory.read32(m_segBase[SS] + esp);
      uint16_t newCs = m_memory.read32(m_segBase[SS] + esp + 4) & 0xFFFF;
      uint32_t newEflags = m_memory.read32(m_segBase[SS] + esp + 8);
      uint8_t oldCpl = m_cpu.getSegReg(CS) & 3;
      uint8_t newCpl = newCs & 3;
      bool wasV86 = (m_cpu.getEFLAGS() & 0x00020000) != 0;
      bool isV86 = (newEflags & 0x00020000) != 0;

      if (!wasV86 && isV86) {
        // Entering V86 from PM requires popping more registers (SS, ESP, ES,
        // DS, FS, GS)
        uint32_t newEsp = m_memory.read32(m_segBase[SS] + esp + 12);
        uint16_t newSs = m_memory.read32(m_segBase[SS] + esp + 16) & 0xFFFF;
        uint16_t newEs = m_memory.read32(m_segBase[SS] + esp + 20) & 0xFFFF;
        uint16_t newDs = m_memory.read32(m_segBase[SS] + esp + 24) & 0xFFFF;
        uint16_t newFs = m_memory.read32(m_segBase[SS] + esp + 28) & 0xFFFF;
        uint16_t newGs = m_memory.read32(m_segBase[SS] + esp + 32) & 0xFFFF;

        m_cpu.setEFLAGS(newEflags);
        m_cpu.setEIP(newEip);
        loadSegment(CS, newCs);
        m_cpu.setReg32(ESP, newEsp);
        loadSegment(SS, newSs);
        loadSegment(ES, newEs);
        loadSegment(DS, newDs);
        loadSegment(FS, newFs);
        loadSegment(GS, newGs);
      } else {
        m_cpu.setEFLAGS(newEflags);
        m_cpu.setEIP(newEip);
        loadSegment(CS, newCs);
        if (newCpl > oldCpl && (m_cpu.getCR(0) & 1)) {
          // PM privilege change (lower privilege)
          uint32_t newEsp = m_memory.read32(m_segBase[SS] + esp + 12);
          uint16_t newSs = m_memory.read32(m_segBase[SS] + esp + 16) & 0xFFFF;
          m_cpu.setReg32(ESP, newEsp);
          loadSegment(SS, newSs);
        } else {
          m_cpu.setReg32(ESP, esp + 12);
        }
      }
    } else {
      uint16_t sp = m_cpu.getReg16(SP);
      uint16_t newIp = m_memory.read16(m_segBase[SS] + sp);
      uint16_t newCs = m_memory.read16(m_segBase[SS] + sp + 2);
      uint16_t newFlags = m_memory.read16(m_segBase[SS] + sp + 4);
      uint8_t oldCpl = m_cpu.getSegReg(CS) & 3;
      uint8_t newCpl = newCs & 3;

      m_cpu.setEFLAGS((m_cpu.getEFLAGS() & 0xFFFF0000) | newFlags);
      m_cpu.setEIP(newIp);
      loadSegment(CS, newCs);

      if (newCpl > oldCpl && (m_cpu.getCR(0) & 1)) {
        uint16_t newSp = m_memory.read16(m_segBase[SS] + sp + 6);
        uint16_t newSs = m_memory.read16(m_segBase[SS] + sp + 8);
        m_cpu.setReg16(SP, newSp);
        loadSegment(SS, newSs);
      } else {
        m_cpu.setReg16(SP, (sp + 6) & 0xFFFF);
      }
    }
    // Do NOT pop HLE frame here. The frame is managed by the 0F FF handler
    // (via popHLEFrameForVector) when the stub processes the INT. This avoids
    // popping our frame when a client thunk uses IRETD to forward to the
    // old handler (our stub) — the thunk's IRETD would incorrectly pop the
    // frame before the stub had a chance to use it.
    break;
  }

  // AL/AX/EAX Immediates (0x04 - 0x3D)
  case 0x04:
    m_cpu.setReg8(AL, aluOp(0, m_cpu.getReg8(AL), fetch8(), 8));
    break;
  case 0x05:
    if (m_hasPrefix66)
      m_cpu.setReg32(EAX, aluOp(0, m_cpu.getReg32(EAX), fetch32(), 32));
    else
      m_cpu.setReg16(AX, aluOp(0, m_cpu.getReg16(AX), fetch16(), 16));
    break;
  case 0x0C:
    m_cpu.setReg8(AL, aluOp(1, m_cpu.getReg8(AL), fetch8(), 8));
    break;
  case 0x0D:
    if (m_hasPrefix66)
      m_cpu.setReg32(EAX, aluOp(1, m_cpu.getReg32(EAX), fetch32(), 32));
    else
      m_cpu.setReg16(AX, aluOp(1, m_cpu.getReg16(AX), fetch16(), 16));
    break;
  case 0x14:
    m_cpu.setReg8(AL, aluOp(2, m_cpu.getReg8(AL), fetch8(), 8));
    break;
  case 0x15:
    if (m_hasPrefix66)
      m_cpu.setReg32(EAX, aluOp(2, m_cpu.getReg32(EAX), fetch32(), 32));
    else
      m_cpu.setReg16(AX, aluOp(2, m_cpu.getReg16(AX), fetch16(), 16));
    break;
  case 0x1C:
    m_cpu.setReg8(AL, aluOp(3, m_cpu.getReg8(AL), fetch8(), 8));
    break;
  case 0x1D:
    if (m_hasPrefix66)
      m_cpu.setReg32(EAX, aluOp(3, m_cpu.getReg32(EAX), fetch32(), 32));
    else
      m_cpu.setReg16(AX, aluOp(3, m_cpu.getReg16(AX), fetch16(), 16));
    break;
  case 0x24:
    m_cpu.setReg8(AL, aluOp(4, m_cpu.getReg8(AL), fetch8(), 8));
    break;
  case 0x25:
    if (m_hasPrefix66)
      m_cpu.setReg32(EAX, aluOp(4, m_cpu.getReg32(EAX), fetch32(), 32));
    else
      m_cpu.setReg16(AX, aluOp(4, m_cpu.getReg16(AX), fetch16(), 16));
    break;
  case 0x2C:
    m_cpu.setReg8(AL, aluOp(5, m_cpu.getReg8(AL), fetch8(), 8));
    break;
  case 0x2D:
    if (m_hasPrefix66)
      m_cpu.setReg32(EAX, aluOp(5, m_cpu.getReg32(EAX), fetch32(), 32));
    else
      m_cpu.setReg16(AX, aluOp(5, m_cpu.getReg16(AX), fetch16(), 16));
    break;
  case 0x34:
    m_cpu.setReg8(AL, aluOp(6, m_cpu.getReg8(AL), fetch8(), 8));
    break;
  case 0x35:
    if (m_hasPrefix66)
      m_cpu.setReg32(EAX, aluOp(6, m_cpu.getReg32(EAX), fetch32(), 32));
    else
      m_cpu.setReg16(AX, aluOp(6, m_cpu.getReg16(AX), fetch16(), 16));
    break;
  case 0x3C:
    aluOp(7, m_cpu.getReg8(AL), fetch8(), 8);
    break;
  case 0x3D:
    if (m_hasPrefix66)
      aluOp(7, m_cpu.getReg32(EAX), fetch32(), 32);
    else
      aluOp(7, m_cpu.getReg16(AX), fetch16(), 16);
    break;

  // BCD / ASCII adjust
  case 0x27: { // DAA
    uint8_t oldAL = m_cpu.getReg8(AL);
    uint8_t oldCF = (m_cpu.getEFLAGS() & FLAG_CARRY) ? 1 : 0;
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_CARRY);
    if ((oldAL & 0x0F) > 9 || (m_cpu.getEFLAGS() & FLAG_AUX)) {
      m_cpu.setReg8(AL, oldAL + 6);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY |
                      (oldCF ? FLAG_CARRY : 0));
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_AUX);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_AUX);
    }
    if (oldAL > 0x99 || oldCF) {
      m_cpu.setReg8(AL, m_cpu.getReg8(AL) + 0x60);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
    }
    break;
  }
  case 0x2F: { // DAS
    uint8_t oldAL = m_cpu.getReg8(AL);
    uint8_t oldCF = (m_cpu.getEFLAGS() & FLAG_CARRY) ? 1 : 0;
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_CARRY);
    if ((oldAL & 0x0F) > 9 || (m_cpu.getEFLAGS() & FLAG_AUX)) {
      m_cpu.setReg8(AL, oldAL - 6);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY |
                      (oldCF ? FLAG_CARRY : 0));
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_AUX);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_AUX);
    }
    if (oldAL > 0x99 || oldCF) {
      m_cpu.setReg8(AL, m_cpu.getReg8(AL) - 0x60);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
    }
    break;
  }
  case 0x37: { // AAA
    if ((m_cpu.getReg8(AL) & 0x0F) > 9 || (m_cpu.getEFLAGS() & FLAG_AUX)) {
      m_cpu.setReg16(AX, m_cpu.getReg16(AX) + 0x106);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_AUX | FLAG_CARRY);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~(FLAG_AUX | FLAG_CARRY));
    }
    m_cpu.setReg8(AL, m_cpu.getReg8(AL) & 0x0F);
    break;
  }
  case 0x3F: { // AAS
    if ((m_cpu.getReg8(AL) & 0x0F) > 9 || (m_cpu.getEFLAGS() & FLAG_AUX)) {
      m_cpu.setReg16(AX, m_cpu.getReg16(AX) - 6);
      m_cpu.setReg8(AH, m_cpu.getReg8(AH) - 1);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_AUX | FLAG_CARRY);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~(FLAG_AUX | FLAG_CARRY));
    }
    m_cpu.setReg8(AL, m_cpu.getReg8(AL) & 0x0F);
    break;
  }
  case 0xD4: { // AAM
    uint8_t base = fetch8();
    if (base == 0)
      triggerInterrupt(0); // Divide by zero
    else {
      m_cpu.setReg8(AH, m_cpu.getReg8(AL) / base);
      m_cpu.setReg8(AL, m_cpu.getReg8(AL) % base);
    }
    break;
  }
  case 0xD5: { // AAD
    uint8_t base = fetch8();
    m_cpu.setReg8(AL, m_cpu.getReg8(AH) * base + m_cpu.getReg8(AL));
    m_cpu.setReg8(AH, 0);
    break;
  }

  // Segment Push/Pop
  case 0x06:
    if (m_hasPrefix66)
      m_cpu.push32(m_cpu.getSegReg(ES));
    else
      m_cpu.push16(m_cpu.getSegReg(ES));
    break;
  case 0x07:
    if (m_hasPrefix66)
      loadSegment(ES, m_cpu.pop32() & 0xFFFF);
    else
      loadSegment(ES, m_cpu.pop16());
    break;
  case 0x0E:
    if (m_hasPrefix66) {
      m_cpu.push32(m_cpu.getSegReg(CS));
    } else
      m_cpu.push16(m_cpu.getSegReg(CS));
    break;
  case 0x16:
    if (m_hasPrefix66)
      m_cpu.push32(m_cpu.getSegReg(SS));
    else
      m_cpu.push16(m_cpu.getSegReg(SS));
    break;
  case 0x17:
    if (m_hasPrefix66)
      loadSegment(SS, m_cpu.pop32() & 0xFFFF);
    else
      loadSegment(SS, m_cpu.pop16());
    break;
  case 0x1E:
    if (m_hasPrefix66)
      m_cpu.push32(m_cpu.getSegReg(DS));
    else
      m_cpu.push16(m_cpu.getSegReg(DS));
    break;
  case 0x1F:
    if (m_hasPrefix66)
      loadSegment(DS, m_cpu.pop32() & 0xFFFF);
    else
      loadSegment(DS, m_cpu.pop16());
    break;

  case 0x9B:
    break; // FWAIT/WAIT — no FPU, no-op

  // PUSHF / POPF
  case 0x9C:
    if (m_hasPrefix66)
      m_cpu.push32(m_cpu.getEFLAGS());
    else
      m_cpu.push16(static_cast<uint16_t>(m_cpu.getEFLAGS()));
    break;
  case 0x9D:
    if (m_hasPrefix66)
      m_cpu.setEFLAGS(m_cpu.pop32());
    else
      m_cpu.setEFLAGS((m_cpu.getEFLAGS() & 0xFFFF0000) | m_cpu.pop16());
    break;

  // SAHF / LAHF
  case 0x9E: { // SAHF — Store AH into Flags (SF,ZF,AF,PF,CF)
    uint32_t flags = m_cpu.getEFLAGS();
    uint8_t ah = m_cpu.getReg8(AH);
    // Only SF,ZF,AF,PF,CF are affected (bits 7,6,4,2,0)
    constexpr uint32_t mask = 0xD5;               // 1101 0101
    flags = (flags & ~mask) | (ah & mask) | 0x02; // bit 1 always set
    m_cpu.setEFLAGS(flags);
    break;
  }
  case 0x9F: { // LAHF — Load AH from Flags (SF,ZF,AF,PF,CF)
    m_cpu.setReg8(AH, static_cast<uint8_t>(m_cpu.getEFLAGS() & 0xFF));
    break;
  }

  // Std Alu r/m block (0x00 - 0x3B pattern matching extracted to aluOp)
  case 0x00:
  case 0x08:
  case 0x10:
  case 0x18:
  case 0x20:
  case 0x28:
  case 0x30:
  case 0x38: {
    uint8_t op = (opcode >> 3) & 0x07;
    ModRM modrm = decodeModRM(fetch8());
    uint8_t res = aluOp(op, readModRM8(modrm), m_cpu.getReg8(modrm.reg), 8);
    if (op != 7)
      writeModRM8(modrm, res);
    break;
  }
  case 0x01:
  case 0x09:
  case 0x11:
  case 0x19:
  case 0x21:
  case 0x29:
  case 0x31:
  case 0x39: {
    uint8_t op = (opcode >> 3) & 0x07;
    ModRM modrm = decodeModRM(fetch8());
    if (m_hasPrefix66) {
      uint32_t res =
          aluOp(op, readModRM32(modrm), m_cpu.getReg32(modrm.reg), 32);
      if (op != 7)
        writeModRM32(modrm, res);
    } else {
      uint16_t res =
          aluOp(op, readModRM16(modrm), m_cpu.getReg16(modrm.reg), 16);
      if (op != 7)
        writeModRM16(modrm, res);
    }
    break;
  }
  case 0x02:
  case 0x0A:
  case 0x12:
  case 0x1A:
  case 0x22:
  case 0x2A:
  case 0x32:
  case 0x3A: {
    uint8_t op = (opcode >> 3) & 0x07;
    ModRM modrm = decodeModRM(fetch8());
    uint8_t res = aluOp(op, m_cpu.getReg8(modrm.reg), readModRM8(modrm), 8);
    if (op != 7)
      m_cpu.setReg8(modrm.reg, res);
    break;
  }
  case 0x03:
  case 0x0B:
  case 0x13:
  case 0x1B:
  case 0x23:
  case 0x2B:
  case 0x33:
  case 0x3B: {
    uint8_t op = (opcode >> 3) & 0x07;
    ModRM modrm = decodeModRM(fetch8());
    if (m_hasPrefix66) {
      uint32_t res =
          aluOp(op, m_cpu.getReg32(modrm.reg), readModRM32(modrm), 32);
      if (op != 7)
        m_cpu.setReg32(modrm.reg, res);
    } else {
      uint16_t res =
          aluOp(op, m_cpu.getReg16(modrm.reg), readModRM16(modrm), 16);
      if (op != 7)
        m_cpu.setReg16(modrm.reg, res);
    }
    break;
  }

  // INC / DEC Reg
  case 0x40:
  case 0x41:
  case 0x42:
  case 0x43:
  case 0x44:
  case 0x45:
  case 0x46:
  case 0x47: {
    uint8_t reg = opcode & 0x07;
    if (m_hasPrefix66)
      m_cpu.setReg32(reg, incOp(m_cpu.getReg32(reg), 32));
    else
      m_cpu.setReg16(reg, incOp(m_cpu.getReg16(reg), 16));
    break;
  }
  case 0x48:
  case 0x49:
  case 0x4A:
  case 0x4B:
  case 0x4C:
  case 0x4D:
  case 0x4E:
  case 0x4F: {
    uint8_t reg = opcode & 0x07;
    if (m_hasPrefix66)
      m_cpu.setReg32(reg, decOp(m_cpu.getReg32(reg), 32));
    else
      m_cpu.setReg16(reg, decOp(m_cpu.getReg16(reg), 16));
    break;
  }

  // PUSH / POP Reg
  case 0x50:
  case 0x51:
  case 0x52:
  case 0x53:
  case 0x54:
  case 0x55:
  case 0x56:
  case 0x57: {
    uint8_t reg = opcode & 0x07;
    if (m_hasPrefix66)
      m_cpu.push32(m_cpu.getReg32(reg));
    else
      m_cpu.push16(m_cpu.getReg16(reg));
    break;
  }
  case 0x58:
  case 0x59:
  case 0x5A:
  case 0x5B:
  case 0x5C:
  case 0x5D:
  case 0x5E:
  case 0x5F: {
    uint8_t reg = opcode & 0x07;
    if (m_hasPrefix66)
      m_cpu.setReg32(reg, m_cpu.pop32());
    else
      m_cpu.setReg16(reg, m_cpu.pop16());
    break;
  }

  case 0x60: { // PUSHA / PUSHAD
    if (m_hasPrefix66) {
      uint32_t valESP = m_cpu.getReg32(ESP);
      m_cpu.push32(m_cpu.getReg32(EAX));
      m_cpu.push32(m_cpu.getReg32(ECX));
      m_cpu.push32(m_cpu.getReg32(EDX));
      m_cpu.push32(m_cpu.getReg32(EBX));
      m_cpu.push32(valESP);
      m_cpu.push32(m_cpu.getReg32(EBP));
      m_cpu.push32(m_cpu.getReg32(ESI));
      m_cpu.push32(m_cpu.getReg32(EDI));
    } else {
      uint16_t valSP = m_cpu.getReg16(SP);
      m_cpu.push16(m_cpu.getReg16(AX));
      m_cpu.push16(m_cpu.getReg16(CX));
      m_cpu.push16(m_cpu.getReg16(DX));
      m_cpu.push16(m_cpu.getReg16(BX));
      m_cpu.push16(valSP);
      m_cpu.push16(m_cpu.getReg16(BP));
      m_cpu.push16(m_cpu.getReg16(SI));
      m_cpu.push16(m_cpu.getReg16(DI));
    }
    break;
  }
  case 0x61: { // POPA / POPAD
    if (m_hasPrefix66) {
      m_cpu.setReg32(EDI, m_cpu.pop32());
      m_cpu.setReg32(ESI, m_cpu.pop32());
      m_cpu.setReg32(EBP, m_cpu.pop32());
      m_cpu.pop32(); // Skip ESP
      m_cpu.setReg32(EBX, m_cpu.pop32());
      m_cpu.setReg32(EDX, m_cpu.pop32());
      m_cpu.setReg32(ECX, m_cpu.pop32());
      m_cpu.setReg32(EAX, m_cpu.pop32());
    } else {
      m_cpu.setReg16(DI, m_cpu.pop16());
      m_cpu.setReg16(SI, m_cpu.pop16());
      m_cpu.setReg16(BP, m_cpu.pop16());
      m_cpu.pop16(); // Skip SP
      m_cpu.setReg16(BX, m_cpu.pop16());
      m_cpu.setReg16(DX, m_cpu.pop16());
      m_cpu.setReg16(CX, m_cpu.pop16());
      m_cpu.setReg16(AX, m_cpu.pop16());
    }
    break;
  }
  case 0x63: { // ARPL r/m16, r16
    ModRM modrm = decodeModRM(fetch8());
    uint16_t src = m_cpu.getReg16(modrm.reg);
    uint16_t dest = readModRM16(modrm);
    if ((dest & 3) < (src & 3)) {
      dest = (dest & ~3) | (src & 3);
      writeModRM16(modrm, dest);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_ZERO);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_ZERO);
    }
    break;
  }

  case 0x69: { // IMUL r, r/m, imm16/32
    ModRM modrm = decodeModRM(fetch8());
    if (m_hasPrefix66) {
      int32_t src1 = (int32_t)readModRM32(modrm);
      int32_t src2 = (int32_t)fetch32();
      int64_t result = (int64_t)src1 * src2;
      m_cpu.setReg32(modrm.reg, (uint32_t)result);
      // CF/OF are set if result doesn't fit in 32 bits
      uint32_t flags = m_cpu.getEFLAGS();
      if (result < -2147483648LL || result > 2147483647LL)
        flags |= (FLAG_CARRY | FLAG_OVERFLOW);
      else
        flags &= ~(FLAG_CARRY | FLAG_OVERFLOW);
      m_cpu.setEFLAGS(flags);
    } else {
      int16_t src1 = (int16_t)readModRM16(modrm);
      int16_t src2 = (int16_t)fetch16();
      int32_t result = (int32_t)src1 * src2;
      m_cpu.setReg16(modrm.reg, (uint16_t)result);
      uint32_t flags = m_cpu.getEFLAGS();
      if (result < -32768 || result > 32767)
        flags |= (FLAG_CARRY | FLAG_OVERFLOW);
      else
        flags &= ~(FLAG_CARRY | FLAG_OVERFLOW);
      m_cpu.setEFLAGS(flags);
    }
    break;
  }
  case 0x6B: { // IMUL r, r/m, imm8
    ModRM modrm = decodeModRM(fetch8());
    if (m_hasPrefix66) {
      int32_t src1 = (int32_t)readModRM32(modrm);
      int32_t src2 = (int32_t)(int8_t)fetch8();
      int64_t result = (int64_t)src1 * src2;
      m_cpu.setReg32(modrm.reg, (uint32_t)result);
      uint32_t flags = m_cpu.getEFLAGS();
      if (result < -2147483648LL || result > 2147483647LL)
        flags |= (FLAG_CARRY | FLAG_OVERFLOW);
      else
        flags &= ~(FLAG_CARRY | FLAG_OVERFLOW);
      m_cpu.setEFLAGS(flags);
    } else {
      int16_t src1 = (int16_t)readModRM16(modrm);
      int16_t src2 = (int16_t)(int8_t)fetch8();
      int32_t result = (int32_t)src1 * src2;
      m_cpu.setReg16(modrm.reg, (uint16_t)result);
      uint32_t flags = m_cpu.getEFLAGS();
      if (result < -32768 || result > 32767)
        flags |= (FLAG_CARRY | FLAG_OVERFLOW);
      else
        flags &= ~(FLAG_CARRY | FLAG_OVERFLOW);
      m_cpu.setEFLAGS(flags);
    }
    break;
  }

  case 0x62:
    decodeModRM(fetch8());
    LOG_CPU("BOUND - stubbed");
    break;

  case 0x68: { // PUSH imm16/32
    if (m_hasPrefix66)
      m_cpu.push32(fetch32());
    else
      m_cpu.push16(fetch16());
    break;
  }
  case 0x6A:
    m_cpu.push16(static_cast<int16_t>(static_cast<int8_t>(fetch8())));
    break; // PUSH imm8
  case 0x6C:
  case 0x6D:
  case 0x6E:
  case 0x6F:
    LOG_CPU("INS/OUTS - stubbed");
    break;

  // Short Jumps
  case 0x70:
  case 0x71:
  case 0x72:
  case 0x73:
  case 0x74:
  case 0x75:
  case 0x76:
  case 0x77:
  case 0x78:
  case 0x79:
  case 0x7A:
  case 0x7B:
  case 0x7C:
  case 0x7D:
  case 0x7E:
  case 0x7F: {
    int8_t disp = static_cast<int8_t>(fetch8());
    if (checkCondition(opcode))
      m_cpu.setEIP(m_cpu.getEIP() + disp);
    break;
  }

  // TEST
  case 0x84: { // TEST r/m8, r8
    ModRM modrm = decodeModRM(fetch8());
    aluOp(4, readModRM8(modrm), m_cpu.getReg8(modrm.reg), 8);
    break;
  }
  case 0x85: { // TEST r/m16/32, r16/32
    ModRM modrm = decodeModRM(fetch8());
    if (m_hasPrefix66)
      aluOp(4, readModRM32(modrm), m_cpu.getReg32(modrm.reg), 32);
    else
      aluOp(4, readModRM16(modrm), m_cpu.getReg16(modrm.reg), 16);
    break;
  }
  case 0xA8:
    aluOp(4, m_cpu.getReg8(AL), fetch8(), 8);
    break;   // TEST AL, imm8
  case 0xA9: // TEST AX/EAX, imm16/32
    if (m_hasPrefix66)
      aluOp(4, m_cpu.getReg32(EAX), fetch32(), 32);
    else
      aluOp(4, m_cpu.getReg16(AX), fetch16(), 16);
    break;

  // MOV
  case 0x88: {
    ModRM m = decodeModRM(fetch8());
    writeModRM8(m, m_cpu.getReg8(m.reg));
    break;
  }
  case 0x89: {
    ModRM m = decodeModRM(fetch8());
    if (m_hasPrefix66)
      writeModRM32(m, m_cpu.getReg32(m.reg));
    else
      writeModRM16(m, m_cpu.getReg16(m.reg));
    break;
  }
  case 0x8A: {
    ModRM m = decodeModRM(fetch8());
    m_cpu.setReg8(m.reg, readModRM8(m));
    break;
  }
  case 0x8B: {
    ModRM m = decodeModRM(fetch8());
    if (m_hasPrefix66)
      m_cpu.setReg32(m.reg, readModRM32(m));
    else
      m_cpu.setReg16(m.reg, readModRM16(m));
    break;
  }
  case 0x8C: {
    ModRM m = decodeModRM(fetch8());
    writeModRM16(m, m_cpu.getSegReg(static_cast<SegRegIndex>(m.reg)));
    break;
  }

  case 0x8D: { // LEA
    ModRM modrm = decodeModRM(fetch8());
    m_hasPrefix67 ? getEffectiveAddress32(modrm) : getEffectiveAddress16(modrm);
    if (m_hasPrefix66)
      m_cpu.setReg32(modrm.reg,
                     m_currentOffset); // Use isolated offset, not EA!
    else
      m_cpu.setReg16(modrm.reg, static_cast<uint16_t>(m_currentOffset));
    break;
  }
  case 0x8E: {
    ModRM m = decodeModRM(fetch8());
    uint16_t segVal = readModRM16(m);
    auto segIdx = static_cast<SegRegIndex>(m.reg);
    loadSegment(segIdx, segVal);
    break;
  }
  case 0x98: { // CBW / CWDE
    if (m_hasPrefix66) {
      m_cpu.setReg32(EAX, (uint32_t)(int16_t)m_cpu.getReg16(AX));
    } else {
      m_cpu.setReg16(AX, (uint16_t)(int8_t)m_cpu.getReg8(AL));
    }
    break;
  }
  case 0x99: { // CWD / CDQ
    if (m_hasPrefix66) {
      int32_t eax = (int32_t)m_cpu.getReg32(EAX);
      m_cpu.setReg32(EDX, eax < 0 ? 0xFFFFFFFF : 0);
    } else {
      int16_t ax = (int16_t)m_cpu.getReg16(AX);
      m_cpu.setReg16(DX, ax < 0 ? 0xFFFF : 0);
    }
    break;
  }

  case 0xC8: { // ENTER
    uint16_t size = fetch16();
    uint8_t level = fetch8();
    if (level != 0) {
      LOG_ERROR("ENTER with non-zero level=", (int)level, " size=", size,
                " at CS:IP=", std::hex, m_cpu.getSegReg(CS), ":",
                m_cpu.getEIP(), " SP=", m_cpu.getReg16(SP));
    }
    m_cpu.push16(m_cpu.getReg16(BP));
    m_cpu.setReg16(BP, m_cpu.getReg16(SP));
    // simplified, doesn't handle nested levels correctly yet
    m_cpu.setReg16(SP, m_cpu.getReg16(SP) - size);
    break;
  }
  case 0xc9: { // LEAVE
    if (m_hasPrefix66) {
      m_cpu.setReg32(ESP, m_cpu.getReg32(EBP));
      m_cpu.setReg32(EBP, m_cpu.pop32());
    } else {
      m_cpu.setReg16(SP, m_cpu.getReg16(BP));
      m_cpu.setReg16(BP, m_cpu.pop16());
    }
    break;
  }

  case 0xC4: { // LES
    ModRM modrm = decodeModRM(fetch8());
    uint32_t ea = m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                : getEffectiveAddress16(modrm);
    if (m_hasPrefix66) {
      m_cpu.setReg32(modrm.reg, m_memory.read32(ea));
      loadSegment(ES, m_memory.read16(ea + 4));
    } else {
      m_cpu.setReg16(modrm.reg, m_memory.read16(ea));
      loadSegment(ES, m_memory.read16(ea + 2));
    }
    break;
  }
  case 0xC5: { // LDS
    ModRM modrm = decodeModRM(fetch8());
    uint32_t ea = m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                : getEffectiveAddress16(modrm);
    if (m_hasPrefix66) {
      m_cpu.setReg32(modrm.reg, m_memory.read32(ea));
      loadSegment(DS, m_memory.read16(ea + 4));
    } else {
      m_cpu.setReg16(modrm.reg, m_memory.read16(ea));
      loadSegment(DS, m_memory.read16(ea + 2));
    }
    break;
  }
  case 0x8F: { // POP r/m
    ModRM modrm = decodeModRM(fetch8());
    if (m_hasPrefix66)
      writeModRM32(modrm, m_cpu.pop32());
    else
      writeModRM16(modrm, m_cpu.pop16());
    break;
  }

  case 0xC6: { // MOV r/m8, imm8
    ModRM modrm = decodeModRM(fetch8());
    // Resolve EA before fetching immediate so displacement is consumed first
    if (modrm.mod != 3) {
      if (m_hasPrefix67)
        getEffectiveAddress32(modrm);
      else
        getEffectiveAddress16(modrm);
    }
    writeModRM8(modrm, fetch8());
    break;
  }

  case 0xC7: { // MOV r/m, imm
    ModRM modrm = decodeModRM(fetch8());
    // Resolve EA before fetching immediate so displacement is consumed first
    if (modrm.mod != 3) {
      if (m_hasPrefix67)
        getEffectiveAddress32(modrm);
      else
        getEffectiveAddress16(modrm);
    }
    if (m_hasPrefix66)
      writeModRM32(modrm, fetch32());
    else
      writeModRM16(modrm, fetch16());
    break;
  }

  // Group 1: 0x80 - 0x83
  case 0x80:
  case 0x81:
  case 0x82:
  case 0x83: {
    ModRM modrm = decodeModRM(fetch8());
    uint32_t val1 = 0, val2 = 0;
    bool is8 = (opcode == 0x80 || opcode == 0x82);
    bool signExt = (opcode == 0x83);

    if (is8) {
      val1 = readModRM8(modrm);
      val2 = fetch8();
    } else if (signExt) {
      if (m_hasPrefix66) {
        val1 = readModRM32(modrm);
        val2 = static_cast<int32_t>(static_cast<int8_t>(fetch8()));
      } else {
        val1 = readModRM16(modrm);
        val2 = static_cast<int16_t>(static_cast<int8_t>(fetch8()));
      }
    } else {
      if (m_hasPrefix66) {
        val1 = readModRM32(modrm);
        val2 = fetch32();
      } else {
        val1 = readModRM16(modrm);
        val2 = fetch16();
      }
    }

    uint32_t res =
        aluOp(modrm.reg, val1, val2, is8 ? 8 : (m_hasPrefix66 ? 32 : 16));
    if (modrm.reg != 7) { // Only write if not CMP
      if (is8)
        writeModRM8(modrm, static_cast<uint8_t>(res));
      else if (m_hasPrefix66)
        writeModRM32(modrm, res);
      else
        writeModRM16(modrm, static_cast<uint16_t>(res));
    }
    break;
  }

  // Group 3: 0xF6, 0xF7
  case 0xF6:
  case 0xF7: {
    ModRM modrm = decodeModRM(fetch8());
    if (opcode == 0xF6) {
      uint8_t rmVal = readModRM8(modrm);
      switch (modrm.reg) {
      case 0:
      case 1:
        aluOp(4, rmVal, fetch8(), 8);
        break; // TEST
      case 2:
        writeModRM8(modrm, ~rmVal);
        break; // NOT
      case 3:
        writeModRM8(modrm, aluOp(5, 0, rmVal, 8));
        break;  // NEG
      case 4: { // MUL
        uint16_t res = static_cast<uint16_t>(m_cpu.getReg8(AL)) * rmVal;
        m_cpu.setReg16(AX, res);
        // OF & CF logic omitted for brevity
        break;
      }
      case 5: { // IMUL
        int16_t res =
            static_cast<int16_t>(static_cast<int8_t>(m_cpu.getReg8(AL))) *
            static_cast<int8_t>(rmVal);
        m_cpu.setReg16(AX, static_cast<uint16_t>(res));
        break;
      }
      case 6: { // DIV
        if (rmVal == 0)
          triggerInterrupt(0);
        else {
          uint16_t ax = m_cpu.getReg16(AX);
          uint16_t quotient = ax / rmVal;
          if (quotient > 0xFF) {
            triggerInterrupt(0);
            break;
          }
          m_cpu.setReg8(AL, static_cast<uint8_t>(quotient));
          m_cpu.setReg8(AH, static_cast<uint8_t>(ax % rmVal));
        }
        break;
      }
      case 7: { // IDIV
        if (rmVal == 0)
          triggerInterrupt(0);
        else {
          int16_t ax = static_cast<int16_t>(m_cpu.getReg16(AX));
          int8_t div = static_cast<int8_t>(rmVal);
          int16_t quotient = ax / div;
          if (quotient > 127 || quotient < -128) {
            triggerInterrupt(0);
            break;
          }
          m_cpu.setReg8(AL, static_cast<uint8_t>(quotient));
          m_cpu.setReg8(AH, static_cast<uint8_t>(ax % div));
        }
        break;
      }
      }
    } else {
      if (m_hasPrefix66) {
        uint32_t rmVal = readModRM32(modrm);
        switch (modrm.reg) {
        case 0:
        case 1:
          aluOp(4, rmVal, fetch32(), 32);
          break; // TEST
        case 2:
          writeModRM32(modrm, ~rmVal);
          break; // NOT
        case 3:
          writeModRM32(modrm, aluOp(5, 0, rmVal, 32));
          break;  // NEG
        case 4: { // MUL
          uint64_t res = static_cast<uint64_t>(m_cpu.getReg32(EAX)) * rmVal;
          m_cpu.setReg32(EAX, static_cast<uint32_t>(res));
          m_cpu.setReg32(EDX, static_cast<uint32_t>(res >> 32));
          break;
        }
        case 5: { // IMUL
          int64_t res =
              static_cast<int64_t>(static_cast<int32_t>(m_cpu.getReg32(EAX))) *
              static_cast<int32_t>(rmVal);
          m_cpu.setReg32(EAX, static_cast<uint32_t>(res));
          m_cpu.setReg32(EDX, static_cast<uint32_t>(res >> 32));
          break;
        }
        case 6: { // DIV
          if (rmVal == 0)
            triggerInterrupt(0);
          else {
            uint64_t edxeax =
                (static_cast<uint64_t>(m_cpu.getReg32(EDX)) << 32) |
                m_cpu.getReg32(EAX);
            uint64_t quotient = edxeax / rmVal;
            if (quotient > 0xFFFFFFFFULL) {
              triggerInterrupt(0);
              break;
            }
            m_cpu.setReg32(EAX, static_cast<uint32_t>(quotient));
            m_cpu.setReg32(EDX, static_cast<uint32_t>(edxeax % rmVal));
          }
          break;
        }
        case 7: { // IDIV
          if (rmVal == 0)
            triggerInterrupt(0);
          else {
            int64_t edxeax = (static_cast<int64_t>(m_cpu.getReg32(EDX)) << 32) |
                             m_cpu.getReg32(EAX);
            int32_t divisor = static_cast<int32_t>(rmVal);
            int64_t quotient = edxeax / divisor;
            if (quotient > INT32_MAX || quotient < INT32_MIN) {
              triggerInterrupt(0);
              break;
            }
            m_cpu.setReg32(EAX, static_cast<uint32_t>(quotient));
            m_cpu.setReg32(EDX, static_cast<uint32_t>(edxeax % divisor));
          }
          break;
        }
        default:
          LOG_WARN("Group 3 r32 reg ", (int)modrm.reg,
                   " not fully implemented");
          break;
        }
      } else {
        uint16_t rmVal = readModRM16(modrm);
        switch (modrm.reg) {
        case 0:
        case 1:
          aluOp(4, rmVal, fetch16(), 16);
          break; // TEST
        case 2:
          writeModRM16(modrm, ~rmVal);
          break; // NOT
        case 3:
          writeModRM16(modrm, aluOp(5, 0, rmVal, 16));
          break;  // NEG
        case 4: { // MUL
          uint32_t res = static_cast<uint32_t>(m_cpu.getReg16(AX)) * rmVal;
          m_cpu.setReg16(AX, static_cast<uint16_t>(res));
          m_cpu.setReg16(DX, static_cast<uint16_t>(res >> 16));
          break;
        }
        case 5: { // IMUL
          int32_t res =
              static_cast<int32_t>(static_cast<int16_t>(m_cpu.getReg16(AX))) *
              static_cast<int16_t>(rmVal);
          m_cpu.setReg16(AX, static_cast<uint16_t>(res));
          m_cpu.setReg16(DX, static_cast<uint16_t>(res >> 16));
          break;
        }
        case 6: { // DIV
          if (rmVal == 0)
            triggerInterrupt(0);
          else {
            uint32_t dxax = (static_cast<uint32_t>(m_cpu.getReg16(DX)) << 16) |
                            m_cpu.getReg16(AX);
            uint32_t quotient = dxax / rmVal;
            if (quotient > 0xFFFF) {
              triggerInterrupt(0);
              break;
            }
            m_cpu.setReg16(AX, static_cast<uint16_t>(quotient));
            m_cpu.setReg16(DX, static_cast<uint16_t>(dxax % rmVal));
          }
          break;
        }
        case 7: { // IDIV
          if (rmVal == 0)
            triggerInterrupt(0);
          else {
            int32_t dxax = (static_cast<int32_t>(m_cpu.getReg16(DX)) << 16) |
                           m_cpu.getReg16(AX);
            int16_t divisor = static_cast<int16_t>(rmVal);
            int32_t quotient = dxax / divisor;
            if (quotient > 32767 || quotient < -32768) {
              triggerInterrupt(0);
              break;
            }
            m_cpu.setReg16(AX, static_cast<uint16_t>(quotient));
            m_cpu.setReg16(DX, static_cast<uint16_t>(dxax % divisor));
          }
          break;
        }
        default:
          LOG_WARN("Group 3 r16 reg ", (int)modrm.reg,
                   " not fully implemented");
          break;
        }
      }
    }
    break;
  }

  // LOOPs
  case 0xE0:
  case 0xE1:
  case 0xE2:
  case 0xE3: {
    int8_t disp = static_cast<int8_t>(fetch8());
    uint32_t count = m_hasPrefix67 ? m_cpu.getReg32(ECX) : m_cpu.getReg16(CX);
    bool jump = false;
    if (opcode == 0xE3) { // JCXZ
      jump = (count == 0);
    } else {
      count--;
      if (m_hasPrefix67)
        m_cpu.setReg32(ECX, count);
      else
        m_cpu.setReg16(CX, static_cast<uint16_t>(count));

      if (opcode == 0xE2)
        jump = (count != 0); // LOOP
      else if (opcode == 0xE1)
        jump = (count != 0 && (m_cpu.getEFLAGS() & 0x0040)); // LOOPE
      else if (opcode == 0xE0)
        jump = (count != 0 && !(m_cpu.getEFLAGS() & 0x0040)); // LOOPNE
    }
    if (jump)
      m_cpu.setEIP(m_cpu.getEIP() + disp);
    break;
  }

  case 0xA0:   // MOV AL, [imm]
  case 0xA1:   // MOV AX/EAX, [imm16/32]
  case 0xA2:   // MOV [imm], AL
  case 0xA3: { // MOV [imm], AX/EAX
    uint8_t moffsSeg = (m_segmentOverride != 0xFF) ? m_segmentOverride : DS;
    uint32_t moffsAddr =
        m_segBase[moffsSeg] + (m_hasPrefix67 ? fetch32() : fetch16());
    if (opcode == 0xA0)
      m_cpu.setReg8(AL, m_memory.read8(moffsAddr));
    else if (opcode == 0xA1) {
      if (m_hasPrefix66)
        m_cpu.setReg32(EAX, m_memory.read32(moffsAddr));
      else
        m_cpu.setReg16(AX, m_memory.read16(moffsAddr));
    } else if (opcode == 0xA2)
      m_memory.write8(moffsAddr, m_cpu.getReg8(AL));
    else {
      if (m_hasPrefix66)
        m_memory.write32(moffsAddr, m_cpu.getReg32(EAX));
      else
        m_memory.write16(moffsAddr, m_cpu.getReg16(AX));
    }
    break;
  }

  case 0xD7: { // XLAT
    uint8_t xlatSeg = (m_segmentOverride != 0xFF) ? m_segmentOverride : DS;
    uint32_t addr = m_segBase[xlatSeg] + m_cpu.getReg16(BX) + m_cpu.getReg8(AL);
    m_cpu.setReg8(AL, m_memory.read8(addr));
    break;
  }
  case 0xD8:
  case 0xD9:
  case 0xDA:
  case 0xDB:
  case 0xDC:
  case 0xDD:
  case 0xDE:
  case 0xDF: { // FPU op stubs
    ModRM modrm = decodeModRM(fetch8());
    if (modrm.mod != 3) {
      if (m_hasPrefix67)
        getEffectiveAddress32(modrm);
      else
        getEffectiveAddress16(modrm);
    }
    break;
  }
  case 0xA4: { // MOVSB
    LOG_CPU("MOVSB");
    uint8_t srcSeg = (m_segmentOverride != 0xFF) ? m_segmentOverride : DS;
    uint32_t srcOff = m_hasPrefix67 ? m_cpu.getReg32(ESI) : m_cpu.getReg16(SI);
    uint32_t dstOff = m_hasPrefix67 ? m_cpu.getReg32(EDI) : m_cpu.getReg16(DI);

    m_memory.write8(m_segBase[ES] + dstOff,
                    m_memory.read8(m_segBase[srcSeg] + srcOff));

    int32_t diff = (m_cpu.getEFLAGS() & 0x0400) ? -1 : 1;
    if (m_hasPrefix67) {
      m_cpu.setReg32(EDI, dstOff + diff);
      m_cpu.setReg32(ESI, srcOff + diff);
    } else {
      m_cpu.setReg16(DI, static_cast<uint16_t>(dstOff + diff));
      m_cpu.setReg16(SI, static_cast<uint16_t>(srcOff + diff));
    }
    break;
  }
  case 0xA5: { // MOVSW/MOVSD
    LOG_CPU("MOVSW/D");
    uint8_t srcSeg = (m_segmentOverride != 0xFF) ? m_segmentOverride : DS;
    uint32_t srcOff = m_hasPrefix67 ? m_cpu.getReg32(ESI) : m_cpu.getReg16(SI);
    uint32_t dstOff = m_hasPrefix67 ? m_cpu.getReg32(EDI) : m_cpu.getReg16(DI);

    if (m_hasPrefix66) {
      m_memory.write32(m_segBase[ES] + dstOff,
                       m_memory.read32(m_segBase[srcSeg] + srcOff));
      int32_t diff = (m_cpu.getEFLAGS() & 0x0400) ? -4 : 4;
      if (m_hasPrefix67) {
        m_cpu.setReg32(EDI, dstOff + diff);
        m_cpu.setReg32(ESI, srcOff + diff);
      } else {
        m_cpu.setReg16(DI, static_cast<uint16_t>(dstOff + diff));
        m_cpu.setReg16(SI, static_cast<uint16_t>(srcOff + diff));
      }
    } else {
      m_memory.write16(m_segBase[ES] + dstOff,
                       m_memory.read16(m_segBase[srcSeg] + srcOff));
      int32_t diff = (m_cpu.getEFLAGS() & 0x0400) ? -2 : 2;
      if (m_hasPrefix67) {
        m_cpu.setReg32(EDI, dstOff + diff);
        m_cpu.setReg32(ESI, srcOff + diff);
      } else {
        m_cpu.setReg16(DI, static_cast<uint16_t>(dstOff + diff));
        m_cpu.setReg16(SI, static_cast<uint16_t>(srcOff + diff));
      }
    }
    break;
  }
  case 0xA6: { // CMPSB
    LOG_CPU("CMPSB");
    uint8_t srcSegC8 = (m_segmentOverride != 0xFF) ? m_segmentOverride : DS;
    uint8_t val1 = m_memory.read8(
        m_segBase[srcSegC8] +
        (m_hasPrefix67 ? m_cpu.getReg32(ESI) : m_cpu.getReg16(SI)));
    uint8_t val2 =
        m_memory.read8(m_segBase[ES] + (m_hasPrefix67 ? m_cpu.getReg32(EDI)
                                                      : m_cpu.getReg16(DI)));
    aluOp(7, val1, val2, 8); // CMP
    int32_t diff = (m_cpu.getEFLAGS() & 0x0400) ? -1 : 1;
    if (m_hasPrefix67) {
      m_cpu.setReg32(ESI, m_cpu.getReg32(ESI) + diff);
      m_cpu.setReg32(EDI, m_cpu.getReg32(EDI) + diff);
    } else {
      m_cpu.setReg16(SI, static_cast<uint16_t>(m_cpu.getReg16(SI) + diff));
      m_cpu.setReg16(DI, static_cast<uint16_t>(m_cpu.getReg16(DI) + diff));
    }
    break;
  }
  case 0xA7: { // CMPSW/CMPSD
    uint8_t srcSegCW = (m_segmentOverride != 0xFF) ? m_segmentOverride : DS;
    if (m_hasPrefix66) {
      uint32_t val1 = m_memory.read32(
          m_segBase[srcSegCW] +
          (m_hasPrefix67 ? m_cpu.getReg32(ESI) : m_cpu.getReg16(SI)));
      uint32_t val2 =
          m_memory.read32(m_segBase[ES] + (m_hasPrefix67 ? m_cpu.getReg32(EDI)
                                                         : m_cpu.getReg16(DI)));
      aluOp(7, val1, val2, 32);
      int32_t diff = (m_cpu.getEFLAGS() & 0x0400) ? -4 : 4;
      if (m_hasPrefix67) {
        m_cpu.setReg32(ESI, m_cpu.getReg32(ESI) + diff);
        m_cpu.setReg32(EDI, m_cpu.getReg32(EDI) + diff);
      } else {
        m_cpu.setReg16(SI, static_cast<uint16_t>(m_cpu.getReg16(SI) + diff));
        m_cpu.setReg16(DI, static_cast<uint16_t>(m_cpu.getReg16(DI) + diff));
      }
    } else {
      uint16_t val1 = m_memory.read16(
          m_segBase[srcSegCW] +
          (m_hasPrefix67 ? m_cpu.getReg32(ESI) : m_cpu.getReg16(SI)));
      uint16_t val2 =
          m_memory.read16(m_segBase[ES] + (m_hasPrefix67 ? m_cpu.getReg32(EDI)
                                                         : m_cpu.getReg16(DI)));
      aluOp(7, val1, val2, 16);
      int32_t diff = (m_cpu.getEFLAGS() & 0x0400) ? -2 : 2;
      if (m_hasPrefix67) {
        m_cpu.setReg32(ESI, m_cpu.getReg32(ESI) + diff);
        m_cpu.setReg32(EDI, m_cpu.getReg32(EDI) + diff);
      } else {
        m_cpu.setReg16(SI, static_cast<uint16_t>(m_cpu.getReg16(SI) + diff));
        m_cpu.setReg16(DI, static_cast<uint16_t>(m_cpu.getReg16(DI) + diff));
      }
    }
    break;
  }
  case 0xAA: { // STOSB
    LOG_CPU("STOSB");
    uint32_t dstOff = m_hasPrefix67 ? m_cpu.getReg32(EDI) : m_cpu.getReg16(DI);
    m_memory.write8(m_segBase[ES] + dstOff, m_cpu.getReg8(AL));
    int32_t diff = (m_cpu.getEFLAGS() & 0x0400) ? -1 : 1;
    if (m_hasPrefix67)
      m_cpu.setReg32(EDI, dstOff + diff);
    else
      m_cpu.setReg16(DI, static_cast<uint16_t>(dstOff + diff));
    break;
  }
  case 0xAB: { // STOSW/STOSD
    LOG_CPU("STOSW/D");
    uint32_t dstOff = m_hasPrefix67 ? m_cpu.getReg32(EDI) : m_cpu.getReg16(DI);
    if (m_hasPrefix66) {
      m_memory.write32(m_segBase[ES] + dstOff, m_cpu.getReg32(EAX));
      int32_t diff = (m_cpu.getEFLAGS() & 0x0400) ? -4 : 4;
      if (m_hasPrefix67)
        m_cpu.setReg32(EDI, dstOff + diff);
      else
        m_cpu.setReg16(DI, static_cast<uint16_t>(dstOff + diff));
    } else {
      m_memory.write16(m_segBase[ES] + dstOff, m_cpu.getReg16(AX));
      int32_t diff = (m_cpu.getEFLAGS() & 0x0400) ? -2 : 2;
      if (m_hasPrefix67)
        m_cpu.setReg32(EDI, dstOff + diff);
      else
        m_cpu.setReg16(DI, static_cast<uint16_t>(dstOff + diff));
    }
    break;
  }
  case 0xAC: { // LODSB
    LOG_CPU("LODSB");
    uint8_t srcSeg = (m_segmentOverride != 0xFF) ? m_segmentOverride : DS;
    uint32_t srcOff = m_hasPrefix67 ? m_cpu.getReg32(ESI) : m_cpu.getReg16(SI);
    m_cpu.setReg8(AL, m_memory.read8(m_segBase[srcSeg] + srcOff));
    int32_t diff = (m_cpu.getEFLAGS() & 0x0400) ? -1 : 1;
    if (m_hasPrefix67)
      m_cpu.setReg32(ESI, srcOff + diff);
    else
      m_cpu.setReg16(SI, static_cast<uint16_t>(srcOff + diff));
    break;
  }
  case 0xAD: { // LODSW/LODSD
    LOG_CPU("LODSW/D");
    uint8_t srcSeg = (m_segmentOverride != 0xFF) ? m_segmentOverride : DS;
    uint32_t srcOff = m_hasPrefix67 ? m_cpu.getReg32(ESI) : m_cpu.getReg16(SI);
    if (m_hasPrefix66) {
      m_cpu.setReg32(EAX, m_memory.read32(m_segBase[srcSeg] + srcOff));
      int32_t diff = (m_cpu.getEFLAGS() & 0x0400) ? -4 : 4;
      if (m_hasPrefix67)
        m_cpu.setReg32(ESI, srcOff + diff);
      else
        m_cpu.setReg16(SI, static_cast<uint16_t>(srcOff + diff));
    } else {
      m_cpu.setReg16(AX, m_memory.read16(m_segBase[srcSeg] + srcOff));
      int32_t diff = (m_cpu.getEFLAGS() & 0x0400) ? -2 : 2;
      if (m_hasPrefix67)
        m_cpu.setReg32(ESI, srcOff + diff);
      else
        m_cpu.setReg16(SI, static_cast<uint16_t>(srcOff + diff));
    }
    break;
  }
  case 0xAE: { // SCASB
    uint8_t val1 = m_cpu.getReg8(AL);
    uint8_t val2 =
        m_memory.read8(m_segBase[ES] + (m_hasPrefix67 ? m_cpu.getReg32(EDI)
                                                      : m_cpu.getReg16(DI)));
    aluOp(7, val1, val2, 8); // CMP
    int32_t diff = (m_cpu.getEFLAGS() & 0x0400) ? -1 : 1;
    if (m_hasPrefix67)
      m_cpu.setReg32(EDI, m_cpu.getReg32(EDI) + diff);
    else
      m_cpu.setReg16(DI, static_cast<uint16_t>(m_cpu.getReg16(DI) + diff));
    break;
  }
  case 0xAF: { // SCASW/SCASD
    if (m_hasPrefix66) {
      uint32_t val1 = m_cpu.getReg32(EAX);
      uint32_t val2 =
          m_memory.read32(m_segBase[ES] + (m_hasPrefix67 ? m_cpu.getReg32(EDI)
                                                         : m_cpu.getReg16(DI)));
      aluOp(7, val1, val2, 32);
      int32_t diff = (m_cpu.getEFLAGS() & 0x0400) ? -4 : 4;
      if (m_hasPrefix67)
        m_cpu.setReg32(EDI, m_cpu.getReg32(EDI) + diff);
      else
        m_cpu.setReg16(DI, static_cast<uint16_t>(m_cpu.getReg16(DI) + diff));
    } else {
      uint16_t val1 = m_cpu.getReg16(AX);
      uint16_t val2 =
          m_memory.read16(m_segBase[ES] + (m_hasPrefix67 ? m_cpu.getReg32(EDI)
                                                         : m_cpu.getReg16(DI)));
      aluOp(7, val1, val2, 16);
      int32_t diff = (m_cpu.getEFLAGS() & 0x0400) ? -2 : 2;
      if (m_hasPrefix67)
        m_cpu.setReg32(EDI, m_cpu.getReg32(EDI) + diff);
      else
        m_cpu.setReg16(DI, static_cast<uint16_t>(m_cpu.getReg16(DI) + diff));
    }
    break;
  }

  // Group 2 (Rotates/Shifts)
  case 0xC0:
  case 0xC1:
  case 0xD0:
  case 0xD1:
  case 0xD2:
  case 0xD3: {
    ModRM modrm = decodeModRM(fetch8());
    // For C0/C1: displacement bytes (if any) come before the immediate count
    // in the instruction stream.  Resolve the EA now so fetch8() below reads
    // the immediate, not the displacement.
    if ((opcode == 0xC0 || opcode == 0xC1) && modrm.mod != 3) {
      if (m_hasPrefix67)
        getEffectiveAddress32(modrm);
      else
        getEffectiveAddress16(modrm);
    }
    uint8_t count =
        (opcode == 0xC0 || opcode == 0xC1)
            ? fetch8()
            : ((opcode == 0xD0 || opcode == 0xD1) ? 1 : m_cpu.getReg8(CL));

    auto shiftRotate = [&](uint32_t val, uint8_t c, int size) -> uint32_t {
      c &= 0x1F;
      if (c == 0)
        return val;
      uint32_t mask = (size == 8)    ? 0xFFu
                      : (size == 16) ? 0xFFFFu
                                     : 0xFFFFFFFFu;
      val &= mask;
      uint32_t flags = m_cpu.getEFLAGS();
      bool cf = flags & FLAG_CARRY;

      switch (modrm.reg) {
      case 0: { // ROL
        uint8_t ec = c % size;
        if (ec)
          val = ((val << ec) | (val >> (size - ec))) & mask;
        cf = val & 1;
        flags = (flags & ~FLAG_CARRY) | (cf ? FLAG_CARRY : 0);
        if (c == 1) {
          bool msb = (val >> (size - 1)) & 1;
          flags = (flags & ~FLAG_OVERFLOW) | ((msb ^ cf) ? FLAG_OVERFLOW : 0);
        }
        m_cpu.setEFLAGS(flags);
        return val;
      }
      case 1: { // ROR
        uint8_t ec = c % size;
        if (ec)
          val = ((val >> ec) | (val << (size - ec))) & mask;
        cf = (val >> (size - 1)) & 1;
        flags = (flags & ~FLAG_CARRY) | (cf ? FLAG_CARRY : 0);
        if (c == 1) {
          bool bit2 = (val >> (size - 2)) & 1;
          flags = (flags & ~FLAG_OVERFLOW) | ((cf ^ bit2) ? FLAG_OVERFLOW : 0);
        }
        m_cpu.setEFLAGS(flags);
        return val;
      }
      case 2: { // RCL
        for (uint8_t i = 0; i < c; ++i) {
          bool msb = (val >> (size - 1)) & 1;
          val = ((val << 1) | (cf ? 1u : 0u)) & mask;
          cf = msb;
        }
        flags = (flags & ~FLAG_CARRY) | (cf ? FLAG_CARRY : 0);
        if (c == 1) {
          bool msb = (val >> (size - 1)) & 1;
          flags = (flags & ~FLAG_OVERFLOW) | ((msb ^ cf) ? FLAG_OVERFLOW : 0);
        }
        m_cpu.setEFLAGS(flags);
        return val;
      }
      case 3: { // RCR
        for (uint8_t i = 0; i < c; ++i) {
          bool lsb = val & 1;
          val = ((val >> 1) | (cf ? (1u << (size - 1)) : 0u)) & mask;
          cf = lsb;
        }
        flags = (flags & ~FLAG_CARRY) | (cf ? FLAG_CARRY : 0);
        if (c == 1) {
          bool msb = (val >> (size - 1)) & 1;
          bool bit2 = (val >> (size - 2)) & 1;
          flags = (flags & ~FLAG_OVERFLOW) | ((msb ^ bit2) ? FLAG_OVERFLOW : 0);
        }
        m_cpu.setEFLAGS(flags);
        return val;
      }
      case 4:
      case 6: { // SHL/SAL
        cf = (val >> (size - c)) & 1;
        val = (val << c) & mask;
        break;
      }
      case 5: { // SHR
        bool oldMsb = (val >> (size - 1)) & 1;
        cf = (val >> (c - 1)) & 1;
        val = (val >> c) & mask;
        flags = (flags & ~FLAG_OVERFLOW);
        if (c == 1)
          flags |= (oldMsb ? FLAG_OVERFLOW : 0);
        break;
      }
      case 7: { // SAR
        int32_t sval = (size == 8)    ? static_cast<int8_t>(val)
                       : (size == 16) ? static_cast<int16_t>(val)
                                      : static_cast<int32_t>(val);
        cf = (sval >> (c - 1)) & 1;
        val = static_cast<uint32_t>(sval >> c) & mask;
        flags &= ~FLAG_OVERFLOW; // OF always 0 for SAR
        break;
      }
      default:
        return val;
      }
      // Common flag update for SHL/SHR/SAR (cases 4-7)
      flags = (flags & ~(FLAG_CARRY | FLAG_ZERO | FLAG_SIGN | FLAG_PARITY));
      if (cf)
        flags |= FLAG_CARRY;
      if (val == 0)
        flags |= FLAG_ZERO;
      if (val & (1u << (size - 1)))
        flags |= FLAG_SIGN;
      {
        uint8_t p = val & 0xFF;
        p ^= p >> 4;
        p ^= p >> 2;
        p ^= p >> 1;
        if (!(p & 1))
          flags |= FLAG_PARITY;
      }
      if (c == 1 && modrm.reg == 4) {
        bool msb = (val >> (size - 1)) & 1;
        flags = (flags & ~FLAG_OVERFLOW) | ((msb ^ cf) ? FLAG_OVERFLOW : 0);
      }
      m_cpu.setEFLAGS(flags);
      return val;
    };

    if (opcode == 0xC0 || opcode == 0xD0 || opcode == 0xD2) {
      writeModRM8(modrm, static_cast<uint8_t>(
                             shiftRotate(readModRM8(modrm), count, 8)));
    } else if (m_hasPrefix66) {
      writeModRM32(modrm, shiftRotate(readModRM32(modrm), count, 32));
    } else {
      writeModRM16(modrm, static_cast<uint16_t>(
                              shiftRotate(readModRM16(modrm), count, 16)));
    }
    break;
  }

  case 0x86: { // XCHG r/m8, r8
    ModRM modrm = decodeModRM(fetch8());
    uint8_t rmVal = readModRM8(modrm);
    writeModRM8(modrm, m_cpu.getReg8(modrm.reg));
    m_cpu.setReg8(modrm.reg, rmVal);
    break;
  }

  // XCHG
  case 0x87: {
    ModRM modrm = decodeModRM(fetch8());
    if (m_hasPrefix66) {
      uint32_t rmVal = readModRM32(modrm);
      writeModRM32(modrm, m_cpu.getReg32(modrm.reg));
      m_cpu.setReg32(modrm.reg, rmVal);
    } else {
      uint16_t rmVal = readModRM16(modrm);
      writeModRM16(modrm, m_cpu.getReg16(modrm.reg));
      m_cpu.setReg16(modrm.reg, rmVal);
    }
    break;
  }
  case 0x91:
  case 0x92:
  case 0x93:
  case 0x94:
  case 0x95:
  case 0x96:
  case 0x97: {
    uint8_t reg = opcode & 0x07;
    if (m_hasPrefix66) {
      uint32_t val = m_cpu.getReg32(EAX);
      m_cpu.setReg32(EAX, m_cpu.getReg32(reg));
      m_cpu.setReg32(reg, val);
    } else {
      uint16_t val = m_cpu.getReg16(AX);
      m_cpu.setReg16(AX, m_cpu.getReg16(reg));
      m_cpu.setReg16(reg, val);
    }
    break;
  }

  case 0xF5:
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() ^ FLAG_CARRY);
    break; // CMC
  case 0xF8:
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~0x0001);
    break; // CLC
  case 0xF9:
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | 0x0001);
    break; // STC
  case 0xFA:
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~0x0200);
    break; // CLI
  case 0xFB:
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | 0x0200);
    break; // STI
  case 0xFC:
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~0x0400);
    break; // CLD
  case 0xFD:
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | 0x0400);
    break; // STD

  // Group 4/5
  case 0xFE:
  case 0xFF: {
    ModRM modrm = decodeModRM(fetch8());
    if (opcode == 0xFE) { // Group 4
      if (modrm.reg == 0)
        writeModRM8(modrm, incOp(readModRM8(modrm), 8)); // INC rm8
      else if (modrm.reg == 1)
        writeModRM8(modrm, decOp(readModRM8(modrm), 8)); // DEC rm8
      break;
    }
    // Group 5
    if (m_hasPrefix66) {
      uint32_t val = (modrm.reg < 2 || modrm.reg == 6) ? readModRM32(modrm) : 0;
      switch (modrm.reg) {
      case 0:
        writeModRM32(modrm, incOp(val, 32));
        break;
      case 1:
        writeModRM32(modrm, decOp(val, 32));
        break;
      case 2: {
        uint32_t target = readModRM32(modrm);
        m_cpu.push32(m_cpu.getEIP());
        m_cpu.setEIP(target);
        break;
      }
      case 3: { // CALL FAR
        uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                      : getEffectiveAddress16(modrm);
        uint32_t jumpIP = m_memory.read32(addr);
        uint16_t jumpCS = m_memory.read16(addr + 4);
        m_cpu.push32(m_cpu.getSegReg(CS));
        m_cpu.push32(m_cpu.getEIP());
        loadSegment(CS, jumpCS);
        m_cpu.setEIP(jumpIP);
        break;
      }
      case 4:
        m_cpu.setEIP(readModRM32(modrm));
        break;
      case 5: { // JMP FAR
        uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                      : getEffectiveAddress16(modrm);
        uint32_t jumpIP = m_memory.read32(addr);
        uint16_t jumpCS = m_memory.read16(addr + 4);
        loadSegment(CS, jumpCS);
        m_cpu.setEIP(jumpIP);
        break;
      }
      case 6:
        m_cpu.push32(val);
        break;
      default:
        triggerInterrupt(6);
        break;
      }
    } else {
      uint16_t val = (modrm.reg < 2 || modrm.reg == 6) ? readModRM16(modrm) : 0;
      switch (modrm.reg) {
      case 0:
        writeModRM16(modrm, incOp(val, 16));
        break;
      case 1:
        writeModRM16(modrm, decOp(val, 16));
        break;
      case 2: {
        uint16_t target = readModRM16(modrm);
        m_cpu.push16(static_cast<uint16_t>(m_cpu.getEIP()));
        m_cpu.setEIP(target);
        break;
      }
      case 3: { // CALL FAR
        uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                      : getEffectiveAddress16(modrm);
        uint16_t jumpIP = m_memory.read16(addr);
        uint16_t jumpCS = m_memory.read16(addr + 2);
        m_cpu.push16(m_cpu.getSegReg(CS));
        m_cpu.push16(static_cast<uint16_t>(m_cpu.getEIP()));
        loadSegment(CS, jumpCS);
        m_cpu.setEIP(jumpIP);
        break;
      }
      case 4:
        m_cpu.setEIP(readModRM16(modrm));
        break;
      case 5: { // JMP FAR
        uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                      : getEffectiveAddress16(modrm);
        uint16_t jumpIP = m_memory.read16(addr);
        uint16_t jumpCS = m_memory.read16(addr + 2);
        loadSegment(CS, jumpCS);
        m_cpu.setEIP(jumpIP);
        break;
      }
      case 6:
        m_cpu.push16(val);
        break;
      default:
        triggerInterrupt(6);
        break;
      }
    }
    break;
  }

  // Return / Branch
  case 0xC3: { // RET Near
    uint32_t eip;
    if (m_hasPrefix66) {
      eip = m_memory.read32(m_segBase[SS] + m_cpu.getReg32(ESP));
      m_cpu.setReg32(ESP, m_cpu.getReg32(ESP) + 4);
    } else {
      eip = m_memory.read16(m_segBase[SS] + m_cpu.getReg16(SP));
      m_cpu.setReg16(SP, m_cpu.getReg16(SP) + 2);
    }
    m_cpu.setEIP(eip);
    break;
  }
  case 0xC2: { // RET imm16 Near
    uint16_t imm = fetch16();
    uint32_t eip;
    if (m_hasPrefix66) {
      eip = m_memory.read32(m_segBase[SS] + m_cpu.getReg32(ESP));
      m_cpu.setReg32(ESP, m_cpu.getReg32(ESP) + 4 + imm);
    } else {
      eip = m_memory.read16(m_segBase[SS] + m_cpu.getReg16(SP));
      m_cpu.setReg16(SP, m_cpu.getReg16(SP) + 2 + imm);
    }
    m_cpu.setEIP(eip);
    break;
  }
  case 0xCB: { // RETF Far
    uint32_t eip;
    uint16_t cs;
    if (m_hasPrefix66) {
      eip = m_memory.read32(m_segBase[SS] + m_cpu.getReg32(ESP));
      cs = m_memory.read32(m_segBase[SS] + m_cpu.getReg32(ESP) + 4) & 0xFFFF;
      m_cpu.setReg32(ESP, m_cpu.getReg32(ESP) + 8);
    } else {
      eip = m_memory.read16(m_segBase[SS] + m_cpu.getReg16(SP));
      cs = m_memory.read16(m_segBase[SS] + m_cpu.getReg16(SP) + 2);
      m_cpu.setReg16(SP, m_cpu.getReg16(SP) + 4);
    }
    loadSegment(CS, cs);
    m_cpu.setEIP(eip);
    break;
  }
  case 0xCA: { // RETF imm16 Far
    uint16_t imm = fetch16();
    uint32_t eip;
    uint16_t cs;
    if (m_hasPrefix66) {
      eip = m_memory.read32(m_segBase[SS] + m_cpu.getReg32(ESP));
      cs = m_memory.read32(m_segBase[SS] + m_cpu.getReg32(ESP) + 4) & 0xFFFF;
      m_cpu.setReg32(ESP, m_cpu.getReg32(ESP) + 8 + imm);
    } else {
      eip = m_memory.read16(m_segBase[SS] + m_cpu.getReg16(SP));
      cs = m_memory.read16(m_segBase[SS] + m_cpu.getReg16(SP) + 2);
      m_cpu.setReg16(SP, m_cpu.getReg16(SP) + 4 + imm);
    }
    loadSegment(CS, cs);
    m_cpu.setEIP(eip);
    break;
  }
  case 0xE8: { // CALL rel
    if (m_hasPrefix66) {
      int32_t rel = static_cast<int32_t>(fetch32());
      m_cpu.push32(m_cpu.getEIP());
      m_cpu.setEIP(m_cpu.getEIP() + rel);
    } else {
      int16_t rel = static_cast<int16_t>(fetch16());
      uint16_t retAddr = static_cast<uint16_t>(m_cpu.getEIP());
      uint16_t target = static_cast<uint16_t>(retAddr + rel);
      m_cpu.push16(retAddr);
      m_cpu.setEIP(target);
    }
    break;
  }
  case 0x9A: { // CALL far imm16:imm16/32
    if (m_hasPrefix66) {
      uint32_t eip = fetch32();
      uint16_t cs = fetch16();
      m_cpu.push32(
          m_cpu.getSegReg(CS)); // Realistically pushes 16-bit CS padded to 32
      m_cpu.push32(m_cpu.getEIP());
      loadSegment(CS, cs);
      m_cpu.setEIP(eip);
    } else {
      uint16_t ip = fetch16();
      uint16_t cs = fetch16();
      m_cpu.push16(m_cpu.getSegReg(CS));
      m_cpu.push16(static_cast<uint16_t>(m_cpu.getEIP()));
      loadSegment(CS, cs);
      m_cpu.setEIP(ip);
    }
    break;
  }
  case 0xE9: { // JMP rel16/32
    if (m_hasPrefix66) {
      int32_t rel = static_cast<int32_t>(fetch32());
      m_cpu.setEIP(m_cpu.getEIP() + rel);
    } else {
      int16_t rel = static_cast<int16_t>(fetch16());
      m_cpu.setEIP(static_cast<uint16_t>(m_cpu.getEIP() + rel));
    }
    break;
  }
  case 0xEA: { // JMP far imm16:imm16/32
    if (m_hasPrefix66) {
      uint32_t eip = fetch32();
      uint16_t cs = fetch16();
      loadSegment(CS, cs);
      m_cpu.setEIP(eip);
    } else {
      uint16_t ip = fetch16();
      uint16_t cs = fetch16();
      loadSegment(CS, cs);
      m_cpu.setEIP(ip);
    }
    break;
  }
  case 0xEB: { // JMP rel8
    int8_t disp = static_cast<int8_t>(fetch8());
    m_cpu.setEIP(m_cpu.getEIP() + disp);
    break;
  }

  // Immediate to Reg Moves
  case 0xB0:
  case 0xB1:
  case 0xB2:
  case 0xB3:
  case 0xB4:
  case 0xB5:
  case 0xB6:
  case 0xB7:
    m_cpu.setReg8(opcode & 0x07, fetch8());
    break;
  case 0xB8:
  case 0xB9:
  case 0xBA:
  case 0xBB:
  case 0xBC:
  case 0xBD:
  case 0xBE:
  case 0xBF:
    if (m_hasPrefix66)
      m_cpu.setReg32(opcode & 0x07, fetch32());
    else
      m_cpu.setReg16(opcode & 0x07, fetch16());
    break;

  // I/O
  case 0xE4:
    m_cpu.setReg8(AL, m_iobus.read8(fetch8()));
    break;   // IN AL, imm8
  case 0xE5: // IN AX/EAX, imm8
    if (m_hasPrefix66)
      m_cpu.setReg32(EAX, m_iobus.read32(fetch8()));
    else
      m_cpu.setReg16(AX, m_iobus.read16(fetch8()));
    break;
  case 0xE6:
    m_iobus.write8(fetch8(), m_cpu.getReg8(AL));
    break;   // OUT imm8, AL
  case 0xE7: // OUT imm8, AX/EAX
    if (m_hasPrefix66)
      m_iobus.write32(fetch8(), m_cpu.getReg32(EAX));
    else
      m_iobus.write16(fetch8(), m_cpu.getReg16(AX));
    break;
  case 0xEC:
    m_cpu.setReg8(AL, m_iobus.read8(m_cpu.getReg16(DX)));
    break;   // IN AL, DX
  case 0xED: // IN AX/EAX, DX
    if (m_hasPrefix66)
      m_cpu.setReg32(EAX, m_iobus.read32(m_cpu.getReg16(DX)));
    else
      m_cpu.setReg16(AX, m_iobus.read16(m_cpu.getReg16(DX)));
    break;
  case 0xEE:
    m_iobus.write8(m_cpu.getReg16(DX), m_cpu.getReg8(AL));
    break;   // OUT DX, AL
  case 0xEF: // OUT DX, AX/EAX
    if (m_hasPrefix66)
      m_iobus.write32(m_cpu.getReg16(DX), m_cpu.getReg32(EAX));
    else
      m_iobus.write16(m_cpu.getReg16(DX), m_cpu.getReg16(AX));
    break;

  case 0xD6: { // SALC – Set AL from Carry (undocumented Intel; used by Borland
               // RTL)
    m_cpu.setReg8(AL, (m_cpu.getEFLAGS() & FLAG_CARRY) ? 0xFF : 0x00);
    break;
  }

  case 0xF1: { // INT1 / ICEBP – treat as debug breakpoint (no-op)
    static int f1_count = 0;
    if (f1_count == 0) {
      LOG_ERROR("FIRST INT1/ICEBP at ", std::hex, m_cpu.getSegReg(CS), ":",
                m_cpu.getEIP() - 1, " csBase=", m_segBase[CS],
                " cpuCsBase=", m_cpu.getSegBase(CS),
                " cr0=", m_cpu.getCR(0), " d32=", m_cpu.is32BitCode());
    }
    f1_count++;
    break;
  }
  case 0xF4:
    LOG_INFO("HLT encountered at ", std::hex, m_cpu.getSegReg(CS), ":",
             m_cpu.getEIP() - 1);
    break;

  default:
    LOG_WARN("Unknown opcode 0x", std::hex, static_cast<int>(opcode),
             " at CS:EIP ", m_cpu.getSegReg(CS), ":", m_cpu.getEIP() - 1);
    triggerInterrupt(6);
    break;
  }
}

void InstructionDecoder::executeOpcode0F(uint8_t opcode) {
  LOG_CPU("Opcode0F: 0x", std::hex, (int)opcode, " at CS:EIP ",
          m_cpu.getSegReg(CS), ":", m_cpu.getEIP() - 1);
  switch (opcode) {
  case 0x00: { // Group 6
    ModRM modrm = decodeModRM(fetch8());
    switch (modrm.reg) {
    case 0: // SLDT
      writeModRM16(modrm, m_cpu.getLDTRSelector());
      break;
    case 1: // STR
      writeModRM16(modrm, m_cpu.getTRSelector());
      break;
    case 2: { // LLDT
      uint16_t selector = readModRM16(modrm);
      LOG_DEBUG("[CPU] LLDT selector: 0x", std::hex, selector);
      m_cpu.setLDTRSelector(selector);
      if (selector == 0) {
        m_cpu.setLDTR({0, 0});
      } else {
        uint32_t gdtBase = m_cpu.getGDTR().base;
        uint32_t entryAddr = gdtBase + (selector & ~7);
        uint32_t low = m_memory.read32(entryAddr);
        uint32_t high = m_memory.read32(entryAddr + 4);
        Descriptor desc = decodeDescriptor(low, high);
        m_cpu.setLDTR({static_cast<uint16_t>(desc.limit & 0xFFFF), desc.base});
        LOG_DEBUG("  LDT base: 0x", std::hex, desc.base, " limit: 0x",
                  desc.limit);
      }
      break;
    }
    case 3: { // LTR
      uint16_t selector = readModRM16(modrm);
      LOG_DEBUG("[CPU] LTR selector: 0x", std::hex, selector);
      m_cpu.setTRSelector(selector);
      if (selector == 0) {
        m_cpu.setTR({0, 0});
      } else {
        uint32_t gdtBase = m_cpu.getGDTR().base;
        uint32_t entryAddr = gdtBase + (selector & ~7);
        uint32_t low = m_memory.read32(entryAddr);
        uint32_t high = m_memory.read32(entryAddr + 4);
        Descriptor desc = decodeDescriptor(low, high);
        m_cpu.setTR({static_cast<uint16_t>(desc.limit & 0xFFFF), desc.base});
        LOG_DEBUG("  Task Register base: 0x", std::hex, desc.base, " limit: 0x",
                  desc.limit);
      }
      break;
    }
    default:
      LOG_WARN("Group 6 (0x0F 0x00) sub-opcode ", (int)modrm.reg,
               " not implemented.");
      triggerInterrupt(6);
      break;
    }
    break;
  }
  case 0x01: { // Group 7
    ModRM modrm = decodeModRM(fetch8());
    switch (modrm.reg) {
    case 0: { // SGDT
      uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                    : getEffectiveAddress16(modrm);
      m_memory.write16(addr, m_cpu.getGDTR().limit);
      // SGDT/SIDT always store 24 bits in 16-bit mode, 32 bits in 32-bit mode
      // Actually, Intel says: "size of the operand is always 6 bytes"
      // "In 16-bit mode, only 24 bits are stored. In 32-bit mode, 32 bits."
      m_memory.write32(addr + 2, m_cpu.getGDTR().base &
                                     (m_hasPrefix66 ? 0xFFFFFFFF : 0xFFFFFF));
      break;
    }
    case 1: { // SIDT
      uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                    : getEffectiveAddress16(modrm);
      m_memory.write16(addr, m_cpu.getIDTR().limit);
      m_memory.write32(addr + 2, m_cpu.getIDTR().base &
                                     (m_hasPrefix66 ? 0xFFFFFFFF : 0xFFFFFF));
      break;
    }
    case 2: { // LGDT
      uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                    : getEffectiveAddress16(modrm);
      uint16_t limit = m_memory.read16(addr);
      uint32_t base = m_memory.read32(addr + 2);
      if (!m_hasPrefix66)
        base &= 0xFFFFFF;
      LOG_DEBUG("[CPU] LGDT base=", std::hex, base, " limit=", limit);
      m_cpu.setGDTR({limit, base});
      break;
    }
    case 3: { // LIDT
      uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                    : getEffectiveAddress16(modrm);
      uint16_t limit = m_memory.read16(addr);
      uint32_t base = m_memory.read32(addr + 2);
      if (!m_hasPrefix66)
        base &= 0xFFFFFF;
      LOG_DEBUG("[CPU] LIDT base=", std::hex, base, " limit=", limit);
      m_cpu.setIDTR({limit, base});
      break;
    }
    case 4: // SMSW
      writeModRM16(modrm, m_cpu.getCR(0) & 0xFFFF);
      break;
    case 6: { // LMSW
      uint16_t cur_msw = m_cpu.getCR(0) & 0xFFFF;
      uint16_t new_msw = readModRM16(modrm);
      // PE bit (bit 0) cannot be cleared by LMSW once set
      new_msw = (new_msw & ~1) | (cur_msw & 1) | (new_msw & 1);
      uint32_t newVal = (m_cpu.getCR(0) & 0xFFFF0000) | new_msw;
      LOG_DEBUG("[CPU] LMSW = ", std::hex, new_msw);
      m_cpu.setCR(0, newVal);
      break;
    }
    default:
      LOG_WARN("Group 7 (0x0F 0x01) sub-opcode ", (int)modrm.reg,
               " not implemented.");
      triggerInterrupt(6);
      break;
    }
    break;
  }
  case 0x02: { // LAR r, r/m
    ModRM modrm = decodeModRM(fetch8());
    uint16_t selector = readModRM16(modrm);
    bool isRealMode = !(m_cpu.getCR(0) & 1) || (m_cpu.getEFLAGS() & 0x00020000);
    if (isRealMode || selector == 0) {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_ZERO);
    } else {
      uint32_t tableBase =
          (selector & 0x04) ? m_cpu.getLDTR().base : m_cpu.getGDTR().base;
      uint16_t tableLimit =
          (selector & 0x04) ? m_cpu.getLDTR().limit : m_cpu.getGDTR().limit;
      uint16_t index = selector & ~7;
      if (index > tableLimit) {
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_ZERO);
      } else {
        uint32_t entryAddr = tableBase + index;
        uint32_t high = m_memory.read32(entryAddr + 4);
        uint8_t access = (high >> 8) & 0xFF;
        // Check descriptor type validity per Intel spec:
        // For non-system descriptors (S=1, bit 12 of access): always valid
        // For system descriptors (S=0): only LDT(2), TSS(1,3,9,11),
        //   call gate(4,12), task gate(5) are valid
        // Also check DPL >= max(CPL, RPL)
        bool sysFlag = (access & 0x10) != 0; // S bit
        uint8_t type4 = access & 0x0F;
        bool typeOk = sysFlag; // Non-system (code/data) always ok
        if (!sysFlag) {
          // System descriptor — only certain types valid for LAR
          typeOk = (type4 == 1 || type4 == 2 || type4 == 3 ||
                    type4 == 4 || type4 == 5 || type4 == 9 ||
                    type4 == 11 || type4 == 12);
        }
        if (access == 0 || !typeOk) {
          // Zero access = empty descriptor slot, or invalid system type
          m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_ZERO);
        } else {
          uint32_t ar = high & 0x00FFFF00;
          if (m_hasPrefix66)
            m_cpu.setReg32(modrm.reg, ar);
          else
            m_cpu.setReg16(modrm.reg, static_cast<uint16_t>(ar));
          m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_ZERO);
        }
      }
    }
    break;
  }
  case 0x03: { // LSL r, r/m
    ModRM modrm = decodeModRM(fetch8());
    uint16_t selector = readModRM16(modrm);
    bool isRealMode = !(m_cpu.getCR(0) & 1) || (m_cpu.getEFLAGS() & 0x00020000);
    if (isRealMode || selector == 0) {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_ZERO);
    } else {
      uint32_t tableBase =
          (selector & 0x04) ? m_cpu.getLDTR().base : m_cpu.getGDTR().base;
      uint16_t tableLimit =
          (selector & 0x04) ? m_cpu.getLDTR().limit : m_cpu.getGDTR().limit;
      uint16_t index = selector & ~7;
      if (index > tableLimit) {
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_ZERO);
      } else {
        uint32_t entryAddr = tableBase + index;
        uint32_t low = m_memory.read32(entryAddr);
        uint32_t high = m_memory.read32(entryAddr + 4);
        uint8_t access = (high >> 8) & 0xFF;
        bool sysFlag = (access & 0x10) != 0;
        uint8_t type4 = access & 0x0F;
        bool typeOk = sysFlag;
        if (!sysFlag) {
          typeOk = (type4 == 1 || type4 == 2 || type4 == 3 ||
                    type4 == 9 || type4 == 11);
        }
        if (access == 0 || !typeOk) {
          m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_ZERO);
        } else {
          uint32_t limit = (low & 0xFFFF) | (high & 0x000F0000);
          if (high & 0x00800000)
            limit = (limit << 12) | 0xFFF;
          if (m_hasPrefix66)
            m_cpu.setReg32(modrm.reg, limit);
          else
            m_cpu.setReg16(modrm.reg, static_cast<uint16_t>(limit));
          m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_ZERO);
        }
      }
    }
    break;
  }
  case 0x08: // INVD  – Invalidate Cache (no-op)
    break;
  case 0x09: // WBINVD – Write Back & Invalidate Cache (no-op)
    break;
  case 0x0B: // UD2  – intentional #UD
    triggerInterrupt(6);
    break;
  case 0x06: { // CLTS
    LOG_CPU("CLTS - stubbed (no task switching)");
    break;
  }
  case 0x10: { // MOVUPS / MOVSS xmm, xmm/m  – SSE stub, just consume ModRM
    ModRM modrm = decodeModRM(fetch8());
    (void)modrm;
    break;
  }
  case 0x11: { // MOVUPS / MOVSS xmm/m, xmm  – SSE stub
    ModRM modrm = decodeModRM(fetch8());
    (void)modrm;
    break;
  }
  case 0x20: { // MOV rd, CRn
    ModRM modrm = decodeModRM(fetch8());
    m_cpu.setReg32(modrm.rm, m_cpu.getCR(modrm.reg));
    break;
  }
  case 0x21: { // MOV rd, DRn
    ModRM modrm = decodeModRM(fetch8());
    m_cpu.setReg32(modrm.rm, m_cpu.getDR(modrm.reg));
    break;
  }
  case 0x22: { // MOV CRn, rd
    ModRM modrm = decodeModRM(fetch8());
    uint32_t val = m_cpu.getReg32(modrm.rm);
    LOG_DEBUG("[CPU] MOV CR", (int)modrm.reg, " = ", std::hex, val);
    m_cpu.setCR(modrm.reg, val);
    break;
  }
  case 0x23: { // MOV DRn, rd
    ModRM modrm = decodeModRM(fetch8());
    m_cpu.setDR(modrm.reg, m_cpu.getReg32(modrm.rm));
    break;
  }
  case 0x35: // SYSEXIT  – no-op in real mode
    break;
  case 0x77: // EMMS – Empty MMX state (no-op)
    break;
  case 0xEE: // FEMMS – AMD 3DNow! fast EMMS (no-op, no ModRM)
    break;
  case 0x80:
  case 0x81:
  case 0x82:
  case 0x83:
  case 0x84:
  case 0x85:
  case 0x86:
  case 0x87:
  case 0x88:
  case 0x89:
  case 0x8A:
  case 0x8B:
  case 0x8C:
  case 0x8D:
  case 0x8E:
  case 0x8F: { // Jcc rel16/32
    uint32_t rel = m_hasPrefix66 ? fetch32() : fetch16();
    if (checkCondition(opcode)) {
      if (m_hasPrefix66)
        m_cpu.setEIP(m_cpu.getEIP() + rel);
      else
        m_cpu.setEIP(static_cast<uint16_t>(m_cpu.getEIP() + rel));
    }
    break;
  }

  case 0x40:
  case 0x41:
  case 0x42:
  case 0x43:
  case 0x44:
  case 0x45:
  case 0x46:
  case 0x47:
  case 0x48:
  case 0x49:
  case 0x4A:
  case 0x4B:
  case 0x4C:
  case 0x4D:
  case 0x4E:
  case 0x4F: { // CMOVcc r16/32, r/m16/32
    ModRM modrm = decodeModRM(fetch8());
    if (checkCondition(opcode)) {
      if (m_hasPrefix66)
        m_cpu.setReg32(modrm.reg, readModRM32(modrm));
      else
        m_cpu.setReg16(modrm.reg, readModRM16(modrm));
    } else {
      // Condition false: still consume the operand but don't write
      if (m_hasPrefix66)
        readModRM32(modrm);
      else
        readModRM16(modrm);
    }
    break;
  }

  case 0x90:
  case 0x91:
  case 0x92:
  case 0x93:
  case 0x94:
  case 0x95:
  case 0x96:
  case 0x97:
  case 0x98:
  case 0x99:
  case 0x9A:
  case 0x9B:
  case 0x9C:
  case 0x9D:
  case 0x9E:
  case 0x9F: { // SETcc rm8
    ModRM modrm = decodeModRM(fetch8());
    writeModRM8(modrm, checkCondition(opcode) ? 1 : 0);
    break;
  }

  // PUSH/POP FS (0F A0/A1) and GS (0F A8/A9)
  case 0xA0: // PUSH FS
    if (m_hasPrefix66)
      m_cpu.push32(m_cpu.getSegReg(FS));
    else
      m_cpu.push16(m_cpu.getSegReg(FS));
    break;
  case 0xA1: // POP FS
    if (m_hasPrefix66)
      loadSegment(FS, m_cpu.pop32() & 0xFFFF);
    else
      loadSegment(FS, m_cpu.pop16());
    break;
  case 0xA8: // PUSH GS
    if (m_hasPrefix66)
      m_cpu.push32(m_cpu.getSegReg(GS));
    else
      m_cpu.push16(m_cpu.getSegReg(GS));
    break;
  case 0xA9: // POP GS
    if (m_hasPrefix66)
      loadSegment(GS, m_cpu.pop32() & 0xFFFF);
    else
      loadSegment(GS, m_cpu.pop16());
    break;

  case 0xA3: { // BT r/m, r
    ModRM modrm = decodeModRM(fetch8());
    uint16_t bit = m_cpu.getReg16(modrm.reg) & 0x0F;
    uint16_t val = readModRM16(modrm);
    if ((val >> bit) & 1)
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
    else
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_CARRY);
    break;
  }
  case 0xAB: { // BTS r/m, r
    ModRM modrm = decodeModRM(fetch8());
    uint16_t bit = m_cpu.getReg16(modrm.reg) & 0x0F;
    uint16_t val = readModRM16(modrm);
    if ((val >> bit) & 1)
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
    else
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_CARRY);
    writeModRM16(modrm, val | static_cast<uint16_t>(1u << bit));
    break;
  }
  case 0xB3: { // BTR r/m, r
    ModRM modrm = decodeModRM(fetch8());
    uint16_t bit = m_cpu.getReg16(modrm.reg) & 0x0F;
    uint16_t val = readModRM16(modrm);
    if ((val >> bit) & 1)
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
    else
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_CARRY);
    writeModRM16(modrm, val & ~static_cast<uint16_t>(1u << bit));
    break;
  }
  case 0xBB: { // BTC r/m, r
    ModRM modrm = decodeModRM(fetch8());
    uint16_t bit = m_cpu.getReg16(modrm.reg) & 0x0F;
    uint16_t val = readModRM16(modrm);
    if ((val >> bit) & 1)
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
    else
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_CARRY);
    writeModRM16(modrm, val ^ static_cast<uint16_t>(1u << bit));
    break;
  }
  case 0xBA: { // Group 8: BT/BTS/BTR/BTC r/m, imm8
    ModRM modrm = decodeModRM(fetch8());
    uint8_t imm = fetch8();
    uint16_t bit = imm & 0x0F;
    uint16_t val = readModRM16(modrm);
    bool oldBit = (val >> bit) & 1;
    if (oldBit)
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
    else
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_CARRY);
    switch (modrm.reg) {
    case 4:
      break; // BT  – read only
    case 5:
      writeModRM16(modrm, val | static_cast<uint16_t>(1u << bit));
      break; // BTS
    case 6:
      writeModRM16(modrm, val & ~static_cast<uint16_t>(1u << bit));
      break; // BTR
    case 7:
      writeModRM16(modrm, val ^ static_cast<uint16_t>(1u << bit));
      break; // BTC
    default:
      LOG_WARN("Group 8 (0x0F 0xBA) unknown sub-opcode ", (int)modrm.reg);
      break;
    }
    break;
  }

  case 0xA4: { // SHLD r/m, r, imm8
    ModRM modrm = decodeModRM(fetch8());
    uint8_t count = fetch8() & 0x1F;
    if (count) {
      uint32_t flags = m_cpu.getEFLAGS();
      if (m_hasPrefix66) {
        uint32_t dst = readModRM32(modrm);
        uint32_t src = m_cpu.getReg32(modrm.reg);
        uint64_t tmp = (static_cast<uint64_t>(dst) << 32) | src;
        uint64_t res64 = tmp << count;
        uint32_t result = static_cast<uint32_t>(res64 >> 32);
        writeModRM32(modrm, result);
        bool cf = (dst >> (32 - count)) & 1;
        flags &=
            ~(FLAG_CARRY | FLAG_ZERO | FLAG_SIGN | FLAG_PARITY | FLAG_OVERFLOW);
        if (cf)
          flags |= FLAG_CARRY;
        if (result == 0)
          flags |= FLAG_ZERO;
        if (result & 0x80000000u)
          flags |= FLAG_SIGN;
        {
          uint8_t p = result & 0xFF;
          p ^= p >> 4;
          p ^= p >> 2;
          p ^= p >> 1;
          if (!(p & 1))
            flags |= FLAG_PARITY;
        }
      } else {
        uint16_t dst = readModRM16(modrm);
        uint16_t src = m_cpu.getReg16(modrm.reg);
        uint32_t tmp = (static_cast<uint32_t>(dst) << 16) | src;
        uint32_t res32 = tmp << count;
        uint16_t result = static_cast<uint16_t>(res32 >> 16);
        writeModRM16(modrm, result);
        bool cf = (dst >> (16 - count)) & 1;
        flags &=
            ~(FLAG_CARRY | FLAG_ZERO | FLAG_SIGN | FLAG_PARITY | FLAG_OVERFLOW);
        if (cf)
          flags |= FLAG_CARRY;
        if (result == 0)
          flags |= FLAG_ZERO;
        if (result & 0x8000u)
          flags |= FLAG_SIGN;
        {
          uint8_t p = result & 0xFF;
          p ^= p >> 4;
          p ^= p >> 2;
          p ^= p >> 1;
          if (!(p & 1))
            flags |= FLAG_PARITY;
        }
      }
      m_cpu.setEFLAGS(flags);
    }
    break;
  }
  case 0xA5: { // SHLD r/m, r, CL
    ModRM modrm = decodeModRM(fetch8());
    uint8_t count = m_cpu.getReg8(CL) & 0x1F;
    if (count) {
      uint32_t flags = m_cpu.getEFLAGS();
      if (m_hasPrefix66) {
        uint32_t dst = readModRM32(modrm);
        uint32_t src = m_cpu.getReg32(modrm.reg);
        uint64_t tmp = (static_cast<uint64_t>(dst) << 32) | src;
        uint64_t res64 = tmp << count;
        uint32_t result = static_cast<uint32_t>(res64 >> 32);
        writeModRM32(modrm, result);
        bool cf = (dst >> (32 - count)) & 1;
        flags &=
            ~(FLAG_CARRY | FLAG_ZERO | FLAG_SIGN | FLAG_PARITY | FLAG_OVERFLOW);
        if (cf)
          flags |= FLAG_CARRY;
        if (result == 0)
          flags |= FLAG_ZERO;
        if (result & 0x80000000u)
          flags |= FLAG_SIGN;
        {
          uint8_t p = result & 0xFF;
          p ^= p >> 4;
          p ^= p >> 2;
          p ^= p >> 1;
          if (!(p & 1))
            flags |= FLAG_PARITY;
        }
      } else {
        uint16_t dst = readModRM16(modrm);
        uint16_t src = m_cpu.getReg16(modrm.reg);
        uint32_t tmp = (static_cast<uint32_t>(dst) << 16) | src;
        uint32_t res32 = tmp << count;
        uint16_t result = static_cast<uint16_t>(res32 >> 16);
        writeModRM16(modrm, result);
        bool cf = (dst >> (16 - count)) & 1;
        flags &=
            ~(FLAG_CARRY | FLAG_ZERO | FLAG_SIGN | FLAG_PARITY | FLAG_OVERFLOW);
        if (cf)
          flags |= FLAG_CARRY;
        if (result == 0)
          flags |= FLAG_ZERO;
        if (result & 0x8000u)
          flags |= FLAG_SIGN;
        {
          uint8_t p = result & 0xFF;
          p ^= p >> 4;
          p ^= p >> 2;
          p ^= p >> 1;
          if (!(p & 1))
            flags |= FLAG_PARITY;
        }
      }
      m_cpu.setEFLAGS(flags);
    }
    break;
  }
  case 0xAC: { // SHRD r/m, r, imm8
    ModRM modrm = decodeModRM(fetch8());
    uint8_t count = fetch8() & 0x1F;
    if (count) {
      uint32_t flags = m_cpu.getEFLAGS();
      if (m_hasPrefix66) {
        uint32_t dst = readModRM32(modrm);
        uint32_t src = m_cpu.getReg32(modrm.reg);
        uint64_t tmp = (static_cast<uint64_t>(src) << 32) | dst;
        uint64_t res = tmp >> count;
        uint32_t result = static_cast<uint32_t>(res & 0xFFFFFFFF);
        writeModRM32(modrm, result);
        bool cf = (dst >> (count - 1)) & 1;
        flags &=
            ~(FLAG_CARRY | FLAG_ZERO | FLAG_SIGN | FLAG_PARITY | FLAG_OVERFLOW);
        if (cf)
          flags |= FLAG_CARRY;
        if (result == 0)
          flags |= FLAG_ZERO;
        if (result & 0x80000000u)
          flags |= FLAG_SIGN;
        {
          uint8_t p = result & 0xFF;
          p ^= p >> 4;
          p ^= p >> 2;
          p ^= p >> 1;
          if (!(p & 1))
            flags |= FLAG_PARITY;
        }
      } else {
        uint16_t dst = readModRM16(modrm);
        uint16_t src = m_cpu.getReg16(modrm.reg);
        uint32_t tmp = (static_cast<uint32_t>(src) << 16) | dst;
        uint32_t res = tmp >> count;
        uint16_t result = static_cast<uint16_t>(res & 0xFFFF);
        writeModRM16(modrm, result);
        bool cf = (dst >> (count - 1)) & 1;
        flags &=
            ~(FLAG_CARRY | FLAG_ZERO | FLAG_SIGN | FLAG_PARITY | FLAG_OVERFLOW);
        if (cf)
          flags |= FLAG_CARRY;
        if (result == 0)
          flags |= FLAG_ZERO;
        if (result & 0x8000u)
          flags |= FLAG_SIGN;
        {
          uint8_t p = result & 0xFF;
          p ^= p >> 4;
          p ^= p >> 2;
          p ^= p >> 1;
          if (!(p & 1))
            flags |= FLAG_PARITY;
        }
      }
      m_cpu.setEFLAGS(flags);
    }
    break;
  }
  case 0xAD: { // SHRD r/m, r, CL
    ModRM modrm = decodeModRM(fetch8());
    uint8_t count = m_cpu.getReg8(CL) & 0x1F;
    if (count) {
      uint32_t flags = m_cpu.getEFLAGS();
      if (m_hasPrefix66) {
        uint32_t dst = readModRM32(modrm);
        uint32_t src = m_cpu.getReg32(modrm.reg);
        uint64_t tmp = (static_cast<uint64_t>(src) << 32) | dst;
        uint64_t res = tmp >> count;
        uint32_t result = static_cast<uint32_t>(res & 0xFFFFFFFF);
        writeModRM32(modrm, result);
        bool cf = (dst >> (count - 1)) & 1;
        flags &=
            ~(FLAG_CARRY | FLAG_ZERO | FLAG_SIGN | FLAG_PARITY | FLAG_OVERFLOW);
        if (cf)
          flags |= FLAG_CARRY;
        if (result == 0)
          flags |= FLAG_ZERO;
        if (result & 0x80000000u)
          flags |= FLAG_SIGN;
        {
          uint8_t p = result & 0xFF;
          p ^= p >> 4;
          p ^= p >> 2;
          p ^= p >> 1;
          if (!(p & 1))
            flags |= FLAG_PARITY;
        }
      } else {
        uint16_t dst = readModRM16(modrm);
        uint16_t src = m_cpu.getReg16(modrm.reg);
        uint32_t tmp = (static_cast<uint32_t>(src) << 16) | dst;
        uint32_t res = tmp >> count;
        uint16_t result = static_cast<uint16_t>(res & 0xFFFF);
        writeModRM16(modrm, result);
        bool cf = (dst >> (count - 1)) & 1;
        flags &=
            ~(FLAG_CARRY | FLAG_ZERO | FLAG_SIGN | FLAG_PARITY | FLAG_OVERFLOW);
        if (cf)
          flags |= FLAG_CARRY;
        if (result == 0)
          flags |= FLAG_ZERO;
        if (result & 0x8000u)
          flags |= FLAG_SIGN;
        {
          uint8_t p = result & 0xFF;
          p ^= p >> 4;
          p ^= p >> 2;
          p ^= p >> 1;
          if (!(p & 1))
            flags |= FLAG_PARITY;
        }
      }
      m_cpu.setEFLAGS(flags);
    }
    break;
  }

  case 0xBC: { // BSF r16/32, r/m16/32
    ModRM modrm = decodeModRM(fetch8());
    uint16_t val = readModRM16(modrm);
    if (val == 0) {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_ZERO);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_ZERO);
      uint16_t idx = 0;
      while (idx < 16 && !((val >> idx) & 1))
        ++idx;
      m_cpu.setReg16(modrm.reg, idx);
    }
    break;
  }
  case 0xBD: { // BSR r16/32, r/m16/32
    ModRM modrm = decodeModRM(fetch8());
    uint16_t val = readModRM16(modrm);
    if (val == 0) {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_ZERO);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_ZERO);
      uint16_t idx = 15;
      while (idx > 0 && !((val >> idx) & 1))
        --idx;
      m_cpu.setReg16(modrm.reg, idx);
    }
    break;
  }

  case 0xB2: { // LSS r16/32, m16:16
    ModRM modrm = decodeModRM(fetch8());
    uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                  : getEffectiveAddress16(modrm);
    if (m_hasPrefix66) {
      m_cpu.setReg32(modrm.reg, m_memory.read32(addr));
      loadSegment(SS, m_memory.read16(addr + 4));
    } else {
      m_cpu.setReg16(modrm.reg, m_memory.read16(addr));
      loadSegment(SS, m_memory.read16(addr + 2));
    }
    break;
  }
  case 0xB4: { // LFS r16/32, m16:16
    ModRM modrm = decodeModRM(fetch8());
    uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                  : getEffectiveAddress16(modrm);
    if (m_hasPrefix66) {
      m_cpu.setReg32(modrm.reg, m_memory.read32(addr));
      loadSegment(FS, m_memory.read16(addr + 4));
    } else {
      m_cpu.setReg16(modrm.reg, m_memory.read16(addr));
      loadSegment(FS, m_memory.read16(addr + 2));
    }
    break;
  }
  case 0xB5: { // LGS r16/32, m16:16
    ModRM modrm = decodeModRM(fetch8());
    uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm)
                                  : getEffectiveAddress16(modrm);
    if (m_hasPrefix66) {
      m_cpu.setReg32(modrm.reg, m_memory.read32(addr));
      loadSegment(GS, m_memory.read16(addr + 4));
    } else {
      m_cpu.setReg16(modrm.reg, m_memory.read16(addr));
      loadSegment(GS, m_memory.read16(addr + 2));
    }
    break;
  }

  case 0xAF: { // IMUL r16/32, rm16/32
    ModRM modrm = decodeModRM(fetch8());
    if (m_hasPrefix66) {
      int64_t res = static_cast<int64_t>(
                        static_cast<int32_t>(m_cpu.getReg32(modrm.reg))) *
                    static_cast<int32_t>(readModRM32(modrm));
      m_cpu.setReg32(modrm.reg, static_cast<uint32_t>(res));
      // Sets OF/CF if significant bits are lost
    } else {
      int32_t res = static_cast<int32_t>(
                        static_cast<int16_t>(m_cpu.getReg16(modrm.reg))) *
                    static_cast<int16_t>(readModRM16(modrm));
      m_cpu.setReg16(modrm.reg, static_cast<uint16_t>(res));
    }
    break;
  }

  case 0xB6: { // MOVZX r16/32, rm8
    ModRM modrm = decodeModRM(fetch8());
    if (m_hasPrefix66)
      m_cpu.setReg32(modrm.reg, readModRM8(modrm));
    else
      m_cpu.setReg16(modrm.reg, readModRM8(modrm));
    break;
  }
  case 0xB7: { // MOVZX r32, rm16
    ModRM modrm = decodeModRM(fetch8());
    if (m_hasPrefix66)
      m_cpu.setReg32(modrm.reg, readModRM16(modrm));
    else
      LOG_WARN("MOVZX r16, rm16 is illegal");
    break;
  }
  case 0xBE: { // MOVSX r16/32, rm8
    ModRM modrm = decodeModRM(fetch8());
    int8_t val = static_cast<int8_t>(readModRM8(modrm));
    if (m_hasPrefix66)
      m_cpu.setReg32(modrm.reg,
                     static_cast<uint32_t>(static_cast<int32_t>(val)));
    else
      m_cpu.setReg16(modrm.reg,
                     static_cast<uint16_t>(static_cast<int16_t>(val)));
    break;
  }
  case 0xBF: { // MOVSX r32, rm16
    ModRM modrm = decodeModRM(fetch8());
    int16_t val = static_cast<int16_t>(readModRM16(modrm));
    if (m_hasPrefix66)
      m_cpu.setReg32(modrm.reg,
                     static_cast<uint32_t>(static_cast<int32_t>(val)));
    else
      LOG_WARN("MOVSX r16, rm16 is illegal");
    break;
  }

  case 0xFF: { // HLE Interrupt Trap (0x0F 0xFF <vector>)
    uint8_t vector = fetch8();
    LOG_DEBUG("HLE Intercept: INT 0x", std::hex, (int)vector);
    bool handled = false;

    // First try DOS, then BIOS
    if (m_dos.handleInterrupt(vector))
      handled = true;
    else if (m_bios.handleInterrupt(vector))
      handled = true;

    if (handled) {
      // Vector 0xE1 = DPMI entry point (reached via CALL FAR, not INT).
      // Vector 0xE2 = DPMI raw switch PM→RM (reached via JMP FAR).
      // Vector 0xE3 = DPMI raw switch RM→PM (reached via JMP FAR).
      // Handlers already set CS:EIP and mode state — no IRET simulation.
      // Sync InstructionDecoder's segment base cache.
      if (vector == 0xE1 || vector == 0xE2 || vector == 0xE3) {
        loadSegment(CS, m_cpu.getSegReg(CS));
        loadSegment(SS, m_cpu.getSegReg(SS));
        loadSegment(DS, m_cpu.getSegReg(DS));
        loadSegment(ES, m_cpu.getSegReg(ES));
        loadSegment(FS, m_cpu.getSegReg(FS));
        loadSegment(GS, m_cpu.getSegReg(GS));
        break;
      }

      // The HLE code was executed successfully. Now simulate an IRET
      // to return to the caller, but merge the emulation status flags
      // back onto the stack so conditions (CF, ZF) survive the return!
      // Find the matching HLE frame by vector number, which also cleans
      // up any orphaned frames from thunks that handled INTs internally.
      auto hf = m_cpu.popHLEFrameForVector(vector);
      uint32_t currentFlags = m_cpu.getEFLAGS();
      uint32_t frameAddr = hf.framePhysAddr;

      // TEMP: trace IRET simulation
      if (m_cpu.getCR(0) & 1) {
        uint32_t readEIP = hf.is32 ? m_memory.read32(frameAddr)
                                   : m_memory.read16(frameAddr);
        uint16_t readCS = hf.is32
                              ? (m_memory.read32(frameAddr + 4) & 0xFFFF)
                              : m_memory.read16(frameAddr + 2);
        LOG_WARN("0FFF-IRET vec=0x", std::hex, (int)vector,
                 " frameAddr=0x", frameAddr, " frameSP=0x", hf.frameSP,
                 " s32=", hf.stackIs32, " is32=", hf.is32,
                 " readCS=0x", readCS, " readEIP=0x", readEIP,
                 " curESP=0x", m_cpu.getReg32(ESP),
                 " curSP=0x", m_cpu.getReg16(SP),
                 " hleDepth=", m_cpu.hleStackSize());
      }

      if (hf.is32) { // 32-bit IRET (IRETD) frame
        uint32_t newEip = m_memory.read32(frameAddr);
        uint16_t newCs = m_memory.read32(frameAddr + 4) & 0xFFFF;
        uint32_t popFlags = m_memory.read32(frameAddr + 8);

        uint32_t mask = FLAG_CARRY | FLAG_ZERO | FLAG_SIGN | FLAG_OVERFLOW |
                        FLAG_AUX | FLAG_PARITY;
        popFlags = (popFlags & ~mask) | (currentFlags & mask);

        m_cpu.setEFLAGS(popFlags);
        m_cpu.setEIP(newEip);
        loadSegment(CS, newCs);
        if (hf.stackIs32)
          m_cpu.setReg32(ESP, hf.frameSP + 12);
        else
          m_cpu.setReg16(SP, static_cast<uint16_t>(hf.frameSP + 12));
      } else { // 16-bit IRET frame
        uint16_t newIp = m_memory.read16(frameAddr);
        uint16_t newCs = m_memory.read16(frameAddr + 2);
        uint16_t popFlags = m_memory.read16(frameAddr + 4);

        uint16_t mask = FLAG_CARRY | FLAG_ZERO | FLAG_SIGN | FLAG_OVERFLOW |
                        FLAG_AUX | FLAG_PARITY;
        popFlags = (popFlags & ~mask) | (currentFlags & mask);

        m_cpu.setEFLAGS((currentFlags & 0xFFFF0000) | popFlags);
        m_cpu.setEIP(newIp);
        loadSegment(CS, newCs);
        m_cpu.setReg16(SP, static_cast<uint16_t>(hf.frameSP + 6));
      }
    } else {
      LOG_WARN("HLE Intercept: Unhandled INT 0x", std::hex, (int)vector);
      // Fallback: If not handled by HLE (should never happen for 0xFF stubs),
      bool handled_fallback = false;
      if (m_dos.handleInterrupt(vector))
        handled_fallback = true;
      if (!handled_fallback && m_bios.handleInterrupt(vector))
        handled_fallback = true;

      if (handled_fallback) {
        auto hf = m_cpu.popHLEFrameForVector(vector);
        uint32_t frameAddr = hf.framePhysAddr;
        if (hf.is32) {
          uint32_t currentFlags = m_cpu.getEFLAGS();
          uint32_t savedFlags = m_memory.read32(frameAddr + 8);
          uint32_t patchedFlags =
              (savedFlags & ~0xFD5) | (currentFlags & 0xFD5);
          m_memory.write32(frameAddr + 8, patchedFlags);

          m_cpu.setEIP(m_memory.read32(frameAddr));
          loadSegment(CS, m_memory.read32(frameAddr + 4) & 0xFFFF);
          m_cpu.setEFLAGS(m_memory.read32(frameAddr + 8));
          if (hf.stackIs32)
            m_cpu.setReg32(ESP, hf.frameSP + 12);
          else
            m_cpu.setReg16(SP, static_cast<uint16_t>(hf.frameSP + 12));
        } else {
          uint16_t currentFlags = static_cast<uint16_t>(m_cpu.getEFLAGS());
          uint16_t savedFlags = m_memory.read16(frameAddr + 4);
          uint16_t patchedFlags =
              (savedFlags & ~0xFD5) | (currentFlags & 0xFD5);
          m_memory.write16(frameAddr + 4, patchedFlags);

          m_cpu.setEIP(m_memory.read16(frameAddr));
          loadSegment(CS, m_memory.read16(frameAddr + 2));
          m_cpu.setEFLAGS((m_cpu.getEFLAGS() & 0xFFFF0000) |
                          m_memory.read16(frameAddr + 4));
          m_cpu.setReg16(SP, static_cast<uint16_t>(hf.frameSP + 6));
        }
        // Frame already popped by popHLEFrameForVector above
      } else {
        LOG_WARN("HLE Intercept: Unhandled INT 0x", std::hex, (int)vector);
        // Pop the frame; the stub's IRET (CF) will handle the stack return
        // but no longer pops HLE frames.
        m_cpu.popHLEFrameForVector(vector);
      }
    }
    break;
  }

  default:
    LOG_WARN("0x0F 0x", std::hex, static_cast<int>(opcode),
             " not implemented. Generating #UD INT 6.");
    // Generate Invalid Opcode Exception.
    // Rewind EIP to the start of the instruction (including any prefixes)
    // so the faulting address pushed on the stack is correct.
    m_cpu.setEIP(m_instrStartEIP);
    triggerInterrupt(6);
    break;
  }
}

void InstructionDecoder::triggerInterrupt(uint8_t vector) {
  LOG_DEBUG("[CPU] INT 0x", std::hex, (int)vector,
            " CS:EIP=", m_cpu.getSegReg(CS), ":", m_cpu.getEIP(),
            " AX=", m_cpu.getReg16(EAX), " BX=", m_cpu.getReg16(EBX),
            " CX=", m_cpu.getReg16(ECX), " DX=", m_cpu.getReg16(EDX));
  if (vector == 6 || vector == 8 || vector == 13 || vector == 14) {
    LOG_ERROR("CRITICAL DUMP: INT 0x", std::hex, (int)vector,
              " at CS:EIP=", m_cpu.getSegReg(CS), ":", m_cpu.getEIP());
    LOG_ERROR("STATE: CR0:", m_cpu.getCR(0), " EFLAGS:", m_cpu.getEFLAGS(),
              " EAX:", m_cpu.getReg32(EAX), " EBX:", m_cpu.getReg32(EBX),
              " ECX:", m_cpu.getReg32(ECX), " EDX:", m_cpu.getReg32(EDX),
              " ESP:", m_cpu.getReg32(ESP), " EBP:", m_cpu.getReg32(EBP),
              " ESI:", m_cpu.getReg32(ESI), " EDI:", m_cpu.getReg32(EDI),
              " CS:", m_cpu.getSegReg(CS), " DS:", m_cpu.getSegReg(DS),
              " ES:", m_cpu.getSegReg(ES), " SS:", m_cpu.getSegReg(SS),
              " FS:", m_cpu.getSegReg(FS), " GS:", m_cpu.getSegReg(GS));
  }
  if (vector == 0) {
    // Log the faulting address from the stack (if pushed) or current CS:IP
    LOG_WARN("INT 0 (DIV exception) at CS:IP=", std::hex, m_cpu.getSegReg(CS),
             ":", m_cpu.getEIP(), " AX=", m_cpu.getReg16(AX),
             " DX=", m_cpu.getReg16(DX), " CX=", m_cpu.getReg16(CX),
             " BX=", m_cpu.getReg16(BX));
  }
  uint16_t cs, ip;
  uint32_t eip32 = 0;
  bool use32 = false;

  uint16_t oldCs = m_cpu.getSegReg(CS);
  uint16_t oldSs = m_cpu.getSegReg(SS);
  uint32_t oldEsp = m_cpu.getReg32(ESP);
  uint8_t oldCpl = oldCs & 3;

  if (m_cpu.getCR(0) & 1) {
    // Protected Mode IDT lookup
    uint32_t idtBase = m_cpu.getIDTR().base;
    uint32_t entryAddr = idtBase + vector * 8;
    uint32_t low = m_memory.read32(entryAddr);
    uint32_t high = m_memory.read32(entryAddr + 4);
    cs = (low >> 16) & 0xFFFF;
    eip32 = (low & 0xFFFF) | (high & 0xFFFF0000);

    // Gate type: bits 8-11 of high dword
    uint8_t type = (high >> 8) & 0x0F;
    use32 = (type & 0x08) != 0;
    bool isInterruptGate = (type & 0x01) == 0;
    uint8_t newCpl = cs & 3;

    LOG_DEBUG("PM INT 0x", std::hex, (int)vector, " selector: 0x", cs,
              " offset: 0x", eip32, " size: ", use32 ? 32 : 16, " type: 0x",
              (int)type, " CPL:", (int)oldCpl, "->", (int)newCpl);

    // TEMP: trace INT 21h AH=FF calls
    if (vector == 0x21 && m_cpu.getReg8(cpu::AH) == 0xFF) {
      LOG_WARN("INT21-FF from CS:EIP=", std::hex, oldCs, ":", m_cpu.getEIP(),
               " AX=", m_cpu.getReg16(cpu::AX),
               " BX=", m_cpu.getReg16(cpu::BX),
               " CX=", m_cpu.getReg16(cpu::CX),
               " DX=", m_cpu.getReg16(cpu::DX),
               " DS=", m_cpu.getSegReg(DS),
               " ES=", m_cpu.getSegReg(ES),
               " -> ", cs, ":", eip32);
    }

    // Privilege change check
    bool privChange = (newCpl < oldCpl) || (m_cpu.getEFLAGS() & 0x00020000);

    m_cpu.pushHLEFrame(use32, vector);

    // HLE check (only handled if it points to our stub, avoiding hooks)
    if (!(m_cpu.getEFLAGS() & 0x00020000)) {
      bool isOrig = m_bios.isOriginalIVT(vector, cs, eip32);
      if (isOrig) {
        if (m_dos.handleInterrupt(vector)) {
          m_cpu.popHLEFrame();
          return;
        }
        if (m_bios.handleInterrupt(vector)) {
          m_cpu.popHLEFrame();
          return;
        }
      }
    }

    // Only clear IF after HLE check — HLE handlers return directly
    // to the caller without an IRET to restore it.
    if (isInterruptGate) {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~fador::cpu::FLAG_INTERRUPT);
    }

    if (privChange) {
      // In a real CPU, we would look up the new SS/ESP in the TSS.
      // For fadors, we'll assume Ring 0 stack is already set up or just push
      // them. Most DPMI clients/servers in Ring 3 -> Ring 0 will need this.
      if (use32) {
        m_cpu.push32(oldSs);
        m_cpu.push32(oldEsp);
      } else {
        m_cpu.push16(oldSs);
        m_cpu.push16(static_cast<uint16_t>(oldEsp & 0xFFFF));
      }
    }

    if (use32) {
      m_cpu.push32(m_cpu.getEFLAGS());
      m_cpu.push32(oldCs);
      m_cpu.push32(m_cpu.getEIP());
    } else {
      m_cpu.push16(static_cast<uint16_t>(m_cpu.getEFLAGS() & 0xFFFF));
      m_cpu.push16(oldCs);
      m_cpu.push16(static_cast<uint16_t>(m_cpu.getEIP() & 0xFFFF));
    }

    // Record IRET frame location in HLE frame so the 0F FF handler
    // can find it even if client thunks modify ESP between push and handler.
    {
      auto &frame = m_cpu.lastHLEFrameMut();
      uint32_t ssB = m_cpu.getSegBase(cpu::SS);
      bool s32 = m_cpu.is32BitStack();
      uint32_t sp = s32 ? m_cpu.getReg32(ESP)
                        : static_cast<uint32_t>(m_cpu.getReg16(SP));
      frame.framePhysAddr = ssB + sp;
      frame.frameSP = sp;
      frame.stackIs32 = s32;
      // TEMP: trace frame recording
      if (vector == 0x31) {
        LOG_WARN("IRET-REC vec=0x31 phys=0x", std::hex, frame.framePhysAddr,
                 " sp=0x", sp, " ssB=0x", ssB, " s32=", s32,
                 " is32=", frame.is32,
                 " hleDepth=", m_cpu.hleStackSize(),
                 " isOrig=", m_bios.isOriginalIVT(vector, cs, eip32));
      }
    }
  } else {
    // Real Mode IVT lookup
    ip = m_memory.read16(vector * 4);
    cs = m_memory.read16(vector * 4 + 2);
    eip32 = ip;
    use32 = false;
    m_cpu.pushHLEFrame(use32, vector);

    // HLE shortcut: if the IVT still points to our original HLE stubs,
    // handle the interrupt directly without pushing an IRET frame.
    bool isOrig = m_bios.isOriginalIVT(vector, cs, eip32);
    if (isOrig) {
      if (m_dos.handleInterrupt(vector)) {
        m_cpu.popHLEFrame();
        return;
      }
      if (m_bios.handleInterrupt(vector)) {
        m_cpu.popHLEFrame();
        return;
      }
    }

    m_cpu.push16(static_cast<uint16_t>(m_cpu.getEFLAGS() & 0xFFFF));
    m_cpu.push16(oldCs);
    m_cpu.push16(static_cast<uint16_t>(m_cpu.getEIP() & 0xFFFF));

    // Record IRET frame location for RM path
    {
      auto &frame = m_cpu.lastHLEFrameMut();
      uint32_t ssB = m_cpu.getSegBase(cpu::SS);
      uint32_t sp = static_cast<uint32_t>(m_cpu.getReg16(SP));
      frame.framePhysAddr = ssB + sp;
      frame.frameSP = sp;
      frame.stackIs32 = false;
    }
  }

  m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~(0x0200 | 0x0100)); // Clear IF and TF
  loadSegment(CS, cs);
  m_cpu.setEIP(eip32);
}

void InstructionDecoder::injectHardwareInterrupt(uint8_t vector) {
  uint16_t cs;
  uint32_t eip32;
  bool use32 = false;

  LOG_ERROR("HW IRQ: Injecting Vector 0x", std::hex, (int)vector,
            " CR0=", m_cpu.getCR(0));

  uint16_t oldCs = m_cpu.getSegReg(CS);
  uint16_t oldSs = m_cpu.getSegReg(SS);
  uint32_t oldEsp = m_cpu.getReg32(ESP);
  uint8_t oldCpl = oldCs & 3;

  if (m_cpu.getCR(0) & 1) {
    uint32_t idtBase = m_cpu.getIDTR().base;
    uint32_t entryAddr = idtBase + vector * 8;
    uint32_t low = m_memory.read32(entryAddr);
    uint32_t high = m_memory.read32(entryAddr + 4);
    cs = (low >> 16) & 0xFFFF;
    eip32 = (low & 0xFFFF) | (high & 0xFFFF0000);

    // Gate type: bits 8-11 of high dword
    uint8_t type = (high >> 8) & 0x0F;
    use32 = (type & 0x08) != 0;
    bool isInterruptGate = (type & 0x01) == 0;
    uint8_t newCpl = cs & 3;

    LOG_DEBUG("HW IRQ PM 0x", std::hex, (int)vector, " selector: 0x", cs,
              " offset: 0x", eip32, " size: ", use32 ? 32 : 16, " type: 0x",
              (int)type, " CPL:", (int)oldCpl, "->", (int)newCpl);

    if (isInterruptGate) {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~fador::cpu::FLAG_INTERRUPT);
    }

    bool privChange = (newCpl < oldCpl) || (m_cpu.getEFLAGS() & 0x00020000);
    m_cpu.pushHLEFrame(use32, vector);

    if (privChange) {
      if (use32) {
        m_cpu.push32(oldSs);
        m_cpu.push32(oldEsp);
      } else {
        m_cpu.push16(oldSs);
        m_cpu.push16(static_cast<uint16_t>(oldEsp & 0xFFFF));
      }
    }

    if (use32) {
      m_cpu.push32(m_cpu.getEFLAGS());
      m_cpu.push32(oldCs);
      m_cpu.push32(m_cpu.getEIP());
    } else {
      m_cpu.push16(static_cast<uint16_t>(m_cpu.getEFLAGS() & 0xFFFF));
      m_cpu.push16(oldCs);
      m_cpu.push16(static_cast<uint16_t>(m_cpu.getEIP() & 0xFFFF));
    }

    // Record IRET frame location for HW IRQ path too
    {
      auto &frame = m_cpu.lastHLEFrameMut();
      uint32_t ssB = m_cpu.getSegBase(cpu::SS);
      bool s32 = m_cpu.is32BitStack();
      uint32_t sp = s32 ? m_cpu.getReg32(ESP)
                        : static_cast<uint32_t>(m_cpu.getReg16(SP));
      frame.framePhysAddr = ssB + sp;
      frame.frameSP = sp;
      frame.stackIs32 = s32;
    }
  } else {
    eip32 = m_memory.read16(vector * 4);
    cs = m_memory.read16(vector * 4 + 2);
    use32 = false;
    m_cpu.pushHLEFrame(use32, vector);
    m_cpu.push16(static_cast<uint16_t>(m_cpu.getEFLAGS() & 0xFFFF));
    m_cpu.push16(oldCs);
    m_cpu.push16(static_cast<uint16_t>(m_cpu.getEIP() & 0xFFFF));

    // Record IRET frame location for HW IRQ RM path
    {
      auto &frame = m_cpu.lastHLEFrameMut();
      uint32_t ssB = m_cpu.getSegBase(cpu::SS);
      uint32_t sp = static_cast<uint32_t>(m_cpu.getReg16(SP));
      frame.framePhysAddr = ssB + sp;
      frame.frameSP = sp;
      frame.stackIs32 = false;
    }
  }

  m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~(0x0200 | 0x0100)); // Clear IF and TF
  loadSegment(CS, cs);
  m_cpu.setEIP(eip32);
}

void InstructionDecoder::syncSegments() {
  for (int i = 0; i < 6; ++i) {
    loadSegment(static_cast<SegRegIndex>(i), m_cpu.getSegReg(i));
  }
}

InstructionDecoder::Descriptor
InstructionDecoder::decodeDescriptor(uint32_t low, uint32_t high) {
  Descriptor desc;
  // Base address: low bits 16-31, high bits 0-7 and 24-31
  desc.base = ((low >> 16) & 0xFFFF) | ((high & 0x000000FF) << 16) |
              (high & 0xFF000000);

  // Segment Limit: low bits 0-15, high bits 16-19
  uint32_t limit = (low & 0xFFFF) | (high & 0x000F0000);
  if (high & 0x00800000) { // Granularity (G) bit (bit 23 of high dword)
    desc.limit = (limit << 12) | 0xFFF;
  } else {
    desc.limit = limit;
  }

  desc.is32Bit = (high & 0x00400000) != 0;   // D/B bit (bit 22 of high dword)
  desc.isPresent = (high & 0x00008000) != 0; // P bit (bit 15 of high dword)
  desc.dpl = (high >> 13) & 3; // DPL bits (bits 13-14 of high dword)
  desc.isSystem = (high & 0x00001000) ==
                  0; // S bit (bit 12 of high dword): 0=Sys, 1=Code/Data
  desc.type = (high >> 8) & 0x0F; // Type bits (bits 8-11 of high dword)

  return desc;
}

void InstructionDecoder::loadSegment(SegRegIndex seg, uint16_t selector) {
  m_cpu.setSegReg(seg, selector);

  bool isRealMode = !(m_cpu.getCR(0) & 1) || (m_cpu.getEFLAGS() & 0x00020000);

  if (isRealMode) {
    // Real Mode or Virtual 8086 Mode
    m_segBase[seg] = static_cast<uint32_t>(selector) << 4;
    m_cpu.setSegBase(seg, m_segBase[seg]);
    if (seg == CS) {
      m_cpu.setIs32BitCode(false);
    }
    if (seg == SS) {
      m_cpu.setIs32BitStack(false);
    }
  } else {
    // Protected Mode (Simplified)
    if (selector == 0 && seg != CS && seg != SS) {
      m_segBase[seg] = 0; // Null segment
      m_cpu.setSegBase(seg, 0);
      return;
    }

    uint32_t tableBase =
        (selector & 0x04) ? m_cpu.getLDTR().base : m_cpu.getGDTR().base;
    uint32_t entryAddr = tableBase + (selector & ~7);

    // Read descriptor (8 bytes)
    uint32_t low = m_memory.read32(entryAddr);
    uint32_t high = m_memory.read32(entryAddr + 4);

    Descriptor desc = decodeDescriptor(low, high);
    m_segBase[seg] = desc.base;
    m_cpu.setSegBase(seg, desc.base);

    if (seg == CS) {
      m_cpu.setIs32BitCode(desc.is32Bit);
      // TEMP: debug CS load
      {
        std::string hex;
        uint32_t codeAddr = desc.base + m_cpu.getEIP();
        for (int i = 0; i < 16; i++) {
          char buf[4];
          snprintf(buf, sizeof(buf), "%02X ", m_memory.read8(codeAddr + i));
          hex += buf;
        }
        // Also check raw memory at the address (bypassing A20)
        std::string rawHex;
        for (int i = 0; i < 16; i++) {
          char buf[4];
          snprintf(buf, sizeof(buf), "%02X ", m_memory.directAccess(0)[codeAddr + i]);
          rawHex += buf;
        }
        LOG_WARN("loadSegment CS sel=0x", std::hex, selector,
                 " tableBase=0x", tableBase,
                 " entryAddr=0x", entryAddr,
               " low=0x", low, " high=0x", high,
               " base=0x", desc.base, " limit=0x", desc.limit,
               " 32bit=", desc.is32Bit,
               " A20=", m_memory.isA20Enabled(),
               " EIP=0x", m_cpu.getEIP(),
               " SS=0x", m_cpu.getSegReg(cpu::SS),
               " ESP=0x", m_cpu.getReg32(cpu::ESP),
               " is32Stack=", m_cpu.is32BitStack(),
               " bytes=", hex,
               " raw=", rawHex);
      }
    }
    if (seg == SS) {
      m_cpu.setIs32BitStack(desc.is32Bit);
    }

    LOG_DEBUG("PM Load Segment ", (int)seg, " selector: 0x", std::hex, selector,
              " base: 0x", desc.base, " limit: 0x", desc.limit,
              " 32bit:", desc.is32Bit);
  }
}

} // namespace fador::cpu