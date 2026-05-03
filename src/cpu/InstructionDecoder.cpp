#include "InstructionDecoder.hpp"
#include "../hw/BIOS.hpp"
#include "../hw/DOS.hpp"
#include "../hw/IOBus.hpp"
#include "../utils/Logger.hpp"
#include <cstring>

namespace fador::cpu {

InstructionDecoder::InstructionDecoder(CPU &cpu, memory::MemoryBus &memory,
                                       hw::IOBus &iobus, hw::BIOS &bios,
                                       hw::DOS &dos)
    : m_cpu(cpu), m_memory(memory), m_iobus(iobus), m_bios(bios), m_dos(dos),
      m_stepCount(0), m_hasPrefix66(false), m_hasPrefix67(false),
      m_hasRepnz(false), m_hasRepz(false), m_segmentOverride(0xFF),
      m_cachedSegStateVersion(0),
      m_currentEA(0), m_currentOffset(0), m_eaResolved(false) {
  m_cpu.setMemoryBus(&m_memory);
  syncSegments();
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

void InstructionDecoder::syncSegmentCacheIfNeeded() {
  if (m_cachedSegStateVersion != m_cpu.getSegmentStateVersion()) {
    syncSegments();
  }
}

bool InstructionDecoder::tryFastRepMovs(uint32_t elementSize) {
  uint32_t count = m_hasPrefix67 ? m_cpu.getReg32(ECX) : m_cpu.getReg16(CX);
  if (count == 0) {
    return true;
  }

  constexpr uint32_t MAX_REP_BATCH = 4096;
  uint32_t iterations = count > MAX_REP_BATCH ? MAX_REP_BATCH : count;
  uint8_t srcSeg = (m_segmentOverride != 0xFF) ? m_segmentOverride : DS;
  uint32_t srcOff = m_hasPrefix67 ? m_cpu.getReg32(ESI) : m_cpu.getReg16(SI);
  uint32_t dstOff = m_hasPrefix67 ? m_cpu.getReg32(EDI) : m_cpu.getReg16(DI);
  bool decrement = (m_cpu.getEFLAGS() & FLAG_DIRECTION) != 0;
  uint32_t srcStart = srcOff;
  uint32_t dstStart = dstOff;
  uint64_t startSpan =
      static_cast<uint64_t>(elementSize) * static_cast<uint64_t>(iterations - 1);

  if (decrement) {
    if (startSpan > srcOff || startSpan > dstOff) {
      return false;
    }
    srcStart -= static_cast<uint32_t>(startSpan);
    dstStart -= static_cast<uint32_t>(startSpan);
  } else {
    uint64_t addressLimit = m_hasPrefix67 ? 0xFFFFFFFFull : 0xFFFFull;
    if (static_cast<uint64_t>(srcOff) + startSpan > addressLimit ||
        static_cast<uint64_t>(dstOff) + startSpan > addressLimit) {
      return false;
    }
  }

  uint32_t byteCount = iterations * elementSize;

  uint8_t *srcPtr =
      m_memory.contiguousAccess(m_segBase[srcSeg] + srcStart, byteCount);
  uint8_t *dstPtr =
      m_memory.contiguousAccess(m_segBase[ES] + dstStart, byteCount);
  if (srcPtr == nullptr || dstPtr == nullptr) {
    return false;
  }

  std::memmove(dstPtr, srcPtr, byteCount);

  uint32_t remaining = count - iterations;
  if (m_hasPrefix67) {
    m_cpu.setReg32(ECX, remaining);
    m_cpu.setReg32(ESI, decrement ? srcOff - byteCount : srcOff + byteCount);
    m_cpu.setReg32(EDI, decrement ? dstOff - byteCount : dstOff + byteCount);
  } else {
    m_cpu.setReg16(CX, static_cast<uint16_t>(remaining));
    m_cpu.setReg16(SI, static_cast<uint16_t>(decrement ? srcOff - byteCount
                                                       : srcOff + byteCount));
    m_cpu.setReg16(DI, static_cast<uint16_t>(decrement ? dstOff - byteCount
                                                       : dstOff + byteCount));
  }

  if (remaining != 0) {
    m_cpu.setEIP(m_instrStartEIP);
  }
  return true;
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
  else {
    uint32_t addr =
        m_hasPrefix67 ? getEffectiveAddress32(modrm) : getEffectiveAddress16(modrm);
    m_memory.write32(addr, value);
  }
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
  syncSegmentCacheIfNeeded();
  bool default32 = m_cpu.is32BitCode();

  m_hasPrefix66 = default32;
  m_hasPrefix67 = default32;
  m_hasRepnz = false;
  m_hasRepz = false;
  m_segmentOverride = 0xFF;
  m_eaResolved = false;

  m_instrStartEIP = m_cpu.getEIP();
  m_cpu.setInstructionStartEIP(m_instrStartEIP);
  uint8_t opcode = fetch8();
  // Temporary diagnostic: log the next few bytes at the instruction start
  {
    // DECODER FETCH instrumentation is noisy; keep it behind an opt-in define.
#ifdef ENABLE_DECODER_FETCH_LOG
    uint32_t phys = m_segBase[CS] + m_instrStartEIP;
    uint8_t b0 = m_memory.read8(phys);
    uint8_t b1 = m_memory.read8(phys + 1);
    uint8_t b2 = m_memory.read8(phys + 2);
    LOG_CPU("DECODER FETCH: CS:EIP=0x", std::hex, m_cpu.getSegReg(CS), ":", m_instrStartEIP,
            " bytes=", std::hex, static_cast<int>(b0), " ", static_cast<int>(b1), " ", static_cast<int>(b2));
#endif
  }

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
        uint32_t movsElementSize = 0;
        if (opcode == 0xA4) {
          movsElementSize = 1;
        } else if (opcode == 0xA5) {
          movsElementSize = m_hasPrefix66 ? 4u : 2u;
        }
        if (movsElementSize != 0 && tryFastRepMovs(movsElementSize)) {
          return;
        }

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
    // Compute the physical address of the IRET source frame BEFORE popping,
    // so we can match it against stored HLE frames afterwards.
    bool iretStackIs32 = m_cpu.is32BitStack();
    uint32_t iretSP = iretStackIs32 ? m_cpu.getReg32(ESP)
                                    : static_cast<uint32_t>(m_cpu.getReg16(SP));
    uint32_t iretPhysAddr = m_cpu.getSegBase(SS) + iretSP;

    // PM handler return interception: if the thunk dispatched to a PM
    // handler (dispatchedToPM flag), intercept the handler's IRET before
    // it pops.  The handler may use the wrong IRET size (16-bit IRET in
    // 32-bit code) causing stack corruption.  We skip the normal pop and
    // restore the full original state from the HLE frame.
    //
    // Guard: verify the IRET return frame on the stack actually contains
    // our pushed origEIP:origCS.  Without this, nested interrupts that
    // return TO the PM handler (same CS) would falsely trigger the
    // interception, prematurely ending the handler and corrupting state.
    {
      uint16_t preIretCS = m_cpu.getSegReg(CS);
      const auto *pmPeek = m_cpu.peekDpmiFrameByDispatchedPM();
      if (pmPeek) {
        // Read what the IRET would pop from the stack
        uint32_t ssBase = m_cpu.getSegBase(SS);
        uint32_t curEsp = m_cpu.is32BitStack()
                              ? m_cpu.getReg32(ESP)
                              : static_cast<uint32_t>(m_cpu.getReg16(SP));
        uint32_t stackEIP = m_memory.read32(ssBase + curEsp);
        uint16_t stackCS = static_cast<uint16_t>(
            m_memory.read32(ssBase + curEsp + 4) & 0xFFFF);

        if (stackEIP == pmPeek->origEIP && stackCS == pmPeek->origCS) {
          // Stack matches our pushed return frame — this is the final IRET
          auto pmFrame = m_cpu.popDpmiFrameByDispatchedPM();
          loadSegment(CS, pmFrame.origCS);
          m_cpu.setEIP(pmFrame.origEIP);
          m_cpu.setEFLAGS(pmFrame.origEFLAGS);
          loadSegment(SS, pmFrame.origSS);
          m_cpu.setReg32(ESP, pmFrame.origESP);
          loadSegment(DS, pmFrame.origDS);
          loadSegment(ES, pmFrame.origES);
          // Restore GPRs + extra segments saved at TH-PM dispatch
          m_cpu.setReg32(EAX, pmFrame.savedEAX);
          m_cpu.setReg32(EBX, pmFrame.savedEBX);
          m_cpu.setReg32(ECX, pmFrame.savedECX);
          m_cpu.setReg32(EDX, pmFrame.savedEDX);
          m_cpu.setReg32(ESI, pmFrame.savedESI);
          m_cpu.setReg32(EDI, pmFrame.savedEDI);
          m_cpu.setReg32(EBP, pmFrame.savedEBP);
          loadSegment(FS, pmFrame.savedFS);
          loadSegment(GS, pmFrame.savedGS);
          LOG_CPU("PM handler IRET intercepted: vec=0x", std::hex,
                  (int)pmFrame.vector,
                  " \u2192 CS:EIP=0x", pmFrame.origCS, ":", pmFrame.origEIP,
                  " SS:ESP=0x", pmFrame.origSS, ":", pmFrame.origESP);
          break;
        }
        // Otherwise: nested interrupt returning to PM handler — let
        // normal IRET proceed.
      }
    }

    bool useIretd = m_hasPrefix66;
    LOG_CPU("IRET: iretStackIs32=", iretStackIs32, " iretSP=0x", std::hex,
            iretSP, " iretPhys=0x", iretPhysAddr, " initialUseIretd=",
            useIretd);
    {
      auto scoreFrame = [&](uint16_t cs, uint32_t ip, uint32_t flags, bool expect32) -> int {
        if ((cs & ~7) == 0) return -100;
        uint32_t tbl = (cs & 0x04) ? m_cpu.getLDTR().base : m_cpu.getGDTR().base;
        uint32_t dLow = m_memory.read32(tbl + (cs & ~7));
        uint32_t dHigh = m_memory.read32(tbl + (cs & ~7) + 4);
        Descriptor d = decodeDescriptor(dLow, dHigh);

        if (d.isSystem || !(d.type & 0x08) || !d.isPresent) return -100;
        
        int score = 100;
        uint32_t limit = d.limit;
        if (ip > limit && limit > 0) return -200;
        else score += 50;
        
        if (d.is32Bit == expect32) score += 20;
        if ((flags & 0x2A) == 0x02) score += 10;
        return score;
      };


      uint32_t currentESP = m_cpu.is32BitStack() ? m_cpu.getReg32(ESP)
                                                 : m_cpu.getReg16(SP);
      uint32_t ssBase = m_cpu.getSegBase(SS);

      uint16_t cs16 = m_memory.read16(ssBase + currentESP + 2);
      uint16_t flags16 = m_memory.read16(ssBase + currentESP + 4);
      uint32_t ip16 = m_memory.read16(ssBase + currentESP);

      uint16_t cs32 = m_memory.read16(ssBase + currentESP + 4);
      uint32_t flags32 = m_memory.read32(ssBase + currentESP + 8);
      uint32_t eip32 = m_memory.read32(ssBase + currentESP);

      int score16 = scoreFrame(cs16, ip16, flags16, false);
      int score32 = scoreFrame(cs32, eip32, flags32, true);

      LOG_DEBUG("IRET scoring: cs16=0x", std::hex, cs16, " ip16=0x", ip16,
                " flags16=0x", flags16, " score16=", score16,
                " cs32=0x", cs32, " eip32=0x", eip32, " flags32=0x",
                flags32, " score32=", score32);

      if ((m_cpu.getCR(0) & 1) && m_cpu.getSegReg(CS) == 0x0008) {
        // Full scoring for HLE stub IRETs (original safe behaviour)
        if (score32 > score16 && score32 >= 0) {
          useIretd = true;
        } else if (score16 > score32 && score16 >= 0) {
          useIretd = false;
        } else if (score16 >= 0 && score32 >= 0 && score16 == score32) {
          useIretd = true;
        }
        LOG_DEBUG("IRET decision: useIretd=", useIretd);
      } else if ((m_cpu.getCR(0) & 1) && !useIretd &&
                 score32 >= 100 && score16 < 0) {
        // Conservative override for 16-bit thunk code doing IRET on a
        // 32-bit gate frame: the 16-bit interpretation gives clearly
        // invalid CS (null/non-code/not-present → negative score) while
        // the 32-bit one yields a valid code segment.  Only promote
        // 16→32; never demote the other direction.
        useIretd = true;
        LOG_DEBUG("IRET conservative 16→32: score16=", score16,
                  " score32=", score32);
      }
    }

    if (useIretd) {
      // IRETD — pops 32-bit EIP, CS (zero-ext), EFLAGS
      uint32_t esp = m_cpu.getReg32(ESP);
      uint32_t ssBaseLocal = m_cpu.getSegBase(SS);
      uint32_t newEip = m_memory.read32(ssBaseLocal + esp);
      uint16_t newCs = m_memory.read32(ssBaseLocal + esp + 4) & 0xFFFF;
      uint32_t newEflags = m_memory.read32(ssBaseLocal + esp + 8);
      uint8_t oldCpl = m_cpu.getSegReg(CS) & 3;
      uint8_t newCpl = newCs & 3;
      bool wasV86 = (m_cpu.getEFLAGS() & 0x00020000) != 0;
      bool isV86 = (newEflags & 0x00020000) != 0;

      LOG_CPU("IRETD: oldEFLAGS=0x", std::hex, m_cpu.getEFLAGS(),
              " newEFLAGS=0x", newEflags, " newCS=0x", newCs,
              " newEIP=0x", newEip);

      if (!wasV86 && isV86) {
        uint32_t newEsp = m_memory.read32(ssBaseLocal + esp + 12);
        uint16_t newSs = m_memory.read32(ssBaseLocal + esp + 16) & 0xFFFF;
        uint16_t newEs = m_memory.read32(ssBaseLocal + esp + 20) & 0xFFFF;
        uint16_t newDs = m_memory.read32(ssBaseLocal + esp + 24) & 0xFFFF;
        uint16_t newFs = m_memory.read32(ssBaseLocal + esp + 28) & 0xFFFF;
        uint16_t newGs = m_memory.read32(ssBaseLocal + esp + 32) & 0xFFFF;

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
          uint32_t newEsp = m_memory.read32(ssBaseLocal + esp + 12);
          uint16_t newSs = m_memory.read32(ssBaseLocal + esp + 16) & 0xFFFF;
          m_cpu.setReg32(ESP, newEsp);
          loadSegment(SS, newSs);
        } else {
          m_cpu.setReg32(ESP, esp + 12);
        }
      }
    } else {
      // 16-bit IRET: pops IP (16), CS (16), FLAGS (16).
      // Stack address size comes from SS.B, not the operand size.
      bool iretStack32 = m_cpu.is32BitStack();
      uint32_t sp = iretStack32 ? m_cpu.getReg32(ESP)
                                : static_cast<uint32_t>(m_cpu.getReg16(SP));
      uint32_t ssBaseLocal2 = m_cpu.getSegBase(SS);
      uint16_t newIp = m_memory.read16(ssBaseLocal2 + sp);
      uint16_t newCs = m_memory.read16(ssBaseLocal2 + sp + 2);
      uint16_t newFlags = m_memory.read16(ssBaseLocal2 + sp + 4);
      uint8_t oldCpl = m_cpu.getSegReg(CS) & 3;
      uint8_t newCpl = newCs & 3;

      LOG_CPU("IRET (16-bit): oldEFLAGS=0x", std::hex, m_cpu.getEFLAGS(),
              " newFlags=0x", newFlags, " newCS=0x", newCs,
              " newIP=0x", newIp);

      uint32_t merged = (m_cpu.getEFLAGS() & 0xFFFF0000) | newFlags;
      m_cpu.setEFLAGS(merged);
      m_cpu.setEIP(newIp);
      loadSegment(CS, newCs);

      if (newCpl > oldCpl && (m_cpu.getCR(0) & 1)) {
        uint16_t newSp = m_memory.read16(ssBaseLocal2 + sp + 6);
        uint16_t newSs = m_memory.read16(ssBaseLocal2 + sp + 8);
        if (iretStack32)
          m_cpu.setReg32(ESP, newSp);
        else
          m_cpu.setReg16(SP, newSp);
        loadSegment(SS, newSs);
      } else {
        if (iretStack32)
          m_cpu.setReg32(ESP, sp + 6);
        else
          m_cpu.setReg16(SP, (sp + 6) & 0xFFFF);
      }
    }
    // Pop the matching HLE frame if this IRET consumes a frame that was
    // pushed by triggerInterrupt. Matches by physical address to distinguish:
    //  - Thunk IRETD (fake frame at different address) → no match → no pop
    //  - Thunk final IRET (original frame address) → match → pop
    // For forwarded interrupts, the 0F FF handler pops via
    // popHLEFrameForVector instead.
    {
        auto poppedFrame = m_cpu.popAndGetHLEFrameByPhysAddr(iretPhysAddr);
        bool popped = (poppedFrame.framePhysAddr != 0);
        LOG_CPU("IRET popHLEFrameByPhysAddr: addr=0x", std::hex, iretPhysAddr,
          " popped=", popped, " hleStackSize=", m_cpu.hleStackSize());

#if FADOR_ENABLE_DEBUG_DIAGNOSTICS
        // TEMP DIAG: log when HLE frames are deep
        if (m_cpu.hleStackSize() > 4) {
          uint16_t iretCS = m_cpu.getSegReg(CS);
          uint32_t iretEIP = m_cpu.getEIP();
          static int s_iretLog = 0;
          if (s_iretLog < 20) {
            ++s_iretLog;
            LOG_ERROR("HLE-IRET: popped=", popped,
                      " depth=", std::dec, m_cpu.hleStackSize(),
                      " iretPhysAddr=0x", std::hex, iretPhysAddr,
                      " CS:EIP=", iretCS, ":", iretEIP,
                      " SS:ESP=", m_cpu.getSegReg(SS), ":", m_cpu.getReg32(ESP),
                      " useIretd=", useIretd);
          }
        }
#endif
        // Fallback: if the physical-address match failed, check if the IRET
        // returned to a CS:EIP that matches a dpmiStackSwitch frame's saved
        // origCS:origEIP.  The DOS/4GW thunk rearranges its internal stack
        // during dispatch, so the IRET happens at a different stack address
        // than where we pushed the frame.  But the thunk's final IRET always
        // restores CS:EIP to the interrupted code — we detect that here.
        if (!popped) {
          uint16_t newCs = m_cpu.getSegReg(CS);
          uint32_t newEip = m_cpu.getEIP();
          // Check if we returned to origCS:origEIP exactly (final return)
          poppedFrame = m_cpu.popDpmiFrameByCSEIP(newCs, newEip);
          popped = (poppedFrame.framePhysAddr != 0);
          if (popped) {
            LOG_CPU("IRET dpmiStackSwitch matched by CS:EIP=0x",
                    std::hex, newCs, ":", newEip,
                    " vec=0x", (int)poppedFrame.vector);
          }
        }
        // Thunk → PM handler dispatch: the thunk's IRET went to origCS
        // but at a different EIP (the PM handler, e.g. I_KeyboardISR).
        // On real hardware the DPMI host would set SS:ESP to the app's
        // stack and push a return frame pointing to the interrupted code.
        // We do that here: fix SS:ESP, push a 32-bit IRET frame, and mark
        // the HLE frame as dispatched so the handler's IRET is intercepted.
        //
        // Guard: only dispatch when the IRET destination is a 32-bit LDT
        // code segment (the app's PM handler).  The thunk's internal IRETs
        // (to its own 16-bit CS=0x8F) and HLE chain calls (to GDT CS=0x08)
        // must NOT trigger this path — doing so would prematurely switch
        // the stack and skip the actual PM handler dispatch.
        if (!popped) {
          uint16_t newCs = m_cpu.getSegReg(CS);
          uint32_t newEip = m_cpu.getEIP();
          auto dpmiFrame = m_cpu.peekUndispatchedHwIrqFrame();
          if (dpmiFrame.framePhysAddr != 0 && dpmiFrame.origEIP != newEip) {
            // Verify the IRET destination is the PM handler (32-bit LDT
            // code segment), not the thunk itself or an HLE stub.
            bool isPMHandler = false;
            if (newCs & 0x04) { // LDT selector
              uint32_t tbl = m_cpu.getLDTR().base;
              uint32_t dLow = m_memory.read32(tbl + (newCs & ~7));
              uint32_t dHigh = m_memory.read32(tbl + (newCs & ~7) + 4);
              Descriptor d = decodeDescriptor(dLow, dHigh);
              isPMHandler = d.is32Bit && d.isPresent && !d.isSystem &&
                            (d.type & 0x08); // 32-bit present code segment
            }
            if (isPMHandler) {
            // Use the application's stack for the PM handler, not the
            // thunk's reflection stack.  When a HW IRQ fires inside a
            // thunk (nested reflection), dpmiFrame.origSS:origESP points
            // to the thunk's reflection stack (SS:0xAF).  Running the PM
            // handler on that same stack corrupts the thunk's data.  The
            // outermost dpmiStackSwitch frame holds the true app stack.
            uint16_t pmSS = dpmiFrame.origSS;
            uint32_t pmESP = dpmiFrame.origESP;
            auto *outerFrame = m_cpu.peekOutermostDpmiFrame();
            if (outerFrame && outerFrame->origSS != dpmiFrame.origSS) {
              // Nested: use the app's stack from the outermost frame
              pmSS = outerFrame->origSS;
              pmESP = outerFrame->origESP;
            }
            // Switch to the chosen stack and restore segment registers
            loadSegment(SS, pmSS);
            loadSegment(DS, dpmiFrame.origDS);
            loadSegment(ES, dpmiFrame.origES);
            uint32_t appEsp = pmESP;
            // Push a 32-bit IRET frame: EFLAGS, CS, EIP
            appEsp -= 4; m_memory.write32(m_cpu.getSegBase(SS) + appEsp, dpmiFrame.origEFLAGS);
            appEsp -= 4; m_memory.write32(m_cpu.getSegBase(SS) + appEsp, dpmiFrame.origCS);
            appEsp -= 4; m_memory.write32(m_cpu.getSegBase(SS) + appEsp, dpmiFrame.origEIP);
            m_cpu.setReg32(ESP, appEsp);
            // Mark the frame so the PM handler's IRET is intercepted
            m_cpu.markDpmiFrameDispatched();
            LOG_CPU("DPMI thunk->PM: vec=0x", std::hex, (int)dpmiFrame.vector,
                    " handler=0x", newCs, ":", newEip,
                    " return=0x", dpmiFrame.origCS, ":", dpmiFrame.origEIP,
                    " SS:ESP=0x", pmSS, ":", appEsp);
            } // isPMHandler
          }
        }
        // DPMI host stack switch restore: if the interrupt dispatch used
        // a reflection stack, restore the caller's full original state.
        // The thunk's IRET may have used the wrong frame size (e.g. 16-bit
        // IRET on a 32-bit frame), corrupting CS:EIP, so we override with
        // the saved originals.
        if (popped && poppedFrame.dpmiStackSwitch) {
          loadSegment(CS, poppedFrame.origCS);
          m_cpu.setEIP(poppedFrame.origEIP);
          m_cpu.setEFLAGS(poppedFrame.origEFLAGS);
          loadSegment(SS, poppedFrame.origSS);
          m_cpu.setReg32(ESP, poppedFrame.origESP);
          loadSegment(DS, poppedFrame.origDS);
          loadSegment(ES, poppedFrame.origES);
          LOG_CPU("DPMI reflect restore: vec=0x", std::hex,
                  (int)poppedFrame.vector,
                  " CS:EIP=0x", poppedFrame.origCS,
                  ":", poppedFrame.origEIP,
                  " SS:ESP=0x", poppedFrame.origSS,
                  ":", poppedFrame.origESP);
        }
    }
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
  case 0x6A: { // PUSH imm8 (sign-extended to operand size)
    int8_t imm8 = static_cast<int8_t>(fetch8());
    if (m_hasPrefix66)
      m_cpu.push32(static_cast<uint32_t>(static_cast<int32_t>(imm8)));
    else
      m_cpu.push16(static_cast<uint16_t>(static_cast<int16_t>(imm8)));
    break;
  }
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
    if (m_hasPrefix66) {
      uint32_t val = m_cpu.getReg32(m.reg);
      writeModRM32(m, val);
    } else {
      uint16_t val = m_cpu.getReg16(m.reg);
      writeModRM16(m, val);
    }
    break;
  }
  case 0x8A: {
    ModRM m = decodeModRM(fetch8());
    m_cpu.setReg8(m.reg, readModRM8(m));
    break;
  }
  case 0x8B: {
    ModRM m = decodeModRM(fetch8());
    if (m_hasPrefix66) {
      uint32_t val = readModRM32(m);
      m_cpu.setReg32(m.reg, val);
    } else {
      uint16_t val = readModRM16(m);
      m_cpu.setReg16(m.reg, val);
    }
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
    uint8_t level = static_cast<uint8_t>(fetch8() & 0x1F);

    if (m_hasPrefix66) {
      uint32_t frameTemp = m_cpu.getReg32(ESP);
      m_cpu.push32(m_cpu.getReg32(EBP));
      uint32_t framePtr = m_cpu.getReg32(ESP);

      if (level > 0) {
        uint32_t ebp = m_cpu.getReg32(EBP);
        for (uint8_t i = 1; i < level; ++i) {
          ebp -= 4;
          m_cpu.push32(m_memory.read32(m_segBase[SS] + ebp));
        }
        m_cpu.push32(frameTemp);
      }

      m_cpu.setReg32(EBP, framePtr);
      m_cpu.setReg32(ESP, m_cpu.getReg32(ESP) - size);
    } else {
      uint16_t frameTemp = m_cpu.getReg16(SP);
      m_cpu.push16(m_cpu.getReg16(BP));
      uint16_t framePtr = m_cpu.getReg16(SP);

      if (level > 0) {
        uint16_t bp = m_cpu.getReg16(BP);
        for (uint8_t i = 1; i < level; ++i) {
          bp = static_cast<uint16_t>(bp - 2);
          m_cpu.push16(m_memory.read16(m_segBase[SS] + bp));
        }
        m_cpu.push16(frameTemp);
      }

      m_cpu.setReg16(BP, framePtr);
      m_cpu.setReg16(SP, static_cast<uint16_t>(m_cpu.getReg16(SP) - size));
    }
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
      uint32_t dstPhys = m_segBase[ES] + dstOff;
      uint32_t srcPhys = m_segBase[srcSeg] + srcOff;
      uint32_t srcVal = m_memory.read32(srcPhys);
      m_memory.write32(dstPhys, srcVal);
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
    uint32_t eip = m_hasPrefix66 ? m_cpu.pop32() : m_cpu.pop16();
    m_cpu.setEIP(eip);
    break;
  }
  case 0xC2: { // RET imm16 Near
    uint16_t imm = fetch16();
    uint32_t eip = m_hasPrefix66 ? m_cpu.pop32() : m_cpu.pop16();
    if (m_cpu.is32BitStack())
      m_cpu.setReg32(ESP, m_cpu.getReg32(ESP) + imm);
    else
      m_cpu.setReg16(SP, static_cast<uint16_t>(m_cpu.getReg16(SP) + imm));
    m_cpu.setEIP(eip);
    break;
  }
  case 0xCB: { // RETF Far
    uint32_t eip;
    uint16_t cs;
    if (m_hasPrefix66) {
      eip = m_cpu.pop32();
      cs = static_cast<uint16_t>(m_cpu.pop32() & 0xFFFF);
    } else {
      eip = m_cpu.pop16();
      cs = m_cpu.pop16();
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
      eip = m_cpu.pop32();
      cs = static_cast<uint16_t>(m_cpu.pop32() & 0xFFFF);
    } else {
      eip = m_cpu.pop16();
      cs = m_cpu.pop16();
    }
    if (m_cpu.is32BitStack())
      m_cpu.setReg32(ESP, m_cpu.getReg32(ESP) + imm);
    else
      m_cpu.setReg16(SP, static_cast<uint16_t>(m_cpu.getReg16(SP) + imm));
    loadSegment(CS, cs);
    m_cpu.setEIP(eip);
    break;
  }
  case 0xE8: { // CALL rel
    if (m_hasPrefix66) {
      int32_t rel = static_cast<int32_t>(fetch32());
      uint32_t retAddr = m_cpu.getEIP();
      m_cpu.push32(retAddr);
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
    f1_count++;
    break;
  }
  case 0xF4: {
    uint16_t haltCs = m_cpu.getSegReg(CS);
    uint32_t haltEip = m_cpu.getEIP() - 1;
    LOG_INFO("HLT encountered at ", std::hex, haltCs, ":", haltEip);
    // DPMI thunk error bailout: if HLT fires inside the thunk code and
    // there's an outermost dpmiStackSwitch frame (the app's DPMI call),
    // the thunk has entered its fatal error path. Restore the app's
    // state from the outermost frame with CF=1 to signal error.
    if (m_cpu.hleStackSize() > 0) {
      auto *outerFrame = m_cpu.peekOutermostDpmiFrame();
      if (outerFrame && outerFrame->origCS != 0 && haltCs != outerFrame->origCS) {
        LOG_DEBUG("DPMI thunk error bailout: restoring to CS:EIP=0x",
                 std::hex, outerFrame->origCS, ":", outerFrame->origEIP,
                 " SS:ESP=0x", outerFrame->origSS, ":", outerFrame->origESP,
                 " origAX=0x", outerFrame->savedEAX,
                 " vec=0x", (int)outerFrame->vector,
                 " hleDepth=", std::dec, m_cpu.hleStackSize());
        loadSegment(CS, outerFrame->origCS);
        m_cpu.setEIP(outerFrame->origEIP);
        uint32_t eflags = outerFrame->origEFLAGS | 1; // Set CF for error
        m_cpu.setEFLAGS(eflags);
        loadSegment(SS, outerFrame->origSS);
        m_cpu.setReg32(ESP, outerFrame->origESP);
        loadSegment(DS, outerFrame->origDS);
        loadSegment(ES, outerFrame->origES);
        // Send EOI for all HW IRQ vectors in the HLE stack so the PIC
        // unblocks future interrupts (especially keyboard IRQ1).
        for (size_t i = 0; i < m_cpu.hleStackSize(); ++i) {
          uint8_t v = m_cpu.hleFrameAt(i).vector;
          bool isHwIrq = (v >= 0x08 && v <= 0x0F) || (v >= 0x70 && v <= 0x77);
          if (isHwIrq) {
            m_bios.sendEOI(v);
          }
        }
        // Clear all HLE frames — the DPMI call is aborted
        while (m_cpu.hleStackSize() > 0)
          m_cpu.popHLEFrame();
      }
    }
    break;
  }

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
    if (m_hasPrefix66) {
      uint8_t bit = m_cpu.getReg32(modrm.reg) & 0x1F;
      uint32_t val = readModRM32(modrm);
      if ((val >> bit) & 1)
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
      else
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_CARRY);
    } else {
      uint8_t bit = m_cpu.getReg16(modrm.reg) & 0x0F;
      uint16_t val = readModRM16(modrm);
      if ((val >> bit) & 1)
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
      else
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_CARRY);
    }
    break;
  }
  case 0xAB: { // BTS r/m, r
    ModRM modrm = decodeModRM(fetch8());
    if (m_hasPrefix66) {
      uint8_t bit = m_cpu.getReg32(modrm.reg) & 0x1F;
      uint32_t val = readModRM32(modrm);
      if ((val >> bit) & 1)
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
      else
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_CARRY);
      writeModRM32(modrm, val | (1u << bit));
    } else {
      uint8_t bit = m_cpu.getReg16(modrm.reg) & 0x0F;
      uint16_t val = readModRM16(modrm);
      if ((val >> bit) & 1)
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
      else
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_CARRY);
      writeModRM16(modrm, val | static_cast<uint16_t>(1u << bit));
    }
    break;
  }
  case 0xB3: { // BTR r/m, r
    ModRM modrm = decodeModRM(fetch8());
    if (m_hasPrefix66) {
      uint8_t bit = m_cpu.getReg32(modrm.reg) & 0x1F;
      uint32_t val = readModRM32(modrm);
      if ((val >> bit) & 1)
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
      else
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_CARRY);
      writeModRM32(modrm, val & ~(1u << bit));
    } else {
      uint8_t bit = m_cpu.getReg16(modrm.reg) & 0x0F;
      uint16_t val = readModRM16(modrm);
      if ((val >> bit) & 1)
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
      else
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_CARRY);
      writeModRM16(modrm, val & ~static_cast<uint16_t>(1u << bit));
    }
    break;
  }
  case 0xBB: { // BTC r/m, r
    ModRM modrm = decodeModRM(fetch8());
    if (m_hasPrefix66) {
      uint8_t bit = m_cpu.getReg32(modrm.reg) & 0x1F;
      uint32_t val = readModRM32(modrm);
      if ((val >> bit) & 1)
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
      else
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_CARRY);
      writeModRM32(modrm, val ^ (1u << bit));
    } else {
      uint8_t bit = m_cpu.getReg16(modrm.reg) & 0x0F;
      uint16_t val = readModRM16(modrm);
      if ((val >> bit) & 1)
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
      else
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_CARRY);
      writeModRM16(modrm, val ^ static_cast<uint16_t>(1u << bit));
    }
    break;
  }
  case 0xBA: { // Group 8: BT/BTS/BTR/BTC r/m, imm8
    ModRM modrm = decodeModRM(fetch8());
    uint8_t imm = fetch8();
    if (m_hasPrefix66) {
      uint8_t bit = imm & 0x1F;
      uint32_t val = readModRM32(modrm);
      bool oldBit = (val >> bit) & 1;
      if (oldBit)
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
      else
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_CARRY);
      switch (modrm.reg) {
      case 4:
        break; // BT  – read only
      case 5:
        writeModRM32(modrm, val | (1u << bit));
        break; // BTS
      case 6:
        writeModRM32(modrm, val & ~(1u << bit));
        break; // BTR
      case 7:
        writeModRM32(modrm, val ^ (1u << bit));
        break; // BTC
      default:
        LOG_WARN("Group 8 (0x0F 0xBA) unknown sub-opcode ", (int)modrm.reg);
        break;
      }
    } else {
      uint8_t bit = imm & 0x0F;
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
      uint32_t newOff = m_memory.read32(addr);
      uint16_t newSel = m_memory.read16(addr + 4);
      m_cpu.setReg32(modrm.reg, newOff);
      loadSegment(SS, newSel);
    } else {
      uint16_t newOff = m_memory.read16(addr);
      uint16_t newSel = m_memory.read16(addr + 2);
      m_cpu.setReg16(modrm.reg, newOff);
      loadSegment(SS, newSel);
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
    m_segBase[SS] = m_cpu.getSegBase(SS);
    uint32_t ssBase = m_segBase[SS];
    uint32_t currentESP = m_cpu.is32BitStack() ? m_cpu.getReg32(ESP) : m_cpu.getReg16(SP);
    uint32_t preCurPhys = ssBase + currentESP;

    auto hfPeek = m_cpu.peekHLEFrameForVector(vector);
    bool hasTrackedFrame = (hfPeek.framePhysAddr != 0);
    bool isChainCall = true;

    if (hasTrackedFrame) {
      isChainCall = (preCurPhys != hfPeek.framePhysAddr);
    }

    // Capture flags at trap entry before any handlers may modify them
    uint32_t entryFlags = m_cpu.getEFLAGS();
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
    }

    // Two cases for returning from the HLE trap:
    // (A) Direct dispatch — triggerInterrupt dispatched directly to our
    //     stub. Pre-handler SP matches the stored HLE frame. Pop the frame
    //     and IRET to the original caller.
    // (B) Chain call — a hooked handler (e.g. DOS/4GW thunk) chained
    //     to our stub via PUSHF+CALL FAR. Pre-handler SP does NOT match
    //     the stored HLE frame. Do inline IRET from the current stack
    //     to return to the chain caller (thunk).
    if (!hasTrackedFrame) {
      isChainCall = true;
    }

    if (isChainCall) {
      bool chainIs32 = m_hasPrefix66;

      if (m_cpu.getCR(0) & 1) { // Protected mode
        auto scoreFrame = [&](uint16_t cs, uint32_t ip, uint32_t flags, bool expect32) -> int {
          if ((cs & ~7) == 0) return -100;
          uint32_t tbl = (cs & 0x04) ? m_cpu.getLDTR().base : m_cpu.getGDTR().base;
          uint32_t dLow = m_memory.read32(tbl + (cs & ~7));
          uint32_t dHigh = m_memory.read32(tbl + (cs & ~7) + 4);
          Descriptor d = decodeDescriptor(dLow, dHigh);
          if (d.isSystem || !(d.type & 0x08) || !d.isPresent) return -100;
          
          int score = 100;
          uint32_t limit = d.limit;
          if (ip > limit && limit > 0) return -200;
          else score += 50;
          
          if (d.is32Bit == expect32) score += 20;
          if ((flags & 0x2A) == 0x02) score += 10;
          return score;
        };

        uint16_t cs16 = m_memory.read16(ssBase + currentESP + 2);
        uint16_t flags16 = m_memory.read16(ssBase + currentESP + 4);
        uint32_t ip16 = m_memory.read16(ssBase + currentESP);

        uint16_t cs32 = m_memory.read16(ssBase + currentESP + 4);
        uint32_t flags32 = m_memory.read32(ssBase + currentESP + 8);
        uint32_t eip32 = m_memory.read32(ssBase + currentESP);

        int score16 = scoreFrame(cs16, ip16, flags16, false);
        int score32 = scoreFrame(cs32, eip32, flags32, true);

        LOG_DEBUG("HLE scoring: vec=0x", std::hex, (int)vector,
                  " score16=", score16, " score32=", score32,
                  " pref32=", chainIs32 ? 1 : 0);

        if (score32 > score16 && score32 >= 0) {
          chainIs32 = true;
        } else if (score16 > score32 && score16 >= 0) {
          chainIs32 = false;
        } else if (score16 >= 0 && score32 >= 0 && score16 == score32) {
          chainIs32 = true;
        }

        LOG_DEBUG("HLE decision: vec=0x", std::hex, (int)vector,
                  " chainIs32=", chainIs32 ? 1 : 0);
      }

      // Capture flags *after* handlers ran so handlers can set CF/other bits
      uint32_t afterFlags = m_cpu.getEFLAGS();
      // Also report entry flags for comparison
      LOG_DEBUG("HLE FLAGS: vec=0x", std::hex, (int)vector,
        " entryFlags=0x", entryFlags, " currentFlags=0x", afterFlags);
      uint32_t mask = 0;
      if (handled) {
        mask = FLAG_CARRY | FLAG_ZERO | FLAG_SIGN | FLAG_OVERFLOW |
               FLAG_AUX | FLAG_PARITY;
        if (vector == 0x31) {
          mask = FLAG_CARRY;
        }
      }
      // DPMI INT 0x31 semantics require the carry bit to be considered for
      // merging into the caller's FLAGS even when the handler path reports
      // 'unhandled' (some DPMI stubs may signal errors by setting CF but
      // still return control). Ensure CF is always merged for vector 0x31.
      if (vector == 0x31)
        mask |= FLAG_CARRY;

      LOG_DEBUG("HLE mask: vec=0x", std::hex, (int)vector,
        " mask=0x", mask, " handled=", handled ? 1 : 0,
        " curFlags=0x", afterFlags);

      if (chainIs32) {
        uint32_t newEip = m_memory.read32(ssBase + currentESP);
        uint16_t newCs = m_memory.read32(ssBase + currentESP + 4) & 0xFFFF;
        uint32_t popFlags = m_memory.read32(ssBase + currentESP + 8);
        // For INT 0x31: prefer handler-updated carry if the handler set it;
        // otherwise preserve the caller's carry (entryFlags). For other
        // vectors, use the flags after handlers ran so handler changes are
        // preserved.
        uint32_t mergeSource32;
        if (vector == 0x31) {
          if ((afterFlags & FLAG_CARRY) != 0)
            mergeSource32 = afterFlags;
          else
            mergeSource32 = entryFlags;
        } else {
          mergeSource32 = afterFlags;
        }
        popFlags = (popFlags & ~mask) | (mergeSource32 & mask);
        LOG_DEBUG("HLE return32: vec=0x", std::hex, (int)vector,
          " newEIP=0x", newEip, " newCS=0x", newCs,
          " popFlags=0x", popFlags,
          " stackIs32=", m_cpu.is32BitStack() ? 1 : 0,
          " preSP=0x", currentESP);
        LOG_DEBUG("HLE setEFLAGS before: vec=0x", std::hex, (int)vector,
              " curEFLAGS=0x", m_cpu.getEFLAGS());
        m_cpu.setEFLAGS(popFlags);
        LOG_DEBUG("HLE setEFLAGS after: vec=0x", std::hex, (int)vector,
              " curEFLAGS=0x", m_cpu.getEFLAGS());
        m_cpu.setEIP(newEip);
        loadSegment(CS, newCs);
        if (m_cpu.is32BitStack()) {
          m_cpu.setReg32(ESP, currentESP + 12);
        } else {
          m_cpu.setReg16(SP, static_cast<uint16_t>(currentESP + 12));
        }
      } else {
        uint16_t newIp = m_memory.read16(ssBase + currentESP);
        uint16_t newCs = m_memory.read16(ssBase + currentESP + 2);
        uint16_t popFlags16 = m_memory.read16(ssBase + currentESP + 4);
        uint16_t mask16 = static_cast<uint16_t>(mask);
        uint32_t mergeSource32_for16;
        if (vector == 0x31) {
          if ((afterFlags & FLAG_CARRY) != 0)
            mergeSource32_for16 = afterFlags;
          else
            mergeSource32_for16 = entryFlags;
        } else {
          mergeSource32_for16 = afterFlags;
        }
        uint16_t mergeSource16 = static_cast<uint16_t>(mergeSource32_for16);
        popFlags16 = (popFlags16 & ~mask16) | (mergeSource16 & mask16);
        uint32_t merged16 = (mergeSource32_for16 & 0xFFFF0000) | popFlags16;
          if (vector == 0x21) {
            LOG_INFO("HLE return16: vec=0x21 newIp=0x", std::hex, newIp,
                     " newCs=0x", newCs, " popFlags16=0x", popFlags16,
                     " merged16=0x", merged16, " preSP=0x", currentESP);
          }
        LOG_DEBUG("HLE setEFLAGS before: vec=0x", std::hex, (int)vector,
            " curEFLAGS=0x", m_cpu.getEFLAGS());
        m_cpu.setEFLAGS(merged16);
          if (vector == 0x21) {
            LOG_INFO("HLE setEFLAGS after: vec=0x21 curEFLAGS=0x", std::hex, m_cpu.getEFLAGS());
          }
        LOG_DEBUG("HLE setEFLAGS after: vec=0x", std::hex, (int)vector,
            " curEFLAGS=0x", m_cpu.getEFLAGS());
        m_cpu.setEIP(newIp);
        loadSegment(CS, newCs);
        if (m_cpu.is32BitStack())
          m_cpu.setReg32(ESP, currentESP + 6);
        else
          m_cpu.setReg16(SP, static_cast<uint16_t>(currentESP + 6));
      }

        if (handled && hasTrackedFrame) {
          bool popped = m_cpu.popHLEFrameByPhysAddr(hfPeek.framePhysAddr);
          LOG_DEBUG("HLE popByPhys: vec=0x", std::hex, (int)vector,
                    " phys=0x", hfPeek.framePhysAddr,
                    " popped=", popped ? 1 : 0,
                    " newHleSize=", m_cpu.hleStackSize());
        }
    } else {
      auto hf = m_cpu.popHLEFrameForVector(vector);
      LOG_DEBUG("HLE popForVector: vec=0x", std::hex, (int)vector,
            " returnedPhys=0x", hf.framePhysAddr,
            " hfSP=0x", hf.frameSP,
            " hfIs32=", hf.is32 ? 1 : 0,
            " hfStackIs32=", hf.stackIs32 ? 1 : 0,
            " newHleSize=", m_cpu.hleStackSize());

      if (hf.framePhysAddr != 0) {
        uint32_t frameAddr = hf.framePhysAddr;
        uint32_t afterFlags = m_cpu.getEFLAGS();
        uint32_t mask = 0;
        if (handled) {
          mask = FLAG_CARRY | FLAG_ZERO | FLAG_SIGN | FLAG_OVERFLOW |
                 FLAG_AUX | FLAG_PARITY;
        }

        if (hf.is32) {
          uint32_t newEip = m_memory.read32(frameAddr);
          uint16_t newCs = m_memory.read32(frameAddr + 4) & 0xFFFF;
          uint32_t popFlags = m_memory.read32(frameAddr + 8);
          uint32_t mergeSrc;
          if (vector == 0x31) {
            if ((afterFlags & FLAG_CARRY) != 0)
              mergeSrc = afterFlags;
            else
              mergeSrc = entryFlags;
          } else {
            mergeSrc = afterFlags;
          }
          popFlags = (popFlags & ~mask) | (mergeSrc & mask);
          LOG_DEBUG("HLE setEFLAGS before: hf.vec=0x", std::hex, (int)vector,
                    " curEFLAGS=0x", afterFlags, " framePhys=0x", frameAddr);
          m_cpu.setEFLAGS(popFlags);
          LOG_DEBUG("HLE setEFLAGS after: hf.vec=0x", std::hex, (int)vector,
                    " curEFLAGS=0x", m_cpu.getEFLAGS(), " framePhys=0x", frameAddr);
          m_cpu.setEIP(newEip);
          loadSegment(CS, newCs);
          if (hf.stackIs32)
            m_cpu.setReg32(ESP, hf.frameSP + 12);
          else
            m_cpu.setReg16(SP, static_cast<uint16_t>(hf.frameSP + 12));
        } else {
          uint16_t newIp = m_memory.read16(frameAddr);
          uint16_t newCs = m_memory.read16(frameAddr + 2);
          uint16_t popFlags = m_memory.read16(frameAddr + 4);
          uint32_t mergeSrc32;
          if (vector == 0x31) {
            if ((afterFlags & FLAG_CARRY) != 0)
              mergeSrc32 = afterFlags;
            else
              mergeSrc32 = entryFlags;
          } else {
            mergeSrc32 = afterFlags;
          }
          uint16_t mergeSrc16 = static_cast<uint16_t>(mergeSrc32);
          popFlags = (static_cast<uint16_t>(popFlags) & static_cast<uint16_t>(~mask)) | (mergeSrc16 & static_cast<uint16_t>(mask));
          uint32_t merged = (mergeSrc32 & 0xFFFF0000) | popFlags;
          LOG_DEBUG("HLE setEFLAGS before: hf.vec=0x", std::hex, (int)vector,
                    " curEFLAGS=0x", m_cpu.getEFLAGS(), " framePhys=0x", frameAddr);
          m_cpu.setEFLAGS(merged);
          LOG_DEBUG("HLE setEFLAGS after: hf.vec=0x", std::hex, (int)vector,
                    " curEFLAGS=0x", m_cpu.getEFLAGS(), " framePhys=0x", frameAddr);
          m_cpu.setEIP(newIp);
          loadSegment(CS, newCs);
          m_cpu.setReg16(SP, static_cast<uint16_t>(hf.frameSP + 6));
        }
      }
    }
    break;
  }

  case 0xA2: { // CPUID
    uint32_t leaf = m_cpu.getReg32(EAX);
    switch (leaf) {
    case 0: // Highest function + vendor ID
      m_cpu.setReg32(EAX, 1);          // Max supported leaf
      m_cpu.setReg32(EBX, 0x756E6547); // "Genu"
      m_cpu.setReg32(EDX, 0x49656E69); // "ineI"
      m_cpu.setReg32(ECX, 0x6C65746E); // "ntel"
      break;
    case 1: // Processor info + features
      // Family 4 (486), Model 0, Stepping 0
      m_cpu.setReg32(EAX, 0x00000400);
      m_cpu.setReg32(EBX, 0);
      m_cpu.setReg32(ECX, 0);
      m_cpu.setReg32(EDX, 0x00000001); // FPU on-chip
      break;
    default:
      m_cpu.setReg32(EAX, 0);
      m_cpu.setReg32(EBX, 0);
      m_cpu.setReg32(ECX, 0);
      m_cpu.setReg32(EDX, 0);
      break;
    }
    break;
  }

  default:
    LOG_WARN("0x0F 0x", std::hex, static_cast<int>(opcode),
             " not implemented. Generating #UD INT 6.");
    {
      static int s_udCount = 0;
      if (s_udCount < 1) {
        ++s_udCount;
        uint32_t csBase = m_cpu.getSegBase(CS);
        uint32_t physAddr = csBase + m_instrStartEIP;
        // Read CS descriptor from LDT
        uint16_t sel = m_cpu.getSegReg(CS);
        bool isLDT2 = (sel & 4) != 0;
        uint16_t idx2 = sel >> 3;
        uint32_t descAddr2 = isLDT2 ? m_cpu.getLDTR().base + idx2 * 8
                                     : m_cpu.getGDTR().base + idx2 * 8;
        uint32_t dLow2 = m_memory.read32(descAddr2);
        uint32_t dHigh2 = m_memory.read32(descAddr2 + 4);
        uint32_t descBase2 = (dLow2 >> 16) | ((dHigh2 & 0xFF) << 16) | (dHigh2 & 0xFF000000);
        LOG_ERROR("UD-DIAG: CS=0x", std::hex, sel,
                  " cached_base=0x", csBase,
                  " desc_base=0x", descBase2,
                  " EIP=0x", m_instrStartEIP);
        char buf2[256];
        int n2 = snprintf(buf2, sizeof(buf2), "  bytes @cached+EIP=0x%08X:", (unsigned)physAddr);
        for (int bi = 0; bi < 16; ++bi) {
          if ((size_t)n2 < sizeof(buf2)) {
            int ret = snprintf(buf2 + n2, sizeof(buf2) - n2, " %02X", m_memory.read8(physAddr + bi));
            if (ret > 0) n2 += ret;
          }
        }
        LOG_ERROR(buf2);
        n2 = snprintf(buf2, sizeof(buf2), "  bytes @desc+EIP=0x%08X:", (unsigned)(descBase2 + m_instrStartEIP));
        for (int bi = 0; bi < 16; ++bi) {
          if ((size_t)n2 < sizeof(buf2)) {
            int ret = snprintf(buf2 + n2, sizeof(buf2) - n2, " %02X", m_memory.read8(descBase2 + m_instrStartEIP + bi));
            if (ret > 0) n2 += ret;
          }
        }
        LOG_ERROR(buf2);
      }
    }
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
  bool clearIFOnEntry = true;
  constexpr uint32_t clearTFMask = ~fador::cpu::FLAG_TRAP;

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

    LOG_DEBUG("PM INT 0x", std::hex, (int)vector, " IDT sel: 0x", cs,
              " off: 0x", eip32, " size: ", use32 ? 32 : 16, " type: 0x",
              (int)type);

    uint8_t newCpl = cs & 3;

    // Self-referential interrupt guard: if the interrupt fires from within
    // the DPMI thunk code (oldCs == IDT handler CS) and the vector is a
    // CPU exception (0-31), the thunk has entered its internal error/halt
    // path.  On real hardware the ring-0 DPMI host would catch these;
    // reflecting them back to the same handler creates infinite loops.
    if (oldCs == cs && vector < 32) {
      return;
    }

    // Privilege change check
    bool privChange = (newCpl < oldCpl) || (m_cpu.getEFLAGS() & 0x00020000);

    m_cpu.pushHLEFrame(use32, vector);

    // HLE check: try handling the interrupt directly via HLE before
    // dispatching to a hooked IDT entry (thunk). This avoids the thunk
    // pushing onto its internal transfer stack, which would never get
    // popped because 0F FF IRET simulation bypasses the thunk's exit.
    //
    // Try HLE only if the IDT entry is unhooked (our original stub).
    // When DOS/4GW hooks vectors, it becomes the dispatcher and will
    // chain to our stubs when needed.
    bool isOrig = false;
    if (!(m_cpu.getEFLAGS() & 0x00020000)) {
      isOrig = m_bios.isOriginalIVT(vector, cs, eip32);
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

    // Capture app-level PM vectors: when INT 31h is dispatched through a
    // hooked vector (DOS extender thunk) and AX=0205h, the app is setting
    // a PM interrupt handler. The DOS extender will intercept this and
    // never forward the original CX:EDX to the DPMI host. Capture them
    // here so we can invoke the app's handler from HLE.
    if (vector == 0x31 && m_cpu.getReg16(AX) == 0x0205) {
      uint8_t targetVec = m_cpu.getReg8(BL);
      uint16_t targetSel = m_cpu.getReg16(CX);
      uint32_t targetOff = m_cpu.getReg32(EDX);
#if FADOR_ENABLE_DEBUG_DIAGNOSTICS
      static int s_appVecLog = 0;
      if (s_appVecLog < 300) {
        ++s_appVecLog;
        LOG_INFO("APP-PM-VEC #", std::dec, s_appVecLog,
                 ": vec=0x", std::hex, static_cast<uint32_t>(targetVec),
                 " CX=", targetSel, " EDX=", targetOff,
                 " callerCS=", oldCs,
                 " idtCS=", cs,
                 " isOrig=", isOrig ? 1 : 0);
      }
#endif
      // Capture if caller is a 32-bit flat code segment (the app itself),
      // not the thunk (0x8F) or DOS/4GW internal code (0x57, 0x0F).
      // The app uses its own code selector (e.g., 0x016F for DOOM).
      if (oldCs != 0x8F && oldCs != 0x57 && oldCs != 0x0F && oldCs != 0x08) {
        m_appPMVectors[targetVec].selector = targetSel;
        m_appPMVectors[targetVec].offset = targetOff;
        m_appPMVectors[targetVec].valid = true;
#if FADOR_ENABLE_DEBUG_DIAGNOSTICS
        LOG_INFO("APP-PM-VEC-SAVED: vec=0x", std::hex,
                 static_cast<uint32_t>(targetVec),
                 " -> ", targetSel, ":", targetOff,
                 " callerCS=", oldCs);
#endif
      }
    }

    // DPMI host stack switch simulation:
    // On real hardware, the DPMI host intercepts interrupts at ring 0
    // and reflects to the client's thunk handler with a host-managed
    // stack (ESP < 0x10000). Without this, DOS/4GW's 16-bit thunk code
    // truncates ESP > 0xFFFF during its SP save/restore, corrupting the
    // return path. We switch to a "reflection stack" at a low physical
    // address before pushing the IRET frame.
    //
    // For ALL hooked vectors (isOrig=false), save the interrupted
    // CS:EIP so the IRET handler can match this frame even when the
    // thunk rearranges its stack (physical-address matching fails).
    if (!isOrig) {
      auto &frame = m_cpu.lastHLEFrameMut();
      frame.origCS = oldCs;
      frame.origEIP = m_cpu.getEIP();
      frame.savedEAX = m_cpu.getReg32(EAX); // Save for bailout diagnostics
      if (!(m_cpu.getEFLAGS() & 0x00020000) && oldEsp > 0xFFFF) {
        frame.dpmiStackSwitch = true;
        frame.origESP = oldEsp;
        frame.origSS = oldSs;
        frame.origEFLAGS = m_cpu.getEFLAGS();
        frame.origDS = m_cpu.getSegReg(DS);
        frame.origES = m_cpu.getSegReg(ES);
        // DOS/4GW thunk expects to run on its own host stack (SS=0xAF)
        // with the app's SS:ESP pushed as part of a privilege-change
        // IRET frame. On real hardware the DPMI host does this at
        // ring 0; we emulate it here for software INTs too.
        if (oldSs == 0xAF) {
          // Already on host stack — keep current ESP.
        } else {
          uint32_t dsAFBase = 0x1197C0; // sel 0xAF base
          uint32_t hostSP = m_memory.read32(dsAFBase + 0x0A42);
          if (hostSP == 0 || hostSP > 0xFFF0) hostSP = 0x4800;
          loadSegment(cpu::SS, 0xAF);
          m_cpu.setReg32(ESP, hostSP);
          privChange = true;
        }
        LOG_DEBUG("DPMI reflect: vec=0x", std::hex, (int)vector,
                  " origSS=0x", oldSs, " origESP=0x", oldEsp,
                  " newSS:ESP=", m_cpu.getSegReg(SS), ":",
                  m_cpu.getReg32(ESP));
      }
    }

    // Hardware-compatible entry order: push original EFLAGS first,
    // then update IF/TF for the handler context.
    clearIFOnEntry = isInterruptGate;

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

  uint32_t entryFlags = m_cpu.getEFLAGS() & clearTFMask;
  if (clearIFOnEntry) {
    entryFlags &= ~fador::cpu::FLAG_INTERRUPT;
  }
  m_cpu.setEFLAGS(entryFlags);
  loadSegment(CS, cs);
  m_cpu.setEIP(eip32);
}

void InstructionDecoder::injectHardwareInterrupt(uint8_t vector) {
  syncSegmentCacheIfNeeded();

  // HLE fast-path: if the BIOS can handle this vector entirely in HLE,
  // just call the handler and skip the full interrupt dispatch.
  // For hooked vectors (like INT 8 under DOS/4GW), we MUST dispatch
  // through the thunk so the application's handler runs and updates
  // its own internal variables (e.g. DOOM's ticcount).
  // Exception: if the CPU is ALREADY executing inside the thunk's code
  // segment (e.g. during a DOS INT 21h call dispatched through DOS/4GW),
  // nesting another interrupt through the same thunk entry corrupts its
  // internal state. In that case, fall back to HLE.
  {
    bool useHLE = false;
    if (m_cpu.getCR(0) & 1) {
      uint32_t idtBase = m_cpu.getIDTR().base;
      uint32_t entryAddr = idtBase + vector * 8;
      uint32_t low = m_memory.read32(entryAddr);
      uint32_t high = m_memory.read32(entryAddr + 4);
      uint16_t idtSel = (low >> 16) & 0xFFFF;
      uint32_t idtOff = (low & 0xFFFF) | (high & 0xFFFF0000);
      useHLE = (idtSel == 0x08) && (idtOff == (0xF0100u + vector * 4));

      // If the IDT vector points to a 16-bit (D=0) code segment — i.e. a
      // DOS extender thunk — use HLE for vectors that have HLE handlers
      // (e.g. INT 8 timer). For other hardware IRQs (e.g. INT 9 keyboard),
      // let them dispatch through the thunk so the application's ISR runs.
      // The thunk's 16-bit dispatch code works correctly in our CPU because
      // triggerInterrupt handles 16-bit gate frames properly.
      if (!useHLE) {
        uint32_t descTableBase;
        if (idtSel & 0x04) {
          descTableBase = m_cpu.getLDTR().base;
        } else {
          descTableBase = m_cpu.getGDTR().base;
        }
        uint32_t descAddr = descTableBase + (idtSel & 0xFFF8);
        auto idtDesc = decodeDescriptor(
            m_memory.read32(descAddr), m_memory.read32(descAddr + 4));
        if (!idtDesc.is32Bit) {
          // Only force HLE for vectors where BIOS has an HLE handler.
          // INT 8 (timer): has HLE + ticcount increment.
          // INT 9 (keyboard) and others: no HLE, let thunk dispatch.
          if (vector == 0x08) {
            useHLE = true;
          }
        }
      }
    } else {
      // In real mode, only hardware IRQs with explicit BIOS HLE behavior
      // should short-circuit here. Other IRQs must dispatch through the IVT
      // so guest-installed handlers (or the BIOS machine-code handler) run.
      useHLE = (vector == 0x08);
    }
    if (useHLE && m_bios.handleInterrupt(vector)) {
      // After HLE, if the vector is hooked by a DOS extender thunk and we're
      // in 32-bit PM code, also dispatch to the app's registered PM callback
      // (if any).  The Watcom C/C++ RTL maintains an internal ISR table;
      // slot 0 at 0x26D3E0 holds the handler for _dos_setvect-installed ISRs.
      // We emulate the handler's effect directly (HLE) rather than jumping
      // into the app code, because the DOS/4GW thunk infra-structure is not
      // functional in our HLE environment.
      //
      // The Watcom-compiled I_TimerISR typically compiles to:
      //   PUSH EDX; MOV EDX,[addr]; INC EDX; ... MOV [addr],EDX; POP EDX; RET
      // We detect the MOV EDX,[imm32] pattern (opcode 8B 15) to extract the
      // counter address, then increment the dword at that address.
      if ((m_cpu.getCR(0) & 1) && m_cpu.is32BitCode() && vector == 0x08) {
        uint32_t appHandler = m_memory.read32(0x26D3E0);
        if (appHandler != 0) {
          // Look for: 8B 15 XX XX XX XX = MOV EDX, [imm32]
          // Handler might start with PUSH EDX (0x52), so check offset 0 or 1.
          uint32_t scan = appHandler;
          if (m_memory.read8(scan) == 0x52) scan++; // skip PUSH EDX
          if (m_memory.read8(scan) == 0x8B && m_memory.read8(scan + 1) == 0x15) {
            uint32_t counterAddr = m_memory.read32(scan + 2);
            if (counterAddr < fador::memory::MemoryBus::MEMORY_SIZE - 3) {
              uint32_t val = m_memory.read32(counterAddr);
              m_memory.write32(counterAddr, val + 1);
            }
          }
        }
      }
      return;
    }
    if (useHLE) {
      // The HLE path was selected (INT vector has an HLE-only handler like
      // INT 8 timer) but handleInterrupt returned false. Send EOI as safety.
      m_bios.sendEOI(vector);
      return;
    }
  }

  uint16_t cs;
  uint32_t eip32;
  bool use32 = false;
  uint16_t oldCs = m_cpu.getSegReg(CS);
  uint16_t oldSs = m_cpu.getSegReg(SS);
  uint32_t oldEsp = m_cpu.getReg32(ESP);
  uint8_t oldCpl = oldCs & 3;

  bool clearIFOnEntry = true;
  constexpr uint32_t clearTFMask = ~fador::cpu::FLAG_TRAP;

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
    clearIFOnEntry = isInterruptGate;

    bool privChange = (newCpl < oldCpl) || (m_cpu.getEFLAGS() & 0x00020000);
    m_cpu.pushHLEFrame(use32, vector);

    // DPMI host stack switch simulation:
    // On real hardware, the DPMI host intercepts hardware interrupts at
    // ring 0, switches to its internal stack (SS=host_SS), and reflects
    // the interrupt to the client's PM handler (thunk). The thunk expects
    // SS to be the DOS extender's data segment (sel 0xAF for DOS/4GW)
    // with a stack pointer managed in its internal data at DS:0x0A42.
    // Without this, the thunk's 16-bit code operating on the app's
    // 32-bit stack (SS=0x177, ESP > 0xFFFF) corrupts state.
    // We switch SS to 0xAF and ESP to the DOS/4GW internal stack pointer.
    //
    // IMPORTANT: If the CPU is already on SS=0xAF (i.e. the interrupt
    // fired inside the thunk itself), do NOT switch to hostSP — the
    // thunk is already using SS:AF and hostSP may alias stack space
    // currently in use, causing the RM handler to overwrite the thunk's
    // local data. In that case, push the interrupt frame below the
    // current ESP like a normal same-privilege interrupt.
    bool isOrig = (cs == 0x08 && eip32 == (0xF0100u + vector * 4));
    if (!isOrig && !(m_cpu.getEFLAGS() & 0x00020000)) {
      auto &frame = m_cpu.lastHLEFrameMut();
      frame.dpmiStackSwitch = true;
      frame.origESP = oldEsp;
      frame.origSS = oldSs;
      frame.origEIP = m_cpu.getEIP();
      frame.origCS = oldCs;
      frame.origEFLAGS = m_cpu.getEFLAGS();
      frame.origDS = m_cpu.getSegReg(DS);
      frame.origES = m_cpu.getSegReg(ES);

      if (oldSs == 0xAF) {
        // Already on the host stack — don't switch.  Keep the current ESP
        // so the interrupt frame is pushed below the thunk's live data.
        // Don't set privChange: same-privilege interrupts don't push SS:ESP.
      } else {
        // Transition from PM app stack → host stack.
        uint32_t dsAFBase = 0x1197C0; // sel 0xAF base
        uint32_t hostSP = m_memory.read32(dsAFBase + 0x0A42);
        if (hostSP == 0 || hostSP > 0xFFF0) hostSP = 0x4800; // fallback
        loadSegment(cpu::SS, 0xAF);
        m_cpu.setReg32(ESP, hostSP);
        // Force privilege-change frame: on real hardware, the HW interrupt
        // always transitions ring 3 → ring 0, so the CPU always pushes
        // SS:ESP.  The thunk expects a full 5-word frame (SS, ESP, FLAGS,
        // CS, EIP).  Without this, the thunk reads garbage for SS:ESP and
        // restores the wrong stack when it IRETs to the PM handler.
        privChange = true;
      }
    }

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

  uint32_t entryFlags = m_cpu.getEFLAGS() & clearTFMask;
  if (clearIFOnEntry) {
    entryFlags &= ~fador::cpu::FLAG_INTERRUPT;
  }
  m_cpu.setEFLAGS(entryFlags);
  loadSegment(CS, cs);

  m_cpu.setEIP(eip32);
}

void InstructionDecoder::syncSegments() {
  uint8_t dirtyMask = m_cpu.getDirtySegmentMask();
  for (int i = 0; i < 6; ++i) {
    if (dirtyMask & (1u << i)) {
      m_cpu.loadSegment(static_cast<SegRegIndex>(i), m_cpu.getSegReg(i));
    }
    m_segBase[i] = m_cpu.getSegBase(static_cast<SegRegIndex>(i));
  }
  m_cachedSegStateVersion = m_cpu.getSegmentStateVersion();
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
  m_cpu.loadSegment(seg, selector);
  syncSegments();
}

} // namespace fador::cpu