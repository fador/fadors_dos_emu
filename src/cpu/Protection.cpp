// 386 protection model for guest-owned protected mode (see
// CPU::nativeProtection()): privilege levels, call gates, TSS stack
// switches, inter-privilege RETF/IRET, virtual-8086 entry/exit and CPU
// exceptions with error codes.
//
// The emulator's own DPMI host keeps using the HLE paths in
// InstructionDecoder.cpp; everything here is only reached when a DOS
// extender switched to protected mode itself (raw mode or VCPI).
#include "InstructionDecoder.hpp"
#include "../utils/Logger.hpp"

namespace fador::cpu {

namespace {
constexpr uint32_t kFlagTF = 0x00000100;
constexpr uint32_t kFlagIF = 0x00000200;
constexpr uint32_t kFlagIOPL = 0x00003000;
constexpr uint32_t kFlagNT = 0x00004000;
constexpr uint32_t kFlagRF = 0x00010000;
constexpr uint32_t kFlagVM = 0x00020000;

// Exceptions that push an error code in protected mode.
bool hasErrorCode(uint8_t vector) {
  return vector == 8 || (vector >= 10 && vector <= 14) || vector == 17;
}
} // namespace

uint8_t InstructionDecoder::currentCPL() const {
  if (!(m_cpu.getCR(0) & 1))
    return 0;
  if (m_cpu.getEFLAGS() & kFlagVM)
    return 3;
  return static_cast<uint8_t>(m_cpu.getSegReg(CS) & 3);
}

bool InstructionDecoder::readDescriptor(uint16_t selector, uint32_t &low,
                                        uint32_t &high) const {
  if ((selector & 0xFFFC) == 0)
    return false;
  const DescriptorRegister table =
      (selector & 4) ? m_cpu.getLDTR() : m_cpu.getGDTR();
  const uint32_t offset = selector & 0xFFF8u;
  if (offset + 7 > table.limit)
    return false;
  low = m_memory.lread32(table.base + offset);
  high = m_memory.lread32(table.base + offset + 4);
  return true;
}

// Ring `dpl` stack from the current TSS (32-bit TSS: SSn at 8n+8, ESPn at
// 8n+4; 16-bit TSS: SSn at 4n+4, SPn at 4n+2).
bool InstructionDecoder::tssStack(uint8_t dpl, uint16_t &ss,
                                  uint32_t &esp) const {
  const DescriptorRegister tr = m_cpu.getTR();
  uint32_t low = 0, high = 0;
  bool is32 = true;
  if (readDescriptor(m_cpu.getTRSelector(), low, high)) {
    const uint8_t type = (high >> 8) & 0x0F;
    is32 = (type & 0x08) != 0;
  }
  if (tr.base == 0 && tr.limit == 0)
    return false;
  if (is32) {
    esp = m_memory.lread32(tr.base + 4 + dpl * 8u);
    ss = m_memory.lread16(tr.base + 8 + dpl * 8u);
  } else {
    esp = m_memory.lread16(tr.base + 2 + dpl * 4u);
    ss = m_memory.lread16(tr.base + 4 + dpl * 4u);
  }
  return true;
}

void InstructionDecoder::setStackPointer(uint32_t value) {
  if (m_cpu.is32BitStack())
    m_cpu.setReg32(ESP, value);
  else
    m_cpu.setReg16(SP, static_cast<uint16_t>(value));
}

void InstructionDecoder::pushSized(bool is32, uint32_t value) {
  if (is32)
    m_cpu.push32(value);
  else
    m_cpu.push16(static_cast<uint16_t>(value));
}

uint32_t InstructionDecoder::popSized(bool is32) {
  return is32 ? m_cpu.pop32() : m_cpu.pop16();
}

void InstructionDecoder::raiseException(uint8_t vector, uint32_t errorCode) {
  m_cpu.setEIP(m_instrStartEIP);
  if (m_cpu.nativeProtection()) {
    nativeInterrupt(vector, false, hasErrorCode(vector), errorCode);
  } else {
    triggerInterrupt(vector);
  }
}

void InstructionDecoder::nativeInterrupt(uint8_t vector, bool software,
                                         bool pushError, uint32_t errorCode) {
  const uint32_t oldFlags = m_cpu.getEFLAGS();
  const bool fromV86 = (oldFlags & kFlagVM) != 0;
  const uint8_t cpl = currentCPL();

  // A software INT in virtual-8086 mode with IOPL < 3 is a #GP to the
  // monitor, which then emulates it.
  if (software && fromV86 && (oldFlags & kFlagIOPL) != kFlagIOPL) {
    software = false;
    pushError = true;
    errorCode = 0;
    vector = 13;
  }

  const DescriptorRegister idtr = m_cpu.getIDTR();
  if (vector * 8u + 7 > idtr.limit) {
    LOG_WARN("PM: vector 0x", std::hex, static_cast<int>(vector),
             " is beyond the IDT limit 0x", idtr.limit);
    return;
  }
  const uint32_t low = m_memory.lread32(idtr.base + vector * 8u);
  const uint32_t high = m_memory.lread32(idtr.base + vector * 8u + 4);
  const uint8_t gateType = (high >> 8) & 0x0F;
  const uint8_t gateDpl = (high >> 13) & 3;
  if (!(high & 0x8000)) {
    LOG_WARN("PM: IDT gate 0x", std::hex, static_cast<int>(vector),
             " not present");
    return;
  }
  if (gateType == 0x5) {
    LOG_WARN("PM: task gates are not supported (vector 0x", std::hex,
             static_cast<int>(vector), ")");
    return;
  }
  if (software && gateDpl < cpl && !fromV86) {
    // INT n may only use gates the caller is allowed to see.
    nativeInterrupt(13, false, true, vector * 8u + 2);
    return;
  }
  const bool gate32 = (gateType & 0x08) != 0;
  const bool interruptGate = (gateType & 0x01) == 0;
  const uint16_t target = static_cast<uint16_t>(low >> 16);
  const uint32_t offset =
      (low & 0xFFFF) | (gate32 ? (high & 0xFFFF0000u) : 0u);

  uint32_t tlow = 0, thigh = 0;
  if (!readDescriptor(target, tlow, thigh)) {
    LOG_WARN("PM: IDT gate 0x", std::hex, static_cast<int>(vector),
             " has an invalid code selector 0x", target);
    return;
  }
  const uint8_t targetDpl = (thigh >> 13) & 3;
  const bool conforming = (thigh & 0x0400) != 0;
  const uint8_t newCpl = (conforming || targetDpl >= cpl) ? cpl : targetDpl;

  const uint16_t oldCS = m_cpu.getSegReg(CS);
  const uint32_t oldEIP = m_cpu.getEIP();
  const uint16_t oldSS = m_cpu.getSegReg(SS);
  const uint32_t oldESP = m_cpu.getReg32(ESP);

  const bool stackSwitch = newCpl < cpl || fromV86;
  if (stackSwitch) {
    uint16_t ss = 0;
    uint32_t esp = 0;
    if (!tssStack(newCpl, ss, esp)) {
      LOG_WARN("PM: privilege change without a TSS (vector 0x", std::hex,
               static_cast<int>(vector), ")");
      return;
    }
    // Leave V86 before touching selectors so they decode as descriptors.
    m_cpu.setEFLAGS(oldFlags & ~kFlagVM);
    loadSegment(SS, ss);
    m_cpu.setReg32(ESP, esp);
    if (fromV86) {
      pushSized(gate32, m_cpu.getSegReg(GS));
      pushSized(gate32, m_cpu.getSegReg(FS));
      pushSized(gate32, m_cpu.getSegReg(DS));
      pushSized(gate32, m_cpu.getSegReg(ES));
      loadSegment(GS, 0);
      loadSegment(FS, 0);
      loadSegment(DS, 0);
      loadSegment(ES, 0);
    }
    pushSized(gate32, oldSS);
    pushSized(gate32, gate32 ? oldESP : (oldESP & 0xFFFF));
  }
  pushSized(gate32, oldFlags);
  pushSized(gate32, oldCS);
  pushSized(gate32, oldEIP);
  if (pushError)
    pushSized(gate32, errorCode);

  uint32_t flags = oldFlags & ~(kFlagTF | kFlagNT | kFlagVM | kFlagRF);
  if (interruptGate)
    flags &= ~kFlagIF;
  m_cpu.setEFLAGS(flags);
  loadSegment(CS, static_cast<uint16_t>((target & 0xFFFC) | newCpl));
  m_cpu.setEIP(offset);
}

void InstructionDecoder::nativeIret(bool op32) {
  const uint32_t flags = m_cpu.getEFLAGS();
  if (flags & kFlagVM) {
    if ((flags & kFlagIOPL) != kFlagIOPL) {
      raiseException(13, 0);
      return;
    }
    // IOPL 3 V86 IRET behaves like real mode; VM and IOPL stay.
    const uint32_t eip = popSized(op32);
    const uint16_t cs = static_cast<uint16_t>(popSized(op32));
    const uint32_t newFlags = popSized(op32);
    const uint32_t keep = kFlagVM | kFlagIOPL | (op32 ? 0 : 0xFFFF0000u);
    m_cpu.setEFLAGS((flags & keep) | (newFlags & ~keep) | 2);
    loadSegment(CS, cs);
    m_cpu.setEIP(eip & 0xFFFF);
    return;
  }
  if (flags & kFlagNT)
    LOG_WARN("PM: IRET with NT set (task return) is not supported");

  const uint8_t cpl = currentCPL();
  const uint32_t eip = popSized(op32);
  const uint16_t cs = static_cast<uint16_t>(popSized(op32));
  const uint32_t newFlags = popSized(op32);

  if (op32 && (newFlags & kFlagVM) && cpl == 0) {
    // Return to virtual-8086 mode.
    const uint32_t esp = m_cpu.pop32();
    const uint16_t ss = static_cast<uint16_t>(m_cpu.pop32());
    const uint16_t es = static_cast<uint16_t>(m_cpu.pop32());
    const uint16_t ds = static_cast<uint16_t>(m_cpu.pop32());
    const uint16_t fs = static_cast<uint16_t>(m_cpu.pop32());
    const uint16_t gs = static_cast<uint16_t>(m_cpu.pop32());
    m_cpu.setEFLAGS(newFlags | 2);
    loadSegment(CS, cs);
    loadSegment(SS, ss);
    loadSegment(ES, es);
    loadSegment(DS, ds);
    loadSegment(FS, fs);
    loadSegment(GS, gs);
    m_cpu.setReg32(ESP, esp);
    m_cpu.setEIP(eip & 0xFFFF);
    return;
  }

  // Which flags this privilege level may change.
  uint32_t changeable = 0x00247FD5u & ~kFlagVM; // CF..OF, NT, AC, ID, ...
  changeable |= kFlagRF;
  if (cpl > 0)
    changeable &= ~kFlagIOPL;
  if (cpl > ((flags & kFlagIOPL) >> 12))
    changeable &= ~kFlagIF;
  if (!op32)
    changeable &= 0xFFFF;
  const uint32_t merged = (flags & ~changeable) | (newFlags & changeable) | 2;

  const uint8_t rpl = cs & 3;
  if (rpl > cpl) {
    const uint32_t esp = popSized(op32);
    const uint16_t ss = static_cast<uint16_t>(popSized(op32));
    m_cpu.setEFLAGS(merged);
    loadSegment(CS, cs);
    m_cpu.setEIP(op32 ? eip : (eip & 0xFFFF));
    loadSegment(SS, ss);
    if (op32)
      m_cpu.setReg32(ESP, esp);
    else
      setStackPointer(esp & 0xFFFF);
    return;
  }
  m_cpu.setEFLAGS(merged);
  loadSegment(CS, cs);
  m_cpu.setEIP(op32 ? eip : (eip & 0xFFFF));
}

void InstructionDecoder::nativeFarTransfer(uint16_t selector, uint32_t offset,
                                           bool isCall, bool op32) {
  const uint8_t cpl = currentCPL();
  if (m_cpu.getEFLAGS() & kFlagVM) {
    // V86 far transfers are plain real-mode ones.
    if (isCall) {
      pushSized(op32, m_cpu.getSegReg(CS));
      pushSized(op32, m_cpu.getEIP());
    }
    loadSegment(CS, selector);
    m_cpu.setEIP(offset & 0xFFFF);
    return;
  }
  uint32_t low = 0, high = 0;
  if (!readDescriptor(selector, low, high)) {
    raiseException(13, selector & 0xFFFC);
    return;
  }
  if (!(high & 0x8000)) {
    raiseException(11, selector & 0xFFFC); // #NP
    return;
  }

  if (high & 0x1000) { // code or data segment
    if (!(high & 0x0800)) {
      raiseException(13, selector & 0xFFFC); // data segment
      return;
    }
    if (isCall) {
      pushSized(op32, m_cpu.getSegReg(CS));
      pushSized(op32, m_cpu.getEIP());
    }
    loadSegment(CS, static_cast<uint16_t>((selector & 0xFFFC) | cpl));
    m_cpu.setEIP(op32 ? offset : (offset & 0xFFFF));
    return;
  }

  const uint8_t type = (high >> 8) & 0x0F;
  if (type != 0x4 && type != 0xC) {
    LOG_WARN("PM: far ", isCall ? "CALL" : "JMP", " to system descriptor type 0x",
             std::hex, static_cast<int>(type),
             " (task switches are not supported), selector 0x", selector);
    raiseException(13, selector & 0xFFFC);
    return;
  }

  // Call gate.
  const bool gate32 = type == 0xC;
  const uint16_t target = static_cast<uint16_t>(low >> 16);
  const uint32_t targetOffset =
      (low & 0xFFFF) | (gate32 ? (high & 0xFFFF0000u) : 0u);
  const uint32_t paramCount = high & 0x1F;
  uint32_t tlow = 0, thigh = 0;
  if (!readDescriptor(target, tlow, thigh) || !(thigh & 0x0800)) {
    raiseException(13, target & 0xFFFC);
    return;
  }
  const uint8_t targetDpl = (thigh >> 13) & 3;
  const bool conforming = (thigh & 0x0400) != 0;

  uint8_t newCpl = cpl;
  if (isCall && !conforming && targetDpl < cpl) {
    uint16_t ss = 0;
    uint32_t esp = 0;
    if (!tssStack(targetDpl, ss, esp)) {
      LOG_WARN("PM: call gate to ring ", static_cast<int>(targetDpl),
               " without a TSS");
      raiseException(10, m_cpu.getTRSelector() & 0xFFFC);
      return;
    }
    const uint16_t oldSS = m_cpu.getSegReg(SS);
    const uint32_t oldESP = m_cpu.getReg32(ESP);
    const uint32_t oldBase = m_cpu.getSegBase(SS);
    const bool oldStack32 = m_cpu.is32BitStack();
    const uint32_t oldSP = oldStack32 ? oldESP : (oldESP & 0xFFFF);
    const uint32_t size = gate32 ? 4 : 2;
    std::array<uint32_t, 32> params{};
    for (uint32_t i = 0; i < paramCount; ++i) {
      const uint32_t addr = oldBase + oldSP + i * size;
      params[i] = gate32 ? m_memory.lread32(addr) : m_memory.lread16(addr);
    }
    const uint16_t oldCS = m_cpu.getSegReg(CS);
    const uint32_t oldEIP = m_cpu.getEIP();

    loadSegment(SS, ss);
    m_cpu.setReg32(ESP, esp);
    pushSized(gate32, oldSS);
    pushSized(gate32, oldESP);
    for (uint32_t i = paramCount; i-- > 0;)
      pushSized(gate32, params[i]);
    pushSized(gate32, oldCS);
    pushSized(gate32, oldEIP);
    newCpl = targetDpl;
  } else if (isCall) {
    pushSized(gate32, m_cpu.getSegReg(CS));
    pushSized(gate32, m_cpu.getEIP());
  }
  loadSegment(CS, static_cast<uint16_t>((target & 0xFFFC) | newCpl));
  m_cpu.setEIP(gate32 ? targetOffset : (targetOffset & 0xFFFF));
}

void InstructionDecoder::nativeRetf(uint16_t release, bool op32) {
  const uint8_t cpl = currentCPL();
  const uint32_t eip = popSized(op32);
  const uint16_t cs = static_cast<uint16_t>(popSized(op32));
  auto releaseParams = [&]() {
    if (m_cpu.is32BitStack())
      m_cpu.setReg32(ESP, m_cpu.getReg32(ESP) + release);
    else
      m_cpu.setReg16(SP, static_cast<uint16_t>(m_cpu.getReg16(SP) + release));
  };
  releaseParams();
  const bool v86 = (m_cpu.getEFLAGS() & kFlagVM) != 0;
  if (!v86 && (cs & 3) > cpl) {
    const uint32_t esp = popSized(op32);
    const uint16_t ss = static_cast<uint16_t>(popSized(op32));
    loadSegment(CS, cs);
    m_cpu.setEIP(op32 ? eip : (eip & 0xFFFF));
    loadSegment(SS, ss);
    if (op32)
      m_cpu.setReg32(ESP, esp);
    else
      setStackPointer(esp & 0xFFFF);
    releaseParams();
    return;
  }
  loadSegment(CS, cs);
  m_cpu.setEIP(op32 ? eip : (eip & 0xFFFF));
}

} // namespace fador::cpu
