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
constexpr uint32_t kFlagVIF = 0x00080000;
constexpr uint32_t kFlagVIP = 0x00100000;

// Exceptions that push an error code in protected mode.
bool hasErrorCode(uint8_t vector) {
  return vector == 8 || (vector >= 10 && vector <= 14) || vector == 17;
}
} // namespace

bool InstructionDecoder::requireFeature(uint32_t featureBits) {
  if (m_cpu.hasFeature(featureBits))
    return true;
  raiseException(6); // #UD: the instruction does not exist on this model
  return false;
}

// POPF/IRET privilege rules (386 SDM): IOPL changes only at CPL 0, IF only
// when CPL <= IOPL, VM never through POPF and through IRET only on the
// CPL-0 return to V86 (handled by the callers), VIF/VIP only by a CPL-0
// protected-mode IRET.  RF is loaded by IRET and cleared by POPF.  Bits a
// model lacks (AC on the 386, ID without CPUID, VIF/VIP before the
// Pentium) are excluded by the model's EFLAGS mask.
//
// Under the emulator's own DPMI host (no native protection) clients keep
// full control of IOPL and IF, as before.
uint32_t InstructionDecoder::flagsWriteMask(bool iret) const {
  uint32_t mask = m_cpu.eflagsMask() & ~kFlagVM;
  const bool pm = (m_cpu.getCR(0) & 1) != 0;
  if (!iret)
    mask &= ~(kFlagRF | kFlagVIF | kFlagVIP);
  else if (!pm)
    mask &= ~(kFlagVIF | kFlagVIP);
  if (!pm || !m_cpu.nativeProtection())
    return mask;
  const uint8_t cpl = currentCPL();
  if (cpl > 0)
    mask &= ~(kFlagIOPL | kFlagVIF | kFlagVIP);
  if (cpl > ((m_cpu.getEFLAGS() & kFlagIOPL) >> 12))
    mask &= ~kFlagIF;
  return mask;
}

void InstructionDecoder::writeFlags(uint32_t value, uint32_t mask, bool op32) {
  if (!op32)
    mask &= 0xFFFFu;
  const uint32_t merged = (m_cpu.getEFLAGS() & ~mask) | (value & mask);
  m_cpu.setEFLAGS((merged & m_cpu.eflagsMask()) | 0x2u);
  if (merged & mask & kFlagRF)
    m_rfLoaded = true; // IRET: the next instruction ignores breakpoints
}

bool InstructionDecoder::checkIOPL() {
  if (!m_cpu.nativeProtection() ||
      currentCPL() <= ((m_cpu.getEFLAGS() & kFlagIOPL) >> 12))
    return true;
  raiseException(13, 0);
  return false;
}

bool InstructionDecoder::requireCPL0() {
  if (!m_cpu.nativeProtection() || currentCPL() == 0)
    return true;
  raiseException(13, 0);
  return false;
}

bool InstructionDecoder::ioBitmapAllows(uint16_t port, uint32_t size) const {
  // Only a 32-bit TSS has an I/O permission bitmap; its offset is the word
  // at 66h and every port needs its bit clear and inside the TSS limit.
  uint32_t low = 0, high = 0;
  if (!readDescriptor(m_cpu.getTRSelector(), low, high) ||
      !((high >> 8) & 0x08))
    return false;
  const DescriptorRegister tr = m_cpu.getTR();
  if (tr.limit < 0x67)
    return false;
  const uint32_t iopb = m_memory.lread16(tr.base + 0x66);
  const uint32_t first = iopb + port / 8u;
  const uint32_t last = iopb + (port + size - 1u) / 8u;
  if (last > tr.limit)
    return false;
  uint32_t bits = m_memory.lread8(tr.base + first);
  if (last != first)
    bits |= static_cast<uint32_t>(m_memory.lread8(tr.base + last)) << 8;
  const uint32_t mask = ((1u << size) - 1u) << (port & 7u);
  return (bits & mask) == 0;
}

bool InstructionDecoder::checkIOPermission(uint16_t port, uint32_t size) {
  if (!m_cpu.nativeProtection())
    return true;
  const uint32_t flags = m_cpu.getEFLAGSNonArith();
  if (!(flags & kFlagVM) && currentCPL() <= ((flags & kFlagIOPL) >> 12))
    return true;
  if (ioBitmapAllows(port, size))
    return true;
  raiseException(13, 0);
  return false;
}

void InstructionDecoder::executeCPUID() {
  const CpuModel &model = m_cpu.model();
  uint32_t leaf = m_cpu.getReg32(EAX);
  // Intel parts of this era answer any leaf above the maximum (including
  // the 8000_0000h range, which they do not implement) with the data of
  // the highest basic leaf.
  if (leaf > model.cpuidMaxLeaf)
    leaf = model.cpuidMaxLeaf;
  uint32_t a = 0, b = 0, c = 0, d = 0;
  switch (leaf) {
  case 0:
    a = model.cpuidMaxLeaf;
    b = 0x756E6547; // "Genu"
    d = 0x49656E69; // "ineI"
    c = 0x6C65746E; // "ntel"
    break;
  case 1:
    a = model.signature;
    d = model.cpuidFeatureEdx;
    if (!m_cpu.fpuPresent())
      d &= ~cpuid_edx::kFpu;
    break;
  case 2: // P6 cache/TLB descriptors: one iteration; ITLB 4K/4M, DTLB 4K,
          // L1I 8K, DTLB 4M, L1D 8K, L2 256K
    a = 0x03020101;
    d = 0x06040A42;
    break;
  default:
    break;
  }
  m_cpu.setReg32(EAX, a);
  m_cpu.setReg32(EBX, b);
  m_cpu.setReg32(ECX, c);
  m_cpu.setReg32(EDX, d);
}

bool InstructionDecoder::readMSR(uint32_t index, uint64_t &value) const {
  switch (index) {
  case 0x10: // time-stamp counter
    value = m_cpu.readTSC();
    return true;
  default:
    return false;
  }
}

bool InstructionDecoder::writeMSR(uint32_t index, uint64_t value) {
  switch (index) {
  case 0x10:
    // The P6 writes only the low 32 bits and clears the high half.
    if (m_cpu.hasFeature(feature::kCmov))
      value &= 0xFFFFFFFFu;
    m_cpu.writeTSC(value);
    return true;
  default:
    return false;
  }
}

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
  ++m_faultCount;
  m_cpu.setEIP(m_instrStartEIP);
  if (m_cpu.nativeProtection()) {
    nativeInterrupt(vector, false, hasErrorCode(vector), errorCode);
  } else {
    triggerInterrupt(vector);
  }
}

void InstructionDecoder::nativeInterrupt(uint8_t vector, bool software,
                                         bool pushError, uint32_t errorCode) {
  ++m_interruptCount;
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
  if (software && gateDpl < cpl) {
    // INT n may only use gates the caller is allowed to see.
    nativeInterrupt(13, false, true, vector * 8u + 2);
    return;
  }
  if (gateType == 0x5) { // task gate: the handler is another task
    taskSwitch(static_cast<uint16_t>(low >> 16), TaskSwitch::Int, pushError,
               errorCode);
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
    // IOPL 3 V86 IRET behaves like real mode; VM, IOPL, VIF and VIP stay
    // (CPL 3 rules of flagsWriteMask()).
    const uint32_t mask = flagsWriteMask(true);
    const uint32_t eip = popSized(op32);
    const uint16_t cs = static_cast<uint16_t>(popSized(op32));
    const uint32_t newFlags = popSized(op32);
    writeFlags(newFlags, mask, op32);
    loadSegment(CS, cs);
    m_cpu.setEIP(eip & 0xFFFF);
    return;
  }
  if (flags & kFlagNT) {
    // Nested task: return to the task in the back link of the current TSS.
    taskSwitch(m_memory.lread16(m_cpu.getTR().base), TaskSwitch::Iret);
    return;
  }

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
    m_cpu.setEFLAGS((newFlags & m_cpu.eflagsMask()) | 2);
    m_rfLoaded = (newFlags & kFlagRF) != 0;
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
  const uint32_t changeable = flagsWriteMask(true);

  const uint8_t rpl = cs & 3;
  if (rpl > cpl) {
    const uint32_t esp = popSized(op32);
    const uint16_t ss = static_cast<uint16_t>(popSized(op32));
    writeFlags(newFlags, changeable, op32);
    loadSegment(CS, cs);
    m_cpu.setEIP(op32 ? eip : (eip & 0xFFFF));
    loadSegment(SS, ss);
    if (op32)
      m_cpu.setReg32(ESP, esp);
    else
      setStackPointer(esp & 0xFFFF);
    return;
  }
  writeFlags(newFlags, changeable, op32);
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
  const uint8_t dpl = (high >> 13) & 3;
  if (type == 0x1 || type == 0x9 || type == 0x5) {
    // Available TSS or task gate: DPL >= max(CPL, RPL).
    if (dpl < cpl || dpl < (selector & 3)) {
      raiseException(13, selector & 0xFFFC);
      return;
    }
    const uint16_t tss =
        type == 0x5 ? static_cast<uint16_t>(low >> 16) : selector;
    taskSwitch(tss, isCall ? TaskSwitch::Call : TaskSwitch::Jmp);
    return;
  }
  if (type != 0x4 && type != 0xC) {
    raiseException(13, selector & 0xFFFC); // busy TSS, LDT, ...
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

// ── Hardware task switches ───────────────────────────────────────────────
//
// 32-bit TSS: 00 link, 04 ESP0/SS0 ... 1C CR3, 20 EIP, 24 EFLAGS,
// 28-44 EAX ECX EDX EBX ESP EBP ESI EDI, 48-5C ES CS SS DS FS GS, 60 LDT,
// 64 T bit, 66 I/O map base.  16-bit (286) TSS: 00 link, 02-0C SP0/SS0..,
// 0E IP, 10 FLAGS, 12-20 AX CX DX BX SP BP SI DI, 22-28 ES CS SS DS, 2A LDT.

void InstructionDecoder::taskSwitch(uint16_t selector, TaskSwitch kind,
                                    bool pushError, uint32_t errorCode) {
  const uint16_t sel = selector & 0xFFFC;
  uint32_t low = 0, high = 0;
  if ((selector & 4) || !readDescriptor(selector, low, high) ||
      (high & 0x1000)) {
    raiseException(kind == TaskSwitch::Iret ? 10 : 13, sel);
    return;
  }
  const uint32_t type = (high >> 8) & 0x0F;
  const bool isTss = type == 0x1 || type == 0x3 || type == 0x9 || type == 0xB;
  const bool isBusy = (type & 0x2) != 0;
  if (!isTss || isBusy != (kind == TaskSwitch::Iret)) {
    // JMP/CALL/INT need an available TSS, IRET a busy one.
    raiseException(kind == TaskSwitch::Iret ? 10 : 13, sel);
    return;
  }
  if (!(high & 0x8000)) {
    raiseException(11, sel);
    return;
  }
  const Descriptor desc = decodeDescriptor(low, high);
  const bool new32 = (type & 0x8) != 0;
  if (desc.limit < (new32 ? 0x67u : 0x2Bu)) {
    raiseException(10, sel);
    return;
  }

  // Save the outgoing task.
  const uint16_t oldSel = m_cpu.getTRSelector();
  const uint32_t oldBase = m_cpu.getTR().base;
  uint32_t oldLow = 0, oldHigh = 0;
  const bool haveOld = readDescriptor(oldSel, oldLow, oldHigh);
  const bool old32 = !haveOld || ((oldHigh >> 8) & 0x8) != 0;
  uint32_t flags = m_cpu.getEFLAGS();
  if (kind == TaskSwitch::Iret)
    flags &= ~kFlagNT;
  if (old32) {
    m_memory.lwrite32(oldBase + 0x20, m_cpu.getEIP());
    m_memory.lwrite32(oldBase + 0x24, flags);
    for (uint8_t r = 0; r < 8; ++r)
      m_memory.lwrite32(oldBase + 0x28 + r * 4u, m_cpu.getReg32(r));
    for (uint8_t s = 0; s < 6; ++s)
      m_memory.lwrite32(oldBase + 0x48 + s * 4u, m_cpu.getSegReg(s));
  } else {
    m_memory.lwrite16(oldBase + 0x0E, static_cast<uint16_t>(m_cpu.getEIP()));
    m_memory.lwrite16(oldBase + 0x10, static_cast<uint16_t>(flags));
    for (uint8_t r = 0; r < 8; ++r)
      m_memory.lwrite16(oldBase + 0x12 + r * 2u, m_cpu.getReg16(r));
    for (uint8_t s = 0; s < 4; ++s)
      m_memory.lwrite16(oldBase + 0x22 + s * 2u, m_cpu.getSegReg(s));
  }

  // Read the incoming task.
  const uint32_t base = desc.base;
  uint32_t eip, newFlags, cr3 = 0;
  std::array<uint32_t, 8> regs{};
  std::array<uint16_t, 6> segs{};
  uint16_t ldt;
  bool debugTrap = false;
  if (new32) {
    cr3 = m_memory.lread32(base + 0x1C);
    eip = m_memory.lread32(base + 0x20);
    newFlags = m_memory.lread32(base + 0x24);
    for (uint8_t r = 0; r < 8; ++r)
      regs[r] = m_memory.lread32(base + 0x28 + r * 4u);
    for (uint8_t s = 0; s < 6; ++s)
      segs[s] = m_memory.lread16(base + 0x48 + s * 4u);
    ldt = m_memory.lread16(base + 0x60);
    debugTrap = (m_memory.lread16(base + 0x64) & 1) != 0;
  } else {
    eip = m_memory.lread16(base + 0x0E);
    newFlags = m_memory.lread16(base + 0x10);
    for (uint8_t r = 0; r < 8; ++r)
      regs[r] = (m_cpu.getReg32(r) & 0xFFFF0000u) |
                m_memory.lread16(base + 0x12 + r * 2u);
    for (uint8_t s = 0; s < 4; ++s)
      segs[s] = m_memory.lread16(base + 0x22 + s * 2u);
    ldt = m_memory.lread16(base + 0x2A);
  }

  // Busy bits and the back link.
  const uint32_t gdt = m_cpu.getGDTR().base;
  if (haveOld && (kind == TaskSwitch::Jmp || kind == TaskSwitch::Iret))
    m_memory.lwrite32(gdt + (oldSel & 0xFFF8u) + 4, oldHigh & ~0x0200u);
  if (kind == TaskSwitch::Call || kind == TaskSwitch::Int) {
    m_memory.lwrite16(base, oldSel);
    newFlags |= kFlagNT;
  }
  if (kind != TaskSwitch::Iret)
    m_memory.lwrite32(gdt + sel + 4, high | 0x0200u);

  m_cpu.setTRSelector(selector);
  m_cpu.setTR({static_cast<uint16_t>(desc.limit), base});
  uint32_t cr0 = m_cpu.getCR(0) | 0x8u; // TS: the x87 state is stale
  m_cpu.setCR(0, cr0);
  if (new32 && (cr0 & 0x80000000u))
    m_cpu.setCR(3, cr3);

  // LDT first: the segment selectors may refer to it.
  m_cpu.setLDTRSelector(ldt);
  uint32_t ldtLow = 0, ldtHigh = 0;
  if ((ldt & 0xFFFC) != 0 && !(ldt & 4) && readDescriptor(ldt, ldtLow, ldtHigh) &&
      (ldtHigh & 0x1F00) == 0x0200) {
    const Descriptor d = decodeDescriptor(ldtLow, ldtHigh);
    m_cpu.setLDTR({static_cast<uint16_t>(d.limit), d.base});
  } else {
    m_cpu.setLDTR({0, 0});
  }

  m_cpu.setEFLAGS((newFlags & m_cpu.eflagsMask()) | 0x2u);
  m_rfLoaded = (newFlags & kFlagRF) != 0;
  for (uint8_t r = 0; r < 8; ++r)
    m_cpu.setReg32(r, regs[r]);
  // CPU::loadSegment decodes V86 selectors as paragraphs (EFLAGS.VM).
  loadSegment(CS, segs[CS]);
  loadSegment(SS, segs[SS]);
  loadSegment(DS, segs[DS]);
  loadSegment(ES, segs[ES]);
  loadSegment(FS, segs[FS]);
  loadSegment(GS, segs[GS]);
  m_cpu.setEIP(eip);
  if (pushError)
    pushSized(new32, errorCode);
  if (debugTrap)
    deliverDebugException(0x8000); // DR6.BT: the new task's T bit
}

// ── Debug registers and single-stepping ─────────────────────────────────

namespace {
constexpr uint32_t kDr6BD = 0x2000; // general detect
constexpr uint32_t kDr6BS = 0x4000; // single step
constexpr uint32_t kDr7GD = 0x2000;
constexpr uint32_t kCr4DE = 0x0008;
} // namespace

void InstructionDecoder::deliverDebugException(uint32_t dr6Bits) {
  // B0-B3 report the breakpoints of this exception only; BD/BS/BT
  // accumulate until software clears them.
  uint32_t dr6 = m_cpu.getDR(6);
  if (dr6Bits & 0xFu)
    dr6 &= ~0xFu;
  m_cpu.setDR(6, dr6 | dr6Bits);
  if (m_cpu.nativeProtection())
    nativeInterrupt(1, false, false, 0);
  else
    triggerInterrupt(1);
}

void InstructionDecoder::stepDebug() {
  syncSegmentCacheIfNeeded();
  const uint32_t flags = m_cpu.getEFLAGSNonArith();
  // Instruction breakpoints are faults taken before the instruction runs;
  // RF (set by the debugger's IRET) lets the instruction run once.
  if (const uint8_t exec = m_cpu.instructionBreakpoints();
      exec != 0 && !(flags & kFlagRF)) {
    const uint32_t linear = m_segBase[CS] + m_cpu.getEIP();
    uint32_t hits = 0;
    for (unsigned i = 0; i < 4; ++i)
      if (((exec >> i) & 1u) && m_cpu.getDR(static_cast<uint8_t>(i)) == linear)
        hits |= 1u << i;
    if (hits != 0) {
      m_instrStartEIP = m_cpu.getEIP();
      ++m_faultCount;
      deliverDebugException(hits);
      return;
    }
  }
  const uint32_t faults = m_faultCount;
  const uint32_t interrupts = m_interruptCount;
  m_rfLoaded = false;
  m_memory.takeDataBreakpointHits();
  stepChecked();
  // RF is cleared once an instruction completes, unless it loaded RF
  // itself (IRET, task switch).
  if (!m_rfLoaded && (m_cpu.getEFLAGSNonArith() & kFlagRF))
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~kFlagRF);
  const uint8_t dataHits = static_cast<uint8_t>(
      m_memory.takeDataBreakpointHits() & m_cpu.dataBreakpoints());
  if (m_faultCount != faults)
    return; // the instruction faulted: no traps
  uint32_t dr6 = dataHits;
  // Single-step trap after an instruction that started with TF set, unless
  // it was an INT n / exception that entered a handler (which runs with
  // TF clear).
  if ((flags & kFlagTF) && m_interruptCount == interrupts)
    dr6 |= kDr6BS;
  if (dr6 != 0)
    deliverDebugException(dr6);
}

bool InstructionDecoder::debugRegisterAccess(uint8_t &dr) {
  if (dr == 4 || dr == 5) {
    if (m_cpu.getCR(4) & kCr4DE) { // debug extensions: DR4/DR5 reserved
      raiseException(6);
      return false;
    }
    dr = static_cast<uint8_t>(dr + 2);
  }
  if (!requireCPL0())
    return false;
  if (m_cpu.getDR(7) & kDr7GD) {
    // General detect: a fault before the MOV, GD cleared for the handler.
    m_cpu.setDR(7, m_cpu.getDR(7) & ~kDr7GD);
    m_cpu.setEIP(m_instrStartEIP);
    ++m_faultCount;
    deliverDebugException(kDr6BD);
    return false;
  }
  return true;
}

uint32_t InstructionDecoder::readDebugRegister(uint8_t dr) const {
  const uint32_t v = m_cpu.getDR(dr);
  if (dr == 6)
    return (v & 0x0000E00Fu) | 0xFFFF0FF0u; // reserved bits read as 1
  if (dr == 7)
    return (v & 0xFFFF2BFFu) | 0x00000400u;
  return v;
}

void InstructionDecoder::writeDebugRegister(uint8_t dr, uint32_t value) {
  if (dr == 6)
    value &= 0x0000E00Fu;
  else if (dr == 7)
    value &= 0xFFFF2BFFu;
  m_cpu.setDR(dr, value);
}

// ── LLDT / LTR ───────────────────────────────────────────────────────────

void InstructionDecoder::nativeLoadLDT(uint16_t selector) {
  if ((selector & 0xFFFC) == 0) {
    m_cpu.setLDTRSelector(selector);
    m_cpu.setLDTR({0, 0});
    return;
  }
  uint32_t low = 0, high = 0;
  if ((selector & 4) || !readDescriptor(selector, low, high) ||
      (high & 0x1F00) != 0x0200) { // system descriptor, type 2 (LDT)
    raiseException(13, selector & 0xFFFC);
    return;
  }
  if (!(high & 0x8000)) {
    raiseException(11, selector & 0xFFFC);
    return;
  }
  const Descriptor d = decodeDescriptor(low, high);
  m_cpu.setLDTRSelector(selector);
  m_cpu.setLDTR({static_cast<uint16_t>(d.limit), d.base});
}

void InstructionDecoder::nativeLoadTR(uint16_t selector) {
  uint32_t low = 0, high = 0;
  if ((selector & 4) || !readDescriptor(selector, low, high) ||
      (((high >> 8) & 0x1F) != 0x01 && ((high >> 8) & 0x1F) != 0x09)) {
    // Null, LDT-based, not a TSS or already busy.
    raiseException(13, selector & 0xFFFC);
    return;
  }
  if (!(high & 0x8000)) {
    raiseException(11, selector & 0xFFFC);
    return;
  }
  const Descriptor d = decodeDescriptor(low, high);
  // LTR marks the task busy.
  m_memory.lwrite32(m_cpu.getGDTR().base + (selector & 0xFFF8u) + 4,
                    high | 0x0200);
  m_cpu.setTRSelector(selector);
  m_cpu.setTR({static_cast<uint16_t>(d.limit), d.base});
}

} // namespace fador::cpu
