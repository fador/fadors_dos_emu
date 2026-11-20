#include "FPU.hpp"

#include <bit>
#include <cstring>

namespace fador::cpu::x87 {

namespace {

constexpr uint16_t kUnordered = SW_C0 | SW_C2 | SW_C3;

inline void put16(uint8_t *p, uint16_t v) { std::memcpy(p, &v, 2); }
inline void put32(uint8_t *p, uint32_t v) { std::memcpy(p, &v, 4); }
inline uint16_t get16(const uint8_t *p) {
  uint16_t v;
  std::memcpy(&v, p, 2);
  return v;
}
inline uint32_t get32(const uint8_t *p) {
  uint32_t v;
  std::memcpy(&v, p, 4);
  return v;
}

Float80 exactFromInt(int64_t v) {
  if (v == 0)
    return kPosZero;
  const bool sign = v < 0;
  const uint64_t mag = sign ? 0 - static_cast<uint64_t>(v) : static_cast<uint64_t>(v);
  const int s = std::countl_zero(mag);
  return Float80::make(sign, static_cast<uint16_t>(16383 + 63 - s), mag << s);
}

} // namespace

void FPU::reset() {
  // FNINIT leaves the register contents alone; only the tags change.
  m_emptyMask = 0xFF;
  m_top = 0;
  m_cw = kDefaultControlWord;
  m_sw = 0;
  m_ferrSignalled = false;
  fip = 0;
  fcs = 0;
  fop = 0;
  fdp = 0;
  fds = 0;
}

uint16_t FPU::tagWord() const {
  uint16_t tw = 0;
  for (int p = 0; p < 8; ++p) {
    const uint16_t tag = ((m_emptyMask >> p) & 1) ? 3 : tagFor(m_regs[p]);
    tw = static_cast<uint16_t>(tw | (tag << (p * 2)));
  }
  return tw;
}

void FPU::setTagWord(uint16_t tw) {
  uint8_t mask = 0;
  for (int p = 0; p < 8; ++p) {
    if (((tw >> (p * 2)) & 3) == 3)
      mask = static_cast<uint8_t>(mask | (1u << p));
  }
  m_emptyMask = mask;
}

bool FPU::push(const Float80 &v) {
  if (pushBlocked()) {
    update(SW_IE | SW_SF | SW_C1, SW_C1);
    return false;
  }
  pushRaw(v);
  return true;
}

bool FPU::pop() {
  if (isEmpty(0)) {
    update(SW_IE | SW_SF, SW_C1);
    return false;
  }
  popRaw();
  return true;
}

void FPU::pushChecked(const Float80 &v, uint16_t flags) {
  if (flags == 0 && !pushBlocked()) [[likely]] {
    pushRaw(v);
    m_sw = static_cast<uint16_t>(m_sw & ~SW_C1);
    return;
  }
  if (pushBlocked()) {
    if (stackOverflow())
      pushRaw(kIndefinite);
    return;
  }
  // An unmasked denormal operand does not stop a load.
  if (flags & ~m_cw & SW_IE) {
    update(flags & SW_EXC, SW_C1);
    return;
  }
  pushRaw(v);
  update(flags & SW_EXC, SW_C1);
}

// ---------------------------------------------------------------------------
// Loads

void FPU::fldValue(const Float80 &v) { pushChecked(v, 0); }

void FPU::fldRegSlow(int i) {
  // An empty source is reported (as underflow) before a full stack.
  if (isEmpty(i)) {
    if (stackUnderflow())
      pushRaw(kIndefinite);
    return;
  }
  if (pushBlocked()) {
    if (stackOverflow())
      pushRaw(kIndefinite);
    return;
  }
  const Float80 v = st(i);
  pushRaw(v);
  update(0, SW_C1);
}

void FPU::fldF32Slow(uint32_t raw) {
  const uint32_t e = (raw >> 23) & 0xFF;
  if (e != 0 && e != 0xFF) [[likely]] { // normal: exact, no exceptions
    const Float80 v = Float80::make((raw >> 31) != 0, static_cast<uint16_t>(e - 127 + 16383),
                                    kIntegerBit | (static_cast<uint64_t>(raw & 0x7FFFFF) << 40));
    pushChecked(v, 0);
    return;
  }
  uint16_t flags = knownFlags();
  const Float80 v = active::loadF32(raw, m_cw, flags);
  pushChecked(v, flags);
}

void FPU::fldF64Slow(uint64_t raw) {
  const uint32_t e = static_cast<uint32_t>((raw >> 52) & 0x7FF);
  if (e != 0 && e != 0x7FF) [[likely]] {
    const Float80 v = Float80::make((raw >> 63) != 0, static_cast<uint16_t>(e - 1023 + 16383),
                                    kIntegerBit | ((raw & 0xFFFFFFFFFFFFFull) << 11));
    pushChecked(v, 0);
    return;
  }
  uint16_t flags = knownFlags();
  const Float80 v = active::loadF64(raw, m_cw, flags);
  pushChecked(v, flags);
}

void FPU::fildValue(int64_t v) { pushChecked(exactFromInt(v), 0); }

void FPU::fbld(const uint8_t *bcd10) {
  if (pushBlocked()) {
    if (stackOverflow())
      pushRaw(kIndefinite);
    return;
  }
  pushChecked(active::loadBCD(bcd10), 0);
}

void FPU::fldConstSlow(Const c) { pushChecked(soft::loadConst(c, m_cw), 0); }

// ---------------------------------------------------------------------------
// Stores

namespace {
// Unmasked exceptions that suppress a store to memory.
constexpr uint16_t kNoStore = SW_IE | SW_OE | SW_UE;
} // namespace

bool FPU::fstF32(uint32_t &out, bool pop) {
  if (isEmpty(0)) {
    if (!stackUnderflow())
      return false;
    out = 0xFFC00000u;
    if (pop)
      popRaw();
    return true;
  }
  uint16_t flags = knownFlags();
#if FADOR_X87_HOST
  if (host::fastReady(m_cw, flags)) [[likely]] {
    host::fastStoreF32(&m_regs[phys(0)], out, flags);
    update(flags, SW_C1);
    if (pop)
      popRaw();
    return true;
  }
#endif
  out = active::storeF32(st(0), m_cw, flags);
  if (flags & ~m_cw & kNoStore) {
    update(flags & SW_EXC, SW_C1);
    return false;
  }
  update(flags, SW_C1);
  if (pop)
    popRaw();
  return true;
}

bool FPU::fstF64Slow(uint64_t &out, bool pop) {
  if (isEmpty(0)) {
    if (!stackUnderflow())
      return false;
    out = 0xFFF8000000000000ull;
    if (pop)
      popRaw();
    return true;
  }
  const Float80 v = st(0);
  const uint16_t e = v.exp();
  // Exactly representable normal double: no rounding, no exceptions.
  if ((v.mant & 0x7FF) == 0 && e > 16383 - 1023 && e < 16383 + 1024 && v.integerBit()) [[likely]] {
    out = (v.sign() ? 0x8000000000000000ull : 0) |
          (static_cast<uint64_t>(e - 16383 + 1023) << 52) |
          ((v.mant >> 11) & 0xFFFFFFFFFFFFFull);
    update(0, SW_C1);
    if (pop)
      popRaw();
    return true;
  }
  uint16_t flags = knownFlags();
#if FADOR_X87_HOST
  if (host::fastReady(m_cw, flags)) [[likely]] {
    host::fastStoreF64(&m_regs[phys(0)], out, flags);
    update(flags, SW_C1);
    if (pop)
      popRaw();
    return true;
  }
#endif
  out = active::storeF64(v, m_cw, flags);
  if (flags & ~m_cw & kNoStore) {
    update(flags & SW_EXC, SW_C1);
    return false;
  }
  update(flags, SW_C1);
  if (pop)
    popRaw();
  return true;
}

bool FPU::fst80(Float80 &out, bool pop) {
  if (isEmpty(0)) {
    if (!stackUnderflow())
      return false;
    out = kIndefinite;
  } else {
    out = st(0);
    update(0, SW_C1);
  }
  if (pop)
    popRaw();
  return true;
}

bool FPU::fist(int bits, int64_t &out, bool pop) {
  if (isEmpty(0)) {
    if (!stackUnderflow())
      return false;
    out = bits == 16 ? -32768 : bits == 32 ? static_cast<int64_t>(INT32_MIN) : INT64_MIN;
    if (pop)
      popRaw();
    return true;
  }
  uint16_t flags = knownFlags();
#if FADOR_X87_HOST
  if (host::fastReady(m_cw, flags)) [[likely]] {
    host::fastStoreInt(&m_regs[phys(0)], bits, out, flags);
    update(flags, SW_C1);
    if (pop)
      popRaw();
    return true;
  }
#endif
  out = active::storeInt(st(0), bits, m_cw, flags);
  if (flags & ~m_cw & SW_IE) {
    update(flags & SW_EXC, SW_C1);
    return false;
  }
  update(flags, SW_C1);
  if (pop)
    popRaw();
  return true;
}

bool FPU::fbstp(uint8_t *bcd10) {
  if (isEmpty(0)) {
    if (!stackUnderflow())
      return false;
    std::memset(bcd10, 0, 10);
    bcd10[7] = 0xC0;
    bcd10[8] = 0xFF;
    bcd10[9] = 0xFF;
    popRaw();
    return true;
  }
  uint16_t flags = knownFlags();
  active::storeBCD(st(0), bcd10, m_cw, flags);
  if (flags & ~m_cw & SW_IE) {
    update(flags & SW_EXC, SW_C1);
    return false;
  }
  update(flags, SW_C1);
  popRaw();
  return true;
}

void FPU::fstp1(int i) {
  if (!isEmpty(0))
    setSt(i, st(0));
  update(0, SW_C1);
  popRaw();
}

void FPU::fstRegSlow(int i, bool pop) {
  if (isEmpty(0)) {
    if (!stackUnderflow())
      return;
    setSt(i, kIndefinite);
  } else {
    setSt(i, st(0));
    update(0, SW_C1);
  }
  if (pop)
    popRaw();
}

// ---------------------------------------------------------------------------
// Arithmetic

void FPU::arithRegSlow(Arith op, int dst, int src, bool pop) {
  if (isEmpty(dst) || isEmpty(src)) {
    if (!stackUnderflow())
      return;
    setSt(dst, kIndefinite);
    if (pop)
      popRaw();
    return;
  }
  uint16_t flags = knownFlags();
#if FADOR_X87_HOST
  if (host::fastReady(m_cw, flags)) [[likely]] {
    host::fastArith(op, &m_regs[phys(dst)], &m_regs[phys(src)], flags);
    update(flags, SW_C1);
    if (pop)
      popRaw();
    return;
  }
#endif
  active::arithTo(op, &m_regs[phys(dst)], m_regs[phys(src)], m_cw, flags);
  if (suppressed(flags)) {
    update(flags & SW_EXC, SW_C1);
    return;
  }
  update(flags, SW_C1);
  if (pop)
    popRaw();
}

void FPU::arithMemSlow(Arith op, MemFmt fmt, uint64_t raw) {
  if (isEmpty(0)) {
    if (stackUnderflow())
      setSt(0, kIndefinite);
    return;
  }
  uint16_t flags = knownFlags();
#if FADOR_X87_HOST
  if (host::fastReady(m_cw, flags)) [[likely]] {
    host::fastArithMem(op, &m_regs[phys(0)], fmt, &raw, flags);
    update(flags, SW_C1);
    return;
  }
#endif
  active::arithMemTo(op, &m_regs[phys(0)], fmt, raw, m_cw, flags);
  if (suppressed(flags)) {
    update(flags & SW_EXC, SW_C1);
    return;
  }
  update(flags, SW_C1);
}

// ---------------------------------------------------------------------------
// Compares

// Compares set the condition codes even when an unmasked exception
// suppresses the pop (unordered for invalid operands / stack underflow).
void FPU::fcomRegSlow(int i, bool unordered, int pops) {
  if (isEmpty(0) || isEmpty(i)) {
    update(SW_IE | SW_SF | kUnordered, SW_CC);
    if (!ieMasked())
      return;
  } else {
    uint16_t flags = knownFlags();
#if FADOR_X87_HOST
    if (host::fastReady(m_cw, flags)) [[likely]] {
      host::fastCompare(&m_regs[phys(0)], &m_regs[phys(i)], unordered, flags);
      update(flags, SW_CC);
      for (int k = 0; k < pops; ++k)
        popRaw();
      return;
    }
#endif
    active::compare(st(0), st(i), unordered, flags);
    update(flags, SW_CC);
    if (suppressed(flags))
      return;
  }
  for (int k = 0; k < pops; ++k)
    popRaw();
}

void FPU::fcomMem(MemFmt fmt, uint64_t raw, bool pop) {
  if (isEmpty(0)) {
    update(SW_IE | SW_SF | kUnordered, SW_CC);
    if (!ieMasked())
      return;
  } else {
    uint16_t flags = knownFlags();
    active::compareMem(st(0), fmt, raw, flags);
    update(flags, SW_CC);
    if (suppressed(flags))
      return;
  }
  if (pop)
    popRaw();
}

uint32_t FPU::fcomi(int i, bool unordered, bool pop, uint32_t eflags) {
  constexpr uint32_t CF = 0x001, PF = 0x004, AF = 0x010, ZF = 0x040, SF = 0x080, OF = 0x800;
  uint16_t cc = 0;
  bool doPop = pop;
  if (isEmpty(0) || isEmpty(i)) {
    update(SW_IE | SW_SF, SW_C1);
    cc = kUnordered;
    doPop = pop && ieMasked();
  } else {
    uint16_t flags = knownFlags();
    active::compare(st(0), st(i), unordered, flags);
    update(flags & SW_EXC, 0);
    cc = flags & kUnordered;
    doPop = pop && !suppressed(flags);
  }
  eflags &= ~(CF | PF | AF | ZF | SF | OF);
  if (cc & SW_C0)
    eflags |= CF;
  if (cc & SW_C2)
    eflags |= PF;
  if (cc & SW_C3)
    eflags |= ZF;
  if (doPop)
    popRaw();
  return eflags;
}

void FPU::ftst() {
  if (isEmpty(0)) {
    update(SW_IE | SW_SF | kUnordered, SW_CC);
    return;
  }
  uint16_t flags = knownFlags();
  active::compare(st(0), kPosZero, false, flags);
  update(flags, SW_CC);
}

void FPU::fxam() {
  const Float80 v = st(0);
  uint16_t cc = v.sign() ? SW_C1 : 0;
  if (isEmpty(0)) {
    cc |= SW_C3 | SW_C0;
  } else {
    switch (classify(v)) {
    case Class::Unsupported:
      break;
    case Class::QNaN:
    case Class::SNaN:
      cc |= SW_C0;
      break;
    case Class::Normal:
      cc |= SW_C2;
      break;
    case Class::Infinity:
      cc |= SW_C2 | SW_C0;
      break;
    case Class::Zero:
      cc |= SW_C3;
      break;
    case Class::Denormal:
      cc |= SW_C3 | SW_C2;
      break;
    }
  }
  update(cc, SW_CC);
}

void FPU::fchs() {
  if (isEmpty(0)) {
    if (stackUnderflow())
      setSt(0, kIndefinite);
    return;
  }
  Float80 v = st(0);
  v.signExp ^= 0x8000;
  setSt(0, v);
  update(0, SW_C1);
}

void FPU::fabs() {
  if (isEmpty(0)) {
    if (stackUnderflow())
      setSt(0, kIndefinite);
    return;
  }
  Float80 v = st(0);
  v.signExp &= 0x7FFF;
  setSt(0, v);
  update(0, SW_C1);
}

void FPU::fxchSlow(int i) {
  if (isEmpty(0) || isEmpty(i)) {
    if (!stackUnderflow())
      return;
    const Float80 a = isEmpty(0) ? kIndefinite : st(0);
    const Float80 b = isEmpty(i) ? kIndefinite : st(i);
    setSt(0, b);
    setSt(i, a);
    return;
  }
  const Float80 a = st(0);
  setSt(0, st(i));
  setSt(i, a);
  update(0, SW_C1);
}

void FPU::fincstp() {
  m_top = static_cast<uint8_t>((m_top + 1) & 7);
  update(0, SW_C1);
}

void FPU::fdecstp() {
  m_top = static_cast<uint8_t>((m_top - 1) & 7);
  update(0, SW_C1);
}

void FPU::fcmov(bool condition, int i) {
  if (isEmpty(0) || isEmpty(i)) {
    if (stackUnderflow())
      setSt(0, kIndefinite);
    return;
  }
  if (condition)
    setSt(0, st(i));
}

void FPU::stackOp(uint8_t modrm) {
  const int nIn = stackOpInputs(modrm);
  const int nOutOk = stackOpOutputs(modrm);
  const uint16_t nanCC = stackOpDefinedCC(modrm, true);
  if (isEmpty(0) || (nIn > 1 && isEmpty(1))) {
    update(SW_IE | SW_SF, nanCC);
    if (!ieMasked())
      return;
    // Masked response: the destination becomes the indefinite QNaN.
    if (nOutOk < nIn) {
      setSt(1, kIndefinite);
      popRaw();
    } else if (nOutOk > nIn) {
      // The push happens even onto a full stack (underflow is reported).
      setSt(0, kIndefinite);
      pushRaw(kIndefinite);
    } else {
      setSt(0, kIndefinite);
    }
    return;
  }
  if (nOutOk > nIn && pushBlocked()) {
    update(SW_IE | SW_SF | SW_C1, nanCC);
    if (ieMasked()) {
      setSt(0, kIndefinite);
      pushRaw(kIndefinite);
    }
    return;
  }
#if FADOR_X87_HOST
  if ((modrm == 0xFA || modrm == 0xFC)) {
    uint16_t flags = knownFlags();
    if (host::fastReady(m_cw, flags)) [[likely]] {
      host::fastUnary(modrm, &m_regs[phys(0)], flags);
      update(flags, SW_C1);
      return;
    }
  }
#endif
  Float80 in[2] = {st(0), nIn > 1 ? st(1) : Float80{}};
  Float80 out[3];
  uint16_t flags = knownFlags();
  const int nOut = active::stackOp(modrm, nIn, in, out, m_cw, flags);
  if (suppressed(flags)) {
    update(flags & SW_EXC, nanCC);
    return;
  }
  const int delta = nOut - nIn;
  if (delta < 0) {
    for (int k = 0; k < nOut; ++k)
      setSt(k - delta, out[k]);
    for (int k = 0; k < -delta; ++k)
      popRaw();
  } else if (delta > 0) {
    m_top = static_cast<uint8_t>((m_top - delta) & 7);
    for (int k = 0; k < nOut; ++k)
      setSt(k, out[k]);
  } else {
    for (int k = 0; k < nOut; ++k)
      setSt(k, out[k]);
  }
  update(flags, stackOpDefinedCC(modrm, isNaN(out[0])));
}

void FPU::fnclex() {
  m_sw = static_cast<uint16_t>(m_sw & ~(SW_EXC | SW_SF | SW_ES | SW_B));
  m_ferrSignalled = false;
}

// ---------------------------------------------------------------------------
// Environment / state images

void FPU::storeEnv(uint8_t *out, ImageMode mode) const {
  const uint16_t cw = m_cw;
  const uint16_t sw = statusWord();
  const uint16_t tw = tagWord();
  // Real-mode images hold linear addresses (selector * 16 + offset).
  const uint32_t fipLinear = static_cast<uint32_t>(fcs) * 16u + fip;
  const uint32_t fdpLinear = static_cast<uint32_t>(fds) * 16u + fdp;
  const uint16_t op = static_cast<uint16_t>(fop & 0x07FF);
  switch (mode) {
  case ImageMode::Prot32:
    put32(out + 0, 0xFFFF0000u | cw);
    put32(out + 4, 0xFFFF0000u | sw);
    put32(out + 8, 0xFFFF0000u | tw);
    put32(out + 12, fip);
    put32(out + 16, static_cast<uint32_t>(fcs) | (static_cast<uint32_t>(op) << 16));
    put32(out + 20, fdp);
    put32(out + 24, 0xFFFF0000u | fds);
    break;
  case ImageMode::Real32:
    put32(out + 0, 0xFFFF0000u | cw);
    put32(out + 4, 0xFFFF0000u | sw);
    put32(out + 8, 0xFFFF0000u | tw);
    put32(out + 12, 0xFFFF0000u | (fipLinear & 0xFFFFu));
    put32(out + 16, ((fipLinear >> 16) << 12) | op);
    put32(out + 20, 0xFFFF0000u | (fdpLinear & 0xFFFFu));
    put32(out + 24, (fdpLinear >> 16) << 12);
    break;
  case ImageMode::Prot16:
    put16(out + 0, cw);
    put16(out + 2, sw);
    put16(out + 4, tw);
    put16(out + 6, static_cast<uint16_t>(fip));
    put16(out + 8, fcs);
    put16(out + 10, static_cast<uint16_t>(fdp));
    put16(out + 12, fds);
    break;
  case ImageMode::Real16:
    put16(out + 0, cw);
    put16(out + 2, sw);
    put16(out + 4, tw);
    put16(out + 6, static_cast<uint16_t>(fipLinear));
    put16(out + 8, static_cast<uint16_t>((((fipLinear >> 16) & 0xF) << 12) | op));
    put16(out + 10, static_cast<uint16_t>(fdpLinear));
    put16(out + 12, static_cast<uint16_t>(((fdpLinear >> 16) & 0xF) << 12));
    break;
  }
}

void FPU::loadEnv(const uint8_t *in, ImageMode mode) {
  switch (mode) {
  case ImageMode::Prot32: {
    m_cw = get16(in + 0);
    setStatusWord(get16(in + 4));
    setTagWord(get16(in + 8));
    fip = get32(in + 12);
    const uint32_t selOp = get32(in + 16);
    fcs = static_cast<uint16_t>(selOp);
    fop = static_cast<uint16_t>((selOp >> 16) & 0x07FF);
    fdp = get32(in + 20);
    fds = get16(in + 24);
    break;
  }
  case ImageMode::Real32: {
    m_cw = get16(in + 0);
    setStatusWord(get16(in + 4));
    setTagWord(get16(in + 8));
    const uint32_t ipHi = get32(in + 16);
    fip = (get32(in + 12) & 0xFFFFu) | (((ipHi >> 12) & 0xFFFFu) << 16);
    fop = static_cast<uint16_t>(ipHi & 0x07FF);
    fcs = 0;
    fdp = (get32(in + 20) & 0xFFFFu) | (((get32(in + 24) >> 12) & 0xFFFFu) << 16);
    fds = 0;
    break;
  }
  case ImageMode::Prot16:
    m_cw = get16(in + 0);
    setStatusWord(get16(in + 2));
    setTagWord(get16(in + 4));
    fip = get16(in + 6);
    fcs = get16(in + 8);
    fop = 0;
    fdp = get16(in + 10);
    fds = get16(in + 12);
    break;
  case ImageMode::Real16: {
    m_cw = get16(in + 0);
    setStatusWord(get16(in + 2));
    setTagWord(get16(in + 4));
    const uint16_t ipHi = get16(in + 8);
    fip = get16(in + 6) | (static_cast<uint32_t>((ipHi >> 12) & 0xF) << 16);
    fop = static_cast<uint16_t>(ipHi & 0x07FF);
    fcs = 0;
    fdp = get16(in + 10) | (static_cast<uint32_t>((get16(in + 12) >> 12) & 0xF) << 16);
    fds = 0;
    break;
  }
  }
  refreshSummary();
}

void FPU::storeRegs(uint8_t *out80) const {
  for (int i = 0; i < 8; ++i)
    storeFloat80(out80 + 10 * i, st(i));
}

void FPU::loadRegs(const uint8_t *in80) {
  for (int i = 0; i < 8; ++i)
    storeWhole(m_regs[phys(i)], loadFloat80(in80 + 10 * i));
}

} // namespace fador::cpu::x87
