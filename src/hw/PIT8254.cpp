#include "PIT8254.hpp"
#include "../utils/Logger.hpp"

namespace fador::hw {

namespace {

uint16_t toBcd16(uint32_t v) {
  v %= 10000;
  return static_cast<uint16_t>((v / 1000) << 12 | ((v / 100) % 10) << 8 |
                               ((v / 10) % 10) << 4 | (v % 10));
}

uint32_t fromBcd16(uint16_t v) {
  return ((v >> 12) & 0xF) * 1000 + ((v >> 8) & 0xF) * 100 +
         ((v >> 4) & 0xF) * 10 + (v & 0xF);
}

// Ticks until a counting element holding ce reaches zero (0 = full count).
uint64_t ticksToZero(uint32_t ce, uint32_t mod) { return ce == 0 ? mod : ce; }

uint32_t wrapSub(uint32_t ce, uint64_t n, uint32_t mod) {
  return static_cast<uint32_t>((ce + mod - n % mod) % mod);
}

} // namespace

PIT8254::PIT8254() {
  // State left by a PC BIOS POST: channel 0 = 18.2 Hz system timer
  // (mode 3, count 0), channel 1 = DRAM refresh (mode 2, count 18),
  // channel 2 = speaker tone (mode 3, ~896 Hz), gated off by port 61h.
  for (auto &c : m_channels) {
    c.mode = c.modeBits = 3;
    c.rw = 3;
    c.crWritten = true;
    c.nullCount = false;
    c.armed = true;
  }
  auto &c0 = m_channels[0];
  c0.cr = 0;
  c0.n3 = 65536;
  c0.half = highHalf(c0.n3);

  auto &c1 = m_channels[1];
  c1.mode = c1.modeBits = 2;
  c1.rw = 1;
  c1.cr = 18;
  c1.ce = 18;

  auto &c2 = m_channels[2];
  c2.cr = 0x0533;
  c2.n3 = 0x0533;
  c2.half = highHalf(c2.n3);
  c2.gate = false;

  recomputeNextIrq();
}

// ────────────────────────────────────────────────────────────
//  time keeping
// ────────────────────────────────────────────────────────────

void PIT8254::sync() {
  if (m_timeSource) {
    const uint64_t now = m_timeSource();
    if (now > m_nowNs)
      m_nowNs = now;
  }
  catchUp();
}

void PIT8254::advanceTime(std::chrono::nanoseconds elapsed) {
  if (elapsed.count() > 0)
    m_nowNs += static_cast<uint64_t>(elapsed.count());
  catchUp();
}

void PIT8254::catchUp() {
  const uint64_t tick = ticksAt(m_nowNs);
  if (tick <= m_lastTick)
    return;
  const uint64_t n = tick - m_lastTick;
  m_lastTick = tick;
  m_pendingIRQ0 += advanceChannel(m_channels[0], n);
  advanceChannel(m_channels[1], n);
  advanceChannel(m_channels[2], n);
  recomputeNextIrq();
}

void PIT8254::recomputeNextIrq() {
  const uint64_t d = ticksToRisingEdge(m_channels[0]);
  m_nextIrqNs = d == kNever ? kNever : nsAtTick(m_lastTick + d);
}

bool PIT8254::checkPendingIRQ0() {
  if (m_pendingIRQ0 == 0)
    return false;
  --m_pendingIRQ0;
  return true;
}

void PIT8254::noteEdge(int index, bool wasHigh) {
  if (index == 0 && !wasHigh && m_channels[0].out)
    ++m_pendingIRQ0;
}

uint32_t PIT8254::initialCount(const Channel &c) {
  const uint32_t v = c.bcd ? fromBcd16(c.cr) : c.cr;
  return v == 0 ? modulus(c) : v;
}

// CR -> CE transfer (on the CLK pulse after a count write or a trigger).
void PIT8254::loadCounter(Channel &c) {
  const uint32_t n = initialCount(c);
  const uint32_t mod = modulus(c);
  c.loadPending = false;
  c.nullCount = false;
  c.armed = true;
  c.done = false;
  switch (c.mode) {
  case 0:
  case 1:
    c.ce = n % mod;
    c.out = false; // mode 1: the one-shot pulse starts
    break;
  case 2:
    c.ce = n % mod;
    c.out = true;
    break;
  case 3:
    c.n3 = n;
    c.half = highHalf(n);
    c.out = true;
    break;
  default: // 4, 5
    c.ce = n % mod;
    c.out = true;
    break;
  }
}

// Advance one channel by `ticks` CLK pulses in closed form; returns the
// number of rising OUT edges.
uint64_t PIT8254::advanceChannel(Channel &c, uint64_t ticks) {
  uint64_t edges = 0;
  uint64_t n = ticks;
  const uint32_t mod = modulus(c);
  while (n > 0) {
    if (c.loadPending) {
      --n;
      loadCounter(c);
      continue;
    }
    if (!c.armed)
      break;
    const bool gateEnables = c.mode != 1 && c.mode != 5;
    if (gateEnables && !c.gate)
      break; // GATE low suspends counting in modes 0, 2, 3, 4

    switch (c.mode) {
    case 0: // interrupt on terminal count
    case 1: // hardware retriggerable one-shot
      if (!c.done && n >= ticksToZero(c.ce, mod)) {
        c.done = true;
        c.out = true;
        ++edges;
      }
      c.ce = wrapSub(c.ce, n, mod); // keeps counting (wraps) afterwards
      n = 0;
      break;

    case 4: // software triggered strobe
    case 5: // hardware triggered strobe
      if (c.done) {
        c.ce = wrapSub(c.ce, n, mod);
        n = 0;
      } else if (!c.out) { // strobe pulse: OUT low for one CLK
        c.out = true;
        c.done = true;
        ++edges;
        c.ce = wrapSub(c.ce, 1, mod);
        --n;
      } else {
        const uint64_t d = ticksToZero(c.ce, mod);
        if (n < d) {
          c.ce = static_cast<uint32_t>(c.ce - n);
          n = 0;
        } else {
          c.ce = 0;
          c.out = false;
          n -= d;
        }
      }
      break;

    case 2: { // rate generator: OUT low while CE == 1, reload after it
      const uint64_t p = ticksToZero(c.ce, mod);
      if (n < p) {
        c.ce = static_cast<uint32_t>(p - n);
        c.out = c.ce != 1;
        n = 0;
        break;
      }
      n -= p;
      ++edges;
      const uint32_t count = initialCount(c); // a new count applies now
      c.nullCount = false;
      const uint64_t k = n / count;
      edges += k;
      n -= k * count;
      c.ce = count % mod;
      c.out = count == 1 || c.ce != 1;
      break;
    }

    default: { // 3: square wave, CE decrements by two per CLK
      if (n < c.half) {
        c.half -= static_cast<uint32_t>(n);
        n = 0;
        break;
      }
      n -= c.half;
      c.n3 = initialCount(c); // a new count applies at the half cycle
      c.nullCount = false;
      if (c.out) {
        c.out = false;
        c.half = lowHalf(c.n3);
      } else {
        c.out = true;
        ++edges;
        c.half = highHalf(c.n3);
        const uint64_t period = highHalf(c.n3) + lowHalf(c.n3);
        const uint64_t k = n / period;
        edges += k;
        n -= k * period;
      }
      break;
    }
    }
  }
  return edges;
}

// CLK pulses from now until the next rising OUT edge (kNever if none is
// scheduled without further programming or gate changes).
uint64_t PIT8254::ticksToRisingEdge(const Channel &c) {
  const bool gateEnables = c.mode != 1 && c.mode != 5;
  if (gateEnables && !c.gate)
    return kNever;
  const uint32_t mod = modulus(c);
  if (c.loadPending) {
    const uint64_t n = initialCount(c);
    switch (c.mode) {
    case 0:
    case 1:
    case 2:
      return 1 + n;
    case 3:
      return 1 + highHalf(static_cast<uint32_t>(n)) +
             lowHalf(static_cast<uint32_t>(n));
    default:
      return 2 + n;
    }
  }
  if (!c.armed)
    return kNever;
  switch (c.mode) {
  case 0:
  case 1:
    return c.done ? kNever : ticksToZero(c.ce, mod);
  case 2:
    return ticksToZero(c.ce, mod);
  case 3:
    return c.out ? c.half + lowHalf(initialCount(c)) : c.half;
  default:
    if (c.done)
      return kNever;
    return c.out ? ticksToZero(c.ce, mod) + 1 : 1;
  }
}

// ────────────────────────────────────────────────────────────
//  register interface
// ────────────────────────────────────────────────────────────

uint16_t PIT8254::visibleCount(const Channel &c) {
  uint32_t v;
  if (c.loadPending && !c.armed) {
    v = initialCount(c); // not loaded yet: show what will be
  } else if (c.mode == 3 && c.armed) {
    // CE runs down by two from the (even-rounded) count each half cycle.
    v = 2 * c.half;
    if (c.out && (c.n3 & 1))
      v -= 2;
  } else {
    v = c.ce;
  }
  v %= modulus(c);
  return c.bcd ? toBcd16(v) : static_cast<uint16_t>(v);
}

uint8_t PIT8254::statusByte(const Channel &c) {
  return static_cast<uint8_t>((c.out ? 0x80 : 0) | (c.nullCount ? 0x40 : 0) |
                              (c.rw << 4) | (c.modeBits << 1) |
                              (c.bcd ? 1 : 0));
}

uint8_t PIT8254::read8(uint16_t port) {
  const int index = port & 3;
  if (index == 3)
    return 0xFF; // the control register is write-only
  sync();
  auto &c = m_channels[index];
  if (c.statusLatched) {
    c.statusLatched = false;
    return c.status;
  }
  const uint16_t v = c.countLatched ? c.latch : visibleCount(c);
  switch (c.rw) {
  case 1:
    c.countLatched = false;
    return static_cast<uint8_t>(v & 0xFF);
  case 2:
    c.countLatched = false;
    return static_cast<uint8_t>(v >> 8);
  default:
    if (!c.readMsbNext) {
      c.readMsbNext = true;
      return static_cast<uint8_t>(v & 0xFF);
    }
    c.readMsbNext = false;
    c.countLatched = false;
    return static_cast<uint8_t>(v >> 8);
  }
}

void PIT8254::write8(uint16_t port, uint8_t value) {
  sync();
  const int index = port & 3;
  if (index == 3)
    writeControl(value);
  else
    writeCount(index, value);
  recomputeNextIrq();
}

void PIT8254::writeControl(uint8_t value) {
  const int sel = value >> 6;
  if (sel == 3) { // read-back command
    const bool latchCount = (value & 0x20) == 0;
    const bool latchStatus = (value & 0x10) == 0;
    for (int i = 0; i < 3; ++i) {
      if (!(value & (2 << i)))
        continue;
      auto &c = m_channels[i];
      if (latchCount && !c.countLatched) {
        c.countLatched = true;
        c.latch = visibleCount(c);
      }
      if (latchStatus && !c.statusLatched) {
        c.statusLatched = true;
        c.status = statusByte(c);
      }
    }
    return;
  }

  auto &c = m_channels[sel];
  const uint8_t rw = (value >> 4) & 3;
  if (rw == 0) { // counter latch command; ignored while a latch is unread
    if (!c.countLatched) {
      c.countLatched = true;
      c.latch = visibleCount(c);
    }
    return;
  }

  const bool wasHigh = c.out;
  c.modeBits = (value >> 1) & 7;
  c.mode = c.modeBits >= 6 ? static_cast<uint8_t>(c.modeBits - 4) : c.modeBits;
  c.rw = rw;
  c.bcd = (value & 1) != 0;
  c.writeMsbNext = false;
  c.readMsbNext = false;
  c.countLatched = false;
  c.statusLatched = false;
  c.crWritten = false;
  c.nullCount = true;
  c.loadPending = false;
  c.armed = false;
  c.done = false;
  c.out = c.mode != 0; // mode 0 starts low, every other mode high
  noteEdge(sel, wasHigh);
}

void PIT8254::writeCount(int index, uint8_t value) {
  auto &c = m_channels[index];
  switch (c.rw) {
  case 1:
    c.cr = value;
    break;
  case 2:
    c.cr = static_cast<uint16_t>(value << 8);
    break;
  default:
    if (!c.writeMsbNext) {
      c.writeLsb = value;
      c.writeMsbNext = true;
      if (c.mode == 0) { // the first byte stops a mode 0 count
        c.armed = false;
        c.loadPending = false;
        c.out = false;
      }
      return;
    }
    c.writeMsbNext = false;
    c.cr = static_cast<uint16_t>(c.writeLsb | (value << 8));
    break;
  }
  countComplete(index);
}

void PIT8254::countComplete(int index) {
  auto &c = m_channels[index];
  c.crWritten = true;
  c.nullCount = true;
  switch (c.mode) {
  case 0: // (re)start: OUT low, load on the next CLK
    c.out = false;
    c.armed = false;
    c.loadPending = true;
    break;
  case 1:
  case 5: // used by the next gate trigger
    break;
  case 2:
  case 3: // first count loads now; later ones at the end of the cycle
    if (!c.armed)
      c.loadPending = true;
    break;
  default: // 4: a new count restarts the strobe
    c.loadPending = true;
    break;
  }
  if (index == 2 && m_ch2Callback)
    m_ch2Callback(static_cast<uint16_t>(initialCount(c)), c.mode);
}

void PIT8254::setGate(int channel, bool level) {
  if (channel < 0 || channel > 2)
    return;
  sync();
  auto &c = m_channels[channel];
  if (c.gate == level)
    return;
  c.gate = level;
  const bool wasHigh = c.out;
  if (level) {
    // Rising edge: triggers modes 1 and 5, restarts modes 2 and 3.
    if (c.crWritten && (c.mode == 1 || c.mode == 2 || c.mode == 3 ||
                        c.mode == 5))
      c.loadPending = true;
  } else if (c.mode == 2 || c.mode == 3) {
    c.out = true; // GATE low forces OUT high at once
  }
  noteEdge(channel, wasHigh);
  recomputeNextIrq();
}

bool PIT8254::output(int channel) {
  if (channel < 0 || channel > 2)
    return false;
  sync();
  return m_channels[channel].out;
}

uint8_t PIT8254::port61Status() {
  sync();
  uint8_t v = 0;
  if ((m_lastTick / kRefreshTicks) & 1)
    v |= 0x10;
  if (m_channels[2].out)
    v |= 0x20;
  return v;
}

} // namespace fador::hw
