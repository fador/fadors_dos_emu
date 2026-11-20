#include "VGAController.hpp"
#include "../utils/Logger.hpp"

#include <algorithm>
#include <chrono>

namespace fador::hw {

namespace {

uint8_t rotateRight(uint8_t value, unsigned count) {
  count &= 7;
  if (count == 0)
    return value;
  return static_cast<uint8_t>((value >> count) | (value << (8 - count)));
}

uint8_t expandPlaneBit(uint8_t value, int plane) {
  return (value & (1u << plane)) != 0 ? 0xFF : 0x00;
}

uint8_t applyLogicalOp(uint8_t op, uint8_t input, uint8_t latch) {
  switch (op & 0x03) {
  case 0x01: return input & latch;
  case 0x02: return input | latch;
  case 0x03: return input ^ latch;
  default: return input;
  }
}

uint64_t steadyNowNs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

// Writable bits of each register (reserved bits read back as zero).
constexpr std::array<uint8_t, 5> kSeqMask{0x03, 0x3D, 0x0F, 0x3F, 0x0E};
constexpr std::array<uint8_t, 9> kGcMask{0x0F, 0x0F, 0x0F, 0x1F, 0x03,
                                         0x7B, 0x0F, 0x0F, 0xFF};

} // namespace

VGAController::VGAController(memory::MemoryBus &memory)
    : m_memory(memory), m_epochNs(steadyNowNs()) {
  updateAccessPath();
}

// ── Time source and CRT timing ─────────────────────────────────────────

void VGAController::setTimeSource(std::function<uint64_t()> nowNs) {
  m_timeSource = std::move(nowNs);
}

uint64_t VGAController::nowNs() const {
  if (m_timeSource)
    return m_timeSource();
  return steadyNowNs() - m_epochNs;
}

uint32_t VGAController::verticalTotal() const {
  return m_crtc[0x06] | ((m_crtc[0x07] & 0x01u) << 8) |
         ((m_crtc[0x07] & 0x20u) << 4);
}

uint32_t VGAController::verticalDisplayEnd() const {
  return m_crtc[0x12] | ((m_crtc[0x07] & 0x02u) << 7) |
         ((m_crtc[0x07] & 0x40u) << 3);
}

uint32_t VGAController::verticalRetraceStart() const {
  return m_crtc[0x10] | ((m_crtc[0x07] & 0x04u) << 6) |
         ((m_crtc[0x07] & 0x80u) << 2);
}

uint32_t VGAController::lineCompare() const {
  return m_crtc[0x18] | ((m_crtc[0x07] & 0x10u) << 4) |
         ((m_crtc[0x09] & 0x40u) << 3);
}

void VGAController::recomputeTiming() const {
  m_timingDirty = false;
  VgaTiming t{}; // defaults: 70 Hz, 449 lines (mode 3)

  const uint32_t htotal = m_crtc[0x00] + 5u;
  const uint32_t hdisp = m_crtc[0x01] + 1u;
  const uint32_t vtotal = verticalTotal() + 2u;
  if (m_crtc[0x00] != 0 && vtotal > 2) {
    const uint32_t dots = (m_seq[1] & 0x01) ? 8u : 9u;
    // Misc output bits 2-3: 0 = 25.175 MHz, 1 = 28.322 MHz
    uint64_t clockHz = ((m_misc >> 2) & 3) == 1 ? 28322000u : 25175000u;
    uint64_t dotsPerLine = static_cast<uint64_t>(htotal) * dots;
    if (m_seq[1] & 0x08) // dot clock / 2
      dotsPerLine *= 2;
    t.linePs = dotsPerLine * 1000000000000ull / clockHz;
    t.displayPs = t.linePs * std::min(hdisp, htotal) / htotal;
    t.totalLines = vtotal;
    t.displayLines = std::min(verticalDisplayEnd() + 1u, vtotal);
    t.retraceStart = verticalRetraceStart();
    if (t.retraceStart >= vtotal)
      t.retraceStart = t.displayLines < vtotal ? t.displayLines : 0;
    // Retrace ends when the low four bits of the line counter match
    // CRTC 11h bits 0-3.
    uint32_t len = ((m_crtc[0x11] & 0x0Fu) - (t.retraceStart & 0x0Fu)) & 0x0Fu;
    t.retraceLines = len ? len : 16u;
  }
  t.frameNs = t.linePs * t.totalLines / 1000u;
  if (t.frameNs == 0)
    t.frameNs = 1;
  m_timing = t;
}

uint32_t VGAController::currentScanline() const {
  const VgaTiming &t = timing();
  const uint64_t pos = nowNs() % t.frameNs;
  return static_cast<uint32_t>(std::min<uint64_t>(pos * 1000u / t.linePs,
                                                  t.totalLines - 1));
}

uint64_t VGAController::frameCounter() const {
  return nowNs() / timing().frameNs;
}

uint8_t VGAController::inputStatus0() {
  const uint64_t now = nowNs();
  if (now >= m_vintNextNs)
    serviceEvents(now);
  return static_cast<uint8_t>((m_vintPending ? 0x80 : 0x00) | 0x10);
}

// ── Frame capture and retrace interrupt ────────────────────────────────

void VGAController::requestFrame() {
  if (!m_sink)
    return;
  if (!m_wantFrame) {
    m_wantFrame = true;
    m_requestNs = nowNs();
  }
  m_rasterActive = true;
  m_rasterNextNs = 0; // the next advanceTo() schedules the capture
  updateNextEvent();
}

void VGAController::setContinuousCapture(bool on) {
  m_continuous = on;
  if (on)
    requestFrame();
  else
    m_wantFrame = false;
}

void VGAController::rasterSync(uint64_t now) {
  if (!m_sink) {
    m_rasterActive = false;
    m_rasterNextNs = UINT64_MAX;
    updateNextEvent();
    return;
  }
  const VgaTiming &t = timing();
  const uint64_t frame = now / t.frameNs;
  auto finishCapture = [&] {
    m_capturing = false;
    if (m_continuous) {
      // The following frame is captured too.  Nothing changed since the
      // captured frame's display ended (every change syncs first), so a
      // capture that starts late in that frame is still exact.
      m_wantFrame = true;
      m_requestNs = (m_capFrame + 1) * t.frameNs;
    }
    m_sink->endFrame();
  };
  if (m_capturing && frame != m_capFrame)
    finishCapture();
  if (!m_capturing) {
    const uint64_t start = frame * t.frameNs;
    if (!m_wantFrame) {
      m_rasterActive = false;
      m_rasterNextNs = UINT64_MAX;
      updateNextEvent();
      return;
    }
    if (start < m_requestNs) {
      // This frame began before the request: its earlier lines may have
      // been shown with a state nobody recorded.  Capture the next one.
      m_rasterNextNs = start + t.frameNs;
      updateNextEvent();
      return;
    }
    m_capturing = true;
    m_capFrame = frame;
    if (!m_continuous)
      m_wantFrame = false;
    m_sink->beginFrame(frame);
  }
  // Lines whose active display the beam has finished
  const uint64_t posPs = (now - frame * t.frameNs) * 1000u;
  uint64_t line = posPs / t.linePs;
  if (posPs % t.linePs >= t.displayPs)
    ++line;
  if (line >= t.displayLines) {
    finishCapture();
    m_rasterActive = m_wantFrame;
    m_rasterNextNs = m_wantFrame ? (frame + 1) * t.frameNs : UINT64_MAX;
  } else {
    m_sink->scanTo(static_cast<uint32_t>(line));
    const uint64_t endPs = (t.displayLines - 1u) * t.linePs + t.displayPs;
    m_rasterNextNs = frame * t.frameNs + (endPs + 999u) / 1000u;
  }
  updateNextEvent();
}

void VGAController::rescheduleRetraceInterrupt(uint64_t now) {
  if (!(m_crtc[0x11] & 0x10)) {
    // Clear Vertical Interrupt = 0 holds the latch cleared
    m_vintPending = false;
    m_vintNextNs = UINT64_MAX;
  } else if (!m_vintPending) {
    const VgaTiming &t = timing();
    uint64_t start = frameStartNs(now) + t.retraceStart * t.linePs / 1000u;
    if (start <= now)
      start += t.frameNs;
    m_vintNextNs = start;
  }
  updateNextEvent();
}

void VGAController::serviceEvents(uint64_t now) {
  if (now >= m_rasterNextNs)
    rasterSync(now);
  if (now >= m_vintNextNs) {
    // Start of vertical retrace: set the latch; the IRQ line follows it
    // unless Disable Vertical Interrupt (CRTC 11h bit 5) is set.  The
    // latch stays set (no further edges) until the program clears it.
    m_vintNextNs = UINT64_MAX;
    if ((m_crtc[0x11] & 0x10) && !m_vintPending) {
      m_vintPending = true;
      if (!(m_crtc[0x11] & 0x20) && m_raiseIrq)
        m_raiseIrq();
    }
  }
  updateNextEvent();
}

uint8_t VGAController::inputStatus1() const {
  const VgaTiming &t = timing();
  const uint64_t posPs = (nowNs() % t.frameNs) * 1000u;
  const uint32_t line = static_cast<uint32_t>(
      std::min<uint64_t>(posPs / t.linePs, t.totalLines - 1));
  const uint64_t inLine = posPs % t.linePs;

  uint32_t sinceRetrace = (line + t.totalLines - t.retraceStart) % t.totalLines;
  const bool vretrace = sinceRetrace < t.retraceLines;
  const bool displayDisabled =
      vretrace || line >= t.displayLines || inLine >= t.displayPs;
  return static_cast<uint8_t>((vretrace ? 0x08 : 0x00) |
                              (displayDisabled ? 0x01 : 0x00));
}

// ── Register helpers ───────────────────────────────────────────────────

bool VGAController::crtcPortActive(uint16_t port) const {
  // Misc output bit 0 selects colour (3Dxh) or monochrome (3Bxh) decoding
  return ((port & 0xFFF0) == 0x3D0) != monoAddressing();
}

void VGAController::setSeqReg(uint8_t index, uint8_t value) {
  if (index >= m_seq.size())
    return;
  value &= kSeqMask[index];
  if ((index == 1 || index == 3) && m_seq[index] != value)
    beforeDisplayChange(); // clocking mode / character map select
  m_seq[index] = value;
  if (index == 1)
    timingChanged();
  if (index == 4 || index == 2)
    updateAccessPath();
}

void VGAController::setGcReg(uint8_t index, uint8_t value) {
  if (index >= m_gc.size())
    return;
  value &= kGcMask[index];
  // Shift register modes (05h bits 5/6) and the memory map / graphics
  // bits (06h) change what the display shows.
  if ((index == 5 && ((m_gc[5] ^ value) & 0x60)) ||
      (index == 6 && m_gc[6] != value))
    beforeDisplayChange();
  m_gc[index] = value;
  updateAccessPath();
}

void VGAController::setCrtcReg(uint8_t index, uint8_t value) {
  if (index >= m_crtc.size())
    return;
  if (m_crtc[index] == value)
    return;
  beforeDisplayChange();
  m_crtc[index] = value;
  if (index == 0x11 && !(m_crtc[0x11] & 0x10))
    rescheduleRetraceInterrupt(0); // Clear Vertical Interrupt
  // Registers recomputeTiming() reads
  if (index <= 0x07 || (index >= 0x10 && index <= 0x12) || index == 0x15 ||
      index == 0x16)
    timingChanged();
}

void VGAController::timingChanged() {
  m_timingDirty = true;
  // The next retrace and frame boundary move with the timing
  if (m_crtc[0x11] & 0x10)
    rescheduleRetraceInterrupt(nowNs());
  if (m_rasterActive) {
    m_rasterNextNs = 0;
    updateNextEvent();
  }
}

void VGAController::writeCrtc(uint8_t index, uint8_t value) {
  if (index >= m_crtc.size())
    return;
  if ((m_crtc[0x11] & 0x80) && index <= 0x07) {
    // Protect bit: registers 0-7 are read-only, except the line compare
    // bit 8 in the overflow register.
    if (index == 0x07)
      setCrtcReg(0x07, static_cast<uint8_t>((m_crtc[0x07] & ~0x10) | (value & 0x10)));
    return;
  }
  setCrtcReg(index, value);
}

void VGAController::setAttrReg(uint8_t index, uint8_t value) {
  static constexpr std::array<uint8_t, 5> kAttrCtlMask{0xEF, 0xFF, 0x3F,
                                                       0x0F, 0x0F};
  if (index >= m_attr.size())
    return;
  value &= index < 0x10 ? 0x3F : kAttrCtlMask[index - 0x10];
  if (m_attr[index] != value) {
    beforeDisplayChange();
    m_attr[index] = value;
  }
}

void VGAController::setAttrPaletteSource(bool enabled) {
  const uint8_t index =
      static_cast<uint8_t>((m_attrIndex & 0x1F) | (enabled ? 0x20 : 0));
  if ((index ^ m_attrIndex) & 0x20)
    beforeDisplayChange();
  m_attrIndex = index;
}

void VGAController::setPelMask(uint8_t value) {
  if (m_pelMask != value) {
    beforeDisplayChange();
    m_pelMask = value;
  }
}

void VGAController::writeAttr(uint8_t value) {
  if (!m_attrFlipFlop) {
    if ((m_attrIndex ^ value) & 0x20)
      beforeDisplayChange(); // palette address source: blanking
    m_attrIndex = value & 0x3F;
  } else {
    uint8_t index = m_attrIndex & 0x1F;
    // The internal palette can only be loaded while the palette address
    // source bit is clear (display blanked), as on IBM VGA.
    if (index >= 0x10 || !attrPaletteSourceEnabled())
      setAttrReg(index, value);
  }
  m_attrFlipFlop = !m_attrFlipFlop;
}

void VGAController::setMiscOutput(uint8_t value) {
  if (m_misc == value)
    return;
  beforeDisplayChange();
  m_misc = value;
  timingChanged();
}

void VGAController::updateAccessPath() {
  const uint8_t map = (m_gc[6] >> 2) & 3;
  const bool gcIdentity = m_gc[1] == 0 && m_gc[3] == 0 &&
                          (m_gc[5] & 0x03) == 0 && m_gc[8] == 0xFF;
  if (m_svgaWindowEnabled)
    m_path = AccessPath::Svga;
  else if (map >= 2) // B0000h / B8000h: the A0000h window is not decoded
    m_path = AccessPath::Disabled;
  else if (isChain4())
    m_path = gcIdentity ? AccessPath::Chain4Fast : AccessPath::Chain4;
  else if (!(m_seq[4] & 0x04) && (m_gc[5] & 0x10))
    m_path = AccessPath::OddEven;
  else
    m_path = AccessPath::Planar;
}

// ── Port I/O ───────────────────────────────────────────────────────────

uint8_t VGAController::read8(uint16_t port) {
  switch (port) {
  // --- Attribute controller ---
  case 0x3C0: return m_attrIndex;
  case 0x3C1: {
    uint8_t index = m_attrIndex & 0x1F;
    return index < m_attr.size() ? m_attr[index] : 0;
  }

  // --- Input status 0 (bit 4: switch sense / colour monitor present) ---
  case 0x3C2: return inputStatus0();
  case 0x3C3: return m_videoEnable;

  // --- Sequencer ---
  case 0x3C4: return m_seqIndex;
  case 0x3C5: return m_seqIndex < m_seq.size() ? m_seq[m_seqIndex] : 0;

  // --- DAC ---
  case 0x3C6: return m_pelMask;
  case 0x3C7: return m_dacState;
  case 0x3C8: return m_dacWriteIndex;
  case 0x3C9: {
    uint8_t val = m_memory.dacPalette()[m_dacReadIndex * 3 + m_dacColorCycle];
    if (++m_dacColorCycle >= 3) {
      m_dacColorCycle = 0;
      m_dacReadIndex++;
    }
    return val;
  }

  case 0x3CA: return m_featureControl;
  case 0x3CC: return m_misc;

  // --- Graphics Controller ---
  case 0x3CE: return m_gcIndex;
  case 0x3CF: return m_gcIndex < m_gc.size() ? m_gc[m_gcIndex] : 0;

  // --- CRTC ---
  case 0x3B4:
  case 0x3D4:
    return crtcPortActive(port) ? m_crtcIndex : 0xFF;
  case 0x3B5:
  case 0x3D5:
    if (!crtcPortActive(port))
      return 0xFF;
    return m_crtcIndex < m_crtc.size() ? m_crtc[m_crtcIndex] : 0;

  // --- Input status 1 (resets the attribute controller flip-flop) ---
  case 0x3BA:
  case 0x3DA:
    if (!crtcPortActive(port))
      return 0xFF;
    m_attrFlipFlop = false;
    return inputStatus1();

  default:
    LOG_TRACE("VGA: Read port 0x", std::hex, port);
    return 0;
  }
}

void VGAController::write8(uint16_t port, uint8_t value) {
  switch (port) {
  case 0x3C0: writeAttr(value); break;
  case 0x3C2: setMiscOutput(value); break;
  case 0x3C3: m_videoEnable = value & 0x01; break;

  // --- Sequencer ---
  case 0x3C4: m_seqIndex = value; break;
  case 0x3C5: setSeqReg(m_seqIndex, value); break;

  // --- DAC ---
  case 0x3C6: setPelMask(value); break;
  case 0x3C7:
    m_dacReadIndex = value;
    m_dacWriteIndex = static_cast<uint8_t>(value + 1);
    m_dacColorCycle = 0;
    m_dacState = 0x03;
    break;
  case 0x3C8:
    m_dacWriteIndex = value;
    m_dacReadIndex = static_cast<uint8_t>(value - 1);
    m_dacColorCycle = 0;
    m_dacState = 0x00;
    break;
  case 0x3C9: {
    uint8_t &entry = m_memory.dacPalette()[m_dacWriteIndex * 3 + m_dacColorCycle];
    if (entry != (value & 0x3F)) {
      beforeDisplayChange();
      entry = value & 0x3F;
    }
    if (++m_dacColorCycle >= 3) {
      m_dacColorCycle = 0;
      m_dacWriteIndex++;
    }
    break;
  }

  // --- Graphics Controller ---
  case 0x3CE: m_gcIndex = value; break;
  case 0x3CF: setGcReg(m_gcIndex, value); break;

  // --- CRTC ---
  case 0x3B4:
  case 0x3D4:
    if (crtcPortActive(port))
      m_crtcIndex = value;
    break;
  case 0x3B5:
  case 0x3D5:
    if (crtcPortActive(port))
      writeCrtc(m_crtcIndex, value);
    break;

  // --- Feature control ---
  case 0x3BA:
  case 0x3DA:
    if (crtcPortActive(port))
      m_featureControl = value;
    break;

  default:
    LOG_TRACE("VGA: Write port 0x", std::hex, port, " = 0x", (int)value);
    break;
  }
}

// ── Plane-aware VRAM access ────────────────────────────────────────────

void VGAController::loadLatches(uint32_t planeOffset) const {
  uint32_t index = planeOffset & 0xFFFF;
  for (int plane = 0; plane < 4; ++plane)
    m_latches[plane] = m_planes[plane][index];
}

void VGAController::writeThroughPipeline(uint32_t planeOff, uint8_t value,
                                         uint8_t planeMask) {
  planeOff &= 0xFFFF;
  const uint8_t logicalOp = (m_gc[3] >> 3) & 0x03;
  const uint8_t bitMask = m_gc[8];
  const uint8_t setReset = m_gc[0];

  // Result = (ALU output & mask) | (latch & ~mask), per enabled plane
  auto commit = [&](int plane, uint8_t data, uint8_t mask) {
    if ((planeMask & (1u << plane)) == 0)
      return;
    const uint8_t latch = m_latches[plane];
    m_planes[plane][planeOff] =
        static_cast<uint8_t>((applyLogicalOp(logicalOp, data, latch) & mask) |
                             (latch & ~mask));
  };

  switch (m_gc[5] & 0x03) {
  case 0x00: {
    const uint8_t rotated = rotateRight(value, m_gc[3] & 0x07);
    for (int plane = 0; plane < 4; ++plane) {
      uint8_t source = (m_gc[1] & (1u << plane)) != 0
                           ? expandPlaneBit(setReset, plane)
                           : rotated;
      commit(plane, source, bitMask);
    }
    break;
  }
  case 0x01:
    // Latch copy: the CPU byte, the logical function and the bit mask
    // are all ignored.
    for (int plane = 0; plane < 4; ++plane)
      if (planeMask & (1u << plane))
        m_planes[plane][planeOff] = m_latches[plane];
    break;
  case 0x02:
    for (int plane = 0; plane < 4; ++plane)
      commit(plane, expandPlaneBit(value, plane), bitMask);
    break;
  case 0x03: {
    // The rotated CPU byte ANDed with the bit mask becomes the mask;
    // set/reset supplies the data.
    const uint8_t mask = rotateRight(value, m_gc[3] & 0x07) & bitMask;
    for (int plane = 0; plane < 4; ++plane)
      commit(plane, expandPlaneBit(setReset, plane), mask);
    break;
  }
  }
}

uint8_t VGAController::readThroughPipeline(uint32_t planeOff, int plane) const {
  loadLatches(planeOff);
  if ((m_gc[5] & 0x08) == 0)
    return m_latches[plane & 3];

  // Read mode 1: colour compare.  A bit is 1 where every plane enabled in
  // Color Don't Care matches the corresponding Color Compare bit.
  uint8_t result = 0xFF;
  for (int p = 0; p < 4; ++p) {
    if ((m_gc[7] & (1u << p)) == 0)
      continue;
    const uint8_t want = (m_gc[2] & (1u << p)) ? 0xFF : 0x00;
    result &= static_cast<uint8_t>(~(m_latches[p] ^ want));
  }
  return result;
}

void VGAController::planeWrite8(uint32_t offset, uint8_t value) {
  switch (m_path) {
  case AccessPath::Chain4Fast: {
    const uint32_t plane = offset & 3;
    if (m_seq[2] & (1u << plane))
      m_planes[plane][offset & 0xFFFC] = value;
    return;
  }
  case AccessPath::Svga:
    if (uint8_t *p = m_memory.directAccess(m_svgaWindowBase + (offset & 0xFFFF)))
      *p = value;
    return;
  case AccessPath::Disabled:
    return;
  case AccessPath::Chain4:
    writeThroughPipeline(offset & 0xFFFC, value,
                         static_cast<uint8_t>(m_seq[2] & (1u << (offset & 3))));
    return;
  case AccessPath::OddEven:
    writeThroughPipeline(offset & 0xFFFE, value,
                         static_cast<uint8_t>(m_seq[2] & ((offset & 1) ? 0x0A : 0x05)));
    return;
  case AccessPath::Planar:
    writeThroughPipeline(offset & 0xFFFF, value, m_seq[2]);
    return;
  }
}

uint8_t VGAController::planeRead8(uint32_t offset) const {
  switch (m_path) {
  case AccessPath::Svga: {
    const uint8_t *p = m_memory.directAccess(m_svgaWindowBase + (offset & 0xFFFF));
    return p ? *p : 0xFF;
  }
  case AccessPath::Disabled:
    return 0xFF;
  case AccessPath::Chain4Fast:
  case AccessPath::Chain4:
    return readThroughPipeline(offset & 0xFFFC, static_cast<int>(offset & 3));
  case AccessPath::OddEven:
    return readThroughPipeline(offset & 0xFFFE,
                               (m_gc[4] & 2) | static_cast<int>(offset & 1));
  case AccessPath::Planar:
  default:
    return readThroughPipeline(offset & 0xFFFF, m_gc[4] & 3);
  }
}

// ── BIOS HLE helper ────────────────────────────────────────────────────

void syncCursorFromBDA(memory::MemoryBus &memory) {
  VGAController *vga = memory.getVGA();
  if (!vga)
    return;
  const uint8_t mode = memory.read8(0x449);
  if (!(mode <= 0x03 || mode == 0x07))
    return;
  const uint8_t page = memory.read8(0x462) & 7;
  const uint16_t cols = memory.read16(0x44A);
  const uint8_t col = memory.read8(0x450 + page * 2);
  const uint8_t row = memory.read8(0x451 + page * 2);
  const uint32_t loc = memory.read16(0x44E) / 2u + row * uint32_t(cols) + col;
  vga->setCrtcReg(0x0E, static_cast<uint8_t>(loc >> 8));
  vga->setCrtcReg(0x0F, static_cast<uint8_t>(loc));
}

} // namespace fador::hw
