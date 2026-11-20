#include "GravisUltrasound.hpp"
#include "../../utils/Logger.hpp"
#include "../../utils/simd/Simd.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <numbers>

namespace fador::hw::audio {

namespace {

// Voice control (00h) and volume control (0Dh) bits.
constexpr uint8_t kStopped = 0x01;   // read: stopped
constexpr uint8_t kStop = 0x02;      // write: stop
constexpr uint8_t kData16 = 0x04;    // voice: 16-bit data; volume: rollover
constexpr uint8_t kLoop = 0x08;
constexpr uint8_t kBidir = 0x10;
constexpr uint8_t kIrqEnable = 0x20;
constexpr uint8_t kDecreasing = 0x40;
constexpr uint8_t kIrqPending = 0x80;

// 2X6 IRQ status bits.
constexpr uint8_t kIrqTimer1 = 0x04;
constexpr uint8_t kIrqTimer2 = 0x08;
constexpr uint8_t kIrqWave = 0x20;
constexpr uint8_t kIrqRamp = 0x40;
constexpr uint8_t kIrqDmaTc = 0x80;

// IRQ / DMA latch selector values (2XB).
constexpr std::array<int, 8> kLatchIrq{0, 2, 5, 3, 7, 11, 12, 15};
constexpr std::array<int, 8> kLatchDma{0, 1, 3, 5, 6, 7, 0, 0};

constexpr uint32_t kChunk = 512;          // GF1 ticks rendered at a time
constexpr size_t kMaxFifoFrames = 65536;  // > 1 s at any GF1 rate
// One full-scale voice at maximum volume panned to one side: -6 dBFS.
constexpr float kOutputGain = 0.5f / 32768.0f;
constexpr uint64_t kDmaClockHz = 650'000; // DMA request rate, divisor 1

double besselI0(double x) {
  double sum = 1.0, term = 1.0;
  for (int k = 1; k < 32; ++k) {
    term *= (x / (2.0 * k)) * (x / (2.0 * k));
    sum += term;
    if (term < 1e-12 * sum)
      break;
  }
  return sum;
}

// Pan: 16 positions at constant power, 0 = full left, 7 = centre (-3 dB
// per side), 15 = full right.
struct PanGains {
  std::array<float, 16> left{}, right{};
};
const PanGains &panGains() {
  static const PanGains gains = [] {
    PanGains g;
    for (int p = 0; p < 16; ++p) {
      const double norm = (p - 7.0) / (p < 7 ? 7.0 : 8.0);
      const double angle = (norm + 1.0) * std::numbers::pi / 4.0;
      g.left[static_cast<size_t>(p)] = static_cast<float>(std::cos(angle));
      g.right[static_cast<size_t>(p)] = static_cast<float>(std::sin(angle));
    }
    g.left[15] = 0.0f; // cos(pi/2) is not exactly 0
    return g;
  }();
  return gains;
}

const std::array<float, 4096> &volumeTable() {
  static const std::array<float, 4096> table = [] {
    std::array<float, 4096> t{};
    for (uint32_t v = 1; v < 4096; ++v) {
      // 4-bit exponent, 8-bit mantissa: (1 + m/256) * 2^(e - 16).
      const uint32_t e = v >> 8, m = v & 0xFF;
      t[v] = static_cast<float>(
          std::ldexp((256.0 + static_cast<double>(m)) / 256.0,
                     static_cast<int>(e) - 16));
    }
    return t;
  }();
  return table;
}

} // namespace

// ── Configuration ─────────────────────────────────────────────────────

bool parseGusSpec(const std::string &spec, GusConfig &config,
                  std::string &error) {
  size_t pos = 0;
  bool first = true;
  while (pos <= spec.size() && !spec.empty()) {
    const size_t comma = spec.find(',', pos);
    const std::string item =
        spec.substr(pos, comma == std::string::npos ? std::string::npos
                                                    : comma - pos);
    pos = comma == std::string::npos ? spec.size() + 1 : comma + 1;
    if (item.empty())
      continue;
    const size_t eq = item.find('=');
    std::string key = eq == std::string::npos ? "" : item.substr(0, eq);
    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    const std::string value = eq == std::string::npos ? item : item.substr(eq + 1);
    char *end = nullptr;
    const bool hex = key.empty() || key == "port";
    const unsigned long n = std::strtoul(value.c_str(), &end, hex ? 16 : 10);
    if (value.empty() || *end != '\0') {
      error = "bad value in '" + item + "'";
      return false;
    }
    if (hex) {
      if (!first && key.empty()) {
        error = "unexpected '" + item + "'";
        return false;
      }
      if (n < 0x210 || n > 0x260 || (n & 0x0F) != 0) {
        error = "port must be 210..260 (hex)";
        return false;
      }
      config.basePort = static_cast<uint16_t>(n);
    } else if (key == "irq" || key == "midiirq") {
      if (std::find(kLatchIrq.begin() + 1, kLatchIrq.end(),
                    static_cast<int>(n)) == kLatchIrq.end() && n != 9) {
        error = "IRQ must be 2/9, 3, 5, 7, 11, 12 or 15";
        return false;
      }
      (key == "irq" ? config.irq : config.midiIrq) =
          n == 9 ? 2 : static_cast<int>(n);
      if (key == "irq")
        config.midiIrq = config.irq;
    } else if (key == "dma" || key == "rdma") {
      if (n == 0 || std::find(kLatchDma.begin() + 1, kLatchDma.begin() + 6,
                              static_cast<int>(n)) == kLatchDma.begin() + 6) {
        error = "DMA must be 1, 3, 5, 6 or 7";
        return false;
      }
      (key == "dma" ? config.dma : config.recordDma) = static_cast<int>(n);
      if (key == "dma")
        config.recordDma = config.dma;
    } else if (key == "mem") {
      if (n != 256 && n != 512 && n != 768 && n != 1024) {
        error = "mem must be 256, 512, 768 or 1024 (KB)";
        return false;
      }
      config.dramKB = static_cast<uint32_t>(n);
    } else {
      error = "unknown option '" + key + "'";
      return false;
    }
    first = false;
  }
  return true;
}

std::string ultrasndVariable(const GusConfig &config) {
  char buf[48];
  std::snprintf(buf, sizeof buf, "%X,%d,%d,%d,%d", config.basePort,
                config.dma, config.recordDma, config.irq, config.midiIrq);
  return buf;
}

// ── Construction ──────────────────────────────────────────────────────

GravisUltrasound::GravisUltrasound(memory::MemoryBus &memory, DMA8237 &dma,
                                   const GusConfig &config, float outputRate)
    : m_memory(memory), m_dma(dma), m_config(config),
      m_outputRate(outputRate),
      m_dramSize(std::clamp<uint32_t>(config.dramKB, 256, 1024) * 1024),
      m_dram(0x100000, 0), m_irqLine(config.irq), m_dmaChannel(config.dma) {
  // The power-on state after ULTRINIT: latches programmed, output on.
  for (size_t i = 1; i < kLatchIrq.size(); ++i)
    if (kLatchIrq[i] == config.irq)
      m_irqLatch = static_cast<uint8_t>(i);
  for (size_t i = 1; i < 6; ++i)
    if (kLatchDma[i] == config.dma)
      m_dmaLatch = static_cast<uint8_t>(i);
  m_timers[0].unitNs = 80'000;
  m_timers[1].unitNs = 320'000;
  m_maskListener = m_dma.addMaskListener(
      [this](int channel, bool masked) { dmaMaskChanged(channel, masked); });
  resetGf1();
  LOG_INFO("Gravis Ultrasound at ", std::hex, m_config.basePort, std::dec,
           "h IRQ=", m_config.irq, " DMA=", m_config.dma, " DRAM=",
           m_dramSize / 1024, " KB");
}

GravisUltrasound::~GravisUltrasound() { m_dma.removeMaskListener(m_maskListener); }

void GravisUltrasound::setTimeSource(std::function<uint64_t()> nowNs) {
  m_nowNs = std::move(nowNs);
  m_lastNs = now();
  m_epochNs = m_lastNs;
  m_epochTick = m_ticksDone;
  invalidateEvents();
}

void GravisUltrasound::resetGf1() {
  for (auto &v : m_voices) {
    v = Voice{};
    updateGain(v);
  }
  m_wavePending = m_rampPending = 0;
  m_queueLen = 0;
  m_irqFlags = 0;
  m_dmaCtrl = 0;
  m_dmaTcPending = false;
  m_dmaState = DmaState::Idle;
  m_timerCtrl = 0;
  m_sampleCtrl = 0;
  for (auto &t : m_timers) {
    t.running = false;
    t.reached = false;
  }
  setActiveVoices(kMinVoices, m_lastNs);
  invalidateEvents();
}

// ── Emulated time ─────────────────────────────────────────────────────

uint64_t GravisUltrasound::ticksDue(uint64_t nowNs) {
  // In `voices` seconds there are exactly 617400 ticks: rebase the epoch
  // by that span so the products below stay far from overflow.
  const uint64_t span = static_cast<uint64_t>(m_activeVoices) * 1'000'000'000;
  while (nowNs - m_epochNs >= span) {
    m_epochNs += span;
    m_epochTick += kGf1Clock;
  }
  return m_epochTick + (nowNs - m_epochNs) * kGf1Clock / span;
}

uint64_t GravisUltrasound::tickTimeNs(uint64_t tick) const {
  if (tick <= m_epochTick)
    return m_epochNs;
  const uint64_t d = std::min<uint64_t>(tick - m_epochTick, uint64_t{1} << 26);
  const uint64_t span = static_cast<uint64_t>(m_activeVoices) * 1'000'000'000;
  return m_epochNs + (d * span + kGf1Clock - 1) / kGf1Clock;
}

void GravisUltrasound::setActiveVoices(int voices, uint64_t nowNs) {
  voices = std::clamp(voices, kMinVoices, kMaxVoices);
  if (voices == m_activeVoices && m_epochTick == m_ticksDone &&
      m_epochNs == nowNs)
    return;
  m_activeVoices = voices;
  m_epochNs = nowNs;
  m_epochTick = m_ticksDone;
  invalidateEvents();
}

void GravisUltrasound::sync(uint64_t nowNs) {
  if (nowNs < m_lastNs)
    nowNs = m_lastNs;
  m_lastNs = nowNs;
  const uint64_t due = ticksDue(nowNs);
  if (due > m_ticksDone)
    runTicks(due - m_ticksDone);
  if (m_dmaState == DmaState::Running)
    runDma(nowNs);
  if (m_timers[0].running || m_timers[1].running)
    runTimers(nowNs);
}

void GravisUltrasound::service(uint64_t nowNs) {
  sync(nowNs);
  computeNextEvent();
}

uint64_t GravisUltrasound::nextEventNs() {
  if (m_eventsDirty)
    computeNextEvent();
  return m_wakeNs;
}

void GravisUltrasound::computeNextEvent() {
  uint64_t best = kNever;
  if (m_reset & 0x01) {
    uint64_t ticks = kNever;
    for (int i = 0; i < m_activeVoices; ++i)
      ticks = std::min(ticks, ticksToVoiceEvent(m_voices[static_cast<size_t>(i)]));
    if (ticks != kNever)
      best = tickTimeNs(m_ticksDone + ticks);
  }
  for (size_t i = 0; i < m_timers.size(); ++i)
    if (m_timers[i].running && (m_timerCtrl & (kIrqTimer1 << i)))
      best = std::min(best, m_timers[i].nextNs);
  if (m_dmaState == DmaState::Running)
    best = std::min(best, dmaCompletionNs());
  m_wakeNs = best;
  m_nextEventNs = best;
  m_eventsDirty = false;
}

// ── Port I/O ──────────────────────────────────────────────────────────

uint8_t GravisUltrasound::read8(uint16_t port) {
  sync(now());
  const uint16_t off = static_cast<uint16_t>(port - m_config.basePort);
  switch (off) {
  case 0x006:
    return irqStatus();
  case 0x008: { // AdLib-style timer status
    uint8_t s = 0;
    if (m_timers[0].reached)
      s |= 0x40;
    if (m_timers[1].reached)
      s |= 0x20;
    if (s)
      s |= 0x80;
    if (m_irqFlags & kIrqTimer1)
      s |= 0x04;
    if (m_irqFlags & kIrqTimer2)
      s |= 0x02;
    return s;
  }
  case 0x00A:
    return m_adlibIndex;
  case 0x100: // MIDI status: transmitter empty, nothing received
    return 0x02;
  case 0x102:
    return m_voiceSel;
  case 0x103:
    return m_regSel;
  case 0x104:
    return static_cast<uint8_t>(readRegister() & 0xFF);
  case 0x105:
    return static_cast<uint8_t>(readRegister() >> 8);
  case 0x107:
    return dramPeek(m_dramAddr);
  default:
    return 0xFF;
  }
}

uint16_t GravisUltrasound::read16(uint16_t port) {
  if (port == m_config.basePort + 0x104) {
    sync(now());
    return readRegister();
  }
  return IODevice::read16(port);
}

void GravisUltrasound::write8(uint16_t port, uint8_t value) {
  sync(now());
  const uint16_t off = static_cast<uint16_t>(port - m_config.basePort);
  // The latches take a write to 2XB only as the I/O write right after
  // 2X0 (SDK 2.13), so probing other cards cannot change them.
  const bool latchArmed = m_latchArmed;
  m_latchArmed = false;
  switch (off) {
  case 0x000:
    m_mixCtrl = value;
    m_latchArmed = true;
    checkIrq();
    break;
  case 0x008:
    m_adlibIndex = value;
    break;
  case 0x009:
    writeTimerControl(value);
    break;
  case 0x00B:
    if (latchArmed)
      writeLatch(value);
    break;
  case 0x00F:
    m_regCtrl = value;
    break;
  case 0x102:
    m_voiceSel = value & 0x1F;
    break;
  case 0x103:
    m_regSel = value;
    break;
  case 0x104:
    m_dataLow = value;
    break;
  case 0x105:
    writeRegister(static_cast<uint16_t>(value << 8 | m_dataLow));
    break;
  case 0x107:
    dramPoke(m_dramAddr, value);
    break;
  default: // MIDI control/data (nothing attached), unused ports
    break;
  }
}

void GravisUltrasound::write16(uint16_t port, uint16_t value) {
  if (port == m_config.basePort + 0x104) {
    sync(now());
    m_dataLow = static_cast<uint8_t>(value);
    writeRegister(value);
    return;
  }
  IODevice::write16(port, value);
}

void GravisUltrasound::writeLatch(uint8_t value) {
  if (m_regCtrl != 0)
    return; // 2XF selects a rev 3.7+ register: not the latches
  // Selector 0 ("none") keeps the current line, as the SDK's power-up
  // sequence writes 0 before the real values.
  if (m_mixCtrl & 0x40) {
    m_irqLatch = value;
    if (const int irq = kLatchIrq[value & 7])
      m_irqLine = irq;
  } else {
    m_dmaLatch = value;
    if (const int ch = kLatchDma[value & 7]; ch && ch != m_dmaChannel) {
      m_dmaChannel = ch;
      if (m_dmaState == DmaState::Running)
        m_dmaState = DmaState::Waiting;
    }
  }
}

void GravisUltrasound::writeTimerControl(uint8_t value) {
  if (value & 0x80) { // reset the timer flags; the other bits are ignored
    m_timers[0].reached = m_timers[1].reached = false;
    return;
  }
  m_timers[0].masked = (value & 0x40) != 0;
  m_timers[1].masked = (value & 0x20) != 0;
  for (size_t i = 0; i < 2; ++i) {
    Timer &t = m_timers[i];
    if (value & (1u << i)) {
      if (!t.running) {
        t.running = true;
        t.nextNs = m_lastNs + t.periodNs();
      }
    } else {
      t.running = false;
    }
  }
  invalidateEvents();
}

uint8_t GravisUltrasound::dramPeek(uint32_t address) const {
  address &= 0xFFFFF;
  return address < m_dramSize ? m_dram[address] : 0xFF;
}

void GravisUltrasound::dramPoke(uint32_t address, uint8_t value) {
  address &= 0xFFFFF;
  if (address < m_dramSize)
    m_dram[address] = value;
}

// ── GF1 registers ─────────────────────────────────────────────────────

uint16_t GravisUltrasound::readRegister() {
  Voice &v = m_voices[m_voiceSel & 0x1F];
  const uint32_t bit = 1u << (m_voiceSel & 0x1F);
  auto hi = [](uint32_t x) { return static_cast<uint16_t>((x & 0xFF) << 8); };
  switch (m_regSel) {
  case 0x41: { // DMA control; bit 6 = terminal count, reading clears it
    const uint8_t r = static_cast<uint8_t>((m_dmaCtrl & 0xBF) |
                                           (m_dmaTcPending ? 0x40 : 0));
    m_dmaTcPending = false;
    m_irqFlags &= static_cast<uint8_t>(~kIrqDmaTc);
    checkIrq();
    return hi(r);
  }
  case 0x42: return m_dmaAddr;
  case 0x43: return static_cast<uint16_t>(m_dramAddr & 0xFFFF);
  case 0x44: return hi(m_dramAddr >> 16);
  case 0x45: return hi(m_timerCtrl);
  case 0x46: return hi(m_timers[0].count);
  case 0x47: return hi(m_timers[1].count);
  case 0x48: return hi(m_sampleFreq);
  case 0x49: return hi(m_sampleCtrl & 0xBF);
  case 0x4C: return hi(m_reset);
  case 0x80: return hi(v.ctrl | ((m_wavePending & bit) ? kIrqPending : 0));
  case 0x81: return v.fc;
  case 0x82: return static_cast<uint16_t>((v.start >> 16) & 0x1FFF);
  case 0x83: return static_cast<uint16_t>(v.start & 0xFFFF);
  case 0x84: return static_cast<uint16_t>((v.end >> 16) & 0x1FFF);
  case 0x85: return static_cast<uint16_t>(v.end & 0xFFFF);
  case 0x86: return hi(v.rampRate);
  case 0x87: return hi(v.rampStart);
  case 0x88: return hi(v.rampEnd);
  case 0x89: return static_cast<uint16_t>(v.vol << 4);
  case 0x8A: return static_cast<uint16_t>((static_cast<uint32_t>(v.pos) >> 16) & 0x1FFF);
  case 0x8B: return static_cast<uint16_t>(v.pos & 0xFFFF);
  case 0x8C: return hi(v.pan);
  case 0x8D: return hi(v.volCtrl | ((m_rampPending & bit) ? kIrqPending : 0));
  case 0x8E: return hi(0xC0u | static_cast<uint32_t>(m_activeVoices - 1));
  case 0x8F: { // IRQ source: next voice from the queue (bits 7/6 active low)
    uint8_t r = static_cast<uint8_t>(0xE0 | m_lastIrqVoice);
    if (m_queueLen > 0) {
      const uint8_t voice = m_queue[0];
      std::copy(m_queue.begin() + 1, m_queue.begin() + m_queueLen,
                m_queue.begin());
      --m_queueLen;
      const uint32_t vb = 1u << voice;
      r = static_cast<uint8_t>(0x20 | voice);
      if (!(m_wavePending & vb))
        r |= 0x80;
      if (!(m_rampPending & vb))
        r |= 0x40;
      m_wavePending &= ~vb;
      m_rampPending &= ~vb;
      m_lastIrqVoice = voice;
      checkIrq(); // more voices queued: the handler is called again
    }
    return hi(r);
  }
  default:
    return 0;
  }
}

void GravisUltrasound::writeRegister(uint16_t value) {
  const uint8_t reg = m_regSel;
  const uint8_t hi = static_cast<uint8_t>(value >> 8);
  if (reg < 0x0E) {
    writeVoiceRegister(m_voices[m_voiceSel & 0x1F], reg, value);
    return;
  }
  switch (reg) {
  case 0x0E: // active voices: bits 5-0 = voices - 1
    setActiveVoices((hi & 0x3F) + 1, m_lastNs);
    break;
  case 0x41:
    writeDmaControl(hi);
    break;
  case 0x42:
    m_dmaAddr = value;
    m_dmaPosValid = false;
    break;
  case 0x43:
    m_dramAddr = (m_dramAddr & 0xF0000) | value;
    break;
  case 0x44:
    m_dramAddr = (m_dramAddr & 0xFFFF) | (static_cast<uint32_t>(hi & 0x0F) << 16);
    break;
  case 0x45: // timer control: bits 2/3 enable the timer 1/2 IRQs
    m_timerCtrl = hi;
    if (!(hi & kIrqTimer1))
      m_irqFlags &= static_cast<uint8_t>(~kIrqTimer1);
    if (!(hi & kIrqTimer2))
      m_irqFlags &= static_cast<uint8_t>(~kIrqTimer2);
    invalidateEvents();
    checkIrq();
    break;
  case 0x46:
    m_timers[0].count = hi;
    break;
  case 0x47:
    m_timers[1].count = hi;
    break;
  case 0x48:
    m_sampleFreq = hi;
    break;
  case 0x49: // sampling control (recording is not emulated)
    m_sampleCtrl = hi;
    break;
  case 0x4C: // reset: bit 0 run (0 = reset), bit 1 DAC, bit 2 IRQs
    m_reset = hi & 0x07;
    if (!(hi & 0x01))
      resetGf1();
    invalidateEvents();
    checkIrq();
    break;
  default:
    break;
  }
}

void GravisUltrasound::writeVoiceRegister(Voice &v, uint8_t reg,
                                          uint16_t value) {
  const uint8_t hi = static_cast<uint8_t>(value >> 8);
  const int index = static_cast<int>(&v - m_voices.data());
  const uint32_t bit = 1u << index;
  switch (reg) {
  case 0x00:
    v.ctrl = hi & 0x7F;
    // Writing IRQ pending + IRQ enable requests an IRQ; anything else
    // clears a pending wavetable IRQ.
    if ((hi & (kIrqPending | kIrqEnable)) == (kIrqPending | kIrqEnable)) {
      queueVoiceIrq(index, 1);
      checkIrq();
    } else if (m_wavePending & bit) {
      m_wavePending &= ~bit;
      dropFromQueue(index);
    }
    break;
  case 0x01:
    v.fc = value;
    break;
  case 0x02:
    v.start = (v.start & 0xFFFF) | (static_cast<int32_t>(value & 0x1FFF) << 16);
    break;
  case 0x03:
    v.start = (v.start & ~0xFFFF) | value;
    break;
  case 0x04:
    v.end = (v.end & 0xFFFF) | (static_cast<int32_t>(value & 0x1FFF) << 16);
    break;
  case 0x05:
    v.end = (v.end & ~0xFFFF) | value;
    break;
  case 0x06:
    v.rampRate = hi;
    v.rampCounter = 1u << (3 * (hi >> 6));
    break;
  case 0x07:
    v.rampStart = hi;
    break;
  case 0x08:
    v.rampEnd = hi;
    break;
  case 0x09:
    v.vol = value >> 4;
    updateGain(v);
    break;
  case 0x0A:
    v.pos = (v.pos & 0xFFFF) | (static_cast<int32_t>(value & 0x1FFF) << 16);
    break;
  case 0x0B:
    v.pos = (v.pos & ~0xFFFF) | value;
    break;
  case 0x0C:
    v.pan = hi & 0x0F;
    updateGain(v);
    break;
  case 0x0D: {
    const bool wasStopped = (v.volCtrl & (kStopped | kStop)) != 0;
    v.volCtrl = hi & 0x7F;
    if (wasStopped && !(v.volCtrl & (kStopped | kStop)))
      v.rampCounter = 1u << (3 * (v.rampRate >> 6));
    if ((hi & (kIrqPending | kIrqEnable)) == (kIrqPending | kIrqEnable)) {
      queueVoiceIrq(index, 2);
      checkIrq();
    } else if (m_rampPending & bit) {
      m_rampPending &= ~bit;
      dropFromQueue(index);
    }
    break;
  }
  default:
    break;
  }
  invalidateEvents();
}

// ── IRQs ──────────────────────────────────────────────────────────────

void GravisUltrasound::queueVoiceIrq(int voice, uint8_t bits) {
  const uint32_t bit = 1u << voice;
  const bool queued = ((m_wavePending | m_rampPending) & bit) != 0;
  if (bits & 1)
    m_wavePending |= bit;
  if (bits & 2)
    m_rampPending |= bit;
  if (!queued && m_queueLen < kMaxVoices)
    m_queue[static_cast<size_t>(m_queueLen++)] = static_cast<uint8_t>(voice);
}

void GravisUltrasound::dropFromQueue(int voice) {
  if ((m_wavePending | m_rampPending) & (1u << voice))
    return;
  auto end = m_queue.begin() + m_queueLen;
  auto it = std::remove(m_queue.begin(), end, static_cast<uint8_t>(voice));
  m_queueLen = static_cast<int>(it - m_queue.begin());
}

uint8_t GravisUltrasound::irqStatus() const {
  const uint32_t active = m_activeVoices >= 32
                              ? 0xFFFFFFFFu
                              : (1u << m_activeVoices) - 1;
  uint8_t s = m_irqFlags;
  if (m_wavePending & active)
    s |= kIrqWave;
  if (m_rampPending & active)
    s |= kIrqRamp;
  return s;
}

bool GravisUltrasound::irqPending() const {
  return (irqStatus() & (kIrqTimer1 | kIrqTimer2 | kIrqWave | kIrqRamp |
                         kIrqDmaTc)) != 0;
}

void GravisUltrasound::checkIrq() {
  if (!m_irqCallback || m_irqLine <= 0)
    return;
  if (!(m_mixCtrl & 0x08))
    return; // latches disabled: the IRQ line is not driven
  // 4Ch bit 2 enables the voice IRQs; timer and DMA IRQs pass anyway.
  const uint8_t mask = (m_reset & 0x04) ? 0xEC : 0x8C;
  if (irqStatus() & mask)
    m_irqCallback(m_irqLine == 2 ? 9 : m_irqLine);
}

// ── Timers ────────────────────────────────────────────────────────────

void GravisUltrasound::runTimers(uint64_t nowNs) {
  bool raised = false;
  for (size_t i = 0; i < m_timers.size(); ++i) {
    Timer &t = m_timers[i];
    if (!t.running || nowNs < t.nextNs)
      continue;
    const uint64_t period = t.periodNs();
    t.nextNs += ((nowNs - t.nextNs) / period + 1) * period;
    if (!t.masked)
      t.reached = true;
    if (m_timerCtrl & (kIrqTimer1 << i)) {
      m_irqFlags |= static_cast<uint8_t>(kIrqTimer1 << i);
      raised = true;
    }
  }
  if (raised)
    checkIrq();
}

// ── DMA ───────────────────────────────────────────────────────────────

uint32_t GravisUltrasound::dmaRate() const {
  return static_cast<uint32_t>(kDmaClockHz / (((m_dmaCtrl >> 3) & 3) + 1));
}

void GravisUltrasound::writeDmaControl(uint8_t value) {
  m_dmaCtrl = value;
  if (!(value & 0x01))
    m_dmaState = DmaState::Idle;
  else if (m_dmaState != DmaState::Running)
    startDma();
  invalidateEvents();
}

void GravisUltrasound::startDma() {
  if (m_dmaChannel < 0 || m_dma.isMasked(m_dmaChannel)) {
    m_dmaState = DmaState::Waiting;
    return;
  }
  if (!m_dmaPosValid) {
    // 42h holds the DRAM address >> 4; for 16-bit transfers on a 16-bit
    // channel the GF1 translates it like a 16-bit voice address (bits
    // 15-14 = 256 KB bank, bits 12-0 = word address >> 4).
    const uint32_t a = m_dmaAddr;
    m_dmaPos = (dma16() ? (((a & 0x1FFF) << 1) | (a & 0xC000)) : a) << 4;
    m_dmaPos &= 0xFFFFF;
    m_dmaPosValid = true;
  }
  m_dmaBase = m_dmaPos;
  m_dmaOffset = 0;
  m_dmaUnits = 0;
  m_dmaStartNs = m_lastNs;
  m_dmaState = DmaState::Running;
}

void GravisUltrasound::dmaMaskChanged(int channel, bool masked) {
  if (channel != m_dmaChannel || m_dmaState == DmaState::Idle)
    return;
  sync(now()); // transfers up to the change
  if (masked && m_dmaState == DmaState::Running)
    m_dmaState = DmaState::Waiting;
  else if (!masked && m_dmaState == DmaState::Waiting)
    startDma();
  invalidateEvents();
}

uint64_t GravisUltrasound::dmaCompletionNs() const {
  const uint64_t total =
      m_dmaUnits + static_cast<uint64_t>(m_dma.getChannelCount(m_dmaChannel)) + 1;
  const uint64_t rate = dmaRate();
  return m_dmaStartNs + (total * 1'000'000'000 + rate - 1) / rate;
}

void GravisUltrasound::runDma(uint64_t nowNs) {
  const int ch = m_dmaChannel;
  if (ch < 0 || m_dma.isMasked(ch)) {
    m_dmaState = DmaState::Waiting;
    return;
  }
  const uint64_t due = (nowNs - m_dmaStartNs) * dmaRate() / 1'000'000'000;
  const uint32_t unitBytes = ch >= 4 ? 2 : 1;
  const bool toDram = !(m_dmaCtrl & 0x02);
  const bool invert = (m_dmaCtrl & 0x80) != 0;
  const bool data16 = (m_dmaCtrl & 0x40) != 0;
  const bool bank16 = dma16();
  auto dramAt = [&](uint32_t offset) {
    return bank16 ? (m_dmaBase & 0xC0000) | ((m_dmaBase + offset) & 0x3FFFF)
                  : (m_dmaBase + offset) & 0xFFFFF;
  };
  bool tc = false;
  while (m_dmaUnits < due && !tc) {
    const uint32_t host = m_dma.getChannelAddress(ch);
    for (uint32_t b = 0; b < unitBytes; ++b) {
      const uint32_t dram = dramAt(m_dmaOffset);
      if (toDram) {
        uint8_t byte = m_memory.read8(host + b);
        if (invert && (!data16 || (m_dmaOffset & 1)))
          byte ^= 0x80;
        dramPoke(dram, byte);
      } else {
        m_memory.write8(host + b, dramPeek(dram));
      }
      ++m_dmaOffset;
    }
    ++m_dmaUnits;
    tc = m_dma.acknowledgeTransfer(ch, static_cast<uint16_t>(unitBytes));
  }
  // 42h follows the transfer, so the next one continues from here.
  m_dmaPos = dramAt(m_dmaOffset);
  m_dmaAddr = static_cast<uint16_t>(
      bank16 ? ((m_dmaPos & 0xC0000) | ((m_dmaPos & 0x3FFFE) >> 1)) >> 4
             : m_dmaPos >> 4);
  if (tc) {
    // The GF1 stops at terminal count; 41h keeps its bits and the next
    // unmask of the channel (or a 41h write) starts another transfer.
    m_dmaState = DmaState::Waiting;
    m_dmaTcPending = true;
    invalidateEvents();
    if (m_dmaCtrl & 0x20) {
      m_irqFlags |= kIrqDmaTc;
      checkIrq();
    }
  }
}

// ── Synthesis ─────────────────────────────────────────────────────────

float GravisUltrasound::volumeGain(uint32_t volume12) {
  return volumeTable()[volume12 & 0xFFF];
}

void GravisUltrasound::updateGain(Voice &v) {
  const float g = volumeTable()[static_cast<uint32_t>(v.vol) & 0xFFF] * kOutputGain;
  v.gainL = g * panGains().left[v.pan & 15];
  v.gainR = g * panGains().right[v.pan & 15];
}

float GravisUltrasound::sampleAt(const Voice &v) const {
  const uint32_t a = (static_cast<uint32_t>(v.pos) >> 9) & 0xFFFFF;
  const float frac = static_cast<float>(v.pos & 0x1FF) * (1.0f / 512.0f);
  const uint8_t *d = m_dram.data();
  float s0, s1;
  if (v.ctrl & kData16) {
    // 16-bit data: the address counts samples within a 256 KB bank.
    const uint32_t bank = a & 0xC0000;
    const uint32_t w0 = bank | ((a << 1) & 0x3FFFE);
    const uint32_t w1 = bank | (((a + 1) << 1) & 0x3FFFE);
    s0 = static_cast<float>(static_cast<int16_t>(d[w0] | d[w0 + 1] << 8));
    s1 = static_cast<float>(static_cast<int16_t>(d[w1] | d[w1 + 1] << 8));
  } else {
    s0 = static_cast<float>(static_cast<int8_t>(d[a]) * 256);
    s1 = static_cast<float>(static_cast<int8_t>(d[(a + 1) & 0xFFFFF]) * 256);
  }
  return s0 + (s1 - s0) * frac;
}

// The position passed a boundary: loop, reverse or stop.  Returns true if
// a wavetable IRQ is due.
bool GravisUltrasound::waveBoundary(Voice &v) {
  const bool irq = (v.ctrl & kIrqEnable) != 0;
  if (v.volCtrl & kData16) // rollover: IRQ only, the voice plays on
    return irq;
  const bool dec = (v.ctrl & kDecreasing) != 0;
  const int32_t left = dec ? v.start - v.pos : v.pos - v.end;
  if (v.ctrl & kLoop) {
    if (v.ctrl & kBidir)
      v.ctrl ^= kDecreasing;
    v.pos = (v.ctrl & kDecreasing) ? v.end - left : v.start + left;
  } else {
    v.ctrl |= kStopped;
    v.pos = dec ? v.start : v.end;
  }
  return irq;
}

// One volume ramp step.  Returns true if a ramp IRQ is due.
bool GravisUltrasound::rampStep(Voice &v) {
  const int32_t add = v.rampRate & 0x3F;
  const int32_t lo = v.rampStart << 4, hi = v.rampEnd << 4;
  const bool dec = (v.volCtrl & kDecreasing) != 0;
  int32_t left;
  if (dec) {
    v.vol -= add;
    left = lo - v.vol;
  } else {
    v.vol += add;
    left = v.vol - hi;
  }
  bool irq = false;
  if (left >= 0) {
    irq = (v.volCtrl & kIrqEnable) != 0;
    if (v.volCtrl & kLoop) {
      if (v.volCtrl & kBidir)
        v.volCtrl ^= kDecreasing;
      v.vol = (v.volCtrl & kDecreasing) ? hi - left : lo + left;
    } else {
      v.volCtrl |= kStopped;
      v.vol = dec ? lo : hi;
    }
  }
  v.vol = std::clamp<int32_t>(v.vol, 0, 4095);
  updateGain(v);
  return irq;
}

uint64_t GravisUltrasound::ticksToVoiceEvent(const Voice &v) const {
  uint64_t best = kNever;
  auto ceilDiv = [](int64_t d, int64_t inc) -> uint64_t {
    return d <= 0 ? 1 : static_cast<uint64_t>((d + inc - 1) / inc);
  };
  if (!(v.ctrl & (kStopped | kStop)) && (v.ctrl & kIrqEnable)) {
    const int64_t inc = v.fc >> 1;
    if (inc > 0) {
      const bool dec = (v.ctrl & kDecreasing) != 0;
      const int64_t d = dec ? int64_t{v.pos} - v.start : int64_t{v.end} - v.pos;
      if (d > 0 || !(v.volCtrl & kData16)) // rollover IRQs only on crossing
        best = ceilDiv(d, inc);
    }
  }
  if (!(v.volCtrl & (kStopped | kStop)) && (v.volCtrl & kIrqEnable)) {
    const int64_t add = v.rampRate & 0x3F;
    const uint64_t period = uint64_t{1} << (3 * (v.rampRate >> 6));
    const bool dec = (v.volCtrl & kDecreasing) != 0;
    const int64_t d = dec ? int64_t{v.vol} - (v.rampStart << 4)
                          : (int64_t{v.rampEnd} << 4) - v.vol;
    if (d <= 0 || add > 0) {
      const uint64_t steps = add > 0 ? ceilDiv(d, add) : 1;
      best = std::min(best, v.rampCounter + (steps - 1) * period);
    }
  }
  return best;
}

void GravisUltrasound::renderVoice(int index, uint32_t ticks, float *left,
                                   float *right) {
  Voice &v = m_voices[static_cast<size_t>(index)];
  bool wave = !(v.ctrl & (kStopped | kStop));
  bool ramp = !(v.volCtrl & (kStopped | kStop));
  const int32_t inc = v.fc >> 1;
  const uint32_t period = 1u << (3 * (v.rampRate >> 6));
  for (uint32_t t = 0; t < ticks && (wave || ramp); ++t) {
    const float s = sampleAt(v);
    left[t] += s * v.gainL;
    right[t] += s * v.gainR;
    if (wave) {
      bool crossed;
      if (v.ctrl & kDecreasing) {
        const int32_t old = v.pos;
        v.pos -= inc;
        crossed = v.pos <= v.start &&
                  (!(v.volCtrl & kData16) || old > v.start);
      } else {
        const int32_t old = v.pos;
        v.pos += inc;
        crossed = v.pos >= v.end && (!(v.volCtrl & kData16) || old < v.end);
      }
      if (crossed) {
        if (waveBoundary(v))
          m_events.push_back({t, static_cast<uint8_t>(index), 1});
        wave = !(v.ctrl & (kStopped | kStop));
      }
    }
    if (ramp && --v.rampCounter == 0) {
      v.rampCounter = period;
      if (rampStep(v))
        m_events.push_back({t, static_cast<uint8_t>(index), 2});
      ramp = !(v.volCtrl & (kStopped | kStop));
    }
  }
}

void GravisUltrasound::runTicks(uint64_t count) {
  m_mixL.resize(kChunk);
  m_mixR.resize(kChunk);
  const bool running = (m_reset & 0x01) != 0;
  const bool output = (m_reset & 0x02) && !(m_mixCtrl & 0x02);
  while (count > 0) {
    const uint32_t n = static_cast<uint32_t>(std::min<uint64_t>(count, kChunk));
    bool sound = false;
    if (running) {
      m_events.clear();
      bool any = false;
      for (int i = 0; i < m_activeVoices; ++i) {
        const Voice &v = m_voices[static_cast<size_t>(i)];
        if ((v.ctrl & (kStopped | kStop)) && (v.volCtrl & (kStopped | kStop)))
          continue;
        if (!any) {
          std::fill_n(m_mixL.begin(), n, 0.0f);
          std::fill_n(m_mixR.begin(), n, 0.0f);
          any = true;
        }
        renderVoice(i, n, m_mixL.data(), m_mixR.data());
      }
      sound = any && output;
      if (!m_events.empty()) {
        std::stable_sort(m_events.begin(), m_events.end(),
                         [](const VoiceEvent &a, const VoiceEvent &b) {
                           return a.tick < b.tick;
                         });
        for (const auto &e : m_events)
          queueVoiceIrq(e.voice, e.bits);
      }
    }
    // Output FIFO for the resampler (m_fifoTick + frames == m_ticksDone);
    // silence longer than the filter is not stored.
    if (sound) {
      const size_t at = m_fifo.size();
      m_fifo.resize(at + 2 * n);
      for (uint32_t t = 0; t < n; ++t) {
        m_fifo[at + 2 * t] = m_mixL[t];
        m_fifo[at + 2 * t + 1] = m_mixR[t];
      }
      m_soundEndTick = m_ticksDone + n;
    } else if (!m_fifo.empty() && m_ticksDone < m_soundEndTick + 2 * kTaps) {
      m_fifo.resize(m_fifo.size() + 2 * n, 0.0f);
    } else {
      m_fifo.clear();
      m_fifoTick = m_ticksDone + n;
    }
    if (m_capture) {
      for (uint32_t t = 0; t < n; ++t) {
        m_capture->push_back(sound ? m_mixL[t] : 0.0f);
        m_capture->push_back(sound ? m_mixR[t] : 0.0f);
      }
    }
    m_ticksDone += n;
    count -= n;
    if (running && !m_events.empty()) {
      m_events.clear();
      checkIrq();
    }
  }
  if (m_fifo.size() > 2 * kMaxFifoFrames) { // nobody is listening
    const size_t drop = m_fifo.size() / 2 - kMaxFifoFrames / 2;
    m_fifo.erase(m_fifo.begin(), m_fifo.begin() + static_cast<std::ptrdiff_t>(2 * drop));
    m_fifoTick += drop;
  }
}

// ── Resampling to the mixer rate ──────────────────────────────────────

void GravisUltrasound::buildResampler() {
  // Kaiser-windowed sinc low-pass at 0.45 x the lower of the two rates.
  const double in = gf1Rate();
  const double cutoff = 0.45 * std::min(1.0, m_outputRate / in);
  constexpr double kBeta = 8.0;
  constexpr double kHalf = kTaps / 2;
  const double i0Beta = besselI0(kBeta);
  m_taps.assign(static_cast<size_t>((kPhases + 1) * kTaps), 0.0f);
  for (int ph = 0; ph <= kPhases; ++ph) {
    const double frac = static_cast<double>(ph) / kPhases;
    std::array<double, kTaps> h{};
    double sum = 0.0;
    for (int k = 0; k < kTaps; ++k) {
      const double x = (k - (kTaps / 2 - 1)) - frac;
      const double arg = 2.0 * cutoff * x;
      const double sinc =
          std::fabs(arg) < 1e-12 ? 1.0 : std::sin(std::numbers::pi * arg) / (std::numbers::pi * arg);
      const double r = x / kHalf;
      const double w = r * r >= 1.0 ? 0.0 : besselI0(kBeta * std::sqrt(1.0 - r * r)) / i0Beta;
      h[static_cast<size_t>(k)] = sinc * w;
      sum += h[static_cast<size_t>(k)];
    }
    for (int k = 0; k < kTaps; ++k)
      m_taps[static_cast<size_t>(ph * kTaps + k)] =
          static_cast<float>(h[static_cast<size_t>(k)] / sum);
  }
  m_tapsForVoices = m_activeVoices;
}

void GravisUltrasound::generateSamples(float *buffer, size_t frames) {
  const uint64_t nowNs = std::max(now(), m_lastNs);
  sync(nowNs);
  if (frames == 0)
    return;
  if (m_tapsForVoices != m_activeVoices)
    buildResampler();
  // Output frame f is at f / outputRate s, the same grid as the main loop.
  const uint64_t outRate = static_cast<uint64_t>(std::lround(m_outputRate));
  const uint64_t endFrame = nowNs / 1'000'000'000 * outRate +
                            (nowNs % 1'000'000'000) * outRate / 1'000'000'000;
  const uint64_t firstFrame = endFrame >= frames ? endFrame - frames : 0;
  const double rate = gf1Rate();
  const double nsPerFrame = 1e9 / static_cast<double>(outRate);
  const double startPos =
      static_cast<double>(m_epochTick) +
      (static_cast<double>(firstFrame) * nsPerFrame -
       static_cast<double>(m_epochNs)) * rate * 1e-9 - kDelayTicks;
  const double step = rate / static_cast<double>(outRate);
  const double endPos = startPos + step * static_cast<double>(frames);
  if (m_fifo.empty() || startPos - kTaps >= static_cast<double>(m_soundEndTick) ||
      endPos + kTaps < static_cast<double>(m_fifoTick))
    return; // silent window

  m_stage.assign(frames * 2, 0.0f);
  const int64_t fifoFrames = static_cast<int64_t>(m_fifo.size() / 2);
  const float *fifo = m_fifo.data();
  for (size_t i = 0; i < frames; ++i) {
    const double p = startPos + step * static_cast<double>(i);
    const double base = std::floor(p);
    const int phase = static_cast<int>((p - base) * kPhases + 0.5);
    const float *h = &m_taps[static_cast<size_t>(phase * kTaps)];
    const int64_t first = static_cast<int64_t>(base) - (kTaps / 2 - 1) -
                          static_cast<int64_t>(m_fifoTick);
    float l = 0.0f, r = 0.0f;
    if (first >= 0 && first + kTaps <= fifoFrames) {
      const float *x = fifo + 2 * first;
      for (int k = 0; k < kTaps; ++k) {
        l += x[2 * k] * h[k];
        r += x[2 * k + 1] * h[k];
      }
    } else {
      for (int k = 0; k < kTaps; ++k) {
        const int64_t j = first + k;
        if (j >= 0 && j < fifoFrames) {
          l += fifo[2 * j] * h[k];
          r += fifo[2 * j + 1] * h[k];
        }
      }
    }
    m_stage[2 * i] = l;
    m_stage[2 * i + 1] = r;
  }
  utils::simd::kernels().mixAddF32(buffer, m_stage.data(), frames * 2, 1.0f);

  // Frames before the next window's first tap are no longer needed.
  const int64_t keep = static_cast<int64_t>(std::floor(endPos)) - kTaps -
                       static_cast<int64_t>(m_fifoTick);
  if (keep > 0) {
    const size_t drop = static_cast<size_t>(std::min(keep, fifoFrames));
    m_fifo.erase(m_fifo.begin(), m_fifo.begin() + static_cast<std::ptrdiff_t>(2 * drop));
    m_fifoTick += drop;
  }
}

} // namespace fador::hw::audio
