#pragma once

#include "../../memory/MemoryBus.hpp"
#include "../DMA8237.hpp"
#include "../IODevice.hpp"
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace fador::hw::audio {

/// Card settings (--gus=...).
struct GusConfig {
  uint16_t basePort = 0x240; // 2X0h; the GF1 registers are at 3X0h
  int irq = 11;              // GF1 IRQ (latch default)
  int midiIrq = 11;          // MIDI IRQ (reported in ULTRASND only)
  int dma = 3;               // playback DMA (latch default)
  int recordDma = 3;         // record DMA (reported in ULTRASND only)
  uint32_t dramKB = 1024;    // 256, 512, 768 or 1024
};

/// Parses "[PORT][,irq=N][,dma=N][,rdma=N][,midiirq=N][,mem=KB]" (PORT in
/// hex, 210..260).  An empty spec keeps the defaults.
bool parseGusSpec(const std::string &spec, GusConfig &config,
                  std::string &error);
/// "240,3,3,11,11": port, play DMA, record DMA, GF1 IRQ, MIDI IRQ.
std::string ultrasndVariable(const GusConfig &config);

/// Gravis Ultrasound (classic, GF1 synthesizer, 256 KB - 1 MB of DRAM).
///
/// Ports (X = 4 for the default base 240h):
///   2X0 mix control       2X6 IRQ status       2X8/2X9 AdLib-style timers
///   2XA timer index (rd)  2XB IRQ/DMA latches  2XF register control
///   3X0/3X1 MIDI (ACIA)   3X2 voice select     3X3 register select
///   3X4/3X5 data low/high 3X7 DRAM peek/poke
///
/// The GF1 runs on emulated time: it produces one stereo frame per voice
/// cycle, 617400 / active voices per second (44100 Hz at 14 voices), with
/// per-voice linear interpolation, 12-bit logarithmic volumes and volume
/// ramps processed at that rate, and 16 pan positions.  Every port access
/// first runs the GF1 up to the current time (sync()); advanceTo() does so
/// only when the next interrupt (voice wavetable/ramp IRQ, timer, DMA
/// terminal count) is due.  generateSamples() resamples the GF1 stream to
/// the 44.1 kHz mixer rate with a windowed-sinc filter.
///
/// Voice, timer and DMA interrupts go through the IRQ latch (2XB) and are
/// gated by mix control bit 3 (enable latches) and reset register 4Ch
/// bit 2 (GF1 master IRQ enable).  DMA uploads read host memory through the
/// 8237 at the rate selected in register 41h.
class GravisUltrasound : public IODevice {
public:
  static constexpr uint32_t kGf1Clock = 617400; // 9.8784 MHz / 16
  static constexpr int kMaxVoices = 32;
  static constexpr int kMinVoices = 14;

  GravisUltrasound(memory::MemoryBus &memory, DMA8237 &dma,
                   const GusConfig &config = {}, float outputRate = 44100.0f);
  ~GravisUltrasound() override = default;

  uint8_t read8(uint16_t port) override;
  void write8(uint16_t port, uint8_t value) override;
  uint16_t read16(uint16_t port) override;
  void write16(uint16_t port, uint16_t value) override;

  /// Virtual time in ns (clock.timeSource()).
  void setTimeSource(std::function<uint64_t()> nowNs);
  /// Called with the IRQ number when the GF1 requests an interrupt.
  void setIrqCallback(std::function<void(int)> cb) { m_irqCallback = std::move(cb); }

  /// Runs the GF1 up to nowNs if an interrupt may be due; a single compare
  /// otherwise.
  void advanceTo(uint64_t nowNs) {
    if (nowNs >= m_nextEventNs)
      service(nowNs);
  }
  /// Earliest time of the next interrupt the GF1 may raise (kNever if none).
  uint64_t nextEventNs();
  static constexpr uint64_t kNever = ~uint64_t{0};

  /// Mixes `frames` stereo frames at the output rate, ending at the current
  /// time on the output-rate grid (frame f is at f / outputRate seconds).
  void generateSamples(float *buffer, size_t frames);

  // ── Diagnostics / tests ──────────────────────────────────────────────
  const GusConfig &config() const { return m_config; }
  uint16_t basePort() const { return m_config.basePort; }
  int irqLine() const { return m_irqLine; }
  int dmaChannel() const { return m_dmaChannel; }
  int activeVoices() const { return m_activeVoices; }
  /// GF1 output rate in Hz (617400 / active voices).
  double gf1Rate() const { return static_cast<double>(kGf1Clock) / m_activeVoices; }
  uint64_t gf1Ticks() const { return m_ticksDone; }
  /// Appends every GF1 output frame (L, R) produced from now on.
  void setCapture(std::vector<float> *sink) { m_capture = sink; }
  uint8_t dramByte(uint32_t address) const { return m_dram[address & 0xFFFFF]; }
  /// Linear gain of a 12-bit GF1 volume (0..4095).
  static float volumeGain(uint32_t volume12);

private:
  struct Voice {
    int32_t pos = 0;    // address << 9 | 9-bit fraction (registers 0Ah/0Bh)
    int32_t start = 0;  // same format (02h/03h)
    int32_t end = 0;    // (04h/05h)
    uint16_t fc = 0;    // frequency control (01h); increment = fc >> 1
    uint8_t ctrl = 0x01;    // voice control (00h) without the IRQ bit
    uint8_t volCtrl = 0x01; // volume control (0Dh) without the IRQ bit
    int32_t vol = 0;        // 12-bit current volume (09h bits 15-4)
    uint8_t rampRate = 0, rampStart = 0, rampEnd = 0;
    uint8_t pan = 7;
    uint32_t rampCounter = 1; // voice cycles until the next ramp step
    float gainL = 0.0f, gainR = 0.0f;
  };
  struct VoiceEvent {
    uint32_t tick; // within the chunk
    uint8_t voice;
    uint8_t bits;  // 1 = wavetable, 2 = volume ramp
  };
  struct Timer {
    bool running = false;
    bool masked = false;
    bool reached = false;
    uint8_t count = 0;
    uint64_t nextNs = 0;
    uint64_t unitNs = 80'000;
    uint64_t periodNs() const { return (256u - count) * unitNs; }
  };

  // Port and register access
  uint16_t readRegister();
  void writeRegister(uint16_t value);
  void writeVoiceRegister(Voice &v, uint8_t reg, uint16_t value);
  void writeLatch(uint8_t value);
  void writeTimerControl(uint8_t value);
  void writeDmaControl(uint8_t value);
  uint8_t dramPeek(uint32_t address) const;
  void dramPoke(uint32_t address, uint8_t value);
  void resetGf1();

  // Emulated time
  uint64_t now() const { return m_nowNs ? m_nowNs() : 0; }
  void sync(uint64_t nowNs);
  void service(uint64_t nowNs);
  uint64_t ticksDue(uint64_t nowNs);
  uint64_t tickTimeNs(uint64_t tick) const;
  void setActiveVoices(int voices, uint64_t nowNs);
  void invalidateEvents() { m_nextEventNs = 0; m_eventsDirty = true; }
  void computeNextEvent();

  // Synthesis
  void runTicks(uint64_t count);
  void renderVoice(int index, uint32_t ticks, float *left, float *right);
  float sampleAt(const Voice &v) const;
  void updateGain(Voice &v);
  bool waveBoundary(Voice &v);
  bool rampStep(Voice &v);
  uint64_t ticksToVoiceEvent(const Voice &v) const;
  void buildResampler();

  // Timers and DMA
  void runTimers(uint64_t nowNs);
  void runDma(uint64_t nowNs);
  uint64_t dmaCompletionNs() const;
  uint32_t dmaRate() const;

  // IRQs
  void queueVoiceIrq(int voice, uint8_t bits);
  void dropFromQueue(int voice);
  uint8_t irqStatus() const;
  bool irqPending() const;
  void checkIrq();

  memory::MemoryBus &m_memory;
  DMA8237 &m_dma;
  GusConfig m_config;
  float m_outputRate;
  std::function<uint64_t()> m_nowNs;
  std::function<void(int)> m_irqCallback;
  uint32_t m_dramSize;
  std::vector<uint8_t> m_dram;

  // Board registers
  uint8_t m_mixCtrl = 0x08;  // 2X0: line in/out enabled, latches enabled
  uint8_t m_regCtrl = 0;     // 2XF
  uint8_t m_adlibIndex = 0;  // 2X8 (read back at 2XA)
  uint8_t m_irqLatch = 0, m_dmaLatch = 0;
  int m_irqLine;
  int m_dmaChannel;
  uint8_t m_voiceSel = 0;    // 3X2
  uint8_t m_regSel = 0;      // 3X3
  uint8_t m_dataLow = 0;     // 3X4 written as a byte
  uint32_t m_dramAddr = 0;   // 43h/44h

  // GF1 global registers
  uint8_t m_reset = 0;       // 4Ch
  uint8_t m_dmaCtrl = 0;     // 41h as written (bit 6 = 16-bit data)
  uint16_t m_dmaAddr = 0;    // 42h
  uint8_t m_timerCtrl = 0;   // 45h
  uint8_t m_sampleFreq = 0;  // 48h
  uint8_t m_sampleCtrl = 0;  // 49h
  uint8_t m_irqFlags = 0;    // 2X6 bits 2, 3 (timers) and 7 (DMA TC)
  std::array<Voice, kMaxVoices> m_voices{};
  int m_activeVoices = kMinVoices;
  uint32_t m_wavePending = 0, m_rampPending = 0; // per voice
  std::array<uint8_t, kMaxVoices> m_queue{};     // voice IRQ order
  int m_queueLen = 0;
  uint8_t m_lastIrqVoice = 0;
  std::array<Timer, 2> m_timers{};

  // DMA engine
  enum class DmaState { Idle, WaitUnmask, Running };
  DmaState m_dmaState = DmaState::Idle;
  uint64_t m_dmaStartNs = 0;
  uint64_t m_dmaUnits = 0;   // transfers since m_dmaStartNs
  uint32_t m_dmaOffset = 0;  // bytes transferred since the start
  uint32_t m_dmaBase = 0;    // DRAM byte address of the transfer

  // Time base: GF1 tick t ends at m_epochNs + (t - m_epochTick) periods.
  uint64_t m_epochNs = 0;
  uint64_t m_epochTick = 0;
  uint64_t m_ticksDone = 0;
  uint64_t m_lastNs = 0;
  uint64_t m_nextEventNs = 0; // advanceTo() threshold (0 = look now)
  uint64_t m_wakeNs = kNever;  // next interrupt for HLT wake-ups
  bool m_eventsDirty = true;

  // Synthesis scratch and output
  std::vector<float> m_mixL, m_mixR;
  std::vector<VoiceEvent> m_events;
  std::vector<float> m_fifo;       // interleaved L/R GF1 frames
  uint64_t m_fifoTick = 0;         // tick of m_fifo[0]
  uint64_t m_soundEndTick = 0;     // ticks after this one are silent
  std::vector<float> *m_capture = nullptr;

  // Resampler: windowed sinc, kTaps taps, kPhases + 1 phases.
  static constexpr int kTaps = 16;
  static constexpr int kPhases = 256;
  static constexpr int kDelayTicks = kTaps / 2 + 2;
  std::vector<float> m_taps;
  int m_tapsForVoices = 0;
  std::vector<float> m_stage;
};

} // namespace fador::hw::audio
