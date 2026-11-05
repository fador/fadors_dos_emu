#pragma once

#include "../../memory/MemoryBus.hpp"
#include "../DMA8237.hpp"
#include "../IODevice.hpp"
#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <queue>
#include <vector>

namespace fador::hw::audio {

class AdLib;

// Sound Blaster 16 (DSP 4.05) with SB Pro compatible mixer.
//
// Digital audio runs on emulated time: advance() (called by the main loop
// between instruction batches) pulls samples through the 8237 DMA
// controller at the DSP's sample rate, raises the block-end IRQs and queues
// the converted output; generateSamples() hands that output to the host.
// The FM ports (base+0..3, base+8/9) are forwarded to the OPL chip.
class SoundBlaster : public IODevice {
public:
  SoundBlaster(fador::memory::MemoryBus &memory, DMA8237 &dma,
               float systemSampleRate = 44100.0f,
               uint16_t basePort = 0x220, int irq = 5,
               int dmaChannel8 = 1, int dmaChannel16 = 5);
  ~SoundBlaster() override = default;

  uint8_t read8(uint16_t port) override;
  void write8(uint16_t port, uint8_t value) override;

  // Run the DSP for `seconds` of emulated time.
  void advance(double seconds);
  // Mix `numSamples` stereo frames of output into `buffer` (runs the DSP
  // further if advance() has not produced enough yet).
  void generateSamples(float *buffer, size_t numSamples);

  void setIRQCallback(std::function<void()> cb) { m_irqCallback = std::move(cb); }
  void setFM(AdLib *fm) { m_fm = fm; }

  uint16_t getBasePort() const { return m_basePort; }
  int getIRQ() const { return m_irq; }

  // Mixer volume (0..31, SB16 scale) for diagnostics/tests.
  uint8_t mixerVolume(uint8_t sb16Register) const { return m_mixer[sb16Register] >> 3; }

private:
  void dspReset();
  void dspWrite(uint8_t value);
  void processCommand();
  int argumentCount(uint8_t cmd) const;
  void startDma(bool is16, bool autoInit, bool stereo, bool isSigned,
                uint32_t length, bool input, bool highSpeed);
  void triggerIRQ(bool is16bit);

  void produce(size_t frames);
  void fetchFrame();

  uint8_t readMixer(uint8_t reg) const;
  void writeMixer(uint8_t reg, uint8_t value);
  void resetMixer();
  static float volumeGain(uint8_t sb16Value);

  fador::memory::MemoryBus &m_memory;
  DMA8237 &m_dma;
  std::function<void()> m_irqCallback;
  AdLib *m_fm = nullptr;

  uint16_t m_basePort;
  int m_irq;
  int m_dmaChannel8;
  int m_dmaChannel16;
  float m_outputRate;

  // DSP command interface
  bool m_resetLatch = false;
  std::queue<uint8_t> m_readQueue;
  uint8_t m_lastRead = 0xFF;
  uint8_t m_command = 0;
  int m_argsNeeded = 0;
  std::vector<uint8_t> m_args;
  bool m_speakerOn = false;
  uint8_t m_testRegister = 0;
  uint8_t m_identifyByte = 0;

  // Sample rate: 0x40 time constant or 0x41/0x42 rate.
  float m_rate = 22050.0f;

  // DMA engine
  bool m_dmaRunning = false;
  bool m_dmaPaused = false;
  bool m_dma16 = false;
  bool m_stereo = false;
  bool m_signed = false;
  bool m_autoInit = false;
  bool m_input = false;
  bool m_highSpeed = false;
  bool m_exitAutoInit = false;
  bool m_halfRate = false;       // SB Pro stereo: the rate counts both channels
  uint32_t m_dacIdleFrames = 0;
  uint32_t m_blockSize = 0x800;  // samples per IRQ block
  uint32_t m_blockLeft = 0;      // samples left in the current block
  uint32_t m_legacyBlock = 0x800; // 0x48 block size for 0x1C/0x90/0x91
  double m_frameAcc = 0.0;
  std::array<float, 2> m_prev{};
  std::array<float, 2> m_cur{};

  // 0x80: silence for N samples, then IRQ
  uint32_t m_silenceLeft = 0;
  double m_silenceAcc = 0.0;

  // Direct DAC (0x10) writes since the last produced frame
  std::vector<float> m_dacPending;
  float m_dacLevel = 0.0f;

  // IRQ status (mixer register 82h)
  bool m_irq8Pending = false;
  bool m_irq16Pending = false;

  // Output queue (stereo frames at the output rate)
  std::deque<std::array<float, 2>> m_out;
  double m_outAcc = 0.0;
  bool m_timeDriven = false; // advance() is in use: never run ahead of it

  // Mixer
  uint8_t m_mixerIndex = 0;
  std::array<uint8_t, 256> m_mixer{};
  uint32_t m_gainKey = 0xFFFFFFFFu; // mixer values m_gain was computed for
  std::array<float, 2> m_gain{1.0f, 1.0f};
};

} // namespace fador::hw::audio
