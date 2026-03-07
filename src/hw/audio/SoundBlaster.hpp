#pragma once

#include "../../memory/MemoryBus.hpp"
#include "../DMA8237.hpp"
#include "../IODevice.hpp"
#include <cstdint>
#include <functional>
#include <queue>

namespace fador::hw::audio {

class SoundBlaster : public IODevice {
public:
  SoundBlaster(fador::memory::MemoryBus &memory, DMA8237 &dma,
               float systemSampleRate = 44100.0f);
  ~SoundBlaster() override = default;

  uint8_t read8(uint16_t port) override;
  void write8(uint16_t port, uint8_t value) override;

  // Called by main loop to render audio and process DMA
  void generateSamples(float *buffer, size_t numSamples);

  void setIRQCallback(std::function<void()> cb) { m_irqCallback = cb; }

private:
  void processCommand(uint8_t cmd);
  void triggerIRQ();

  fador::memory::MemoryBus &m_memory;
  DMA8237 &m_dma;
  std::function<void()> m_irqCallback;

  uint16_t m_basePort = 0x220;

  // DSP State
  bool m_resetState = false;
  std::queue<uint8_t> m_readQueue;
  std::queue<uint8_t> m_writeQueue; // bytes sent to us for arguments

  int m_expectedArgs = 0;
  uint8_t m_currentCommand = 0;

  // Playback State
  bool m_speakerOn = false;
  float m_dspSampleRate = 22050.0f;
  float m_systemSampleRate = 44100.0f;

  // DMA execution
  bool m_dmaActive = false;
  bool m_autoInitDma = false;
  uint16_t m_dmaLength = 0;
  float m_sampleAccumulator = 0.0f; // Used for sample rate conversion

  // High speed / precision modifiers
  bool m_highSpeed = false;
  bool m_16bit = false;
  bool m_stereo = false;

  // Last read sample value (held for oversampling)
  float m_lastSample = 0.0f;
};

} // namespace fador::hw::audio
