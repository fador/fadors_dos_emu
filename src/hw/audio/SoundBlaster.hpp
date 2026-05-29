#pragma once

#include "../../memory/MemoryBus.hpp"
#include "../DMA8237.hpp"
#include "../IODevice.hpp"
#include <array>
#include <cstdint>
#include <functional>
#include <queue>

namespace fador::hw::audio {

class SoundBlaster : public IODevice {
public:
  SoundBlaster(fador::memory::MemoryBus &memory, DMA8237 &dma,
               float systemSampleRate = 44100.0f,
               uint16_t basePort = 0x220, int irq = 5,
               int dmaChannel8 = 1, int dmaChannel16 = 5);
  ~SoundBlaster() override = default;

  uint8_t read8(uint16_t port) override;
  void write8(uint16_t port, uint8_t value) override;

  void generateSamples(float *buffer, size_t numSamples);

  void setIRQCallback(std::function<void()> cb) { m_irqCallback = cb; }

  // Expose config for testability
  uint16_t getBasePort() const { return m_basePort; }
  int getIRQ() const { return m_irq; }

  struct MixerState {
    uint8_t masterLeft = 0xC;   // default 75%
    uint8_t masterRight = 0xC;
    uint8_t voiceLeft = 0xC;
    uint8_t voiceRight = 0xC;
    uint8_t midiLeft = 0xC;
    uint8_t midiRight = 0xC;
    uint8_t cdLeft = 0x0;
    uint8_t cdRight = 0x0;
    uint8_t lineInLeft = 0x0;
    uint8_t lineInRight = 0x0;
    uint8_t micLevel = 0x0;
    uint8_t pcSpeaker = 0x0;
    uint8_t outputMixerSwitch = 0x1E; // voice, midi, cd, line-in enabled
    uint8_t inputMixerSwitch = 0x05;  // mic selected
    uint8_t recordSource = 0x00;      // mic
    uint8_t recordGainLeft = 0x00;
    uint8_t recordGainRight = 0x00;
    uint8_t agc = 0x00;
    uint8_t micGain = 0x00;
    uint8_t trebleLeft = 0x8;
    uint8_t trebleRight = 0x8;
    uint8_t bassLeft = 0x8;
    uint8_t bassRight = 0x8;
  };

  const MixerState &getMixerState() const { return m_mixer; }

private:
  void processCommand(uint8_t cmd);
  void triggerIRQ(bool is16bit);

  // Mixer helpers
  uint8_t readMixerRegister(uint8_t reg) const;
  void writeMixerRegister(uint8_t reg, uint8_t value);

  // Volume helpers
  static float mixerVolumeToFloat(uint8_t nibble);

  fador::memory::MemoryBus &m_memory;
  DMA8237 &m_dma;
  std::function<void()> m_irqCallback;

  // Hardware config
  uint16_t m_basePort;
  int m_irq;
  int m_dmaChannel8;
  int m_dmaChannel16;

  // DSP State
  bool m_resetState = false;
  std::queue<uint8_t> m_readQueue;
  std::queue<uint8_t> m_writeQueue;

  int m_expectedArgs = 0;
  uint8_t m_currentCommand = 0;

  // Playback State
  bool m_speakerOn = true;
  float m_dspSampleRate = 22050.0f;
  float m_systemSampleRate = 44100.0f;

  // DMA execution
  bool m_dmaActive = false;
  bool m_autoInitDma = false;
  uint16_t m_dmaLength = 0;
  uint16_t m_dmaBaseLength = 0;
  float m_sampleAccumulator = 0.0f;

  // High speed / precision modifiers
  bool m_highSpeed = false;
  bool m_16bit = false;
  bool m_stereo = false;

  // Interpolation state
  float m_lastSampleL = 0.0f;
  float m_lastSampleR = 0.0f;
  float m_prevSampleL = 0.0f;
  float m_prevSampleR = 0.0f;

  // IRQ state — separate 8-bit and 16-bit
  bool m_irq8Pending = false;
  bool m_irq16Pending = false;

  // Mixer
  uint8_t m_mixerIndex = 0;
  MixerState m_mixer;

  // DSP test register (0xE4/0xE8)
  uint8_t m_dspTestReg = 0x00;
};

} // namespace fador::hw::audio
