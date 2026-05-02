#pragma once

#include "../IODevice.hpp"
#include <array>
#include <cstdint>
#include <vector>

namespace fador::hw::audio {

// Simplified ADSR envelope state
enum class EnvelopeState { OFF, ATTACK, DECAY, SUSTAIN, RELEASE };

struct Operator {
  // Registers
  uint8_t flags20 = 0;  // AM(7), VIB(6), EGT(5), KSR(4), MULT(3-0)
  uint8_t flags40 = 0;  // KSL(7-6), TL(5-0)
  uint8_t flags60 = 0;  // AR(7-4), DR(3-0)
  uint8_t flags80 = 0;  // SL(7-4), RR(3-0)
  uint8_t waveform = 0; // WS(1-0)

  // Running state
  float phase = 0.0f;
  float currentLevel = 0.0f;
  EnvelopeState state = EnvelopeState::OFF;

  // Parsed parameters
  float attackRate = 0;
  float decayRate = 0;
  float sustainLevel = 0;
  float releaseRate = 0;
  float totalLevel = 0;
  float mult = 1.0f;
  bool isCarrier = false; // Based on connection
};

struct Channel {
  Operator op1; // Modulator (usually)
  Operator op2; // Carrier (usually)
  uint16_t fnum = 0;
  uint8_t block = 0;
  bool keyOn = false;
  uint8_t feedback = 0;
  bool connection = false; // false = FM (op1->op2), true = Additive (op1+op2)
  float op1LastOutput = 0.0f;
};

class AdLib : public IODevice {
public:
  AdLib(float sampleRate = 44100.0f, const uint64_t *cpuCycles = nullptr);
  ~AdLib() override = default;

  uint8_t read8(uint16_t port) override;
  void write8(uint16_t port, uint8_t value) override;

  // Render audio into an interleaved stereo float buffer
  void generateSamples(float *buffer, size_t numSamples);

  // Update timers (should be called frequently by the emulator loop to emulate
  // OPL timers)
  void updateTimers(double dt);

private:
  void writeRegister(uint8_t reg, uint8_t val);
  float renderChannel(int ch);
  float computePhaseStep(uint16_t fnum, uint8_t block, float mult);
  void updateEnvelope(Operator &op, bool keyOn, double dt, uint16_t fnum,
                      uint8_t block);

  uint8_t m_currentRegister = 0;
  std::array<uint8_t, 256> m_registers{};
  std::array<Channel, 9> m_channels;

  float m_sampleRate;
  double m_time = 0.0;

  // Timers
  uint8_t m_timer1Data = 0;
  uint8_t m_timer2Data = 0;
  uint8_t m_timerControl = 0;
  double m_timer1Counter = 0;
  double m_timer2Counter = 0;
  uint8_t m_status = 0;

  bool m_waveformEnable = false;

  // CPU cycle counter for accurate timer advancement on status reads
  const uint64_t *m_cpuCycles = nullptr;
  uint64_t m_lastTimerCycles = 0;
  static constexpr double CPU_FREQ = 33000000.0; // ~33 MHz 386

  // Lookups mapping register bases to operator indices
  static const int opOffsets[18];
};

} // namespace fador::hw::audio
