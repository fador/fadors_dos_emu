#pragma once

#include "../IODevice.hpp"
#include <array>
#include <cstdint>
#include <memory>

namespace fador::hw::audio {

class OplChip;

// AdLib / Sound Blaster FM synthesizer: a YMF262 (OPL3, backwards
// compatible with the YM3812/OPL2 of the original AdLib) emulated by ymfm.
//
// Ports (relative to 388h): 0 = status / bank-0 address, 1 = data,
// 2 = bank-1 address, 3 = data.  The Sound Blaster forwards its own FM
// ports (base+0..3 and base+8/9) here.
class AdLib : public IODevice {
public:
  // `cpuCycles` (optional, 33 MHz) lets unit tests advance the OPL timers
  // deterministically; the emulator advances them with updateTimers().
  AdLib(float sampleRate = 44100.0f, const uint64_t *cpuCycles = nullptr);
  ~AdLib() override;

  uint8_t read8(uint16_t port) override;
  void write8(uint16_t port, uint8_t value) override;

  // Direct register access (bank 0 registers 00h-FFh, bank 1 100h-1FFh),
  // used by the MPU-401 General MIDI synthesizer.
  void writeRegister(uint16_t reg, uint8_t val);
  uint8_t readRegister(uint16_t reg) const { return m_registers[reg & 0x1FF]; }

  // Render audio and add it into an interleaved stereo float buffer.
  void generateSamples(float *buffer, size_t numSamples);

  // Advance the OPL timers by dt seconds of emulated time.
  void updateTimers(double dt);

  // Lookups mapping register bases to operator indices (OPL2 layout)
  static const int opOffsets[18];

private:
  void advanceFromCycles();

  std::unique_ptr<OplChip> m_chip;
  float m_sampleRate;
  uint16_t m_address = 0;
  std::array<uint8_t, 0x200> m_registers{};

  // Resampling from the chip rate (49716 Hz) to the output rate.
  double m_chipRate;
  double m_resamplePos = 0.0; // fraction between m_prev and m_next
  std::array<float, 2> m_prev{};
  std::array<float, 2> m_next{};

  const uint64_t *m_cpuCycles = nullptr;
  uint64_t m_lastTimerCycles = 0;
  static constexpr double CPU_FREQ = 33000000.0;
  // An ISA bus read takes about a microsecond; status-polling delay loops
  // (OPL detection waits 80 us by reading the status port ~100 times)
  // rely on that, so each status read advances the timers by this much.
  static constexpr double kIoReadTime = 1.0e-6;
};

} // namespace fador::hw::audio
