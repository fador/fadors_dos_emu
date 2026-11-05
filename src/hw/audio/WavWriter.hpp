#pragma once
#include <cstdint>
#include <cstdio>
#include <string>

namespace fador::hw::audio {

// Records the emulator's mixed output (interleaved stereo float) as a
// 16-bit PCM WAV file (--audio-wav).  Useful for checking sound headless.
class WavWriter {
public:
  WavWriter() = default;
  ~WavWriter() { close(); }
  WavWriter(const WavWriter &) = delete;
  WavWriter &operator=(const WavWriter &) = delete;

  bool open(const std::string &path, int sampleRate, int channels);
  void write(const float *interleaved, size_t sampleCount); // floats
  void close();
  bool isOpen() const { return m_file != nullptr; }
  uint64_t framesWritten() const { return m_dataBytes / (2u * m_channels); }
  float peak() const { return m_peak; }

private:
  void writeHeader();
  std::FILE *m_file = nullptr;
  int m_sampleRate = 44100;
  int m_channels = 2;
  uint64_t m_dataBytes = 0;
  float m_peak = 0.0f;
};

} // namespace fador::hw::audio
