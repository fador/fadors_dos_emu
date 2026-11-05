#include "WavWriter.hpp"
#include <algorithm>
#include <cmath>

namespace fador::hw::audio {

namespace {
void put16(std::FILE *f, uint16_t v) {
  const uint8_t b[2] = {static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8)};
  std::fwrite(b, 1, 2, f);
}
void put32(std::FILE *f, uint32_t v) {
  const uint8_t b[4] = {static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8),
                        static_cast<uint8_t>(v >> 16),
                        static_cast<uint8_t>(v >> 24)};
  std::fwrite(b, 1, 4, f);
}
} // namespace

bool WavWriter::open(const std::string &path, int sampleRate, int channels) {
  close();
  m_file = std::fopen(path.c_str(), "wb");
  if (!m_file)
    return false;
  m_sampleRate = sampleRate;
  m_channels = channels;
  m_dataBytes = 0;
  m_peak = 0.0f;
  writeHeader();
  return true;
}

void WavWriter::writeHeader() {
  const uint32_t data = static_cast<uint32_t>(
      std::min<uint64_t>(m_dataBytes, 0xFFFFFFFFu - 36));
  std::fseek(m_file, 0, SEEK_SET);
  std::fwrite("RIFF", 1, 4, m_file);
  put32(m_file, 36 + data);
  std::fwrite("WAVEfmt ", 1, 8, m_file);
  put32(m_file, 16);
  put16(m_file, 1); // PCM
  put16(m_file, static_cast<uint16_t>(m_channels));
  put32(m_file, static_cast<uint32_t>(m_sampleRate));
  put32(m_file, static_cast<uint32_t>(m_sampleRate * m_channels * 2));
  put16(m_file, static_cast<uint16_t>(m_channels * 2));
  put16(m_file, 16);
  std::fwrite("data", 1, 4, m_file);
  put32(m_file, data);
}

void WavWriter::write(const float *interleaved, size_t sampleCount) {
  if (!m_file)
    return;
  for (size_t i = 0; i < sampleCount; ++i) {
    const float v = std::clamp(interleaved[i], -1.0f, 1.0f);
    m_peak = std::max(m_peak, std::fabs(v));
    put16(m_file, static_cast<uint16_t>(static_cast<int16_t>(
                      std::lrint(v * 32767.0f))));
  }
  m_dataBytes += sampleCount * 2;
}

void WavWriter::close() {
  if (!m_file)
    return;
  writeHeader();
  std::fclose(m_file);
  m_file = nullptr;
}

} // namespace fador::hw::audio
