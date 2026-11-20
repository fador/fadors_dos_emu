#include "WavWriter.hpp"
#include "../../utils/simd/Simd.hpp"
#include <algorithm>
#include <array>
#include <bit>

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
  // Clamp, convert and track the peak a block at a time (host-SIMD kernel;
  // NaN samples become -1), then write the block with one fwrite.
  std::array<int16_t, 4096> pcm;
  const auto &k = utils::simd::kernels();
  for (size_t done = 0; done < sampleCount;) {
    const size_t n = std::min(pcm.size(), sampleCount - done);
    m_peak = std::max(m_peak, k.f32ToS16(pcm.data(), interleaved + done, n));
    if constexpr (std::endian::native == std::endian::big)
      for (size_t i = 0; i < n; ++i)
        pcm[i] = static_cast<int16_t>(
            (static_cast<uint16_t>(pcm[i]) >> 8) |
            (static_cast<uint16_t>(pcm[i]) << 8));
    std::fwrite(pcm.data(), sizeof(int16_t), n, m_file);
    done += n;
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
