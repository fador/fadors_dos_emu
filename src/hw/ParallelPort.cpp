#include "ParallelPort.hpp"
#include "audio/LptDac.hpp"

namespace fador::hw {

ParallelPort::ParallelPort(uint16_t basePort) : m_base(basePort) {}

ParallelPort::~ParallelPort() = default;

void ParallelPort::setDevice(std::unique_ptr<ParallelDevice> device) {
  m_device = std::move(device);
  m_ackEndNs = kNever;
}

void ParallelPort::finishAck() {
  m_ackEndNs = kNever;
  if ((m_control & 0x10) && m_raiseIrq)
    m_raiseIrq();
}

uint8_t ParallelPort::read8(uint16_t port) {
  const uint64_t now = nowNs();
  advanceTo(now);
  switch ((port - m_base) & 3) {
  case 0:
    return m_data;
  case 1: {
    uint8_t status =
        m_device ? m_device->status(m_data, now) : kFloatingStatus;
    if (m_ackEndNs != kNever)
      status &= static_cast<uint8_t>(~0x40); // ACK# pulse in progress
    return status;
  }
  case 2:
    return static_cast<uint8_t>(0xE0 | (m_control & 0x1F));
  default:
    return 0xFF;
  }
}

void ParallelPort::write8(uint16_t port, uint8_t value) {
  const uint64_t now = nowNs();
  advanceTo(now);
  switch ((port - m_base) & 3) {
  case 0:
    m_data = value;
    if (m_device)
      m_device->dataWritten(value, now);
    break;
  case 2: {
    const uint8_t old = m_control;
    m_control = value;
    if (m_device) {
      m_device->controlWritten(old, value, m_data, now);
      // STROBE (bit 0) asserted: the printer takes the byte.
      if (!(old & 0x01) && (value & 0x01) && m_device->strobe(m_data, now))
        m_ackEndNs = now + kAckPulseNs;
    }
    break;
  }
  default:
    break; // the status register is read-only
  }
}

void ParallelPort::generateSamples(float *buffer, size_t frames) {
  if (m_device)
    m_device->generateSamples(buffer, frames, nowNs());
}

// ── File printer ──────────────────────────────────────────────────────

FilePrinter::FilePrinter(std::string path) : m_path(std::move(path)) {
  m_file = std::fopen(m_path.c_str(), "wb");
}

FilePrinter::~FilePrinter() {
  if (m_file)
    std::fclose(m_file);
}

bool FilePrinter::strobe(uint8_t data, uint64_t nowNs) {
  (void)nowNs;
  if (m_file) {
    std::fputc(data, m_file);
    m_dirty = true;
  }
  return true;
}

uint8_t FilePrinter::status(uint8_t data, uint64_t nowNs) {
  (void)data;
  (void)nowNs;
  return 0xDF; // not busy, no ACK, paper present, selected, no error
}

void FilePrinter::poll(uint64_t nowNs) {
  (void)nowNs;
  if (m_dirty && m_file) {
    std::fflush(m_file);
    m_dirty = false;
  }
}

// ── Factory ───────────────────────────────────────────────────────────

bool makeParallelDevice(const std::string &spec, bool &present,
                        std::unique_ptr<ParallelDevice> &device,
                        std::string &error) {
  present = true;
  device.reset();
  if (spec == "off" || spec == "disabled") {
    present = false;
    return true;
  }
  if (spec == "none")
    return true;
  if (spec == "covox") {
    device = std::make_unique<audio::CovoxDac>();
    return true;
  }
  if (spec == "disney" || spec == "dss") {
    device = std::make_unique<audio::DisneySoundSource>();
    return true;
  }
  if (spec.rfind("file:", 0) == 0) {
    auto printer = std::make_unique<FilePrinter>(spec.substr(5));
    if (spec.size() == 5 || !printer->isOpen()) {
      error = "cannot open '" + spec.substr(5) + "' for writing";
      return false;
    }
    device = std::move(printer);
    return true;
  }
  error = "unknown parallel device '" + spec +
          "' (use none, off, file:PATH, covox or disney)";
  return false;
}

} // namespace fador::hw
