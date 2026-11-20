#include "SerialBackends.hpp"
#include "../utils/HostSocket.hpp"
#include "../utils/Logger.hpp"
#include <charconv>

namespace fador::hw {

// ── File ──────────────────────────────────────────────────────────────

FileSerialBackend::FileSerialBackend(std::string path)
    : m_path(std::move(path)) {
  m_file = std::fopen(m_path.c_str(), "wb");
}

FileSerialBackend::~FileSerialBackend() {
  if (m_file)
    std::fclose(m_file);
}

void FileSerialBackend::transmit(uint8_t byte, uint64_t nowNs) {
  (void)nowNs;
  if (!m_file)
    return;
  std::fputc(byte, m_file);
  m_dirty = true;
}

void FileSerialBackend::poll(uint64_t nowNs) {
  (void)nowNs;
  if (m_dirty && m_file) {
    std::fflush(m_file);
    m_dirty = false;
  }
}

// ── TCP null modem ────────────────────────────────────────────────────

TcpSerialBackend::TcpSerialBackend(std::string host, uint16_t port)
    : m_host(std::move(host)), m_port(port), m_listen(false),
      m_stream(std::make_unique<utils::TcpStream>()) {}

TcpSerialBackend::TcpSerialBackend(uint16_t listenPort)
    : m_port(listenPort), m_listen(true),
      m_stream(std::make_unique<utils::TcpStream>()),
      m_listener(std::make_unique<utils::TcpListener>()) {
  if (!m_listener->listen(listenPort))
    LOG_ERROR("Serial: cannot listen on TCP port ", listenPort);
}

TcpSerialBackend::~TcpSerialBackend() = default;

bool TcpSerialBackend::isValid() const {
  return !m_listen || m_listener->isOpen();
}

std::string TcpSerialBackend::describe() const {
  return m_listen ? "listen:" + std::to_string(m_port)
                  : "tcp:" + m_host + ":" + std::to_string(m_port);
}

void TcpSerialBackend::transmit(uint8_t byte, uint64_t nowNs) {
  (void)nowNs;
  if (m_connected && m_tx.size() < 65536)
    m_tx.push_back(byte);
}

uint8_t TcpSerialBackend::modemStatus() const {
  return m_connected ? (kCTS | kDSR | kDCD) : 0;
}

uint8_t TcpSerialBackend::take() {
  const uint8_t b = m_rx.front();
  m_rx.pop_front();
  return b;
}

void TcpSerialBackend::poll(uint64_t nowNs) {
  if (m_listen) {
    if (!m_stream->isOpen() && m_listener->accept(*m_stream))
      LOG_INFO("Serial: ", describe(), " client connected");
  } else if (!m_stream->isOpen() && nowNs >= m_nextAttemptNs) {
    m_nextAttemptNs = nowNs + kRetryNs;
    m_stream->connect(m_host, m_port);
  }
  const bool connected = m_stream->connected();
  if (connected != m_connected) {
    LOG_INFO("Serial: ", describe(), connected ? " connected" : " disconnected");
    m_connected = connected;
    m_tx.clear();
  }
  if (!connected)
    return;
  if (!m_tx.empty()) {
    const size_t sent = m_stream->send(m_tx.data(), m_tx.size());
    m_tx.erase(m_tx.begin(), m_tx.begin() + static_cast<std::ptrdiff_t>(sent));
  }
  uint8_t buf[512];
  while (m_rx.size() < kMaxRxBuffered) {
    const size_t n = m_stream->receive(buf, sizeof buf);
    if (n == 0)
      break;
    m_rx.insert(m_rx.end(), buf, buf + n);
  }
}

// ── Factory ───────────────────────────────────────────────────────────

namespace {
bool parsePort(const std::string &text, uint16_t &port) {
  unsigned value = 0;
  const auto [end, ec] =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (ec != std::errc() || end != text.data() + text.size() || value == 0 ||
      value > 65535)
    return false;
  port = static_cast<uint16_t>(value);
  return true;
}
} // namespace

bool makeSerialBackend(const std::string &spec,
                       const SerialMouse::InputSource &mouseSource,
                       bool &present, std::unique_ptr<SerialBackend> &backend,
                       std::string &error) {
  present = true;
  backend.reset();
  if (spec == "off" || spec == "disabled") {
    present = false;
    return true;
  }
  if (spec == "none")
    return true;
  if (spec == "mouse") {
    backend = std::make_unique<SerialMouse>(mouseSource);
    return true;
  }
  if (spec.rfind("file:", 0) == 0) {
    auto file = std::make_unique<FileSerialBackend>(spec.substr(5));
    if (spec.size() == 5 || !file->isOpen()) {
      error = "cannot open '" + spec.substr(5) + "' for writing";
      return false;
    }
    backend = std::move(file);
    return true;
  }
  if (spec.rfind("listen:", 0) == 0) {
    uint16_t port = 0;
    if (!parsePort(spec.substr(7), port)) {
      error = "bad TCP port in '" + spec + "'";
      return false;
    }
    auto tcp = std::make_unique<TcpSerialBackend>(port);
    if (!tcp->isValid()) {
      error = "cannot listen on TCP port " + spec.substr(7);
      return false;
    }
    backend = std::move(tcp);
    return true;
  }
  if (spec.rfind("tcp:", 0) == 0) {
    const std::string rest = spec.substr(4);
    const auto colon = rest.rfind(':');
    uint16_t port = 0;
    if (colon == std::string::npos || colon == 0 ||
        !parsePort(rest.substr(colon + 1), port)) {
      error = "use tcp:HOST:PORT";
      return false;
    }
    backend = std::make_unique<TcpSerialBackend>(rest.substr(0, colon), port);
    return true;
  }
  error = "unknown serial device '" + spec +
          "' (use none, off, mouse, file:PATH, tcp:HOST:PORT or listen:PORT)";
  return false;
}

} // namespace fador::hw
