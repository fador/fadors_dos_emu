#pragma once

#include "SerialBackend.hpp"
#include "SerialMouse.hpp"
#include <cstdio>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace fador::utils {
class TcpStream;
class TcpListener;
} // namespace fador::utils

namespace fador::hw {

/// Writes everything the guest transmits to a host file (serial printer,
/// logging).  Presents CTS, DSR and DCD so polled senders (INT 14h) see a
/// ready device.
class FileSerialBackend : public SerialBackend {
public:
  explicit FileSerialBackend(std::string path);
  ~FileSerialBackend() override;
  bool isOpen() const { return m_file != nullptr; }

  void transmit(uint8_t byte, uint64_t nowNs) override;
  uint8_t modemStatus() const override { return kCTS | kDSR | kDCD; }
  void poll(uint64_t nowNs) override;
  std::string describe() const override { return "file:" + m_path; }

private:
  std::string m_path;
  std::FILE *m_file = nullptr;
  bool m_dirty = false;
};

/// Null-modem cable to a TCP peer: either connects to host:port or listens
/// on a port and accepts one client at a time.  Bytes are passed through
/// unchanged.  While connected CTS, DSR and DCD are active.
class TcpSerialBackend : public SerialBackend {
public:
  /// Connects to host:port (retried every few seconds while unconnected).
  TcpSerialBackend(std::string host, uint16_t port);
  /// Listens on port.
  explicit TcpSerialBackend(uint16_t listenPort);
  ~TcpSerialBackend() override;
  /// False when the listening socket could not be opened.
  bool isValid() const;

  void transmit(uint8_t byte, uint64_t nowNs) override;
  uint8_t modemStatus() const override;
  bool hasData() const override { return !m_rx.empty(); }
  uint8_t take() override;
  void poll(uint64_t nowNs) override;
  std::string describe() const override;

private:
  static constexpr uint64_t kRetryNs = 2'000'000'000;
  static constexpr size_t kMaxRxBuffered = 4096;

  std::string m_host;
  uint16_t m_port;
  bool m_listen;
  std::unique_ptr<utils::TcpStream> m_stream;
  std::unique_ptr<utils::TcpListener> m_listener;
  bool m_connected = false;
  uint64_t m_nextAttemptNs = 0;
  std::vector<uint8_t> m_tx;
  std::deque<uint8_t> m_rx;
};

/// Parses a --comN= value: off (no port), none (port with nothing
/// attached), mouse, file:PATH, tcp:HOST:PORT or listen:PORT.  On success
/// sets `present` and `backend` (nullptr for none / off).
bool makeSerialBackend(const std::string &spec,
                       const SerialMouse::InputSource &mouseSource,
                       bool &present, std::unique_ptr<SerialBackend> &backend,
                       std::string &error);

} // namespace fador::hw
