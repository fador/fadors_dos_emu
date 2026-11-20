#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace fador::utils {

/// Minimal non-blocking TCP stream over the platform socket API (Winsock 2
/// on Windows, BSD sockets elsewhere).  Everything is polled: no call
/// blocks and no threads are used.
class TcpStream {
public:
  TcpStream();
  ~TcpStream();
  TcpStream(const TcpStream &) = delete;
  TcpStream &operator=(const TcpStream &) = delete;

  /// Starts a non-blocking connect to host:port.  False when the name
  /// cannot be resolved or the socket cannot be created.
  bool connect(const std::string &host, uint16_t port);
  /// True once the connection is established (completes a pending
  /// connect).  A failed or closed connection makes isOpen() false.
  bool connected();
  bool isOpen() const;
  void close();

  /// Sends as much as the socket accepts; returns the bytes sent.
  size_t send(const uint8_t *data, size_t size);
  /// Receives up to size bytes without blocking; 0 when nothing is there
  /// (the connection is closed when the peer hung up).
  size_t receive(uint8_t *data, size_t size);

private:
  friend class TcpListener;
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

/// Non-blocking listening socket (all interfaces).
class TcpListener {
public:
  TcpListener();
  ~TcpListener();
  TcpListener(const TcpListener &) = delete;
  TcpListener &operator=(const TcpListener &) = delete;

  bool listen(uint16_t port);
  bool isOpen() const;
  /// Accepts a pending connection into `stream` (replacing what it had).
  bool accept(TcpStream &stream);

private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace fador::utils
