#include "HostSocket.hpp"

#include <cstring>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

namespace fador::utils {

namespace {

#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalid = INVALID_SOCKET;

bool startNetworking() {
  static const bool ok = [] {
    WSADATA data;
    return WSAStartup(MAKEWORD(2, 2), &data) == 0;
  }();
  return ok;
}
void closeHandle(SocketHandle s) { closesocket(s); }
bool setNonBlocking(SocketHandle s) {
  u_long on = 1;
  return ioctlsocket(s, FIONBIO, &on) == 0;
}
bool wouldBlock() {
  const int e = WSAGetLastError();
  return e == WSAEWOULDBLOCK || e == WSAEINPROGRESS;
}
constexpr int kSendFlags = 0;
#else
using SocketHandle = int;
constexpr SocketHandle kInvalid = -1;

bool startNetworking() { return true; }
void closeHandle(SocketHandle s) { ::close(s); }
bool setNonBlocking(SocketHandle s) {
  const int flags = fcntl(s, F_GETFL, 0);
  return flags >= 0 && fcntl(s, F_SETFL, flags | O_NONBLOCK) == 0;
}
bool wouldBlock() {
  return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINPROGRESS ||
         errno == EINTR;
}
#ifdef MSG_NOSIGNAL
constexpr int kSendFlags = MSG_NOSIGNAL;
#else
constexpr int kSendFlags = 0;
#endif
#endif

void configureStream(SocketHandle s) {
  setNonBlocking(s);
  int one = 1;
  setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char *>(&one),
             sizeof one);
#if defined(SO_NOSIGPIPE)
  setsockopt(s, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
}

} // namespace

// ── TcpStream ────────────────────────────────────────────────────────

struct TcpStream::Impl {
  SocketHandle socket = kInvalid;
  bool connecting = false;
};

TcpStream::TcpStream() : m_impl(std::make_unique<Impl>()) {}
TcpStream::~TcpStream() { close(); }

bool TcpStream::isOpen() const { return m_impl->socket != kInvalid; }

void TcpStream::close() {
  if (m_impl->socket != kInvalid)
    closeHandle(m_impl->socket);
  m_impl->socket = kInvalid;
  m_impl->connecting = false;
}

bool TcpStream::connect(const std::string &host, uint16_t port) {
  close();
  if (!startNetworking())
    return false;
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo *list = nullptr;
  const std::string service = std::to_string(port);
  if (getaddrinfo(host.c_str(), service.c_str(), &hints, &list) != 0 || !list)
    return false;
  bool ok = false;
  for (addrinfo *ai = list; ai && !ok; ai = ai->ai_next) {
    SocketHandle s = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (s == kInvalid)
      continue;
    configureStream(s);
    if (::connect(s, ai->ai_addr, static_cast<int>(ai->ai_addrlen)) == 0) {
      m_impl->connecting = false;
    } else if (wouldBlock()) {
      m_impl->connecting = true;
    } else {
      closeHandle(s);
      continue;
    }
    m_impl->socket = s;
    ok = true;
  }
  freeaddrinfo(list);
  return ok;
}

bool TcpStream::connected() {
  if (m_impl->socket == kInvalid)
    return false;
  if (!m_impl->connecting)
    return true;
  fd_set writable, failed;
  FD_ZERO(&writable);
  FD_ZERO(&failed);
  FD_SET(m_impl->socket, &writable);
  FD_SET(m_impl->socket, &failed);
  timeval tv{0, 0};
  const int n = select(static_cast<int>(m_impl->socket + 1), nullptr,
                       &writable, &failed, &tv);
  if (n <= 0)
    return false;
  int err = 0;
  socklen_t len = sizeof err;
  getsockopt(m_impl->socket, SOL_SOCKET, SO_ERROR,
             reinterpret_cast<char *>(&err), &len);
  if (err != 0 || FD_ISSET(m_impl->socket, &failed)) {
    close();
    return false;
  }
  m_impl->connecting = false;
  return true;
}

size_t TcpStream::send(const uint8_t *data, size_t size) {
  if (size == 0 || !connected())
    return 0;
  const auto n = ::send(m_impl->socket, reinterpret_cast<const char *>(data),
                        static_cast<int>(size), kSendFlags);
  if (n > 0)
    return static_cast<size_t>(n);
  if (n < 0 && !wouldBlock())
    close();
  return 0;
}

size_t TcpStream::receive(uint8_t *data, size_t size) {
  if (size == 0 || !connected())
    return 0;
  const auto n = ::recv(m_impl->socket, reinterpret_cast<char *>(data),
                        static_cast<int>(size), 0);
  if (n > 0)
    return static_cast<size_t>(n);
  if (n == 0 || !wouldBlock())
    close(); // peer closed the connection, or an error
  return 0;
}

// ── TcpListener ──────────────────────────────────────────────────────

struct TcpListener::Impl {
  SocketHandle socket = kInvalid;
};

TcpListener::TcpListener() : m_impl(std::make_unique<Impl>()) {}
TcpListener::~TcpListener() {
  if (m_impl->socket != kInvalid)
    closeHandle(m_impl->socket);
}

bool TcpListener::isOpen() const { return m_impl->socket != kInvalid; }

bool TcpListener::listen(uint16_t port) {
  if (!startNetworking())
    return false;
  SocketHandle s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s == kInvalid)
    return false;
  int one = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&one),
             sizeof one);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port);
  if (::bind(s, reinterpret_cast<sockaddr *>(&addr), sizeof addr) != 0 ||
      ::listen(s, 1) != 0 || !setNonBlocking(s)) {
    closeHandle(s);
    return false;
  }
  m_impl->socket = s;
  return true;
}

bool TcpListener::accept(TcpStream &stream) {
  if (m_impl->socket == kInvalid)
    return false;
  SocketHandle s = ::accept(m_impl->socket, nullptr, nullptr);
  if (s == kInvalid)
    return false;
  stream.close();
  configureStream(s);
  stream.m_impl->socket = s;
  stream.m_impl->connecting = false;
  return true;
}

} // namespace fador::utils
