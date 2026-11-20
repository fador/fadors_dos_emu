#pragma once

#include <cstdint>
#include <string>

namespace fador::hw {

/// What is plugged into a serial port (the far end of the RS-232 cable).
///
/// The UART calls the backend on emulated time: transmit() when the stop
/// bit of a character has left the transmitter, controlLines() when DTR or
/// RTS change.  The backend offers received bytes through hasData()/take();
/// the UART clocks them in at the line rate.  poll() does the host side
/// I/O (sockets, host mouse) and is called from the main loop, never from
/// a port access, so port accesses stay cheap.
class SerialBackend {
public:
  // Modem status inputs as they appear in MSR bits 4-7.
  static constexpr uint8_t kCTS = 0x10;
  static constexpr uint8_t kDSR = 0x20;
  static constexpr uint8_t kRI = 0x40;
  static constexpr uint8_t kDCD = 0x80;

  virtual ~SerialBackend() = default;

  /// A character left the UART (after its stop bits) at emulated time nowNs.
  virtual void transmit(uint8_t byte, uint64_t nowNs) = 0;
  /// DTR / RTS as driven on the cable (both false while the UART is in
  /// loopback mode).
  virtual void controlLines(bool dtr, bool rts, uint64_t nowNs) {
    (void)dtr;
    (void)rts;
    (void)nowNs;
  }
  /// CTS / DSR / RI / DCD (kCTS | kDSR | kRI | kDCD) seen by the UART.
  virtual uint8_t modemStatus() const { return 0; }
  /// A byte is waiting to be sent to the UART.
  virtual bool hasData() const { return false; }
  /// Remove the next byte (only called when hasData()).
  virtual uint8_t take() { return 0; }
  /// Host-side work: socket I/O, sampling the host mouse.
  virtual void poll(uint64_t nowNs) { (void)nowNs; }
  /// Short description for log messages ("mouse", "file:out.txt", ...).
  virtual std::string describe() const = 0;
};

} // namespace fador::hw
