#pragma once

#include "SerialBackend.hpp"
#include <cstdint>
#include <deque>
#include <functional>

namespace fador::hw {

/// Microsoft 2-button serial mouse (1200 baud, 7 data bits, no parity,
/// one stop bit).
///
/// The mouse is powered from DTR and RTS: when both become active it
/// resets and answers with the identification byte 'M', which is how
/// mouse drivers detect it.  While powered it sends a three-byte packet
/// whenever it has moved or a button changed:
///
///   byte 1: 0 1 L R Y7 Y6 X7 X6   (bit 6 marks the first byte)
///   byte 2: 0 0 X5 X4 X3 X2 X1 X0
///   byte 3: 0 0 Y5 Y4 Y3 Y2 Y1 Y0
///
/// with X / Y the signed 8-bit motion since the previous packet (+Y is
/// down).  A new packet is only built once the previous one has been
/// clocked into the UART, so motion accumulates at the line rate like on
/// the real device.
class SerialMouse : public SerialBackend {
public:
  /// Host mouse state: motion counters that only ever accumulate (host
  /// mickeys) and the button bits (bit 0 left, bit 1 right).
  struct Input {
    int32_t motionX = 0;
    int32_t motionY = 0;
    uint8_t buttons = 0;
  };
  using InputSource = std::function<Input()>;

  explicit SerialMouse(InputSource source);

  void transmit(uint8_t byte, uint64_t nowNs) override;
  void controlLines(bool dtr, bool rts, uint64_t nowNs) override;
  bool hasData() const override { return !m_queue.empty(); }
  uint8_t take() override;
  void poll(uint64_t nowNs) override;
  std::string describe() const override { return "mouse"; }

  bool powered() const { return m_powered; }

  /// Encodes one Microsoft mouse packet (dx, dy clamped to -128..127).
  static void encodePacket(int dx, int dy, bool left, bool right,
                           uint8_t out[3]);

private:
  InputSource m_source;
  std::deque<uint8_t> m_queue;
  bool m_powered = false;
  int32_t m_lastX = 0, m_lastY = 0; // motion already reported
  uint8_t m_lastButtons = 0;
};

} // namespace fador::hw
