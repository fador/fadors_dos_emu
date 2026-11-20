#include "SerialMouse.hpp"
#include "../utils/Logger.hpp"
#include <algorithm>

namespace fador::hw {

SerialMouse::SerialMouse(InputSource source) : m_source(std::move(source)) {}

void SerialMouse::transmit(uint8_t byte, uint64_t nowNs) {
  (void)byte; // a mouse ignores data sent to it
  (void)nowNs;
}

void SerialMouse::controlLines(bool dtr, bool rts, uint64_t nowNs) {
  (void)nowNs;
  const bool power = dtr && rts;
  if (power == m_powered)
    return;
  m_powered = power;
  m_queue.clear();
  if (!power)
    return;
  // Power-up reset: forget motion so far and identify as a Microsoft mouse.
  if (m_source) {
    const Input in = m_source();
    m_lastX = in.motionX;
    m_lastY = in.motionY;
    m_lastButtons = in.buttons & 3;
  }
  m_queue.push_back('M');
  LOG_DEBUG("Serial mouse: reset, sending 'M'");
}

uint8_t SerialMouse::take() {
  const uint8_t b = m_queue.front();
  m_queue.pop_front();
  return b;
}

void SerialMouse::encodePacket(int dx, int dy, bool left, bool right,
                               uint8_t out[3]) {
  dx = std::clamp(dx, -128, 127);
  dy = std::clamp(dy, -128, 127);
  const auto ux = static_cast<uint8_t>(dx);
  const auto uy = static_cast<uint8_t>(dy);
  out[0] = static_cast<uint8_t>(0x40 | (left ? 0x20 : 0) | (right ? 0x10 : 0) |
                                ((uy >> 4) & 0x0C) | (ux >> 6));
  out[1] = ux & 0x3F;
  out[2] = uy & 0x3F;
}

void SerialMouse::poll(uint64_t nowNs) {
  (void)nowNs;
  if (!m_powered || !m_queue.empty() || !m_source)
    return;
  const Input in = m_source();
  const uint8_t buttons = in.buttons & 3;
  const int64_t pendingX = static_cast<int64_t>(in.motionX) - m_lastX;
  const int64_t pendingY = static_cast<int64_t>(in.motionY) - m_lastY;
  if (pendingX == 0 && pendingY == 0 && buttons == m_lastButtons)
    return;
  const int dx = static_cast<int>(std::clamp<int64_t>(pendingX, -128, 127));
  const int dy = static_cast<int>(std::clamp<int64_t>(pendingY, -128, 127));
  m_lastX += dx;
  m_lastY += dy;
  m_lastButtons = buttons;
  uint8_t packet[3];
  encodePacket(dx, dy, buttons & 1, buttons & 2, packet);
  m_queue.insert(m_queue.end(), packet, packet + 3);
}

} // namespace fador::hw
