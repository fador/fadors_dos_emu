#pragma once

#include "IODevice.hpp"
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <string>

namespace fador::hw {

/// What is plugged into a parallel port.
class ParallelDevice {
public:
  virtual ~ParallelDevice() = default;

  /// The data latch (base+0) was written.
  virtual void dataWritten(uint8_t value, uint64_t nowNs) {
    (void)value;
    (void)nowNs;
  }
  /// The control register (base+2) changed from oldValue to newValue
  /// (register values: bit 0 STROBE, 1 AUTOFD, 2 INIT#, 3 SELECT IN,
  /// 4 IRQ enable).
  virtual void controlWritten(uint8_t oldValue, uint8_t newValue,
                              uint8_t data, uint64_t nowNs) {
    (void)oldValue;
    (void)newValue;
    (void)data;
    (void)nowNs;
  }
  /// STROBE was asserted with `data` on the lines.  True when the device
  /// takes the byte and answers with an ACK pulse.
  virtual bool strobe(uint8_t data, uint64_t nowNs) {
    (void)data;
    (void)nowNs;
    return false;
  }
  /// The status register as read at base+1: bit 7 = not BUSY, 6 = ACK#,
  /// 5 = paper end, 4 = SELECT, 3 = ERROR#, bits 0-2 read as 1.
  virtual uint8_t status(uint8_t data, uint64_t nowNs) = 0;
  /// Mixes stereo frames that end at endNs into buffer (sound devices).
  virtual void generateSamples(float *buffer, size_t frames, uint64_t endNs) {
    (void)buffer;
    (void)frames;
    (void)endNs;
  }
  /// Host-side work (flushing a file), from the main loop.
  virtual void poll(uint64_t nowNs) { (void)nowNs; }
  virtual std::string describe() const = 0;
};

/// Standard (SPP) parallel port at base..base+2: data latch, status and
/// control registers.  The status register shows the lines with the PC's
/// polarity: bit 7 is the inverted BUSY line, ERROR# and ACK# are active
/// low.  Without a device the inputs float high (status 7Fh: busy, paper
/// end, selected, no error), which the BIOS reports as "not ready".
///
/// When the device acknowledges a strobe, ACK# is pulsed low for
/// kAckPulseNs; with control bit 4 set the end of the pulse raises the
/// IRQ (IRQ 7 for LPT1).
class ParallelPort : public IODevice {
public:
  static constexpr uint64_t kNever = std::numeric_limits<uint64_t>::max();
  static constexpr uint64_t kAckPulseNs = 10'000;
  static constexpr uint8_t kFloatingStatus = 0x7F;

  explicit ParallelPort(uint16_t basePort = 0x378);
  ~ParallelPort() override;

  uint8_t read8(uint16_t port) override;
  void write8(uint16_t port, uint8_t value) override;

  uint16_t basePort() const { return m_base; }
  void setDevice(std::unique_ptr<ParallelDevice> device);
  ParallelDevice *device() const { return m_device.get(); }

  void setTimeSource(std::function<uint64_t()> nowNs) {
    m_timeSource = std::move(nowNs);
  }
  void setIrqCallback(std::function<void()> raise) {
    m_raiseIrq = std::move(raise);
  }
  /// Main loop: ends the ACK pulse (and raises the IRQ) when due.
  void advanceTo(uint64_t nowNs) {
    if (nowNs >= m_ackEndNs)
      finishAck();
  }
  void poll(uint64_t nowNs) {
    if (m_device)
      m_device->poll(nowNs);
  }
  /// Mixes the attached sound device's output up to now.
  void generateSamples(float *buffer, size_t frames);

  uint8_t data() const { return m_data; }
  uint8_t control() const { return m_control; }

private:
  uint64_t nowNs() const { return m_timeSource ? m_timeSource() : 0; }
  void finishAck();

  uint16_t m_base;
  std::unique_ptr<ParallelDevice> m_device;
  std::function<uint64_t()> m_timeSource;
  std::function<void()> m_raiseIrq;
  uint8_t m_data = 0;
  uint8_t m_control = 0x0C; // INIT# high, SELECT IN asserted
  uint64_t m_ackEndNs = kNever;
};

/// A printer that writes every strobed byte to a host file.  Always ready:
/// status DFh (not busy, selected, no error).
class FilePrinter : public ParallelDevice {
public:
  explicit FilePrinter(std::string path);
  ~FilePrinter() override;
  bool isOpen() const { return m_file != nullptr; }

  bool strobe(uint8_t data, uint64_t nowNs) override;
  uint8_t status(uint8_t data, uint64_t nowNs) override;
  void poll(uint64_t nowNs) override;
  std::string describe() const override { return "file:" + m_path; }

private:
  std::string m_path;
  std::FILE *m_file = nullptr;
  bool m_dirty = false;
};

/// Parses a --lptN= value: off (no port), none, file:PATH, covox or
/// disney.  On success sets `present` and `device` (nullptr for none/off).
bool makeParallelDevice(const std::string &spec, bool &present,
                        std::unique_ptr<ParallelDevice> &device,
                        std::string &error);

} // namespace fador::hw
