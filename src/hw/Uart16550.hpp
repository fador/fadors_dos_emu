#pragma once

#include "IODevice.hpp"
#include "SerialBackend.hpp"
#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>

namespace fador::hw {

/// National Semiconductor 16550A UART (one COM port, 8 registers at
/// base..base+7).
///
/// Registers: RBR/THR and IER (DLL/DLM with LCR.DLAB), IIR/FCR, LCR, MCR,
/// LSR, MSR, SCR.  The receive and transmit FIFOs hold 16 characters when
/// enabled (FCR bit 0), one otherwise.  Characters take their real line
/// time (start + data + parity + stop bits at 115200 / divisor baud) on
/// emulated time: the time source is read lazily on port access and the
/// main loop calls advanceTo() between batches, which is a single compare
/// until the next character completes or the receive timeout expires.
///
/// Interrupts follow the 16550A priority order (line status, received
/// data / character timeout, THR empty, modem status).  The IRQ output is
/// gated by MCR OUT2 like on the PC; the callback is invoked on each rising
/// edge of the gated line (the 8259 is edge triggered).  In loopback mode
/// (MCR bit 4) the transmitter feeds the receiver, the modem status inputs
/// follow DTR/RTS/OUT1/OUT2, the cable sees DTR = RTS = 0 and the IRQ pin
/// is disconnected (OUT2 is forced inactive).
class Uart16550 : public IODevice {
public:
  static constexpr uint64_t kNever = std::numeric_limits<uint64_t>::max();
  static constexpr uint32_t kBaseClockHz = 115200; // 1.8432 MHz / 16
  static constexpr size_t kFifoSize = 16;

  // Register offsets
  static constexpr uint8_t RBR = 0, THR = 0, DLL = 0, IER = 1, DLM = 1,
                           IIR = 2, FCR = 2, LCR = 3, MCR = 4, LSR = 5,
                           MSR = 6, SCR = 7;
  // LSR bits
  static constexpr uint8_t LSR_DR = 0x01, LSR_OE = 0x02, LSR_PE = 0x04,
                           LSR_FE = 0x08, LSR_BI = 0x10, LSR_THRE = 0x20,
                           LSR_TEMT = 0x40, LSR_FIFOERR = 0x80;
  // MCR bits
  static constexpr uint8_t MCR_DTR = 0x01, MCR_RTS = 0x02, MCR_OUT1 = 0x04,
                           MCR_OUT2 = 0x08, MCR_LOOP = 0x10;
  // IIR interrupt identification (bits 0-3)
  static constexpr uint8_t IIR_NONE = 0x01, IIR_MODEM = 0x00, IIR_THRE = 0x02,
                           IIR_RDA = 0x04, IIR_RLS = 0x06, IIR_TIMEOUT = 0x0C;

  explicit Uart16550(uint16_t basePort = 0x3F8);
  ~Uart16550() override;

  uint8_t read8(uint16_t port) override;
  void write8(uint16_t port, uint8_t value) override;

  uint16_t basePort() const { return m_base; }

  /// What is attached to the port (nullptr: nothing, all inputs low).
  void setBackend(std::unique_ptr<SerialBackend> backend);
  SerialBackend *backend() const { return m_backend.get(); }

  /// Emulated time in ns (wire with clock.timeSource()).
  void setTimeSource(std::function<uint64_t()> nowNs);
  /// Called on each rising edge of the IRQ output (gated by OUT2).
  void setIrqCallback(std::function<void()> raise) {
    m_raiseIrq = std::move(raise);
  }

  /// Main loop, between batches: completes characters whose line time has
  /// passed and raises the IRQ.  A single compare when nothing is due.
  void advanceTo(uint64_t nowNs) {
    if (nowNs >= m_nextEventNs)
      sync(nowNs);
  }
  /// Main loop, from the periodic host services: lets the backend do host
  /// I/O and starts receiving what it has to send.
  void pollBackend(uint64_t nowNs);

  /// Earliest emulated time at which the UART has something to do.
  uint64_t nextEventNs() const { return m_nextEventNs; }
  /// Duration of one character with the current LCR and divisor.
  uint64_t charTimeNs() const;
  /// Baud rate from the divisor latch.
  double baudRate() const;
  /// State of the IRQ pin after OUT2 / loopback gating.
  bool irqLine() const { return m_irqLine; }
  /// Current emulated time from the time source.
  uint64_t nowNs() const { return m_timeSource ? m_timeSource() : m_nowNs; }

private:
  void sync(uint64_t nowNs);
  void completeTx(uint64_t t);
  void completeRx(uint64_t t);
  void receiveByte(uint8_t byte, uint64_t t);
  void startTx(uint64_t t);
  void tryStartRx(uint64_t t);
  void recomputeNextEvent();
  uint8_t interruptId() const;
  void updateIrq();
  void setModemInputs(uint8_t inputs);
  void refreshModemInputs();
  void driveControlLines(uint64_t t);
  uint8_t lineStatus() const;
  size_t rxCapacity() const { return m_fifoEnabled ? kFifoSize : 1; }
  size_t txCapacity() const { return m_fifoEnabled ? kFifoSize : 1; }
  bool loopback() const { return (m_mcr & MCR_LOOP) != 0; }
  uint64_t rxTimeoutAt() const;
  bool rxDataAvailable() const;

  uint16_t m_base;
  std::unique_ptr<SerialBackend> m_backend;
  std::function<uint64_t()> m_timeSource;
  std::function<void()> m_raiseIrq;
  uint64_t m_nowNs = 0;
  uint64_t m_nextEventNs = kNever;

  // Registers
  uint8_t m_ier = 0;
  uint8_t m_lcr = 0;
  uint8_t m_mcr = 0;
  uint8_t m_msr = 0;       // bits 4-7 inputs, bits 0-3 deltas
  uint8_t m_lsrErrors = 0; // OE (PE/FE/BI are never produced), cleared by reading LSR
  uint8_t m_scr = 0;
  uint16_t m_divisor = 12; // 9600 baud
  bool m_fifoEnabled = false;
  uint8_t m_rxTrigger = 1;
  uint8_t m_lastRbr = 0;

  // Receiver
  std::deque<uint8_t> m_rxFifo;
  bool m_rxBusy = false;       // a character is being shifted in
  uint8_t m_rxByte = 0;
  uint64_t m_rxDoneNs = kNever;
  uint64_t m_rxLineFreeNs = 0; // stop bit of the previous character
  uint64_t m_rxTimerNs = 0;    // last receive or RBR read (timeout timer)
  bool m_timeoutPending = false;

  // Transmitter
  std::deque<uint8_t> m_txFifo;
  bool m_tsrBusy = false;
  uint8_t m_tsrByte = 0;
  uint64_t m_txDoneNs = kNever;
  bool m_thrIntPending = false;

  bool m_irqLine = false;
  bool m_dtrOut = false, m_rtsOut = false; // as last told to the backend
};

} // namespace fador::hw
