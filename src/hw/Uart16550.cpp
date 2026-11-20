#include "Uart16550.hpp"
#include "../utils/Logger.hpp"
#include <algorithm>

namespace fador::hw {

Uart16550::Uart16550(uint16_t basePort) : m_base(basePort) {}

Uart16550::~Uart16550() = default;

void Uart16550::setTimeSource(std::function<uint64_t()> nowNs) {
  m_timeSource = std::move(nowNs);
  m_nowNs = m_timeSource ? m_timeSource() : 0;
  m_rxLineFreeNs = m_rxTimerNs = m_nowNs;
}

void Uart16550::setBackend(std::unique_ptr<SerialBackend> backend) {
  m_backend = std::move(backend);
  m_dtrOut = m_rtsOut = false;
  const uint64_t now = nowNs();
  if (m_backend)
    m_backend->controlLines(false, false, now);
  driveControlLines(now);
  refreshModemInputs();
  recomputeNextEvent();
}

uint64_t Uart16550::charTimeNs() const {
  const uint32_t dataBits = 5u + (m_lcr & 3u);
  const uint32_t parity = (m_lcr & 0x08) ? 1u : 0u;
  // Stop bits in half bits: 1, or 1.5 (5 data bits) / 2 with LCR bit 2.
  const uint32_t stopHalf = (m_lcr & 0x04) ? (dataBits == 5 ? 3u : 4u) : 2u;
  const uint64_t halfBits = 2u * (1u + dataBits + parity) + stopHalf;
  const uint64_t divisor = m_divisor ? m_divisor : 0x10000u;
  return halfBits * divisor * 1'000'000'000ull / (2ull * kBaseClockHz);
}

double Uart16550::baudRate() const {
  return static_cast<double>(kBaseClockHz) / (m_divisor ? m_divisor : 0x10000);
}

uint64_t Uart16550::rxTimeoutAt() const {
  return m_rxTimerNs + 4 * charTimeNs();
}

bool Uart16550::rxDataAvailable() const {
  return m_fifoEnabled ? m_rxFifo.size() >= m_rxTrigger : !m_rxFifo.empty();
}

// ── Time ──────────────────────────────────────────────────────────────

void Uart16550::sync(uint64_t now) {
  if (now < m_nowNs)
    now = m_nowNs;
  // Process transmitter, receiver and timeout events in time order.
  for (;;) {
    const uint64_t tx = m_tsrBusy ? m_txDoneNs : kNever;
    const uint64_t rx = m_rxBusy ? m_rxDoneNs : kNever;
    const uint64_t to = (!m_timeoutPending && m_fifoEnabled && !m_rxFifo.empty())
                            ? rxTimeoutAt()
                            : kNever;
    const uint64_t t = std::min({tx, rx, to});
    if (t > now)
      break;
    if (t == tx)
      completeTx(t);
    else if (t == rx)
      completeRx(t);
    else
      m_timeoutPending = true;
  }
  m_nowNs = now;
  updateIrq();
  recomputeNextEvent();
}

void Uart16550::recomputeNextEvent() {
  uint64_t next = kNever;
  if (m_tsrBusy)
    next = std::min(next, m_txDoneNs);
  if (m_rxBusy)
    next = std::min(next, m_rxDoneNs);
  if (!m_timeoutPending && m_fifoEnabled && !m_rxFifo.empty())
    next = std::min(next, rxTimeoutAt());
  m_nextEventNs = next;
}

void Uart16550::pollBackend(uint64_t now) {
  if (!m_backend)
    return;
  sync(now);
  m_backend->poll(m_nowNs);
  if (!loopback()) {
    refreshModemInputs();
    tryStartRx(m_nowNs);
  }
  updateIrq();
  recomputeNextEvent();
}

// ── Transmitter / receiver ────────────────────────────────────────────

void Uart16550::startTx(uint64_t t) {
  m_tsrByte = m_txFifo.front();
  m_txFifo.pop_front();
  m_tsrBusy = true;
  m_txDoneNs = t + charTimeNs();
  if (m_txFifo.empty())
    m_thrIntPending = true; // THR (or the transmit FIFO) just became empty
}

void Uart16550::completeTx(uint64_t t) {
  m_tsrBusy = false;
  m_txDoneNs = kNever;
  if (loopback())
    receiveByte(m_tsrByte, t);
  else if (m_backend)
    m_backend->transmit(m_tsrByte, t);
  if (!m_txFifo.empty())
    startTx(t);
}

void Uart16550::tryStartRx(uint64_t t) {
  if (m_rxBusy || loopback() || !m_backend || !m_backend->hasData())
    return;
  m_rxByte = m_backend->take();
  m_rxBusy = true;
  m_rxDoneNs = std::max(t, m_rxLineFreeNs) + charTimeNs();
}

void Uart16550::completeRx(uint64_t t) {
  m_rxBusy = false;
  m_rxDoneNs = kNever;
  m_rxLineFreeNs = t;
  receiveByte(m_rxByte, t);
  tryStartRx(t);
}

void Uart16550::receiveByte(uint8_t byte, uint64_t t) {
  if (m_rxFifo.size() >= rxCapacity()) {
    m_lsrErrors |= LSR_OE;
    // 16450 mode: the new character overwrites RBR.  FIFO mode: the
    // character in the shift register is lost, the FIFO is kept.
    if (!m_fifoEnabled)
      m_rxFifo.back() = byte;
  } else {
    m_rxFifo.push_back(byte);
  }
  m_rxTimerNs = t;
}

// ── Modem lines ───────────────────────────────────────────────────────

void Uart16550::setModemInputs(uint8_t inputs) {
  inputs &= 0xF0;
  const uint8_t old = m_msr & 0xF0;
  const uint8_t changed = old ^ inputs;
  uint8_t delta = 0;
  if (changed & SerialBackend::kCTS)
    delta |= 0x01;
  if (changed & SerialBackend::kDSR)
    delta |= 0x02;
  if ((old & SerialBackend::kRI) && !(inputs & SerialBackend::kRI))
    delta |= 0x04; // trailing edge of RI
  if (changed & SerialBackend::kDCD)
    delta |= 0x08;
  m_msr = static_cast<uint8_t>(inputs | (m_msr & 0x0F) | delta);
}

void Uart16550::refreshModemInputs() {
  if (loopback()) {
    uint8_t in = 0;
    if (m_mcr & MCR_RTS)
      in |= SerialBackend::kCTS;
    if (m_mcr & MCR_DTR)
      in |= SerialBackend::kDSR;
    if (m_mcr & MCR_OUT1)
      in |= SerialBackend::kRI;
    if (m_mcr & MCR_OUT2)
      in |= SerialBackend::kDCD;
    setModemInputs(in);
  } else {
    setModemInputs(m_backend ? m_backend->modemStatus() : 0);
  }
}

void Uart16550::driveControlLines(uint64_t t) {
  const bool dtr = !loopback() && (m_mcr & MCR_DTR);
  const bool rts = !loopback() && (m_mcr & MCR_RTS);
  if (dtr == m_dtrOut && rts == m_rtsOut)
    return;
  m_dtrOut = dtr;
  m_rtsOut = rts;
  if (m_backend)
    m_backend->controlLines(dtr, rts, t);
}

// ── Interrupts ────────────────────────────────────────────────────────

uint8_t Uart16550::interruptId() const {
  if ((m_ier & 0x04) && (m_lsrErrors & (LSR_OE | LSR_PE | LSR_FE | LSR_BI)))
    return IIR_RLS;
  if (m_ier & 0x01) {
    if (rxDataAvailable())
      return IIR_RDA;
    if (m_timeoutPending)
      return IIR_TIMEOUT;
  }
  if ((m_ier & 0x02) && m_thrIntPending)
    return IIR_THRE;
  if ((m_ier & 0x08) && (m_msr & 0x0F))
    return IIR_MODEM;
  return IIR_NONE;
}

void Uart16550::updateIrq() {
  const bool line = interruptId() != IIR_NONE && (m_mcr & MCR_OUT2) &&
                    !loopback();
  if (line && !m_irqLine && m_raiseIrq)
    m_raiseIrq();
  m_irqLine = line;
}

uint8_t Uart16550::lineStatus() const {
  uint8_t lsr = m_lsrErrors;
  if (!m_rxFifo.empty())
    lsr |= LSR_DR;
  if (m_txFifo.empty()) {
    lsr |= LSR_THRE;
    if (!m_tsrBusy)
      lsr |= LSR_TEMT;
  }
  return lsr;
}

// ── Port access ───────────────────────────────────────────────────────

uint8_t Uart16550::read8(uint16_t port) {
  sync(nowNs());
  const bool dlab = (m_lcr & 0x80) != 0;
  uint8_t value = 0xFF;
  switch ((port - m_base) & 7) {
  case RBR:
    if (dlab) {
      value = static_cast<uint8_t>(m_divisor & 0xFF);
      break;
    }
    if (!m_rxFifo.empty()) {
      m_lastRbr = m_rxFifo.front();
      m_rxFifo.pop_front();
    }
    value = m_lastRbr;
    m_timeoutPending = false;
    m_rxTimerNs = m_nowNs;
    break;
  case IER:
    value = dlab ? static_cast<uint8_t>(m_divisor >> 8) : m_ier;
    break;
  case IIR: {
    const uint8_t id = interruptId();
    if (id == IIR_THRE)
      m_thrIntPending = false; // reading IIR acknowledges THRE
    value = static_cast<uint8_t>(id | (m_fifoEnabled ? 0xC0 : 0x00));
    break;
  }
  case LCR:
    value = m_lcr;
    break;
  case MCR:
    value = m_mcr;
    break;
  case LSR:
    value = lineStatus();
    m_lsrErrors = 0;
    break;
  case MSR:
    if (!loopback())
      refreshModemInputs();
    value = m_msr;
    m_msr &= 0xF0; // reading MSR clears the delta bits
    break;
  case SCR:
    value = m_scr;
    break;
  }
  updateIrq();
  recomputeNextEvent();
  return value;
}

void Uart16550::write8(uint16_t port, uint8_t value) {
  sync(nowNs());
  const bool dlab = (m_lcr & 0x80) != 0;
  switch ((port - m_base) & 7) {
  case THR:
    if (dlab) {
      m_divisor = static_cast<uint16_t>((m_divisor & 0xFF00) | value);
      break;
    }
    if (m_txFifo.size() < txCapacity())
      m_txFifo.push_back(value);
    m_thrIntPending = false;
    if (!m_tsrBusy)
      startTx(m_nowNs);
    break;
  case IER:
    if (dlab) {
      m_divisor = static_cast<uint16_t>((m_divisor & 0x00FF) | (value << 8));
      break;
    }
    m_ier = value & 0x0F;
    // Enabling (or re-enabling) the THRE interrupt while THR is empty
    // raises it at once; drivers use this to start transmitting.
    if ((m_ier & 0x02) && m_txFifo.empty())
      m_thrIntPending = true;
    break;
  case FCR: {
    const bool enable = (value & 0x01) != 0;
    if (enable != m_fifoEnabled) {
      if (!m_txFifo.empty())
        m_thrIntPending = true;
      m_rxFifo.clear();
      m_txFifo.clear();
      m_timeoutPending = false;
      m_fifoEnabled = enable;
    }
    if (!enable)
      break; // the other bits need FCR bit 0 set
    if (value & 0x02) {
      m_rxFifo.clear();
      m_timeoutPending = false;
    }
    if ((value & 0x04) && !m_txFifo.empty()) {
      m_txFifo.clear();
      m_thrIntPending = true;
    }
    static constexpr uint8_t kTrigger[4] = {1, 4, 8, 14};
    m_rxTrigger = kTrigger[value >> 6];
    break;
  }
  case LCR:
    m_lcr = value;
    break;
  case MCR: {
    const bool wasLoop = loopback();
    m_mcr = value & 0x1F;
    if (wasLoop != loopback())
      LOG_DEBUG("UART ", std::hex, m_base, ": loopback ",
                loopback() ? "on" : "off");
    refreshModemInputs();
    driveControlLines(m_nowNs);
    tryStartRx(m_nowNs);
    break;
  }
  case LSR:
  case MSR:
    break; // read-only (factory test registers)
  case SCR:
    m_scr = value;
    break;
  }
  updateIrq();
  recomputeNextEvent();
}

} // namespace fador::hw
