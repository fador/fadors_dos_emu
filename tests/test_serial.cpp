// 16550A UART, serial mouse and serial backends.

#include "test_framework.hpp"
#include "hw/SerialBackends.hpp"
#include "hw/SerialMouse.hpp"
#include "hw/Uart16550.hpp"
#include "utils/HostSocket.hpp"
#include <chrono>
#include <thread>
#include <vector>

using namespace fador;
using hw::Uart16550;

namespace {

constexpr uint16_t kBase = 0x3F8;

// Records what the UART transmits and lets the test feed received bytes
// and modem inputs.
struct FakeBackend : hw::SerialBackend {
  struct Sent {
    uint8_t byte;
    uint64_t ns;
  };
  std::vector<Sent> *sent;
  std::vector<uint8_t> toSend;
  uint8_t inputs = 0;
  bool dtr = false, rts = false;
  explicit FakeBackend(std::vector<Sent> *log) : sent(log) {}
  void transmit(uint8_t byte, uint64_t ns) override { sent->push_back({byte, ns}); }
  void controlLines(bool d, bool r, uint64_t) override {
    dtr = d;
    rts = r;
  }
  uint8_t modemStatus() const override { return inputs; }
  bool hasData() const override { return !toSend.empty(); }
  uint8_t take() override {
    const uint8_t b = toSend.front();
    toSend.erase(toSend.begin());
    return b;
  }
  std::string describe() const override { return "fake"; }
};

struct UartEnv {
  uint64_t now = 1'000'000; // ns
  Uart16550 uart{kBase};
  int irqs = 0;
  UartEnv() {
    uart.setTimeSource([this] { return now; });
    uart.setIrqCallback([this] { ++irqs; });
  }
  uint8_t in(uint8_t reg) { return uart.read8(kBase + reg); }
  void out(uint8_t reg, uint8_t v) { uart.write8(kBase + reg, v); }
  void setLine(uint16_t divisor, uint8_t lcr) {
    out(Uart16550::LCR, 0x80);
    out(Uart16550::DLL, divisor & 0xFF);
    out(Uart16550::DLM, divisor >> 8);
    out(Uart16550::LCR, lcr);
  }
  void advance(uint64_t ns) {
    now += ns;
    uart.advanceTo(now);
  }
};

} // namespace

TEST_CASE("UART DLAB selects the divisor latch, SCR and IER bits", "[serial][uart]") {
  UartEnv e;
  e.out(Uart16550::LCR, 0x83);
  e.out(Uart16550::DLL, 0x0C);
  e.out(Uart16550::DLM, 0x00);
  REQUIRE(e.in(Uart16550::DLL) == 0x0C);
  REQUIRE(e.in(Uart16550::DLM) == 0x00);
  REQUIRE(e.in(Uart16550::LCR) == 0x83);
  e.out(Uart16550::LCR, 0x03);
  REQUIRE(e.uart.baudRate() == 9600.0);
  // With DLAB clear offset 1 is IER (upper four bits read as zero).
  e.out(Uart16550::IER, 0xFF);
  REQUIRE(e.in(Uart16550::IER) == 0x0F);
  e.out(Uart16550::IER, 0x00);
  e.out(Uart16550::LCR, 0x80);
  REQUIRE(e.in(Uart16550::DLL) == 0x0C); // divisor untouched by IER writes
  e.out(Uart16550::LCR, 0x03);
  e.out(Uart16550::SCR, 0x5A);
  REQUIRE(e.in(Uart16550::SCR) == 0x5A);
  // After reset: no interrupt, transmitter empty, MCR bits 5-7 zero.
  REQUIRE(e.in(Uart16550::IIR) == 0x01);
  REQUIRE(e.in(Uart16550::LSR) == 0x60);
  e.out(Uart16550::MCR, 0xFF);
  REQUIRE(e.in(Uart16550::MCR) == 0x1F);
}

TEST_CASE("UART character time follows baud rate and frame format", "[serial][uart]") {
  UartEnv e;
  e.setLine(12, 0x03); // 9600 8N1: 10 bits
  REQUIRE(e.uart.charTimeNs() == 10ull * 12 * 1'000'000'000 / 115200);
  e.setLine(96, 0x02); // 1200 7N1: 9 bits (serial mouse)
  REQUIRE(e.uart.charTimeNs() == 7'500'000);
  e.setLine(1, 0x1F); // 115200 8E2: 12 bits
  REQUIRE(e.uart.charTimeNs() == 12ull * 1'000'000'000 / 115200);
  e.setLine(1, 0x04); // 5 data bits, 1.5 stop bits: 7.5 bits
  REQUIRE(e.uart.charTimeNs() == 15ull * 1'000'000'000 / 230400);
}

TEST_CASE("UART LSR THRE/TEMT timing at 9600 baud", "[serial][uart]") {
  UartEnv e;
  std::vector<FakeBackend::Sent> sent;
  e.uart.setBackend(std::make_unique<FakeBackend>(&sent));
  e.setLine(12, 0x03);
  const uint64_t charNs = e.uart.charTimeNs();
  const uint64_t t0 = e.now;
  e.out(Uart16550::THR, 'A');
  // 'A' moved to the shift register: THR empty again, transmitter busy.
  REQUIRE(e.in(Uart16550::LSR) == 0x20);
  e.out(Uart16550::THR, 'B'); // waits in THR
  REQUIRE(e.in(Uart16550::LSR) == 0x00);
  e.now = t0 + charNs - 1;
  REQUIRE(e.in(Uart16550::LSR) == 0x00);
  REQUIRE(sent.empty());
  e.now = t0 + charNs;
  REQUIRE(e.in(Uart16550::LSR) == 0x20); // 'B' now shifting out
  REQUIRE(sent.size() == 1);
  REQUIRE(sent[0].byte == 'A');
  REQUIRE(sent[0].ns == t0 + charNs);
  e.now = t0 + 2 * charNs;
  REQUIRE(e.in(Uart16550::LSR) == 0x60);
  REQUIRE(sent.size() == 2);
  REQUIRE(sent[1].byte == 'B');
  REQUIRE(sent[1].ns == t0 + 2 * charNs);
}

TEST_CASE("UART loopback round trip and MSR deltas", "[serial][uart]") {
  UartEnv e;
  std::vector<FakeBackend::Sent> sent;
  auto backend = std::make_unique<FakeBackend>(&sent);
  FakeBackend *fake = backend.get();
  fake->inputs = hw::SerialBackend::kCTS | hw::SerialBackend::kDSR;
  e.uart.setBackend(std::move(backend));
  e.setLine(1, 0x03);
  e.out(Uart16550::MCR, 0x03); // DTR + RTS on the cable
  REQUIRE(fake->dtr);
  REQUIRE(fake->rts);
  e.in(Uart16550::MSR);

  // Loopback with all outputs off: the cable sees DTR/RTS inactive and the
  // inputs follow MCR (all low now).
  e.out(Uart16550::MCR, 0x10);
  REQUIRE(!fake->dtr);
  REQUIRE(!fake->rts);
  REQUIRE(e.in(Uart16550::MSR) == 0x03); // CTS and DSR fell
  REQUIRE(e.in(Uart16550::MSR) == 0x00);
  e.out(Uart16550::MCR, 0x1F);
  REQUIRE(e.in(Uart16550::MSR) == 0xFB); // all high; deltas except TERI
  REQUIRE(e.in(Uart16550::MSR) == 0xF0);
  e.out(Uart16550::MCR, 0x1B); // OUT1 off: trailing edge of RI
  REQUIRE(e.in(Uart16550::MSR) == 0xB4);
  e.out(Uart16550::MCR, 0x13); // OUT2 off: DCD falls
  REQUIRE(e.in(Uart16550::MSR) == 0x38);

  // Data written comes back after one character time, not on the cable.
  e.out(Uart16550::THR, 0x55);
  REQUIRE((e.in(Uart16550::LSR) & 0x01) == 0);
  e.advance(e.uart.charTimeNs());
  REQUIRE(e.in(Uart16550::LSR) == 0x61);
  REQUIRE(e.in(Uart16550::RBR) == 0x55);
  REQUIRE(e.in(Uart16550::LSR) == 0x60);
  REQUIRE(sent.empty());

  // Leaving loopback restores the real inputs and the cable outputs.
  e.out(Uart16550::MCR, 0x03);
  REQUIRE(fake->dtr);
  REQUIRE((e.in(Uart16550::MSR) & 0xF0) == 0x30);
}

TEST_CASE("UART IIR reports interrupts in 16550 priority order", "[serial][uart]") {
  UartEnv e;
  e.setLine(1, 0x03);
  e.out(Uart16550::MCR, 0x10); // loopback
  e.in(Uart16550::MSR);
  e.out(Uart16550::IER, 0x0F);
  REQUIRE(e.in(Uart16550::IIR) == 0x02); // THR empty
  REQUIRE(e.in(Uart16550::IIR) == 0x01); // reading IIR acknowledged it
  // Two characters without reading: the second overruns RBR.
  e.out(Uart16550::THR, 'X');
  e.advance(e.uart.charTimeNs());
  e.out(Uart16550::THR, 'Y');
  e.advance(e.uart.charTimeNs());
  e.out(Uart16550::MCR, 0x11); // DTR in loopback: DSR delta
  REQUIRE(e.in(Uart16550::IIR) == 0x06); // line status (overrun) first
  REQUIRE(e.in(Uart16550::LSR) == 0x63); // OE | DR | THRE | TEMT
  REQUIRE(e.in(Uart16550::IIR) == 0x04); // received data
  REQUIRE(e.in(Uart16550::RBR) == 'Y');  // 16450 mode: new char overwrote
  REQUIRE(e.in(Uart16550::IIR) == 0x02); // THRE (set again after 'Y')
  REQUIRE(e.in(Uart16550::IIR) == 0x00); // modem status
  REQUIRE(e.in(Uart16550::MSR) == 0x22);
  REQUIRE(e.in(Uart16550::IIR) == 0x01);
}

TEST_CASE("UART FIFO trigger level and character timeout", "[serial][uart][fifo]") {
  UartEnv e;
  e.setLine(1, 0x03);
  e.out(Uart16550::MCR, 0x10);
  e.out(Uart16550::FCR, 0xC7); // enable, clear both, trigger 14
  REQUIRE((e.in(Uart16550::IIR) & 0xC0) == 0xC0); // 16550A signature
  e.out(Uart16550::IER, 0x01);
  const uint64_t c = e.uart.charTimeNs();
  for (int i = 0; i < 16; ++i)
    e.out(Uart16550::THR, static_cast<uint8_t>('a' + i)); // TX FIFO holds 16
  REQUIRE((e.in(Uart16550::LSR) & 0x20) == 0);
  e.advance(13 * c);
  REQUIRE(e.in(Uart16550::IIR) == 0xC1); // 13 < trigger 14
  e.advance(c);
  REQUIRE(e.in(Uart16550::IIR) == 0xC4);
  e.advance(2 * c);
  for (int i = 0; i < 14; ++i)
    REQUIRE(e.in(Uart16550::RBR) == 'a' + i);
  REQUIRE(e.in(Uart16550::IIR) == 0xC1); // 2 left, below the trigger
  e.advance(4 * c - 1);
  REQUIRE(e.in(Uart16550::IIR) == 0xC1);
  e.advance(1);
  REQUIRE(e.in(Uart16550::IIR) == 0xCC); // character timeout
  REQUIRE(e.in(Uart16550::RBR) == 'o');
  REQUIRE(e.in(Uart16550::IIR) == 0xC1); // reading RBR cleared it
  e.advance(4 * c);
  REQUIRE(e.in(Uart16550::IIR) == 0xCC);
  REQUIRE(e.in(Uart16550::RBR) == 'p');
  e.advance(10 * c);
  REQUIRE(e.in(Uart16550::IIR) == 0xC1); // empty FIFO: no timeout

  // FIFO overrun keeps the FIFO and drops the new character.
  e.out(Uart16550::FCR, 0x01);
  for (int i = 0; i < 17; ++i) {
    e.out(Uart16550::THR, static_cast<uint8_t>(i));
    e.advance(c);
  }
  REQUIRE((e.in(Uart16550::LSR) & 0x03) == 0x03);
  for (int i = 0; i < 16; ++i)
    REQUIRE(e.in(Uart16550::RBR) == i);
  REQUIRE((e.in(Uart16550::LSR) & 0x01) == 0);
  // Disabling the FIFOs returns to 16450 mode.
  e.out(Uart16550::FCR, 0x00);
  REQUIRE((e.in(Uart16550::IIR) & 0xC0) == 0x00);
}

TEST_CASE("UART IRQ output is gated by OUT2 and edge triggered", "[serial][uart][irq]") {
  UartEnv e;
  std::vector<FakeBackend::Sent> sent;
  auto backend = std::make_unique<FakeBackend>(&sent);
  FakeBackend *fake = backend.get();
  e.uart.setBackend(std::move(backend));
  e.setLine(1, 0x03);
  e.out(Uart16550::IER, 0x02); // THRE interrupt pending at once
  REQUIRE(e.irqs == 0);        // OUT2 off: line held inactive
  e.out(Uart16550::MCR, 0x08);
  REQUIRE(e.irqs == 1);
  REQUIRE(e.uart.irqLine());
  REQUIRE(e.in(Uart16550::IIR) == 0x02);
  REQUIRE(!e.uart.irqLine());
  e.out(Uart16550::THR, 'Z'); // straight to the shift register: THRE again
  REQUIRE(e.irqs == 2);
  e.in(Uart16550::IIR);
  e.out(Uart16550::THR, 'Z'); // stays in THR until 'Z' is sent
  REQUIRE(e.irqs == 2);
  e.advance(e.uart.charTimeNs());
  REQUIRE(e.irqs == 3); // raised from the main loop's advanceTo()
  e.in(Uart16550::IIR);
  // Received data from the backend.
  e.out(Uart16550::IER, 0x01);
  fake->toSend = {0x42};
  e.uart.pollBackend(e.now);
  REQUIRE(e.irqs == 3);
  REQUIRE(e.uart.nextEventNs() == e.now + e.uart.charTimeNs());
  e.advance(e.uart.charTimeNs());
  REQUIRE(e.irqs == 4);
  REQUIRE(e.in(Uart16550::IIR) == 0x04);
  REQUIRE(e.in(Uart16550::RBR) == 0x42);
  REQUIRE(!e.uart.irqLine());
  // Loopback disconnects the IRQ pin.
  e.out(Uart16550::MCR, 0x18);
  e.out(Uart16550::IER, 0x02);
  REQUIRE(!e.uart.irqLine());
  REQUIRE(e.irqs == 4);
}

TEST_CASE("Serial mouse identifies itself and sends Microsoft packets", "[serial][mouse]") {
  UartEnv e;
  hw::SerialMouse::Input input;
  e.uart.setBackend(std::make_unique<hw::SerialMouse>([&] { return input; }));
  e.setLine(96, 0x02); // 1200 baud 7N1
  e.out(Uart16550::FCR, 0x07);
  e.out(Uart16550::IER, 0x01);
  const uint64_t c = e.uart.charTimeNs();
  REQUIRE(c == 7'500'000);

  // Powering the mouse (DTR + RTS) resets it: it answers 'M'.
  e.out(Uart16550::MCR, 0x0B);
  REQUIRE((e.in(Uart16550::LSR) & 0x01) == 0);
  e.advance(c);
  REQUIRE(e.irqs == 1);
  REQUIRE(e.in(Uart16550::RBR) == 'M');

  // Motion +5,-3 with the left button: one three-byte packet.
  input.motionX = 5;
  input.motionY = -3;
  input.buttons = 1;
  e.uart.pollBackend(e.now);
  e.advance(3 * c);
  REQUIRE(e.in(Uart16550::RBR) == 0x6C); // 0 1 L=1 R=0 Y7Y6=11 X7X6=00
  REQUIRE(e.in(Uart16550::RBR) == 0x05);
  REQUIRE(e.in(Uart16550::RBR) == 0x3D);
  e.uart.pollBackend(e.now);
  e.advance(3 * c);
  REQUIRE((e.in(Uart16550::LSR) & 0x01) == 0); // no change: no packet

  // Large motion is split into packets of at most 127.
  input.motionX += 200;
  input.buttons = 2;
  e.uart.pollBackend(e.now);
  e.advance(3 * c);
  REQUIRE(e.in(Uart16550::RBR) == 0x51); // R, X7X6 = 01
  REQUIRE(e.in(Uart16550::RBR) == 0x3F);
  REQUIRE(e.in(Uart16550::RBR) == 0x00);
  e.uart.pollBackend(e.now);
  e.advance(3 * c);
  REQUIRE(e.in(Uart16550::RBR) == 0x51); // remaining 73 = 01 001001
  REQUIRE(e.in(Uart16550::RBR) == 0x09);
  REQUIRE(e.in(Uart16550::RBR) == 0x00);

  // Dropping RTS powers it off; raising it again re-identifies.
  e.out(Uart16550::MCR, 0x09);
  input.motionX += 10;
  e.uart.pollBackend(e.now);
  e.advance(3 * c);
  REQUIRE((e.in(Uart16550::LSR) & 0x01) == 0);
  e.out(Uart16550::MCR, 0x0B);
  e.advance(c);
  REQUIRE(e.in(Uart16550::RBR) == 'M');
  e.uart.pollBackend(e.now);
  e.advance(3 * c);
  REQUIRE((e.in(Uart16550::LSR) & 0x01) == 0); // motion before reset is gone

  uint8_t packet[3];
  hw::SerialMouse::encodePacket(-1000, 1000, true, true, packet);
  REQUIRE(packet[0] == 0x40 + 0x20 + 0x10 + 0x04 + 0x02); // -128, +127
  REQUIRE(packet[1] == 0x00);
  REQUIRE(packet[2] == 0x3F);
}

TEST_CASE("Serial backend specs are parsed", "[serial]") {
  bool present = false;
  std::unique_ptr<hw::SerialBackend> backend;
  std::string error;
  REQUIRE(hw::makeSerialBackend("none", {}, present, backend, error));
  REQUIRE(present);
  REQUIRE(!backend);
  REQUIRE(hw::makeSerialBackend("off", {}, present, backend, error));
  REQUIRE(!present);
  REQUIRE(hw::makeSerialBackend("mouse", {}, present, backend, error));
  REQUIRE(backend->describe() == "mouse");
  REQUIRE(!hw::makeSerialBackend("tcp:host", {}, present, backend, error));
  REQUIRE(!hw::makeSerialBackend("listen:99999", {}, present, backend, error));
  REQUIRE(!hw::makeSerialBackend("bogus", {}, present, backend, error));
}

TEST_CASE("Serial file backend writes transmitted bytes", "[serial]") {
  const std::string path = "T_SERIAL_OUT.TMP";
  {
    UartEnv e;
    bool present = false;
    std::unique_ptr<hw::SerialBackend> backend;
    std::string error;
    REQUIRE(hw::makeSerialBackend("file:" + path, {}, present, backend, error));
    REQUIRE(backend->modemStatus() == 0xB0); // CTS, DSR, DCD
    e.uart.setBackend(std::move(backend));
    e.setLine(1, 0x03);
    e.out(Uart16550::FCR, 0x01);
    for (char ch : std::string("Hi!"))
      e.out(Uart16550::THR, static_cast<uint8_t>(ch));
    e.advance(3 * e.uart.charTimeNs());
    e.uart.pollBackend(e.now);
  }
  std::FILE *f = std::fopen(path.c_str(), "rb");
  REQUIRE(f != nullptr);
  char buf[8] = {};
  const size_t n = std::fread(buf, 1, sizeof buf, f);
  std::fclose(f);
  std::remove(path.c_str());
  REQUIRE(n == 3);
  REQUIRE(std::string(buf, n) == "Hi!");
}

TEST_CASE("Serial TCP null modem passes bytes both ways", "[serial][tcp]") {
  // Uses a fixed local port; skipped when it is taken.
  constexpr uint16_t kPort = 47213;
  hw::TcpSerialBackend server(kPort);
  if (!server.isValid())
    return;
  hw::TcpSerialBackend client("127.0.0.1", kPort);
  uint64_t now = 0;
  bool up = false;
  for (int i = 0; i < 400 && !up; ++i) {
    client.poll(now);
    server.poll(now);
    up = client.modemStatus() && server.modemStatus();
    if (!up)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  REQUIRE(up);
  client.transmit('x', now);
  client.transmit('y', now);
  server.transmit('z', now);
  std::string atServer, atClient;
  for (int i = 0; i < 400 && (atServer.size() < 2 || atClient.empty()); ++i) {
    client.poll(now);
    server.poll(now);
    while (server.hasData())
      atServer += static_cast<char>(server.take());
    while (client.hasData())
      atClient += static_cast<char>(client.take());
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  REQUIRE(atServer == "xy");
  REQUIRE(atClient == "z");
}
