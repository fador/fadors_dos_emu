#pragma once
#include "IODevice.hpp"
#include "../memory/MemoryBus.hpp"
#include <array>
#include <cstdint>
#include <functional>

namespace fador::hw {

// CRT timing derived from the programmed CRTC, sequencer and miscellaneous
// output registers.  Used for the input status register (port 3DAh/3BAh)
// and exposed so frontends can pace themselves to the guest's refresh rate.
struct VgaTiming {
  uint64_t linePs = 31778000;   // one scanline in picoseconds
  uint64_t displayPs = 25422000; // active (display-enabled) part of a line
  uint32_t totalLines = 449;    // vertical total + 2
  uint32_t displayLines = 400;  // vertical display end + 1
  uint32_t retraceStart = 412;  // first line of vertical retrace
  uint32_t retraceLines = 2;    // length of vertical retrace in lines
  uint64_t frameNs = 14268000;  // totalLines * linePs / 1000

  double refreshHz() const {
    return frameNs ? 1e9 / static_cast<double>(frameNs) : 0.0;
  }
};

// IBM VGA register file and plane memory.
//
// Ports: attribute controller (3C0h/3C1h), input status 0 / misc output
// (3C2h/3CCh), video subsystem enable (3C3h), sequencer (3C4h/3C5h), DAC
// (3C6h-3C9h), feature control (3CAh read, 3BAh/3DAh write), graphics
// controller (3CEh/3CFh), CRTC (3B4h/3B5h or 3D4h/3D5h, selected by misc
// output bit 0) and input status 1 (3BAh/3DAh).  The DAC palette itself is
// held by MemoryBus::dacPalette() (256 * 3 bytes, 6-bit RGB).
//
// Plane memory follows the hardware layout: four 64 KiB planes.  In chain-4
// mode CPU address A goes to plane A&3 at plane offset A&~3 (so mode 13h
// uses every fourth byte of each plane and the CRTC reads it in doubleword
// mode).  Text modes keep their character/attribute bytes in conventional
// RAM at B8000h/B0000h; the font lives in plane 2 as on real hardware.
class VGAController : public IODevice {
public:
  explicit VGAController(memory::MemoryBus &memory);

  uint8_t read8(uint16_t port) override;
  void write8(uint16_t port, uint8_t value) override;

  // ── Time source ────────────────────────────────────────────────────
  // Retrace status is derived from this clock (nanoseconds, monotonic).
  // Unset (or empty), the host steady_clock is used.
  void setTimeSource(std::function<uint64_t()> nowNs);
  uint64_t nowNs() const;

  const VgaTiming &timing() const {
    if (m_timingDirty)
      recomputeTiming();
    return m_timing;
  }
  // Scanline the beam is on right now (0 = first displayed line).
  uint32_t currentScanline() const;
  // Number of whole frames since the time source's epoch (blink phase).
  uint64_t frameCounter() const;
  // Input status 1 value without the flip-flop side effect.
  uint8_t inputStatus1() const;

  // ── Register access for BIOS HLE and renderers ────────────────────
  uint8_t seqReg(uint8_t index) const { return index < 5 ? m_seq[index] : 0; }
  uint8_t gcReg(uint8_t index) const { return index < 9 ? m_gc[index] : 0; }
  uint8_t crtcReg(uint8_t index) const {
    return index < m_crtc.size() ? m_crtc[index] : 0;
  }
  uint8_t attrReg(uint8_t index) const {
    return index < m_attr.size() ? m_attr[index] : 0;
  }
  uint8_t miscOutput() const { return m_misc; }
  uint8_t pelMask() const { return m_pelMask; }
  // Attribute controller palette address source (index bit 5): 0 blanks
  // the display so the palette can be reloaded.
  bool attrPaletteSourceEnabled() const { return (m_attrIndex & 0x20) != 0; }
  bool screenOff() const { return (m_seq[1] & 0x20) != 0; }

  // Direct register writes (bypass the CRTC protect bit and the attribute
  // flip-flop); used by the BIOS to load mode parameter tables.
  void setSeqReg(uint8_t index, uint8_t value);
  void setGcReg(uint8_t index, uint8_t value);
  void setCrtcReg(uint8_t index, uint8_t value);
  void setAttrReg(uint8_t index, uint8_t value);
  void setMiscOutput(uint8_t value);
  void setPelMask(uint8_t value) { m_pelMask = value; }
  void setAttrPaletteSource(bool enabled) {
    m_attrIndex = static_cast<uint8_t>((m_attrIndex & 0x1F) | (enabled ? 0x20 : 0));
  }
  void resetAttrFlipFlop() { m_attrFlipFlop = false; }

  // Composite CRTC values (overflow bits included)
  uint32_t verticalTotal() const;      // register value (lines - 2)
  uint32_t verticalDisplayEnd() const; // register value (lines - 1)
  uint32_t verticalRetraceStart() const;
  uint32_t lineCompare() const;
  uint8_t maxScanLine() const { return m_crtc[0x09] & 0x1F; }
  bool doubleScan() const { return (m_crtc[0x09] & 0x80) != 0; }
  bool monoAddressing() const { return (m_misc & 0x01) == 0; }

  // Plane state for direct VRAM access in planar modes
  uint8_t getMapMask() const { return m_seq[2] & 0x0F; }
  uint8_t getReadMapSelect() const { return m_gc[4] & 0x03; }
  bool isChain4() const { return (m_seq[4] & 0x08) != 0; }

  // Plane-aware VRAM access (called by MemoryBus for 0xA0000-0xAFFFF)
  void planeWrite8(uint32_t offset, uint8_t value);
  uint8_t planeRead8(uint32_t offset) const;

  // VBE banked framebuffer window.  When enabled, the A0000h-AFFFFh
  // window is a 64 KiB view into SVGA video memory at physical
  // `windowBase` (VBE_LFB_BASE + bank * 64 KiB) instead of the VGA planes.
  void setSvgaWindow(bool enabled, uint32_t windowBase) {
    m_svgaWindowEnabled = enabled;
    m_svgaWindowBase = windowBase;
    updateAccessPath();
  }
  bool svgaWindowEnabled() const { return m_svgaWindowEnabled; }

  // Read / write a specific plane byte (renderer, BIOS font loading)
  uint8_t readPlane(int plane, uint32_t offset) const {
    return m_planes[plane & 3][offset & 0xFFFF];
  }
  void writePlane(int plane, uint32_t offset, uint8_t value) {
    m_planes[plane & 3][offset & 0xFFFF] = value;
  }
  const uint8_t *planeData(int plane) const { return m_planes[plane & 3].data(); }

  // Clear all four 64KB planes (used on video mode change)
  void clearPlanes() {
    for (auto &plane : m_planes)
      plane.fill(0);
  }
  // Clear planes 0, 1 and 3 only (text mode set keeps the font in plane 2)
  void clearPlanesExceptFont() {
    m_planes[0].fill(0);
    m_planes[1].fill(0);
    m_planes[3].fill(0);
  }

  // CRTC start address (for page flipping / display offset)
  uint32_t getDisplayStart() const {
    return (uint32_t(m_crtc[0x0C]) << 8) | m_crtc[0x0D];
  }
  // CRTC cursor location (character address)
  uint32_t getCursorLocation() const {
    return (uint32_t(m_crtc[0x0E]) << 8) | m_crtc[0x0F];
  }

  // CRTC Offset register (index 0x13): the row stride is offset*2
  // memory-address-counter units (scaled by the byte/word/dword mode).
  uint8_t getCRTCOffset() const { return m_crtc[0x13]; }

  static constexpr uint32_t VGA_WINDOW_START = 0xA0000;
  static constexpr uint32_t VGA_WINDOW_SIZE = 0x10000; // 64KB

private:
  enum class AccessPath : uint8_t {
    Svga,         // VBE banked window
    Disabled,     // GC memory map does not decode A0000h
    Chain4Fast,   // chain-4 with an identity graphics-controller pipeline
    Chain4,       // chain-4 through the full write pipeline
    Planar,       // sequential planar (map mask / read map select)
    OddEven       // host odd/even addressing
  };

  void loadLatches(uint32_t planeOffset) const;
  void writeThroughPipeline(uint32_t planeOff, uint8_t value, uint8_t planeMask);
  uint8_t readThroughPipeline(uint32_t planeOff, int plane) const;
  void updateAccessPath();
  void recomputeTiming() const;
  void writeCrtc(uint8_t index, uint8_t value);
  void writeAttr(uint8_t value);
  bool crtcPortActive(uint16_t port) const;

  memory::MemoryBus &m_memory;

  // DAC (3C6h-3C9h)
  uint8_t m_dacWriteIndex = 0;
  uint8_t m_dacReadIndex = 0;
  uint8_t m_dacColorCycle = 0; // 0=R, 1=G, 2=B
  uint8_t m_dacState = 0x03;   // 00 = write mode, 11 = read mode
  uint8_t m_pelMask = 0xFF;

  // Miscellaneous output (3C2h write / 3CCh read).  0x67: colour (3Dx)
  // addressing, RAM enabled, 28 MHz clock, 400-line sync polarity.
  uint8_t m_misc = 0x67;
  uint8_t m_featureControl = 0;
  uint8_t m_videoEnable = 0x01; // 3C3h

  // Sequencer (3C4h/3C5h): 0 reset, 1 clocking mode, 2 map mask,
  // 3 character map select, 4 memory mode.  Default: chain-4, all planes.
  uint8_t m_seqIndex = 0;
  std::array<uint8_t, 5> m_seq{0x03, 0x00, 0x0F, 0x00, 0x0E};

  // Graphics controller (3CEh/3CFh)
  uint8_t m_gcIndex = 0;
  std::array<uint8_t, 9> m_gc{0, 0, 0, 0, 0, 0, 0, 0x0F, 0xFF};
  mutable std::array<uint8_t, 4> m_latches{};

  // CRTC (3x4h/3x5h)
  uint8_t m_crtcIndex = 0;
  std::array<uint8_t, 0x19> m_crtc{};

  // Attribute controller (3C0h/3C1h).  The index register's bit 5 (PAS)
  // must be set for the display to show the palette output.
  uint8_t m_attrIndex = 0x20;
  bool m_attrFlipFlop = false; // false: next 3C0h write is an index
  std::array<uint8_t, 0x15> m_attr{};

  AccessPath m_path = AccessPath::Chain4Fast;

  bool m_svgaWindowEnabled = false;
  uint32_t m_svgaWindowBase = 0;

  // Timing
  std::function<uint64_t()> m_timeSource;
  uint64_t m_epochNs = 0;
  mutable VgaTiming m_timing{};
  mutable bool m_timingDirty = true;

  // VGA plane memory (4 planes × 64KB each = 256KB total)
  std::array<std::array<uint8_t, 65536>, 4> m_planes{};
};

// BIOS-HLE helper: program the CRTC cursor location from the BDA cursor
// position of the active page (text modes).  Code that moves the cursor
// by writing the BDA directly (DOS console output) calls this so the
// hardware cursor follows, as the real BIOS does.
void syncCursorFromBDA(memory::MemoryBus &memory);

} // namespace fador::hw
