// VGA BIOS: mode parameter tables, default DAC palettes, ROM fonts and the
// INT 10h palette (AH=10h), character generator (AH=11h) and alternate
// select (AH=12h) services.
//
// Mode sets load the same register values as the IBM VGA BIOS parameter
// table, so the register-driven renderer (ui/FrameRenderer) shows standard
// modes exactly like tweaked ones.
#include "BIOS.hpp"
#include "VGAController.hpp"
#include "VideoMode.hpp"
#include "../ui/VGAFont.hpp"
#include "../utils/Logger.hpp"

#include <algorithm>
#include <array>

namespace fador::hw {

namespace {

struct VgaModeParams {
  uint8_t misc;
  std::array<uint8_t, 4> seq;   // sequencer registers 1-4
  std::array<uint8_t, 25> crtc; // CRTC 00h-18h
  std::array<uint8_t, 20> attr; // attribute 00h-13h (14h is 0)
  std::array<uint8_t, 9> gc;    // graphics controller 00h-08h
  uint16_t regenSize;           // BDA 44Ch
};

constexpr std::array<uint8_t, 20> kAttrText = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x14, 0x07, 0x38, 0x39,
    0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x0C, 0x00, 0x0F, 0x08};
constexpr std::array<uint8_t, 20> kAttrEga16 = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x14, 0x07, 0x38, 0x39,
    0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x01, 0x00, 0x0F, 0x00};
constexpr std::array<uint8_t, 20> kAttrCga16 = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x10, 0x11,
    0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x01, 0x00, 0x0F, 0x00};
constexpr std::array<uint8_t, 9> kGcText = {0x00, 0x00, 0x00, 0x00, 0x00,
                                            0x10, 0x0E, 0x0F, 0xFF};
constexpr std::array<uint8_t, 9> kGcPlanar = {0x00, 0x00, 0x00, 0x00, 0x00,
                                              0x00, 0x05, 0x0F, 0xFF};

// IBM VGA BIOS video parameter table (400-line text variants)
const VgaModeParams kMode01 = {
    0x67, {0x08, 0x03, 0x00, 0x02},
    {0x2D, 0x27, 0x28, 0x90, 0x2B, 0xA0, 0xBF, 0x1F, 0x00, 0x4F, 0x0D, 0x0E, 0x00,
     0x00, 0x00, 0x00, 0x9C, 0x8E, 0x8F, 0x14, 0x1F, 0x96, 0xB9, 0xA3, 0xFF},
    kAttrText, kGcText, 0x0800};
const VgaModeParams kMode23 = {
    0x67, {0x00, 0x03, 0x00, 0x02},
    {0x5F, 0x4F, 0x50, 0x82, 0x55, 0x81, 0xBF, 0x1F, 0x00, 0x4F, 0x0D, 0x0E, 0x00,
     0x00, 0x00, 0x00, 0x9C, 0x8E, 0x8F, 0x28, 0x1F, 0x96, 0xB9, 0xA3, 0xFF},
    kAttrText, kGcText, 0x1000};
const VgaModeParams kMode45 = {
    0x63, {0x09, 0x03, 0x00, 0x02},
    {0x2D, 0x27, 0x28, 0x90, 0x2B, 0x80, 0xBF, 0x1F, 0x00, 0xC1, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x9C, 0x8E, 0x8F, 0x14, 0x00, 0x96, 0xB9, 0xA2, 0xFF},
    {0x00, 0x13, 0x15, 0x17, 0x02, 0x04, 0x06, 0x07, 0x10, 0x11, 0x12, 0x13,
     0x14, 0x15, 0x16, 0x17, 0x01, 0x00, 0x03, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x0F, 0x0F, 0xFF}, 0x4000};
const VgaModeParams kMode6 = {
    0x63, {0x01, 0x01, 0x00, 0x06},
    {0x5F, 0x4F, 0x50, 0x82, 0x54, 0x80, 0xBF, 0x1F, 0x00, 0xC1, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x9C, 0x8E, 0x8F, 0x28, 0x00, 0x96, 0xB9, 0xC2, 0xFF},
    {0x00, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17, 0x17,
     0x17, 0x17, 0x17, 0x17, 0x01, 0x00, 0x01, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x0F, 0xFF}, 0x4000};
const VgaModeParams kMode7 = {
    0x66, {0x00, 0x03, 0x00, 0x02},
    {0x5F, 0x4F, 0x50, 0x82, 0x55, 0x81, 0xBF, 0x1F, 0x00, 0x4F, 0x0D, 0x0E, 0x00,
     0x00, 0x00, 0x00, 0x9C, 0x8E, 0x8F, 0x28, 0x0F, 0x96, 0xB9, 0xA3, 0xFF},
    {0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x10, 0x18, 0x18, 0x18,
     0x18, 0x18, 0x18, 0x18, 0x0E, 0x00, 0x0F, 0x08},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x0A, 0x0F, 0xFF}, 0x1000};
const VgaModeParams kModeD = {
    0x63, {0x09, 0x0F, 0x00, 0x06},
    {0x2D, 0x27, 0x28, 0x90, 0x2B, 0x80, 0xBF, 0x1F, 0x00, 0xC0, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x9C, 0x8E, 0x8F, 0x14, 0x00, 0x96, 0xB9, 0xE3, 0xFF},
    kAttrCga16, kGcPlanar, 0x2000};
const VgaModeParams kModeE = {
    0x63, {0x01, 0x0F, 0x00, 0x06},
    {0x5F, 0x4F, 0x50, 0x82, 0x54, 0x80, 0xBF, 0x1F, 0x00, 0xC0, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x9C, 0x8E, 0x8F, 0x28, 0x00, 0x96, 0xB9, 0xE3, 0xFF},
    kAttrCga16, kGcPlanar, 0x4000};
const VgaModeParams kModeF = {
    0xA2, {0x01, 0x0F, 0x00, 0x06},
    {0x5F, 0x4F, 0x50, 0x82, 0x54, 0x80, 0xBF, 0x1F, 0x00, 0x40, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x83, 0x85, 0x5D, 0x28, 0x0F, 0x63, 0xBA, 0xE3, 0xFF},
    {0x00, 0x08, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00,
     0x00, 0x18, 0x00, 0x00, 0x0B, 0x00, 0x05, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0xFF}, 0x8000};
const VgaModeParams kMode10 = {
    0xA3, {0x01, 0x0F, 0x00, 0x06},
    {0x5F, 0x4F, 0x50, 0x82, 0x54, 0x80, 0xBF, 0x1F, 0x00, 0x40, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x83, 0x85, 0x5D, 0x28, 0x0F, 0x63, 0xBA, 0xE3, 0xFF},
    kAttrEga16, kGcPlanar, 0x8000};
const VgaModeParams kMode11 = {
    0xE3, {0x01, 0x0F, 0x00, 0x06},
    {0x5F, 0x4F, 0x50, 0x82, 0x54, 0x80, 0x0B, 0x3E, 0x00, 0x40, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0xEA, 0x8C, 0xDF, 0x28, 0x00, 0xE7, 0x04, 0xC3, 0xFF},
    {0x00, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F,
     0x3F, 0x3F, 0x3F, 0x3F, 0x01, 0x00, 0x0F, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x01, 0xFF}, 0xA000};
const VgaModeParams kMode12 = {
    0xE3, {0x01, 0x0F, 0x00, 0x06},
    {0x5F, 0x4F, 0x50, 0x82, 0x54, 0x80, 0x0B, 0x3E, 0x00, 0x40, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0xEA, 0x8C, 0xDF, 0x28, 0x00, 0xE7, 0x04, 0xE3, 0xFF},
    kAttrEga16, kGcPlanar, 0xA000};
const VgaModeParams kMode13 = {
    0x63, {0x01, 0x0F, 0x00, 0x0E},
    {0x5F, 0x4F, 0x50, 0x82, 0x54, 0x80, 0xBF, 0x1F, 0x00, 0x41, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x9C, 0x8E, 0x8F, 0x28, 0x40, 0x96, 0xB9, 0xA3, 0xFF},
    {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
     0x0C, 0x0D, 0x0E, 0x0F, 0x41, 0x00, 0x0F, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x05, 0x0F, 0xFF}, 0xFA00};

const VgaModeParams *findModeParams(uint8_t mode) {
  switch (mode) {
  case 0x00: case 0x01: return &kMode01;
  case 0x02: case 0x03: return &kMode23;
  case 0x04: case 0x05: return &kMode45;
  case 0x06: return &kMode6;
  case 0x07: return &kMode7;
  case 0x0D: return &kModeD;
  case 0x0E: return &kModeE;
  case 0x0F: return &kModeF;
  case 0x10: return &kMode10;
  case 0x11: return &kMode11;
  case 0x12: return &kMode12;
  case 0x13: return &kMode13;
  default: return nullptr;
  }
}

bool isTextMode(uint8_t mode) { return mode <= 0x03 || mode == 0x07; }

// Vertical resolution of a mode's scan-out (used to pick fonts/palettes)
int modeScanLines(uint8_t mode) {
  switch (mode) {
  case 0x04: case 0x05: case 0x06: case 0x0D: case 0x0E: case 0x13: return 200;
  case 0x0F: case 0x10: return 350;
  case 0x11: case 0x12: return 480;
  default: return 400;
  }
}

// ── Default DAC tables ───────────────────────────────────────────────

// 64-colour EGA palette: bits 0-2 = blue/green/red at 2/3 intensity,
// bits 3-5 = blue/green/red at 1/3 intensity.
std::array<uint8_t, 3> egaColour(int i) {
  auto c = [i](int hi, int lo) {
    return static_cast<uint8_t>(((i >> hi) & 1) * 42 + ((i >> lo) & 1) * 21);
  };
  return {c(2, 5), c(1, 4), c(0, 3)};
}

// CGA-compatible 64 entries for 200-line modes: bits 0-2 colour, bit 4
// intensity, with colour 6 shown as brown.
std::array<uint8_t, 3> cgaColour(int i) {
  const uint8_t in = ((i >> 4) & 1) ? 21 : 0;
  std::array<uint8_t, 3> c{static_cast<uint8_t>(((i >> 2) & 1) * 42 + in),
                           static_cast<uint8_t>(((i >> 1) & 1) * 42 + in),
                           static_cast<uint8_t>((i & 1) * 42 + in)};
  if ((i & 0x17) == 0x06)
    c[1] = 21;
  return c;
}

std::array<uint8_t, 3> monoColour(int i) {
  const uint8_t v =
      static_cast<uint8_t>(((i & 0x08) ? 42 : 0) + ((i & 0x10) ? 21 : 0));
  return {v, v, v};
}

// The IBM VGA mode 13h default palette: 16 CGA colours, a 16-step grey
// ramp, 24 hues x 3 saturations x 3 intensities, then 8 black entries.
std::array<uint8_t, 3> default256Colour(int i) {
  static constexpr std::array<std::array<uint8_t, 3>, 16> kCga = {{
      {0, 0, 0}, {0, 0, 42}, {0, 42, 0}, {0, 42, 42},
      {42, 0, 0}, {42, 0, 42}, {42, 21, 0}, {42, 42, 42},
      {21, 21, 21}, {21, 21, 63}, {21, 63, 21}, {21, 63, 63},
      {63, 21, 21}, {63, 21, 63}, {63, 63, 21}, {63, 63, 63}}};
  static constexpr std::array<uint8_t, 16> kGrey = {
      0, 5, 8, 11, 14, 17, 20, 24, 28, 32, 36, 40, 45, 50, 56, 63};
  // Five-step ramps from the "low" to the "high" component for each
  // intensity (63, 28, 16) and saturation (high, medium, low).
  static constexpr uint8_t kSteps[3][3][5] = {
      {{0, 16, 31, 47, 63}, {31, 39, 47, 55, 63}, {45, 49, 54, 58, 63}},
      {{0, 7, 14, 21, 28}, {14, 17, 21, 24, 28}, {20, 22, 24, 26, 28}},
      {{0, 4, 8, 12, 16}, {8, 10, 12, 14, 16}, {11, 12, 13, 15, 16}}};
  if (i < 16)
    return kCga[i];
  if (i < 32) {
    uint8_t g = kGrey[i - 16];
    return {g, g, g};
  }
  if (i >= 248)
    return {0, 0, 0};
  const int k = i - 32;
  const uint8_t *s = kSteps[k / 72][(k / 24) % 3];
  const int hue = k % 24;
  const int sector = hue / 4, step = hue % 4;
  // Hue wheel: blue -> magenta -> red -> yellow -> green -> cyan -> blue
  switch (sector) {
  case 0: return {s[step], s[0], s[4]};      // R rises
  case 1: return {s[4], s[0], s[4 - step]};  // B falls
  case 2: return {s[4], s[step], s[0]};      // G rises
  case 3: return {s[4 - step], s[4], s[0]};  // R falls
  case 4: return {s[0], s[4], s[step]};      // B rises
  default: return {s[0], s[4 - step], s[4]}; // G falls
  }
}

// ROM font variants derived from the 8x16 VGA font
void glyph8x14(int ch, uint8_t *out) {
  for (int r = 0; r < 14; ++r)
    out[r] = ui::kVGAFont8x16[ch][r + 1];
}

void glyph8x8(int ch, uint8_t *out) {
  for (int r = 0; r < 8; ++r)
    out[r] = static_cast<uint8_t>(ui::kVGAFont8x16[ch][r * 2] |
                                  ui::kVGAFont8x16[ch][r * 2 + 1]);
}

// Offset of character map `block` (0-7) inside plane 2
uint32_t fontBlockOffset(uint8_t block) {
  return (block & 3u) * 0x4000u + ((block >> 2) & 1u) * 0x2000u;
}

} // namespace

// ── Mode set ─────────────────────────────────────────────────────────

BIOS::TextScanLines BIOS::textScanLines() const {
  const uint8_t flags = m_memory.read8(0x489);
  if (flags & 0x80)
    return TextScanLines::Lines200;
  if (flags & 0x10)
    return TextScanLines::Lines400;
  return TextScanLines::Lines350;
}

uint8_t BIOS::currentCharHeight() const {
  uint8_t points = m_memory.read8(0x485);
  return points ? points : 16;
}

void BIOS::programVGARegisters(uint8_t mode) {
  if (!m_vga)
    return;
  const VgaModeParams *p = findModeParams(mode);
  if (!p)
    return;

  uint8_t misc = p->misc;
  std::array<uint8_t, 25> crtc = p->crtc;
  if (isTextMode(mode)) {
    switch (textScanLines()) {
    case TextScanLines::Lines350:
      misc = static_cast<uint8_t>((misc & 0x3F) | 0x80);
      crtc[0x06] = 0xBF; crtc[0x07] = 0x1F; crtc[0x09] = 0x4D;
      crtc[0x0A] = 0x0B; crtc[0x0B] = 0x0C; crtc[0x10] = 0x83;
      crtc[0x11] = 0x85; crtc[0x12] = 0x5D; crtc[0x15] = 0x63;
      crtc[0x16] = 0xBA;
      if (mode == 0x07)
        crtc[0x14] = 0x0D;
      break;
    case TextScanLines::Lines200:
      crtc[0x09] = 0xC7; crtc[0x0A] = 0x06; crtc[0x0B] = 0x07;
      if (mode == 0x07)
        crtc[0x14] = 0x07;
      break;
    case TextScanLines::Lines400:
      break;
    }
  }

  m_vga->setSeqReg(0, 0x01); // synchronous reset while the clock changes
  m_vga->setMiscOutput(misc);
  for (uint8_t i = 0; i < 4; ++i)
    m_vga->setSeqReg(static_cast<uint8_t>(i + 1), p->seq[i]);
  m_vga->setSeqReg(0, 0x03);
  for (uint8_t i = 0; i < crtc.size(); ++i)
    m_vga->setCrtcReg(i, crtc[i]);
  for (uint8_t i = 0; i < p->attr.size(); ++i)
    m_vga->setAttrReg(i, p->attr[i]);
  m_vga->setAttrReg(0x14, 0x00);
  for (uint8_t i = 0; i < p->gc.size(); ++i)
    m_vga->setGcReg(i, p->gc[i]);
  m_vga->setPelMask(0xFF);
  m_vga->resetAttrFlipFlop();
  m_vga->setAttrPaletteSource(true);
  LOG_DEBUG("BIOS: Programmed VGA registers for mode 0x", std::hex, (int)mode);
}

void BIOS::loadDefaultDac(uint8_t mode) {
  uint8_t *pal = m_memory.dacPalette();
  auto put = [pal](int i, const std::array<uint8_t, 3> &c) {
    pal[i * 3 + 0] = c[0];
    pal[i * 3 + 1] = c[1];
    pal[i * 3 + 2] = c[2];
  };
  if (mode == 0x13) {
    for (int i = 0; i < 256; ++i)
      put(i, default256Colour(i));
    return;
  }
  const bool mono = mode == 0x07 || mode == 0x0F;
  const bool cga = !mono && (isTextMode(mode)
                                 ? textScanLines() == TextScanLines::Lines200
                                 : modeScanLines(mode) == 200);
  for (int i = 0; i < 64; ++i)
    put(i, mono ? monoColour(i) : cga ? cgaColour(i) : egaColour(i));
}

void BIOS::setVgaMode(uint8_t mode, bool clearScreen) {
  const VideoModeInfo *mi = findVideoMode(mode);
  const VgaModeParams *p = findModeParams(mode);
  if (!mi || !p)
    return;

  const bool text = !mi->isGraphics;
  uint8_t points = 16;
  if (text) {
    switch (textScanLines()) {
    case TextScanLines::Lines200: points = 8; break;
    case TextScanLines::Lines350: points = 14; break;
    case TextScanLines::Lines400: points = 16; break;
    }
  } else {
    const int lines = modeScanLines(mode);
    points = lines == 200 ? 8 : lines == 350 ? 14 : 16;
  }

  // BIOS data area
  m_memory.write8(0x449, mode);
  m_memory.write16(0x44A, mi->textCols);
  m_memory.write16(0x44C, p->regenSize);
  m_memory.write16(0x44E, 0);
  for (int pg = 0; pg < 8; ++pg)
    m_memory.write16(0x450 + pg * 2, 0);
  m_memory.write16(0x460, text ? 0x0607 : 0x0000);
  m_memory.write8(0x462, 0);
  m_memory.write16(0x463, mode == 0x07 ? 0x3B4 : 0x3D4);
  static constexpr uint8_t kModeCtl[8] = {0x2C, 0x28, 0x2D, 0x29,
                                          0x2A, 0x2E, 0x1E, 0x29};
  m_memory.write8(0x465, mode < 8 ? kModeCtl[mode] : 0x29);
  m_memory.write8(0x466, mode == 0x06 ? 0x3F : 0x30);
  m_memory.write8(0x484, static_cast<uint8_t>(mi->textRows - 1));
  m_memory.write16(0x485, points);
  m_memory.write8(0x487, static_cast<uint8_t>((m_memory.read8(0x487) & 0x7F) |
                                              (clearScreen ? 0x00 : 0x80)));

  programVGARegisters(mode);
  if (!(m_memory.read8(0x489) & 0x08))
    loadDefaultDac(mode);

  if (text) {
    loadRomFontToPlane(points, 0);
    if (clearScreen) {
      const uint32_t base = mode == 0x07 ? 0xB0000 : 0xB8000;
      for (uint32_t i = 0; i < 0x8000; i += 2)
        m_memory.write16(base + i, 0x0720);
    }
  } else {
    if (clearScreen) {
      m_memory.clearVGA();
      if (mode <= 0x06)
        for (uint32_t i = 0; i < 0x4000; ++i)
          m_memory.write8(0xB8000 + i, 0);
    }
    // INT 43h: graphics character table matching the character height
    const uint16_t font = points == 8    ? ROM_FONT_8X8
                          : points == 14 ? ROM_FONT_8X14
                                         : ROM_FONT_8X16;
    m_memory.write16(0x43 * 4, font);
    m_memory.write16(0x43 * 4 + 2, VGA_ROM_SEG);
  }
  syncCursorFromBDA(m_memory);
}

// ── ROM fonts and character generator ────────────────────────────────

void BIOS::initializeVgaRom() {
  const uint32_t base = static_cast<uint32_t>(VGA_ROM_SEG) << 4;
  uint8_t rows[16];
  for (int ch = 0; ch < 256; ++ch) {
    glyph8x8(ch, rows);
    for (int r = 0; r < 8; ++r)
      m_memory.write8(base + ROM_FONT_8X8 + ch * 8 + r, rows[r]);
    glyph8x14(ch, rows);
    for (int r = 0; r < 14; ++r)
      m_memory.write8(base + ROM_FONT_8X14 + ch * 14 + r, rows[r]);
    for (int r = 0; r < 16; ++r)
      m_memory.write8(base + ROM_FONT_8X16 + ch * 16 + r, ui::kVGAFont8x16[ch][r]);
  }
  // Alternate 9-dot tables: empty (terminated by character code 0)
  m_memory.write16(base + ROM_FONT_ALT9X14, 0);
  m_memory.write16(base + ROM_FONT_ALT9X16, 0);
  // INT 1Fh: upper half of the 8x8 font; INT 43h: full 8x8 font
  m_memory.write16(0x1F * 4, ROM_FONT_8X8 + 128 * 8);
  m_memory.write16(0x1F * 4 + 2, VGA_ROM_SEG);
  m_memory.write16(0x43 * 4, ROM_FONT_8X8);
  m_memory.write16(0x43 * 4 + 2, VGA_ROM_SEG);
}

void BIOS::loadUserFontToPlane(uint32_t src, uint16_t count, uint16_t first,
                               uint8_t block, uint8_t points) {
  if (!m_vga || points == 0)
    return;
  points = std::min<uint8_t>(points, 32);
  const uint32_t blockBase = fontBlockOffset(block);
  for (uint32_t i = 0; i < count && first + i < 256; ++i) {
    const uint32_t dst = blockBase + (first + i) * 32u;
    for (uint32_t r = 0; r < 32; ++r)
      m_vga->writePlane(2, dst + r,
                        r < points ? m_memory.read8(src + i * points + r) : 0);
  }
}

void BIOS::loadRomFontToPlane(uint8_t points, uint8_t block) {
  const uint16_t off = points == 8    ? ROM_FONT_8X8
                       : points == 14 ? ROM_FONT_8X14
                                      : ROM_FONT_8X16;
  if (points != 8 && points != 14)
    points = 16;
  loadUserFontToPlane((static_cast<uint32_t>(VGA_ROM_SEG) << 4) + off, 256, 0,
                      block, points);
}

void BIOS::recalcTextRows(uint8_t points) {
  if (!m_vga || points == 0 || points > 32)
    return;
  const uint8_t mode = m_memory.read8(0x449);
  if (!isTextMode(mode))
    return;
  const uint32_t scanLines = m_vga->verticalDisplayEnd() + 1;
  const uint32_t rows = std::max<uint32_t>(1, scanLines / points);
  const uint32_t vde = rows * points - 1;
  m_vga->setCrtcReg(0x09, static_cast<uint8_t>((m_vga->crtcReg(0x09) & 0xE0) |
                                               (points - 1)));
  m_vga->setCrtcReg(0x12, static_cast<uint8_t>(vde));
  uint8_t ovf = static_cast<uint8_t>(m_vga->crtcReg(0x07) & ~0x42);
  ovf = static_cast<uint8_t>(ovf | ((vde >> 7) & 0x02) | ((vde >> 3) & 0x40));
  m_vga->setCrtcReg(0x07, ovf);
  const uint8_t curStart = static_cast<uint8_t>(points > 8 ? points - 3 : points - 2);
  m_vga->setCrtcReg(0x0A, curStart);
  m_vga->setCrtcReg(0x0B, static_cast<uint8_t>(curStart + 1));
  if (mode == 0x07)
    m_vga->setCrtcReg(0x14, static_cast<uint8_t>(points - 1));

  const uint16_t cols = m_memory.read16(0x44A);
  m_memory.write8(0x484, static_cast<uint8_t>(rows - 1));
  m_memory.write16(0x485, points);
  m_memory.write16(0x44C, static_cast<uint16_t>((rows * cols * 2 + 255) & ~255u));
}

void BIOS::handleFontService() {
  const uint8_t al = m_cpu.getReg8(cpu::AL);
  const uint8_t bl = m_cpu.getReg8(cpu::BL);
  const uint32_t esbp = (static_cast<uint32_t>(m_cpu.getSegReg(cpu::ES)) << 4) +
                        m_cpu.getReg16(cpu::BP);
  auto setInt43 = [&](uint16_t seg, uint16_t off, uint8_t points) {
    m_memory.write16(0x43 * 4, off);
    m_memory.write16(0x43 * 4 + 2, seg);
    m_memory.write16(0x485, points);
    uint8_t rows;
    switch (bl) {
    case 0x00: rows = m_cpu.getReg8(cpu::DL); break;
    case 0x01: rows = 14; break;
    case 0x03: rows = 43; break;
    default: rows = 25; break;
    }
    if (rows)
      m_memory.write8(0x484, static_cast<uint8_t>(rows - 1));
  };

  switch (al) {
  case 0x00:
  case 0x10: {
    const uint8_t points = m_cpu.getReg8(cpu::BH);
    loadUserFontToPlane(esbp, m_cpu.getReg16(cpu::CX), m_cpu.getReg16(cpu::DX),
                        bl, points);
    if (al == 0x10)
      recalcTextRows(points);
    break;
  }
  case 0x01: case 0x11:
  case 0x02: case 0x12:
  case 0x04: case 0x14: {
    const uint8_t points = (al & 0x0F) == 0x01 ? 14 : (al & 0x0F) == 0x02 ? 8 : 16;
    loadRomFontToPlane(points, bl);
    if (al & 0x10)
      recalcTextRows(points);
    break;
  }
  case 0x03: // Set block specifier (character map select)
    if (m_vga)
      m_vga->setSeqReg(3, bl);
    break;
  case 0x20: // INT 1Fh -> user 8x8 graphics characters 80h-FFh
    m_memory.write16(0x1F * 4, m_cpu.getReg16(cpu::BP));
    m_memory.write16(0x1F * 4 + 2, m_cpu.getSegReg(cpu::ES));
    break;
  case 0x21: // INT 43h -> user graphics font
    setInt43(m_cpu.getSegReg(cpu::ES), m_cpu.getReg16(cpu::BP),
             static_cast<uint8_t>(m_cpu.getReg16(cpu::CX)));
    break;
  case 0x22: setInt43(VGA_ROM_SEG, ROM_FONT_8X14, 14); break;
  case 0x23: setInt43(VGA_ROM_SEG, ROM_FONT_8X8, 8); break;
  case 0x24: setInt43(VGA_ROM_SEG, ROM_FONT_8X16, 16); break;
  case 0x30: { // Get font information
    uint16_t seg = VGA_ROM_SEG, off = ROM_FONT_8X16;
    switch (m_cpu.getReg8(cpu::BH)) {
    case 0x00:
      off = m_memory.read16(0x1F * 4);
      seg = m_memory.read16(0x1F * 4 + 2);
      break;
    case 0x01:
      off = m_memory.read16(0x43 * 4);
      seg = m_memory.read16(0x43 * 4 + 2);
      break;
    case 0x02: off = ROM_FONT_8X14; break;
    case 0x03: off = ROM_FONT_8X8; break;
    case 0x04: off = ROM_FONT_8X8 + 128 * 8; break;
    case 0x05: off = ROM_FONT_ALT9X14; break;
    case 0x06: off = ROM_FONT_8X16; break;
    case 0x07: off = ROM_FONT_ALT9X16; break;
    default: break;
    }
    m_cpu.setSegReg(cpu::ES, seg);
    m_cpu.setReg16(cpu::BP, off);
    m_cpu.setReg16(cpu::CX, currentCharHeight());
    m_cpu.setReg8(cpu::DL, m_memory.read8(0x484));
    break;
  }
  default:
    LOG_DEBUG("BIOS INT 10h AH=11h: Font subfn AL=", std::hex, (int)al,
              " (unsupported)");
    break;
  }
}

// ── Cursor, pages, CGA palette ───────────────────────────────────────

void BIOS::setCursorShape(uint8_t start, uint8_t end) {
  m_memory.write8(0x460, end);
  m_memory.write8(0x461, start);
  if (!m_vga || !isTextMode(m_memory.read8(0x449)))
    return;
  // Cursor emulation (BDA 487h bit 0 clear): CGA-style values for an
  // 8-line cell are scaled to the current character height.
  if (!(m_memory.read8(0x487) & 0x01) && (start & 0x60) == 0x20) {
    start = 0x1E; // CGA "cursor off" -> start below the cell
    end = 0x00;
  } else if (!(m_memory.read8(0x487) & 0x01) && !(start & 0xE0) && !(end & 0xE0)) {
    const uint8_t last = static_cast<uint8_t>(currentCharHeight() - 1);
    if (end < start) {
      if (end) {
        start = end;
        end = last;
      }
    } else if ((start | end) >= last || end != last - 1 || start != last) {
      if (end > 3) {
        if (start + 2 < end) {
          if (start > 2)
            start = static_cast<uint8_t>((last + 1) / 2);
          end = last;
        } else {
          start = static_cast<uint8_t>(start - end + last);
          end = last;
          if (last > 0x0C) {
            --start;
            --end;
          }
        }
      }
    }
  }
  m_vga->setCrtcReg(0x0A, static_cast<uint8_t>((m_vga->crtcReg(0x0A) & 0xC0) | (start & 0x3F)));
  m_vga->setCrtcReg(0x0B, static_cast<uint8_t>((m_vga->crtcReg(0x0B) & 0x80) | (end & 0x7F)));
}

void BIOS::setActivePage(uint8_t page) {
  if (page > 7)
    page = 0;
  const uint16_t offset = static_cast<uint16_t>(page * m_memory.read16(0x44C));
  m_memory.write8(0x462, page);
  m_memory.write16(0x44E, offset);
  if (!m_vga)
    return;
  // Word-mode modes (text, CGA) count the start address in words
  const uint32_t start = (m_vga->crtcReg(0x17) & 0x40) ? offset : offset / 2u;
  m_vga->setCrtcReg(0x0C, static_cast<uint8_t>(start >> 8));
  m_vga->setCrtcReg(0x0D, static_cast<uint8_t>(start));
}

void BIOS::setCgaPalette(uint8_t bh, uint8_t bl) {
  if (!m_vga)
    return;
  const uint8_t mode = m_memory.read8(0x449);
  const bool graphics = !isTextMode(mode);
  uint8_t cgaPal = m_memory.read8(0x466);
  if (bh == 0x00) {
    const uint8_t colour = static_cast<uint8_t>((bl & 0x07) | ((bl & 0x08) << 1));
    m_vga->setAttrReg(0x11, colour);
    if (graphics) {
      m_vga->setAttrReg(0x00, colour);
      if (mode == 0x04 || mode == 0x05)
        for (uint8_t i = 1; i < 4; ++i)
          m_vga->setAttrReg(i, static_cast<uint8_t>((m_vga->attrReg(i) & 0x07) |
                                                    (bl & 0x10)));
    }
    cgaPal = static_cast<uint8_t>((cgaPal & 0xE0) | (bl & 0x1F));
  } else if (bh == 0x01 && (mode == 0x04 || mode == 0x05)) {
    for (uint8_t i = 1; i < 4; ++i)
      m_vga->setAttrReg(i, static_cast<uint8_t>((m_vga->attrReg(i) & 0x10) |
                                                (i * 2 + (bl & 1))));
    cgaPal = static_cast<uint8_t>((cgaPal & 0xDF) | ((bl & 1) << 5));
  }
  m_memory.write8(0x466, cgaPal);
}

// ── INT 10h AH=10h: palette registers and DAC ────────────────────────

void BIOS::handlePaletteService() {
  const uint8_t al = m_cpu.getReg8(cpu::AL);
  const uint8_t bl = m_cpu.getReg8(cpu::BL);
  const uint8_t bh = m_cpu.getReg8(cpu::BH);
  uint8_t *pal = m_memory.dacPalette();
  const uint32_t esdx = (static_cast<uint32_t>(m_cpu.getSegReg(cpu::ES)) << 4) +
                        m_cpu.getReg16(cpu::DX);
  VGAController *vga = m_vga;

  switch (al) {
  case 0x00: // Set individual palette register
    if (vga && bl < 0x10)
      vga->setAttrReg(bl, bh);
    break;
  case 0x01: // Set overscan (border) colour
    if (vga)
      vga->setAttrReg(0x11, bh);
    break;
  case 0x02: // Set all palette registers and overscan
    if (vga) {
      for (uint8_t i = 0; i < 16; ++i)
        vga->setAttrReg(i, m_memory.read8(esdx + i));
      vga->setAttrReg(0x11, m_memory.read8(esdx + 16));
    }
    break;
  case 0x03: // Toggle intensity (BL=0) / blinking (BL=1)
    if (vga)
      vga->setAttrReg(0x10, static_cast<uint8_t>((vga->attrReg(0x10) & ~0x08) |
                                                 (bl ? 0x08 : 0x00)));
    m_memory.write8(0x465, static_cast<uint8_t>((m_memory.read8(0x465) & ~0x20) |
                                                (bl ? 0x20 : 0x00)));
    break;
  case 0x07: // Read individual palette register
    m_cpu.setReg8(cpu::BH, vga && bl < 0x10 ? vga->attrReg(bl) : 0);
    break;
  case 0x08: // Read overscan register
    m_cpu.setReg8(cpu::BH, vga ? vga->attrReg(0x11) : 0);
    break;
  case 0x09: // Read all palette registers and overscan
    for (uint8_t i = 0; i < 16; ++i)
      m_memory.write8(esdx + i, vga ? vga->attrReg(i) : i);
    m_memory.write8(esdx + 16, vga ? vga->attrReg(0x11) : 0);
    break;
  case 0x10: { // Set individual DAC register
    const uint16_t reg = m_cpu.getReg16(cpu::BX);
    if (reg < 256) {
      pal[reg * 3 + 0] = static_cast<uint8_t>(m_cpu.getReg8(cpu::DH) & 0x3F);
      pal[reg * 3 + 1] = static_cast<uint8_t>(m_cpu.getReg8(cpu::CH) & 0x3F);
      pal[reg * 3 + 2] = static_cast<uint8_t>(m_cpu.getReg8(cpu::CL) & 0x3F);
    }
    break;
  }
  case 0x12: { // Set block of DAC registers
    const uint16_t first = m_cpu.getReg16(cpu::BX);
    const uint16_t count = m_cpu.getReg16(cpu::CX);
    for (uint32_t i = 0; i < count && first + i < 256; ++i)
      for (uint32_t c = 0; c < 3; ++c)
        pal[(first + i) * 3 + c] =
            static_cast<uint8_t>(m_memory.read8(esdx + i * 3 + c) & 0x3F);
    break;
  }
  case 0x13: // Select colour paging mode / page
    if (vga) {
      const uint8_t mc = vga->attrReg(0x10);
      if (bl == 0x00) {
        vga->setAttrReg(0x10, static_cast<uint8_t>((mc & 0x7F) | ((bh & 1) << 7)));
      } else if (bl == 0x01) {
        const uint8_t cs = (mc & 0x80) ? static_cast<uint8_t>(bh & 0x0F)
                                       : static_cast<uint8_t>((bh & 0x03) << 2);
        vga->setAttrReg(0x14, cs);
      }
    }
    break;
  case 0x15: { // Read individual DAC register
    const uint16_t reg = m_cpu.getReg16(cpu::BX);
    if (reg < 256) {
      m_cpu.setReg8(cpu::DH, pal[reg * 3 + 0]);
      m_cpu.setReg8(cpu::CH, pal[reg * 3 + 1]);
      m_cpu.setReg8(cpu::CL, pal[reg * 3 + 2]);
    }
    break;
  }
  case 0x17: { // Read block of DAC registers
    const uint16_t first = m_cpu.getReg16(cpu::BX);
    const uint16_t count = m_cpu.getReg16(cpu::CX);
    for (uint32_t i = 0; i < count && first + i < 256; ++i)
      for (uint32_t c = 0; c < 3; ++c)
        m_memory.write8(esdx + i * 3 + c, pal[(first + i) * 3 + c]);
    break;
  }
  case 0x18: // Set PEL mask
    if (vga)
      vga->setPelMask(bl);
    break;
  case 0x19: // Read PEL mask
    m_cpu.setReg8(cpu::BL, vga ? vga->pelMask() : 0xFF);
    break;
  case 0x1A: { // Read colour page state
    const uint8_t mc = vga ? vga->attrReg(0x10) : 0;
    const uint8_t cs = vga ? vga->attrReg(0x14) : 0;
    m_cpu.setReg8(cpu::BL, (mc >> 7) & 1);
    m_cpu.setReg8(cpu::BH, (mc & 0x80) ? (cs & 0x0F) : ((cs >> 2) & 0x03));
    break;
  }
  case 0x1B: { // Sum DAC registers to grey shades
    const uint16_t first = m_cpu.getReg16(cpu::BX);
    const uint16_t count = m_cpu.getReg16(cpu::CX);
    for (uint32_t i = first; i < 256u && i < uint32_t(first) + count; ++i) {
      const uint32_t grey = (30u * pal[i * 3] + 59u * pal[i * 3 + 1] +
                             11u * pal[i * 3 + 2] + 50u) / 100u;
      pal[i * 3] = pal[i * 3 + 1] = pal[i * 3 + 2] =
          static_cast<uint8_t>(std::min<uint32_t>(grey, 63));
    }
    break;
  }
  default:
    LOG_DEBUG("BIOS INT 10h AH=10h: Palette subfn AL=", std::hex, (int)al,
              " (unsupported)");
    break;
  }
}

// ── INT 10h AH=12h: alternate select ─────────────────────────────────

void BIOS::handleAltSelectService() {
  const uint8_t bl = m_cpu.getReg8(cpu::BL);
  const uint8_t al = m_cpu.getReg8(cpu::AL);
  auto setFlag = [this](uint16_t addr, uint8_t bit, bool on) {
    uint8_t v = m_memory.read8(addr);
    m_memory.write8(addr, static_cast<uint8_t>(on ? (v | bit) : (v & ~bit)));
  };
  switch (bl) {
  case 0x10: // Get EGA information
    m_cpu.setReg8(cpu::BH, m_memory.read8(0x449) == 0x07 ? 0x01 : 0x00);
    m_cpu.setReg8(cpu::BL, 0x03); // 256K
    m_cpu.setReg8(cpu::CH, 0x00);
    m_cpu.setReg8(cpu::CL, m_memory.read8(0x488) & 0x0F);
    break;
  case 0x30: { // Select vertical resolution for text modes
    uint8_t v = static_cast<uint8_t>(m_memory.read8(0x489) & ~0x90);
    if (al == 0x00)
      v |= 0x80; // 200 lines
    else if (al == 0x02)
      v |= 0x10; // 400 lines
    else if (al != 0x01)
      break;
    m_memory.write8(0x489, v);
    m_cpu.setReg8(cpu::AL, 0x12);
    break;
  }
  case 0x31: // Default palette loading on mode set (0 = enable)
    setFlag(0x489, 0x08, al != 0);
    m_cpu.setReg8(cpu::AL, 0x12);
    break;
  case 0x32: // Video addressing enable/disable
    if (m_vga)
      m_vga->setMiscOutput(static_cast<uint8_t>((m_vga->miscOutput() & ~0x02) |
                                                (al ? 0x00 : 0x02)));
    m_cpu.setReg8(cpu::AL, 0x12);
    break;
  case 0x33: // Grey-scale summing
    setFlag(0x489, 0x02, al == 0);
    m_cpu.setReg8(cpu::AL, 0x12);
    break;
  case 0x34: // Cursor emulation (0 = enable)
    setFlag(0x487, 0x01, al != 0);
    m_cpu.setReg8(cpu::AL, 0x12);
    break;
  case 0x36: // Video refresh control (screen on/off)
    if (m_vga)
      m_vga->setSeqReg(1, static_cast<uint8_t>((m_vga->seqReg(1) & ~0x20) |
                                               (al ? 0x20 : 0x00)));
    m_cpu.setReg8(cpu::AL, 0x12);
    break;
  default:
    LOG_DEBUG("BIOS INT 10h AH=12h: Alt Select BL=", std::hex, (int)bl,
              " (unsupported)");
    m_cpu.setReg8(cpu::AL, 0x12);
    break;
  }
}

} // namespace fador::hw
