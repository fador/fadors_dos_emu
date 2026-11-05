#pragma once
#include <cstdint>

namespace fador::ui {

// Standard 8x16 VGA ROM font (CP437).  Each glyph is 16 rows of 8 bits,
// MSB = leftmost pixel.  Shared by every frontend that rasterises text mode.
extern const uint8_t kVGAFont8x16[256][16];

} // namespace fador::ui
