#pragma once

#include <string>

namespace fador::ui {

// In-process benchmarks of the host-SIMD kernels (--bench=render-13h,
// render-modex, render-planar, render-vbe, audio-mix, mem-fill).  Each one
// runs at every SIMD level the host supports and logs the time per frame
// (or per block) and the throughput.  Returns false for unknown names.
bool isHostSimdBenchmark(const std::string &name);
bool runHostSimdBenchmark(const std::string &name);

// Comma-separated list of the names above, for help texts.
const char *hostSimdBenchmarkList();

} // namespace fador::ui
