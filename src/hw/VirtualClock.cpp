#include "VirtualClock.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <thread>

namespace fador::hw {

// ────────────────────────────────────────────────────────────
//  machine presets
// ────────────────────────────────────────────────────────────

namespace {

// CPI values are rough averages for typical DOS code: an 8088 needs ~15
// clocks per instruction (with its 8-bit bus), a 286 ~10 including wait
// states, a 386 ~4.5, a 486 ~2 and a Pentium ~1.2.
constexpr std::array<MachineSpeed, 13> kPresets{{
    {"8088", 4.77, 14.5},
    {"286-12", 12.0, 10.0},
    {"386sx-16", 16.0, 5.0},
    {"386dx-25", 25.0, 4.5},
    {"386dx-33", 33.0, 4.5},
    {"486sx-25", 25.0, 2.0},
    {"486dx-33", 33.0, 2.0},
    {"486dx2-66", 66.0, 2.0},
    {"486dx4-100", 100.0, 2.0},
    {"pentium-60", 60.0, 1.2},
    {"pentium-90", 90.0, 1.2},
    {"pentium-100", 100.0, 1.2},
    {"pentium-133", 133.0, 1.2},
}};

struct Alias {
  std::string_view alias;
  std::string_view preset;
};
constexpr std::array<Alias, 11> kAliases{{
    {"xt", "8088"},
    {"pc-xt", "8088"},
    {"286", "286-12"},
    {"at", "286-12"},
    {"at-12", "286-12"},
    {"386", "386dx-33"},
    {"386dx", "386dx-33"},
    {"486", "486dx2-66"},
    {"486dx2", "486dx2-66"},
    {"pentium", "pentium-90"},
    {"p90", "pentium-90"},
}};

} // namespace

std::optional<MachineSpeed> findMachinePreset(std::string_view name) {
  for (const auto &a : kAliases)
    if (a.alias == name)
      name = a.preset;
  for (const auto &p : kPresets)
    if (p.name == name)
      return p;
  return std::nullopt;
}

std::string machinePresetList() {
  std::string list;
  for (const auto &p : kPresets) {
    if (!list.empty())
      list += ", ";
    list += p.name;
  }
  return list;
}

MachineSpeed defaultDeterministicMachine() {
  return *findMachinePreset("486dx2-66");
}

// ────────────────────────────────────────────────────────────
//  clock
// ────────────────────────────────────────────────────────────

VirtualClock::VirtualClock() {
  m_hostNow = [] {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
  };
  m_hostSleep = [](uint64_t ns) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(ns));
  };
  recomputeNominalRate();
  setPacing(Pacing::Auto);
}

const char *VirtualClock::pacingName(Pacing pacing) {
  switch (pacing) {
  case Pacing::Auto:
    return "auto";
  case Pacing::Fixed:
    return "fixed";
  case Pacing::Deterministic:
    return "deterministic";
  }
  return "?";
}

uint64_t VirtualClock::toFx(double ns) {
  if (ns <= 0.0)
    return 0;
  return static_cast<uint64_t>(ns * 4294967296.0 + 0.5);
}

void VirtualClock::recomputeNominalRate() {
  const double nsPerCycle = 1e9 / m_cpuHz;
  m_nsPerCycleFx = toFx(nsPerCycle);
  m_nominalFx = toFx(nsPerCycle * m_cpi);
  if (m_pacing != Pacing::Auto)
    m_nsPerUnitFx = m_nominalFx;
}

void VirtualClock::setPacing(Pacing pacing) {
  rebase();
  m_pacing = pacing;
  if (pacing == Pacing::Auto) {
    // Until the first measurement: assume the nominal machine speed.
    if (m_rateFx == 0)
      m_rateFx = m_nominalFx;
    m_nsPerUnitFx = m_rateFx;
    m_interpCapNs = kMaxInterpolationNs;
  } else {
    m_nsPerUnitFx = m_nominalFx;
    m_interpCapNs = std::numeric_limits<uint64_t>::max();
  }
}

void VirtualClock::setCpuHz(double hz) {
  rebase();
  m_cpuHz = hz > 1.0 ? hz : 1.0;
  recomputeNominalRate();
}

void VirtualClock::setCyclesPerInstruction(double cpi) {
  rebase();
  m_cpi = cpi > 0.0 ? cpi : 1.0;
  recomputeNominalRate();
}

void VirtualClock::setMachine(const MachineSpeed &machine) {
  rebase();
  m_cpuHz = machine.cyclesPerSecond() > 1.0 ? machine.cyclesPerSecond() : 1.0;
  m_cpi = machine.cpi > 0.0 ? machine.cpi : 1.0;
  recomputeNominalRate();
}

void VirtualClock::attachInstructionCounter(const uint64_t *counter) {
  rebase();
  m_counter = counter;
  m_baseCount = counter ? *counter : 0;
  m_lastSyncCount = m_baseCount;
  m_lastIdleCount = m_baseCount;
}

void VirtualClock::setHostClock(HostNowFn now, HostSleepFn sleep) {
  m_hostNow = std::move(now);
  m_hostSleep = std::move(sleep);
}

uint64_t VirtualClock::hostNow() const { return m_hostNow(); }

// Fold the work done since the last rebase into m_baseNs, keeping the
// sub-nanosecond remainder so long runs do not drift.
void VirtualClock::rebase() {
  if (!m_counter)
    return;
  const uint64_t count = *m_counter;
  const uint64_t units = count - m_baseCount;
  m_baseCount = count;
  if (units == 0)
    return;
  // units * fx as a 96-bit number: whole ns and a 32-bit fraction (the
  // low 32 bits of the wrapped 64-bit product are exact).
  const uint64_t lo = units * (m_nsPerUnitFx & 0xFFFFFFFFu);
  const uint64_t whole = fxMulInt(units, m_nsPerUnitFx);
  const uint64_t frac = (lo & 0xFFFFFFFFu) + m_baseFrac;
  uint64_t d = whole + (frac >> 32);
  if (d > m_interpCapNs) {
    d = m_interpCapNs;
    m_baseFrac = 0;
  } else {
    m_baseFrac = static_cast<uint32_t>(frac);
  }
  m_baseNs += d;
}

void VirtualClock::advanceCycles(uint64_t cycles) {
  rebase();
  // Low 32 bits of cycles * fx (wrap-around keeps them exact).
  const uint64_t lo = cycles * (m_nsPerCycleFx & 0xFFFFFFFFu);
  const uint64_t frac = (lo & 0xFFFFFFFFu) + m_baseFrac;
  m_baseNs += fxMulInt(cycles, m_nsPerCycleFx) + (frac >> 32);
  m_baseFrac = static_cast<uint32_t>(frac);
}

void VirtualClock::advanceNs(uint64_t ns) {
  rebase();
  m_baseNs += ns;
}

void VirtualClock::start() {
  rebase();
  m_started = true;
  m_virtualOrigin = static_cast<int64_t>(m_baseNs);
  m_droppedNs = 0;
  m_skewNs = 0;
  m_lastSyncCount = m_counter ? *m_counter : 0;
  if (m_pacing == Pacing::Deterministic) {
    m_hostOrigin = 0;
    m_lastSyncHost = 0;
    return;
  }
  m_hostOrigin = hostNow();
  m_lastSyncHost = 0;
}

int64_t VirtualClock::wallTargetNs(uint64_t hostElapsed) const {
  return m_virtualOrigin + static_cast<int64_t>(hostElapsed) -
         static_cast<int64_t>(m_droppedNs) + m_skewNs;
}

void VirtualClock::sync() {
  rebase();
  if (m_pacing == Pacing::Deterministic || !m_started)
    return;

  const uint64_t host = hostNow() - m_hostOrigin;
  const int64_t wall = wallTargetNs(host);
  const int64_t now = static_cast<int64_t>(m_baseNs);

  if (m_pacing == Pacing::Fixed) {
    if (now > wall + static_cast<int64_t>(kSleepSlackNs)) {
      m_hostSleep(static_cast<uint64_t>(now - wall));
    } else if (wall - now > static_cast<int64_t>(kMaxCatchUpNs)) {
      m_droppedNs += static_cast<uint64_t>(wall - now) - kMaxCatchUpNs;
    }
    return;
  }

  // Auto: measure how fast the host retires guest work, then pull virtual
  // time to the wall clock.  Intervals that contained idle waits say
  // nothing about the instruction rate.
  const uint64_t count = m_counter ? *m_counter : 0;
  const uint64_t units = count - m_lastSyncCount;
  uint64_t dh = host - m_lastSyncHost;
  if (!m_idleSinceSync && units >= 256 && dh > 0) {
    dh = std::min<uint64_t>(dh, 1ULL << 30);
    uint64_t measured = (dh << 32) / units;
    // A host stall is not a slow instruction mix: let the estimate rise
    // by at most 1.75x per interval.
    measured = std::min(measured, 4 * m_rateFx);
    measured = std::clamp<uint64_t>(measured, toFx(0.05), toFx(1000.0));
    m_rateFx = (3 * m_rateFx + measured) / 4;
  }
  m_lastSyncCount = count;
  m_lastSyncHost = host;
  m_idleSinceSync = false;

  if (wall > now) {
    uint64_t d = static_cast<uint64_t>(wall - now);
    if (d > kMaxJumpNs) {
      m_droppedNs += d - kMaxJumpNs;
      d = kMaxJumpNs;
    }
    m_baseNs += d;
    m_nsPerUnitFx = m_rateFx;
  } else {
    // Ahead of the wall clock: slow the interpolation down, stop it at
    // kMaxLeadNs.  Time never runs backwards.
    const uint64_t lead = static_cast<uint64_t>(now - wall);
    m_nsPerUnitFx =
        lead >= kMaxLeadNs
            ? 0
            : fxMulInt(m_rateFx, ((kMaxLeadNs - lead) << 32) / kMaxLeadNs);
  }
}

void VirtualClock::idle() {
  const uint64_t count = m_counter ? *m_counter : 0;
  const bool waiting = count - m_lastIdleCount < kIdleWorkThreshold;
  m_lastIdleCount = count;
  switch (m_pacing) {
  case Pacing::Auto:
    m_idleSinceSync = true;
    break;
  case Pacing::Fixed:
    if (waiting && m_started) {
      rebase();
      const int64_t wall = wallTargetNs(hostNow() - m_hostOrigin);
      if (wall > static_cast<int64_t>(m_baseNs))
        m_baseNs = static_cast<uint64_t>(wall);
    }
    break;
  case Pacing::Deterministic:
    if (waiting)
      advanceNs(kIdleQuantumNs);
    break;
  }
}

void VirtualClock::adjustSkew(int64_t ns) {
  m_skewNs = std::clamp<int64_t>(m_skewNs + ns, -kMaxSkewNs, kMaxSkewNs);
}

uint64_t VirtualClock::hostElapsedNs() const {
  if (!m_started || m_pacing == Pacing::Deterministic)
    return 0;
  return hostNow() - m_hostOrigin;
}

uint64_t VirtualClock::droppedNs() const { return m_droppedNs; }

double VirtualClock::estimatedNsPerUnit() const {
  return static_cast<double>(m_rateFx) / 4294967296.0;
}

} // namespace fador::hw
