#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

namespace fador::hw {

/// Speed of an emulated machine: core clock, an average
/// cycles-per-instruction (CPI) estimate and the CPU model (--cpu) whose
/// per-opcode cycle table drives the guest clock.
///
/// The emulator models 386 and later CPUs.  Presets for older or slower
/// parts (8088, 286, 386SX) run on the nearest model's cycle table and
/// correct for the difference with cycleScale: the real part needs
/// cycleScale times as many clocks as the model for the same instruction
/// mix (the ratio of the presets' CPI estimates, e.g. an 8088 at 14.5
/// cycles/instruction against a 386 at 4.5 gives 3.2), so the model's
/// cycles are counted at mhz / cycleScale.
struct MachineSpeed {
  std::string_view name;
  double mhz = 0.0;
  double cpi = 1.0;
  std::string_view cpu = {}; // --cpu model name (empty: none, e.g. --throttle-ips)
  double cycleScale = 1.0;

  double cyclesPerSecond() const { return mhz * 1e6; }
  /// Rate of the model's guest cycles (CPU::cycles()) per emulated second.
  double modelCyclesPerSecond() const { return mhz * 1e6 / cycleScale; }
  uint64_t instructionsPerSecond() const {
    return static_cast<uint64_t>(mhz * 1e6 / cpi + 0.5);
  }
};

/// Named machine presets for --throttle-machine (and the default speed of
/// --deterministic).  Accepts the aliases xt, 286, 386, 486, pentium, ...
std::optional<MachineSpeed> findMachinePreset(std::string_view name);
/// Comma-separated list of the preset names (for help and error messages).
std::string machinePresetList();
/// Preset used by --deterministic when no speed is given.
MachineSpeed defaultDeterministicMachine();

/// The emulator's single time base.
///
/// Virtual (emulated) time advances with retired guest work, never by
/// itself: every device that needs time (PIT, RTC, Sound Blaster, PC
/// speaker, VGA retrace, refresh toggle on port 61h) reads nowNs().
///
/// Work is counted in guest cycles.  The emulator attaches the CPU model's
/// guest clock (CPU::cycles(), with setCyclesPerInstruction(1) and the CPU
/// clock as cpuHz; see MachineTime); --throttle-ips attaches the retired-
/// instruction counter instead.  Work can also be pushed with
/// advanceCycles(), and waits for a device event use waitUntil().
/// Between two sync() calls the clock interpolates from the counter, so a
/// timer read in the middle of a batch sees time move with each instruction
/// at no per-instruction cost.
///
/// How virtual time relates to the host clock is the pacing mode:
///  - Auto (default, like DOSBox "cycles=max"): the guest runs at full host
///    speed while the instruction rate is estimated continuously so that
///    virtual time tracks the host wall clock.  After a host stall the
///    clock moves forward by at most kMaxJumpNs; the rest is dropped.
///  - Fixed: virtual time = cycles / cpuHz.  sync() sleeps whenever virtual
///    time gets ahead of the wall clock (the --throttle-* options).
///  - Deterministic: virtual time is a pure function of the guest work
///    (and of idle calls); the host clock is never read, so headless runs
///    are reproducible.
class VirtualClock {
public:
  enum class Pacing { Auto, Fixed, Deterministic };
  using HostNowFn = std::function<uint64_t()>;     // monotonic host ns
  using HostSleepFn = std::function<void(uint64_t)>; // sleep for ns

  static constexpr uint64_t kNsPerSecond = 1'000'000'000ULL;
  /// Auto: largest forward step after the host stalls (debugger, swapping).
  static constexpr uint64_t kMaxJumpNs = 20'000'000;
  /// Auto: interpolation between two sync() calls never exceeds this.
  static constexpr uint64_t kMaxInterpolationNs = 5'000'000;
  /// Auto: at this lead over the wall clock virtual time stops advancing.
  static constexpr uint64_t kMaxLeadNs = 1'000'000;
  /// Fixed: sleep only once this far ahead (sleeps are coarse).
  static constexpr uint64_t kSleepSlackNs = 1'000'000;
  /// Fixed: a slow host never makes the guest run in bursts to catch up
  /// more than this much.
  static constexpr uint64_t kMaxCatchUpNs = 100'000'000;
  /// Deterministic: virtual time that passes per idle() of a waiting guest.
  static constexpr uint64_t kIdleQuantumNs = 1'000'000;
  /// idle() calls fewer than this many work units apart count as a guest
  /// that is only waiting (blocking BIOS/DOS input, tight polling).  The
  /// default suits an instruction counter; setIdleWorkThreshold() sets it
  /// for a cycle counter.
  static constexpr uint64_t kIdleWorkThreshold = 64;
  /// Auto: waitUntil() sleeps the host at most this long per call, so host
  /// input and the caller's checks keep a low latency.
  static constexpr uint64_t kMaxIdleSleepNs = 1'000'000;

  VirtualClock();

  // ── configuration ────────────────────────────────────────────────
  void setPacing(Pacing pacing);
  Pacing pacing() const { return m_pacing; }
  static const char *pacingName(Pacing pacing);

  /// Guest cycles per second of virtual time (Fixed / Deterministic).
  void setCpuHz(double hz);
  double cpuHz() const { return m_cpuHz; }
  /// Cycles charged per unit of the attached counter (an estimate until
  /// the CPU model reports real per-instruction cycles).
  void setCyclesPerInstruction(double cpi);
  double cyclesPerInstruction() const { return m_cpi; }
  /// Convenience: setCpuHz(mhz) + setCyclesPerInstruction(cpi).
  void setMachine(const MachineSpeed &machine);
  /// Instructions per second of virtual time (cpuHz / cpi).
  double instructionsPerSecond() const { return m_cpuHz / m_cpi; }
  /// Work units between idle() / noteIdle() calls below which the guest
  /// counts as only waiting.
  void setIdleWorkThreshold(uint64_t units) { m_idleWorkThreshold = units; }
  uint64_t idleWorkThreshold() const { return m_idleWorkThreshold; }

  /// Counter of retired guest work (CPU::cyclesRef() or the instruction
  /// counter CPU::getCyclesRef()).  Must stay valid and only grow while
  /// attached.
  void attachInstructionCounter(const uint64_t *counter);
  /// Replace the host clock (unit tests).  Defaults to steady_clock and
  /// std::this_thread::sleep_for.
  void setHostClock(HostNowFn now, HostSleepFn sleep);

  // ── guest work ───────────────────────────────────────────────────
  /// Explicitly retired guest cycles (in addition to the counter).
  void advanceCycles(uint64_t cycles);
  /// Let virtual time pass without guest work.
  void advanceNs(uint64_t ns);

  /// Current virtual time in nanoseconds.  Cheap: one load, a 64x64
  /// multiply and a compare.
  uint64_t nowNs() const {
    const uint64_t units = m_counter ? *m_counter - m_baseCount : 0;
    uint64_t d = fxMulInt(units, m_nsPerUnitFx);
    if (d > m_interpCapNs)
      d = m_interpCapNs;
    return m_baseNs + d;
  }
  double nowSeconds() const {
    return static_cast<double>(nowNs()) / static_cast<double>(kNsPerSecond);
  }
  /// A time source for devices: setTimeSource(clock.timeSource()).
  std::function<uint64_t()> timeSource() const {
    return [this] { return nowNs(); };
  }
  /// Whole periods of a hz clock in ns of virtual time (ns * hz / 1e9,
  /// exact for hz < 2^34).
  static uint64_t ticksAt(uint64_t ns, uint64_t hz) {
    return ns / kNsPerSecond * hz + (ns % kNsPerSecond) * hz / kNsPerSecond;
  }

  // ── host pacing ──────────────────────────────────────────────────
  /// Anchor virtual time to the host clock; call when guest execution
  /// starts.
  void start();
  /// Between batches: fold the counter into the base and (Auto / Fixed)
  /// compare with the host clock, adjust the rate or sleep.
  void sync();
  /// The guest is waiting (blocking input, DOS idle).  A guest that only
  /// waits lets time pass: Deterministic adds kIdleQuantumNs, Fixed
  /// catches up with the wall clock, Auto leaves the rate estimate alone.
  void idle();
  /// Records a BIOS/DOS idle call; true when the guest retired fewer than
  /// idleWorkThreshold() units since the previous one (it only waits).
  bool noteIdle();
  /// The guest waits (HLT, blocking input) for a device event due at
  /// virtual time deadlineNs; the caller caps it to keep latency low.
  ///  - Deterministic: virtual time jumps to deadlineNs.
  ///  - Fixed: virtual time jumps to deadlineNs and the host sleeps until
  ///    the wall clock has caught up.
  ///  - Auto: the host sleeps until the wall clock reaches deadlineNs (at
  ///    most kMaxIdleSleepNs), then virtual time follows the wall clock.
  ///    The interval does not count towards the instruction-rate estimate.
  void waitUntil(uint64_t deadlineNs);
  /// Auto: shift the wall-clock target (audio queue feedback).  The total
  /// skew stays within +-kMaxSkewNs.
  void adjustSkew(int64_t ns);
  static constexpr int64_t kMaxSkewNs = 50'000'000;

  /// Host time since start() (0 in Deterministic mode).
  uint64_t hostElapsedNs() const;
  /// Virtual time dropped so far because the host could not keep up.
  uint64_t droppedNs() const;
  /// Auto: current estimate of host ns per work unit.
  double estimatedNsPerUnit() const;

private:
  /// (a * fx) >> 32 for a 32.32 fixed-point fx, without overflow.
  static uint64_t fxMulInt(uint64_t a, uint64_t fx) {
    const uint64_t aLo = a & 0xFFFFFFFFu, aHi = a >> 32;
    const uint64_t fLo = fx & 0xFFFFFFFFu, fHi = fx >> 32;
    // a*fx = aHi*fHi<<64 + (aHi*fLo + aLo*fHi)<<32 + aLo*fLo; the first
    // term only matters for results >= 2^64 ns, which never occur.
    return ((aHi * fHi) << 32) + aHi * fLo + aLo * fHi + ((aLo * fLo) >> 32);
  }
  static uint64_t toFx(double ns);
  void rebase();
  void recomputeNominalRate();
  uint64_t hostNow() const;
  int64_t wallTargetNs(uint64_t hostElapsed) const;

  Pacing m_pacing = Pacing::Auto;
  double m_cpuHz = 66e6;
  double m_cpi = 2.0;

  const uint64_t *m_counter = nullptr;
  uint64_t m_baseCount = 0;   // counter value folded into m_baseNs
  uint64_t m_baseNs = 0;      // virtual time at m_baseCount
  uint32_t m_baseFrac = 0;    // sub-ns remainder (32-bit fraction)
  uint64_t m_nsPerUnitFx = 0; // interpolation rate, 32.32 ns per unit
  uint64_t m_nsPerCycleFx = 0;
  uint64_t m_nominalFx = 0; // Fixed/Deterministic ns per unit
  uint64_t m_interpCapNs = std::numeric_limits<uint64_t>::max();

  HostNowFn m_hostNow;
  HostSleepFn m_hostSleep;
  bool m_started = false;
  uint64_t m_hostOrigin = 0;
  int64_t m_virtualOrigin = 0; // virtual time at start()
  uint64_t m_droppedNs = 0;    // virtual time given up after host stalls
  int64_t m_skewNs = 0;        // Auto: audio feedback
  uint64_t m_lastSyncHost = 0;
  uint64_t m_lastSyncCount = 0;
  uint64_t m_rateFx = 0;       // Auto: measured host ns per unit
  bool m_idleSinceSync = false;
  uint64_t m_lastIdleCount = 0;
  uint64_t m_idleWorkThreshold = kIdleWorkThreshold;
};

} // namespace fador::hw
