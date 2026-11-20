#pragma once
// Glue between the CPU model and the emulated time base: which CPU model
// and clock rate the command line selects (--cpu, --throttle-machine,
// --throttle-ips, --deterministic), the time-stamp counter, and what
// happens while the guest waits (HLT, blocking BIOS/DOS calls).
#include "../cpu/CpuModel.hpp"
#include "VirtualClock.hpp"

#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace fador::cpu {
class CPU;
}

namespace fador::hw {

/// Speed-related command-line options.
struct SpeedOptions {
  std::optional<cpu::CpuModelId> cpu;  // --cpu (given explicitly)
  std::optional<MachineSpeed> preset;  // --throttle-machine
  uint64_t throttleIps = 0;            // --throttle-ips (0 = not given)
  bool deterministic = false;          // --deterministic
};

/// The resolved machine speed.
///
/// Emulated time is driven by a work counter: normally the guest clock
/// (CPU::cycles(), the model's per-opcode costs), so in fixed and
/// deterministic pacing emulated time = cycles / workHz.  --throttle-ips
/// counts retired instructions instead (workHz instructions per second).
/// In auto pacing the guest runs as fast as the host allows and emulated
/// time follows the wall clock; workHz only seeds the rate estimate.
struct SpeedPlan {
  cpu::CpuModelId model = cpu::kDefaultCpuModel;
  VirtualClock::Pacing pacing = VirtualClock::Pacing::Auto;
  double workHz = 0.0;
  bool countInstructions = false;
  /// Rate of the time-stamp counter: the emulated core clock.
  uint64_t tscHz = 0;
  std::string label;                 // for the start-up log line
  std::vector<std::string> warnings; // e.g. --cpu vs preset mismatches
};

/// CPU model a --throttle-machine preset selects when --cpu is not given.
std::optional<cpu::CpuModelId> presetCpuModel(const MachineSpeed &preset);

/// Resolves the options:
///  - --throttle-machine=P selects P's CPU model (cycle table, CPUID) and
///    clock (corrected by P's cycleScale for pre-386 parts); with an
///    explicit, different --cpu the preset only sets the clock (MHz) and a
///    warning is added.
///  - --throttle-ips=N paces by instructions at N per second.
///  - otherwise the model runs at its nominal clock (cpu.model().clockHz).
/// Pacing: deterministic if requested, else fixed with a throttle option,
/// else auto.
SpeedPlan planSpeed(const SpeedOptions &options);

/// Nominal guest cycles charged for one call of a high-level-emulated BIOS
/// or DOS service, standing in for the real ROM/DOS code it replaces, so
/// programs that time these calls see plausible (non-zero) durations.
/// Rough 486 figures; they are the same on every model, so a faster clock
/// makes the services faster.  Wait states (disk, keyboard) are not
/// included: blocking calls idle via MachineTime::guestIdle().
namespace hle_cost {
inline constexpr uint32_t kVideo = 300;    // INT 10h
inline constexpr uint32_t kDisk = 1000;    // INT 13h
inline constexpr uint32_t kKeyboard = 100; // INT 16h
inline constexpr uint32_t kDos = 500;      // INT 21h
} // namespace hle_cost

/// Runtime part: applies a SpeedPlan to the clock and the CPU, provides the
/// time-stamp counter and implements HLT and the BIOS/DOS idle callbacks.
class MachineTime {
public:
  static constexpr uint64_t kNever = std::numeric_limits<uint64_t>::max();
  /// Longest single wait step: host input, scripted keys and the run's
  /// time limit are checked at least this often (virtual or host time).
  static constexpr uint64_t kWaitSliceNs = 1'000'000;
  /// Idle calls fewer than this many guest cycles apart mean the guest is
  /// only waiting (a blocking INT 16h retry costs a few hundred cycles).
  static constexpr uint64_t kIdleWorkThresholdCycles = 2048;

  MachineTime(VirtualClock &clock, cpu::CPU &cpu);

  /// Attaches the work counter, sets the rate and pacing and installs the
  /// CPU's time-stamp counter source and HLT callback.
  void configure(const SpeedPlan &plan);
  const SpeedPlan &plan() const { return m_plan; }

  /// A device that raises interrupts by itself (PIT, RTC, Sound Blaster,
  /// UART, ...): returns the virtual time of its next event or kNever.
  void addWakeSource(std::function<uint64_t()> nextEventNs);
  /// Earliest event of all wake sources.
  uint64_t nextWakeNs() const;

  struct Hooks {
    /// Bring the devices up to the current virtual time and raise the
    /// IRQs that are due (the main loop's between-batch service).
    std::function<void()> serviceDevices;
    /// Deliver one pending, unmasked hardware interrupt if IF allows it;
    /// true if one was delivered.
    std::function<bool()> deliverInterrupt;
    /// Host input, scripted keys, rendering, time limit; false when the
    /// emulator is about to stop.
    std::function<bool()> servicePeriodic;
  };
  void setHooks(Hooks hooks) { m_hooks = std::move(hooks); }

  /// HLT (installed as the CPU's halt callback).  IF=1: waits until an
  /// interrupt is delivered; emulated time moves to the next device event
  /// (deterministic / fixed) or the host sleeps (auto).  IF=0: logs once
  /// and continues (a real CPU would wait for NMI or reset forever).
  void halt();
  /// BIOS/DOS idle callback: when the guest only waits (see
  /// kIdleWorkThresholdCycles), waits one slice for the next device event.
  void guestIdle();
  /// Waits for the next device event, at most maxNs of virtual time.
  void waitForEvent(uint64_t maxNs = kWaitSliceNs);
  /// A BIOS delay (INT 15h AH=86h): lets ns of emulated time pass, keeping
  /// the devices and host services running; interrupts that become due are
  /// raised but delivered after the call.
  void delay(uint64_t ns);

  /// Time-stamp counter: core clock periods of emulated time,
  /// emulatedNs * tscHz / 1e9.  Equals CPU::cycles() in fixed and
  /// deterministic pacing as long as the guest did not wait.
  uint64_t timeStampCounter() const;

  uint64_t halts() const { return m_halts; }
  uint64_t haltedNs() const { return m_haltedNs; }
  uint64_t haltsWithInterruptsDisabled() const { return m_haltsIfClear; }

private:
  VirtualClock &m_clock;
  cpu::CPU &m_cpu;
  SpeedPlan m_plan;
  Hooks m_hooks;
  std::vector<std::function<uint64_t()>> m_wakeSources;
  uint64_t m_halts = 0;
  uint64_t m_haltedNs = 0;
  uint64_t m_haltsIfClear = 0;
};

} // namespace fador::hw
