#include "MachineTime.hpp"

#include "../cpu/CPU.hpp"
#include "../utils/Logger.hpp"

#include <algorithm>
#include <sstream>

namespace fador::hw {

std::optional<cpu::CpuModelId> presetCpuModel(const MachineSpeed &preset) {
  if (preset.cpu.empty())
    return std::nullopt;
  return cpu::parseCpuModel(preset.cpu);
}

SpeedPlan planSpeed(const SpeedOptions &options) {
  using Pacing = VirtualClock::Pacing;
  SpeedPlan plan;
  const auto presetModel =
      options.preset ? presetCpuModel(*options.preset) : std::nullopt;
  plan.model = options.cpu.value_or(presetModel.value_or(cpu::kDefaultCpuModel));
  const cpu::CpuModel &model = cpu::cpuModel(plan.model);
  const bool throttled = options.throttleIps > 0 || options.preset.has_value();
  plan.pacing = options.deterministic ? Pacing::Deterministic
                : throttled           ? Pacing::Fixed
                                      : Pacing::Auto;
  plan.tscHz = model.clockHz;

  if (options.throttleIps > 0) {
    plan.countInstructions = true;
    plan.workHz = static_cast<double>(options.throttleIps);
    plan.label = std::to_string(options.throttleIps) + " instructions/s";
    return plan;
  }
  if (options.preset) {
    const MachineSpeed &p = *options.preset;
    plan.label = std::string(p.name);
    if (options.cpu && presetModel && *options.cpu != *presetModel) {
      std::ostringstream w;
      w << "--cpu=" << model.cliName << " with --throttle-machine=" << p.name
        << " (a " << cpu::cpuModel(*presetModel).cliName
        << "): the preset only sets the clock to " << p.mhz << " MHz";
      plan.warnings.push_back(w.str());
      plan.workHz = p.cyclesPerSecond();
    } else {
      plan.workHz = p.modelCyclesPerSecond();
    }
    plan.tscHz = static_cast<uint64_t>(plan.workHz + 0.5);
    return plan;
  }
  plan.workHz = static_cast<double>(model.clockHz);
  plan.label = "nominal clock";
  return plan;
}

MachineTime::MachineTime(VirtualClock &clock, cpu::CPU &cpu)
    : m_clock(clock), m_cpu(cpu) {}

void MachineTime::configure(const SpeedPlan &plan) {
  m_plan = plan;
  if (plan.countInstructions) {
    m_clock.attachInstructionCounter(&m_cpu.getCyclesRef());
    m_clock.setIdleWorkThreshold(VirtualClock::kIdleWorkThreshold);
  } else {
    m_clock.attachInstructionCounter(&m_cpu.cyclesRef());
    m_clock.setIdleWorkThreshold(kIdleWorkThresholdCycles);
  }
  m_clock.setCyclesPerInstruction(1.0);
  m_clock.setCpuHz(plan.workHz);
  m_clock.setPacing(plan.pacing);
  m_cpu.setTimeStampSource([this] { return timeStampCounter(); });
  m_cpu.setHaltCallback([this] { halt(); });
}

void MachineTime::addWakeSource(std::function<uint64_t()> nextEventNs) {
  m_wakeSources.push_back(std::move(nextEventNs));
}

uint64_t MachineTime::nextWakeNs() const {
  uint64_t next = kNever;
  for (const auto &source : m_wakeSources)
    next = std::min(next, source());
  return next;
}

uint64_t MachineTime::timeStampCounter() const {
  return VirtualClock::ticksAt(m_clock.nowNs(), m_plan.tscHz);
}

void MachineTime::waitForEvent(uint64_t maxNs) {
  const uint64_t now = m_clock.nowNs();
  const uint64_t limit = maxNs > kNever - now ? kNever : now + maxNs;
  m_clock.waitUntil(std::min(nextWakeNs(), limit));
}

void MachineTime::guestIdle() {
  if (m_clock.noteIdle())
    waitForEvent();
}

void MachineTime::delay(uint64_t ns) {
  const uint64_t now = m_clock.nowNs();
  const uint64_t deadline = ns > kNever - now ? kNever : now + ns;
  for (;;) {
    if (m_hooks.serviceDevices)
      m_hooks.serviceDevices();
    const uint64_t t = m_clock.nowNs();
    if (t >= deadline)
      return;
    if (m_hooks.servicePeriodic && !m_hooks.servicePeriodic())
      return;
    waitForEvent(std::min(deadline - t, kWaitSliceNs));
  }
}

void MachineTime::halt() {
  // HLT completes an STI shadow: the interrupt is taken after it.
  m_cpu.endInterruptShadow();
  if (!(m_cpu.getEFLAGS() & cpu::FLAG_INTERRUPT)) {
    if (m_haltsIfClear++ == 0)
      LOG_WARN("HLT with interrupts disabled at ", std::hex,
               m_cpu.getInstructionStartCS(), ":",
               m_cpu.getInstructionStartEIP(),
               ": a real CPU would hang here; continuing");
    return;
  }
  if (!m_hooks.serviceDevices || !m_hooks.deliverInterrupt)
    return;
  ++m_halts;
  const uint64_t start = m_clock.nowNs();
  for (;;) {
    m_hooks.serviceDevices();
    if (m_hooks.deliverInterrupt())
      break;
    if (m_hooks.servicePeriodic && !m_hooks.servicePeriodic())
      break;
    waitForEvent();
  }
  m_haltedNs += m_clock.nowNs() - start;
}

} // namespace fador::hw
