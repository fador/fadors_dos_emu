#pragma once
// Selectable x86 CPU models: identification (reset signature, CPUID),
// architectural differences (EFLAGS/CR0/CR4 bits, model-gated
// instructions) and the approximate per-instruction clock costs that drive
// the guest cycle counter (CPU::cycles()).
#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

namespace fador::cpu {

enum class CpuModelId : uint8_t {
  I386DX,     // 386DX + 387: no CPUID, no AC flag, no 486 instructions
  I486DX,     // early 486DX (family 4 model 0): 486 ISA, AC flag, no CPUID
  I486DX2,    // 486DX2 with CPUID (family 4 model 3)
  I486DX4,    // IntelDX4 (family 4 model 8)
  Pentium,    // P54C (family 5 model 2): CPUID, TSC, MSRs, CMPXCHG8B, CR4
  PentiumPro, // P6 (family 6 model 1): adds CMOVcc / FCMOVcc / FCOMI
};

// The default reproduces the emulator's historic identity (CPUID present,
// family 4) as closely as possible: a CPUID-capable 486.
inline constexpr CpuModelId kDefaultCpuModel = CpuModelId::I486DX2;

// Model-dependent instructions and architectural features.  Checked only
// by the rare opcodes they gate, never on the hot path.
namespace feature {
// BSWAP, CMPXCHG, XADD, INVD, WBINVD, INVLPG, EFLAGS.AC, CR0.NE/WP/AM/NW/CD
inline constexpr uint32_t k486 = 1u << 0;
inline constexpr uint32_t kCpuid = 1u << 1; // CPUID, EFLAGS.ID writable
inline constexpr uint32_t kTsc = 1u << 2;   // RDTSC, CR4.TSD
inline constexpr uint32_t kMsr = 1u << 3;   // RDMSR / WRMSR
inline constexpr uint32_t kCx8 = 1u << 4;   // CMPXCHG8B
inline constexpr uint32_t kCr4 = 1u << 5;   // CR4 exists; EFLAGS.VIF/VIP
inline constexpr uint32_t kCmov = 1u << 6;  // CMOVcc, FCMOVcc, FCOMI/FUCOMI(P)
} // namespace feature

// CPUID leaf 1 EDX feature flags.
namespace cpuid_edx {
inline constexpr uint32_t kFpu = 1u << 0;
inline constexpr uint32_t kVme = 1u << 1;
inline constexpr uint32_t kDe = 1u << 2;
inline constexpr uint32_t kPse = 1u << 3;
inline constexpr uint32_t kTsc = 1u << 4;
inline constexpr uint32_t kMsr = 1u << 5;
inline constexpr uint32_t kCx8 = 1u << 8;
inline constexpr uint32_t kCmov = 1u << 15;
} // namespace cpuid_edx

// EFLAGS bits.
namespace eflags {
inline constexpr uint32_t kReserved1 = 0x00000002; // always reads as 1
inline constexpr uint32_t kBase = 0x00007FD5; // CF PF AF ZF SF TF IF DF OF IOPL NT
inline constexpr uint32_t kIF = 0x00000200;
inline constexpr uint32_t kIOPL = 0x00003000;
inline constexpr uint32_t kNT = 0x00004000;
inline constexpr uint32_t kRF = 0x00010000;
inline constexpr uint32_t kVM = 0x00020000;
inline constexpr uint32_t kAC = 0x00040000;
inline constexpr uint32_t kVIF = 0x00080000;
inline constexpr uint32_t kVIP = 0x00100000;
inline constexpr uint32_t kID = 0x00200000;
} // namespace eflags

// Instruction cost classes for the guest cycle counter.
enum class CycleClass : uint8_t {
  Alu,        // reg/imm ALU, MOV, LEA, INC/DEC, TEST, CMC/CLC/..., NOP
  Shift,      // shifts and rotates, CBW/CWD
  Movx,       // MOVZX/MOVSX, SETcc, SHLD/SHRD, XADD
  Xchg,       // XCHG
  Push,       // PUSH r/imm/sreg
  Pop,        // POP r, MOV accumulator <-> moffs, XLAT
  PushPopAll, // PUSHA / POPA
  SegLoad,    // MOV/POP sreg, LDS/LES/LSS/LFS/LGS (real-mode timing)
  Branch,     // Jcc (taken/not-taken average), JMP near/short
  Loop,       // LOOP/LOOPcc/JCXZ
  Call,       // CALL near
  Ret,        // RET near
  FarCall,    // CALL/JMP far, RETF (real-mode timing)
  Interrupt,  // INT n / INT3 / INTO (real-mode timing)
  Iret,       // IRET
  Flags,      // PUSHF/POPF/SAHF/LAHF/CLI/STI
  Mul,        // IMUL r,r/m(,imm), AAM/AAD; MUL/IMUL/DIV/IDIV add extra
  Div,        // reference cost of DIV/IDIV (added by the F6/F7 handlers)
  Bit,        // BT/BTS/BTR/BTC, CMPXCHG
  Scan,       // BSF/BSR
  String,     // one MOVS/CMPS/SCAS/LODS/STOS (REP adds per-iteration cost)
  Io,         // IN/OUT/INS/OUTS: dominated by the ISA bus (~0.8 us)
  EnterLeave, // ENTER/LEAVE/BOUND
  Bcd,        // DAA/DAS/AAA/AAS
  System,     // descriptor-table, control-register, CPUID, MSR, cache ops
  Fpu,        // any x87 instruction (flat average)
  Hlt,        // HLT (the wait itself is not modelled here)
  Count
};
inline constexpr size_t kCycleClassCount = static_cast<size_t>(CycleClass::Count);

// Per-iteration costs of REP-prefixed string instructions (REP INS/OUTS
// use the Io cost per element).
struct RepCycles {
  uint8_t movs, stos, lods, cmps, scas;
};

struct CpuModel {
  CpuModelId id;
  std::string_view cliName;     // value of --cpu=
  std::string_view description; // human-readable name
  uint32_t clockHz;             // nominal core clock of the modelled part
  uint32_t signature;           // EDX at reset; CPUID.1:EAX on CPUID models
  uint32_t features;            // feature:: bits
  uint32_t cpuidMaxLeaf;        // CPUID.0:EAX (CPUID models only)
  uint32_t cpuidFeatureEdx;     // CPUID.1:EDX: only features really emulated
  uint32_t eflagsMask;          // EFLAGS bits that exist (excl. bit 1)
  uint32_t cr0Mask;             // CR0 bits MOV CR0 / LMSW can change
  uint32_t cr0Fixed;            // CR0 bits hardwired to 1 (ET on 486+)
  uint32_t cr4Mask;             // CR4 bits that may be set (kCr4 models)
  std::array<uint8_t, kCycleClassCount> cycles;
  uint8_t memOperandCycles;     // extra cost of a ModRM memory operand
  RepCycles rep;
};

const CpuModel &cpuModel(CpuModelId id);
std::optional<CpuModelId> parseCpuModel(std::string_view name);
// Comma-separated --cpu values for help texts.
std::string_view cpuModelNames();

// Cost class of a one-byte (twoByte=false) or 0F-prefixed opcode.
CycleClass cycleClassOf(uint8_t opcode, bool twoByte);

// Cycle table layout used by the decoder: [0..255] one-byte opcodes,
// [256..511] 0F xx opcodes, [512..767] REP per-iteration cost by opcode.
inline constexpr size_t kCycleTableSize = 768;
using CycleTable = std::array<uint8_t, kCycleTableSize>;
void buildCycleTable(const CpuModel &model, CycleTable &table);

} // namespace fador::cpu
