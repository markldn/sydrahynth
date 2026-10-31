#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace hydra::dsp {

class DspJit;
class DspJitArm64;

// The custom DSP used both for the 8 main DSP cores and for the two effects DSPs.
//
// Programming model:
//  - 32-bit instructions, 12-bit PC over a 4096-word program memory.
//  - r0-r15, 32 bit. r15 doubles as the status register: bit 0 enables
//    interrupts, bit 1 enables the sample interrupt, and on interrupt entry
//    the return PC is kept in bits 16-27.
//  - Data memory: masked local RAM; addresses 0-15 alias r0-r15; addresses at
//    or above Memory::dataLimit are host registers (the effects DSPs' delay RAM port).
//  - "io" space (17 bit): shared RAM at 0x8000-0xFEFF and 0x10000-0x1FBFF
//    (word = address - 0x8000) is accessed directly; everything else goes to
//    the host. "ext" (16 bit) and "port" spaces always go to the host.
//  - Two interrupt lines: line 1 (vector 1) and line 2, the sample
//    interrupt (vector 2, also gated by r15 bit 1).
//  - A 16-entry return stack for call/return.
//
// Instruction word: [31:28] major, [27:24] A, [23:20] B, [19:16] minor,
// [15:0] operand. See decode() for the per-opcode layouts.
enum class Op : std::uint8_t {
    Nop, Invalid, AddEightMem,
    IoLoad, IoStore, ExtLoad, ExtStore, IoLoadIndexed, IoStoreIndexed,
    Load, Store, LoadIndexed, StoreIndexed,
    AddImm, SubImm, MovImm, AddImmLow, SubImmLow, MovImmLow, AndImmLow, OrImmLow, MovImmHigh, AndImmHigh,
    BranchGreater, BranchGreaterEqual, BranchEqual, BranchNotEqual,
    BranchHighEqualLow, BranchHighNotEqualLow, BranchLowEqual, BranchLowNotEqual,
    BranchBitSet, BranchBitClear,
    JumpIfZero, JumpIfNotZero, Loop, Call, JumpLink, Jump, Return, JumpRegister, ReturnFromInterrupt,
    Mul, MulAcc, MulMem, AddMem, AddMemSatStore, AddMemSat, SubMem, DelayRead, DelayWrite,
    Add, AddSat, Sub, SubSat, And, Or, Shl, Sar, Max, Min,
    LoadReg, LoadRegInc, StoreReg, StoreRegInc, IoLoadReg, IoLoadRegInc, IoStoreReg, IoStoreRegInc,
    Biquad, SarImm, ShlImm, ShrImm, PortRead, PortWrite,
    Mov, MovHighToLow, MovLowToHigh, MovHigh, NegSat, AbsSat,
};

struct Decoded {
    Op op = Op::Invalid;
    std::uint8_t a = 0, b = 0, c = 0, shift = 0;
    std::int16_t offset = 0;    // relative branch offset
    std::uint32_t operand = 0;  // immediate, address or jump target
};

Decoded decode(std::uint32_t instruction);

class Host {
public:
    virtual ~Host() = default;
    virtual std::uint32_t ioRead(int core, std::uint32_t address) = 0;
    virtual void ioWrite(int core, std::uint32_t address, std::uint32_t value) = 0;
    virtual std::uint32_t extRead(int core, std::uint32_t address) = 0;
    virtual void extWrite(int core, std::uint32_t address, std::uint32_t value) = 0;
    virtual std::uint32_t portRead(int core, std::uint32_t port) = 0;
    virtual void portWrite(int core, std::uint32_t port, std::uint32_t value) = 0;
    // Data memory addresses at or above Memory::dataLimit (memory-mapped registers).
    virtual std::uint32_t dataRegRead(int, std::uint32_t) { return 0; }
    virtual void dataRegWrite(int, std::uint32_t, std::uint32_t) {}
};

class DspCore {
public:
    static constexpr std::uint32_t kProgramWords = 4096;
    static constexpr std::uint32_t kSharedWords = 0x17C00;

    struct Memory {
        std::uint32_t* program;  // kProgramWords
        std::uint32_t* data;     // dataMask + 1 words
        std::uint32_t dataMask;
        std::uint32_t* shared;   // kSharedWords, or nullptr
        // Masked data addresses from here on go to Host::dataRegRead/Write.
        std::uint32_t dataLimit = UINT32_MAX;
    };

    DspCore(int id, Host& host, Memory memory);
    ~DspCore();

    void reset();
    // Executes up to `instructions` instructions; returns how many were
    // accounted for. An idle loop (a backward branch that changes nothing)
    // consumes the rest of the budget at once.
    std::uint64_t run(std::uint64_t instructions);
    // Compiles DSP programs on supported hosts instead of interpreting them.
    // Use HYDRA_DSP_JIT to override the host's default during development.
    void setJit(bool enabled);
    static bool jitByDefault();
    void raiseInterrupt(int line) { pending_ |= 1u << line; }
    // Program memory word `pc` changed: drop its decoded form. Every write to
    // program memory after construction must be reported (the host's job).
    void invalidate(std::uint32_t pc);

    // Idle detection for unconditional backward jumps (conditional branches always use it).
    void setJumpIdleDetection(bool enabled) { jumpIdleDetection_ = enabled; }

    std::array<std::uint32_t, 16>& regs() { return r_; }
    std::uint32_t pc() const { return pc_; }
    void setPc(std::uint32_t pc) { pc_ = pc & 0xFFF; }
    bool trapped() const { return trapped_; }
    const std::string& trapMessage() const { return trapMessage_; }
    std::uint32_t pending() const { return pending_; }
    bool inInterrupt() const { return inInterrupt_; }
    std::uint64_t interruptsTaken(int line) const { return interruptsTaken_[line]; }
    // Diagnostic: called for every io-space write (address, value) when set.
    std::function<void(std::uint32_t, std::uint32_t)> onIoWrite;

private:
    struct CacheEntry {
        Decoded decoded;
        bool valid = false;
    };

    friend class DspJit;
    friend class DspJitArm64;
    friend struct JitThunks;

    // The interpreter: runs up to `limit` instructions of a run() with `budget`
    // (an idle loop returns `budget`).
    std::uint64_t interpret(std::uint64_t budget, std::uint64_t limit);
    const Decoded& fetch(std::uint32_t pc);
    bool takeInterrupt();
    std::uint32_t dataLoad(std::uint32_t address);
    void dataStore(std::uint32_t address, std::uint32_t value);
    std::uint32_t ioRead(std::uint32_t address);
    void ioWrite(std::uint32_t address, std::uint32_t value);
    // Returns true when the branch closes an idle loop.
    bool branchIdle(std::uint32_t target);

    int id_;
    Host& host_;
    Memory mem_;
    std::array<std::uint32_t, 16> r_{};
    std::uint32_t pc_ = 0;
    std::uint32_t pending_ = 0;
    bool inInterrupt_ = false;
    bool trapped_ = false;
    bool jumpIdleDetection_ = false;
    std::array<std::uint64_t, 3> interruptsTaken_{};
    std::string trapMessage_;
    std::array<std::uint32_t, 16> stack_{};
    std::uint32_t sp_ = 0;
    std::array<CacheEntry, kProgramWords> cache_{};

    // Idle-loop detection: state seen at the last backward branch.
    std::uint64_t writes_ = 0;
    std::uint32_t idleTarget_ = ~0u;
    std::uint64_t idleWrites_ = ~0ull;
    std::array<std::uint32_t, 16> idleRegs_{};

#if defined(__x86_64__)
    std::unique_ptr<DspJit> jit_;
#endif
#if defined(__aarch64__) || defined(__arm64__)
    std::unique_ptr<DspJitArm64> arm64Jit_;
#endif
};

}  // namespace hydra::dsp
