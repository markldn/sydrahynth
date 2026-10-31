#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include <oaknut/code_block.hpp>
#include <oaknut/oaknut.hpp>

#include "dsp_core.h"

namespace hydra::dsp {

class DspJitArm64 {
public:
    explicit DspJitArm64(DspCore& core);
    ~DspJitArm64();

    std::uint64_t run(std::uint64_t budget);
    void invalidate() { flush_.store(true, std::memory_order_release); }

private:
    enum Result : std::uint32_t { kContinue, kIdle, kBudget };
    using Enter = std::uint32_t (*)(DspCore*, const void*, std::uint64_t*, std::uint64_t);
    struct Instruction { std::uint32_t pc; Decoded d; };

    bool supports(const Decoded& d) const;
    const void* compile(std::uint32_t pc);
    void flushAll();
    void emitCommon();
    void emitInstruction(const Instruction& in);
    void emitLoadData(std::uint32_t address);
    void emitLoadDataDynamic();
    void emitStoreData(std::uint32_t address);
    void emitStoreDataDynamic();
    void emitIncrementWrites();
    void emitSetPcAndDispatch(std::uint32_t target, bool checkIdle);
    void emitDispatch();
    void emitExit(Result result, bool count = true);
    void emitCall(const void* function);
    void emitLoadReg(oaknut::WReg target, int index);
    void emitStoreReg(int index, oaknut::WReg value);

    static std::uint32_t idleThunk(DspCore* core, std::uint32_t target);
    static std::uint32_t dataRegReadThunk(DspCore* core, std::uint32_t address);
    static void dataRegWriteThunk(DspCore* core, std::uint32_t address, std::uint32_t value);
    static std::uint32_t ioReadThunk(DspCore* core, std::uint32_t address);
    static void ioWriteThunk(DspCore* core, std::uint32_t address, std::uint32_t value);
    static std::uint32_t extReadThunk(DspCore* core, std::uint32_t address);
    static void extWriteThunk(DspCore* core, std::uint32_t address, std::uint32_t value);
    static std::uint32_t portReadThunk(DspCore* core, std::uint32_t address);
    static void portWriteThunk(DspCore* core, std::uint32_t address, std::uint32_t value);
    static void biquadThunk(DspCore* core, std::uint32_t shift);

    DspCore& core_;
    oaknut::CodeBlock memory_;
    oaknut::CodeGenerator code_;
    std::array<const void*, DspCore::kProgramWords> blocks_{};
    std::atomic<bool> flush_{false};
    bool disabled_ = false;
    Enter enter_ = nullptr;
    const void* dispatch_ = nullptr;
    const void* exit_ = nullptr;
    const void* exitNormal_ = nullptr;
    std::int32_t pcOffset_ = 0, pendingOffset_ = 0, writesOffset_ = 0, spOffset_ = 0;
    std::int32_t stackOffset_ = 0, interruptOffset_ = 0;
    std::uint32_t currentBlockLength_ = 0;
    bool profileFallbacks_ = false;
    std::array<std::uint64_t, static_cast<std::size_t>(Op::AbsSat) + 1> fallbackCounts_{};
    std::array<std::uint64_t, DspCore::kProgramWords> fallbackPcCounts_{};
};

}  // namespace hydra::dsp
