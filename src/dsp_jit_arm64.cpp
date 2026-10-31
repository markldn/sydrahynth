#include "dsp_jit_arm64.h"

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

namespace hydra::dsp {
namespace {
using namespace oaknut;
using namespace oaknut::util;

constexpr std::size_t kCodeSize = 4u << 20;
constexpr std::uint32_t kMaxBlock = 64;

template <typename T>
std::int32_t offsetIn(const DspCore& core, const T& member) {
    return static_cast<std::int32_t>(reinterpret_cast<const char*>(&member) - reinterpret_cast<const char*>(&core));
}

bool isControlFlow(Op op) {
    switch (op) {
    case Op::BranchGreater: case Op::BranchGreaterEqual: case Op::BranchEqual: case Op::BranchNotEqual:
    case Op::BranchHighEqualLow: case Op::BranchHighNotEqualLow: case Op::BranchLowEqual: case Op::BranchLowNotEqual:
    case Op::BranchBitSet: case Op::BranchBitClear: case Op::JumpIfZero: case Op::JumpIfNotZero:
    case Op::Loop: case Op::Call: case Op::JumpLink: case Op::Jump: case Op::Return: case Op::JumpRegister:
    case Op::ReturnFromInterrupt:
        return true;
    default:
        return false;
    }
}
}  // namespace

DspJitArm64::DspJitArm64(DspCore& core)
    : core_(core), memory_(kCodeSize), code_(memory_.ptr()) {
    profileFallbacks_ = std::getenv("HYDRA_DSP_JIT_PROFILE") != nullptr;
    pcOffset_ = offsetIn(core, core.pc_);
    pendingOffset_ = offsetIn(core, core.pending_);
    writesOffset_ = offsetIn(core, core.writes_);
    spOffset_ = offsetIn(core, core.sp_);
    stackOffset_ = offsetIn(core, core.stack_);
    interruptOffset_ = offsetIn(core, core.inInterrupt_);
    if (memory_.ptr() == reinterpret_cast<std::uint32_t*>(-1))
        throw std::bad_alloc{};
    // MAP_JIT pages start in execute mode on macOS. The common entry/exit
    // stubs are emitted here before any call to compile(), so explicitly
    // enable writes first.
    memory_.unprotect();
    emitCommon();
    memory_.invalidate(memory_.ptr(), static_cast<std::size_t>(code_.offset()));
    memory_.protect();
}

DspJitArm64::~DspJitArm64() {
    if (!profileFallbacks_)
        return;
    for (std::size_t op = 0; op < fallbackCounts_.size(); ++op)
        if (fallbackCounts_[op] != 0)
            std::fprintf(stderr, "DSP JIT core %d fallback op %zu: %llu\n", core_.id_, op,
                         static_cast<unsigned long long>(fallbackCounts_[op]));
    auto counts = fallbackPcCounts_;
    for (int n = 0; n < 12; ++n) {
        const auto it = std::max_element(counts.begin(), counts.end());
        if (it == counts.end() || *it == 0)
            break;
        const std::size_t pc = static_cast<std::size_t>(it - counts.begin());
        std::fprintf(stderr, "DSP JIT core %d fallback pc %03zx: %llu (instruction %08x)\n", core_.id_, pc,
                     static_cast<unsigned long long>(*it), core_.mem_.program[pc]);
        *it = 0;
    }
}

std::uint32_t DspJitArm64::idleThunk(DspCore* core, std::uint32_t target) {
    return core->branchIdle(target) ? 1u : 0u;
}

std::uint32_t DspJitArm64::dataRegReadThunk(DspCore* core, std::uint32_t address) {
    return core->host_.dataRegRead(core->id_, address);
}

void DspJitArm64::dataRegWriteThunk(DspCore* core, std::uint32_t address, std::uint32_t value) {
    core->host_.dataRegWrite(core->id_, address, value);
}

std::uint32_t DspJitArm64::ioReadThunk(DspCore* core, std::uint32_t address) {
    return core->ioRead(address);
}

void DspJitArm64::ioWriteThunk(DspCore* core, std::uint32_t address, std::uint32_t value) {
    core->ioWrite(address, value);
}

std::uint32_t DspJitArm64::extReadThunk(DspCore* core, std::uint32_t address) {
    return core->host_.extRead(core->id_, address);
}

void DspJitArm64::extWriteThunk(DspCore* core, std::uint32_t address, std::uint32_t value) {
    core->host_.extWrite(core->id_, address, value);
}

std::uint32_t DspJitArm64::portReadThunk(DspCore* core, std::uint32_t address) {
    return core->host_.portRead(core->id_, address);
}

void DspJitArm64::portWriteThunk(DspCore* core, std::uint32_t address, std::uint32_t value) {
    core->host_.portWrite(core->id_, address, value);
}

void DspJitArm64::biquadThunk(DspCore* core, std::uint32_t shift) {
    auto& r = core->r_;
    for (int i = 0; i < 2; ++i) {
        const std::uint32_t x = core->dataLoad(r[11]);
        const __int128 acc = static_cast<__int128>(static_cast<std::int64_t>(static_cast<std::int32_t>(x)) * static_cast<std::int32_t>(r[10])) +
                             static_cast<std::int64_t>(static_cast<std::int32_t>(r[9])) * static_cast<std::int32_t>(r[5]) +
                             static_cast<std::int64_t>(static_cast<std::int32_t>(r[8])) * static_cast<std::int32_t>(r[4]) +
                             static_cast<std::int64_t>(static_cast<std::int32_t>(r[7])) * static_cast<std::int32_t>(r[3]) +
                             static_cast<std::int64_t>(static_cast<std::int32_t>(r[6])) * static_cast<std::int32_t>(r[2]);
        __int128 y = acc >> shift;
        if (y < -0x80000000LL)
            y = -0x80000000LL;
        if (y > 0x7FFFFFFF)
            y = 0x7FFFFFFF;
        r[4] = r[5];
        r[5] = x;
        r[2] = r[3];
        r[3] = static_cast<std::uint32_t>(static_cast<std::int64_t>(y));
        const std::uint32_t address = r[11]++;
        ++core->writes_;
        core->dataStore(address, r[3]);
    }
}

void DspJitArm64::emitCall(const void* function) {
    code_.MOV(X16, reinterpret_cast<std::uint64_t>(function));
    code_.BLR(X16);
}

void DspJitArm64::emitLoadReg(WReg target, int index) {
    code_.LDR(target, X22, index * 4);
}

void DspJitArm64::emitStoreReg(int index, WReg value) {
    code_.STR(value, X22, index * 4);
}

void DspJitArm64::emitIncrementWrites() {
    // writes_ follows the decoded-program cache and can exceed Oaknut's
    // 12-bit scaled LDR/STR immediate. Form the full address in a register.
    code_.MOV(X10, static_cast<std::uint64_t>(writesOffset_));
    code_.ADD(X10, X19, X10);
    code_.LDR(X9, X10);
    code_.ADD(X9, X9, 1);
    code_.STR(X9, X10);
}

void DspJitArm64::emitExit(Result result, bool count) {
    if (count) {
        code_.LDR(X9, X20);
        code_.ADD(X9, X9, currentBlockLength_);
        code_.STR(X9, X20);
    }
    code_.MOV(W0, static_cast<std::uint32_t>(result));
    code_.B(exit_);
}

void DspJitArm64::emitCommon() {
    using namespace oaknut;
    using namespace oaknut::util;

    const std::uintptr_t base = reinterpret_cast<std::uintptr_t>(memory_.ptr());
    enter_ = reinterpret_cast<Enter>(base + code_.offset());
    code_.STP(X19, X20, SP, PreIndexed{}, -16);
    code_.STP(X21, X22, SP, PreIndexed{}, -16);
    code_.STP(X23, X30, SP, PreIndexed{}, -16);
    code_.MOV(X19, X0);
    code_.MOV(X20, X2);
    code_.MOV(X21, X3);
    code_.MOV(X22, reinterpret_cast<std::uint64_t>(&core_.r_));
    code_.MOV(X23, reinterpret_cast<std::uint64_t>(core_.mem_.data));
    code_.BR(X1);

    exit_ = reinterpret_cast<const void*>(base + code_.offset());
    code_.LDP(X23, X30, SP, PostIndexed{}, 16);
    code_.LDP(X21, X22, SP, PostIndexed{}, 16);
    code_.LDP(X19, X20, SP, PostIndexed{}, 16);
    code_.RET();

    exitNormal_ = reinterpret_cast<const void*>(base + code_.offset());
    code_.MOV(W0, kContinue);
    code_.B(exit_);

    dispatch_ = reinterpret_cast<const void*>(base + code_.offset());
    code_.STR(W9, X19, pcOffset_);
    code_.LDR(W10, X19, pendingOffset_);
    code_.CBNZ(W10, exitNormal_);
    code_.MOV(X10, reinterpret_cast<std::uint64_t>(&flush_));
    code_.LDRB(W10, X10);
    code_.CBNZ(W10, exitNormal_);
    code_.LDR(X10, X20);
    code_.CMP(X10, X21);
    code_.B(HS, exitNormal_);
    code_.AND(W9, W9, 0xFFF);
    code_.MOV(X10, reinterpret_cast<std::uint64_t>(blocks_.data()));
    code_.ADD(X10, X10, W9, AddSubExt::UXTW, 3);
    code_.LDR(X10, X10);
    code_.CBZ(X10, exitNormal_);
    code_.BR(X10);
}

void DspJitArm64::emitDispatch() {
    code_.LDR(X10, X20);
    code_.ADD(X10, X10, currentBlockLength_);
    code_.STR(X10, X20);
    code_.B(dispatch_);
}

void DspJitArm64::emitSetPcAndDispatch(std::uint32_t target, bool checkIdle) {
    code_.MOV(W9, target & 0xFFF);
    code_.STR(W9, X19, pcOffset_);
    if (checkIdle) {
        Label idle;
        code_.MOV(X0, X19);
        code_.MOV(W1, target & 0xFFF);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::idleThunk));
        code_.CBNZ(W0, idle);
        code_.MOV(W9, target & 0xFFF);
        emitDispatch();
        code_.l(idle);
        emitExit(kIdle);
    } else {
        emitDispatch();
    }
}

void DspJitArm64::emitLoadData(std::uint32_t address) {
    const DspCore::Memory& mem = core_.mem_;
    address &= mem.dataMask;
    if (address <= 15) {
        emitLoadReg(W9, static_cast<int>(address));
    } else if (address >= mem.dataLimit) {
        code_.MOV(X0, X19);
        code_.MOV(W1, address);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::dataRegReadThunk));
        code_.MOV(W9, W0);
    } else {
        code_.LDR(W9, X23, address * 4);
    }
}

void DspJitArm64::emitLoadDataDynamic() {
    const DspCore::Memory& mem = core_.mem_;
    Label isRegister, host, done;
    code_.MOV(W10, mem.dataMask);
    code_.AND(W9, W9, W10);
    code_.CMP(W9, 15);
    code_.B(LS, isRegister);
    if (mem.dataLimit <= mem.dataMask) {
        code_.MOV(W10, mem.dataLimit);
        code_.CMP(W9, W10);
        code_.B(HS, host);
    }
    code_.ADD(X10, X23, W9, AddSubExt::UXTW, 2);
    code_.LDR(W9, X10);
    code_.B(done);
    code_.l(isRegister);
    code_.ADD(X10, X22, W9, AddSubExt::UXTW, 2);
    code_.LDR(W9, X10);
    code_.B(done);
    if (mem.dataLimit <= mem.dataMask) {
        code_.l(host);
        code_.MOV(X0, X19);
        code_.MOV(W1, W9);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::dataRegReadThunk));
        code_.MOV(W9, W0);
    }
    code_.l(done);
}

void DspJitArm64::emitStoreData(std::uint32_t address) {
    const DspCore::Memory& mem = core_.mem_;
    address &= mem.dataMask;
    if (address <= 15) {
        emitStoreReg(static_cast<int>(address), W11);
    } else if (address >= mem.dataLimit) {
        code_.MOV(X0, X19);
        code_.MOV(W1, address);
        code_.MOV(W2, W11);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::dataRegWriteThunk));
    } else {
        code_.STR(W11, X23, address * 4);
    }
}

void DspJitArm64::emitStoreDataDynamic() {
    const DspCore::Memory& mem = core_.mem_;
    Label isRegister, host, done;
    code_.MOV(W10, mem.dataMask);
    code_.AND(W9, W9, W10);
    code_.CMP(W9, 15);
    code_.B(LS, isRegister);
    if (mem.dataLimit <= mem.dataMask) {
        code_.MOV(W10, mem.dataLimit);
        code_.CMP(W9, W10);
        code_.B(HS, host);
    }
    code_.ADD(X10, X23, W9, AddSubExt::UXTW, 2);
    code_.STR(W11, X10);
    code_.B(done);
    code_.l(isRegister);
    code_.ADD(X10, X22, W9, AddSubExt::UXTW, 2);
    code_.STR(W11, X10);
    code_.B(done);
    if (mem.dataLimit <= mem.dataMask) {
        code_.l(host);
        code_.MOV(X0, X19);
        code_.MOV(W1, W9);
        code_.MOV(W2, W11);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::dataRegWriteThunk));
    }
    code_.l(done);
}

bool DspJitArm64::supports(const Decoded& d) const {
    switch (d.op) {
    case Op::Nop: case Op::AddEightMem: case Op::Load: case Op::Store: case Op::LoadIndexed: case Op::StoreIndexed:
    case Op::ExtLoad: case Op::ExtStore: case Op::IoLoad: case Op::IoStore: case Op::IoLoadIndexed: case Op::IoStoreIndexed:
    case Op::AddImm: case Op::SubImm: case Op::MovImm: case Op::AddImmLow: case Op::SubImmLow:
    case Op::MovImmLow: case Op::AndImmLow: case Op::OrImmLow: case Op::MovImmHigh: case Op::AndImmHigh:
    case Op::BranchGreater: case Op::BranchGreaterEqual: case Op::BranchEqual: case Op::BranchNotEqual:
    case Op::BranchHighEqualLow: case Op::BranchHighNotEqualLow: case Op::BranchLowEqual: case Op::BranchLowNotEqual:
    case Op::BranchBitSet: case Op::BranchBitClear: case Op::JumpIfZero: case Op::JumpIfNotZero:
    case Op::Loop: case Op::Call: case Op::JumpLink: case Op::Jump: case Op::Return: case Op::JumpRegister:
    case Op::ReturnFromInterrupt: case Op::Mul: case Op::MulAcc: case Op::MulMem: case Op::AddMem:
    case Op::AddMemSatStore: case Op::AddMemSat: case Op::SubMem: case Op::Add: case Op::AddSat:
    case Op::Sub: case Op::SubSat: case Op::And: case Op::Or: case Op::Shl: case Op::Sar:
    case Op::Max: case Op::Min: case Op::LoadReg: case Op::LoadRegInc: case Op::StoreReg: case Op::StoreRegInc:
    case Op::IoLoadReg: case Op::IoLoadRegInc: case Op::IoStoreReg: case Op::IoStoreRegInc:
    case Op::Biquad: case Op::SarImm: case Op::ShlImm: case Op::ShrImm: case Op::PortRead: case Op::PortWrite:
    case Op::Mov: case Op::MovHighToLow: case Op::MovLowToHigh: case Op::MovHigh: case Op::NegSat: case Op::AbsSat:
        return true;
    default:
        return false;
    }
}

void DspJitArm64::emitInstruction(const Instruction& in) {
    const Decoded& d = in.d;
    const std::uint32_t next = (in.pc + 1) & 0xFFF;
    const auto loadA = [&] { emitLoadReg(W9, d.a); };
    const auto loadB = [&] { emitLoadReg(W10, d.b); };
    const auto storeA = [&] { emitStoreReg(d.a, W9); };
    const auto storeC = [&] { emitStoreReg(d.c, W9); };
    const auto branch = [&](Cond cond) {
        Label taken;
        code_.B(cond, taken);
        emitSetPcAndDispatch(next, false);
        code_.l(taken);
        const std::uint32_t target = (in.pc + static_cast<std::uint32_t>(d.offset) + 1) & 0xFFF;
        emitSetPcAndDispatch(target, d.offset < 0);
    };

    switch (d.op) {
    case Op::Nop: return;
    case Op::AddEightMem:
        emitIncrementWrites(); emitLoadData(d.operand); code_.ADD(W9, W9, 8); code_.MOV(W11, W9); emitStoreData(d.operand); return;
    case Op::Load: emitLoadData(d.operand); storeA(); return;
    case Op::Store: emitIncrementWrites(); loadA(); code_.MOV(W11, W9); emitStoreData(d.operand); return;
    case Op::LoadIndexed:
        loadB(); code_.MOV(W11, d.operand); code_.ADD(W9, W10, W11); emitLoadDataDynamic(); storeA(); return;
    case Op::StoreIndexed:
        emitIncrementWrites(); loadB(); code_.MOV(W11, d.operand); code_.ADD(W9, W10, W11);
        emitLoadReg(W11, d.a); emitStoreDataDynamic(); return;
    case Op::ExtLoad:
        code_.MOV(X0, X19); code_.MOV(W1, d.operand);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::extReadThunk)); code_.MOV(W9, W0); storeA(); return;
    case Op::ExtStore:
        emitIncrementWrites(); loadA(); code_.MOV(X0, X19); code_.MOV(W1, d.operand); code_.MOV(W2, W9);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::extWriteThunk)); return;
    case Op::LoadReg:
        loadB(); code_.MOV(W9, W10); emitLoadDataDynamic(); storeA(); return;
    case Op::LoadRegInc:
        loadB(); code_.ADD(W11, W10, 1); emitStoreReg(d.b, W11); code_.MOV(W9, W10);
        emitLoadDataDynamic(); storeA(); return;
    case Op::StoreReg:
        emitIncrementWrites(); loadA(); loadB(); code_.MOV(W11, W10); emitStoreDataDynamic(); return;
    case Op::StoreRegInc:
        loadA(); code_.MOV(W12, W9); code_.ADD(W11, W9, 1); emitStoreReg(d.a, W11); emitIncrementWrites();
        loadB(); code_.MOV(W11, W10); code_.MOV(W9, W12); emitStoreDataDynamic(); return;
    case Op::IoLoad:
        code_.MOV(W9, d.operand & 0x1FFFF); code_.MOV(X0, X19); code_.MOV(W1, W9);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::ioReadThunk)); code_.MOV(W9, W0); storeA(); return;
    case Op::IoLoadIndexed:
        loadB(); code_.MOV(W11, d.operand); code_.ADD(W9, W10, W11); code_.AND(W9, W9, 0xFFFF);
        code_.MOV(X0, X19); code_.MOV(W1, W9);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::ioReadThunk)); code_.MOV(W9, W0); storeA(); return;
    case Op::IoStoreIndexed:
        loadB(); code_.MOV(W11, d.operand); code_.ADD(W9, W10, W11); code_.AND(W9, W9, 0xFFFF);
        code_.MOV(W12, W9); loadA(); code_.MOV(X0, X19); code_.MOV(W1, W12); code_.MOV(W2, W9);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::ioWriteThunk)); return;
    case Op::IoLoadReg:
        loadB(); code_.AND(W9, W10, 0x1FFFF); code_.MOV(X0, X19); code_.MOV(W1, W9);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::ioReadThunk)); code_.MOV(W9, W0); storeA(); return;
    case Op::IoLoadRegInc:
        loadB(); code_.ADD(W11, W10, 1); emitStoreReg(d.b, W11); code_.AND(W9, W10, 0x1FFFF);
        code_.MOV(X0, X19); code_.MOV(W1, W9); emitCall(reinterpret_cast<const void*>(&DspJitArm64::ioReadThunk));
        code_.MOV(W9, W0); storeA(); return;
    case Op::IoStore:
        code_.MOV(W9, d.operand & 0x1FFFF); loadA(); code_.MOV(W11, W9); code_.MOV(W9, d.operand & 0x1FFFF);
        code_.MOV(X0, X19); code_.MOV(W1, W9); code_.MOV(W2, W11);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::ioWriteThunk)); return;
    case Op::IoStoreReg:
        loadA(); code_.AND(W9, W9, 0x1FFFF); loadB(); code_.MOV(W11, W10);
        code_.MOV(X0, X19); code_.MOV(W1, W9); code_.MOV(W2, W11);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::ioWriteThunk)); return;
    case Op::IoStoreRegInc:
        loadA(); code_.ADD(W11, W9, 1); emitStoreReg(d.a, W11); code_.AND(W9, W9, 0x1FFFF);
        loadB(); code_.MOV(W11, W10); code_.MOV(X0, X19); code_.MOV(W1, W9); code_.MOV(W2, W11);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::ioWriteThunk)); return;
    case Op::AddImm: loadA(); code_.MOV(W10, d.operand); code_.ADD(W9, W9, W10); storeA(); return;
    case Op::SubImm: loadA(); code_.MOV(W10, d.operand); code_.SUB(W9, W9, W10); storeA(); return;
    case Op::MovImm: code_.MOV(W9, d.operand); storeA(); return;
    case Op::AddImmLow: case Op::SubImmLow:
        loadA(); code_.MOV(W10, d.operand);
        if (d.op == Op::AddImmLow) code_.ADD(W11, W9, W10); else code_.SUB(W11, W9, W10);
        code_.BFI(W9, W11, 0, 16); storeA(); return;
    case Op::MovImmLow:
        loadA(); code_.MOV(W10, d.operand); code_.BFI(W9, W10, 0, 16); storeA(); return;
    case Op::AndImmLow: loadA(); code_.AND(W9, W9, d.operand | 0xFFFF0000u); storeA(); return;
    case Op::OrImmLow: loadA(); code_.ORR(W9, W9, d.operand); storeA(); return;
    case Op::MovImmHigh: loadA(); code_.MOV(W10, d.operand); code_.BFI(W9, W10, 16, 16); storeA(); return;
    case Op::AndImmHigh: loadA(); code_.MOV(W10, (d.operand << 16) | 0xFFFFu); code_.AND(W9, W9, W10); storeA(); return;
    case Op::Add: case Op::Sub: case Op::And: case Op::Or: case Op::Max: case Op::Min:
        loadA(); loadB();
        if (d.op == Op::Add) code_.ADD(W9, W9, W10);
        else if (d.op == Op::Sub) code_.SUB(W9, W9, W10);
        else if (d.op == Op::And) code_.AND(W9, W9, W10);
        else if (d.op == Op::Or) code_.ORR(W9, W9, W10);
        else { code_.CMP(W9, W10); code_.CSEL(W9, W9, W10, d.op == Op::Max ? GT : LT); }
        storeC(); return;
    case Op::Shl: case Op::Sar:
        loadA(); loadB(); code_.AND(W10, W10, 31);
        if (d.op == Op::Shl) code_.LSLV(W9, W9, W10); else code_.ASRV(W9, W9, W10);
        storeC(); return;
    case Op::Mul: case Op::MulAcc: case Op::MulMem: {
        if (d.op == Op::MulMem) { emitLoadData(d.operand); code_.MOV(W10, W9); } else loadB();
        loadA(); code_.SXTW(X9, W9); code_.SXTW(X10, W10); code_.MUL(X9, X9, X10);
        if (d.shift) code_.ASR(X9, X9, d.shift);
        if (d.op == Op::MulAcc) { emitLoadReg(W10, d.c); code_.ADD(W9, W9.toW(), W10); }
        if (d.op == Op::MulMem) storeA(); else storeC(); return;
    }
    case Op::AddMem: case Op::SubMem:
        emitLoadData(d.operand); code_.MOV(W10, W9); loadA();
        if (d.op == Op::AddMem) code_.ADD(W9, W9, W10); else code_.SUB(W9, W9, W10);
        storeA(); return;
    case Op::AddMemSat: case Op::AddMemSatStore: case Op::AddSat: case Op::SubSat: {
        if (d.op == Op::AddMemSatStore) emitIncrementWrites();
        if (d.op == Op::AddMemSat || d.op == Op::AddMemSatStore) { emitLoadData(d.operand); code_.MOV(W10, W9); }
        else loadB();
        loadA(); code_.SXTW(X9, W9); code_.SXTW(X10, W10);
        if (d.op == Op::SubSat) code_.SUB(X9, X9, X10); else code_.ADD(X9, X9, X10);
        code_.MOV(X11, 0x7FFFFFFF); code_.CMP(X9, X11); code_.CSEL(X9, X9, X11, LE);
        code_.MOV(X11, 0xFFFFFFFF80000000ull); code_.CMP(X9, X11); code_.CSEL(X9, X9, X11, GE);
        if (d.op == Op::AddMemSatStore) { code_.MOV(W11, W9); emitStoreData(d.operand); }
        else if (d.op == Op::AddMemSat) storeA();
        else storeC();
        return;
    }
    case Op::Mov: loadB(); code_.MOV(W9, W10); storeA(); return;
    case Op::MovHighToLow: loadB(); code_.LSR(W9, W10, 16); storeA(); return;
    case Op::MovLowToHigh: loadB(); code_.LSL(W9, W10, 16); storeA(); return;
    case Op::MovHigh: loadB(); code_.AND(W9, W10, 0xFFFF0000u); storeA(); return;
    case Op::NegSat: case Op::AbsSat:
        loadA(); code_.SXTW(X9, W9);
        if (d.op == Op::AbsSat) { Label positive; code_.CMP(X9, 0); code_.B(GE, positive); code_.NEG(X9, X9); code_.l(positive); }
        else code_.NEG(X9, X9);
        code_.MOV(X10, 0x7FFFFFFF); code_.CMP(X9, X10); code_.CSEL(X9, X9, X10, LE);
        if (d.op == Op::AbsSat) storeC(); else storeA(); return;
    case Op::BranchGreater: case Op::BranchGreaterEqual: case Op::BranchEqual: case Op::BranchNotEqual:
    case Op::BranchHighEqualLow: case Op::BranchHighNotEqualLow: case Op::BranchLowEqual: case Op::BranchLowNotEqual:
    case Op::BranchBitSet: case Op::BranchBitClear: {
        loadA(); loadB();
        if (d.op == Op::BranchHighEqualLow || d.op == Op::BranchHighNotEqualLow) { code_.LSR(W9, W9, 16); code_.AND(W10, W10, 0xFFFF); }
        if (d.op == Op::BranchLowEqual || d.op == Op::BranchLowNotEqual) { code_.AND(W9, W9, 0xFFFF); code_.AND(W10, W10, 0xFFFF); }
        Cond cond = EQ;
        if (d.op == Op::BranchBitSet || d.op == Op::BranchBitClear) {
            code_.TST(W9, BitImm32{1u << (d.c & 31)}); cond = d.op == Op::BranchBitSet ? NE : EQ;
        } else {
            if (d.op == Op::BranchNotEqual || d.op == Op::BranchHighNotEqualLow || d.op == Op::BranchLowNotEqual) cond = NE;
            else if (d.op == Op::BranchGreater) cond = GT;
            else if (d.op == Op::BranchGreaterEqual) cond = GE;
            code_.CMP(W9, W10);
        }
        branch(cond); return;
    }
    case Op::JumpIfZero: case Op::JumpIfNotZero: {
        loadA(); code_.CMP(W9, 0); Label taken; code_.B(d.op == Op::JumpIfZero ? EQ : NE, taken);
        emitSetPcAndDispatch(next, false); code_.l(taken);
        emitSetPcAndDispatch(d.operand & 0xFFF, core_.jumpIdleDetection_ && d.operand <= in.pc); return;
    }
    case Op::Jump: emitSetPcAndDispatch(d.operand & 0xFFF, core_.jumpIdleDetection_ && d.operand <= in.pc); return;
    case Op::JumpLink: code_.MOV(W9, next); emitStoreReg(d.a, W9); emitSetPcAndDispatch(d.operand & 0xFFF, false); return;
    case Op::JumpRegister: loadA(); code_.AND(W9, W9, 0xFFF); emitDispatch(); return;
    case Op::Loop: {
        loadA(); code_.MOV(W10, W9); code_.SUB(W9, W9, 1); storeA(); code_.CMP(W10, 1);
        Label done; code_.B(EQ, done); emitSetPcAndDispatch(d.operand & 0xFFF, false);
        code_.l(done); emitSetPcAndDispatch(next, false); return;
    }
    case Op::Call: {
        code_.LDR(W9, X19, spOffset_); code_.ADD(W10, W9, 1); code_.STR(W10, X19, spOffset_);
        code_.AND(W9, W9, 15); code_.MOV(W10, next);
        code_.MOV(X11, static_cast<std::uint64_t>(stackOffset_)); code_.ADD(X11, X19, X11);
        code_.ADD(X11, X11, W9, AddSubExt::UXTW, 2); code_.STR(W10, X11);
        emitSetPcAndDispatch(d.operand & 0xFFF, false); return;
    }
    case Op::Return: {
        code_.LDR(W9, X19, spOffset_); code_.SUB(W9, W9, 1); code_.STR(W9, X19, spOffset_);
        code_.AND(W9, W9, 15); code_.MOV(X10, static_cast<std::uint64_t>(stackOffset_)); code_.ADD(X10, X19, X10);
        code_.ADD(X10, X10, W9, AddSubExt::UXTW, 2); code_.LDR(W9, X10); code_.AND(W9, W9, 0xFFF);
        emitDispatch(); return;
    }
    case Op::ReturnFromInterrupt:
        emitLoadReg(W9, 15); code_.LSR(W9, W9, 16); code_.AND(W9, W9, 0xFFF);
        code_.MOV(W10, 0); code_.STRB(W10, X19, interruptOffset_); emitLoadReg(W10, 15);
        code_.ORR(W10, W10, 1); emitStoreReg(15, W10); emitDispatch(); return;
    case Op::Biquad:
        code_.MOV(X0, X19); code_.MOV(W1, d.shift);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::biquadThunk)); return;
    case Op::SarImm:
        loadA(); code_.ASR(W9, W9, d.shift); storeC(); return;
    case Op::ShlImm:
        loadA(); code_.LSL(W9, W9, d.shift); storeC(); return;
    case Op::ShrImm:
        loadA(); code_.LSR(W9, W9, d.shift); storeC(); return;
    case Op::PortRead:
        code_.MOV(X0, X19); code_.MOV(W1, d.operand);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::portReadThunk));
        code_.MOV(W9, W0); storeA(); return;
    case Op::PortWrite:
        emitIncrementWrites(); loadA(); code_.MOV(X0, X19); code_.MOV(W1, d.operand); code_.MOV(W2, W9);
        emitCall(reinterpret_cast<const void*>(&DspJitArm64::portWriteThunk)); return;
    default: return;
    }
}

const void* DspJitArm64::compile(std::uint32_t start) {
    if (static_cast<std::size_t>(code_.offset()) + (64u << 10) > kCodeSize)
        flushAll();
    std::vector<Instruction> block;
    for (std::uint32_t pc = start; block.size() < kMaxBlock; pc = (pc + 1) & 0xFFF) {
        const Decoded d = decode(core_.mem_.program[pc]);
        if (d.op == Op::Invalid || !supports(d)) break;
        block.push_back({pc, d});
        if (isControlFlow(d.op)) break;
    }
    if (block.empty()) return nullptr;

    memory_.unprotect();
    code_.align(16);
    const auto startOffset = code_.offset();
    const auto* entry = reinterpret_cast<const void*>(reinterpret_cast<std::uintptr_t>(memory_.ptr()) + startOffset);
    currentBlockLength_ = static_cast<std::uint32_t>(block.size());
    const std::uint32_t length = currentBlockLength_;
    Label overBudget;

    code_.LDR(X9, X20); code_.ADD(X10, X9, length); code_.CMP(X10, X21); code_.B(HI, overBudget);

    bool controlFlow = false;
    for (const Instruction& in : block) {
        emitInstruction(in);
        if (isControlFlow(in.d.op)) { controlFlow = true; break; }
    }
    if (!controlFlow)
        emitSetPcAndDispatch((block.back().pc + 1) & 0xFFF, false);
    code_.l(overBudget);
    code_.MOV(W9, start); code_.STR(W9, X19, pcOffset_); emitExit(kBudget, false);

    const std::size_t bytes = static_cast<std::size_t>(code_.offset() - startOffset);
    memory_.invalidate(reinterpret_cast<std::uint32_t*>(const_cast<void*>(entry)), bytes);
    memory_.protect();
    blocks_[start] = entry;
    return entry;
}

void DspJitArm64::flushAll() {
    memory_.unprotect();
    code_.set_offset(0);
    blocks_.fill(nullptr);
    emitCommon();
    memory_.invalidate(memory_.ptr(), static_cast<std::size_t>(code_.offset()));
    memory_.protect();
}

std::uint64_t DspJitArm64::run(std::uint64_t budget) {
    if (disabled_) return core_.interpret(budget, budget);
    // MAP_JIT write protection is per thread on Apple platforms. DSP cores
    // move between workers, so restore execute mode on every worker before
    // entering a cached block.
    memory_.protect();
    std::uint64_t executed = 0;
    while (executed < budget) {
        if (core_.trapped_) return executed;
        if (flush_.exchange(false, std::memory_order_acq_rel)) flushAll();
        if (core_.pending_ != 0) {
            core_.takeInterrupt();
            if (core_.pending_ != 0) {
                executed += core_.interpret(budget - executed, 1);
                continue;
            }
        }
        const std::uint32_t pc = core_.pc_ & 0xFFF;
        const void* entry = blocks_[pc];
        if (!entry) {
            try {
                entry = compile(pc);
            } catch (...) {
                disabled_ = true;
                flushAll();
                memory_.protect();
                return executed + core_.interpret(budget - executed, budget - executed);
            }
        }
        if (!entry) {
            if (profileFallbacks_) {
                ++fallbackCounts_[static_cast<std::size_t>(core_.fetch(pc).op)];
                ++fallbackPcCounts_[pc];
            }
            executed += core_.interpret(budget - executed, 1);
            continue;
        }
        switch (enter_(&core_, entry, &executed, budget)) {
        case kIdle: return budget;
        case kBudget: return executed + core_.interpret(budget - executed, budget - executed);
        default: break;
        }
    }
    return executed;
}

}  // namespace hydra::dsp
