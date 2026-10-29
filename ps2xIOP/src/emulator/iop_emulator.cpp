#include "iop_emulator.h"
#include "imports/iop_cdvd.h"
#include "core/iop_cpu.h"
#include "imports/iop_heaplib.h"
#include "imports/iop_imports.h"
#include "imports/iop_intrman.h"
#include "imports/iop_ioman.h"
#include "imports/iop_libsd.h"
#include "core/iop_kernel.h"
#include "imports/iop_loadcore.h"
#include "core/iop_memory.h"
#include "services/iop_module_loader.h"
#include "services/iop_rpc.h"
#include "imports/iop_stdio.h"
#include "imports/iop_sysclib.h"
#include "imports/iop_sysmem.h"
#include "imports/iop_timrman.h"
#include "imports/iop_vblank.h"
#include "spu2/iop_spu2.h"
#include "iop_emulator_const.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <map>
#include <optional>
#include <span>
#include <sstream>
#include <utility>

namespace ps2x::iop::detail
{
    namespace
    {
        constexpr uint32_t kRamSize = IopMemory::RamSize;
        constexpr uint32_t kKernelHeapBase = IopMemory::HeapBase;
        constexpr uint32_t kKernelHeapLimit = IopMemory::HeapLimit;
        constexpr uint32_t kCallStackBase = kKernelHeapLimit;
        constexpr uint32_t kCallStackLimit = 0x001FFF00u;
        constexpr uint32_t kCallStackSize = 0x2000u;
        constexpr uint32_t kCallStackCapacity = (kCallStackLimit - kCallStackBase) / kCallStackSize;
        constexpr uint64_t kCdvdCompletionCycles = 128u;
        // The SPU2 is brought up to the current IOP cycle at every libsd call and at least every 64 samples
        // (1.3 ms of guest time); its output reaches the host in blocks of 256 frames or more.
        constexpr uint64_t kSpu2SyncCycles = 64u * IopSpu2::IopCyclesPerSample;
        constexpr size_t kSpu2OutputFrames = 256u;

        uint32_t physicalAddress(uint32_t address)
        {
            return IopMemory::physicalAddress(address);
        }

        int32_t sign16(uint32_t value)
        {
            return static_cast<int16_t>(value & 0xFFFFu);
        }

        bool iequals(std::string_view lhs, std::string_view rhs)
        {
            if (lhs.size() != rhs.size())
                return false;
            for (size_t i = 0; i < lhs.size(); ++i)
            {
                if (std::tolower(static_cast<unsigned char>(lhs[i])) !=
                    std::tolower(static_cast<unsigned char>(rhs[i])))
                    return false;
            }
            return true;
        }

    }

    class IopEmulator::Impl final : public IopGuestExecutor
    {
    public:
        using CpuState = IopCpuState;

        struct Module
        {
            int id = 0;
            std::string path;
            std::string name;
            uint32_t base = 0;
            uint32_t size = 0;
            uint32_t entry = 0;
            uint32_t gp = 0;
            bool resident = false;
        };

        struct GuestCallback
        {
            uint32_t function = 0;
            uint32_t gp = 0;
        };

        struct ScheduledGuestCallback
        {
            uint32_t function = 0u;
            uint32_t gp = 0u;
            uint32_t argument = 0u;
        };

        explicit Impl(IopHost &hostRef)
            : host(hostRef),
              sysmem(host, memory),
              kernel(memory),
              cdvd(host, memory, kernel),
              vblank(kernel),
              rpc(host, memory, kernel),
              sysclib(memory),
              stdio(host, memory),
              heaplib(memory),
              intrman(memory),
              timrman(),
              ioman(memory),
              libsd(host, memory, spu2),
              cpuCore(memory),
              imports(memory),
              loadcore(memory, imports)
        {
            reset();
        }

        void reset()
        {
            memory.reset();
            kernel.reset();
            modules.clear();
            imports.reset();
            rpc.reset();
            cdvd.reset();
            intrman.reset();
            timrman.reset();
            ioman.reset();
            spu2.reset();
            libsd.reset();
            spu2Epoch = 0;
            spu2LastSync = 0;
            audioOut.clear();
            pendingDmaInterrupts.clear();
            pendingGuestCallbacks.clear();
            nextModuleId = 1;
            moduleCursor = kModuleLoadBase;
            totalCycles = 0;
            eeClockCycles = 0;
            totalInstructions = 0;
            eeCycleCarry = 0;
            activeCpu = nullptr;
            lastError.clear();
            servicingDmaInterrupts = false;
            servicingGuestCallbacks = false;
            callDepth = 0u;
            secrMcCommandHandler = {};
            secrMcDevIdHandler = {};
            checkKelfPathCallback = {};
        }

        uint8_t read8(uint32_t address) const
        {
            return memory.read8(address);
        }

        uint16_t read16(uint32_t address) const
        {
            return memory.read16(address);
        }

        uint32_t read32(uint32_t address) const
        {
            return memory.read32(address);
        }

        void write8(uint32_t address, uint8_t value)
        {
            memory.write8(address, value);
            schedulePendingDma();
        }

        void write16(uint32_t address, uint16_t value)
        {
            memory.write16(address, value);
            schedulePendingDma();
        }

        void write32(uint32_t address, uint32_t value)
        {
            memory.write32(address, value);
            schedulePendingDma();
        }

        void schedulePendingDma()
        {
            if (const auto dma = memory.takeDmaStart())
                pendingDmaInterrupts[dma->irq] = totalCycles + dma->delayCycles;
        }

        bool readRam(uint32_t address, void *destination, size_t size) const
        {
            return memory.readRam(address, destination, size);
        }

        bool writeRam(uint32_t address, const void *source, size_t size)
        {
            return memory.writeRam(address, source, size);
        }

        bool zeroRam(uint32_t address, size_t size)
        {
            return memory.zeroRam(address, size);
        }

        bool isHardwareAddress(uint32_t phys) const
        {
            return memory.isHardwareAddress(phys);
        }

        uint32_t allocate(uint32_t size, uint32_t alignment = 16u, std::optional<uint32_t> fixed = std::nullopt)
        {
            return memory.allocate(size, alignment, fixed);
        }

        bool freeAllocation(uint32_t address)
        {
            return memory.freeAllocation(address);
        }

        void log(LogLevel level, std::string_view text)
        {
            host.log(level, text);
        }

        bool checkInterrupt(CpuState &cpu)
        {
            const uint32_t status = cpu.cop0[12];
            if ((status & 1u) == 0u)
                return false;
            if ((status & 0x2u) != 0u)
                return false;
            const bool pending = memory.interruptControl() != 0u && (memory.interruptStatus() & memory.interruptMask()) != 0u;
            if (!pending)
                return false;
            cpu.cop0[13] |= 0x400u;
            cpuCore.raiseException(cpu, 0u, cpu.pc, false);
            return true;
        }

        enum class ImportDisposition
        {
            Handled,
            JumpToGuest,
            Missing,
        };

        ImportDisposition dispatchImport(const IopImportCall &call, CpuState &cpu)
        {
            const uint32_t a0 = cpu.gpr[4];
            auto setV0 = [&](uint32_t value)
            {
                cpu.gpr[2] = value;
            };

            if (iequals(call.library, "sysmem") && sysmem.dispatchImport(call.ordinal, cpu))
                return ImportDisposition::Handled;

            if (iequals(call.library, "cdvdman") && cdvd.dispatchImport(call.ordinal, cpu))
            {
                if (const auto callback = cdvd.takeCompletionCallback())
                {
                    pendingGuestCallbacks.emplace(
                        totalCycles + kCdvdCompletionCycles,
                        ScheduledGuestCallback{
                            callback->address,
                            callback->gp,
                            callback->reason,
                        });
                }
                return ImportDisposition::Handled;
            }

            if (iequals(call.library, "loadcore") && loadcore.dispatchImport(call.ordinal, cpu))
                return ImportDisposition::Handled;

            if (iequals(call.library, "thbase") || iequals(call.library, "threadman"))
            {
                return kernel.dispatchThreadImport(call.ordinal, cpu, totalCycles)
                           ? ImportDisposition::Handled
                           : ImportDisposition::Missing;
            }
            if (iequals(call.library, "thsemap"))
            {
                return kernel.dispatchSemaphoreImport(call.ordinal, cpu)
                           ? ImportDisposition::Handled
                           : ImportDisposition::Missing;
            }
            if (iequals(call.library, "thevent"))
            {
                return kernel.dispatchEventImport(call.ordinal, cpu)
                           ? ImportDisposition::Handled
                           : ImportDisposition::Missing;
            }
            if (iequals(call.library, "sifcmd"))
            {
                return rpc.dispatchSifCmdImport(call.ordinal, cpu)
                           ? ImportDisposition::Handled
                           : ImportDisposition::Missing;
            }
            if (iequals(call.library, "intrman") && intrman.dispatchImport(call.ordinal, cpu, *this))
                return ImportDisposition::Handled;
            if (iequals(call.library, "secrman"))
            {
                switch (call.ordinal)
                {
                case 4: // SecrSetMcCommandHandler
                    secrMcCommandHandler = {a0, cpu.gpr[28]};
                    setV0(0);
                    return ImportDisposition::Handled;
                case 5: // SecrSetMcDevIDHandler
                    secrMcDevIdHandler = {a0, cpu.gpr[28]};
                    setV0(0);
                    return ImportDisposition::Handled;
                default:
                    break;
                }
            }
            if (iequals(call.library, "modload") && call.ordinal == 13u)
            {
                checkKelfPathCallback = {a0, cpu.gpr[28]};
                setV0(0);
                return ImportDisposition::Handled;
            }
            if (iequals(call.library, "ioman") && ioman.dispatchImport(call.ordinal, cpu, *this))
                return ImportDisposition::Handled;
            if (iequals(call.library, "sifman"))
            {
                return rpc.dispatchSifManImport(call.ordinal, cpu)
                           ? ImportDisposition::Handled
                           : ImportDisposition::Missing;
            }
            if (iequals(call.library, "vblank") && vblank.dispatchImport(call.ordinal, cpu, totalCycles))
                return ImportDisposition::Handled;
            if (iequals(call.library, "timrman") && timrman.dispatchImport(call.ordinal, cpu, totalCycles))
                return ImportDisposition::Handled;
            if (iequals(call.library, "dmacman"))
            {
                setV0(0);
                return ImportDisposition::Handled;
            }
            if (iequals(call.library, "stdio") && stdio.dispatchImport(call.ordinal, cpu))
                return ImportDisposition::Handled;
            if (iequals(call.library, "sysclib"))
            {
                return sysclib.dispatchImport(call.ordinal, cpu)
                           ? ImportDisposition::Handled
                           : ImportDisposition::Missing;
            }
            if (iequals(call.library, "heaplib") && heaplib.dispatchImport(call.ordinal, cpu))
                return ImportDisposition::Handled;
            if (iequals(call.library, "libsd"))
            {
                syncSpu2();
                std::optional<IopLibsd::Callback> callback;
                const std::array<uint32_t, 4> arguments{cpu.gpr[4], cpu.gpr[5], cpu.gpr[6], cpu.gpr[7]};
                if (call.ordinal == 7u && ((cpu.gpr[4] >> 8u) & 0xFFu) == 0x15u) // KON: keep each voice's last peak
                    for (uint32_t v = 0; v < IopSpu2::VoicesPerCore; ++v)
                        peakBeforeKeyOn[(cpu.gpr[4] & 1u) * IopSpu2::VoicesPerCore + v] = spu2.voicePeak(cpu.gpr[4] & 1u, v);
                (void)libsd.dispatchImport(call.ordinal, cpu, totalCycles, callback);
                // PS2X_LIBSD_TRACE=<n>: log the first n calls of each libsd function.
                static const uint64_t traceLimit = []
                {
                    const char *value = std::getenv("PS2X_LIBSD_TRACE");
                    return value ? std::strtoull(value, nullptr, 0) : 0ull;
                }();
                if (traceLimit != 0u && libsd.calls(call.ordinal) <= traceLimit)
                {
                    std::ostringstream out;
                    out << "[IOP:libsd] #" << std::dec << call.ordinal << '(' << std::hex << "0x" << arguments[0]
                        << ", 0x" << arguments[1] << ", 0x" << arguments[2] << ", 0x" << arguments[3] << ") -> 0x"
                        << cpu.gpr[2] << " ra=0x" << cpu.gpr[31] << " sample=" << std::dec << spu2.samplesMixed();
                    log(LogLevel::Debug, out.str());
                }
                // PS2X_SPU2_KON_TRACE=<n>: describe the first n voices keyed on (sceSdSetSwitch KON), and the first n
                // voice transfers.
                static const uint64_t konTraceLimit = []
                {
                    const char *value = std::getenv("PS2X_SPU2_KON_TRACE");
                    return value ? std::strtoull(value, nullptr, 0) : 0ull;
                }();
                if (konTraceLimit != 0u)
                {
                    static uint64_t traced = 0u, transfers = 0u;
                    if (call.ordinal == 7u && ((arguments[0] >> 8u) & 0xFFu) == 0x15u)
                    {
                        for (uint32_t v = 0; v < IopSpu2::VoicesPerCore && traced < konTraceLimit; ++v)
                        {
                            if ((arguments[1] & (1u << v)) == 0u)
                                continue;
                            ++traced;
                            std::ostringstream out;
                            out << "[IOP:spu2] KON #" << traced << " sample " << spu2.samplesMixed() << " ra=0x"
                                << std::hex << cpu.gpr[31] << std::dec << ": "
                                << spu2.describeVoice(arguments[0] & 1u, v) << " | previous sound's peak "
                                << peakBeforeKeyOn[(arguments[0] & 1u) * IopSpu2::VoicesPerCore + v];
                            log(LogLevel::Info, out.str());
                        }
                    }
                    else if (call.ordinal == 17u && transfers < konTraceLimit)
                    {
                        ++transfers;
                        std::ostringstream out;
                        out << "[IOP:spu2] VoiceTrans #" << transfers << " sample " << spu2.samplesMixed() << " ch "
                            << (arguments[0] & 1u) << " mode 0x" << std::hex << arguments[1] << " IOP 0x" << arguments[2]
                            << " -> SPU 0x" << arguments[3] << " size 0x" << memory.read32(cpu.gpr[29] + 16u)
                            << " -> 0x" << cpu.gpr[2];
                        log(LogLevel::Info, out.str());
                    }
                }
                if (call.ordinal == 4u)
                {
                    // sceSdInit restarts the sample clock with the SPU2 it has reset.
                    spu2Epoch = totalCycles - spu2.samplesMixed() * IopSpu2::IopCyclesPerSample;
                    spu2LastSync = totalCycles;
                }
                if (callback)
                    pendingGuestCallbacks.emplace(callback->cycle,
                                                  ScheduledGuestCallback{callback->function, callback->gp, callback->argument});
                return ImportDisposition::Handled;
            }

            const uint32_t target = imports.resolve(call.library, call.ordinal, call.version);
            if (target != 0u)
            {
                cpu.pc = target;
                cpu.branchPending = false;
                return ImportDisposition::JumpToGuest;
            }

            std::ostringstream out;
            out << "[IOP] unhandled import " << call.library << ':' << call.ordinal
                << " version=0x" << std::hex << call.version << " pc=0x" << cpu.pc;
            log(LogLevel::Warning, out.str());
            setV0(0);
            return ImportDisposition::Missing;
        }

        bool step(CpuState &cpu)
        {
            if (cpu.stopped)
                return false;
            if (cpu.pc == kThreadReturnSentinel || cpu.pc == kCallReturnSentinel)
            {
                cpu.stopped = true;
                return false;
            }
            if (physicalAddress(cpu.pc) >= kRamSize)
            {
                std::ostringstream out;
                out << "[IOP] execution outside RAM pc=0x" << std::hex << cpu.pc;
                log(LogLevel::Error, out.str());
                cpu.stopped = true;
                return false;
            }
            if (checkInterrupt(cpu))
                return true;

            if (const auto import = imports.decode(cpu.pc))
            {
                const ImportDisposition disposition = dispatchImport(*import, cpu);
                ++totalInstructions;
                ++totalCycles;
                if (disposition == ImportDisposition::JumpToGuest)
                    return true;
                cpu.pc = cpu.gpr[31];
                cpu.branchPending = false;
                return !cpu.stopped;
            }

            const bool running = cpuCore.executeInstruction(cpu);
            schedulePendingDma();
            ++totalInstructions;
            ++totalCycles;
            return running;
        }

        uint32_t runCpu(CpuState &cpu, uint32_t instructionBudget)
        {
            CpuState *previous = activeCpu;
            activeCpu = &cpu;
            const uint64_t start = totalInstructions;
            while (!cpu.stopped && !cpu.yielded && totalInstructions - start < instructionBudget)
            {
                if (!step(cpu))
                    break;
                if (!servicingDmaInterrupts && !pendingDmaInterrupts.empty())
                    servicePendingDmaInterrupts();
                if (!servicingGuestCallbacks && !pendingGuestCallbacks.empty())
                    servicePendingGuestCallbacks();
            }
            activeCpu = previous;
            return static_cast<uint32_t>(totalInstructions - start);
        }

        uint32_t callFunction(uint32_t address,
                              uint32_t a0,
                              uint32_t a1,
                              uint32_t a2,
                              uint32_t a3,
                              uint32_t gp,
                              uint32_t budget = kMaxCallInstructions)
        {
            struct CallDepthGuard
            {
                uint32_t &depth;
                ~CallDepthGuard() { --depth; }
            };

            const uint32_t depth = callDepth++;
            const CallDepthGuard depthGuard{callDepth};
            CpuState cpu{};
            cpu.pc = address;
            cpu.gpr[4] = a0;
            cpu.gpr[5] = a1;
            cpu.gpr[6] = a2;
            cpu.gpr[7] = a3;
            cpu.gpr[28] = gp;
            if (depth < kCallStackCapacity)
            {
                const uint32_t stackTop = kCallStackLimit - depth * kCallStackSize;
                cpu.gpr[29] = stackTop - 32u;
            }
            else if (activeCpu && activeCpu->gpr[29] > kCallStackBase + kStackGuardBytes)
            {
                // Extremely deep re-entrancy borrows unused space below the
                // suspended caller's live frame. Stack growth remains away
                // from the caller, so its saved registers stay intact.
                cpu.gpr[29] = (activeCpu->gpr[29] - kStackGuardBytes) & ~15u;
            }
            else
            {
                cpu.gpr[29] = kCallStackBase - 32u;
            }
            cpu.gpr[31] = kCallReturnSentinel;
            runCpu(cpu, budget);
            return cpu.gpr[2];
        }

        uint32_t executeGuestFunction(uint32_t address,
                                      uint32_t a0,
                                      uint32_t a1,
                                      uint32_t a2,
                                      uint32_t a3,
                                      uint32_t gp) override
        {
            return callFunction(address, a0, a1, a2, a3, gp);
        }

        uint32_t executeGuestFunctionWithBudget(uint32_t address,
                                                uint32_t a0,
                                                uint32_t a1,
                                                uint32_t a2,
                                                uint32_t a3,
                                                uint32_t gp,
                                                uint32_t instructionBudget) override
        {
            return callFunction(address, a0, a1, a2, a3, gp, instructionBudget);
        }

        // Not that good to use exception handling for control flow but will do for now
        void servicePendingDmaInterrupts()
        {
            if (servicingDmaInterrupts || pendingDmaInterrupts.empty())
                return;

            servicingDmaInterrupts = true;

            std::vector<int> completed;
            for (auto it = pendingDmaInterrupts.begin(); it != pendingDmaInterrupts.end();)
            {
                if (it->second > totalCycles)
                {
                    ++it;
                    continue;
                }
                completed.push_back(it->first);
                it = pendingDmaInterrupts.erase(it);
            }
            try
            {
                for (const int irq : completed)
                    (void)intrman.dispatchInterrupt(irq, *this);
            }
            catch (...)
            {
                servicingDmaInterrupts = false;
                throw;
            }
            servicingDmaInterrupts = false;
        }

        // Brings the SPU2 up to the current IOP cycle and hands full output blocks to the host.
        void syncSpu2()
        {
            if (!libsd.initialized())
                return;
            spu2LastSync = totalCycles;
            const uint64_t target = (totalCycles - spu2Epoch) / IopSpu2::IopCyclesPerSample;
            if (spu2.samplesMixed() >= target)
                return;
            const auto mixStart = std::chrono::steady_clock::now();
            while (spu2.samplesMixed() < target)
            {
                const uint64_t frames = std::min<uint64_t>(target - spu2.samplesMixed(), 4096u);
                spu2.run(static_cast<uint32_t>(frames), audioOut);
                if (audioOut.size() >= kSpu2OutputFrames * 2u)
                {
                    const auto outputStart = std::chrono::steady_clock::now();
                    host.audioOutput(audioOut.data(), audioOut.size() / 2u);
                    audioOut.clear();
                    const uint64_t ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                                  std::chrono::steady_clock::now() - outputStart)
                                                                  .count());
                    hostOutputNs += ns;
                    hostOutputMaxNs = std::max(hostOutputMaxNs, ns);
                }
            }
            const uint64_t ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                          std::chrono::steady_clock::now() - mixStart)
                                                          .count());
            hostSpu2Ns += ns;
            hostSpu2MaxNs = std::max(hostSpu2MaxNs, ns);
        }

        void servicePendingGuestCallbacks()
        {
            if (servicingGuestCallbacks || pendingGuestCallbacks.empty())
                return;

            std::vector<ScheduledGuestCallback> callbacks;
            for (auto it = pendingGuestCallbacks.begin(); it != pendingGuestCallbacks.end();)
            {
                if (it->first > totalCycles)
                    break;
                callbacks.push_back(it->second);
                it = pendingGuestCallbacks.erase(it);
            }
            if (callbacks.empty())
                return;

            servicingGuestCallbacks = true;
            try
            {
                for (const ScheduledGuestCallback &callback : callbacks)
                {
                    if (callback.function != 0u)
                    {
                        (void)callFunction(callback.function,
                                           callback.argument,
                                           0u,
                                           0u,
                                           0u,
                                           callback.gp,
                                           100000u);
                    }
                }
            }
            catch (...)
            {
                servicingGuestCallbacks = false;
                throw;
            }
            servicingGuestCallbacks = false;
        }

        void runCycles(uint64_t cycles) noexcept
        {
            try
            {
                const uint64_t target = totalCycles + cycles;
                if (totalCycles < idleUntil && target <= idleUntil)
                {
                    totalCycles = target; // what the loop below would do: nothing is due, nothing can run
                    return;
                }
                idleUntil = 0;
                const auto runStart = std::chrono::steady_clock::now();
                ++hostActiveRuns;
                while (totalCycles < target)
                {
                    servicePendingDmaInterrupts();
                    servicePendingGuestCallbacks();
                    timrman.serviceDue(totalCycles, *this);
                    IopThread *next = kernel.beginNextReady(totalCycles);
                    if (!next)
                    {
                        uint64_t nextEvent = kernel.nextWakeCycle(UINT64_MAX);
                        for (const auto &[irq, completionCycle] : pendingDmaInterrupts)
                            nextEvent = std::min(nextEvent, completionCycle);
                        if (!pendingGuestCallbacks.empty())
                            nextEvent = std::min(nextEvent, pendingGuestCallbacks.begin()->first);
                        nextEvent = timrman.nextEventCycle(nextEvent);
                        totalCycles = std::max(totalCycles + 1u, std::min(target, nextEvent));
                        if (totalCycles >= target)
                            idleUntil = nextEvent;
                        continue;
                    }
                    const uint64_t before = totalCycles;
                    runCpu(next->cpu, static_cast<uint32_t>(std::min<uint64_t>(kDefaultSlice, target - totalCycles)));
                    kernel.endTimeslice(*next, kThreadReturnSentinel);
                    if (totalCycles == before)
                        ++totalCycles;
                }
                hostRunNs += static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                       std::chrono::steady_clock::now() - runStart)
                                                       .count());
            }
            catch (...)
            {
                // Runtime scheduling must never throw through EeScheduler::accountCycles().
            }
        }

        ModuleLoadResult loadImage(std::string path, std::span<const uint8_t> image, const void *arguments, uint32_t argumentSize)
        {
            ModuleLoadResult result{true, -1, -1};
            const IopImageLoadResult loaded = IopModuleLoader::load(image, memory, moduleCursor);
            moduleCursor = loaded.nextModuleCursor;
            if (!loaded)
            {
                if (loaded.error == IopImageLoadError::InvalidElf)
                    log(LogLevel::Error, "[IOP] rejected invalid/non-MIPS IRX ELF");
                else if (loaded.error == IopImageLoadError::ArenaExhausted)
                    log(LogLevel::Error, "[IOP] module arena exhausted");
                return result;
            }
            if (!loaded.relocationsComplete)
                log(LogLevel::Warning, "[IOP] one or more IRX relocations were unsupported");

            Module module;
            module.id = nextModuleId++;
            module.path = std::move(path);
            const size_t slash = module.path.find_last_of("/\\:");
            module.name = slash == std::string::npos ? module.path : module.path.substr(slash + 1u);
            module.base = loaded.base;
            module.size = loaded.size;
            module.entry = loaded.entry;
            module.gp = loaded.gp;

            // A module starts as start(argc, argv): argv[0] is the module path, then each NUL-terminated string of
            // the argument block, then a null pointer.
            std::vector<std::string> argv{module.path};
            if (arguments && argumentSize)
            {
                const char *text = static_cast<const char *>(arguments);
                size_t begin = 0u;
                for (size_t i = 0; i <= argumentSize; ++i)
                {
                    if (i == argumentSize ? i > begin : text[i] == '\0')
                    {
                        argv.emplace_back(text + begin, i - begin);
                        begin = i + 1u;
                    }
                }
            }
            uint32_t stringBytes = 0u;
            for (const std::string &arg : argv)
                stringBytes += static_cast<uint32_t>(arg.size()) + 1u;
            const uint32_t pointerOffset = (stringBytes + 3u) & ~3u;
            const uint32_t block = allocate(pointerOffset + 4u * static_cast<uint32_t>(argv.size() + 1u), 16u);
            uint32_t argvAddress = 0u;
            if (block)
            {
                argvAddress = block + pointerOffset;
                uint32_t cursor = block;
                for (size_t i = 0; i < argv.size(); ++i)
                {
                    writeRam(cursor, argv[i].c_str(), argv[i].size() + 1u);
                    write32(argvAddress + 4u * static_cast<uint32_t>(i), cursor);
                    cursor += static_cast<uint32_t>(argv[i].size()) + 1u;
                }
                write32(argvAddress + 4u * static_cast<uint32_t>(argv.size()), 0u);
            }
            const uint32_t startResult = callFunction(module.entry, block ? static_cast<uint32_t>(argv.size()) : 0u,
                                                      argvAddress, 0u, 0u, module.gp);
            if (block)
                freeAllocation(block);
            module.resident = startResult == 0u || startResult == 2u;
            result.moduleId = module.id;
            result.startResult = static_cast<int32_t>(startResult);
            modules[module.id] = std::move(module);

            std::ostringstream out;
            out << "[IOP] loaded IRX id=" << result.moduleId
                << " entry=0x" << std::hex << modules[result.moduleId].entry
                << " base=0x" << modules[result.moduleId].base
                << " start=" << std::dec << result.startResult;
            log(LogLevel::Info, out.str());
            return result;
        }

        ModuleLoadResult loadModule(std::string_view path, const void *arguments, uint32_t argumentSize)
        {
            std::vector<uint8_t> image;
            if (!IopModuleLoader::readWholeHostFile(host, path, image))
            {
                log(LogLevel::Warning, std::string("[IOP] failed to open IRX '") + std::string(path) + "'");
                return {true, -1, -1};
            }
            return loadImage(std::string(path), image, arguments, argumentSize);
        }

        ModuleLoadResult loadModuleBuffer(uint32_t guestAddress, const void *arguments, uint32_t argumentSize)
        {
            std::vector<uint8_t> image;
            if (!IopModuleLoader::readElfFromGuest(host, guestAddress, image))
                return {true, -1, -1};
            std::ostringstream tag;
            tag << "buffer@0x" << std::hex << guestAddress;
            return loadImage(tag.str(), image, arguments, argumentSize);
        }

        bool stopModule(int32_t moduleId, int32_t *result)
        {
            auto it = modules.find(moduleId);
            if (it == modules.end())
                return false;
            // A removable IRX normally exposes a stop entry through module metadata. We do not guess it; terminate owned execution and release the image cleanly.
            kernel.terminateThreadsInRange(it->second.base, it->second.size);
            rpc.removeServersInRange(it->second.base, it->second.size);
            imports.eraseRange(it->second.base, it->second.size);
            modules.erase(it);
            kernel.cleanupDeadThreads();
            if (result)
                *result = 0;
            return true;
        }

        IopHost &host;
        IopMemory memory;
        IopSysmem sysmem;
        IopKernel kernel;
        IopCdvd cdvd;
        IopVblank vblank;
        IopRpcBridge rpc;
        IopSysclib sysclib;
        IopStdio stdio;
        IopHeaplib heaplib;
        IopIntrman intrman;
        IopTimrman timrman;
        IopIoman ioman;
        IopSpu2 spu2;
        IopLibsd libsd;
        uint64_t spu2Epoch = 0;    // IOP cycle of SPU2 sample 0
        uint64_t spu2LastSync = 0; // IOP cycle of the last SPU2 sync
        std::vector<int16_t> audioOut;
        std::array<int32_t, 2u * IopSpu2::VoicesPerCore> peakBeforeKeyOn{}; // PS2X_SPU2_KON_TRACE diagnostics
        IopCpuCore cpuCore;
        IopImportRegistry imports;
        IopLoadcore loadcore;
        std::map<int, Module> modules;
        std::map<int, uint64_t> pendingDmaInterrupts;
        std::multimap<uint64_t, ScheduledGuestCallback> pendingGuestCallbacks;
        uint32_t nextModuleId = 1;
        uint32_t moduleCursor = kModuleLoadBase;
        uint64_t totalCycles = 0;
        // IOP cycles the EE has granted so far (EE cycles / 8). Code run synchronously for the EE (RPC server
        // functions, module start) also advances totalCycles, so the IOP only runs while it is behind this clock;
        // otherwise every such call would push IOP time, and the SPU2 sample clock with it, ahead of the EE.
        uint64_t eeClockCycles = 0;
        uint64_t totalInstructions = 0;
        uint64_t eeCycleCarry = 0;
        // runCycles: no thread is ready and no event is due before this cycle (0: unknown). Only time has passed since
        // it was computed: every entry point that can change IOP state clears it.
        uint64_t idleUntil = 0;
        uint64_t hostRunNs = 0;     // host time in runCycles slices that were not skipped as idle
        uint64_t hostActiveRuns = 0;
        uint64_t hostSpu2Ns = 0;    // host time mixing SPU2 output, including handing it to the host
        uint64_t hostSpu2MaxNs = 0; // longest single SPU2 sync since the last hostTiming() call
        uint64_t hostOutputNs = 0;  // of hostSpu2Ns, time inside IopHost::audioOutput
        uint64_t hostOutputMaxNs = 0;
        CpuState *activeCpu = nullptr;
        std::string lastError;
        bool servicingDmaInterrupts = false;
        bool servicingGuestCallbacks = false;
        uint32_t callDepth = 0u;
        GuestCallback secrMcCommandHandler;
        GuestCallback secrMcDevIdHandler;
        GuestCallback checkKelfPathCallback;
    };

    IopEmulator::IopEmulator(IopHost &host)
        : m_impl(std::make_unique<Impl>(host))
    {
    }

    IopEmulator::~IopEmulator() = default;

    void IopEmulator::reset()
    {
        m_impl->idleUntil = 0;
        m_impl->reset();
    }

    ModuleLoadResult IopEmulator::loadModule(std::string_view path, const void *arguments, uint32_t argumentSize)
    {
        m_impl->idleUntil = 0;
        return m_impl->loadModule(path, arguments, argumentSize);
    }

    ModuleLoadResult IopEmulator::loadModuleBuffer(uint32_t guestAddress, const void *arguments, uint32_t argumentSize)
    {
        m_impl->idleUntil = 0;
        return m_impl->loadModuleBuffer(guestAddress, arguments, argumentSize);
    }

    bool IopEmulator::stopModule(int32_t moduleId, int32_t *result)
    {
        m_impl->idleUntil = 0;
        return m_impl->stopModule(moduleId, result);
    }

    void IopEmulator::runEeCycles(uint64_t eeCycles) noexcept
    {
        const uint64_t total = m_impl->eeCycleCarry + eeCycles;
        m_impl->eeClockCycles += total / 8u;
        m_impl->eeCycleCarry = total % 8u;
        if (m_impl->totalCycles < m_impl->eeClockCycles)
            m_impl->runCycles(m_impl->eeClockCycles - m_impl->totalCycles);
        if (m_impl->libsd.initialized() && m_impl->totalCycles - m_impl->spu2LastSync >= kSpu2SyncCycles)
        {
            try
            {
                m_impl->syncSpu2();
            }
            catch (...)
            {
                // Runtime scheduling must never throw through EeScheduler::accountCycles().
            }
        }
    }

    uint64_t IopEmulator::idleEeCycles() const noexcept
    {
        // The last IOP cycle the EE clock may reach with runEeCycles() changing nothing but the clocks: up to where
        // the IOP already is, or up to its next event while it idles; and short of the next SPU2 sync.
        const Impl &impl = *m_impl;
        uint64_t last = impl.totalCycles < impl.idleUntil ? impl.idleUntil : impl.totalCycles;
        if (impl.libsd.initialized())
        {
            const uint64_t syncDue = impl.spu2LastSync + kSpu2SyncCycles;
            if (impl.totalCycles >= syncDue)
                return 0u;
            last = std::min(last, syncDue - 1u);
        }
        if (last < impl.eeClockCycles)
            return 0u;
        const uint64_t span = std::min<uint64_t>(last - impl.eeClockCycles, 1ull << 32); // idleUntil may be "never"
        return (span + 1u) * 8u - impl.eeCycleCarry;
    }

    RpcResult IopEmulator::handleRpc(const RpcRequest &request)
    {
        m_impl->idleUntil = 0;
        return m_impl->rpc.handleRpc(request, *m_impl);
    }

    bool IopEmulator::hasRpcServer(uint32_t sid) const noexcept
    {
        return m_impl->rpc.hasServer(sid);
    }

    void IopEmulator::onSifTransfer(const SifTransfer &transfer)
    {
        m_impl->idleUntil = 0;
        m_impl->rpc.onSifTransfer(transfer);
    }

    uint32_t IopEmulator::allocateMemory(uint32_t size, uint32_t alignment)
    {
        m_impl->idleUntil = 0;
        return m_impl->memory.allocate(size, alignment);
    }

    bool IopEmulator::freeMemory(uint32_t address)
    {
        m_impl->idleUntil = 0;
        return m_impl->memory.freeAllocation(address);
    }

    bool IopEmulator::readMemory(uint32_t address, void *destination, size_t size) const
    {
        return isMemoryRange(address, size) &&
               m_impl->memory.readRam(address, destination, size);
    }

    bool IopEmulator::writeMemory(uint32_t address, const void *source, size_t size)
    {
        m_impl->idleUntil = 0;
        return isMemoryRange(address, size) &&
               m_impl->memory.writeRam(address, source, size);
    }

    bool IopEmulator::zeroMemory(uint32_t address, size_t size)
    {
        m_impl->idleUntil = 0;
        return isMemoryRange(address, size) &&
               m_impl->memory.zeroRam(address, size);
    }

    bool IopEmulator::isMemoryRange(uint32_t address, size_t size) const
    {
        const bool physicalSegment = address < IopMemory::RamSize;
        const bool cachedSegment = address >= 0x80000000u && address < 0x80200000u;
        const bool uncachedSegment = address >= 0xA0000000u && address < 0xA0200000u;
        if (!physicalSegment && !cachedSegment && !uncachedSegment)
            return false;
        const uint32_t physical = IopMemory::physicalAddress(address);
        return physical <= IopMemory::RamSize && size <= IopMemory::RamSize - physical;
    }

    uint64_t IopEmulator::cycles() const noexcept
    {
        return m_impl->totalCycles;
    }

    uint64_t IopEmulator::instructions() const noexcept
    {
        return m_impl->totalInstructions;
    }

    void IopEmulator::hostTiming(HostTiming &timing) const noexcept
    {
        timing.runNs = m_impl->hostRunNs;
        timing.activeRuns = m_impl->hostActiveRuns;
        timing.spu2Ns = m_impl->hostSpu2Ns;
        timing.spu2MaxNs = std::exchange(m_impl->hostSpu2MaxNs, 0u);
        timing.outputNs = m_impl->hostOutputNs;
        timing.outputMaxNs = std::exchange(m_impl->hostOutputMaxNs, 0u);
        timing.spu2Samples = m_impl->spu2.samplesMixed();
    }

    uint32_t IopEmulator::loadedModuleCount() const noexcept
    {
        return static_cast<uint32_t>(m_impl->modules.size());
    }

    uint32_t IopEmulator::threadCount() const noexcept
    {
        return static_cast<uint32_t>(m_impl->kernel.threadCount());
    }

    uint32_t IopEmulator::rpcServerCount() const noexcept
    {
        return static_cast<uint32_t>(m_impl->rpc.serverCount());
    }

}
