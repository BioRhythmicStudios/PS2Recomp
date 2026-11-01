#pragma once

#include "ps2x/iop/iop_host.h"
#include "ps2x/iop/iop_types.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace ps2x::iop::detail
{
    class IopEmulator
    {
    public:
        explicit IopEmulator(IopHost &host);
        ~IopEmulator();

        IopEmulator(const IopEmulator &) = delete;
        IopEmulator &operator=(const IopEmulator &) = delete;

        void reset();
        [[nodiscard]] ModuleLoadResult loadModule(std::string_view path, const void *arguments, uint32_t argumentSize);
        [[nodiscard]] ModuleLoadResult loadModuleBuffer(uint32_t guestAddress, const void *arguments, uint32_t argumentSize);
        [[nodiscard]] bool stopModule(int32_t moduleId, int32_t *result);
        void runEeCycles(uint64_t eeCycles) noexcept;
        void runToCycle(uint64_t iopCycle) noexcept;
        [[nodiscard]] uint64_t idleEeCycles() const noexcept;
        [[nodiscard]] RpcResult handleRpc(const RpcRequest &request);
        [[nodiscard]] bool hasRpcServer(uint32_t sid) const noexcept;
        void onSifTransfer(const SifTransfer &transfer);
        // An EE sceSdRemote call for LIBSD's RPC server (see IopLibsd::remote).
        [[nodiscard]] bool sdRemote(uint32_t function, const std::array<uint32_t, 5> &args, uint32_t &result);
        [[nodiscard]] bool soundInputRunning() const noexcept;

        [[nodiscard]] uint32_t allocateMemory(uint32_t size, uint32_t alignment = 16u);
        [[nodiscard]] bool freeMemory(uint32_t address);
        [[nodiscard]] bool readMemory(uint32_t address, void *destination, size_t size) const;
        [[nodiscard]] bool writeMemory(uint32_t address, const void *source, size_t size);
        [[nodiscard]] bool zeroMemory(uint32_t address, size_t size);
        [[nodiscard]] bool isMemoryRange(uint32_t address, size_t size) const;

        [[nodiscard]] uint64_t cycles() const noexcept;
        [[nodiscard]] uint64_t instructions() const noexcept;
        [[nodiscard]] uint32_t loadedModuleCount() const noexcept;
        [[nodiscard]] uint32_t threadCount() const noexcept;
        [[nodiscard]] uint32_t rpcServerCount() const noexcept;
        // Host-time counters for performance reports. The maxima cover the time since the previous call.
        void hostTiming(HostTiming &timing) const noexcept;

    private:
        class Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
