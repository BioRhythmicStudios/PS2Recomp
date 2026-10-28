#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace ps2x::iop
{
    class IopHost;
}

namespace ps2x::iop::detail
{
    struct IopCpuState;
    class IopMemory;
    class IopSpu2;

    // Native Sony libsd (the IOP SPU2 library, export table "libsd" version 1.4) over the emulated SPU2. The
    // register addressing follows the game's own LIBSD.IRX: the same parameter table, entry encoding and
    // address formats, so SPU2 register state matches what the original library would leave behind.
    class IopLibsd
    {
    public:
        struct Callback
        {
            uint64_t cycle = 0;
            uint32_t function = 0;
            uint32_t gp = 0;
            uint32_t argument = 0;
        };

        IopLibsd(IopHost &host, IopMemory &memory, IopSpu2 &spu2) noexcept;

        void reset();

        // Handles libsd import `ordinal` at IOP cycle `cycle`. A DMA transfer completes later, so the call can
        // leave a completion callback in `callback` for the caller to schedule.
        [[nodiscard]] bool dispatchImport(uint16_t ordinal, IopCpuState &cpu, uint64_t cycle,
                                          std::optional<Callback> &callback);

        [[nodiscard]] bool initialized() const noexcept { return m_initialized; }
        [[nodiscard]] uint64_t calls(uint16_t ordinal) const noexcept
        {
            return ordinal < m_calls.size() ? m_calls[ordinal] : 0u;
        }
        [[nodiscard]] uint64_t bytesTransferred() const noexcept { return m_bytesTransferred; }

        // SPU2 register offset (from 0x1F900000) that LIBSD derives for a parameter, switch or address entry.
        [[nodiscard]] static uint32_t paramOffset(uint32_t entry);
        [[nodiscard]] static uint32_t switchOffset(uint32_t entry);
        [[nodiscard]] static uint32_t addressOffset(uint32_t entry);

    private:
        void init(uint32_t flag);
        void setCoreAttr(uint32_t entry, uint32_t value);
        [[nodiscard]] uint32_t getCoreAttr(uint32_t entry) const;
        [[nodiscard]] int32_t voiceTrans(uint32_t channel, uint32_t mode, uint32_t iopAddress, uint32_t spuAddress,
                                         uint32_t size, uint64_t cycle, std::optional<Callback> &callback);
        void warnOnce(uint16_t ordinal, const std::string &message);

        IopHost &m_host;
        IopMemory &m_memory;
        IopSpu2 &m_spu2;
        bool m_initialized = false;
        std::array<uint64_t, 32> m_calls{};
        std::array<bool, 32> m_warned{};
        std::array<uint32_t, 2> m_transferCallback{};
        std::array<uint32_t, 2> m_transferCallbackGp{};
        std::array<uint32_t, 2> m_transferHandler{};
        std::array<uint32_t, 2> m_transferHandlerGp{};
        std::array<uint32_t, 2> m_transferHandlerArg{};
        std::array<uint64_t, 2> m_busyUntil{};
        uint32_t m_irqCallback = 0;
        uint32_t m_spu2Handler = 0;
        uint32_t m_spu2HandlerArg = 0;
        std::array<uint32_t, 2> m_effectMode{};
        uint64_t m_bytesTransferred = 0;
    };
}
