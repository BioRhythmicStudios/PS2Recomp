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

        // The EE's sceSdRemote calls, which LIBSD's RPC server (SID 0x80000701) runs on a PS2: `function` is the
        // remote function number (rSdInit 0x8000, rSdSetParam 0x8010 ...) and `args` its arguments. Returns false,
        // leaving `result` alone, for a function not supported here.
        [[nodiscard]] bool remote(uint32_t function, const std::array<uint32_t, 5> &args, uint32_t &result);
        // The SPU2's sound data input asks for the next block of a running block transfer (IopSpu2::InputFetch).
        bool fetchInputBlock(uint32_t core, int16_t block[512]);
        [[nodiscard]] uint64_t inputBlocks() const noexcept { return m_inputBlocks; }

        // SPU2 register offset (from 0x1F900000) that LIBSD derives for a parameter, switch or address entry.
        [[nodiscard]] static uint32_t paramOffset(uint32_t entry);
        [[nodiscard]] static uint32_t switchOffset(uint32_t entry);
        [[nodiscard]] static uint32_t addressOffset(uint32_t entry);

    private:
        // A block transfer: the sound data input of one core, read from IOP memory a 1 KiB block at a time.
        struct BlockTransfer
        {
            bool running = false;
            bool loop = false;
            uint32_t base = 0;
            uint32_t size = 0;
            uint32_t offset = 0; // of the next block the SPU2 reads
        };

        void init(uint32_t flag);
        void setParam(uint32_t entry, uint32_t value);
        [[nodiscard]] uint32_t getParam(uint32_t entry) const;
        void setSwitch(uint32_t entry, uint32_t value);
        [[nodiscard]] uint32_t getSwitch(uint32_t entry) const;
        void setAddr(uint32_t entry, uint32_t address);
        [[nodiscard]] uint32_t getAddr(uint32_t entry) const;
        [[nodiscard]] int32_t blockTrans(uint32_t channel, uint32_t mode, uint32_t iopAddress, uint32_t size,
                                         uint32_t start);
        [[nodiscard]] uint32_t blockTransStatus(uint32_t channel) const;
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
        std::array<BlockTransfer, 2> m_blockTransfer{};
        uint64_t m_bytesTransferred = 0;
        uint64_t m_inputBlocks = 0;
    };
}
