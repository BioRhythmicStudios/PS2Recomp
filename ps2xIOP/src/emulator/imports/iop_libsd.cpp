#include "iop_libsd.h"

#include "../core/iop_cpu.h"
#include "../core/iop_memory.h"
#include "../spu2/iop_spu2.h"
#include "ps2x/iop/iop_host.h"

#include <sstream>
#include <vector>

namespace ps2x::iop::detail
{
    namespace
    {
        constexpr uint32_t kInvalidOffset = 0xFFFFFFFFu;

        // LIBSD's register table: the SPU2 register of each parameter, switch and address entry, for core 0 and
        // voice 0 (entry bits 15-8 select the row).
        constexpr uint16_t kRegisterTable[] = {
            0x000, 0x002, 0x004, 0x006, 0x008, 0x00A, 0x00C, 0x00E, // VOLL..VOLXR
            0x198,                                                  // MMIX
            0x760, 0x762, 0x764, 0x766, 0x768, 0x76A, 0x76C, 0x76E, // MVOL, EVOL, AVOL, BVOL
            0x770, 0x772,                                           // MVOLX
            0x180, 0x184, 0x1A0, 0x1A4, 0x340,                      // PMON NON KON KOFF ENDX
            0x188, 0x18C, 0x190, 0x194,                             // VMIXL VMIXEL VMIXR VMIXER
            0x2E0, 0x33C, 0x1A8, 0x19C,                             // ESA EEA TSA IRQA
            0x1C0, 0x1C4, 0x1C8,                                    // SSA LSAX NAX
            0x19A, 0x1A8, 0x1AC, 0x1AE, 0x1B0,
        };
        constexpr uint32_t kRegisterRows = sizeof(kRegisterTable) / sizeof(kRegisterTable[0]);

        constexpr uint32_t kAttr = 0x19Au;
        constexpr uint32_t kCoreStride = 0x400u;
        constexpr uint32_t kMasterStride = 0x28u;
        constexpr uint32_t kEeaEntry = 0x1D00u;

        // SetCoreAttr: the ATTR bit of each attribute (indexed by attribute / 2), as in LIBSD.
        constexpr int kCoreAttrBit[] = {0, 7, 6, 14, 8};
        constexpr uint32_t kCoreAttrNoiseClock = 8u;
        constexpr uint32_t kCoreAttrSpdifMode = 10u;

        // Halfwords per transfer cycle: the SPU2 DMA is modelled at 4 IOP cycles per halfword.
        constexpr uint64_t kTransferCyclesPerHalfword = 4u;
        constexpr uint32_t kTransferModeRead = 1u;
        constexpr uint32_t kTransferModeIo = 8u;

        // sceSdBlockTrans modes and the sound data input's block: 256 left then 256 right 16-bit samples.
        constexpr uint32_t kBlockTransferWrite = 0u;
        constexpr uint32_t kBlockTransferStop = 2u;
        constexpr uint32_t kBlockTransferWriteFrom = 3u;
        constexpr uint32_t kBlockTransferLoop = 0x10u;
        constexpr uint32_t kInputBlockBytes = 1024u;

        uint32_t tableOffset(uint32_t entry)
        {
            const uint32_t row = (entry >> 8u) & 0xFFu;
            return row < kRegisterRows ? kRegisterTable[row] : kInvalidOffset;
        }
    }

    IopLibsd::IopLibsd(IopHost &host, IopMemory &memory, IopSpu2 &spu2) noexcept
        : m_host(host), m_memory(memory), m_spu2(spu2)
    {
    }

    void IopLibsd::reset()
    {
        m_initialized = false;
        m_calls.fill(0u);
        m_warned.fill(false);
        m_transferCallback.fill(0u);
        m_transferCallbackGp.fill(0u);
        m_transferHandler.fill(0u);
        m_transferHandlerGp.fill(0u);
        m_transferHandlerArg.fill(0u);
        m_busyUntil.fill(0u);
        m_irqCallback = 0u;
        m_spu2Handler = 0u;
        m_spu2HandlerArg = 0u;
        m_effectMode.fill(0u);
        m_blockTransfer.fill(BlockTransfer{});
        m_bytesTransferred = 0u;
        m_inputBlocks = 0u;
    }

    uint32_t IopLibsd::paramOffset(uint32_t entry)
    {
        const uint32_t base = tableOffset(entry);
        if (base == kInvalidOffset)
            return kInvalidOffset;
        const uint32_t coreStride = (entry & 0x80u) ? kMasterStride : kCoreStride;
        return base + (entry & 1u) * coreStride + ((entry & 0x3Eu) << 3u);
    }

    uint32_t IopLibsd::switchOffset(uint32_t entry)
    {
        const uint32_t base = tableOffset(entry);
        return base == kInvalidOffset ? kInvalidOffset : base + (entry & 1u) * kCoreStride;
    }

    uint32_t IopLibsd::addressOffset(uint32_t entry)
    {
        const uint32_t base = tableOffset(entry);
        return base == kInvalidOffset ? kInvalidOffset : base + (entry & 1u) * kCoreStride + (entry & 0x3Eu) * 6u;
    }

    void IopLibsd::warnOnce(uint16_t ordinal, const std::string &message)
    {
        if (ordinal < m_warned.size())
        {
            if (m_warned[ordinal])
                return;
            m_warned[ordinal] = true;
        }
        m_host.log(LogLevel::Warning, message);
    }

    void IopLibsd::init(uint32_t flag)
    {
        const bool resetHardware = (flag & 0xFu) == 0u;
        if (resetHardware)
        {
            // sceSdInit(0) resets the SPU2: every voice keyed off, pitch modulation and noise off, master volume 0.
            // LIBSD writes registers only: sound RAM keeps what was transferred before (Maximo uploads its resident
            // sample bank in the front end and calls sceSdInit again when a level starts).
            m_spu2.resetRegisters();
            for (uint32_t core = 0; core < 2u; ++core)
            {
                m_spu2.write16(core * kCoreStride + 0x1A4u, 0xFFFFu); // KOFF
                m_spu2.write16(core * kCoreStride + 0x1A6u, 0xFFFFu);
                m_spu2.write16(core * kCoreStride + 0x1B0u, 0u);      // ADMAS
            }
            m_busyUntil.fill(0u);
            m_blockTransfer.fill(BlockTransfer{}); // the register reset stopped the sound data input too
        }

        // InitCoreVolume(flag & 15), register for register.
        const bool effects = (flag & 0xFu) != 0u;
        m_spu2.write16(0x7C0u, 0xC032u); // SPDIF_OUT
        m_spu2.write16(kAttr, effects ? 0xC080u : 0xC000u);
        m_spu2.write16(kCoreStride + kAttr, effects ? 0xC081u : 0xC001u);
        for (uint32_t core = 0; core < 2u; ++core)
        {
            const uint32_t base = core * kCoreStride;
            for (const uint32_t mix : {0x188u, 0x18Cu, 0x190u, 0x194u}) // VMIXL VMIXEL VMIXR VMIXER: all voices
            {
                m_spu2.write16(base + mix, 0xFFFFu);
                m_spu2.write16(base + mix + 2u, 0x00FFu);
            }
        }
        m_spu2.write16(0x198u, 0x0FF0u);               // MMIX core 0: voices and sound input, dry and wet
        m_spu2.write16(kCoreStride + 0x198u, 0x0FFCu); // MMIX core 1: also core 0's output, dry
        if (!effects)
        {
            for (uint32_t core = 0; core < 2u; ++core)
            {
                const uint32_t master = 0x760u + core * kMasterStride;
                for (uint32_t reg = 0; reg < 8u; reg += 2u) // MVOLL MVOLR EVOLL EVOLR
                    m_spu2.write16(master + reg, 0u);
            }
            m_spu2.write16(0x33Cu, 0x000Eu);               // EEA core 0
            m_spu2.write16(kCoreStride + 0x33Cu, 0x000Fu); // EEA core 1
        }
        m_spu2.write16(0x768u, 0u); // AVOL core 0
        m_spu2.write16(0x76Au, 0u);
        m_spu2.write16(0x790u, 0x7FFFu); // AVOL core 1: core 0's output at full volume
        m_spu2.write16(0x792u, 0x7FFFu);
        for (const uint32_t bvol : {0x76Cu, 0x76Eu, 0x794u, 0x796u})
            m_spu2.write16(bvol, 0u);
        m_initialized = true;
    }

    void IopLibsd::setParam(uint32_t entry, uint32_t value)
    {
        if (const uint32_t offset = paramOffset(entry); offset != kInvalidOffset)
            m_spu2.write16(offset, static_cast<uint16_t>(value));
    }

    uint32_t IopLibsd::getParam(uint32_t entry) const
    {
        const uint32_t offset = paramOffset(entry);
        return offset != kInvalidOffset ? m_spu2.read16(offset) : 0u;
    }

    void IopLibsd::setSwitch(uint32_t entry, uint32_t value)
    {
        if (const uint32_t offset = switchOffset(entry); offset != kInvalidOffset)
        {
            m_spu2.write16(offset, static_cast<uint16_t>(value));
            m_spu2.write16(offset + 2u, static_cast<uint16_t>((value >> 16u) & 0xFFu));
        }
    }

    uint32_t IopLibsd::getSwitch(uint32_t entry) const
    {
        const uint32_t offset = switchOffset(entry);
        return offset != kInvalidOffset
                   ? (m_spu2.read16(offset) | (static_cast<uint32_t>(m_spu2.read16(offset + 2u)) << 16u))
                   : 0u;
    }

    void IopLibsd::setAddr(uint32_t entry, uint32_t address)
    {
        if (const uint32_t offset = addressOffset(entry); offset != kInvalidOffset)
        {
            m_spu2.write16(offset, static_cast<uint16_t>(address >> 17u));
            if ((entry & 0xFF00u) != kEeaEntry) // EEA has only the high halfword
                m_spu2.write16(offset + 2u, static_cast<uint16_t>((address >> 1u) & 0xFFF8u));
        }
    }

    uint32_t IopLibsd::getAddr(uint32_t entry) const
    {
        const uint32_t offset = addressOffset(entry);
        if (offset == kInvalidOffset)
            return 0u;
        const uint32_t high = static_cast<uint32_t>(m_spu2.read16(offset)) << 17u;
        const uint32_t low = (entry & 0xFF00u) == kEeaEntry ? 0x1FFFFu
                                                           : static_cast<uint32_t>(m_spu2.read16(offset + 2u)) << 1u;
        return high | low;
    }

    // sceSdBlockTrans(channel, mode, iopAddress, size, start). Only writes to the SPU2 (the sound data input) are
    // supported. With SD_BLOCK_LOOP the transfer wraps round the buffer until it is stopped; WRITE_FROM starts it at
    // `start` inside the buffer. The SPU2 reads the buffer as it mixes (fetchInputBlock), so the read position
    // follows the audio clock. The one-shot transfer stops at the end of the buffer.
    int32_t IopLibsd::blockTrans(uint32_t channel, uint32_t mode, uint32_t iopAddress, uint32_t size, uint32_t start)
    {
        BlockTransfer &transfer = m_blockTransfer[channel & 1u];
        const uint32_t direction = mode & 3u;
        if (direction == kBlockTransferStop)
        {
            const uint32_t status = blockTransStatus(channel);
            transfer.running = false;
            m_spu2.stopInput(channel);
            return static_cast<int32_t>(status);
        }
        if ((direction != kBlockTransferWrite && direction != kBlockTransferWriteFrom) || size < kInputBlockBytes ||
            (size % kInputBlockBytes) != 0u)
            return -1;
        transfer.running = true;
        transfer.loop = (mode & kBlockTransferLoop) != 0u;
        transfer.base = iopAddress;
        transfer.size = size;
        transfer.offset = direction == kBlockTransferWriteFrom && start - iopAddress < size
                              ? (start - iopAddress) / kInputBlockBytes * kInputBlockBytes
                              : 0u;
        m_spu2.startInput(channel);
        return 0;
    }

    // The buffer half the transfer is in (bit 24) and the IOP address it reads next.
    uint32_t IopLibsd::blockTransStatus(uint32_t channel) const
    {
        const BlockTransfer &transfer = m_blockTransfer[channel & 1u];
        if (transfer.size == 0u)
            return 0u;
        return (transfer.offset >= transfer.size / 2u ? 1u << 24u : 0u) | ((transfer.base + transfer.offset) & 0xFFFFFFu);
    }

    bool IopLibsd::fetchInputBlock(uint32_t core, int16_t block[512])
    {
        BlockTransfer &transfer = m_blockTransfer[core & 1u];
        if (!transfer.running)
            return false;
        if (!m_memory.readRam(transfer.base + transfer.offset, block, kInputBlockBytes))
        {
            transfer.running = false;
            return false;
        }
        ++m_inputBlocks;
        transfer.offset += kInputBlockBytes;
        if (transfer.offset >= transfer.size)
        {
            transfer.offset = 0u;
            if (!transfer.loop)
                transfer.running = false; // this was the last block: the SPU2 plays it, then asks again and stops
        }
        return true;
    }

    bool IopLibsd::remote(uint32_t function, const std::array<uint32_t, 5> &args, uint32_t &result)
    {
        switch (function)
        {
        case 0x8000u: // rSdInit(flag)
            init(args[0]);
            result = 0u;
            return true;
        case 0x8010u: // rSdSetParam(entry, value)
            setParam(args[0], args[1]);
            result = 0u;
            return true;
        case 0x8020u: // rSdGetParam(entry)
            result = getParam(args[0]);
            return true;
        case 0x8030u: // rSdSetSwitch(entry, value)
            setSwitch(args[0], args[1]);
            result = 0u;
            return true;
        case 0x8040u: // rSdGetSwitch(entry)
            result = getSwitch(args[0]);
            return true;
        case 0x8050u: // rSdSetAddr(entry, address)
            setAddr(args[0], args[1]);
            result = 0u;
            return true;
        case 0x8060u: // rSdGetAddr(entry)
            result = getAddr(args[0]);
            return true;
        case 0x8070u: // rSdSetCoreAttr(entry, value)
            setCoreAttr(args[0], args[1]);
            result = 0u;
            return true;
        case 0x8080u: // rSdGetCoreAttr(entry)
            result = getCoreAttr(args[0]);
            return true;
        case 0x80E0u: // rSdBlockTrans(channel, mode, iopAddress, size, start)
            result = static_cast<uint32_t>(blockTrans(args[0], args[1], args[2], args[3], args[4]));
            return true;
        case 0x8100u: // rSdBlockTransStatus(channel, flag)
            result = blockTransStatus(args[0]);
            return true;
        default:
            return false;
        }
    }

    void IopLibsd::setCoreAttr(uint32_t entry, uint32_t value)
    {
        const uint32_t core = entry & 1u;
        const uint32_t attribute = entry & 0xEu;
        const uint32_t offset = core * kCoreStride + kAttr;
        const uint16_t attr = m_spu2.read16(offset);
        if (attribute == kCoreAttrNoiseClock)
        {
            m_spu2.write16(offset, static_cast<uint16_t>((attr & ~0x3F00u) | ((value & 0x3Fu) << 8u)));
            return;
        }
        if (attribute == kCoreAttrSpdifMode)
            return; // digital output configuration only: no effect on the mixed sound
        const int bit = kCoreAttrBit[attribute >> 1u];
        m_spu2.write16(offset, static_cast<uint16_t>((attr & ~(1u << bit)) | ((value & 1u) << bit)));
    }

    uint32_t IopLibsd::getCoreAttr(uint32_t entry) const
    {
        const uint32_t core = entry & 1u;
        const uint32_t attribute = entry & 0xEu;
        const uint16_t attr = m_spu2.read16(core * kCoreStride + kAttr);
        if (attribute == kCoreAttrNoiseClock)
            return (attr >> 8u) & 0x3Fu;
        if (attribute == kCoreAttrSpdifMode || attribute == 0u)
            return 0u;
        return (attr >> kCoreAttrBit[attribute >> 1u]) & 1u;
    }

    int32_t IopLibsd::voiceTrans(uint32_t channel, uint32_t mode, uint32_t iopAddress, uint32_t spuAddress,
                                 uint32_t size, uint64_t cycle, std::optional<Callback> &callback)
    {
        channel &= 1u;
        if (cycle < m_busyUntil[channel])
            return -1; // the previous transfer on this channel has not completed
        const uint32_t direction = mode & 3u;
        if (direction > kTransferModeRead)
            return -1;

        // TSA is left at the transfer address, as the hardware transfer does.
        m_spu2.write16(channel * kCoreStride + 0x1A8u, static_cast<uint16_t>(spuAddress >> 17u));
        m_spu2.write16(channel * kCoreStride + 0x1AAu, static_cast<uint16_t>((spuAddress >> 1u) & 0xFFF8u));

        std::vector<uint8_t> buffer(size);
        if (direction == 0u)
        {
            if (!m_memory.readRam(iopAddress, buffer.data(), buffer.size()))
                return -1;
            m_spu2.writeRam(spuAddress, buffer.data(), buffer.size());
        }
        else
        {
            m_spu2.readRam(spuAddress, buffer.data(), buffer.size());
            if (!m_memory.writeRam(iopAddress, buffer.data(), buffer.size()))
                return -1;
        }
        m_bytesTransferred += size;

        if ((mode & kTransferModeIo) != 0u)
            return static_cast<int32_t>(size); // programmed I/O: complete on return, no interrupt

        const uint64_t done = cycle + 1u + (static_cast<uint64_t>(size) / 2u) * kTransferCyclesPerHalfword;
        m_busyUntil[channel] = done;
        // The DMA-end interrupt calls the transfer interrupt handler if one is set, else the transfer callback.
        if (m_transferHandler[channel] != 0u)
            callback = Callback{done, m_transferHandler[channel], m_transferHandlerGp[channel], channel};
        else if (m_transferCallback[channel] != 0u)
            callback = Callback{done, m_transferCallback[channel], m_transferCallbackGp[channel], channel};
        return static_cast<int32_t>(size);
    }

    bool IopLibsd::dispatchImport(uint16_t ordinal, IopCpuState &cpu, uint64_t cycle,
                                  std::optional<Callback> &callback)
    {
        const uint32_t a0 = cpu.gpr[4];
        const uint32_t a1 = cpu.gpr[5];
        const uint32_t a2 = cpu.gpr[6];
        const uint32_t a3 = cpu.gpr[7];
        const auto setV0 = [&](uint32_t value) { cpu.gpr[2] = value; };
        if (ordinal < m_calls.size())
            ++m_calls[ordinal];

        switch (ordinal)
        {
        case 2: // sceSdQuit
            setV0(0u);
            return true;
        case 4: // sceSdInit(flag)
            init(a0);
            setV0(0u);
            return true;
        case 5: // sceSdSetParam(entry, value)
            setParam(a0, a1);
            setV0(0u);
            return true;
        case 6: // sceSdGetParam(entry)
            setV0(getParam(a0));
            return true;
        case 7: // sceSdSetSwitch(entry, value)
            setSwitch(a0, a1);
            setV0(0u);
            return true;
        case 8: // sceSdGetSwitch(entry)
            setV0(getSwitch(a0));
            return true;
        case 9: // sceSdSetAddr(entry, address)
            setAddr(a0, a1);
            setV0(0u);
            return true;
        case 10: // sceSdGetAddr(entry)
            setV0(getAddr(a0));
            return true;
        case 11: // sceSdSetCoreAttr(entry, value)
            setCoreAttr(a0, a1);
            setV0(0u);
            return true;
        case 12: // sceSdGetCoreAttr(entry)
            setV0(getCoreAttr(a0));
            return true;
        case 17: // sceSdVoiceTrans(channel, mode, iopAddress, spuAddress, size)
        {
            const uint32_t size = m_memory.read32(cpu.gpr[29] + 16u);
            setV0(static_cast<uint32_t>(voiceTrans(a0, a1, a2, a3, size, cycle, callback)));
            return true;
        }
        case 19: // sceSdVoiceTransStatus(channel, flag): 1 when the transfer has completed
            setV0(cycle >= m_busyUntil[a0 & 1u] ? 1u : 0u);
            return true;
        case 21: // sceSdSetTransCallback(channel, function) -> previous
        {
            const uint32_t channel = a0 & 0xFFFFu;
            if (channel > 1u)
            {
                setV0(0u);
                return true;
            }
            setV0(m_transferCallback[channel]);
            m_transferCallback[channel] = a1;
            m_transferCallbackGp[channel] = cpu.gpr[28];
            return true;
        }
        case 22: // sceSdSetIRQCallback(function) -> previous
            setV0(m_irqCallback);
            m_irqCallback = a0;
            return true;
        case 23: // sceSdSetEffectAttr(core, attr)
        {
            const uint32_t core = a0 & 1u;
            const uint32_t mode = m_memory.read32(a1 + 4u) & ~0x100u;
            if (mode >= 10u)
            {
                setV0(0xFFFFFFFFu);
                return true;
            }
            m_effectMode[core] = mode;
            std::ostringstream out;
            out << "[IOP:libsd] sceSdSetEffectAttr(core=" << core << ", mode=" << mode
                << "): reverb is not emulated; the wet mix is silent";
            warnOnce(ordinal, out.str());
            setV0(0u);
            return true;
        }
        case 24: // sceSdGetEffectAttr(core, attr)
            m_memory.write32(a1 + 4u, m_effectMode[a0 & 1u]);
            setV0(0u);
            return true;
        case 25: // sceSdClearEffectWorkArea(core, channel, mode)
            setV0(0u);
            return true;
        case 26: // sceSdSetTransIntrHandler(channel, handler, argument) -> previous
        {
            const uint32_t channel = a0 & 1u;
            setV0(m_transferHandler[channel]);
            m_transferHandler[channel] = a1;
            m_transferHandlerGp[channel] = cpu.gpr[28];
            m_transferHandlerArg[channel] = a2;
            return true;
        }
        case 27: // sceSdSetSpu2IntrHandler(handler, argument) -> previous
            setV0(m_spu2Handler);
            m_spu2Handler = a0;
            m_spu2HandlerArg = a1;
            return true;
        default:
        {
            std::ostringstream out;
            out << "[IOP:libsd] unimplemented libsd:" << ordinal << " (a0=0x" << std::hex << a0 << ")";
            warnOnce(ordinal, out.str());
            setV0(0u);
            return true;
        }
        }
    }
}
