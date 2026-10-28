#include "runtime/ps2_ipu.h"

#include "ps2_ipu_mpeg.h"
#include "runtime/ps2_memory.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace ps2_ipu
{
    // The IPU's input: qwords from the input FIFO and DMA channel 4, read MSB first. BP is the bit
    // offset in the first buffered qword; consumed qwords are dropped when a step commits.
    class DeviceBitReader : public BitReader
    {
    public:
        explicit DeviceBitReader(Ps2Ipu &ipu) : m_ipu(ipu) {}

        void append(const uint8_t *qword) { m_buf.insert(m_buf.end(), qword, qword + 16); }
        void clear(uint32_t bp)
        {
            m_buf.clear();
            m_pos = bp;
        }
        void compact()
        {
            const size_t whole = std::min<size_t>(static_cast<size_t>(m_pos / 128u), m_buf.size() / 16u);
            if (whole == 0u)
                return;
            m_buf.erase(m_buf.begin(), m_buf.begin() + static_cast<std::ptrdiff_t>(whole * 16u));
            m_pos -= whole * 128u;
        }
        uint32_t qwords() const { return static_cast<uint32_t>(m_buf.size() / 16u); }
        uint32_t bitPointer() const { return static_cast<uint32_t>(m_pos & 127u); }

    protected:
        bool more() override
        {
            std::array<uint8_t, 16> q{};
            if (!m_ipu.pullInputQword(q))
                return false;
            append(q.data());
            return true;
        }

    private:
        Ps2Ipu &m_ipu;
    };
}

namespace
{
    constexpr uint32_t kCmdBclr = 0, kCmdIdec = 1, kCmdBdec = 2, kCmdVdec = 3, kCmdFdec = 4, kCmdSetiq = 5,
                       kCmdSetvq = 6, kCmdCsc = 7, kCmdPack = 8, kCmdSetth = 9;

    // IPU_CTRL bits
    constexpr uint32_t kCtrlEcd = 1u << 14;
    constexpr uint32_t kCtrlScd = 1u << 15;
    constexpr uint32_t kCtrlWritable = 0x07F30000u; // IDP, AS, IVF, QST, MP1, PCT
    constexpr uint32_t kCtrlRst = 1u << 30;
    constexpr uint32_t kCtrlBusy = 1u << 31;

    constexpr uint32_t kOutFifoQwords = 8;
    constexpr uint32_t kInFifoQwords = 8;
    constexpr uint32_t kDmaStr = 0x100u;
    constexpr uint32_t kDmacCtrl = 0x1000E000u;

    bool traceEnabled()
    {
        static const bool enabled = []
        {
            const char *v = std::getenv("PS2_IPU_TRACE");
            return v && *v && *v != '0';
        }();
        return enabled;
    }

    const char *commandName(uint32_t code)
    {
        static const char *names[] = {"BCLR", "IDEC", "BDEC", "VDEC", "FDEC", "SETIQ", "SETVQ", "CSC",
                                      "PACK", "SETTH", "?A", "?B", "?C", "?D", "?E", "?F"};
        return names[code & 15u];
    }
}

Ps2Ipu::Ps2Ipu(PS2Memory &memory)
    : m_memory(memory), m_bits(std::make_unique<ps2_ipu::DeviceBitReader>(*this)),
      m_state(std::make_unique<ps2_ipu::DecoderState>())
{
    reset();
}

Ps2Ipu::~Ps2Ipu() = default;

void Ps2Ipu::reset()
{
    m_bits->clear(0);
    m_output.clear();
    m_ctrl &= kCtrlWritable;
    m_cmdData = 0;
    m_top = 0;
    m_topValid = false;
    m_command = 0;
    m_busy = false;
    m_awaitingDrain = false;
    m_remaining = 0;
    m_idecFirst = true;
    m_state->resetDcPredictors();
}

// ---- registers -----------------------------------------------------------------------------------

uint32_t Ps2Ipu::bufferedInputQwords() const { return m_bits->qwords(); }

void Ps2Ipu::setTopFromInput()
{
    try
    {
        m_top = m_bits->peek(32);
        m_topValid = true;
    }
    catch (const ps2_ipu::IpuStarved &)
    {
        m_topValid = false;
    }
}

uint32_t Ps2Ipu::readRegister32(uint32_t address)
{
    switch (address & 0x3Cu)
    {
    case 0x00: // IPU_CMD.DATA
        return m_cmdData;
    case 0x04: // IPU_CMD.BUSY
        return m_busy ? 0x80000000u : 0u;
    case 0x10: // IPU_CTRL
    {
        const uint32_t n = bufferedInputQwords();
        const uint32_t ifc = std::min(n > 2u ? n - 2u : 0u, kInFifoQwords);
        const uint32_t ofc = std::min(static_cast<uint32_t>(m_output.size()), kOutFifoQwords);
        return (m_ctrl & ~(kCtrlBusy | 0xFFu)) | ifc | (ofc << 4) | (m_busy ? kCtrlBusy : 0u);
    }
    case 0x20: // IPU_BP
    {
        const uint32_t n = bufferedInputQwords();
        const uint32_t fp = std::min(n, 2u);
        const uint32_t ifc = std::min(n - fp, kInFifoQwords);
        return m_bits->bitPointer() | (ifc << 8) | (fp << 16);
    }
    case 0x30: // IPU_TOP.BSTOP
        if (!m_busy && !m_topValid)
            setTopFromInput();
        return m_top;
    case 0x34: // IPU_TOP.BUSY
        if (!m_busy && !m_topValid)
            setTopFromInput();
        return (m_busy || !m_topValid) ? 0x80000000u : 0u;
    default:
        return 0u;
    }
}

uint64_t Ps2Ipu::readRegister64(uint32_t address)
{
    const uint32_t base = address & ~7u;
    return static_cast<uint64_t>(readRegister32(base)) | (static_cast<uint64_t>(readRegister32(base + 4u)) << 32);
}

void Ps2Ipu::writeRegister32(uint32_t address, uint32_t value)
{
    switch (address & 0x3Cu)
    {
    case 0x00:
        startCommand(value);
        break;
    case 0x10:
        if (value & kCtrlRst)
        {
            if (traceEnabled())
                std::fprintf(stderr, "[ipu] reset (ctrl=0x%08X)\n", value);
            reset();
        }
        // RST self-clears (PCSX2's ipuSoftReset masks it out too). Latching it is wrong: the MPEG library
        // read-modify-writes IPU_CTRL to set MP1/PCT, and a latched RST would reset the FIFO and BP each time.
        m_ctrl = (m_ctrl & ~kCtrlWritable) | (value & kCtrlWritable);
        m_state->intraDcPrecision = (value >> 16) & 3u;
        m_state->alternateScan = (value >> 20) & 1u;
        m_state->intraVlcFormat = (value >> 21) & 1u;
        m_state->qScaleType = (value >> 22) & 1u;
        m_state->mpeg1 = (value >> 23) & 1u;
        m_state->pictureType = (value >> 24) & 7u;
        break;
    default:
        break; // IPU_BP and IPU_TOP are read-only
    }
}

void Ps2Ipu::writeInputFifo(const uint8_t qword[16])
{
    m_bits->append(qword);
    ++m_stats.inputQwords;
    m_topValid = false;
    process();
}

void Ps2Ipu::readOutputFifo(uint8_t qword[16])
{
    if (m_output.empty())
    {
        std::memset(qword, 0, 16);
        return;
    }
    std::memcpy(qword, m_output.front().data(), 16);
    m_output.pop_front();
    process();
}

// ---- DMA -----------------------------------------------------------------------------------------

uint32_t Ps2Ipu::reg(uint32_t address) const
{
    const auto it = m_memory.m_ioRegisters.find(address);
    return it == m_memory.m_ioRegisters.end() ? 0u : it->second;
}

void Ps2Ipu::setReg(uint32_t address, uint32_t value) { m_memory.m_ioRegisters[address] = value; }

bool Ps2Ipu::channelActive(uint32_t channelBase) const
{
    const auto it = m_memory.m_ioRegisters.find(kDmacCtrl);
    const bool dmacEnabled = it == m_memory.m_ioRegisters.end() || (it->second & 1u) != 0u;
    return dmacEnabled && (reg(channelBase) & kDmaStr) != 0u;
}

void Ps2Ipu::completeChannel(uint32_t channelBase, uint32_t cause)
{
    setReg(channelBase + 0x20u, 0u);
    m_memory.completeDmacChannel(channelBase, cause);
    if (traceEnabled())
        std::fprintf(stderr, "[ipu] DMA ch%u done (MADR=0x%08X)\n", cause, reg(channelBase + 0x10u));
}

bool Ps2Ipu::readMemoryQword(uint32_t address, Qword &out)
{
    try
    {
        // DMA addresses: bit 31 selects the scratchpad (SPR).
        const bool scratch = (address & 0x80000000u) != 0u || m_memory.isScratchpad(address);
        const uint32_t phys = scratch ? (address & 0x3FF0u) : (m_memory.translateAddress(address) & ~15u);
        const uint8_t *base = scratch ? m_memory.m_scratchpad : m_memory.m_rdram;
        const uint32_t size = scratch ? PS2_SCRATCHPAD_SIZE : PS2_RAM_SIZE;
        if (phys + 16u > size)
            return false;
        std::memcpy(out.data(), base + phys, 16);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool Ps2Ipu::writeMemoryQword(uint32_t address, const Qword &data)
{
    try
    {
        // DMA addresses: bit 31 selects the scratchpad (SPR).
        const bool scratch = (address & 0x80000000u) != 0u || m_memory.isScratchpad(address);
        const uint32_t phys = scratch ? (address & 0x3FF0u) : (m_memory.translateAddress(address) & ~15u);
        uint8_t *base = scratch ? m_memory.m_scratchpad : m_memory.m_rdram;
        const uint32_t size = scratch ? PS2_SCRATCHPAD_SIZE : PS2_RAM_SIZE;
        if (phys + 16u > size)
            return false;
        std::memcpy(base + phys, data.data(), 16);
        if (!scratch)
            m_memory.markModified(phys, 16);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool Ps2Ipu::pullInputQword(Qword &out)
{
    const uint32_t ch = kToIpuChannel;
    for (int guard = 0; guard < 4096; ++guard)
    {
        if (!channelActive(ch))
            return false;
        uint32_t chcr = reg(ch);
        const uint32_t mode = (chcr >> 2) & 3u;
        const uint32_t qwc = reg(ch + 0x20u) & 0xFFFFu;
        const bool lastTag = mode == 0u || m_toIpuChainEnded;
        if (qwc != 0u)
        {
            const uint32_t madr = reg(ch + 0x10u);
            if (!readMemoryQword(madr, out))
            {
                completeChannel(ch, 4u);
                return false;
            }
            setReg(ch + 0x10u, madr + 16u);
            setReg(ch + 0x20u, qwc - 1u);
            ++m_stats.inputQwords;
            if (qwc == 1u && lastTag)
                completeChannel(ch, 4u);
            return true;
        }
        if (lastTag)
        {
            completeChannel(ch, 4u);
            return false;
        }

        // Source chain: fetch the next DMAtag.
        uint32_t tadr = reg(ch + 0x30u);
        Qword tagBytes{};
        if (!readMemoryQword(tadr, tagBytes))
        {
            completeChannel(ch, 4u);
            return false;
        }
        uint64_t tag = 0;
        std::memcpy(&tag, tagBytes.data(), 8);
        const uint32_t tagQwc = static_cast<uint32_t>(tag & 0xFFFFu);
        const uint32_t id = static_cast<uint32_t>((tag >> 28) & 7u);
        const bool irq = ((tag >> 31) & 1u) != 0u;
        uint32_t addr = static_cast<uint32_t>((tag >> 32) & 0x7FFFFFF0u);
        if ((tag >> 63) & 1u)
            addr = 0x70000000u | (addr & 0x3FF0u); // SPR
        uint32_t madr = 0;
        switch (id)
        {
        case 0: // refe
            madr = addr;
            tadr += 16u;
            m_toIpuChainEnded = true;
            break;
        case 1: // cnt
            madr = tadr + 16u;
            tadr = madr + tagQwc * 16u;
            break;
        case 2: // next
            madr = tadr + 16u;
            tadr = addr;
            break;
        case 3: // ref
        case 4: // refs
            madr = addr;
            tadr += 16u;
            break;
        case 5: // call
            madr = tadr + 16u;
            if (m_toIpuAsp < 2u)
                setReg(ch + (m_toIpuAsp == 0u ? 0x40u : 0x50u), madr + tagQwc * 16u);
            ++m_toIpuAsp;
            tadr = addr;
            break;
        case 6: // ret
            madr = tadr + 16u;
            if (m_toIpuAsp > 0u)
            {
                --m_toIpuAsp;
                tadr = reg(ch + (m_toIpuAsp == 0u ? 0x40u : 0x50u));
            }
            else
            {
                m_toIpuChainEnded = true;
            }
            break;
        default: // end
            madr = tadr + 16u;
            m_toIpuChainEnded = true;
            break;
        }
        if (irq && (chcr & 0x80u)) // TIE
            m_toIpuChainEnded = true;
        if (traceEnabled())
            std::fprintf(stderr, "[ipu] toIPU tag id=%u qwc=%u addr=0x%08X at 0x%08X%s\n", id, tagQwc, addr, reg(ch + 0x30u),
                         m_toIpuChainEnded ? " (last)" : "");
        chcr = (chcr & 0x0000FFCFu) | ((m_toIpuAsp & 3u) << 4) | (static_cast<uint32_t>((tag >> 16) & 0xFFFFu) << 16);
        setReg(ch, chcr);
        setReg(ch + 0x10u, madr);
        setReg(ch + 0x20u, tagQwc);
        setReg(ch + 0x30u, tadr);
    }
    return false;
}

void Ps2Ipu::fillInput()
{
    // The input FIFO (8 qwords) and the 2-qword bit buffer fill as soon as data is available.
    Qword q{};
    while (bufferedInputQwords() < kInFifoQwords + 2u && pullInputQword(q))
    {
        m_bits->append(q.data());
        m_topValid = false;
    }
}

void Ps2Ipu::drainOutput()
{
    const uint32_t ch = kFromIpuChannel;
    while (!m_output.empty() && channelActive(ch))
    {
        const uint32_t qwc = reg(ch + 0x20u) & 0xFFFFu;
        if (qwc == 0u)
        {
            completeChannel(ch, 3u);
            break;
        }
        const uint32_t madr = reg(ch + 0x10u);
        writeMemoryQword(madr, m_output.front());
        m_output.pop_front();
        ++m_stats.outputQwords;
        setReg(ch + 0x10u, madr + 16u);
        setReg(ch + 0x20u, qwc - 1u);
        if (qwc == 1u)
            completeChannel(ch, 3u);
    }
}

void Ps2Ipu::pushOutput(const uint8_t *data, uint32_t bytes)
{
    for (uint32_t off = 0; off < bytes; off += 16u)
    {
        Qword q{};
        std::memcpy(q.data(), data + off, std::min(16u, bytes - off));
        m_output.push_back(q);
    }
}

void Ps2Ipu::dmaControlWritten(uint32_t channelBase)
{
    const uint32_t chcr = reg(channelBase);
    if ((chcr & kDmaStr) == 0u)
        return;
    if (channelBase == kToIpuChannel)
    {
        // A chain (re)start continues from TADR unless CHCR.TAG holds an ending tag (refe, end, or
        // IRQ with TIE): then only the remaining QWC moves. Sony's MPEG library restores a paused
        // chain this way and sets TAG to ref when it wants the chain to go on. With QWC=0 nothing is
        // left of the old tag, so the DMAC reads a new tag from TADR whatever CHCR.TAG holds.
        const uint32_t tagId = (chcr >> 28) & 7u;
        const bool resuming = (reg(channelBase + 0x20u) & 0xFFFFu) != 0u;
        m_toIpuChainEnded =
            resuming && (tagId == 0u || tagId == 7u || ((chcr >> 31) != 0u && (chcr & 0x80u) != 0u));
        m_toIpuAsp = (chcr >> 4) & 3u;
        if (traceEnabled())
            std::fprintf(stderr, "[ipu] toIPU start chcr=0x%08X madr=0x%08X qwc=%u tadr=0x%08X\n", chcr,
                         reg(channelBase + 0x10u), reg(channelBase + 0x20u) & 0xFFFFu, reg(channelBase + 0x30u));
    }
    else if (traceEnabled())
    {
        std::fprintf(stderr, "[ipu] fromIPU start madr=0x%08X qwc=%u (output queued %zu)\n", reg(channelBase + 0x10u),
                     reg(channelBase + 0x20u) & 0xFFFFu, m_output.size());
    }
    process();
}

// ---- commands ------------------------------------------------------------------------------------

void Ps2Ipu::startCommand(uint32_t command)
{
    const uint32_t code = command >> 28;
    if (m_busy)
    {
        if (traceEnabled())
            std::fprintf(stderr, "[ipu] %s 0x%08X ignored: busy with 0x%08X\n", commandName(code), command, m_command);
        return;
    }
    ++m_stats.commands[code];
    if (traceEnabled())
        std::fprintf(stderr, "[ipu] %s 0x%08X (ctrl=0x%08X bp=%u in=%u out=%zu)\n", commandName(code), command, m_ctrl,
                     m_bits->bitPointer(), bufferedInputQwords(), m_output.size());
    m_command = command;
    m_ctrl &= ~(kCtrlEcd | kCtrlScd);
    m_topValid = false;

    switch (code)
    {
    case kCmdBclr:
        m_bits->clear(command & 0x7Fu);
        finishCommand();
        process();
        return;
    case kCmdSetth:
        m_th0 = command & 0x1FFu;
        m_th1 = (command >> 16) & 0x1FFu;
        finishCommand();
        return;
    case kCmdIdec:
        m_idecFirst = true;
        m_state->quantiserScaleCode = (command >> 16) & 31u;
        m_state->resetDcPredictors();
        break;
    case kCmdCsc:
    case kCmdPack:
        m_remaining = command & 0x7FFu;
        break;
    default:
        break;
    }
    m_fbPending = true;
    m_busy = true;
    m_awaitingDrain = false;
    process();
}

void Ps2Ipu::finishCommand()
{
    if (traceEnabled())
        std::fprintf(stderr, "[ipu] done %s data=0x%08X ctrl=0x%08X bp=%u in=%u out=%zu\n", commandName(m_command >> 28),
                     m_cmdData, m_ctrl, m_bits->bitPointer(), bufferedInputQwords(), m_output.size());
    m_busy = false;
    m_awaitingDrain = false;
    m_memory.queueCompletedIntcCause(8u); // INTC IPU
}

void Ps2Ipu::process()
{
    for (int guard = 0; guard < 1 << 20; ++guard)
    {
        drainOutput();
        if (!m_busy)
            break;
        if (m_awaitingDrain)
        {
            if (m_output.size() > kOutFifoQwords)
                break;
            m_bits->compact();
            setTopFromInput();
            finishCommand();
            break;
        }
        if (m_output.size() > kOutFifoQwords)
            break; // output FIFO full: wait for DMA or FIFO reads
        if (!step())
            break;
    }
    if (!m_busy)
        fillInput();
}

namespace
{
    // Runs fn as one transaction: on IpuStarved the bit position and decoder state roll back.
    template <typename Fn>
    bool transact(ps2_ipu::DeviceBitReader &bits, ps2_ipu::DecoderState &state, Fn &&fn)
    {
        const uint64_t pos = bits.position();
        const ps2_ipu::DecoderState saved = state;
        try
        {
            fn();
        }
        catch (const ps2_ipu::IpuStarved &)
        {
            bits.setPosition(pos);
            state = saved;
            return false;
        }
        bits.compact();
        return true;
    }

    void storeBlock(const int32_t pixels[64], int16_t *mb, int block, bool fieldDct, int lo, int hi)
    {
        for (int r = 0; r < 8; ++r)
        {
            for (int c = 0; c < 8; ++c)
            {
                const int16_t v = static_cast<int16_t>(std::clamp(pixels[r * 8 + c], lo, hi));
                if (block < 4)
                {
                    const int x = (block & 1) * 8 + c;
                    const int y = fieldDct ? (block >> 1) + 2 * r : (block >> 1) * 8 + r;
                    mb[y * 16 + x] = v;
                }
                else
                {
                    mb[(block == 4 ? 256 : 320) + r * 8 + c] = v;
                }
            }
        }
    }
}

bool Ps2Ipu::step()
{
    const uint32_t code = m_command >> 28;
    const unsigned fb = m_command & 0x3Fu;
    ps2_ipu::DeviceBitReader &bits = *m_bits;
    ps2_ipu::DecoderState &state = *m_state;

    switch (code)
    {
    case kCmdVdec:
    {
        const uint32_t tbl = (m_command >> 26) & 3u;
        uint32_t result = 0;
        bool error = false;
        const bool ok = transact(bits, state, [&]
        {
            if (m_fbPending)
                bits.skip(fb);
            unsigned len = 0;
            uint32_t value = 0;
            switch (tbl)
            {
            case 0:
                value = ps2_ipu::decodeMacroblockAddressIncrement(bits, state.mpeg1, len);
                error = value == 0u;
                break;
            case 1:
                value = ps2_ipu::decodeMacroblockType(bits, state.pictureType, len);
                error = value == 0u;
                break;
            case 2:
            {
                bool valid = true;
                value = static_cast<uint32_t>(ps2_ipu::decodeMotionCode(bits, len, valid)) & 0xFFFFu;
                error = !valid;
                break;
            }
            default:
                value = static_cast<uint32_t>(ps2_ipu::decodeDmVector(bits, len)) & 0xFFFFu;
                break;
            }
            result = (value & 0xFFFFu) | (len << 16);
        });
        if (!ok)
            return false;
        m_fbPending = false;
        m_cmdData = result;
        if (error)
        {
            m_ctrl |= kCtrlEcd;
            ++m_stats.errors;
        }
        m_awaitingDrain = true;
        return true;
    }
    case kCmdFdec:
    {
        uint32_t result = 0;
        const bool ok = transact(bits, state, [&]
        {
            if (m_fbPending)
                bits.skip(fb);
            result = bits.peek(32);
        });
        if (!ok)
            return false;
        m_fbPending = false;
        m_cmdData = result;
        m_awaitingDrain = true;
        return true;
    }
    case kCmdSetiq:
    {
        uint8_t matrix[64];
        const bool ok = transact(bits, state, [&]
        {
            if (m_fbPending)
                bits.skip(fb);
            for (uint8_t &b : matrix)
                b = static_cast<uint8_t>(bits.get(8));
        });
        if (!ok)
            return false;
        m_fbPending = false;
        std::memcpy((m_command >> 27) & 1u ? state.nonIntraIq : state.intraIq, matrix, 64);
        m_awaitingDrain = true;
        return true;
    }
    case kCmdSetvq:
    {
        uint8_t clut[32];
        const bool ok = transact(bits, state, [&]
        {
            for (uint8_t &b : clut)
                b = static_cast<uint8_t>(bits.get(8));
        });
        if (!ok)
            return false;
        for (int i = 0; i < 16; ++i)
            m_vqClut[static_cast<size_t>(i)] = static_cast<uint16_t>(clut[i * 2] | (clut[i * 2 + 1] << 8));
        m_awaitingDrain = true;
        return true;
    }
    case kCmdBdec:
        return stepBdec();
    case kCmdIdec:
        return stepIdecMacroblock();
    case kCmdCsc:
        return stepCscMacroblock();
    case kCmdPack:
        return stepPackMacroblock();
    default:
        if (traceEnabled())
            std::fprintf(stderr, "[ipu] unknown command 0x%08X\n", m_command);
        m_ctrl |= kCtrlEcd;
        m_awaitingDrain = true;
        return true;
    }
}

bool Ps2Ipu::stepBdec()
{
    const unsigned fb = m_command & 0x3Fu;
    const uint32_t qsc = (m_command >> 16) & 31u;
    const bool fieldDct = (m_command >> 25) & 1u;
    const bool dcReset = (m_command >> 26) & 1u;
    const bool intra = (m_command >> 27) & 1u;
    ps2_ipu::DeviceBitReader &bits = *m_bits;
    ps2_ipu::DecoderState &state = *m_state;

    int16_t mb[384] = {};
    int32_t cbp = 0;
    bool error = false;
    const bool ok = transact(bits, state, [&]
    {
        std::fill(std::begin(mb), std::end(mb), int16_t{0});
        error = false;
        if (m_fbPending)
            bits.skip(fb);
        state.quantiserScaleCode = qsc;
        if (dcReset)
            state.resetDcPredictors();
        cbp = intra ? 63 : ps2_ipu::decodeCodedBlockPattern(bits);
        if (cbp < 0)
        {
            error = true;
            cbp = 0;
            return;
        }
        for (int b = 0; b < 6 && !error; ++b)
        {
            if ((cbp & (32 >> b)) == 0)
                continue;
            int32_t coeffs[64], pixels[64];
            const int cc = b < 4 ? 0 : b - 3;
            const bool decoded = intra ? ps2_ipu::decodeIntraBlock(bits, state, cc, coeffs)
                                       : ps2_ipu::decodeNonIntraBlock(bits, state, coeffs);
            if (!decoded)
            {
                error = true;
                break;
            }
            ps2_ipu::inverseDct(coeffs, pixels);
            storeBlock(pixels, mb, b, fieldDct, intra ? 0 : -256, 255);
        }
    });
    if (!ok)
        return false;
    m_fbPending = false;
    m_ctrl = (m_ctrl & ~(0x3Fu << 8)) | (static_cast<uint32_t>(cbp & 63) << 8);
    if (error)
    {
        m_ctrl |= kCtrlEcd;
        ++m_stats.errors;
    }
    pushOutput(reinterpret_cast<const uint8_t *>(mb), sizeof(mb));
    ++m_stats.macroblocks;
    try
    {
        if (bits.peek(23) == 0u)
            m_ctrl |= kCtrlScd;
    }
    catch (const ps2_ipu::IpuStarved &)
    {
    }
    m_awaitingDrain = true;
    return true;
}

bool Ps2Ipu::stepIdecMacroblock()
{
    const unsigned fb = m_command & 0x3Fu;
    const bool decodeDctType = (m_command >> 24) & 1u; // DTD
    const bool dither = (m_command >> 26) & 1u;        // DTE
    const bool rgb16 = (m_command >> 27) & 1u;         // OFM
    ps2_ipu::DeviceBitReader &bits = *m_bits;
    ps2_ipu::DecoderState &state = *m_state;

    bool endOfSlice = false;
    bool error = false;
    ps2_ipu::MacroblockRaw8 raw{};
    const bool ok = transact(bits, state, [&]
    {
        endOfSlice = false;
        error = false;
        if (m_fbPending)
            bits.skip(fb);
        if (!m_idecFirst)
        {
            // The next macroblock_address_increment, or the end of the slice at a start code.
            uint32_t increment = 0;
            for (;;)
            {
                if (bits.peek(23) == 0u)
                {
                    endOfSlice = true;
                    return;
                }
                unsigned len = 0;
                const uint32_t v = ps2_ipu::decodeMacroblockAddressIncrement(bits, state.mpeg1, len);
                if (v == 0x23u)
                {
                    increment += 33u;
                    continue;
                }
                if (v == 0x22u)
                    continue;
                if (v == 0u)
                {
                    error = true;
                    return;
                }
                increment += v;
                break;
            }
            if (increment > 1u)
                state.resetDcPredictors();
        }
        unsigned len = 0;
        const uint32_t type = ps2_ipu::decodeMacroblockType(bits, state.pictureType == 4u ? 4u : 1u, len);
        if ((type & ps2_ipu::MB_INTRA) == 0u)
        {
            error = true;
            return;
        }
        if (type & ps2_ipu::MB_QUANT)
            state.quantiserScaleCode = bits.get(5);
        const bool fieldDct = decodeDctType && bits.get(1);
        int16_t mb[384] = {};
        for (int b = 0; b < 6; ++b)
        {
            int32_t coeffs[64], pixels[64];
            if (!ps2_ipu::decodeIntraBlock(bits, state, b < 4 ? 0 : b - 3, coeffs))
            {
                error = true;
                return;
            }
            ps2_ipu::inverseDct(coeffs, pixels);
            storeBlock(pixels, mb, b, fieldDct, 0, 255);
        }
        for (int i = 0; i < 256; ++i)
            raw.y[i] = static_cast<uint8_t>(mb[i]);
        for (int i = 0; i < 64; ++i)
        {
            raw.cb[i] = static_cast<uint8_t>(mb[256 + i]);
            raw.cr[i] = static_cast<uint8_t>(mb[320 + i]);
        }
    });
    if (!ok)
        return false;
    m_fbPending = false;
    if (endOfSlice || error)
    {
        if (endOfSlice)
            m_ctrl |= kCtrlScd;
        if (error)
        {
            m_ctrl |= kCtrlEcd;
            ++m_stats.errors;
        }
        m_awaitingDrain = true;
        return true;
    }
    m_idecFirst = false;
    uint32_t rgba[256];
    ps2_ipu::macroblockToRgb32(raw, m_th0, m_th1, rgba);
    if (rgb16)
    {
        uint16_t out[256];
        ps2_ipu::rgb32ToRgb16(rgba, dither, out);
        pushOutput(reinterpret_cast<const uint8_t *>(out), sizeof(out));
    }
    else
    {
        pushOutput(reinterpret_cast<const uint8_t *>(rgba), sizeof(rgba));
    }
    ++m_stats.macroblocks;
    return true;
}

namespace
{
    // PS2_IPU_DUMP_CSC=<n>: write the first n 640x416 pictures that CSC produces (macroblocks in
    // raster order) as ipu_csc_NN.ppm in the working directory.
    void debugDumpCsc(const uint32_t rgba[256])
    {
        static const int frames = []
        {
            const char *v = std::getenv("PS2_IPU_DUMP_CSC");
            return v ? std::atoi(v) : 0;
        }();
        static std::vector<uint32_t> picture(640u * 416u);
        static uint32_t mb = 0;
        static int written = 0;
        if (written >= frames)
            return;
        const uint32_t mx = mb % 40u, my = mb / 40u;
        for (uint32_t y = 0; y < 16; ++y)
            for (uint32_t x = 0; x < 16; ++x)
                picture[(my * 16u + y) * 640u + mx * 16u + x] = rgba[y * 16u + x];
        if (++mb < 40u * 26u)
            return;
        mb = 0;
        char name[64];
        std::snprintf(name, sizeof(name), "ipu_csc_%02d.ppm", written++);
        if (FILE *f = std::fopen(name, "wb"))
        {
            std::fprintf(f, "P6\n640 416\n255\n");
            for (uint32_t px : picture)
            {
                const uint8_t c[3] = {static_cast<uint8_t>(px), static_cast<uint8_t>(px >> 8), static_cast<uint8_t>(px >> 16)};
                std::fwrite(c, 1, 3, f);
            }
            std::fclose(f);
        }
    }
}

bool Ps2Ipu::stepCscMacroblock()
{
    if (m_remaining == 0u)
    {
        m_awaitingDrain = true;
        return true;
    }
    const bool dither = (m_command >> 26) & 1u;
    const bool rgb16 = (m_command >> 27) & 1u;
    ps2_ipu::MacroblockRaw8 raw{};
    const bool ok = transact(*m_bits, *m_state, [&]
    {
        uint8_t *dst = reinterpret_cast<uint8_t *>(&raw);
        for (size_t i = 0; i < sizeof(raw); i += 4)
        {
            const uint32_t v = m_bits->get(32);
            dst[i] = static_cast<uint8_t>(v >> 24);
            dst[i + 1] = static_cast<uint8_t>(v >> 16);
            dst[i + 2] = static_cast<uint8_t>(v >> 8);
            dst[i + 3] = static_cast<uint8_t>(v);
        }
    });
    if (!ok)
        return false;
    uint32_t rgba[256];
    ps2_ipu::macroblockToRgb32(raw, m_th0, m_th1, rgba);
    debugDumpCsc(rgba);
    if (rgb16)
    {
        uint16_t out[256];
        ps2_ipu::rgb32ToRgb16(rgba, dither, out);
        pushOutput(reinterpret_cast<const uint8_t *>(out), sizeof(out));
    }
    else
    {
        pushOutput(reinterpret_cast<const uint8_t *>(rgba), sizeof(rgba));
    }
    ++m_stats.macroblocks;
    if (--m_remaining == 0u)
        m_awaitingDrain = true;
    return true;
}

bool Ps2Ipu::stepPackMacroblock()
{
    if (m_remaining == 0u)
    {
        m_awaitingDrain = true;
        return true;
    }
    const bool dither = (m_command >> 26) & 1u;
    const bool rgb16 = (m_command >> 27) & 1u; // OFM: 1 RGB16, 0 INDX4
    uint32_t rgba[256];
    const bool ok = transact(*m_bits, *m_state, [&]
    {
        for (uint32_t &px : rgba)
        {
            const uint32_t v = m_bits->get(32);
            px = (v >> 24) | ((v >> 8) & 0xFF00u) | ((v << 8) & 0xFF0000u) | (v << 24); // memory order
        }
    });
    if (!ok)
        return false;
    uint16_t out16[256];
    ps2_ipu::rgb32ToRgb16(rgba, dither, out16);
    if (rgb16)
    {
        pushOutput(reinterpret_cast<const uint8_t *>(out16), sizeof(out16));
    }
    else
    {
        // INDX4: the nearest SETVQ CLUT entry per pixel, two pixels per byte (low nibble first).
        uint8_t out[128] = {};
        for (int i = 0; i < 256; ++i)
        {
            int best = 0, bestDist = 1 << 30;
            for (int c = 0; c < 16; ++c)
            {
                const uint16_t e = m_vqClut[static_cast<size_t>(c)];
                const int dr = static_cast<int>(e & 31) - (out16[i] & 31);
                const int dg = static_cast<int>((e >> 5) & 31) - ((out16[i] >> 5) & 31);
                const int db = static_cast<int>((e >> 10) & 31) - ((out16[i] >> 10) & 31);
                const int d = dr * dr + dg * dg + db * db;
                if (d < bestDist)
                {
                    bestDist = d;
                    best = c;
                }
            }
            out[i >> 1] |= static_cast<uint8_t>(best << ((i & 1) * 4));
        }
        pushOutput(out, sizeof(out));
    }
    ++m_stats.macroblocks;
    if (--m_remaining == 0u)
        m_awaitingDrain = true;
    return true;
}
