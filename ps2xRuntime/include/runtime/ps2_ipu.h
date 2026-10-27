#ifndef PS2_IPU_H
#define PS2_IPU_H

// Image Processing Unit (MPEG decoder): IPU_CMD/CTRL/BP/TOP at 0x10002000, the input FIFO at
// 0x10007010, the output FIFO at 0x10007000, DMA channel 4 (toIPU) and channel 3 (fromIPU).
//
// Commands execute synchronously in the EE access that makes progress possible (a command write,
// FIFO access or DMA start). A decode that runs out of input stays BUSY at its last completed
// macroblock and resumes when more data arrives; output beyond the 8-qword FIFO holds the
// command BUSY until DMA or FIFO reads drain it, as on hardware. Each completed command raises
// INTC cause 8 (IPU); each finished DMA transfer raises the DMAC channel cause.

#include <array>
#include <cstdint>
#include <deque>
#include <memory>

class PS2Memory;

namespace ps2_ipu
{
    class DeviceBitReader;
    struct DecoderState;
}

class Ps2Ipu
{
public:
    explicit Ps2Ipu(PS2Memory &memory);
    ~Ps2Ipu();

    static constexpr uint32_t kBase = 0x10002000u;
    static constexpr uint32_t kOutFifo = 0x10007000u;
    static constexpr uint32_t kInFifo = 0x10007010u;
    static constexpr uint32_t kFromIpuChannel = 0x1000B000u; // DMA channel 3
    static constexpr uint32_t kToIpuChannel = 0x1000B400u;   // DMA channel 4

    static bool isRegister(uint32_t address) { return address >= kBase && address < kBase + 0x40u; }

    uint32_t readRegister32(uint32_t address);
    uint64_t readRegister64(uint32_t address);
    void writeRegister32(uint32_t address, uint32_t value);

    void writeInputFifo(const uint8_t qword[16]);
    void readOutputFifo(uint8_t qword[16]);

    // Called after the EE wrote CHCR of channel 3 or 4 (the value is already in the IO registers).
    void dmaControlWritten(uint32_t channelBase);

    struct Stats
    {
        uint64_t commands[16] = {};
        uint64_t macroblocks = 0;
        uint64_t errors = 0;
        uint64_t inputQwords = 0;
        uint64_t outputQwords = 0;
    };
    const Stats &stats() const { return m_stats; }

private:
    friend class ps2_ipu::DeviceBitReader;

    using Qword = std::array<uint8_t, 16>;

    void reset();
    void startCommand(uint32_t command);
    void process();
    bool step(); // one unit of the current command; false when it cannot progress
    void finishCommand();
    bool pullInputQword(Qword &out);
    void fillInput();
    void drainOutput();
    void pushOutput(const uint8_t *data, uint32_t bytes);
    uint32_t bufferedInputQwords() const;

    bool stepBdec();
    bool stepIdecMacroblock();
    bool stepCscMacroblock();
    bool stepPackMacroblock();
    void setTopFromInput();

    // DMA helpers (registers live in PS2Memory's IO map).
    uint32_t reg(uint32_t address) const;
    void setReg(uint32_t address, uint32_t value);
    bool channelActive(uint32_t channelBase) const;
    void completeChannel(uint32_t channelBase, uint32_t cause);
    bool readMemoryQword(uint32_t address, Qword &out);
    bool writeMemoryQword(uint32_t address, const Qword &data);

    PS2Memory &m_memory;
    std::unique_ptr<ps2_ipu::DeviceBitReader> m_bits;
    std::unique_ptr<ps2_ipu::DecoderState> m_state;
    std::deque<Qword> m_output;

    uint32_t m_ctrl = 0;        // IPU_CTRL writable/status fields (IFC/OFC/BUSY computed on read)
    uint32_t m_cmdData = 0;     // IPU_CMD.DATA
    uint32_t m_top = 0;         // IPU_TOP.BSTOP
    bool m_topValid = false;
    uint32_t m_command = 0;     // the command word being executed
    bool m_busy = false;
    bool m_awaitingDrain = false; // computation done, waiting for output to fit the FIFO
    uint32_t m_remaining = 0;   // macroblocks left for CSC/PACK/IDEC (IDEC: unbounded)
    bool m_idecFirst = true;
    bool m_fbPending = false;   // the command's FB bits are still to be skipped
    uint32_t m_th0 = 0, m_th1 = 0;
    std::array<uint16_t, 16> m_vqClut{};

    // toIPU source-chain state
    bool m_toIpuChainEnded = false;
    uint32_t m_toIpuAsp = 0;

    Stats m_stats;
};

#endif // PS2_IPU_H
