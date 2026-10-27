#pragma once

// MPEG-1/2 bitstream primitives used by the IPU: a resumable bit reader, the VLC tables of
// ISO/IEC 13818-2 Annex B, block decoding with inverse quantisation, the IDCT and colour
// conversion. The IPU device (ps2_ipu.cpp) drives these one command (or one macroblock) at a
// time; a decode that runs out of input throws IpuStarved so the device can roll back to the
// saved bit position and retry when more data arrives.

#include <cstdint>
#include <vector>

namespace ps2_ipu
{
    struct IpuStarved
    {
    };

    // Bits are read MSB first in memory byte order, as the IPU reads its input FIFO.
    class BitReader
    {
    public:
        virtual ~BitReader() = default;

        uint32_t peek(unsigned n)
        {
            if (n == 0u)
                return 0u;
            ensure(n);
            const uint64_t bytePos = m_pos >> 3;
            const unsigned shift = static_cast<unsigned>(m_pos & 7u);
            uint64_t window = 0;
            for (unsigned i = 0; i < 5; ++i)
            {
                const uint64_t idx = bytePos + i;
                window = (window << 8) | (idx < m_buf.size() ? m_buf[static_cast<size_t>(idx)] : 0u);
            }
            // shift + n <= 39, so the field lies inside the 40-bit window.
            return static_cast<uint32_t>(window >> (40u - shift - n)) & (n == 32u ? 0xFFFFFFFFu : ((1u << n) - 1u));
        }
        void skip(unsigned n)
        {
            ensure(n);
            m_pos += n;
        }
        uint32_t get(unsigned n)
        {
            const uint32_t v = peek(n);
            m_pos += n;
            return v;
        }
        // Bits available without asking for more input.
        uint64_t available() const { return m_buf.size() * 8u - m_pos; }
        uint64_t position() const { return m_pos; }
        void setPosition(uint64_t pos) { m_pos = pos; }

    protected:
        // Appends input to m_buf; returns false when nothing more is available right now.
        virtual bool more() = 0;
        void ensure(unsigned n)
        {
            while (m_pos + n > m_buf.size() * 8u)
            {
                if (!more())
                    throw IpuStarved{};
            }
        }

        std::vector<uint8_t> m_buf;
        uint64_t m_pos = 0; // bit position from m_buf[0]
    };

    // A reader over a fixed byte range (tests and offline tools).
    class MemoryBitReader : public BitReader
    {
    public:
        MemoryBitReader(const uint8_t *data, size_t size) { m_buf.assign(data, data + size); }

    protected:
        bool more() override { return false; }
    };

    enum MacroblockType : uint32_t
    {
        MB_INTRA = 0x01,
        MB_PATTERN = 0x02,
        MB_BACKWARD = 0x04,
        MB_FORWARD = 0x08,
        MB_QUANT = 0x10,
    };

    // Decoder state the IPU keeps between commands (IPU_CTRL fields, IQ matrices, DC predictors).
    struct DecoderState
    {
        uint32_t intraDcPrecision = 0; // IPU_CTRL.IDP
        bool alternateScan = false;    // IPU_CTRL.AS
        bool intraVlcFormat = false;   // IPU_CTRL.IVF
        bool qScaleType = false;       // IPU_CTRL.QST
        bool mpeg1 = false;            // IPU_CTRL.MP1
        uint32_t pictureType = 0;      // IPU_CTRL.PCT: 1 I, 2 P, 3 B, 4 D
        uint8_t intraIq[64] = {};      // indexed by scan position, as loaded by SETIQ
        uint8_t nonIntraIq[64] = {};
        int32_t dcPred[3] = {128, 128, 128};
        uint32_t quantiserScaleCode = 0;

        void resetDcPredictors()
        {
            dcPred[0] = dcPred[1] = dcPred[2] = 128 << intraDcPrecision;
        }
        int quantiserScale() const;
    };

    // VDEC tables. Each returns the decoded value and sets len to the code length in bits.
    // macroblock_address_increment: 1..33, 0x22 stuffing (MPEG-1), 0x23 escape, 0 = invalid.
    uint32_t decodeMacroblockAddressIncrement(BitReader &bits, bool mpeg1, unsigned &len);
    // macroblock_type flags (MacroblockType) for the picture type; 0 = invalid.
    uint32_t decodeMacroblockType(BitReader &bits, uint32_t pictureType, unsigned &len);
    // motion_code, -16..16. ok is false for an invalid code.
    int32_t decodeMotionCode(BitReader &bits, unsigned &len, bool &ok);
    // dmvector: -1, 0, 1.
    int32_t decodeDmVector(BitReader &bits, unsigned &len);
    // coded_block_pattern (4:2:0), 0..63; -1 = invalid.
    int32_t decodeCodedBlockPattern(BitReader &bits);

    // Decodes one block into dequantised coefficients in raster order. cc: 0 luma, 1 Cb, 2 Cr.
    // Returns false on an invalid code.
    bool decodeIntraBlock(BitReader &bits, DecoderState &state, int cc, int32_t coeffs[64]);
    bool decodeNonIntraBlock(BitReader &bits, DecoderState &state, int32_t coeffs[64]);

    // Separable IDCT (double precision, rounded), no clamping.
    void inverseDct(const int32_t coeffs[64], int32_t out[64]);

    // 16x16 macroblock of 8-bit samples: Y[256], Cb[64], Cr[64].
    struct MacroblockRaw8
    {
        uint8_t y[256];
        uint8_t cb[64];
        uint8_t cr[64];
    };

    // CSC: RAW8 macroblock to 256 RGBA32 pixels (R in the low byte), alpha from TH0/TH1.
    void macroblockToRgb32(const MacroblockRaw8 &mb, uint32_t th0, uint32_t th1, uint32_t out[256]);
    // RGBA32 to RGB16 (A1B5G5R5), optionally dithered.
    void rgb32ToRgb16(const uint32_t in[256], bool dither, uint16_t out[256]);

    // Table self-checks for tests: every code table is prefix-free and decodes to the listed values.
    bool verifyTables(std::vector<const char *> &errors);
}
