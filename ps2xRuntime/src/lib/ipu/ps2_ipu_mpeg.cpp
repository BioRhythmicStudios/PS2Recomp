#include "ps2_ipu_mpeg.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace ps2_ipu
{
    namespace
    {
        // ---- VLC tables (ISO/IEC 13818-2 Annex B). Codes are written MSB first. -------------------

        struct Code
        {
            const char *bits;
            int value;
        };

        // B.1 macroblock_address_increment. 34 = macroblock_stuffing, 35 = macroblock_escape.
        constexpr Code kMbai[] = {
            {"1", 1}, {"011", 2}, {"010", 3}, {"0011", 4}, {"0010", 5}, {"00011", 6}, {"00010", 7},
            {"0000111", 8}, {"0000110", 9}, {"00001011", 10}, {"00001010", 11}, {"00001001", 12},
            {"00001000", 13}, {"00000111", 14}, {"00000110", 15}, {"0000010111", 16}, {"0000010110", 17},
            {"0000010101", 18}, {"0000010100", 19}, {"0000010011", 20}, {"0000010010", 21},
            {"00000100011", 22}, {"00000100010", 23}, {"00000100001", 24}, {"00000100000", 25},
            {"00000011111", 26}, {"00000011110", 27}, {"00000011101", 28}, {"00000011100", 29},
            {"00000011011", 30}, {"00000011010", 31}, {"00000011001", 32}, {"00000011000", 33},
            {"00000001111", 34}, {"00000001000", 35},
        };

        // B.2-B.4 macroblock_type for I, P and B pictures (MacroblockType flags).
        constexpr Code kMbTypeI[] = {{"1", 0x01}, {"01", 0x11}};
        constexpr Code kMbTypeP[] = {
            {"1", 0x0A}, {"01", 0x02}, {"001", 0x08}, {"00011", 0x01},
            {"00010", 0x1A}, {"00001", 0x12}, {"000001", 0x11},
        };
        constexpr Code kMbTypeB[] = {
            {"10", 0x0C}, {"11", 0x0E}, {"010", 0x04}, {"011", 0x06}, {"0010", 0x08}, {"0011", 0x0A},
            {"00011", 0x01}, {"00010", 0x1E}, {"000011", 0x1A}, {"000010", 0x16}, {"000001", 0x11},
        };
        constexpr Code kMbTypeD[] = {{"1", 0x01}};

        // B.9 coded_block_pattern (4:2:0).
        constexpr Code kCbp[] = {
            {"111", 60}, {"1101", 4}, {"1100", 8}, {"1011", 16}, {"1010", 32}, {"10011", 12}, {"10010", 48},
            {"10001", 20}, {"10000", 40}, {"01111", 28}, {"01110", 44}, {"01101", 52}, {"01100", 56},
            {"01011", 1}, {"01010", 61}, {"01001", 2}, {"01000", 62}, {"001111", 24}, {"001110", 36},
            {"001101", 3}, {"001100", 63}, {"0010111", 5}, {"0010110", 9}, {"0010101", 17}, {"0010100", 33},
            {"0010011", 6}, {"0010010", 10}, {"0010001", 18}, {"0010000", 34}, {"00011111", 7},
            {"00011110", 11}, {"00011101", 19}, {"00011100", 35}, {"00011011", 13}, {"00011010", 49},
            {"00011001", 21}, {"00011000", 41}, {"00010111", 14}, {"00010110", 50}, {"00010101", 22},
            {"00010100", 42}, {"00010011", 15}, {"00010010", 51}, {"00010001", 23}, {"00010000", 43},
            {"00001111", 25}, {"00001110", 37}, {"00001101", 26}, {"00001100", 38}, {"00001011", 29},
            {"00001010", 45}, {"00001001", 53}, {"00001000", 57}, {"00000111", 30}, {"00000110", 46},
            {"00000101", 54}, {"00000100", 58}, {"000000111", 31}, {"000000110", 47}, {"000000101", 55},
            {"000000100", 59}, {"000000011", 27}, {"000000010", 39}, {"000000001", 0},
        };

        // B.10 motion_code magnitude (a sign bit follows every nonzero code).
        constexpr Code kMotionCode[] = {
            {"1", 0}, {"01", 1}, {"001", 2}, {"0001", 3}, {"000011", 4}, {"0000101", 5}, {"0000100", 6},
            {"0000011", 7}, {"000001011", 8}, {"000001010", 9}, {"000001001", 10}, {"0000010001", 11},
            {"0000010000", 12}, {"0000001111", 13}, {"0000001110", 14}, {"0000001101", 15},
            {"0000001100", 16},
        };

        // B.12/B.13 dct_dc_size.
        constexpr Code kDcSizeLuma[] = {
            {"100", 0}, {"00", 1}, {"01", 2}, {"101", 3}, {"110", 4}, {"1110", 5}, {"11110", 6},
            {"111110", 7}, {"1111110", 8}, {"11111110", 9}, {"111111110", 10}, {"111111111", 11},
        };
        constexpr Code kDcSizeChroma[] = {
            {"00", 0}, {"01", 1}, {"10", 2}, {"110", 3}, {"1110", 4}, {"11110", 5}, {"111110", 6},
            {"1111110", 7}, {"11111110", 8}, {"111111110", 9}, {"1111111110", 10}, {"1111111111", 11},
        };

        // B.14/B.15 DCT coefficients; the sign bit is not part of the code. run = kEob / kEscape
        // mark the special codes. "11" (0,1) is the dct_coeff_next form; the first coefficient of a
        // non-intra block uses "1" (handled by the caller).
        constexpr int kEob = -1;
        constexpr int kEscape = -2;
        struct DctCode
        {
            const char *bits;
            int run;
            int level;
        };
        constexpr DctCode kDctB14[] = {
            {"10", kEob, 0}, {"11", 0, 1}, {"011", 1, 1}, {"0100", 0, 2}, {"0101", 2, 1}, {"00101", 0, 3},
            {"00111", 3, 1}, {"00110", 4, 1}, {"000110", 1, 2}, {"000111", 5, 1}, {"000101", 6, 1},
            {"000100", 7, 1}, {"0000110", 0, 4}, {"0000100", 2, 2}, {"0000111", 8, 1}, {"0000101", 9, 1},
            {"000001", kEscape, 0}, {"00100110", 0, 5}, {"00100001", 0, 6}, {"00100101", 1, 3},
            {"00100100", 3, 2}, {"00100111", 10, 1}, {"00100011", 11, 1}, {"00100010", 12, 1},
            {"00100000", 13, 1}, {"0000001010", 0, 7}, {"0000001100", 1, 4}, {"0000001011", 2, 3},
            {"0000001111", 4, 2}, {"0000001001", 5, 2}, {"0000001110", 14, 1}, {"0000001101", 15, 1},
            {"0000001000", 16, 1}, {"000000011101", 0, 8}, {"000000011000", 0, 9}, {"000000010011", 0, 10},
            {"000000010000", 0, 11}, {"000000011011", 1, 5}, {"000000010100", 2, 4}, {"000000011100", 3, 3},
            {"000000010010", 4, 3}, {"000000011110", 6, 2}, {"000000010101", 7, 2}, {"000000010001", 8, 2},
            {"000000011111", 17, 1}, {"000000011010", 18, 1}, {"000000011001", 19, 1},
            {"000000010111", 20, 1}, {"000000010110", 21, 1}, {"0000000011010", 0, 12},
            {"0000000011001", 0, 13}, {"0000000011000", 0, 14}, {"0000000010111", 0, 15},
            {"0000000010110", 1, 6}, {"0000000010101", 1, 7}, {"0000000010100", 2, 5},
            {"0000000010011", 3, 4}, {"0000000010010", 5, 3}, {"0000000010001", 9, 2},
            {"0000000010000", 10, 2}, {"0000000011111", 22, 1}, {"0000000011110", 23, 1},
            {"0000000011101", 24, 1}, {"0000000011100", 25, 1}, {"0000000011011", 26, 1},
            {"00000000011111", 0, 16}, {"00000000011110", 0, 17}, {"00000000011101", 0, 18},
            {"00000000011100", 0, 19}, {"00000000011011", 0, 20}, {"00000000011010", 0, 21},
            {"00000000011001", 0, 22}, {"00000000011000", 0, 23}, {"00000000010111", 0, 24},
            {"00000000010110", 0, 25}, {"00000000010101", 0, 26}, {"00000000010100", 0, 27},
            {"00000000010011", 0, 28}, {"00000000010010", 0, 29}, {"00000000010001", 0, 30},
            {"00000000010000", 0, 31}, {"000000000011000", 0, 32}, {"000000000010111", 0, 33},
            {"000000000010110", 0, 34}, {"000000000010101", 0, 35}, {"000000000010100", 0, 36},
            {"000000000010011", 0, 37}, {"000000000010010", 0, 38}, {"000000000010001", 0, 39},
            {"000000000010000", 0, 40}, {"000000000011111", 1, 8}, {"000000000011110", 1, 9},
            {"000000000011101", 1, 10}, {"000000000011100", 1, 11}, {"000000000011011", 1, 12},
            {"000000000011010", 1, 13}, {"000000000011001", 1, 14}, {"0000000000010011", 1, 15},
            {"0000000000010010", 1, 16}, {"0000000000010001", 1, 17}, {"0000000000010000", 1, 18},
            {"0000000000010100", 6, 3}, {"0000000000011010", 11, 2}, {"0000000000011001", 12, 2},
            {"0000000000011000", 13, 2}, {"0000000000010111", 14, 2}, {"0000000000010110", 15, 2},
            {"0000000000010101", 16, 2}, {"0000000000011111", 27, 1}, {"0000000000011110", 28, 1},
            {"0000000000011101", 29, 1}, {"0000000000011100", 30, 1}, {"0000000000011011", 31, 1},
        };
        // B.15 lists only the codes that differ from B.14; codes of 13 bits and more are shared,
        // except those whose (run, level) moved to a shorter B.15 code.
        constexpr DctCode kDctB15Short[] = {
            {"0110", kEob, 0}, {"10", 0, 1}, {"010", 1, 1}, {"110", 0, 2}, {"00101", 2, 1}, {"0111", 0, 3},
            {"00111", 3, 1}, {"000110", 4, 1}, {"00110", 1, 2}, {"000111", 5, 1}, {"0000110", 6, 1},
            {"0000100", 7, 1}, {"11100", 0, 4}, {"0000111", 2, 2}, {"0000101", 8, 1}, {"1111000", 9, 1},
            {"000001", kEscape, 0}, {"11101", 0, 5}, {"000101", 0, 6}, {"1111001", 1, 3},
            {"00100110", 3, 2}, {"1111010", 10, 1}, {"00100001", 11, 1}, {"00100101", 12, 1},
            {"00100100", 13, 1}, {"000100", 0, 7}, {"00100111", 1, 4}, {"11111100", 2, 3},
            {"11111101", 4, 2}, {"000000100", 5, 2}, {"000000101", 14, 1}, {"000000111", 15, 1},
            {"0000001101", 16, 1}, {"1111011", 0, 8}, {"1111100", 0, 9}, {"00100011", 0, 10},
            {"00100010", 0, 11}, {"00100000", 1, 5}, {"0000001100", 2, 4}, {"000000011100", 3, 3},
            {"000000010010", 4, 3}, {"000000011110", 6, 2}, {"000000010101", 7, 2}, {"000000010001", 8, 2},
            {"000000011111", 17, 1}, {"000000011010", 18, 1}, {"000000011001", 19, 1},
            {"000000010111", 20, 1}, {"000000010110", 21, 1}, {"11111010", 0, 12}, {"11111011", 0, 13},
            {"11111110", 0, 14}, {"11111111", 0, 15},
        };

        // ---- Lookup tables built from the code lists. ---------------------------------------------

        constexpr unsigned kLutBits = 16;

        struct LutEntry
        {
            int16_t value = 0; // or run for DCT tables
            int16_t level = 0;
            uint8_t len = 0;   // 0 = invalid
        };

        uint32_t codeValue(const char *bits, unsigned &len)
        {
            uint32_t v = 0;
            len = 0;
            for (const char *p = bits; *p; ++p, ++len)
                v = (v << 1) | static_cast<uint32_t>(*p == '1');
            return v;
        }

        // Fills lut (indexed by the next kLutBits bits). Returns false if two codes overlap.
        bool fillLut(std::vector<LutEntry> &lut, const char *bits, int value, int level)
        {
            unsigned len = 0;
            const uint32_t code = codeValue(bits, len);
            const uint32_t first = code << (kLutBits - len);
            const uint32_t count = 1u << (kLutBits - len);
            bool ok = true;
            for (uint32_t i = first; i < first + count; ++i)
            {
                if (lut[i].len != 0)
                    ok = false;
                lut[i].value = static_cast<int16_t>(value);
                lut[i].level = static_cast<int16_t>(level);
                lut[i].len = static_cast<uint8_t>(len);
            }
            return ok;
        }

        struct Tables
        {
            std::vector<LutEntry> mbai, mbTypeI, mbTypeP, mbTypeB, mbTypeD, cbp, motion, dcLuma, dcChroma, b14, b15;
            std::vector<const char *> errors;

            template <size_t N>
            void build(std::vector<LutEntry> &lut, const Code (&codes)[N], const char *name)
            {
                lut.assign(1u << kLutBits, LutEntry{});
                for (const Code &c : codes)
                    if (!fillLut(lut, c.bits, c.value, 0))
                        errors.push_back(name);
            }
            template <size_t N>
            void buildDct(std::vector<LutEntry> &lut, const DctCode (&codes)[N], const char *name)
            {
                for (const DctCode &c : codes)
                    if (!fillLut(lut, c.bits, c.run, c.level))
                        errors.push_back(name);
            }

            Tables()
            {
                build(mbai, kMbai, "macroblock_address_increment");
                build(mbTypeI, kMbTypeI, "macroblock_type I");
                build(mbTypeP, kMbTypeP, "macroblock_type P");
                build(mbTypeB, kMbTypeB, "macroblock_type B");
                build(mbTypeD, kMbTypeD, "macroblock_type D");
                build(cbp, kCbp, "coded_block_pattern");
                build(motion, kMotionCode, "motion_code");
                build(dcLuma, kDcSizeLuma, "dct_dc_size_luminance");
                build(dcChroma, kDcSizeChroma, "dct_dc_size_chrominance");
                b14.assign(1u << kLutBits, LutEntry{});
                buildDct(b14, kDctB14, "DCT B.14");

                // B.15: the short codes, then the shared long B.14 codes whose (run, level) did
                // not move to a short B.15 code.
                b15.assign(1u << kLutBits, LutEntry{});
                buildDct(b15, kDctB15Short, "DCT B.15");
                for (const DctCode &c : kDctB14)
                {
                    unsigned len = 0;
                    codeValue(c.bits, len);
                    if (len < 13)
                        continue;
                    bool moved = false;
                    for (const DctCode &s : kDctB15Short)
                        moved = moved || (s.run == c.run && s.level == c.level);
                    if (!moved && !fillLut(b15, c.bits, c.run, c.level))
                        errors.push_back("DCT B.15 (shared)");
                }
            }
        };

        const Tables &tables()
        {
            static const Tables t;
            return t;
        }

        const LutEntry &lookup(BitReader &bits, const std::vector<LutEntry> &lut)
        {
            // Peek only what is available at the end of the input, so a short final code still decodes.
            uint32_t window;
            try
            {
                window = bits.peek(kLutBits);
            }
            catch (const IpuStarved &)
            {
                const unsigned have = static_cast<unsigned>(std::min<uint64_t>(bits.available(), kLutBits));
                window = have ? (bits.peek(have) << (kLutBits - have)) : 0u;
                const LutEntry &e = lut[window];
                if (e.len == 0 || e.len > have)
                    throw; // the code needs more input
                return e;
            }
            return lut[window];
        }

        constexpr uint8_t kScanZigzag[64] = {
            0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
            12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
            35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
            58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
        };
        constexpr uint8_t kScanAlternate[64] = {
            0, 8, 16, 24, 1, 9, 2, 10, 17, 25, 32, 40, 48, 56, 57, 49,
            41, 33, 26, 18, 3, 11, 4, 12, 19, 27, 34, 42, 50, 58, 35, 43,
            51, 59, 20, 28, 5, 13, 6, 14, 21, 29, 36, 44, 52, 60, 37, 45,
            53, 61, 22, 30, 7, 15, 23, 31, 38, 46, 54, 62, 39, 47, 55, 63,
        };
        constexpr int kNonLinearQuantiserScale[32] = {
            0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16, 18, 20, 22,
            24, 28, 32, 36, 40, 44, 48, 52, 56, 64, 72, 80, 88, 96, 104, 112,
        };

        int32_t saturate(int32_t v) { return std::clamp(v, -2048, 2047); }

        // Reads one run/level pair. Returns false on an invalid code; run = kEob at the end of block.
        bool readCoefficient(BitReader &bits, const DecoderState &state, bool useB15, bool firstNonIntra,
                             int &run, int &level)
        {
            if (firstNonIntra && bits.peek(1) == 1u)
            {
                bits.skip(1);
                run = 0;
                level = bits.get(1) ? -1 : 1;
                return true;
            }
            const LutEntry &e = lookup(bits, useB15 ? tables().b15 : tables().b14);
            if (e.len == 0)
                return false;
            bits.skip(e.len);
            if (e.value == kEob)
            {
                run = kEob;
                return true;
            }
            if (e.value == kEscape)
            {
                run = static_cast<int>(bits.get(6));
                if (state.mpeg1)
                {
                    int l = static_cast<int>(bits.get(8));
                    if (l == 0)
                        l = static_cast<int>(bits.get(8));
                    else if (l == 128)
                        l = static_cast<int>(bits.get(8)) - 256;
                    else if (l > 128)
                        l -= 256;
                    level = l;
                }
                else
                {
                    int l = static_cast<int>(bits.get(12));
                    if (l & 0x800)
                        l -= 0x1000;
                    if (l == 0 || l == -2048)
                        return false;
                    level = l;
                }
                return true;
            }
            run = e.value;
            level = bits.get(1) ? -e.level : e.level;
            return true;
        }

        // Shared AC loop. W is indexed by scan position (the IPU keeps SETIQ data in scan order).
        bool decodeAc(BitReader &bits, DecoderState &state, bool intra, int32_t coeffs[64], int start)
        {
            const uint8_t *scan = state.alternateScan ? kScanAlternate : kScanZigzag;
            const uint8_t *w = intra ? state.intraIq : state.nonIntraIq;
            const int qs = state.quantiserScale();
            const bool useB15 = intra && state.intraVlcFormat && !state.mpeg1;
            int32_t sum = coeffs[0];
            int i = start - 1;
            for (;;)
            {
                int run = 0, level = 0;
                if (!readCoefficient(bits, state, useB15, !intra && i < 0, run, level))
                    return false;
                if (run == kEob)
                    break;
                i += run + 1;
                if (i > 63)
                    return false;
                const int mag = std::abs(level);
                int32_t val = intra ? (mag * qs * w[i]) >> 4 : ((2 * mag + 1) * qs * w[i]) >> 5;
                if (state.mpeg1 && val != 0 && (val & 1) == 0)
                    val -= 1; // MPEG-1 oddification
                if (level < 0)
                    val = -val;
                val = saturate(val);
                coeffs[scan[i]] = val;
                sum += val;
            }
            if (!state.mpeg1 && (sum & 1) == 0)
                coeffs[63] ^= 1; // MPEG-2 mismatch control
            return true;
        }

        struct IdctTable
        {
            double c[8][8]; // c[x][u] = C(u)/2 * cos((2x+1)u*pi/16)
            IdctTable()
            {
                for (int x = 0; x < 8; ++x)
                    for (int u = 0; u < 8; ++u)
                        c[x][u] = (u == 0 ? std::sqrt(0.5) : 1.0) * 0.5 * std::cos((2 * x + 1) * u * 3.14159265358979323846 / 16.0);
            }
        };
    }

    int DecoderState::quantiserScale() const
    {
        const uint32_t code = quantiserScaleCode & 31u;
        return qScaleType && !mpeg1 ? kNonLinearQuantiserScale[code] : static_cast<int>(code * 2u);
    }

    uint32_t decodeMacroblockAddressIncrement(BitReader &bits, bool mpeg1, unsigned &len)
    {
        const LutEntry &e = lookup(bits, tables().mbai);
        len = e.len;
        if (e.len == 0 || (e.value == 34 && !mpeg1))
            return 0u;
        bits.skip(e.len);
        return e.value == 34 ? 0x22u : e.value == 35 ? 0x23u : static_cast<uint32_t>(e.value);
    }

    uint32_t decodeMacroblockType(BitReader &bits, uint32_t pictureType, unsigned &len)
    {
        const std::vector<LutEntry> *lut = nullptr;
        switch (pictureType)
        {
        case 1: lut = &tables().mbTypeI; break;
        case 2: lut = &tables().mbTypeP; break;
        case 3: lut = &tables().mbTypeB; break;
        case 4: lut = &tables().mbTypeD; break;
        default: len = 0; return 0u;
        }
        const LutEntry &e = lookup(bits, *lut);
        len = e.len;
        if (e.len == 0)
            return 0u;
        bits.skip(e.len);
        return static_cast<uint32_t>(e.value);
    }

    int32_t decodeMotionCode(BitReader &bits, unsigned &len, bool &ok)
    {
        const LutEntry &e = lookup(bits, tables().motion);
        ok = e.len != 0;
        len = e.len;
        if (!ok)
            return 0;
        bits.skip(e.len);
        if (e.value == 0)
            return 0;
        ++len;
        return bits.get(1) ? -e.value : e.value;
    }

    int32_t decodeDmVector(BitReader &bits, unsigned &len)
    {
        if (bits.peek(1) == 0u)
        {
            bits.skip(1);
            len = 1;
            return 0;
        }
        len = 2;
        return bits.get(2) == 3u ? -1 : 1;
    }

    int32_t decodeCodedBlockPattern(BitReader &bits)
    {
        const LutEntry &e = lookup(bits, tables().cbp);
        if (e.len == 0)
            return -1;
        bits.skip(e.len);
        return e.value;
    }

    bool decodeIntraBlock(BitReader &bits, DecoderState &state, int cc, int32_t coeffs[64])
    {
        std::fill(coeffs, coeffs + 64, 0);
        const LutEntry &e = lookup(bits, cc == 0 ? tables().dcLuma : tables().dcChroma);
        if (e.len == 0)
            return false;
        bits.skip(e.len);
        const int size = e.value;
        int diff = 0;
        if (size != 0)
        {
            diff = static_cast<int>(bits.get(static_cast<unsigned>(size)));
            if ((diff & (1 << (size - 1))) == 0)
                diff -= (1 << size) - 1;
        }
        state.dcPred[cc] += diff;
        const uint32_t precision = state.mpeg1 ? 0u : state.intraDcPrecision;
        coeffs[0] = state.dcPred[cc] << (3u - precision);
        if (state.mpeg1)
        {
            // MPEG-1 has no EOB after D pictures' DC-only blocks; D pictures are not used by games.
        }
        return decodeAc(bits, state, true, coeffs, 1);
    }

    bool decodeNonIntraBlock(BitReader &bits, DecoderState &state, int32_t coeffs[64])
    {
        std::fill(coeffs, coeffs + 64, 0);
        return decodeAc(bits, state, false, coeffs, 0);
    }

    void inverseDct(const int32_t coeffs[64], int32_t out[64])
    {
        static const IdctTable t;
        double tmp[64];
        // Rows: tmp[v][x] = sum_u c[x][u] F[v][u]
        for (int v = 0; v < 8; ++v)
        {
            const int32_t *row = coeffs + v * 8;
            for (int x = 0; x < 8; ++x)
            {
                double s = 0.0;
                for (int u = 0; u < 8; ++u)
                    if (row[u])
                        s += t.c[x][u] * row[u];
                tmp[v * 8 + x] = s;
            }
        }
        // Columns: out[y][x] = sum_v c[y][v] tmp[v][x]
        for (int x = 0; x < 8; ++x)
        {
            for (int y = 0; y < 8; ++y)
            {
                double s = 0.0;
                for (int v = 0; v < 8; ++v)
                    s += t.c[y][v] * tmp[v * 8 + x];
                out[y * 8 + x] = static_cast<int32_t>(std::floor(s + 0.5));
            }
        }
    }

    void macroblockToRgb32(const MacroblockRaw8 &mb, uint32_t th0, uint32_t th1, uint32_t out[256])
    {
        // BT.601 with the IPU's 7-bit fixed-point coefficients.
        for (int y = 0; y < 16; ++y)
        {
            for (int x = 0; x < 16; ++x)
            {
                const int lum = (static_cast<int>(mb.y[y * 16 + x]) - 16) * 149;
                const int cb = static_cast<int>(mb.cb[(y >> 1) * 8 + (x >> 1)]) - 128;
                const int cr = static_cast<int>(mb.cr[(y >> 1) * 8 + (x >> 1)]) - 128;
                const int r = std::clamp((lum + 204 * cr + 64) >> 7, 0, 255);
                const int g = std::clamp((lum - 50 * cb - 104 * cr + 64) >> 7, 0, 255);
                const int b = std::clamp((lum + 258 * cb + 64) >> 7, 0, 255);
                uint32_t a = 0x80u;
                uint32_t pixel = static_cast<uint32_t>(r) | (static_cast<uint32_t>(g) << 8) | (static_cast<uint32_t>(b) << 16);
                if (static_cast<uint32_t>(r) < th0 && static_cast<uint32_t>(g) < th0 && static_cast<uint32_t>(b) < th0)
                {
                    pixel = 0u;
                    a = 0u;
                }
                else if (static_cast<uint32_t>(r) < th1 && static_cast<uint32_t>(g) < th1 && static_cast<uint32_t>(b) < th1)
                {
                    a = 0x40u;
                }
                out[y * 16 + x] = pixel | (a << 24);
            }
        }
    }

    void rgb32ToRgb16(const uint32_t in[256], bool dither, uint16_t out[256])
    {
        static constexpr int kDither[4][4] = {{-4, 0, -3, 1}, {2, -2, 3, -1}, {-3, 1, -4, 0}, {3, -1, 2, -2}};
        for (int i = 0; i < 256; ++i)
        {
            const int d = dither ? kDither[(i >> 4) & 3][i & 3] : 0;
            const int r = std::clamp(static_cast<int>(in[i] & 0xFF) + d, 0, 255) >> 3;
            const int g = std::clamp(static_cast<int>((in[i] >> 8) & 0xFF) + d, 0, 255) >> 3;
            const int b = std::clamp(static_cast<int>((in[i] >> 16) & 0xFF) + d, 0, 255) >> 3;
            const uint16_t a = (in[i] >> 24) >= 0x40u ? 0x8000u : 0u;
            out[i] = static_cast<uint16_t>(r | (g << 5) | (b << 10) | a);
        }
    }

    bool verifyTables(std::vector<const char *> &errors)
    {
        errors = tables().errors;
        auto isPermutation = [](const uint8_t *scan)
        {
            std::array<bool, 64> seen{};
            for (int i = 0; i < 64; ++i)
            {
                if (scan[i] >= 64 || seen[scan[i]])
                    return false;
                seen[scan[i]] = true;
            }
            return true;
        };
        if (!isPermutation(kScanZigzag))
            errors.push_back("zigzag scan");
        if (!isPermutation(kScanAlternate))
            errors.push_back("alternate scan");
        std::array<bool, 64> cbpSeen{};
        for (const Code &c : kCbp)
            cbpSeen[static_cast<size_t>(c.value)] = true;
        if (std::find(cbpSeen.begin(), cbpSeen.end(), false) != cbpSeen.end())
            errors.push_back("coded_block_pattern values");
        return errors.empty();
    }
}
