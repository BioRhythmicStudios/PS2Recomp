#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace ps2x::iop::detail
{
    // SPU2 sound processor: 2 MB of sound RAM and two cores of 24 voices, mixed at 48 kHz.
    // Registers are addressed by their byte offset from 0x1F900000 (0xBF900000 through KSEG1), as the IOP sees
    // them. Sound RAM addresses are byte addresses; the registers hold halfword (16-bit word) addresses.
    class IopSpu2
    {
    public:
        static constexpr uint32_t RamBytes = 2u * 1024u * 1024u;
        static constexpr uint32_t SampleRate = 48000u;
        static constexpr uint32_t IopCyclesPerSample = 768u; // 36.864 MHz / 48 kHz
        static constexpr uint32_t VoicesPerCore = 24u;

        IopSpu2();

        void reset();

        [[nodiscard]] uint16_t read16(uint32_t offset) const;
        void write16(uint32_t offset, uint16_t value);

        void writeRam(uint32_t byteAddress, const uint8_t *data, size_t size);
        void readRam(uint32_t byteAddress, uint8_t *data, size_t size) const;
        void zeroRam(uint32_t byteAddress, size_t size);

        // Mixes `frames` output samples and appends them to `out` as interleaved signed 16-bit L/R.
        void run(uint32_t frames, std::vector<int16_t> &out);

        [[nodiscard]] uint64_t samplesMixed() const noexcept { return m_samples; }
        [[nodiscard]] uint32_t activeVoices() const noexcept;
        [[nodiscard]] uint64_t keyOns() const noexcept { return m_keyOns; }

        // The envelope and interpolation helpers are public for unit tests.
        struct Envelope
        {
            int32_t level = 0;
            int32_t counter = 0;
            uint8_t phase = 0; // 0 off, 1 attack, 2 decay, 3 sustain, 4 release
        };
        static void tickAdsr(Envelope &envelope, uint16_t adsr1, uint16_t adsr2);
        static int16_t decodeAdpcmBlock(const uint8_t block[16], int16_t &history1, int16_t &history2,
                                        int16_t out[28]);
        [[nodiscard]] static const std::array<int16_t, 512> &interpolationTable();

    private:
        struct Volume
        {
            uint16_t reg = 0;
            int32_t current = 0; // -0x8000..0x7FFF
            int32_t counter = 0;
            void write(uint16_t value);
            void tick();
        };

        struct Voice
        {
            Volume left;
            Volume right;
            uint16_t pitch = 0;
            uint16_t adsr1 = 0;
            uint16_t adsr2 = 0;
            uint32_t ssa = 0;   // halfword addresses
            uint32_t lsax = 0;
            uint32_t block = 0; // halfword address of the current 8-halfword ADPCM block
            uint32_t counter = 0; // pitch counter, 12-bit fraction
            uint8_t sampleIndex = 0;
            uint8_t flags = 0;
            bool needFetch = true;
            bool lsaxWritten = false;
            int16_t history1 = 0;
            int16_t history2 = 0;
            std::array<int16_t, 28> decoded{};
            std::array<int16_t, 4> previous{}; // previous[0] is the newest
            Envelope envelope;
            int32_t output = 0; // last voice output (after envelope), drives pitch modulation
        };

        struct Core
        {
            std::array<Voice, VoicesPerCore> voices;
            uint32_t endx = 0;
            uint32_t pmon = 0;
            uint32_t non = 0;
            uint32_t vmixl = 0;
            uint32_t vmixr = 0;
            uint32_t vmixel = 0;
            uint32_t vmixer = 0;
            uint16_t mmix = 0;
            uint16_t attr = 0;
            Volume masterLeft;
            Volume masterRight;
            int32_t effectLeft = 0;  // EVOL
            int32_t effectRight = 0;
            int32_t externalLeft = 0; // AVOL
            int32_t externalRight = 0;
            int32_t inputLeft = 0;    // BVOL
            int32_t inputRight = 0;
            int32_t noiseLevel = 0;
            int32_t noiseCounter = 0;
        };

        void keyOn(Core &core, uint32_t bits);
        void keyOff(Core &core, uint32_t bits);
        int16_t nextSample(Core &core, uint32_t coreIndex, uint32_t voiceIndex);
        void fetchBlock(Voice &voice);
        void tickNoise(Core &core);
        void mixCore(uint32_t coreIndex, int32_t externalLeft, int32_t externalRight, int32_t &outLeft,
                     int32_t &outRight);
        [[nodiscard]] uint16_t voiceNax(const Voice &voice, bool high) const;

        std::vector<uint8_t> m_ram;
        std::array<uint16_t, 0x400> m_regs{}; // raw shadow of every register halfword
        std::array<Core, 2> m_cores;
        uint64_t m_samples = 0;
        uint64_t m_keyOns = 0;
    };
}
