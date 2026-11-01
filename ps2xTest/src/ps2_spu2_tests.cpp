#include "MiniTest.h"
#include "../../ps2xIOP/tests/iop_compat_test_support.h"

#include "emulator/core/iop_cpu.h"
#include "emulator/core/iop_memory.h"
#include "emulator/imports/iop_libsd.h"
#include "emulator/spu2/iop_spu2.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <optional>
#include <vector>

namespace
{
    using ps2x::iop::detail::IopCpuState;
    using ps2x::iop::detail::IopLibsd;
    using ps2x::iop::detail::IopMemory;
    using ps2x::iop::detail::IopSpu2;

    // One ADPCM block: shift 12 - shift = `shift`, filter 0, every nibble `nibble`.
    std::vector<uint8_t> constantBlock(uint8_t nibble, uint8_t shift, uint8_t flags)
    {
        std::vector<uint8_t> block(16u, static_cast<uint8_t>(nibble | (nibble << 4u)));
        block[0] = shift;
        block[1] = flags;
        return block;
    }

    struct Libsd
    {
        iop_test::Host host;
        IopMemory memory;
        IopSpu2 spu2;
        IopLibsd libsd{host, memory, spu2};
        uint64_t cycle = 0;

        uint32_t call(uint16_t ordinal, uint32_t a0 = 0, uint32_t a1 = 0, uint32_t a2 = 0, uint32_t a3 = 0)
        {
            IopCpuState cpu{};
            cpu.gpr[4] = a0;
            cpu.gpr[5] = a1;
            cpu.gpr[6] = a2;
            cpu.gpr[7] = a3;
            cpu.gpr[29] = 0x1000u;
            std::optional<IopLibsd::Callback> callback;
            (void)libsd.dispatchImport(ordinal, cpu, cycle, callback);
            lastCallback = callback;
            return cpu.gpr[2];
        }
        std::optional<IopLibsd::Callback> lastCallback;
    };
}

void register_ps2_spu2_tests()
{
    MiniTest::Case("PS2Spu2", [](TestCase &tc)
    {
        tc.Run("ADPCM blocks decode with shift, filter and history", [](TestCase &t)
        {
            int16_t h1 = 0, h2 = 0, out[28];
            const std::vector<uint8_t> plain = constantBlock(0x1u, 0u, 0u);
            t.Equals(static_cast<int>(IopSpu2::decodeAdpcmBlock(plain.data(), h1, h2, out)), 0, "flags byte");
            t.Equals(static_cast<int>(out[0]), 0x1000, "shift 0 places the nibble in bits 15-12");
            t.Equals(static_cast<int>(out[27]), 0x1000, "filter 0 has no prediction");

            std::vector<uint8_t> negative = constantBlock(0x8u, 12u, 0u);
            h1 = h2 = 0;
            IopSpu2::decodeAdpcmBlock(negative.data(), h1, h2, out);
            t.Equals(static_cast<int>(out[0]), -8, "nibbles are signed and shifted right by `shift`");

            // Filter 1 (60/64 of the previous sample) on a zero residual decays the history.
            std::vector<uint8_t> filtered = constantBlock(0x0u, 0x10u, 0u);
            h1 = 1000;
            h2 = 0;
            IopSpu2::decodeAdpcmBlock(filtered.data(), h1, h2, out);
            t.Equals(static_cast<int>(out[0]), (1000 * 60 + 32) >> 6, "filter 1 predicts from the last sample");
            t.Equals(static_cast<int>(h1), static_cast<int>(out[27]), "history carries into the next block");
        });

        tc.Run("ADSR attack, decay, sustain and release follow the rates", [](TestCase &t)
        {
            IopSpu2::Envelope envelope{0, 0, 1};
            const uint16_t adsr1 = 0x000Fu; // fastest linear attack, fastest decay, sustain level 0x8000
            const uint16_t adsr2 = 0x0000u; // fastest sustain increase, fastest linear release
            for (int i = 0; i < 16 && envelope.phase == 1u; ++i)
                IopSpu2::tickAdsr(envelope, adsr1, adsr2);
            t.IsTrue(envelope.phase >= 2u, "rate 0 attack reaches full level within 16 samples");
            t.Equals(envelope.level, 0x7FFF, "attack ends at the maximum level");

            envelope.phase = 4;
            envelope.counter = 0;
            for (int i = 0; i < 64 && envelope.phase != 0u; ++i)
                IopSpu2::tickAdsr(envelope, adsr1, adsr2);
            t.Equals(static_cast<int>(envelope.phase), 0, "a fast release turns the voice off");
            t.Equals(envelope.level, 0, "release ends at zero");

            IopSpu2::Envelope slow{0, 0, 1};
            for (int i = 0; i < 16; ++i)
                IopSpu2::tickAdsr(slow, 0x7F00u, 0u); // slowest attack rate
            t.Equals(static_cast<int>(slow.phase), 1, "a slow attack is still rising");
        });

        tc.Run("ADSR decay and release fields are shifts, not rates", [](TestCase &t)
        {
            // Maximo's Graveyard loop (effect 0x1C4) uses ADSR2 0x1FCF: linear release shift 15, which steps -8 every
            // 1 << (15 - 11) = 16 samples, so full level is gone in 0x7FFF / 8 * 16 = 65,536 samples (1.37 s).
            IopSpu2::Envelope release{0x7FFF, 0, 4};
            int samples = 0;
            for (; samples < 70000 && release.phase != 0u; ++samples)
                IopSpu2::tickAdsr(release, 0x39FFu, 0x1FCFu);
            t.Equals(static_cast<int>(release.phase), 0, "a linear release of shift 15 ends");
            t.IsTrue(samples > 65000 && samples <= 65536, "it takes 16 samples per step of 8");

            // Decay shift 8 (ADSR1 0x0080, sustain level 0x800): exponential steps of -8 * level / 0x8000 every sample.
            IopSpu2::Envelope decay{0x7FFF, 0, 2};
            samples = 0;
            for (; samples < 20000 && decay.phase == 2u; ++samples)
                IopSpu2::tickAdsr(decay, 0x0080u, 0u);
            t.Equals(static_cast<int>(decay.phase), 3, "a decay of shift 8 reaches the sustain level");
            t.IsTrue(samples < 20000, "within a few thousand samples");
        });

        tc.Run("interpolation taps sum to unity gain", [](TestCase &t)
        {
            const auto &table = IopSpu2::interpolationTable();
            for (uint32_t i = 0; i < 256u; ++i)
            {
                const int sum = table[255u - i] + table[511u - i] + table[256u + i] + table[i];
                if (sum != 0x8000)
                {
                    t.Equals(sum, 0x8000, "four taps of a phase sum to 0x8000");
                    return;
                }
            }
            t.IsTrue(table[511] > table[256] && table[256] > table[0], "the kernel peaks at the nearest sample");
        });

        tc.Run("libsd entries address the registers LIBSD uses", [](TestCase &t)
        {
            t.Equals(IopLibsd::paramOffset(0x0207u), 0x434u, "PITCH, core 1, voice 3");
            t.Equals(IopLibsd::paramOffset(0x0981u), 0x788u, "MVOLL, core 1 (master stride 0x28)");
            t.Equals(IopLibsd::paramOffset(0x0E80u), 0x76Au, "AVOLR, core 0");
            t.Equals(IopLibsd::switchOffset(0x1501u), 0x5A0u, "KON, core 1");
            t.Equals(IopLibsd::switchOffset(0x1700u), 0x340u, "ENDX, core 0");
            t.Equals(IopLibsd::addressOffset(0x220Au), 0x204u, "NAX, core 0, voice 5");
            t.Equals(IopLibsd::addressOffset(0x2001u), 0x5C0u, "SSA, core 1, voice 0");
        });

        tc.Run("a keyed voice plays, loops and reports NAX and ENDX through libsd", [](TestCase &t)
        {
            Libsd sd;
            sd.call(4, 0u); // sceSdInit(0)
            // Two blocks at SPU byte address 0x5000: loop start, then loop end + repeat.
            std::vector<uint8_t> sample = constantBlock(0x4u, 0u, 4u);
            const std::vector<uint8_t> second = constantBlock(0x4u, 0u, 3u);
            sample.insert(sample.end(), second.begin(), second.end());
            t.IsTrue(sd.memory.writeRam(0x20000u, sample.data(), sample.size()), "stage the sample in IOP RAM");
            const uint32_t size = static_cast<uint32_t>(sample.size());
            sd.memory.write32(0x1000u + 16u, size); // fifth argument on the stack
            t.Equals(sd.call(17, 0u, 0u, 0x20000u, 0x5000u), size, "sceSdVoiceTrans returns the size");
            t.IsTrue(sd.call(17, 0u, 0u, 0x20000u, 0x5000u) == 0xFFFFFFFFu, "a busy channel refuses a new transfer");
            sd.cycle += 1000000u;

            const uint32_t voice = 2u << 1u; // core 0, voice 2
            sd.call(9, 0x2000u | voice, 0x5000u);     // SSA
            sd.call(5, 0x0200u | voice, 0x1000u);     // PITCH: 48 kHz
            sd.call(5, 0x0300u | voice, 0x000Fu);     // ADSR1: fastest attack
            sd.call(5, 0x0400u | voice, 0x0000u);     // ADSR2
            sd.call(5, 0x0000u | voice, 0x3FFFu);     // VOLL
            sd.call(5, 0x0100u | voice, 0x3FFFu);     // VOLR
            sd.call(5, 0x0980u, 0x3FFFu);             // MVOLL core 0
            sd.call(5, 0x0A80u, 0x3FFFu);             // MVOLR core 0
            sd.call(5, 0x0981u, 0x3FFFu);             // MVOLL core 1
            sd.call(5, 0x0A81u, 0x3FFFu);             // MVOLR core 1
            sd.call(7, 0x1500u, 1u << 2u);            // KON voice 2

            std::vector<int16_t> out;
            sd.spu2.run(80u, out); // two 28-sample blocks at 48 kHz
            const int16_t peak = *std::max_element(out.begin(), out.end());
            t.IsTrue(peak > 0x1000, "the voice is audible at both cores' master volume");
            t.IsTrue((sd.call(8, 0x1700u) & (1u << 2u)) != 0u, "ENDX is set after the loop-end block");
            const uint32_t nax = sd.call(10, 0x2200u | voice);
            t.IsTrue(nax >= 0x5000u && nax < 0x5020u, "NAX stays inside the looping sample");
            t.Equals(static_cast<int>(sd.spu2.activeVoices()), 1, "a repeating loop keeps the voice on");

            sd.call(7, 0x1600u, 1u << 2u); // KOFF
            sd.spu2.run(2000u, out);
            t.Equals(static_cast<int>(sd.spu2.activeVoices()), 0, "key-off releases the voice");
        });

        tc.Run("sceSdInit keeps the samples already in sound RAM", [](TestCase &t)
        {
            // Maximo uploads its resident bank in the front end, then calls sceSdInit(0) again when a level starts.
            Libsd sd;
            sd.call(4, 0u);
            const std::vector<uint8_t> sample = constantBlock(0x4u, 0u, 3u); // loop end + repeat
            t.IsTrue(sd.memory.writeRam(0x20000u, sample.data(), sample.size()), "stage the sample in IOP RAM");
            sd.memory.write32(0x1000u + 16u, static_cast<uint32_t>(sample.size()));
            t.Equals(sd.call(17, 0u, 8u, 0x20000u, 0x30000u), static_cast<uint32_t>(sample.size()),
                     "I/O-mode transfer to SPU 0x30000");
            sd.call(4, 0u); // the level's sceSdInit(0)

            const uint32_t voice = 5u << 1u; // core 0, voice 5
            sd.call(9, 0x2000u | voice, 0x30000u); // SSA
            sd.call(5, 0x0200u | voice, 0x1000u);  // PITCH
            sd.call(5, 0x0300u | voice, 0x000Fu);  // ADSR1
            sd.call(5, 0x0000u | voice, 0x3FFFu);  // VOLL
            sd.call(5, 0x0100u | voice, 0x3FFFu);  // VOLR
            for (const uint32_t master : {0x0980u, 0x0A80u, 0x0981u, 0x0A81u})
                sd.call(5, master, 0x3FFFu);
            sd.call(7, 0x1500u, 1u << 5u); // KON voice 5

            std::vector<int16_t> out;
            sd.spu2.run(80u, out);
            t.IsTrue(*std::max_element(out.begin(), out.end()) > 0x1000, "the sample transferred before sceSdInit plays");
            t.Equals(static_cast<int>(sd.spu2.activeVoices()), 1, "the voice is on");
        });

        tc.Run("DMA voice transfers complete through the transfer callback", [](TestCase &t)
        {
            Libsd sd;
            sd.call(4, 0u);
            t.Equals(sd.call(21, 1u, 0x4321u), 0u, "no previous callback");
            sd.memory.write32(0x1000u + 16u, 0x100u);
            sd.cycle = 50u;
            t.Equals(sd.call(17, 1u, 0u, 0x20000u, 0x8000u), 0x100u, "sceSdVoiceTrans on channel 1");
            t.IsTrue(sd.lastCallback.has_value(), "a DMA transfer schedules its completion");
            t.Equals(sd.lastCallback->function, 0x4321u, "the completion calls the transfer callback");
            t.Equals(sd.lastCallback->argument, 1u, "the callback receives the channel");
            t.IsTrue(sd.lastCallback->cycle > 50u, "completion takes guest time");
            t.Equals(sd.call(19, 1u, 0u), 0u, "sceSdVoiceTransStatus reports the transfer in flight");
            sd.cycle = sd.lastCallback->cycle;
            t.Equals(sd.call(19, 1u, 0u), 1u, "and complete once its cycle is reached");
            sd.memory.write32(0x1000u + 16u, 0x10u);
            t.Equals(sd.call(17, 1u, 8u, 0x20000u, 0x8000u), 0x10u, "an I/O-mode transfer");
            t.IsFalse(sd.lastCallback.has_value(), "programmed I/O completes without an interrupt");
        });

        tc.Run("IOP time spent serving RPCs comes out of the EE-granted clock", [](TestCase &t)
        {
            // The SPU2's sample clock is IOP time, so IOP time must not run ahead of EE time / 8 when an emulated
            // module serves the EE synchronously.
            iop_test::Host host;
            ps2x::iop::IopSubsystem iop(host);
            const uint32_t sid = 0x00012345u;
            iop_test::rpcServer(sid, 0x1234u).install(host, 0x1000u);
            t.IsTrue(iop.loadModuleBuffer(0x1000u).moduleId > 0, "RPC server loads");
            ps2x::iop::RpcRequest request{};
            request.sid = sid;
            request.receive = {0x800u, 4u};
            for (int i = 0; i < 100; ++i)
                t.IsTrue(iop.handleRpc(request).handled, "the server answers");
            const uint64_t served = iop.emulatorCycles();
            t.IsTrue(served > 100u, "serving takes IOP cycles");
            iop.runEeCycles(8u); // one IOP cycle granted: the IOP is already ahead and must not advance
            t.Equals(iop.emulatorCycles(), served, "an IOP ahead of the EE waits");
            iop.runEeCycles(8u * (served + 1000u));
            t.Equals(iop.emulatorCycles(), served + 1001u, "IOP cycles equal the EE's grant once it catches up");
        });

        tc.Run("EE cycles withheld while the IOP idles leave it where stepping would", [](TestCase &t)
        {
            // The runtime hands the IOP its EE cycles only once idleEeCycles() of them have built up.
            const uint32_t sid = 0x00012345u;
            iop_test::Host steppedHost, heldHost;
            ps2x::iop::IopSubsystem stepped(steppedHost), held(heldHost);
            iop_test::rpcServer(sid, 0x1234u).install(steppedHost, 0x1000u);
            iop_test::rpcServer(sid, 0x1234u).install(heldHost, 0x1000u);
            t.IsTrue(stepped.loadModuleBuffer(0x1000u).moduleId > 0, "RPC server loads");
            t.IsTrue(held.loadModuleBuffer(0x1000u).moduleId > 0, "RPC server loads twice");
            ps2x::iop::RpcRequest request{};
            request.sid = sid;
            request.receive = {0x800u, 4u};

            uint64_t pending = 0u, idle = 0u, skipped = 0u;
            for (int checkpoint = 0; checkpoint < 200000; ++checkpoint)
            {
                const uint32_t cycles = 29u + static_cast<uint32_t>(checkpoint % 7);
                stepped.runEeCycles(cycles);
                pending += cycles;
                if (pending >= idle)
                {
                    held.runEeCycles(pending);
                    pending = 0u;
                    idle = held.idleEeCycles();
                }
                else
                {
                    ++skipped;
                }
                if (checkpoint % 5000 == 0)
                {
                    held.runEeCycles(pending); // flushed before any other call
                    pending = 0u;
                    t.IsTrue(stepped.handleRpc(request).handled && held.handleRpc(request).handled, "the server answers");
                    idle = 0u;
                }
            }
            held.runEeCycles(pending);
            t.IsTrue(skipped > 100000u, "most checkpoints are skipped while the server sleeps");
            t.Equals(held.emulatorCycles(), stepped.emulatorCycles(), "same IOP time");
        });

        tc.Run("runToCycle runs the IOP to a clock of its own, and the EE's grant continues from there", [](TestCase &t)
        {
            // The real-time audio clock runs the IOP up to the audio device's position instead of EE time / 8.
            const uint32_t sid = 0x00012345u;
            iop_test::Host host;
            ps2x::iop::IopSubsystem iop(host);
            iop_test::rpcServer(sid, 0x1234u).install(host, 0x1000u);
            t.IsTrue(iop.loadModuleBuffer(0x1000u).moduleId > 0, "RPC server loads");
            const uint64_t start = iop.emulatorCycles();
            iop.runToCycle(start + 50000u);
            t.Equals(iop.emulatorCycles(), start + 50000u, "the IOP reaches the requested cycle");
            iop.runToCycle(start + 10000u);
            t.Equals(iop.emulatorCycles(), start + 50000u, "an earlier cycle changes nothing");
            ps2x::iop::RpcRequest request{};
            request.sid = sid;
            request.receive = {0x800u, 4u};
            t.IsTrue(iop.handleRpc(request).handled, "the server answers");
            const uint64_t served = iop.emulatorCycles();
            t.IsTrue(served > start + 50000u, "serving takes IOP cycles");
            iop.runToCycle(served + 1000u);
            t.Equals(iop.emulatorCycles(), served + 1000u, "the clock continues from the served time");
            iop.runEeCycles(8u * 500u);
            t.Equals(iop.emulatorCycles(), served + 1500u, "EE cycles granted afterwards add on");
        });

        tc.Run("sceSdRemote block transfers stream 1 KiB blocks, loop and report the buffer half", [](TestCase &t)
        {
            Libsd sd;
            uint32_t result = 0xFFu;
            t.IsTrue(sd.libsd.remote(0x8000u, {0u}, result) && result == 0u, "rSdInit(0)");
            t.IsTrue(!sd.libsd.remote(0x8090u, {}, result), "an unsupported remote function is refused");

            std::array<int16_t, 1024> ring{}; // two blocks: 256 left then 256 right samples each
            for (size_t i = 0; i < 256u; ++i)
            {
                ring[i] = 1000;
                ring[256u + i] = -1000;
                ring[512u + i] = 2000;
                ring[768u + i] = -2000;
            }
            t.IsTrue(sd.memory.writeRam(0x20000u, ring.data(), sizeof(ring)), "stage the ring in IOP RAM");

            t.IsTrue(sd.libsd.remote(0x80E0u, {0u, 0x13u, 0x20000u, 0x800u, 0x20400u}, result) && result == 0u,
                     "a looping write-from transfer starts");
            t.Equals(sd.libsd.remote(0x8100u, {0u}, result) ? result : 0u, (1u << 24u) | 0x20400u,
                     "write-from starts at the block holding `start`, in the second half");
            int16_t block[512];
            t.IsTrue(sd.libsd.fetchInputBlock(0u, block) && block[0] == 2000 && block[256] == -2000,
                     "the first block read is the second ring block");
            t.Equals(sd.libsd.remote(0x8100u, {0u}, result) ? result : 0u, 0x20000u, "the ring wraps to its start");
            t.IsTrue(sd.libsd.fetchInputBlock(0u, block) && block[0] == 1000 && block[511] == -1000,
                     "a looping transfer reads the first block again");
            t.Equals(static_cast<int>(sd.libsd.inputBlocks()), 2, "two blocks read");

            t.IsTrue(sd.libsd.remote(0x80E0u, {0u, 2u, 0u, 0u, 0u}, result), "stop the transfer");
            t.Equals(result, (1u << 24u) | 0x20400u, "stop returns the last status");
            t.IsTrue(!sd.libsd.fetchInputBlock(0u, block), "a stopped transfer gives no more blocks");

            t.IsTrue(sd.libsd.remote(0x80E0u, {1u, 0u, 0x20000u, 0x800u, 0u}, result) && result == 0u,
                     "a one-shot write transfer starts on core 1");
            t.IsTrue(sd.libsd.fetchInputBlock(1u, block) && sd.libsd.fetchInputBlock(1u, block) && block[0] == 2000,
                     "a one-shot transfer reads both blocks");
            t.IsTrue(!sd.libsd.fetchInputBlock(1u, block), "and then ends");
            t.IsTrue(sd.libsd.remote(0x80E0u, {0u, 0x10u, 0x20000u, 0x900u, 0u}, result) && result == 0xFFFFFFFFu,
                     "a size that is not a multiple of 1 KiB is refused");
        });

        tc.Run("the sound data input plays through BVOL", [](TestCase &t)
        {
            auto play = [](uint32_t bvol, std::vector<int16_t> &out)
            {
                Libsd sd;
                sd.spu2.setInputFetch([&sd](uint32_t core, int16_t block[512])
                                      { return sd.libsd.fetchInputBlock(core, block); });
                uint32_t result = 0u;
                (void)sd.libsd.remote(0x8000u, {0u}, result);
                for (uint32_t entry : {0x0980u, 0x0A80u, 0x0981u, 0x0A81u}) // MVOL L/R of both cores
                    (void)sd.libsd.remote(0x8010u, {entry, 0x3FFFu}, result);
                (void)sd.libsd.remote(0x8010u, {0x0F80u, bvol}, result); // BVOLL core 0
                (void)sd.libsd.remote(0x8010u, {0x1080u, bvol}, result); // BVOLR core 0
                std::array<int16_t, 512> ring{};
                for (size_t i = 0; i < 256u; ++i)
                {
                    ring[i] = 8000;
                    ring[256u + i] = -8000;
                }
                (void)sd.memory.writeRam(0x20000u, ring.data(), sizeof(ring));
                (void)sd.libsd.remote(0x80E0u, {0u, 0x10u, 0x20000u, 0x400u, 0u}, result);
                sd.spu2.run(1024u, out);
                return sd.spu2.inputRunning(0u);
            };
            std::vector<int16_t> out;
            t.IsTrue(play(0x7FFFu, out), "a looping input keeps running");
            t.IsTrue(*std::max_element(out.begin(), out.end()) > 2000, "the left channel plays the input");
            t.IsTrue(*std::min_element(out.begin(), out.end()) < -2000, "the right channel plays the input");
            std::vector<int16_t> muted;
            (void)play(0u, muted);
            t.IsTrue(std::all_of(muted.begin(), muted.end(), [](int16_t s) { return s == 0; }),
                     "BVOL 0 silences the input");
        });

        tc.Run("IRX modules start with argc and argv", [](TestCase &t)
        {
            iop_test::Host host;
            ps2x::iop::IopSubsystem iop(host);
            iop_test::Irx image;
            image.words(0u, {
                0x8CA80004u, // lw t0, 4(a1): argv[1]
                0x00000000u,
                0x91020000u, // lbu v0, 0(t0)
                0x00042200u, // sll a0, a0, 8
                0x03E00008u, // jr ra
                0x00441021u, // addu v0, v0, a0
            });
            image.install(host, 0x1000u);
            const char args[] = {'C', '\0', 'x', '\0'};
            const ps2x::iop::ModuleLoadResult loaded = iop.loadModuleBuffer(0x1000u, args, sizeof(args));
            t.Equals(loaded.startResult, 0x343, "argc 3 (path, \"C\", \"x\") and argv[1][0] == 'C'");
        });
    });
}
