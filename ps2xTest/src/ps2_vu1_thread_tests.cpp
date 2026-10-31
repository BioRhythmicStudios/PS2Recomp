#include "MiniTest.h"
#include "runtime/ps2_memory.h"
#include "runtime/ps2_vu1_thread.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <random>
#include <stdexcept>
#include <thread>
#include <vector>

// PS2X_VU1_THREAD (VU1_THREAD_PLAN.md, design B): the threaded back end must give exactly what the inline one gives.
namespace
{
    uint32_t vifCmd(uint8_t opcode, uint8_t num, uint16_t imm, bool irq = false)
    {
        return (irq ? 0x80000000u : 0u) | (static_cast<uint32_t>(opcode) << 24) | (static_cast<uint32_t>(num) << 16) |
               static_cast<uint32_t>(imm);
    }

    void put32(std::vector<uint8_t> &dst, uint32_t value)
    {
        const size_t at = dst.size();
        dst.resize(at + 4u);
        std::memcpy(dst.data() + at, &value, 4u);
    }

    void put64(std::vector<uint8_t> &dst, uint64_t value)
    {
        const size_t at = dst.size();
        dst.resize(at + 8u);
        std::memcpy(dst.data() + at, &value, 8u);
    }

    uint64_t gifTag(uint32_t nloop, uint32_t flg, uint32_t nreg, bool eop)
    {
        return static_cast<uint64_t>(nloop & 0x7FFFu) | (eop ? (1ull << 15) : 0ull) |
               (static_cast<uint64_t>(flg & 3u) << 58) | (static_cast<uint64_t>(nreg & 0xFu) << 60);
    }

    // DIRECT payloads must start on a quadword boundary after the command: pad with NOPs.
    void alignForDirect(std::vector<uint8_t> &dst)
    {
        while ((dst.size() + 4u) % 16u != 0u)
            put32(dst, vifCmd(0x00u, 0u, 0u));
    }

    // One PS2Memory with a recording GIF sink and a stand-in VU1 program (reads and writes VU1 data, kicks PATH1).
    struct Rig
    {
        struct GifRecord
        {
            uint32_t path;
            bool synthetic;
            std::vector<uint8_t> bytes;
            bool operator==(const GifRecord &o) const
            {
                return path == o.path && synthetic == o.synthetic && bytes == o.bytes;
            }
        };

        PS2Memory mem;
        GifArbiter arbiter;
        std::vector<GifRecord> gif;
        std::vector<std::array<uint32_t, 4>> runs;
        std::atomic<uint32_t> workerRuns{0}, gameRuns{0};
        std::atomic<uint32_t> slowRunMs{0};
        bool needsInline = false;
        uint32_t eeMscals = 0;

        explicit Rig(bool threaded)
        {
            mem.initialize();
            arbiter.setPathPacketFn([this](GifPathId path, const uint8_t *data, uint32_t size, bool synthetic)
                                    { gif.push_back({static_cast<uint32_t>(path), synthetic, std::vector<uint8_t>(data, data + size)}); });
            mem.setGifArbiter(&arbiter);
            mem.setVu1MscalCallback([this](uint32_t pc, uint32_t top, uint32_t itop)
                                    { program(pc, top, itop, 1u); });
            mem.setVu1MscntCallback([this](uint32_t top, uint32_t itop)
                                    { program(0u, top, itop, 2u); });
            mem.setVu1ThreadHooks([this]()
                                  { return needsInline; },
                                  [this]()
                                  { ++eeMscals; });
            if (threaded)
            {
                mem.startVu1Thread();
                mem.m_vu1ThreadCheck = true;
            }
        }

        ~Rig() { mem.stopVu1Thread(); }

        void program(uint32_t pc, uint32_t top, uint32_t itop, uint32_t kind)
        {
            (Vu1Thread::onWorker() ? workerRuns : gameRuns).fetch_add(1);
            if (const uint32_t ms = slowRunMs.load())
                std::this_thread::sleep_for(std::chrono::milliseconds(ms));
            uint8_t *data = mem.getVU1Data();
            const uint8_t *code = mem.getVU1Code();
            uint32_t in = 0, op = 0;
            std::memcpy(&in, data + (top * 16u) % PS2_VU1_DATA_SIZE, 4u);
            std::memcpy(&op, code + (pc % PS2_VU1_CODE_SIZE), 4u);
            const uint32_t out = in * 31u + op + itop * 7u + kind;
            std::memcpy(data + ((top + 1u) * 16u) % PS2_VU1_DATA_SIZE, &out, 4u);
            runs.push_back({pc, top, itop, out});
            std::vector<uint8_t> packet;
            put64(packet, gifTag(1u, 0u, 1u, true));
            put64(packet, 0x0Eull); // A+D
            put64(packet, out);
            put64(packet, 0x3Full);
            mem.submitGifPacket(GifPathId::Path1, packet.data(), static_cast<uint32_t>(packet.size()));
        }
    };

    // Random VIF1 streams covering every command the parser handles, with the stream-carried state (CYCLE, MODE,
    // pending PATH2 IMAGE) that both passes must agree on.
    struct StreamGen
    {
        std::mt19937 rng;
        uint32_t cl = 1u, wl = 1u;
        explicit StreamGen(uint32_t seed) : rng(seed) {}
        uint32_t pick(uint32_t n) { return static_cast<uint32_t>(rng() % n); }
        bool irq() { return pick(9u) == 0u; }

        void unpack(std::vector<uint8_t> &out)
        {
            static const uint8_t kFormats[] = {0x60u, 0x64u, 0x68u, 0x6Cu, 0x6Du, 0x6Eu, 0x6Fu, 0x61u, 0x62u, 0x65u};
            uint8_t op = kFormats[pick(sizeof(kFormats))];
            if (pick(3u) == 0u)
                op |= 0x10u; // masked
            const uint8_t vn = (op >> 2) & 3u, vl = op & 3u;
            const uint32_t num = 1u + pick(12u);
            const uint32_t bits = (vl == 3u && vn == 3u) ? 16u : (vn + 1u) * (vl == 0u ? 32u : vl == 1u ? 16u : vl == 2u ? 8u : 16u);
            const uint32_t bytesPerVector = (bits + 7u) / 8u;
            uint32_t source = num;
            if (cl < wl)
            {
                uint32_t rem = num % wl;
                if (rem > cl)
                    rem = cl;
                source = (num / wl) * cl + rem;
            }
            const uint32_t total = ((source * bytesPerVector) + 3u) & ~3u;
            uint16_t imm = static_cast<uint16_t>(pick(1024u));
            if (pick(2u) == 0u)
                imm |= 0x8000u; // relative to TOPS
            if (pick(2u) == 0u)
                imm |= 0x4000u; // zero-extend
            put32(out, vifCmd(op, static_cast<uint8_t>(num), imm, irq()));
            for (uint32_t i = 0; i < total; ++i)
                out.push_back(static_cast<uint8_t>(rng()));
        }

        void direct(std::vector<uint8_t> &out)
        {
            alignForDirect(out);
            if (pick(3u) == 0u)
            {
                // IMAGE transfer split over two DIRECTs: the second one continues the pending image.
                const uint32_t total = 2u + pick(3u), first = 1u;
                put32(out, vifCmd(0x50u, 0u, static_cast<uint16_t>(1u + first)));
                put64(out, gifTag(total, 2u, 0u, true));
                put64(out, 0ull);
                for (uint32_t i = 0; i < first * 16u; ++i)
                    out.push_back(static_cast<uint8_t>(rng()));
                alignForDirect(out);
                put32(out, vifCmd(pick(2u) ? 0x51u : 0x50u, 0u, static_cast<uint16_t>(total - first)));
                for (uint32_t i = 0; i < (total - first) * 16u; ++i)
                    out.push_back(static_cast<uint8_t>(rng()));
                return;
            }
            put32(out, vifCmd(pick(2u) ? 0x51u : 0x50u, 0u, 2u, irq()));
            put64(out, gifTag(1u, 0u, 1u, true));
            put64(out, 0x0Eull);
            put64(out, rng());
            put64(out, 0x3Full);
        }

        std::vector<uint8_t> stream(uint32_t commands)
        {
            std::vector<uint8_t> out;
            for (uint32_t i = 0; i < commands; ++i)
            {
                switch (pick(16u))
                {
                case 0:
                    put32(out, vifCmd(0x00u, 0u, 0u, irq()));
                    break;
                case 1:
                    cl = 1u + pick(4u);
                    wl = 1u + pick(4u);
                    put32(out, vifCmd(0x01u, 0u, static_cast<uint16_t>(cl | (wl << 8)), irq()));
                    break;
                case 2:
                    put32(out, vifCmd(0x02u, 0u, static_cast<uint16_t>(pick(512u)), irq()));
                    break;
                case 3:
                    put32(out, vifCmd(0x03u, 0u, static_cast<uint16_t>(pick(512u)), irq()));
                    break;
                case 4:
                    put32(out, vifCmd(0x04u, 0u, static_cast<uint16_t>(pick(1024u)), irq()));
                    break;
                case 5:
                    put32(out, vifCmd(0x05u, 0u, static_cast<uint16_t>(pick(3u)), irq()));
                    break;
                case 6:
                    put32(out, vifCmd(0x07u, 0u, static_cast<uint16_t>(pick(65536u)), irq()));
                    break;
                case 7:
                    put32(out, vifCmd(0x20u, 0u, 0u, irq()));
                    put32(out, static_cast<uint32_t>(rng()));
                    break;
                case 8:
                    put32(out, vifCmd(pick(2u) ? 0x30u : 0x31u, 0u, 0u, irq()));
                    for (int k = 0; k < 4; ++k)
                        put32(out, static_cast<uint32_t>(rng()));
                    break;
                case 9:
                {
                    while (out.size() % 8u != 4u)
                        put32(out, vifCmd(0x00u, 0u, 0u));
                    const uint32_t n = 1u + pick(4u);
                    put32(out, vifCmd(0x4Au, static_cast<uint8_t>(n), static_cast<uint16_t>(pick(2040u)), irq()));
                    for (uint32_t k = 0; k < n * 8u; ++k)
                        out.push_back(static_cast<uint8_t>(rng()));
                    break;
                }
                case 10:
                    put32(out, vifCmd(pick(2u) ? 0x14u : 0x15u, 0u, static_cast<uint16_t>(pick(2048u)), irq()));
                    break;
                case 11:
                    put32(out, vifCmd(0x17u, 0u, 0u, irq()));
                    break;
                case 12:
                    put32(out, vifCmd(0x06u, 0u, pick(2u) ? 0x8000u : 0u, irq()));
                    break;
                case 13:
                case 14:
                    unpack(out);
                    break;
                default:
                    direct(out);
                    break;
                }
            }
            return out;
        }
    };

    bool sameRegs(const VIFRegisters &a, const VIFRegisters &b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }

    // Drives the same operation sequence into both rigs, comparing what the game thread observes as it goes.
    void runEquivalence(TestCase &t, uint32_t seed, uint32_t steps, bool toggleInline)
    {
        Rig inl(false), thr(true);
        StreamGen gen(seed);
        std::mt19937 ops(seed * 7919u + 1u);
        uint32_t observeMismatches = 0;
        for (uint32_t step = 0; step < steps; ++step)
        {
            const uint32_t what = ops() % 10u;
            if (toggleInline && what == 0u)
            {
                inl.needsInline = thr.needsInline = (ops() % 2u) == 0u;
            }
            else if (what <= 4u)
            {
                const std::vector<uint8_t> s = gen.stream(1u + ops() % 6u);
                // Sometimes split one stream over two kicks at an arbitrary word boundary.
                const uint32_t words = static_cast<uint32_t>(s.size() / 4u);
                const uint32_t cut = (ops() % 3u == 0u && words > 1u) ? 4u * (1u + ops() % (words - 1u)) : static_cast<uint32_t>(s.size());
                for (Rig *r : {&inl, &thr})
                {
                    r->mem.processVIF1Data(s.data(), cut);
                    if (cut < s.size())
                        r->mem.processVIF1Data(s.data() + cut, static_cast<uint32_t>(s.size() - cut));
                }
            }
            else if (what == 5u)
            {
                // The i-bit handler: read STAT and MARK, clear MARK, resume with STC.
                for (Rig *r : {&inl, &thr})
                {
                    r->mem.writeIORegister(0x10003C30u, 0u);
                    r->mem.writeIORegister(0x10003C10u, 0x8u);
                }
            }
            else if (what == 6u)
            {
                static const uint32_t kRegs[] = {0x10003C40u, 0x10003C50u, 0x10003C70u, 0x10003C90u,
                                                 0x10003CA0u, 0x10003CB0u, 0x10003CC0u, 0x10003C30u};
                const uint32_t reg = kRegs[ops() % 8u];
                uint32_t value = static_cast<uint32_t>(ops());
                if (reg == 0x10003C40u)
                {
                    // CYCLE: keep the generator's UNPACK sizes in step with the registers.
                    gen.cl = 1u + ops() % 4u;
                    gen.wl = 1u + ops() % 4u;
                    value = gen.cl | (gen.wl << 8);
                }
                inl.mem.writeIORegister(reg, value);
                thr.mem.writeIORegister(reg, value);
            }
            else if (what == 7u)
            {
                // A PATH3 packet from the GIF DMA side (held while MSKPATH3 masks it).
                std::vector<uint8_t> packet;
                put64(packet, gifTag(1u, 0u, 1u, true));
                put64(packet, 0x0Eull);
                put64(packet, ops());
                put64(packet, 0x3Full);
                inl.mem.processGIFPacket(packet.data(), static_cast<uint32_t>(packet.size()));
                thr.mem.processGIFPacket(packet.data(), static_cast<uint32_t>(packet.size()));
            }
            else if (what == 8u)
            {
                // The EE reads VU1 memory: it must see the worker's results immediately.
                const uint32_t addr = PS2_VU1_DATA_BASE + 16u * (ops() % 1024u);
                if (inl.mem.read32(addr) != thr.mem.read32(addr))
                    ++observeMismatches;
            }
            else
            {
                if (inl.mem.readIORegister(0x10003C00u) != thr.mem.readIORegister(0x10003C00u) ||
                    inl.mem.readIORegister(0x10003C30u) != thr.mem.readIORegister(0x10003C30u) ||
                    inl.mem.readIORegister(0x10003020u) != thr.mem.readIORegister(0x10003020u))
                    ++observeMismatches;
                if (inl.mem.consumeCompletedIntcCauses() != thr.mem.consumeCompletedIntcCauses())
                    ++observeMismatches;
            }
        }
        thr.mem.vu1ThreadWait(Vu1Thread::kWaitFrame);

        t.Equals(observeMismatches, 0u, "the game thread should observe the same VIF1 registers, interrupts and VU1 memory");
        t.IsTrue(sameRegs(inl.mem.vif1_regs, thr.mem.vif1_regs), "game-thread VIF1 registers should match inline");
        t.IsTrue(sameRegs(thr.mem.vif1_regs, thr.mem.m_vif1WorkerRegs), "the worker's VIF1 register copy should match");
        t.Equals(thr.mem.m_vif1WorkerPendingPath2ImageQwc, inl.mem.m_vif1PendingPath2ImageQwc,
                 "pending PATH2 image state should match");
        t.IsTrue(std::memcmp(inl.mem.getVU1Data(), thr.mem.getVU1Data(), PS2_VU1_DATA_SIZE) == 0,
                 "VU1 data memory should be identical");
        t.IsTrue(std::memcmp(inl.mem.getVU1Code(), thr.mem.getVU1Code(), PS2_VU1_CODE_SIZE) == 0,
                 "VU1 code memory should be identical");
        t.Equals(thr.runs.size(), inl.runs.size(), "the same number of VU1 programs should run");
        t.IsTrue(thr.runs == inl.runs, "VU1 programs should run with the same PC, TOP, ITOP and inputs, in order");
        t.Equals(thr.gif.size(), inl.gif.size(), "the GS should receive the same number of packets");
        t.IsTrue(thr.gif == inl.gif, "the GS should receive the same packets on the same paths in the same order");
        t.Equals(thr.mem.vu1ThreadCheckMismatches(), 0ull, "the worker's VIF1 state should match at every stretch");
        t.IsTrue(thr.mem.vu1ThreadChecks() > 0u, "the register cross-check should have run");
        t.IsTrue(inl.runs.size() > 20u, "the stream should have run VU1 programs");
        if (!toggleInline)
            t.Equals(thr.gameRuns.load(), 0u, "without D/T stops every VU1 program should run on the worker");
    }
}

void register_ps2_vu1_thread_tests()
{
    MiniTest::Case("PS2Vu1Thread", [](TestCase &tc)
    {
        tc.Run("threaded back end matches inline on random VIF1 streams", [](TestCase &t)
        {
            for (uint32_t seed = 1; seed <= 6; ++seed)
                runEquivalence(t, seed, 1500u, false);
        });

        tc.Run("kicks with VU1 D/T stops enabled run inline and the worker stays in step", [](TestCase &t)
        {
            for (uint32_t seed = 11; seed <= 13; ++seed)
                runEquivalence(t, seed, 1500u, true);
        });

        tc.Run("an EE read of VU1 memory waits for the worker's program", [](TestCase &t)
        {
            Rig r(true);
            r.slowRunMs = 30u;
            std::vector<uint8_t> s;
            put32(s, vifCmd(0x14u, 0u, 0u)); // MSCAL: the slow program writes VU1 data at TOP + 1
            r.mem.processVIF1Data(s.data(), static_cast<uint32_t>(s.size()));
            const uint32_t seen = r.mem.read32(PS2_VU1_DATA_BASE + 16u);
            t.Equals(r.runs.size(), static_cast<size_t>(1u), "the program should have finished before the read returned");
            t.Equals(seen, r.runs.empty() ? 0xFFFFFFFFu : r.runs[0][3], "the EE should read the value the program wrote");
        });

        tc.Run("an EE read of GS CSR waits for the worker's GS", [](TestCase &t)
        {
            Rig r(true);
            std::atomic<bool> finishSet{false};
            r.arbiter.setPathPacketFn([&](GifPathId, const uint8_t *, uint32_t, bool)
                                      {
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                r.mem.gs().csr.fetch_or(0x2u); // FINISH, as the GS front end raises it
                finishSet = true; });
            std::vector<uint8_t> s;
            put32(s, vifCmd(0x14u, 0u, 0u)); // MSCAL: the program kicks a PATH1 packet
            r.mem.processVIF1Data(s.data(), static_cast<uint32_t>(s.size()));
            t.IsTrue(!finishSet.load(), "the kick should return before the worker's GS has the packet");
            const uint64_t csr = r.mem.read64(0x12001000u);
            t.IsTrue((csr & 0x2u) != 0u, "a CSR read should see FINISH raised by the worker's GS");
        });

        tc.Run("PATH3 packets released by MSKPATH3 follow the VIF1 stream", [](TestCase &t)
        {
            for (bool threaded : {false, true})
            {
                Rig r(threaded);
                std::vector<uint8_t> mask;
                put32(mask, vifCmd(0x06u, 0u, 0x8000u));
                r.mem.processVIF1Data(mask.data(), static_cast<uint32_t>(mask.size()));
                std::vector<uint8_t> p3;
                put64(p3, gifTag(1u, 0u, 1u, true));
                put64(p3, 0x0Eull);
                put64(p3, 0x3333ull);
                put64(p3, 0x3Full);
                r.mem.processGIFPacket(p3.data(), static_cast<uint32_t>(p3.size()));
                std::vector<uint8_t> s;
                put32(s, vifCmd(0x14u, 0u, 0u));       // PATH1 before the unmask
                put32(s, vifCmd(0x06u, 0u, 0u));       // unmask: the held PATH3 packet goes now
                put32(s, vifCmd(0x14u, 0u, 8u));       // PATH1 after it
                r.mem.processVIF1Data(s.data(), static_cast<uint32_t>(s.size()));
                r.mem.vu1ThreadWait(Vu1Thread::kWaitFrame);
                t.Equals(r.gif.size(), static_cast<size_t>(3u), "three packets should reach the GS");
                if (r.gif.size() == 3u)
                {
                    t.Equals(r.gif[0].path, 1u, "first the PATH1 packet kicked before the unmask");
                    t.Equals(r.gif[1].path, 3u, "then the released PATH3 packet");
                    t.Equals(r.gif[2].path, 1u, "then the PATH1 packet kicked after the unmask");
                }
            }
        });

        tc.Run("a worker error is reported on the game thread and shutdown stays clean", [](TestCase &t)
        {
            Rig r(true);
            r.mem.setVu1MscalCallback([](uint32_t, uint32_t, uint32_t)
                                      { throw std::runtime_error("vu1 program failed"); });
            std::vector<uint8_t> s;
            put32(s, vifCmd(0x14u, 0u, 0u));
            r.mem.processVIF1Data(s.data(), static_cast<uint32_t>(s.size()));
            bool threw = false;
            try
            {
                r.mem.vu1ThreadWait(Vu1Thread::kWaitFrame);
            }
            catch (const std::runtime_error &)
            {
                threw = true;
            }
            t.IsTrue(threw, "the next wait should rethrow the worker's exception");
            r.mem.stopVu1Thread();
            t.IsTrue(!r.mem.vu1ThreadActive(), "the worker should stop after an error");
        });

        tc.Run("stop drains queued work, then the inline path carries on from the same state", [](TestCase &t)
        {
            Rig inl(false), thr(true);
            thr.slowRunMs = 2u;
            StreamGen g1(77u), g2(77u);
            for (int i = 0; i < 40; ++i)
            {
                const std::vector<uint8_t> a = g1.stream(4u), b = g2.stream(4u);
                inl.mem.processVIF1Data(a.data(), static_cast<uint32_t>(a.size()));
                thr.mem.processVIF1Data(b.data(), static_cast<uint32_t>(b.size()));
                inl.mem.writeIORegister(0x10003C10u, 0x8u);
                thr.mem.writeIORegister(0x10003C10u, 0x8u);
            }
            thr.mem.stopVu1Thread();
            t.Equals(thr.runs.size(), inl.runs.size(), "every queued VU1 program should have run before the worker stopped");
            for (int i = 0; i < 20; ++i)
            {
                const std::vector<uint8_t> a = g1.stream(4u), b = g2.stream(4u);
                inl.mem.processVIF1Data(a.data(), static_cast<uint32_t>(a.size()));
                thr.mem.processVIF1Data(b.data(), static_cast<uint32_t>(b.size()));
                inl.mem.writeIORegister(0x10003C10u, 0x8u);
                thr.mem.writeIORegister(0x10003C10u, 0x8u);
            }
            t.IsTrue(std::memcmp(inl.mem.getVU1Data(), thr.mem.getVU1Data(), PS2_VU1_DATA_SIZE) == 0,
                     "VU1 memory should match after switching back to inline");
            t.IsTrue(thr.gif == inl.gif, "GS output should match after switching back to inline");
            t.IsTrue(sameRegs(inl.mem.vif1_regs, thr.mem.vif1_regs), "VIF1 registers should match");
        });

        tc.Run("Vu1Thread applies records in order and wait() returns only when idle", [](TestCase &t)
        {
            std::vector<uint32_t> seen;
            std::atomic<uint32_t> applied{0};
            Vu1Thread worker([&](const Vu1Thread::Record &rec, const uint8_t *payload)
                             {
                uint32_t v = 0;
                if (rec.size >= 4u)
                    std::memcpy(&v, payload, 4u);
                seen.push_back(rec.a + v);
                if (rec.a % 1000u == 0u)
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                applied.fetch_add(1); });
            uint32_t expected = 0;
            for (uint32_t i = 0; i < 20000u; ++i)
            {
                const uint32_t v = i * 3u;
                worker.append(Vu1Thread::kVif1Data, i, 0u, reinterpret_cast<const uint8_t *>(&v), 4u);
                ++expected;
                if (i % 64u == 0u)
                    worker.publish();
                if (i % 4096u == 0u)
                {
                    worker.wait(Vu1Thread::kWaitFrame);
                    t.Equals(applied.load(), expected, "wait() should return with every record applied");
                }
            }
            worker.wait(Vu1Thread::kWaitFrame);
            bool ordered = seen.size() == 20000u;
            for (uint32_t i = 0; ordered && i < seen.size(); ++i)
                ordered = seen[i] == i + i * 3u;
            t.IsTrue(ordered, "records should be applied exactly once, in append order");
            worker.stop();
            worker.stop(); // idempotent
        });
    });
}
