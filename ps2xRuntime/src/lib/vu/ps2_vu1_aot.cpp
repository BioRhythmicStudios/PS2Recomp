// Dispatch to ahead-of-time compiled VU1 microprograms, the handover back to the interpreter, the image dump for the
// generator, and the differential verify mode. See runtime/ps2_vu1_aot.h.
#include "runtime/ps2_vu1_aot.h"
#include "runtime/ps2_memory.h"

#include <atomic>
#include <array>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <ps2_log.h>

namespace
{
    struct Config
    {
        bool enabled = true;
        bool verify = false;
        bool stats = false; // PS2X_VU1_AOT_STATS: periodic histogram of handover PCs on stderr
        std::string dumpDir;
    };

    const Config &config()
    {
        static const Config value = []
        {
            Config c;
            if (const char *v = std::getenv("PS2X_VU1_AOT"))
                c.enabled = std::atoi(v) != 0;
            if (const char *v = std::getenv("PS2X_VU1_AOT_VERIFY"))
                c.verify = std::atoi(v) != 0;
            if (const char *v = std::getenv("PS2X_VU1_AOT_STATS"))
                c.stats = std::atoi(v) != 0;
            if (const char *v = std::getenv("PS2X_VU1_AOT_DUMP"))
                c.dumpDir = v;
            return c;
        }();
        return value;
    }

    std::vector<VU1AotImage> &registry()
    {
        static std::vector<VU1AotImage> images;
        return images;
    }

    std::atomic<uint64_t> g_compiledCycles{0}, g_interpretedCycles{0};
    std::atomic<uint64_t> g_calls{0}, g_completed{0}, g_handovers{0}, g_notCompiled{0}, g_verified{0}, g_mismatches{0};

    // PS2X_VU1_AOT_DUMP: each image once, and each (image, entry PC) once.
    void dumpEntry(uint64_t hash, const uint8_t *code, uint32_t codeSize, uint32_t pc)
    {
        static std::mutex mutex;
        static std::set<uint64_t> images;
        static std::set<std::pair<uint64_t, uint32_t>> entries;
        std::lock_guard lock(mutex);
        char name[64];
        std::snprintf(name, sizeof name, "vu1_%016llX", static_cast<unsigned long long>(hash));
        const std::filesystem::path base = std::filesystem::path(config().dumpDir) / name;
        if (images.insert(hash).second)
        {
            std::error_code ec;
            std::filesystem::create_directories(config().dumpDir, ec);
            if (std::FILE *f = std::fopen((base.string() + ".bin").c_str(), "wb"))
            {
                std::fwrite(code, 1, codeSize, f);
                std::fclose(f);
            }
        }
        if (entries.insert({hash, pc}).second)
        {
            if (std::FILE *f = std::fopen((base.string() + ".pcs").c_str(), "a"))
            {
                std::fprintf(f, "0x%04X\n", pc);
                std::fclose(f);
            }
        }
    }

    uint64_t stateDigest(const VU1State &s)
    {
        uint64_t h = 0xCBF29CE484222325ull;
        auto mix = [&h](const void *data, size_t size)
        {
            const uint8_t *p = static_cast<const uint8_t *>(data);
            for (size_t i = 0; i < size; ++i)
                h = (h ^ p[i]) * 0x100000001B3ull;
        };
        mix(s.vf, sizeof s.vf);
        mix(s.vi, sizeof s.vi);
        mix(s.acc, sizeof s.acc);
        const uint32_t words[] = {s.r, s.pc, s.mac, s.clip, s.status, s.top, s.itop, s.branchTarget, s.branchDelay};
        mix(words, sizeof words);
        mix(&s.q, 4);
        mix(&s.p, 4);
        mix(&s.i, 4);
        mix(&s.cycles, 8);
        const uint8_t flags[] = {s.ebit, s.haltAfterDelaySlot, s.stoppedByD, s.stoppedByT, s.branchPending,
                                 s.dBitEnabled, s.tBitEnabled};
        mix(flags, sizeof flags);
        return h;
    }
}

namespace vu1aot
{
    // Copies of the interpreter's EFU approximations (ps2_vu1_lower.cpp).
    float efuAtan(float value)
    {
        constexpr float coefficients[] = {0.999999344348907f, -0.333298563957214f, 0.199465364217758f,
                                          -0.13085337519646f, 0.096420042216778f, -0.055909886956215f,
                                          0.021861229091883f, -0.004054057877511f};
        constexpr float quarterPi = 0.785398185253143f;
        const float squared = value * value;
        float polynomial = coefficients[7];
        for (int index = 6; index >= 0; --index)
            polynomial = coefficients[index] + squared * polynomial;
        return quarterPi + value * polynomial;
    }

    float efuSin(float value)
    {
        constexpr float coefficients[] = {1.0f, -0.166666567325592f, 0.008333025500178f, -0.000198074136279f,
                                          0.000002601886990f};
        const float squared = value * value;
        float polynomial = coefficients[4];
        for (int index = 3; index >= 0; --index)
            polynomial = coefficients[index] + squared * polynomial;
        return value * polynomial;
    }

    float efuExp(float value)
    {
        constexpr float coefficients[] = {0.249998688697815f, 0.031257584691048f, 0.002591371303424f,
                                          0.000171562001924f, 0.000005430199963f, 0.000000690600018f};
        float polynomial = coefficients[5];
        for (int index = 4; index >= 0; --index)
            polynomial = coefficients[index] + value * polynomial;
        polynomial = 1.0f + value * polynomial;
        polynomial *= polynomial;
        polynomial *= polynomial;
        return polynomial != 0.0f ? 1.0f / polynomial : std::numeric_limits<float>::max();
    }
}

uint64_t VU1AotRuntime::hashCode(const uint8_t *code, uint32_t size)
{
    uint64_t h = 0xCBF29CE484222325ull;
    for (uint32_t i = 0; i < size; i += 8)
    {
        uint64_t word;
        std::memcpy(&word, code + i, 8);
        h = (h ^ word) * 0x100000001B3ull;
    }
    return h;
}

VU1AotResult VU1AotRuntime::handover(VU1Interpreter &vu, const VU1AotHandover &h, uint64_t cycle,
                                     const VU1AotPending &pending, const VU1AotLive &live)
{
    vu.m_cycle = cycle;
    vu.m_state.cycles = cycle;
    vu.m_flagPipeline = {};
    vu.m_fdiv = {};
    vu.m_efu = {};
    vu.m_storePipeline = {};
    vu.m_vfWritePipeline = {};
    vu.m_viWritePipeline = {};
    vu.m_accWritePipeline = {};
    vu.m_flagValid = vu.m_storeValid = vu.m_vfWriteValid = vu.m_viWriteValid = vu.m_accWriteValid = 0u;
    vu.m_vfReady = {};
    vu.m_viReady = {};
    vu.m_accReady = {};
    vu.m_vfLatestWrite = {};
    vu.m_viLatestWrite = {};
    vu.m_accLatestWrite = {};

    for (uint32_t index = 0; index < h.entryCount; ++index)
    {
        const VU1AotEntry &e = h.entries[index];
        const uint64_t ready = cycle + e.ready;
        switch (e.kind)
        {
        case VU1AotEntry::Flag:
        {
            auto &f = vu.m_flagPipeline[e.slot];
            f.valid = true;
            f.readyCycle = ready;
            f.issueCycle = ready - VU1Interpreter::kFmacLatency;
            f.mac = pending.flag[e.slot].mac;
            f.status = pending.flag[e.slot].status;
            f.extraSticky = pending.flag[e.slot].extraSticky;
            f.clip = pending.flag[e.slot].clip;
            f.writesMac = (e.bits & 1u) != 0u;
            f.writesStatus = (e.bits & 2u) != 0u;
            f.writesSticky = (e.bits & 4u) != 0u;
            f.writesClip = (e.bits & 8u) != 0u;
            vu.m_flagValid |= 1u << e.slot;
            break;
        }
        case VU1AotEntry::Fdiv:
            vu.m_fdiv.valid = true;
            vu.m_fdiv.readyCycle = ready;
            vu.m_fdiv.value = pending.q;
            vu.m_fdiv.statusDi = pending.qStatusDi;
            break;
        case VU1AotEntry::Efu:
            vu.m_efu[e.slot].valid = true;
            vu.m_efu[e.slot].readyCycle = ready;
            vu.m_efu[e.slot].value = pending.p[e.slot];
            break;
        case VU1AotEntry::Store:
        {
            auto &st = vu.m_storePipeline[e.slot];
            st.valid = true;
            st.readyCycle = ready;
            st.address = pending.store[e.slot].address;
            std::memcpy(st.words.data(), pending.store[e.slot].words, sizeof(st.words));
            st.laneMask = e.lanes;
            vu.m_storeValid |= 1u << e.slot;
            break;
        }
        case VU1AotEntry::VfWrite:
        {
            auto &w = vu.m_vfWritePipeline[e.slot];
            w.valid = true;
            w.readyCycle = ready;
            w.sequence = e.seq;
            w.reg = e.reg;
            w.laneMask = e.lanes;
            std::memcpy(w.value.data(), pending.vf[e.slot], sizeof(w.value));
            for (uint32_t c = 0; c < 4u; ++c)
                if ((e.latest & (1u << (3u - c))) != 0u)
                    vu.m_vfLatestWrite[e.reg][c] = e.seq;
            vu.m_vfWriteValid |= 1u << e.slot;
            break;
        }
        case VU1AotEntry::ViWrite:
        {
            auto &w = vu.m_viWritePipeline[e.slot];
            w.valid = true;
            w.readyCycle = ready;
            w.sequence = e.seq;
            w.reg = e.reg;
            w.value = pending.vi[e.slot];
            if (e.latest)
                vu.m_viLatestWrite[e.reg] = e.seq;
            vu.m_viWriteValid |= 1u << e.slot;
            break;
        }
        case VU1AotEntry::AccWrite:
        {
            auto &w = vu.m_accWritePipeline[e.slot];
            w.valid = true;
            w.readyCycle = ready;
            w.sequence = e.seq;
            w.laneMask = e.lanes;
            std::memcpy(w.value.data(), pending.acc[e.slot], sizeof(w.value));
            for (uint32_t c = 0; c < 4u; ++c)
                if ((e.latest & (1u << (3u - c))) != 0u)
                    vu.m_accLatestWrite[c] = e.seq;
            vu.m_accWriteValid |= 1u << e.slot;
            break;
        }
        case VU1AotEntry::VfReady:
            for (uint32_t c = 0; c < 4u; ++c)
                if ((e.lanes & (1u << (3u - c))) != 0u)
                    vu.m_vfReady[e.reg][c] = ready;
            break;
        case VU1AotEntry::ViReady:
            vu.m_viReady[e.reg] = ready;
            break;
        case VU1AotEntry::AccReady:
            for (uint32_t c = 0; c < 4u; ++c)
                if ((e.lanes & (1u << (3u - c))) != 0u)
                    vu.m_accReady[c] = ready;
            break;
        }
    }
    vu.m_nextWriteSequence = h.nextSequence;
    vu.m_nextCommitCycle = 0; // the next commit rescans every pipeline
    vu.m_efuResourceReady = h.efuResourceReady != 0u ? cycle + h.efuResourceReady : 0u;
    vu.m_workingClip = live.workingClip;
    vu.m_viBranchBackupValid = h.viBackupReg != 0u;
    vu.m_viBranchBackupReg = h.viBackupReg;
    vu.m_viBranchBackupValue = live.viBackupValue;
    vu.m_pendingHaltD = false;
    vu.m_pendingHaltT = false;
    VU1State &s = vu.m_state;
    s.pc = h.pc == VU1AotHandover::kDynamicPc ? live.pc : h.pc;
    s.branchPending = h.branchPending != 0u;
    if (h.branchPending != 0u) // otherwise the compiled code already left the last issued target in place
        s.branchTarget = h.branchTargetDynamic ? live.branchTarget : h.branchTarget;
    s.branchDelay = h.branchDelay;
    s.ebit = h.ebit != 0u;
    s.haltAfterDelaySlot = h.haltAfterDelaySlot != 0u;
    const uint64_t handovers = g_handovers.fetch_add(1, std::memory_order_relaxed) + 1u;
    if (config().stats)
    {
        // Where compiled code gives up most often (the handover PC; the interpreter continues there).
        static std::array<uint64_t, 0x4000u / 8u> byPc{};
        ++byPc[(s.pc & 0x3FF8u) / 8u];
        if ((handovers & 0xFFFFu) == 0u)
        {
            std::vector<std::pair<uint64_t, uint32_t>> top;
            for (uint32_t i = 0; i < byPc.size(); ++i)
                if (byPc[i] != 0u)
                    top.push_back({byPc[i], i * 8u});
            std::sort(top.rbegin(), top.rend());
            std::fprintf(stderr, "[vu1-aot] %llu handovers; top PCs:", static_cast<unsigned long long>(handovers));
            for (size_t i = 0; i < top.size() && i < 8u; ++i)
                std::fprintf(stderr, " 0x%04X=%llu", top[i].second, static_cast<unsigned long long>(top[i].first));
            std::fprintf(stderr, "\n");
        }
    }
    return VU1AotResult::Handover;
}

void VU1Interpreter::registerAotImages(const VU1AotImage *images, size_t count)
{
    registry().insert(registry().end(), images, images + count);
}

VU1Interpreter::AotStats VU1Interpreter::aotStats()
{
    AotStats s;
    s.calls = g_calls.load(std::memory_order_relaxed);
    s.completed = g_completed.load(std::memory_order_relaxed);
    s.handovers = g_handovers.load(std::memory_order_relaxed);
    s.notCompiled = g_notCompiled.load(std::memory_order_relaxed);
    s.verified = g_verified.load(std::memory_order_relaxed);
    s.mismatches = g_mismatches.load(std::memory_order_relaxed);
    s.compiledCycles = g_compiledCycles.load(std::memory_order_relaxed);
    s.interpretedCycles = g_interpretedCycles.load(std::memory_order_relaxed);
    return s;
}

void VU1Interpreter::countAotCycles(uint64_t compiled, uint64_t interpreted)
{
    g_compiledCycles.fetch_add(compiled, std::memory_order_relaxed);
    g_interpretedCycles.fetch_add(interpreted, std::memory_order_relaxed);
}

VU1AotResult VU1Interpreter::tryRunAot(const uint8_t *vuCode, uint32_t codeSize, uint32_t dataSize, PS2Memory *memory,
                                       uint64_t budgetEnd)
{
    const Config &cfg = config();
    if (m_unit != Unit::VU1 || memory == nullptr || vuCode != memory->getVU1Code() || codeSize != PS2_VU1_CODE_SIZE ||
        dataSize != PS2_VU1_DATA_SIZE || (registry().empty() && cfg.dumpDir.empty()))
        return VU1AotResult::NotCompiled;

    const uint64_t generation = memory->getVU1CodeGeneration();
    if (!m_aotGenerationValid || generation != m_aotGeneration)
    {
        m_aotGeneration = generation;
        m_aotGenerationValid = true;
        m_aotHash = VU1AotRuntime::hashCode(vuCode, codeSize);
        m_aotImage = nullptr;
        for (const VU1AotImage &image : registry())
            if (image.codeHash == m_aotHash)
                m_aotImage = &image;
    }
    if (!cfg.dumpDir.empty())
        dumpEntry(m_aotHash, vuCode, codeSize, m_state.pc);
    if (!cfg.enabled || m_aotImage == nullptr)
        return VU1AotResult::NotCompiled;

    // Compiled code starts from drained pipelines (every MSCAL, and MSCNT after a normal program end).
    g_calls.fetch_add(1, std::memory_order_relaxed);
    if (m_flagValid | m_storeValid | m_vfWriteValid | m_viWriteValid | m_accWriteValid || m_fdiv.valid ||
        m_efu[0].valid || m_efu[1].valid || m_xgkick.active || m_state.branchPending || m_state.ebit ||
        m_state.haltAfterDelaySlot || m_viBranchBackupValid || m_stopRequested)
    {
        g_notCompiled.fetch_add(1, std::memory_order_relaxed);
        return VU1AotResult::NotCompiled;
    }
    const VU1AotResult result = m_aotImage->entry(*this, budgetEnd);
    if (result == VU1AotResult::Completed)
        g_completed.fetch_add(1, std::memory_order_relaxed);
    else if (result == VU1AotResult::NotCompiled)
        g_notCompiled.fetch_add(1, std::memory_order_relaxed);
    return result;
}

// PS2X_VU1_AOT_VERIFY: run the compiled code on a copy, then the interpreter for real, and compare everything
// visible: registers, flags, cycle count, data memory and PATH1 packets. Returns false when not verifying this call.
bool VU1Interpreter::verifyAot(uint8_t *vuCode, uint32_t codeSize, uint8_t *vuData, uint32_t dataSize, GS &gs,
                               PS2Memory *memory, uint32_t maxCycles)
{
    if (!config().verify || !config().enabled || registry().empty() || m_unit != Unit::VU1)
        return false;

    std::unique_ptr<VU1Interpreter> compiled = std::make_unique<VU1Interpreter>(*this);
    std::vector<uint8_t> compiledData(vuData, vuData + dataSize);
    std::vector<uint8_t> compiledPackets, referencePackets;
    const uint32_t startPc = m_state.pc;
    compiled->m_aotVerifyCopy = true;
    compiled->m_packetCapture = &compiledPackets;
    compiled->m_packetPassthrough = false;
    const uint64_t completedBefore = g_completed.load(), handoversBefore = g_handovers.load();
    compiled->run(vuCode, codeSize, compiledData.data(), dataSize, gs, memory, maxCycles);
    const bool ranCompiled = g_completed.load() != completedBefore || g_handovers.load() != handoversBefore;

    m_aotSuppressed = true;
    m_packetCapture = &referencePackets;
    m_packetPassthrough = true;
    run(vuCode, codeSize, vuData, dataSize, gs, memory, maxCycles);
    m_aotSuppressed = false;
    m_packetCapture = nullptr;
    m_packetPassthrough = false;
    // The reference must not reuse a stale image lookup: it ran with the same generation.
    m_aotGeneration = compiled->m_aotGeneration;
    m_aotGenerationValid = compiled->m_aotGenerationValid;
    m_aotImage = compiled->m_aotImage;
    m_aotHash = compiled->m_aotHash;
    if (!ranCompiled)
        return true;

    g_verified.fetch_add(1, std::memory_order_relaxed);
    const char *what = nullptr;
    if (stateDigest(compiled->m_state) != stateDigest(m_state))
        what = "registers/flags";
    else if (compiled->m_cycle != m_cycle)
        what = "cycle count";
    else if (std::memcmp(compiledData.data(), vuData, dataSize) != 0)
        what = "data memory";
    else if (compiledPackets != referencePackets)
        what = "PATH1 packets";
    else if (compiled->m_workingClip != m_workingClip || compiled->m_stopRequested != m_stopRequested ||
             compiled->m_xgkick.active != m_xgkick.active ||
             (compiled->m_flagValid | compiled->m_storeValid | compiled->m_vfWriteValid | compiled->m_viWriteValid |
              compiled->m_accWriteValid) != (m_flagValid | m_storeValid | m_vfWriteValid | m_viWriteValid | m_accWriteValid))
        what = "pipeline state";
    if (what && std::getenv("PS2X_VU1_AOT_VERIFY_DETAIL") && g_mismatches.load() < 3)
    {
        // Field-level differences (compiled vs interpreter).
        const VU1State &a = compiled->m_state, &b = m_state;
        for (int r = 0; r < 32; ++r)
            for (int c = 0; c < 4; ++c)
                if (std::memcmp(&a.vf[r][c], &b.vf[r][c], 4) != 0)
                    std::fprintf(stderr, "  vf%d[%d]: %08X vs %08X\n", r, c, *reinterpret_cast<const uint32_t *>(&a.vf[r][c]),
                                 *reinterpret_cast<const uint32_t *>(&b.vf[r][c]));
        for (int r = 0; r < 16; ++r)
            if (a.vi[r] != b.vi[r])
                std::fprintf(stderr, "  vi%d: %d vs %d\n", r, a.vi[r], b.vi[r]);
        for (int c = 0; c < 4; ++c)
            if (std::memcmp(&a.acc[c], &b.acc[c], 4) != 0)
                std::fprintf(stderr, "  acc[%d] differs\n", c);
        auto f = [](const char *n, uint64_t x, uint64_t y) { if (x != y) std::fprintf(stderr, "  %s: %llX vs %llX\n", n, (unsigned long long)x, (unsigned long long)y); };
        f("q", *reinterpret_cast<const uint32_t *>(&a.q), *reinterpret_cast<const uint32_t *>(&b.q));
        f("p", *reinterpret_cast<const uint32_t *>(&a.p), *reinterpret_cast<const uint32_t *>(&b.p));
        f("i", *reinterpret_cast<const uint32_t *>(&a.i), *reinterpret_cast<const uint32_t *>(&b.i));
        f("r", a.r, b.r); f("pc", a.pc, b.pc); f("mac", a.mac, b.mac); f("clip", a.clip, b.clip); f("status", a.status, b.status);
        f("cycles", a.cycles, b.cycles); f("m_cycle", compiled->m_cycle, m_cycle);
        f("ebit", a.ebit, b.ebit); f("branchPending", a.branchPending, b.branchPending);
        f("top", a.top, b.top); f("itop", a.itop, b.itop); f("branchTarget", a.branchTarget, b.branchTarget); f("branchDelay", a.branchDelay, b.branchDelay);
        f("halt", a.haltAfterDelaySlot, b.haltAfterDelaySlot); f("stopD", a.stoppedByD, b.stoppedByD); f("stopT", a.stoppedByT, b.stoppedByT); f("dbit", a.dBitEnabled, b.dBitEnabled); f("tbit", a.tBitEnabled, b.tBitEnabled);
        for (uint32_t i = 0; i < dataSize; i += 16)
            if (std::memcmp(compiledData.data() + i, vuData + i, 16) != 0)
            {
                std::fprintf(stderr, "  data qword 0x%X differs\n", i / 16);
                break;
            }
        std::fprintf(stderr, "  packets: %zu vs %zu bytes\n", compiledPackets.size(), referencePackets.size());
    }
    if (what)
    {
        const uint64_t n = g_mismatches.fetch_add(1, std::memory_order_relaxed);
        if (n < 20)
            RUNTIME_ERROR("[vu1-aot verify] image " << std::hex << m_aotHash << " entry pc 0x" << startPc << std::dec
                                                    << ": " << what << " differ (compiled cycles " << compiled->m_cycle
                                                    << ", interpreter " << m_cycle << ")\n");
    }
    return true;
}
