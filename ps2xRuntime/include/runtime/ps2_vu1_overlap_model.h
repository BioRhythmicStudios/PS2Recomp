#pragma once

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

// Measurement-only model of moving back-end work to its own host thread (PS2X_VU1_OVERLAP_MODEL=1). Nothing changes
// for the guest: the work still runs inline. Each design is replayed on a virtual timeline in which its work is handed
// to a worker instead, and every guest observation of that work's results waits for the worker. The report says how
// much game-thread time would move off the thread and how much would come back as waits, before any threading exists.
//
// Designs:
// - kWhole: the whole DMA kick back end (processPendingTransfers, VIF1 resume) on the worker. Any read of VIF1/GIF
//   state, VU1 memory or GS privileged registers waits for it.
// - kSplit: VIF command parsing, VIF registers and VIF interrupts stay on the game thread (as on the PS2, where VIF1
//   runs ahead of a busy VU1); VU1 programs and GIF output run on the worker in stream order. Only reads of VU1
//   memory, GS privileged registers and GS read-back wait.
//
// Virtual EE time = host time - moved work + modelled waits. The worker starts a piece of work when it is issued
// (plus kHandoffNs to wake it if it was idle) or when it finishes earlier work, and is busy for its measured duration.
namespace ps2_vu1_overlap_model
{
    enum Design : uint32_t
    {
        kWhole = 0,
        kSplit,
        kDesignCount
    };

    enum SyncReason : uint32_t
    {
        kSyncVuMem = 0, // EE load/store to VU1 memory
        kSyncGsPriv,    // EE read of a GS privileged register (CSR, ...)
        kSyncVifReg,    // EE read of a VIF1 register or GIF_STAT
        kSyncVSync,     // sceGsSyncV / VBlank wait (frame boundary)
        kSyncGsStub,    // an HLE entry point that reads GS state back (image download, ...)
        kSyncVif0,      // a kick that includes VIF0 (VU0 is shared with EE macro mode)
        kSyncCount
    };

    // Which reasons wait for which design's worker.
    constexpr bool kWaits[kDesignCount][kSyncCount] = {
        {true, true, true, true, true, true},
        {true, true, false, true, true, false},
    };

    inline const char *reasonName(uint32_t r)
    {
        static const char *names[kSyncCount] = {"vumem", "gspriv", "vifreg", "vsync", "gsstub", "vif0"};
        return r < kSyncCount ? names[r] : "?";
    }

    constexpr uint64_t kHandoffNs = 5000; // waking an idle worker
    constexpr uint64_t kReportNs = 5000000000ull;

    struct Model
    {
        uint32_t depth = 0;
        uint64_t workStart = 0;
        uint64_t virtualStart = 0;
        uint64_t movedNs = 0;
        uint64_t stallNs = 0;
        uint64_t workerFree = 0; // virtual time
        // Since the last report.
        uint64_t pieces = 0, periodMoved = 0, periodStall = 0;
        uint64_t syncs[kSyncCount] = {}, syncsWaited[kSyncCount] = {}, stallBy[kSyncCount] = {};
    };

    struct State
    {
        bool enabled = false;
        uint64_t firstNs = 0;
        uint64_t lastReport = 0;
        Model models[kDesignCount];
        uint64_t copyBytes = 0, vif1Interrupts = 0;
        uint64_t vifRegReads[34] = {}; // VIF1 registers by (address - 0x10003C00) / 16; [32] GIF_STAT
    };

    inline State &state()
    {
        static State s = []
        {
            State init;
            const char *v = std::getenv("PS2X_VU1_OVERLAP_MODEL");
            init.enabled = v && *v && *v != '0';
            return init;
        }();
        return s;
    }

    inline uint64_t nowNs()
    {
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                         std::chrono::steady_clock::now().time_since_epoch())
                                         .count());
    }

    inline uint64_t virtualNow(const Model &m, uint64_t now) { return now - m.movedNs + m.stallNs; }

    inline void report(State &s, uint64_t now)
    {
        const double secs = (now - s.lastReport) / 1e9;
        static const char *designNames[kDesignCount] = {"whole", "split"};
        for (uint32_t d = 0; d < kDesignCount; ++d)
        {
            Model &m = s.models[d];
            std::fprintf(stderr, "[vu1-model] t=%.1f %s pieces=%llu moved=%.2f ms/s waited=%.2f ms/s net=%.2f ms/s",
                         (now - s.firstNs) / 1e9, designNames[d], static_cast<unsigned long long>(m.pieces),
                         m.periodMoved / 1e6 / secs, m.periodStall / 1e6 / secs,
                         (double(m.periodMoved) - double(m.periodStall)) / 1e6 / secs);
            for (uint32_t r = 0; r < kSyncCount; ++r)
            {
                if (m.syncs[r] != 0 && kWaits[d][r])
                    std::fprintf(stderr, " %s=%llu/%llu:%.2fms", reasonName(r),
                                 static_cast<unsigned long long>(m.syncs[r]),
                                 static_cast<unsigned long long>(m.syncsWaited[r]), m.stallBy[r] / 1e6);
            }
            std::fprintf(stderr, "\n");
            m.pieces = m.periodMoved = m.periodStall = 0;
            for (uint32_t r = 0; r < kSyncCount; ++r)
                m.syncs[r] = m.syncsWaited[r] = m.stallBy[r] = 0;
        }
        std::fprintf(stderr, "[vu1-model] t=%.1f copy=%.0f KB/s vif1irq=%llu vifreads", (now - s.firstNs) / 1e9,
                     s.copyBytes / 1024.0 / secs, static_cast<unsigned long long>(s.vif1Interrupts));
        for (uint32_t i = 0; i < 34; ++i)
        {
            if (s.vifRegReads[i] != 0)
                std::fprintf(stderr, " %s%X=%llu", i == 32 ? "GIF_STAT" : "0x10003C", i == 32 ? 0u : i * 16u,
                             static_cast<unsigned long long>(s.vifRegReads[i]));
            s.vifRegReads[i] = 0;
        }
        std::fprintf(stderr, "\n");
        s.lastReport = now;
        s.copyBytes = s.vif1Interrupts = 0;
    }

    // The guest observes back-end state: each design whose worker produces that state waits (virtually) for it.
    inline void sync(SyncReason reason)
    {
        State &s = state();
        if (!s.enabled)
            return;
        const uint64_t now = nowNs();
        for (uint32_t d = 0; d < kDesignCount; ++d)
        {
            Model &m = s.models[d];
            if (m.depth != 0 || !kWaits[d][reason])
                continue;
            const uint64_t v = virtualNow(m, now);
            ++m.syncs[reason];
            if (m.workerFree > v)
            {
                const uint64_t stall = m.workerFree - v;
                m.stallNs += stall;
                m.periodStall += stall;
                m.stallBy[reason] += stall;
                ++m.syncsWaited[reason];
            }
        }
    }

    inline void noteVifRegRead(uint32_t address)
    {
        State &s = state();
        if (!s.enabled)
            return;
        if (address == 0x10003020u)
            ++s.vifRegReads[32];
        else if (address >= 0x10003C00u && address < 0x10003E00u)
            ++s.vifRegReads[(address - 0x10003C00u) / 16u];
        sync(kSyncVifReg);
    }

    // Brackets work that design d's worker would run. Nested work counts once.
    inline void beginWork(Design d)
    {
        State &s = state();
        if (!s.enabled)
            return;
        Model &m = s.models[d];
        if (m.depth++ != 0)
            return;
        const uint64_t now = nowNs();
        if (s.firstNs == 0)
            s.firstNs = s.lastReport = now;
        m.workStart = now;
        m.virtualStart = virtualNow(m, now);
    }

    inline void endWork(Design d)
    {
        State &s = state();
        if (!s.enabled)
            return;
        Model &m = s.models[d];
        if (--m.depth != 0)
            return;
        const uint64_t now = nowNs();
        const uint64_t dur = now - m.workStart;
        // A busy worker takes queued work without a wake-up; an idle one pays kHandoffNs to be woken.
        const bool idle = m.workerFree <= m.virtualStart;
        const uint64_t start = idle ? m.virtualStart + kHandoffNs : m.workerFree;
        m.workerFree = start + dur;
        m.movedNs += dur;
        m.periodMoved += dur;
        ++m.pieces;
        if (now - s.lastReport >= kReportNs)
            report(s, now);
    }

    // Bytes a real worker would have to copy at the kick (DMA payloads read from guest memory at processing time).
    inline void noteCopy(uint64_t bytes)
    {
        State &s = state();
        if (s.enabled)
            s.copyBytes += bytes;
    }

    // A VIF1 i-bit interrupt raised by back-end work.
    inline void noteVif1Interrupt()
    {
        State &s = state();
        if (s.enabled)
            ++s.vif1Interrupts;
    }

    struct WorkScope
    {
        explicit WorkScope(Design d) : m_design(d) { beginWork(d); }
        ~WorkScope() { endWork(m_design); }
        WorkScope(const WorkScope &) = delete;
        WorkScope &operator=(const WorkScope &) = delete;

    private:
        Design m_design;
    };
}
