#pragma once

#include "runtime/ps2_host_profile.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>

// PS2X_PACING_TRACE=<file>: a timestamped record of the events that decide when a guest frame reaches the screen
// (guest VBlanks, frames published by the GS backend, frames taken and presented by the window), written as CSV
// when the run ends. Off unless the variable is set; recording is one atomic increment and a store.
namespace ps2_pacing_trace
{
    enum Kind : uint32_t
    {
        kVBlank = 'V',   // a = VSync tick, b = lateness behind its host deadline (ns)
        kSyncEntry = 'E', // game thread entered sceGsSyncV: a = guest frame
        kPublish = 'P',  // GS backend published a frame: a = guest frame, b = generation
        kTake = 'T',     // presenter took a new frame: b = generation
        kSwap = 'S',     // presenter's EndDrawing returned: a = 1 if this present showed a new frame
        kPacing = 'A',   // audio pacing correction applied to the next VBlank: a = ppm (signed)
    };

    struct Record
    {
        uint64_t ns;
        uint32_t kind;
        uint32_t a;
        uint64_t b;
    };

    constexpr uint32_t kCapacity = 1u << 22; // about 6 hours at 200 events per second

    struct State
    {
        State()
        {
            path = std::getenv("PS2X_PACING_TRACE");
            if (path && path[0])
                records.reset(new Record[kCapacity]);
            else
                path = nullptr;
        }
        const char *path = nullptr;
        std::unique_ptr<Record[]> records;
        std::atomic<uint32_t> next{0};
        std::atomic<bool> written{false};
    };

    inline State &state()
    {
        static State s;
        return s;
    }

    inline bool enabled() { return state().path != nullptr; }

    inline void note(Kind kind, uint32_t a = 0, uint64_t b = 0)
    {
        State &s = state();
        if (!s.path)
            return;
        const uint32_t i = s.next.fetch_add(1, std::memory_order_relaxed);
        if (i < kCapacity)
            s.records[i] = Record{ps2_host_profile::nowNs(), kind, a, b};
    }

    // Writes the trace once; later calls do nothing. Records still being written by another thread may be torn.
    inline void write()
    {
        State &s = state();
        if (!s.path || s.written.exchange(true))
            return;
        std::FILE *f = std::fopen(s.path, "w");
        if (!f)
            return;
        const uint32_t n = (std::min)(s.next.load(), kCapacity);
        std::fprintf(f, "ns,kind,a,b\n");
        for (uint32_t i = 0; i < n; ++i)
        {
            const Record &r = s.records[i];
            std::fprintf(f, "%llu,%c,%u,%llu\n", static_cast<unsigned long long>(r.ns), static_cast<char>(r.kind), r.a,
                         static_cast<unsigned long long>(r.b));
        }
        std::fclose(f);
    }
}
