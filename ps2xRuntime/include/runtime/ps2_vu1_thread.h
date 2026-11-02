#pragma once

#include "runtime/ps2_host_profile.h"

#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

// Back-end worker for PS2X_VU1_THREAD=1 (VU1_THREAD_PLAN.md, design B). The game thread keeps VIF1 command parsing,
// the VIF1 registers, i-bit stalls, interrupts and DMA completion. VU1 data and code memory, VU1 programs, the GIF
// arbiter and the GS run on this worker, fed in stream order by records that the game thread appends to a batch.
//
// Ordering: one producer (the game thread) and one consumer (the worker); batches and the records inside them are
// applied strictly in the order they were appended. Memory visibility: a batch is handed over under the mutex, and
// wait() returns only after the worker has finished every batch and released the mutex, so all of the worker's
// writes (VU1 memory, GS state, GS privileged registers) happen-before whatever the game thread does next.
// The game thread may touch worker-owned state only after wait(). The worker never touches game-thread state.
class Vu1Thread
{
public:
    enum Kind : uint32_t
    {
        kVif1Data = 1, // a stretch of VIF1 stream the game thread has parsed (payload = the bytes)
        kVif1Reg,      // an EE write to a VIF1 register: a = address, b = value
        kGif,          // a GIF packet: a = path | directHl << 8 | syntheticImageTag << 9 (payload = the bytes)
        kGifDrain,     // GifArbiter::drain()
        kVif1Check,    // PS2X_VU1_THREAD_CHECK: the game thread's VIF1 parse state after the preceding stretch
        kGsPriv,       // pipelined frames: an EE write to a GS privileged register, a = address, b = byte width
                       // (4 or 8), payload = the 64-bit value
        kCsrClear,     // pipelined frames: the write-one-to-clear CSR bits (SIGNAL, FINISH) an EE write set, in b
        kFrame,        // pipelined frames: a frame boundary (appendFrame)
    };

    struct Record
    {
        uint32_t kind, size, a, b; // followed by size payload bytes, padded to 16
    };
    static_assert(sizeof(Record) == 16u, "records stay 16-byte aligned");

    // Why the game thread waited (stats only).
    enum WaitReason : uint32_t
    {
        kWaitVuMem = 0, // EE access to VU1 memory
        kWaitGsPriv,    // EE access to a GS privileged register
        kWaitGsStub,    // an HLE GS entry point or native GIF shortcut that uses the GS directly
        kWaitFrame,     // frame boundary (sceGsSyncV, display flip)
        kWaitInline,    // a kick that must run inline (VU1 D/T stop bits enabled)
        kWaitUnpack,    // a difference-mode UNPACK whose ROW update reads VU1 memory
        kWaitStop,      // shutdown
        kWaitCount
    };

    using ProcessFn = std::function<void(const Record &, const uint8_t *payload)>;

    explicit Vu1Thread(ProcessFn fn) : m_process(std::move(fn))
    {
        m_thread = std::thread([this]
                               { workerLoop(); });
    }

    ~Vu1Thread() { stop(); }

    Vu1Thread(const Vu1Thread &) = delete;
    Vu1Thread &operator=(const Vu1Thread &) = delete;

    static bool onWorker() { return t_onWorker; }

    void append(Kind kind, uint32_t a, uint32_t b, const uint8_t *payload = nullptr, uint32_t size = 0)
    {
        const size_t at = m_batch.size();
        const size_t padded = (static_cast<size_t>(size) + 15u) & ~static_cast<size_t>(15u);
        m_batch.resize(at + sizeof(Record) + padded);
        const Record rec{kind, size, a, b};
        std::memcpy(m_batch.data() + at, &rec, sizeof(rec));
        if (size != 0)
            std::memcpy(m_batch.data() + at + sizeof(Record), payload, size);
        ++m_batchRecords;
        // Let the worker start on a long kick before the game thread has parsed all of it.
        if (m_batch.size() >= kEarlyPublishBytes)
            publish();
    }

    // Hands the current batch to the worker (one wake-up per batch, and only if the worker sleeps).
    void publish()
    {
        if (m_batch.empty())
            return;
        bool wake = false;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_bytes += m_batch.size();
            m_records += m_batchRecords;
            ++m_batches;
            m_pending.push_back(std::move(m_batch));
            wake = m_workerSleeping;
            if (wake)
                ++m_wakes;
            if (!m_free.empty())
            {
                m_batch = std::move(m_free.back());
                m_free.pop_back();
            }
            else
            {
                m_batch = std::vector<uint8_t>();
            }
        }
        m_batch.clear();
        m_batchRecords = 0;
        if (wake)
            m_cvWork.notify_one();
    }

    // Publishes the current batch and blocks until the worker has applied everything. Rethrows a worker exception.
    void wait(WaitReason reason)
    {
        publish();
        std::unique_lock<std::mutex> lock(m_mutex);
        ++m_waits[reason];
        if (!m_pending.empty() || m_busy)
        {
            ps2_host_profile::Scope profileScope(ps2_host_profile::kVu1Wait);
            const uint64_t start = ps2_host_profile::nowNs();
            m_eeWaiting = true;
            m_cvIdle.wait(lock, [this]
                          { return (m_pending.empty() && !m_busy) || m_error; });
            m_eeWaiting = false;
            ++m_waitsBlocked[reason];
            m_waitNs[reason] += ps2_host_profile::nowNs() - start;
        }
        if (m_error)
        {
            std::exception_ptr error = m_error;
            m_error = nullptr;
            std::rethrow_exception(error);
        }
    }

    // Pipelined frames (PS2X_VU1_PIPELINE=1). The game thread appends a frame boundary with the work that presents
    // the frame (run on the worker once everything before it is applied) and then waits only for the previous
    // boundary, so the worker draws frame N while the game builds frame N+1. Returns the frame's sequence number.
    uint64_t appendFrame(std::function<void()> present)
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_frameFns.push_back(std::move(present));
        }
        const uint64_t seq = ++m_framesAppended;
        append(kFrame, static_cast<uint32_t>(seq), 0u);
        publish();
        return seq;
    }

    // Blocks until the worker has passed frame boundary `seq` (0: returns at once) and returns the CSR SIGNAL and
    // FINISH bits it saw there. Rethrows a worker exception.
    uint32_t waitFrame(uint64_t seq)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        ++m_waits[kWaitFrame];
        if (m_framesDone < seq && !m_error)
        {
            ps2_host_profile::Scope profileScope(ps2_host_profile::kVu1Wait);
            const uint64_t start = ps2_host_profile::nowNs();
            m_eeWaiting = true;
            m_cvIdle.wait(lock, [&]
                          { return m_framesDone >= seq || m_error; });
            m_eeWaiting = false;
            ++m_waitsBlocked[kWaitFrame];
            m_waitNs[kWaitFrame] += ps2_host_profile::nowNs() - start;
        }
        if (m_error)
        {
            std::exception_ptr error = m_error;
            m_error = nullptr;
            std::rethrow_exception(error);
        }
        return seq == 0 ? 0u : m_frameCsr[seq % kFrameCsrSlots];
    }

    // Worker, for a kFrame record: presents the frame, then records the boundary as passed with `csrBits`.
    void runFrame(uint64_t seq, const std::function<uint32_t()> &csrBits)
    {
        std::function<void()> present;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (!m_frameFns.empty())
            {
                present = std::move(m_frameFns.front());
                m_frameFns.pop_front();
            }
        }
        if (present)
            present();
        const uint32_t bits = csrBits();
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_frameCsr[seq % kFrameCsrSlots] = bits;
            m_framesDone = seq;
        }
        m_cvIdle.notify_one();
    }

    void noteFallback(WaitReason reason)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        ++m_fallbacks[reason];
    }

    // Drains everything, then joins the worker. Idempotent.
    void stop()
    {
        if (!m_thread.joinable())
            return;
        try
        {
            wait(kWaitStop);
        }
        catch (const std::exception &e)
        {
            std::fprintf(stderr, "[vu1-thread] worker error at shutdown: %s\n", e.what());
        }
        catch (...)
        {
            std::fprintf(stderr, "[vu1-thread] worker error at shutdown\n");
        }
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_stop = true;
        }
        m_cvWork.notify_one();
        m_thread.join();
    }

    struct Stats
    {
        uint64_t batches = 0, records = 0, bytes = 0, wakes = 0, workerBusyNs = 0;
        uint64_t waits[kWaitCount] = {}, waitsBlocked[kWaitCount] = {}, waitNs[kWaitCount] = {};
        uint64_t fallbacks[kWaitCount] = {};
    };

    // Counters since the previous call.
    Stats takeStats()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        Stats s;
        s.batches = m_batches;
        s.records = m_records;
        s.bytes = m_bytes;
        s.wakes = m_wakes;
        s.workerBusyNs = m_workerBusyNs;
        m_batches = m_records = m_bytes = m_wakes = m_workerBusyNs = 0;
        for (uint32_t r = 0; r < kWaitCount; ++r)
        {
            s.waits[r] = m_waits[r];
            s.waitsBlocked[r] = m_waitsBlocked[r];
            s.waitNs[r] = m_waitNs[r];
            s.fallbacks[r] = m_fallbacks[r];
            m_waits[r] = m_waitsBlocked[r] = m_waitNs[r] = m_fallbacks[r] = 0;
        }
        return s;
    }

    static const char *reasonName(uint32_t r)
    {
        static const char *names[kWaitCount] = {"vumem", "gspriv", "gsstub", "frame", "inline", "unpack", "stop"};
        return r < kWaitCount ? names[r] : "?";
    }

    // Game thread, at frame boundaries: a stats line every kReportNs (PS2X_VU1_THREAD_STATS=0 silences it).
    void maybeReport()
    {
        static const bool quiet = []
        {
            const char *v = std::getenv("PS2X_VU1_THREAD_STATS");
            return v && v[0] == '0';
        }();
        const uint64_t now = ps2_host_profile::nowNs();
        if (m_lastReportNs == 0)
        {
            m_lastReportNs = now;
            takeStats();
            return;
        }
        if (now - m_lastReportNs < kReportNs)
            return;
        const double seconds = (now - m_lastReportNs) / 1e9;
        m_lastReportNs = now;
        if (quiet)
            takeStats();
        else
            report(seconds);
    }

    // One stats line covering the last `seconds` (game thread).
    void report(double seconds)
    {
        const Stats s = takeStats();
        char line[1024];
        int n = std::snprintf(line, sizeof(line),
                              "[vu1-thread] %.1fs batches=%.0f/s records=%.0f/s %.1f MB/s wakes=%.0f/s worker busy=%.2f ms/s",
                              seconds, s.batches / seconds, s.records / seconds, s.bytes / 1048576.0 / seconds,
                              s.wakes / seconds, s.workerBusyNs / 1e6 / seconds);
        uint64_t totalWaitNs = 0;
        for (uint32_t r = 0; r < kWaitCount; ++r)
            totalWaitNs += s.waitNs[r];
        n += std::snprintf(line + n, sizeof(line) - n, " game-thread wait=%.2f ms/s", totalWaitNs / 1e6 / seconds);
        for (uint32_t r = 0; r < kWaitCount && n < static_cast<int>(sizeof(line)) - 64; ++r)
        {
            if (s.waits[r] != 0 || s.fallbacks[r] != 0)
                n += std::snprintf(line + n, sizeof(line) - n, " %s=%llu/%llu:%.2fms", reasonName(r),
                                   static_cast<unsigned long long>(s.waits[r]),
                                   static_cast<unsigned long long>(s.waitsBlocked[r]), s.waitNs[r] / 1e6);
            if (s.fallbacks[r] != 0)
                n += std::snprintf(line + n, sizeof(line) - n, "(fallback %llu)",
                                   static_cast<unsigned long long>(s.fallbacks[r]));
        }
        std::fprintf(stderr, "%s\n", line);
    }

private:
    static constexpr size_t kEarlyPublishBytes = 256u * 1024u;
    static constexpr uint64_t kReportNs = 5000000000ull;

    inline static thread_local bool t_onWorker = false;

    void workerLoop()
    {
        t_onWorker = true;
        std::unique_lock<std::mutex> lock(m_mutex);
        for (;;)
        {
            while (m_pending.empty() && !m_stop)
            {
                m_workerSleeping = true;
                m_cvWork.wait(lock);
                m_workerSleeping = false;
            }
            if (m_pending.empty())
                break; // stop requested and nothing left
            std::vector<uint8_t> batch = std::move(m_pending.front());
            m_pending.pop_front();
            m_busy = true;
            const bool skip = m_error != nullptr; // after a failure the back-end state is not trustworthy
            lock.unlock();

            const uint64_t start = ps2_host_profile::nowNs();
            std::exception_ptr error;
            if (!skip)
            {
                try
                {
                    applyBatch(batch);
                }
                catch (...)
                {
                    error = std::current_exception();
                }
            }
            const uint64_t busy = ps2_host_profile::nowNs() - start;

            lock.lock();
            m_workerBusyNs += busy;
            if (error && !m_error)
                m_error = error;
            m_busy = false;
            batch.clear();
            if (m_free.size() < 8u)
                m_free.push_back(std::move(batch));
            if (m_eeWaiting && (m_pending.empty() || m_error))
                m_cvIdle.notify_one();
        }
    }

    void applyBatch(const std::vector<uint8_t> &batch)
    {
        size_t at = 0;
        while (at + sizeof(Record) <= batch.size())
        {
            Record rec;
            std::memcpy(&rec, batch.data() + at, sizeof(rec));
            at += sizeof(Record);
            m_process(rec, batch.data() + at);
            at += (static_cast<size_t>(rec.size) + 15u) & ~static_cast<size_t>(15u);
        }
    }

    ProcessFn m_process;
    std::thread m_thread;

    // Game thread only.
    uint64_t m_framesAppended = 0;
    std::vector<uint8_t> m_batch;
    uint64_t m_batchRecords = 0;
    uint64_t m_lastReportNs = 0;

    // Shared, under m_mutex.
    std::mutex m_mutex;
    std::condition_variable m_cvWork, m_cvIdle;
    std::deque<std::vector<uint8_t>> m_pending;
    static constexpr uint64_t kFrameCsrSlots = 4u; // the game thread waits for frame N-1 after appending frame N
    std::deque<std::function<void()>> m_frameFns;
    uint64_t m_framesDone = 0;
    uint32_t m_frameCsr[kFrameCsrSlots] = {};
    std::vector<std::vector<uint8_t>> m_free;
    bool m_busy = false, m_stop = false, m_workerSleeping = false, m_eeWaiting = false;
    std::exception_ptr m_error;
    uint64_t m_batches = 0, m_records = 0, m_bytes = 0, m_wakes = 0, m_workerBusyNs = 0;
    uint64_t m_waits[kWaitCount] = {}, m_waitsBlocked[kWaitCount] = {}, m_waitNs[kWaitCount] = {};
    uint64_t m_fallbacks[kWaitCount] = {};
};
