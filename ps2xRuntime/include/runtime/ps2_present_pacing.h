#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>

// PS2X_PRESENT_PACING: how the window paces its presents. Unset (the default), raylib's own 60 Hz timer.
//   vblank   : once per guest VBlank, woken by the EE scheduler when it processes the VBlank.
//   deadline : once per guest VBlank, at the VBlank's host deadline on a high-resolution timer, so the window is
//              not delayed by the game thread's own wake-up latency. The frame for that VBlank is already published
//              (the guest publishes before it waits for the VBlank), so this adds no latency.
namespace ps2_present_pacing
{
    struct State
    {
        std::atomic<bool> enabled{false};
        std::atomic<int64_t> nextDeadlineNs{0}; // steady_clock time of the next VBlank's host deadline
        std::mutex mutex;
        std::condition_variable cv;
        uint64_t vblanks = 0;
    };

    inline State &state()
    {
        static State s;
        return s;
    }

    inline int64_t toNs(std::chrono::steady_clock::time_point t)
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
    }

    // Set by the presenter before the game thread starts.
    inline void enable() { state().enabled.store(true, std::memory_order_release); }

    // EE scheduler, at each VBlank start, with the host deadline it scheduled for the next one.
    inline void notifyVBlank(std::chrono::steady_clock::time_point nextDeadline)
    {
        State &s = state();
        if (!s.enabled.load(std::memory_order_acquire))
            return;
        {
            std::lock_guard<std::mutex> lock(s.mutex);
            ++s.vblanks;
            s.nextDeadlineNs.store(toNs(nextDeadline), std::memory_order_release);
        }
        s.cv.notify_one();
    }

    // Presenter: waits until a VBlank after `seen` (or the timeout), then records the latest one in `seen`.
    // Returns false on a timeout.
    inline bool waitVBlank(uint64_t &seen, std::chrono::milliseconds timeout)
    {
        State &s = state();
        std::unique_lock<std::mutex> lock(s.mutex);
        const bool woke = s.cv.wait_for(lock, timeout, [&]() { return s.vblanks != seen; });
        seen = s.vblanks;
        return woke;
    }

    // Sleeps until the steady_clock time `ns` on the platform's most precise sleeping timer (no busy wait).
    void sleepUntilNs(int64_t ns);
}
