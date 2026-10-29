#include "runtime/ps2_present_pacing.h"

#include <thread>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif
#endif

namespace ps2_present_pacing
{
    void sleepUntilNs(int64_t ns)
    {
        const int64_t now = toNs(std::chrono::steady_clock::now());
        if (ns <= now)
            return;
#if defined(_WIN32)
        // A high-resolution waitable timer (Windows 10 1803+) wakes within a fraction of a millisecond, where
        // Sleep and condition variables round to the 1 ms timer tick. One timer per presenting thread.
        thread_local HANDLE timer = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                                                           TIMER_ALL_ACCESS);
        if (timer)
        {
            LARGE_INTEGER due;
            due.QuadPart = -((ns - now) / 100); // relative, in 100 ns units
            if (SetWaitableTimer(timer, &due, 0, nullptr, nullptr, FALSE))
            {
                WaitForSingleObject(timer, INFINITE);
                return;
            }
        }
#endif
        std::this_thread::sleep_for(std::chrono::nanoseconds(ns - now));
    }
}
