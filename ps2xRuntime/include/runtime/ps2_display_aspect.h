#pragma once

#include <atomic>
#include <cstdint>

// Optional 16:9 presentation for a game-side widescreen mode. The host enables it before PS2Runtime::initialize and
// reports each frame in which the game prepared its widescreen 3D view; the presenter shows 16:9 while those reports
// keep coming and 4:3 otherwise (movies, 2D-only screens). Disabled, the presenter is unchanged.
namespace ps2_display_aspect
{
    inline std::atomic<bool> &wideEnabled()
    {
        static std::atomic<bool> enabled{false};
        return enabled;
    }

    inline std::atomic<uint64_t> &wideFrames()
    {
        static std::atomic<uint64_t> frames{0};
        return frames;
    }

    inline void enableWide() { wideEnabled().store(true, std::memory_order_relaxed); }
    inline bool isWideEnabled() { return wideEnabled().load(std::memory_order_relaxed); }
    inline void noteWideFrame() { wideFrames().fetch_add(1, std::memory_order_relaxed); }
    inline uint64_t wideFrameCount() { return wideFrames().load(std::memory_order_relaxed); }

    // The size in window pixels of the picture rectangle the presenter last drew (width << 32 | height, 0 before the
    // first present), so a host can render the frame at exactly that size (e.g. GPU upscaling). Purely informative.
    inline std::atomic<uint64_t> &presentTarget()
    {
        static std::atomic<uint64_t> target{0};
        return target;
    }
    inline void notePresentTarget(uint32_t width, uint32_t height)
    {
        presentTarget().store(uint64_t(width) << 32 | height, std::memory_order_relaxed);
    }
    inline bool presentTargetSize(uint32_t &width, uint32_t &height)
    {
        const uint64_t v = presentTarget().load(std::memory_order_relaxed);
        width = uint32_t(v >> 32);
        height = uint32_t(v);
        return width != 0 && height != 0;
    }
}
