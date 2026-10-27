#pragma once

#include <cstdint>
#include <cstdlib>

// Opt-in GS timing, enabled by PS2_GS_PROFILE=1. The counters are consumed on
// the rendering thread at the host's one-second performance reporting point.
namespace gs_cpu_profile
{
enum Stage : unsigned
{
    Triangle, Sprite, Pixel, Texture, TextureRead, FrameRead, DepthRead,
    Blend, FrameWrite, DepthWrite, PageCopy, Count
};

struct Snapshot
{
    uint64_t nanoseconds[Count]{};
    uint64_t calls[Count]{};
};

// Worker-owned, opt-in primitive histogram. Bucket edges are clipped bounding-box
// pixels: 0, 1-16, 17-64, 65-256, 257-1024, 1025-4096, >4096.
struct PrimitiveBin
{
    uint64_t count = 0, nanoseconds = 0, boxPixels = 0, coveredPixels = 0;
    uint64_t textured = 0, blended = 0;
};
struct PrimitiveSnapshot
{
    PrimitiveBin bins[3][7]{}; // triangle, sprite, other
};
inline bool primitiveEnabled()
{
    static const bool value = [] { const char *v = std::getenv("PS2_GS_PRIM_PROFILE"); return v && v[0] == '1'; }();
    return value;
}
void recordPrimitive(unsigned type, uint64_t boxPixels, uint64_t coveredPixels,
                     bool textured, bool blended, uint64_t nanoseconds);
PrimitiveSnapshot takePrimitives();

// Inline: the per-pixel paths test this, and an out-of-line call cost about 1.4% of a 3D frame.
inline bool enabled()
{
    static const bool value = [] { const char *v = std::getenv("PS2_GS_PROFILE"); return v && v[0] == '1'; }();
    return value;
}
Snapshot take();
void add(Stage stage, uint64_t nanoseconds, uint64_t calls = 1);
uint64_t now();
uint64_t timerOverhead();

class Scope
{
public:
    Scope(Stage stage, bool selected) : m_stage(stage), m_start(selected ? now() : 0) {}
    ~Scope() { if (m_start) add(m_stage, now() - m_start); }
private:
    Stage m_stage;
    uint64_t m_start;
};
}
