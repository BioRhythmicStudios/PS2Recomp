#pragma once

#include <cstdint>

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

bool enabled();
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
