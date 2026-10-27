# Parallel tiled GS rasterisation: design and safety review

27 October 2026. Review only; no tiled renderer has been implemented. Baseline is
PS2Recomp `boot-runtime` `b10427d`, with `PS2X_GS_THREAD=1` for windowed runs.
The synchronous renderer remains the default and the headless fallback.

## Evidence and current bottleneck

The existing GS worker serially consumes ordered GIF packets, preserves each
packet's producer floating-point rounding mode, and latches display flips in
stream order (`gs_async_queue.cpp`). Its 4 MiB packet-byte bound permits about
three 3D frames in flight. At fixed guest frames 2800:2880, the Level 5 attract
demo takes 57.2–57.9 ms/frame (17.3–17.5 fps) with the worker; at menu frames
1600:1640 it takes 86.2–88.7 ms/frame (11.3–11.6 fps). These are actual game
frame intervals at `sceGsSyncV`, not presentation rate. The worker is busy about
56 ms and 86 ms respectively, so reducing worker service time can improve
throughput. See `PERFORMANCE.md`, Phase 1. There is no new timed run in this
review.

The available GS stage profile predates the worker but uses the same rasteriser.
On the 3D menu it estimates 73.0 ms/frame in texture sampling (including 26.7
in texture VRAM reads), 22.3 in pixel output, 56.1 inclusive in triangles and
63.5 inclusive in sprites. These estimates overlap and profiling adds overhead;
they cannot be summed or treated as current worker timings. The historical
attract log around host seconds 68–75 likewise reports triangles around 53–57
ms/frame and sprites around 16–18 ms/frame in the instrumented build. This
establishes that both types matter, but does not provide a reliable primitive
size distribution. Before choosing a tile size, add opt-in counters for exact
per-frame primitive counts, clipped bounding-box area distribution, tiles
touched per primitive, textured/untextured mix, and transfer/CLUT frequency
over the two fixed guest-frame ranges. Record worker time with profiling off.

## Ordering model

The GS worker remains the sole command-stream owner. GIF arbitration, register
decoding, transfer sequencing, CLUT loads, TEXFLUSH, and display flips stay in
their existing order. A tiler may dispatch *pixel work* only after the state
for that primitive is immutable. For each destination pixel, primitives must
be applied in original stream order. That preserves alpha and destination-alpha
tests, PABE/alpha blending, framebuffer masks, FBA, depth comparisons and
writes, alpha-test framebuffer-only/depth-only outcomes, and colour conversion.
Do not sort primitives by texture or state. Dithering and colour clamp, if
introduced later, must also be checked for any cross-pixel state.

There are two plausible dispatch strategies:

1. **One primitive at a time, split across disjoint tiles, then join.** This
   gives an obvious stream-order proof and serialises all commands between
   primitives. It is useful as an exactness prototype, but roughly a thousand
   or more small draws per frame could make a barrier per draw expensive.
2. **Ordered tile lists for a command epoch.** Append each primitive to every
   touched tile in stream order, then run tiles concurrently. End the epoch
   before every transfer, CLUT load, TEXFLUSH, framebuffer/texture alias,
   presentation, readback, reset, or other observable command. A tile drains
   its list in order. This amortises dispatch but needs an exact physical-memory
   independence proof for concurrent tiles. It is the potential speed path,
   after the first strategy passes exactness tests.

An epoch barrier means all tile jobs finish with an acquire/release handoff
before the GS worker applies the next serial command. `Submit`, `LoadClut`,
`UploadImage`, `Present`, `SnapshotVram`, `ClearFramebuffer`, and reset currently
use one backend mutex. The worker must never hold that mutex while waiting on
jobs that need it. Readback and host presentation must observe completed jobs.
The existing GS queue's `waitIdle()` is not by itself a tile-pool barrier unless
the worker joins the epoch before declaring an item complete. Shutdown must
drain or cancel only after all workers have stopped touching backend state.

## VRAM and texture hazards

Logical screen rectangles alone do **not** prove disjoint physical writes.
`WritePixel` uses format-specific, swizzled VRAM handlers. CT24 preserves bits
of a 32-bit word; 16/4-bit formats may share a word between logical pixels.
Framebuffer and Z buffers can overlap each other or texture pages through their
base/block settings. Even writes to different (x,y) may then be concurrent
read-modify-writes to one byte or word. A safe first implementation must either
prove disjoint *physical byte ranges* for every active framebuffer/depth format
and mapping, including wraparound, or execute that primitive/epoch serially.
ThreadSanitizer or a race detector can help but is not a substitute for this
mapping proof. Do not use atomics to hide conflicts: they would not preserve
the current ordered read-modify-write semantics.

Texture reads can observe earlier framebuffer or depth writes in the same VRAM.
Render-to-texture, framebuffer feedback, CLUT source aliasing, and transfer
destinations therefore create read-after-write hazards. A conservative policy
is to end an epoch before a textured draw whenever its texture/CLUT pages may
alias any pending framebuffer/depth writes, then draw it serially unless a
per-primitive snapshot reproduces the current sequential read order. Never
assume Maximo avoids feedback from the two existing scenes alone. Host-to-local
uploads, local-to-local copies, CLUT loads (`TEX0.CLD`), `TEXFLUSH`, external
VRAM writes, and clears must drain pending jobs before execution. The current
texture cache invalidates on `TextureFlush` and reset, and `WriteVramUnlocked`
does not invalidate it on every write. Preserve those exact invalidation
points; adding thread-local caches with different visibility can change pixels.

`GSCpuBackend` owns one mutable `TexturePageCache` (two 8 KiB copies), one CLUT
array and CLUT CBP state, transfer state/buffers, VRAM pointer, and mutex.
`TexturePageCache::Resolve` mutates its slot bytes and tags on misses, so it
cannot be called concurrently. Worker jobs need separate cache instances with
an epoch generation/invalidation policy matching the serial renderer. CLUT
contents can be shared read-only only between barriers, with no `LoadClut`
overlap. The GS stage profiler uses thread-local snapshots and sampling RNGs;
aggregation must collect every tile thread, but profiling must be off for
benchmark runs. Packet rounding modes must be set on **every** tile thread
before work, including reused pool threads. Preserve the current floating-point
expression order and the absolute x/y coordinates in interpolation; restarting
recurrences at tile edges or changing compiler FP flags may alter captures.

## Proposed limited prototype and worker counts

Keep `PS2X_GS_TILES` unset/off by default. A future `=1` mode should exercise
the new scheduler while remaining byte-identical to serial output; `=2` and
`=N` then exercise parallelism. Reject invalid values cleanly and cap N to the
number of useful tiles and a modest CPU limit. Use a persistent `std::thread`
pool inside the GS worker rather than starting threads for each primitive.
The full synchronous path, headless path, and VU1 replay stay unchanged.

Start with a small subset: CT32 frame and Z surfaces whose physical address
ranges are provably separate across tile boundaries, no feedback/aliasing, and
no in-epoch transfer or CLUT mutation. Fall back to the exact serial path for
all other cases. Try 64x32 or 64x64 logical tiles only after the size histogram
exists; compare 1, 2, 4 and the useful logical-core count, and measure barrier
time, jobs/tile, cache misses, and serial fallback fraction. The current
`DrawTriangle` and `DrawSprite` recompute per-pixel values from absolute x/y,
which is compatible with clipped tile loops; lines/points can remain serial.
The full-primitive bounding-box and interpolation setup must be preserved so
that tile clipping does not change edge inclusion, texture coordinates or
rounding. No dependency download is required.

The tempting immediate proof of concept is **not yet safe**: physical-address
alias checks, cache ownership, and a primitive size histogram are missing.
This review therefore stops before runtime edits. A serial dry-run tiler could
be developed separately, but cannot establish parallel safety by itself.

## Performance expectations and stop criteria

Attract is bounded by roughly 35 ms/frame of EE plus VU1 (about 28 fps) even
if GS becomes free; menu is bounded by roughly 59 ms/frame (about 17 fps).
Those are arithmetic upper bounds, not tiled-renderer predictions. If a
fraction `p` of the 56/86 ms GS work is safely parallel and `N` workers are
effective, an optimistic service-time model is `Tserial + Tparallel/N +
dispatch + memory contention`. Frequent tiny primitives, feedback barriers,
VRAM bandwidth, per-worker texture cache duplication and producer contention
can erase the gain. A barrier-per-primitive prototype may be slower. Keep it
only if exact and useful as a test scaffold; select batching based on measured
size and hazard distributions. Benchmark guest frame intervals, GS busy time,
producer stalls and presentation separately; report median and spread from
repeated menu 1600:1640 and attract 2800:2880 runs with captures/profile off.

## Validation gate for any future implementation

1. Build and run all 455 runtime tests. Add meaningful synthetic GS cases for
   edge pixels shared by adjacent tiles, huge triangles/sprites, overlapping
   transparent draws, destination-alpha and framebuffer masks, Z equality,
   alpha-test framebuffer-only/depth-only writes, CT16/CT24 packed writes,
   aliasing, feedback, CLUT reloads, TEXFLUSH, host uploads, and reset/readback.
   Compare full VRAM, not just the presented surface, at 1/2/N threads and
   across repeated schedules. Run a race detector where available.
2. Run the 1,000-call VU1 replay against `vu1-sequence.expect`; require `0
   differ from live; all expected digests match`. It checks VU1/PATH1, not
   full GS command equivalence.
3. Run `recomp/compare/validate.sh <label> <exe> 0` in both inline and
   `PS2X_GS_THREAD=1` modes, with tiling disabled and separately at 1/2/N.
   Require exact reference hashes for menu frames 1600/1620/1639 and attract
   frames 2800/2840/2879. Inspect scene pixels if any mismatch occurs; no
   tolerance or timing dismissal. Verify a headless boot and windowed runs.
4. Only after exactness, run `bench.sh` with repeated fixed guest-frame ranges
   and report actual game-frame throughput. Keep the previous `b10427d`
   checkpoint as rollback. Do not claim full-speed gameplay from these two
   scenes; other levels and cut-scenes remain unverified.

The six hash comparisons establish equivalence with the current CPU renderer,
not correctness against PS2 hardware. The five existing visual questions still
need a trusted reference comparison.
