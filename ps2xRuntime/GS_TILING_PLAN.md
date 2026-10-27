# Parallel tiled GS rasterisation: design and safety review

27 October 2026. Investigation and opt-in instrumentation only; no tiled renderer
has been implemented. Starting baseline: PS2Recomp `boot-runtime` `c371604`
(same renderer as `b10427d`), with `PS2X_GS_THREAD=1` for windowed runs.
The synchronous renderer remains the default and the headless fallback.

## Investigation result and recommendation

**Go for a narrow, guarded tiled prototype after owner approval.** Large
primitives account for 62% of measured draw time in the menu and 85% in attract
gameplay, so there is enough parallel work to justify an experiment. A general
tile renderer is not yet safe: framebuffer, Z and texture physical address sets
must be checked before parallel writes, and texture-cache state must be private.
Begin with an opt-in, physically disjoint subset and retain serial execution
for all other commands. Broadening the subset requires byte-identical full
VRAM and six captured 3D frames at 1/2/N workers, plus a measured game-frame
gain after scheduling cost. No architectural change is authorised here.

## New measurement: primitive distribution

`PS2_GS_PRIM_PROFILE=1` records clipped bounding-box pixels, covered raster
pixels, primitive type, texture/blend flags and draw time on the GS worker.
`PS2_GS_PRIM_RANGE=begin:end` selects display-flip numbers to print. Normal
runs do no counting or logging. Coverage is raster pixel visits before
alpha/depth tests, not unique screen area. A separate edge pass counts triangle
coverage after the draw timer stops; it adds profile-only overhead. Size bins
are 0, 1-16, 17-64, 65-256, 257-1024, 1025-4096, and >4096 clipped box
pixels. Draw time includes texture sampling and pixel output, and excludes the
coverage pass. It is useful for relative cost by size, not unprofiled GS time.

Runs: `recomp/boot/runs/tiling-menu-20261027` (flips 1600:1640, 40 frames)
and `tiling-attract-20261027` (flips 2800:2880, 80 frames), both windowed with
`PS2X_GS_THREAD=1`, using the same scripts and guest ranges as `bench.sh`.
`runs/analyze_tiling_profile.py` aggregates the raw worker rows. A display
flip counter can differ from the guest-frame counter in another scene; here
the one-per-frame flips align with the named guest ranges.

| Scene | Type | Draws/frame | Raster visits/frame | Timed draw ms/frame | >4096-box draws/frame | >4096-box timed ms/frame |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Menu | Triangle | 12,093 | 531,000 | 38.71 | 79.0 | 19.05 |
| Menu | Sprite | 248 | 967,000 | 33.43 | 39.1 | 25.84 |
| Attract | Triangle | 5,145 | 542,000 | 38.06 | 120.5 | 31.02 |
| Attract | Sprite | 109 | 649,000 | 13.66 | 18.0 | 13.19 |

Menu triangles with a zero clipped box average 4,801 per frame but consume
only 0.28 timed ms; 257 triangles with 1025-4096 pixels consume 8.70 ms. The
menu's >4096-pixel draws are just 118 of about 12,341 draws yet consume
44.89/72.14 timed draw ms (62%). Including 1025-4096-pixel draws raises the
share to about 84%. Attract's >4096-pixel draws are about 139 of 5,254 draws
and consume 44.21/51.72 ms (85%). Large menu sprites are almost all textured
and blended; all large attract triangles are textured and about 42% have
blending enabled. Total raster visits are about 1.50 million/menu frame and
1.19 million/attract frame; these count overdraw, not unique displayed pixels.

The instrumented guest-frame rates were 10.8 fps menu and 16.7 fps attract;
the edge pass and logging make those unsuitable as speed baselines. With all
profiling disabled, two repeated fixed-range runs of the same binary measured
menu **84.86-86.40 ms/frame (11.6-11.8 fps)**, GS worker 83.64-85.13 ms;
attract **55.92-55.95 ms/frame (17.9 fps)**, GS worker 54.68-54.77 ms.
These are actual game-frame intervals, not presentation FPS. They are broadly
consistent with the older session; the small difference is not attributed to
the profiler code.

`PS2_GS_PROFILE=1` uses thread-local stage counters that the boot runner reads
on the game thread. With the optional GS worker enabled, those stage rows do
not reliably describe worker stages. The older stage profile below locates hot
code but cannot be combined numerically with the new worker histogram.

## Evidence and current bottleneck

The existing GS worker serially consumes ordered GIF packets, preserves each
packet's producer floating-point rounding mode, and latches display flips in
stream order (`gs_async_queue.cpp`). Its 4 MiB packet-byte bound permits about
three 3D frames in flight. At fixed guest frames 2800:2880, the Level 5 attract
demo takes 57.2–57.9 ms/frame (17.3–17.5 fps) with the worker; at menu frames
1600:1640 it takes 86.2–88.7 ms/frame (11.3–11.6 fps). These are actual game
frame intervals at `sceGsSyncV`, not presentation rate. The worker is busy about
56 ms and 86 ms respectively, so reducing worker service time can improve
throughput. See `PERFORMANCE.md`, Phase 1, and the new runs above.

The available GS stage profile predates the worker but uses the same rasteriser.
On the 3D menu it estimates 73.0 ms/frame in texture sampling (including 26.7
in texture VRAM reads), 22.3 in pixel output, 56.1 inclusive in triangles and
63.5 inclusive in sprites. These estimates overlap and profiling adds overhead;
they cannot be summed or treated as current worker timings. The historical
attract log around host seconds 68–75 likewise reports triangles around 53–57
ms/frame and sprites around 16–18 ms/frame in the instrumented build. This
establishes that both types matter. The new worker histogram supplies primitive
counts, clipped box areas and coverage. It does not yet count tile intersections,
active target/texture formats, or transfer/CLUT frequency per guest range;
those are useful before choosing final tile and eligibility rules.

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
   primitives. It is useful as an exactness prototype, but 12,341 menu draws
   per frame make a barrier per draw likely expensive.
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
`WritePixel` dispatches framebuffer and Z reads/writes by PSM. The address
traits in `ps2_gs_memory.h` compute a page from `base / 32`, buffer width and
the x/y page coordinates; a PSM-specific block and column table supplies the
swizzled offset. They convert that to a bit offset, then mask the byte address
into 4 MiB of VRAM. CT32/Z32 write four bytes, CT16/Z16 write two. CT24/Z24
read and rewrite the full four-byte word to preserve its upper byte. Indexed
4-bit writes (used by transfers, not the normal colour output path) do a
byte-level read-modify-write. Pixel formats sharing the C32 address table
(including CT24 and high-byte indexed textures) can alias the same words.

Within one page-aligned CT32 framebuffer, distinct pixels have distinct
four-byte locations and a 64x32 page-aligned split is a plausible disjoint
partition. CT16 uses a 64x64 page; a 64x32 split shares a *page* but its
individual two-byte pixel locations can still be distinct. Therefore the
previous statement that adjacent CT16 pixels necessarily race on a 32-bit
word was too broad: this code copies two bytes for CT16. The proof must be at
the byte level, not page membership alone. Frame base (`FBP`) and Z base
(`ZBP`) are each converted from pages to blocks, but their swizzle tables and
buffer widths can differ. Frame and Z may overlap each other or texture pages
through their base/layout settings, and the 4 MiB mask can wrap addresses.
Two different screen tiles can therefore access the same physical bytes via
*different surfaces*, even when each single surface is injective. The current
histogram did not capture actual FBP/ZBP/PSM/TBP tuples, so eligibility of the
measured large draws remains **unmeasured**. A safe first implementation must
trace those tuples and prove disjoint physical byte sets for all framebuffer
and Z reads/writes of concurrently scheduled tiles; otherwise execute the
draw or epoch serially. Avoid a guard based only on base-page equality or
texture bounding rectangles.
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
all other cases. Try 64x32 or 64x64 logical tiles as starting points; compare
1, 2, 4 and the useful logical-core count, and measure barrier
time, jobs/tile, cache misses, and serial fallback fraction. The current
`DrawTriangle` and `DrawSprite` recompute per-pixel values from absolute x/y,
which is compatible with clipped tile loops; lines/points can remain serial.
The full-primitive bounding-box and interpolation setup must be preserved so
that tile clipping does not change edge inclusion, texture coordinates or
rounding. No dependency download is required.

The size histogram now exists, but physical-address alias checks and private
cache ownership remain prerequisites. This investigation stops before tiler
implementation. A serial dry-run tiler alone cannot establish parallel safety.

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

For scale only, applying four effective raster workers to **25%** of current
GS service time with 5 ms dispatch/memory overhead gives roughly 74 ms menu
GS and 49 ms attract GS, or about 14 and 20 fps if the game thread does not
slow. At **50%** eligibility it gives roughly 58 ms menu GS and 39 ms attract
GS; the menu's roughly 60 ms EE+VU1 floor and attract's roughly 35 ms floor
then limit rates to about 17 and 26 fps. The 25/50% eligibility, four effective
workers and 5 ms overhead are **assumptions, not measurements**. The measured
large-draw share is an upper bound on attractive candidates, not proof of
eligibility. A useful result requires measuring physical hazards, serial
fallback fraction, actual worker efficiency, VU1 slowdown from core sharing,
and queue stalls. A 64x32 or 64x64 tile is only an initial experiment.

**Go/no-go for implementation and continuation:** first verify that the large
draws in these ranges can be partitioned into disjoint physical byte sets, or
derive a conservative runtime guard. Keep all ambiguous formats, aliases,
feedback and transfers serial. After implementation, require zero data races,
full-VRAM agreement on synthetic alias/format tests, all six exact game-frame
hashes at 1/2/N workers, 455/455 tests and VU1 replay agreement. Continue
performance work only if at least two repeated unprofiled guest-frame runs
show a clear gain beyond the current run-to-run spread in *both* scenes,
without increasing queue latency or VU1 time enough to erase it. If the guard
serialises most of the >1024-pixel draw time, or overhead consumes the gain,
stop tiling and target the large textured sprite/triangle pixel paths first.

## Validation gate for any future implementation

For this instrumentation change, the final build passed 455/455 tests and the
1,000-call VU1 replay (`0 differ from live; all expected digests match`).
`validate.sh` with zero benchmark repetitions matched all six reference 3D
frame hashes with the GS worker on and with the synchronous renderer. An
initial instrumented build had 12 differing pixels at menu frame 1600 with
profiling **off**; placing a profiler check in the hot triangle loop changed
floating-point code generation. That build was rejected. The final version
counts triangle coverage in a separate opt-in pass and passed both modes.
This is evidence that even apparently inert hot-loop changes need the full
pixel gate. The 3D captures establish equivalence to the current CPU renderer,
not hardware accuracy.

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
