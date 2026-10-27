#pragma once

#include "runtime/gs/gs_frontend.h"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

// Runs GS work on its own thread (PS2X_GS_THREAD=1, windowed runs only).
//
// The game thread stays the only producer: the GIF arbiter hands its packets over in arbitration order and the
// privileged display registers are snapshotted whenever the game writes them. The worker replays both in order
// through the ordinary GS entry points, so the GS sees exactly the stream it sees when it runs inline. A display
// flip is latched for presentation once the worker has processed every packet submitted before it, and before any
// packet submitted after it.
//
// The floating-point rounding mode is part of that stream: VU1 runs in round-toward-zero and rasterises the
// packets it kicks inline under it, while PATH2/3 packets are drawn in the default mode. Each packet records the
// producer's mode when it is handed over, and the worker draws it in that mode, so results stay byte-identical.
//
// The game never reads GS results back in the scenes measured (GS_THREADING_PLAN.md); paths that would are made
// synchronous through waitIdle() by the GS itself.
class GSAsyncQueue
{
public:
    // maxQueuedBytes bounds the packet bytes in flight; the producer blocks when it is reached.
    GSAsyncQueue(GS &gs, size_t maxQueuedBytes);
    ~GSAsyncQueue();
    GSAsyncQueue(const GSAsyncQueue &) = delete;
    GSAsyncQueue &operator=(const GSAsyncQueue &) = delete;

    void submitPacket(std::vector<uint8_t> &&bytes);
    void submitDisplayRegs(const GSDisplayRegs &regs);

    // Blocks until every submitted item has been processed and a pending flip has been latched.
    void waitIdle();
    // Drains the queue and joins the worker.
    void stop();

    bool onWorkerThread() const { return std::this_thread::get_id() == m_workerId; }

private:
    struct Item
    {
        bool isDisplay = false;
        int roundingMode = 0; // the producer's fegetround() at hand-over
        std::vector<uint8_t> bytes;
        GSDisplayRegs regs{};
    };

    void run();
    void latchPendingFlip();
    bool idleLocked() const;

    GS &m_gs;
    const size_t m_maxQueuedBytes;
    std::mutex m_mutex;
    std::condition_variable m_itemsAvailable;
    std::condition_variable m_spaceAvailable;
    std::condition_variable m_idle;
    std::deque<Item> m_items;
    size_t m_queuedBytes = 0;
    bool m_workerBusy = false;
    bool m_flipPending = false; // display registers changed and the flip is not latched yet
    bool m_stopping = false;
    std::thread m_worker;
    std::thread::id m_workerId;
};
