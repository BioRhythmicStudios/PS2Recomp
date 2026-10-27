#include "runtime/gs/gs_async_queue.h"

#include "runtime/ps2_host_profile.h"

#include <cfenv>

GSAsyncQueue::GSAsyncQueue(GS &gs, size_t maxQueuedBytes)
    : m_gs(gs), m_maxQueuedBytes(maxQueuedBytes)
{
    m_worker = std::thread([this] { run(); });
    m_workerId = m_worker.get_id();
}

GSAsyncQueue::~GSAsyncQueue()
{
    stop();
}

void GSAsyncQueue::submitPacket(std::vector<uint8_t> &&bytes)
{
    if (bytes.empty())
        return;
    const int roundingMode = std::fegetround();
    std::unique_lock<std::mutex> lock(m_mutex);
    if (m_queuedBytes != 0 && m_queuedBytes + bytes.size() > m_maxQueuedBytes)
    {
        // Producer stall: charged separately so the frame report can show how long the game waited on the GS.
        ps2_host_profile::Scope stall(ps2_host_profile::kGsWait);
        m_spaceAvailable.wait(lock, [&] { return m_queuedBytes == 0 || m_queuedBytes + bytes.size() <= m_maxQueuedBytes; });
    }
    m_queuedBytes += bytes.size();
    Item item;
    item.roundingMode = roundingMode;
    item.bytes = std::move(bytes);
    m_items.push_back(std::move(item));
    lock.unlock();
    m_itemsAvailable.notify_one();
}

void GSAsyncQueue::submitDisplayRegs(const GSDisplayRegs &regs)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    Item item;
    item.isDisplay = true;
    item.regs = regs;
    m_items.push_back(std::move(item));
    lock.unlock();
    m_itemsAvailable.notify_one();
}

bool GSAsyncQueue::idleLocked() const
{
    return m_items.empty() && !m_workerBusy && !m_flipPending;
}

void GSAsyncQueue::waitIdle()
{
    if (onWorkerThread())
        return;
    std::unique_lock<std::mutex> lock(m_mutex);
    if (idleLocked())
        return;
    ps2_host_profile::Scope stall(ps2_host_profile::kGsWait);
    m_idle.wait(lock, [&] { return idleLocked(); });
}

void GSAsyncQueue::stop()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_worker.joinable())
            return;
        m_stopping = true;
    }
    m_itemsAvailable.notify_one();
    m_worker.join();
}

void GSAsyncQueue::latchPendingFlip()
{
    // The presenter latched in the default rounding mode; so does the worker.
    const int roundingMode = std::fegetround();
    if (roundingMode != FE_TONEAREST)
        std::fesetround(FE_TONEAREST);
    ps2_host_profile::Scope present(ps2_host_profile::kPresent);
    m_gs.latchHostPresentationFrame();
    if (roundingMode != FE_TONEAREST)
        std::fesetround(roundingMode);
}

void GSAsyncQueue::run()
{
    for (;;)
    {
        Item item;
        bool latchFirst = false;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            if (m_items.empty() && m_flipPending)
            {
                // Everything submitted so far is drawn: the waiting flip can be latched now.
                m_workerBusy = true;
                lock.unlock();
                latchPendingFlip();
                lock.lock();
                m_flipPending = false;
                m_workerBusy = false;
            }
            if (m_items.empty())
            {
                m_idle.notify_all();
                m_itemsAvailable.wait(lock, [&] { return !m_items.empty() || m_stopping; });
                if (m_items.empty())
                    return; // stopping, and fully drained
            }
            item = std::move(m_items.front());
            m_items.pop_front();
            m_workerBusy = true;
            // The game writes several display registers per flip with no packets between them: latch once, after
            // the last of them and before the first packet that follows.
            latchFirst = !item.isDisplay && m_flipPending;
        }

        if (latchFirst)
            latchPendingFlip();
        if (item.isDisplay)
            m_gs.setQueuedDisplayRegs(item.regs);
        else
        {
            if (std::fegetround() != item.roundingMode)
                std::fesetround(item.roundingMode);
            m_gs.processGIFPacket(item.bytes.data(), static_cast<uint32_t>(item.bytes.size()));
        }

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (latchFirst)
                m_flipPending = false;
            if (item.isDisplay)
                m_flipPending = true;
            else
                m_queuedBytes -= item.bytes.size();
            m_workerBusy = false;
        }
        m_spaceAvailable.notify_one();
    }
}
