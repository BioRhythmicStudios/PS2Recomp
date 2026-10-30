#pragma once

#include <atomic>

// Why PS2Runtime::run's window loop ended, for the host's shutdown log (static strings only). The first reason
// recorded wins.
namespace ps2_exit_reason
{
    inline std::atomic<const char *> &value()
    {
        static std::atomic<const char *> reason{nullptr};
        return reason;
    }

    inline void set(const char *reason)
    {
        const char *expected = nullptr;
        value().compare_exchange_strong(expected, reason);
    }

    inline const char *get()
    {
        const char *reason = value().load();
        return reason ? reason : "unknown";
    }
}
