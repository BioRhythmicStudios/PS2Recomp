#include "iop_stdio.h"

#include "../core/iop_cpu.h"
#include "../core/iop_memory.h"
#include "ps2x/iop/iop_host.h"

#include <cstdio>
#include <cstring>
#include <string>

namespace ps2x::iop::detail
{
    namespace
    {
        // Expands a printf format with the IOP o32 arguments: `firstArgument` counts from a0; arguments past a3 are
        // on the caller's stack at sp+16.
        std::string formatGuest(const IopMemory &memory, const IopCpuState &cpu, const std::string &format,
                                uint32_t firstArgument)
        {
            uint32_t next = firstArgument;
            const auto argument = [&]() -> uint32_t
            {
                const uint32_t index = next++;
                return index < 4u ? cpu.gpr[4u + index] : memory.read32(cpu.gpr[29] + 16u + (index - 4u) * 4u);
            };
            std::string out;
            for (size_t i = 0; i < format.size(); ++i)
            {
                if (format[i] != '%')
                {
                    out += format[i];
                    continue;
                }
                std::string spec = "%";
                ++i;
                while (i < format.size() && std::strchr("-+ #0123456789.", format[i]))
                    spec += format[i++];
                while (i < format.size() && (format[i] == 'l' || format[i] == 'h'))
                    ++i;
                if (i >= format.size())
                    break;
                const char conversion = format[i];
                char buffer[256];
                switch (conversion)
                {
                case '%':
                    out += '%';
                    continue;
                case 's':
                {
                    const std::string text = memory.readString(argument(), 512u);
                    std::snprintf(buffer, sizeof(buffer), (spec + 's').c_str(), text.c_str());
                    break;
                }
                case 'c':
                    std::snprintf(buffer, sizeof(buffer), (spec + 'c').c_str(), static_cast<int>(argument() & 0xFFu));
                    break;
                case 'd':
                case 'i':
                    std::snprintf(buffer, sizeof(buffer), (spec + 'd').c_str(), static_cast<int32_t>(argument()));
                    break;
                case 'u':
                case 'x':
                case 'X':
                case 'o':
                    std::snprintf(buffer, sizeof(buffer), (spec + conversion).c_str(), argument());
                    break;
                case 'p':
                    std::snprintf(buffer, sizeof(buffer), "0x%08x", argument());
                    break;
                default:
                    buffer[0] = '%';
                    buffer[1] = conversion;
                    buffer[2] = '\0';
                    break;
                }
                out += buffer;
            }
            return out;
        }
    }

    IopStdio::IopStdio(IopHost &host, IopMemory &memory) noexcept
        : m_host(host), m_memory(memory)
    {
    }

    bool IopStdio::dispatchImport(uint16_t ordinal, IopCpuState &cpu)
    {
        const uint32_t a0 = cpu.gpr[4];
        const uint32_t a1 = cpu.gpr[5];
        const auto setV0 = [&](uint32_t value)
        {
            cpu.gpr[2] = value;
        };
        const auto logString = [&](std::string_view prefix, uint32_t address, uint32_t resultBias = 0u)
        {
            const std::string text = m_memory.readString(address, 2048u);
            m_host.log(LogLevel::Info, std::string(prefix) + text);
            setV0(static_cast<uint32_t>(text.size()) + resultBias);
        };

        switch (ordinal)
        {
        case 4: // printf
        {
            const std::string text = formatGuest(m_memory, cpu, m_memory.readString(a0, 2048u), 1u);
            m_host.log(LogLevel::Info, "[IOP printf] " + text);
            setV0(static_cast<uint32_t>(text.size()));
            return true;
        }
        case 5: // getchar
        case 10:
            setV0(0xFFFFFFFFu);
            return true;
        case 6: // putchar
            m_host.log(LogLevel::Info, std::string("[IOP putchar] ") + static_cast<char>(a0 & 0xFFu));
            setV0(a0 & 0xFFu);
            return true;
        case 7: // puts
            logString("[IOP puts] ", a0, 1u);
            return true;
        case 8: // gets
        case 13:
            setV0(0u);
            return true;
        case 9: // fdprintf
            logString("[IOP fdprintf] ", a1);
            return true;
        case 11:
            setV0(a0 & 0xFFu);
            return true;
        case 12: // fdputs
            logString("[IOP fdputs] ", a0);
            return true;
        case 14: // vfdprintf
            logString("[IOP vfdprintf] ", a1);
            return true;
        default:
            return false;
        }
    }
}
