#include "MiniTest.h"
#include "ps2x/iop/ps2_path.h"
#include "../../ps2xIOP/tests/iop_compat_test_support.h"
#include "emulator/core/iop_memory.h"

void register_ps2_iop_tests()
{
    MiniTest::Case("PS2IopSubsystem", [](TestCase &tc)
    {
        tc.Run("PS2 path parsing is shared and normalizes ISO/module names", [](TestCase &t)
        {
            const ps2x::iop::ParsedPs2Path cd = ps2x::iop::parsePs2Path("CDROM0:\\MODULES\\LIBSD.IRX;1");
            t.Equals(cd.device, ps2x::iop::Ps2PathDevice::Cdrom,
                     "device names should be case-insensitive");
            t.Equals(cd.path, std::string("MODULES/LIBSD.IRX"),
                     "separators and ISO version suffixes should normalize once");
            t.Equals(ps2x::iop::ps2PathLeafKey(cd), std::string("libsd"),
                     "module lookup should use a normalized IRX leaf key");

            const ps2x::iop::ParsedPs2Path rom = ps2x::iop::parsePs2Path("rom0:ROMVER");
            t.Equals(rom.device, ps2x::iop::Ps2PathDevice::Rom0,
                     "ROM0 should remain a distinct virtual device");
            t.IsFalse(static_cast<bool>(ps2x::iop::parsePs2Path("unknown0:file.irx")),
                      "unsupported devices must not fall through to cdrom0");
        });

        tc.Run("HLE services activate only after a recognized module load", [](TestCase &t)
        {
            iop_test::Host host;
            ps2x::iop::IopSubsystem subsystem(host);
            t.IsFalse(subsystem.canBindRpc(0x80000701u),
                      "LIBSD RPC must not exist before LIBSD is loaded");

            const ps2x::iop::ModuleLoadResult unknown = subsystem.loadModule("rom0:NOT_A_REAL_MODULE");
            t.IsTrue(unknown.handled, "the module manager should return a real load result");
            t.IsTrue(unknown.moduleId < 0, "unknown ROM modules must fail instead of receiving fake IDs");

            const ps2x::iop::ModuleLoadResult loaded = subsystem.loadModule("rom0:LIBSD");
            t.IsTrue(loaded.moduleId > 0, "a registered no-BIOS HLE module should load");
            t.IsTrue(subsystem.canBindRpc(0x80000701u),
                     "loading LIBSD should activate its HLE RPC endpoint");

            ps2x::iop::RpcRequest request{};
            request.sid = 0x80000701u;
            request.function = 3u;
            t.IsTrue(subsystem.handleRpc(request).handled,
                     "the activated LIBSD service should handle its RPC");
            t.Equals(host.audioCalls, 1u, "the RPC should reach the HLE audio contract");

            int32_t stopResult = -1;
            t.IsTrue(subsystem.stopModule(loaded.moduleId, &stopResult),
                     "an HLE module should have a real stoppable lifecycle");
            t.Equals(stopResult, 0, "stopping an HLE module should report success");
            t.IsFalse(subsystem.canBindRpc(0x80000701u),
                      "stopping LIBSD should deactivate its RPC endpoint");
        });

        tc.Run("unknown SID remains unhandled without a loaded IRX", [](TestCase &t)
        {
            iop_test::Host host;
            ps2x::iop::IopSubsystem subsystem(host);

            ps2x::iop::RpcRequest request{};
            request.sid = 0xDEADC0DEu;
            request.function = 0x99u;
            const ps2x::iop::RpcResult result = subsystem.handleRpc(request);
            t.IsFalse(result.handled, "an unknown SID should not be claimed by the IOP subsystem");
            t.Equals(result.resultAddress, 0u, "an unknown SID should not return a guest result address");
            t.IsFalse(result.signalNowaitCompletion, "an unknown SID should not signal nowait completion");
            t.Equals(result.callbackPolicy, ps2x::iop::CallbackPolicy::RuntimeDefault,
                     "an unknown SID should preserve runtime callback handling");

        });

        tc.Run("IOP heap reuses freed blocks first-fit and reports its free space", [](TestCase &t)
        {
            using ps2x::iop::detail::IopMemory;
            IopMemory memory;
            const uint32_t heap = IopMemory::HeapLimit - IopMemory::HeapBase;
            t.Equals(memory.maxFreeMemory(), heap, "an empty heap is one free block");

            // MJOLNIR's bank switch: a bank buffer and a scratch block, both freed before the next bank is loaded.
            for (int load = 0; load < 64; ++load)
            {
                const uint32_t bank = memory.allocate(0x18A00u, 16u);
                const uint32_t scratch = memory.allocate(0x6280u, 16u);
                t.Equals(bank, IopMemory::HeapBase, "the bank buffer goes back to the bottom of the heap");
                t.Equals(scratch, IopMemory::HeapBase + 0x18A00u, "the scratch block follows it");
                t.IsTrue(memory.freeAllocation(scratch), "free the scratch block");
                t.IsTrue(memory.freeAllocation(bank), "free the bank buffer");
            }
            t.Equals(memory.totalFreeMemory(), heap, "repeated loads leave the whole heap free");

            const uint32_t a = memory.allocate(0x1000u, 16u);
            const uint32_t b = memory.allocate(0x2000u, 16u);
            const uint32_t c = memory.allocate(0x1000u, 16u);
            t.IsTrue(memory.freeAllocation(b), "free the middle block");
            t.Equals(memory.totalFreeMemory(), heap - 0x2000u, "total free counts the hole");
            t.Equals(memory.maxFreeMemory(), heap - 0x4000u, "largest free block is the one above c");
            t.Equals(memory.allocate(0x3000u, 16u), c + 0x1000u, "a block too big for the hole goes above c");
            t.Equals(memory.allocate(0x800u, 16u), a + 0x1000u, "a block that fits the hole goes in it");
            t.Equals(memory.allocate(heap, 16u), 0u, "a block larger than the free space fails");
        });

    });
}
