#ifndef PS2_PAD_H
#define PS2_PAD_H

#include <cstddef>
#include <cstdint>

class PSPadBackend
{
public:
    PSPadBackend() = default;
    ~PSPadBackend() = default;

    bool readState(int port, int slot, uint8_t *data, size_t size);
};

// Converts a host analog stick (each axis -1..1, round gate) to DualShock 2 stick bytes (0x00..0xFF, 0x80 centred).
// A DualShock 2 reaches the corners of its square range when pushed fully on a diagonal; a round-gate stick only
// reaches about 0.71 per axis there. Games judge the push by the larger axis (Maximo runs above about 0.8 and walks
// below), so the stick is mapped from its circle onto the square: same direction, larger axis = distance pushed.
void ps2PadStickBytes(float x, float y, uint8_t &outX, uint8_t &outY);

#endif
