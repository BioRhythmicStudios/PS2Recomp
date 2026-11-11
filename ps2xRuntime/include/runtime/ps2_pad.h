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

// Main thread only, once per presented frame (after EndDrawing has polled input). Picks the controller readState
// uses: the first connected one that has a gamepad mapping, so an unmapped device (a wheel, a virtual joystick, an
// unknown pad) in the first slot no longer hides a working controller. Logs every connected controller when the set
// changes ("[pad]" lines). On the first call it adds mappings GLFW lacks, plus gamecontrollerdb.txt from the game
// exe's folder or the file named by PS2X_GAMEPAD_DB, if present (SDL_GameControllerDB format).
void ps2PadHostUpdate();

#endif
