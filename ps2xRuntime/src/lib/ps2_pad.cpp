#include "runtime/ps2_pad.h"
#include "ps2_host_backend.h"
#include "ps2_log.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#if !defined(PLATFORM_VITA) && !defined(__ANDROID__)
#define PS2X_PAD_GLFW 1
// raylib's desktop build contains GLFW 3.4. raylib's IsGamepadAvailable() is true for any joystick, mapped or not,
// so the mapping check needs GLFW's own joystick functions (GLFW/glfw3.h).
extern "C"
{
    int glfwJoystickPresent(int jid);
    int glfwJoystickIsGamepad(int jid);
    const char *glfwGetJoystickName(int jid);
    const char *glfwGetJoystickGUID(int jid);
    int glfwUpdateGamepadMappings(const char *string);
}
#else
#define PS2X_PAD_GLFW 0
#endif

namespace
{
    constexpr uint8_t kPadAnalogMarker = 0x73;
    constexpr uint8_t kPadStickCenter = 0x80;

    constexpr uint16_t PAD_LEFT = 0x0080u;
    constexpr uint16_t PAD_DOWN = 0x0040u;
    constexpr uint16_t PAD_RIGHT = 0x0020u;
    constexpr uint16_t PAD_UP = 0x0010u;
    constexpr uint16_t PAD_START = 0x0008u;
    constexpr uint16_t PAD_R3 = 0x0004u;
    constexpr uint16_t PAD_L3 = 0x0002u;
    constexpr uint16_t PAD_SELECT = 0x0001u;
    constexpr uint16_t PAD_SQUARE = 0x8000u;
    constexpr uint16_t PAD_CROSS = 0x4000u;
    constexpr uint16_t PAD_CIRCLE = 0x2000u;
    constexpr uint16_t PAD_TRIANGLE = 0x1000u;
    constexpr uint16_t PAD_R1 = 0x0800u;
    constexpr uint16_t PAD_L1 = 0x0400u;
    constexpr uint16_t PAD_R2 = 0x0200u;
    constexpr uint16_t PAD_L2 = 0x0100u;

    constexpr int kMaxHostPads = 4;   // raylib's MAX_GAMEPADS
    std::atomic<int> g_activePad{-1}; // set on the main thread by ps2PadHostUpdate, read on the game thread

#if PS2X_PAD_GLFW
    // Mappings missing from GLFW 3.4's built-in list, in SDL_GameControllerDB format.
    // DualSense Edge (054C:0DF2): the DualSense's DirectInput layout (GLFW's "PS5 Controller" line), new product ID.
    const char *const kExtraMappings =
        "030000004c050000f20d000000000000,DualSense Edge,a:b1,b:b2,back:b8,dpdown:h0.4,dpleft:h0.8,dpright:h0.2,"
        "dpup:h0.1,guide:b12,leftshoulder:b4,leftstick:b10,lefttrigger:a3,leftx:a0,lefty:a1,misc1:b13,"
        "rightshoulder:b5,rightstick:b11,righttrigger:a4,rightx:a2,righty:a5,start:b9,x:b0,y:b3,platform:Windows,\n";

    void loadExtraMappings()
    {
        if (!glfwUpdateGamepadMappings(kExtraMappings))
            RUNTIME_LOG("[pad] built-in extra controller mappings were rejected\n");

        std::string file;
        if (const char *env = std::getenv("PS2X_GAMEPAD_DB"); env && *env)
            file = env;
        else
            file = std::string(GetApplicationDirectory()) + "gamecontrollerdb.txt"; // raylib: ends in a separator
        std::ifstream in(file, std::ios::binary);
        if (!in)
            return;
        std::ostringstream text;
        text << in.rdbuf();
        // GLFW skips lines it can't parse and other platforms' lines; it fails only on a bad call.
        const bool ok = glfwUpdateGamepadMappings(text.str().c_str()) != 0;
        RUNTIME_LOG("[pad] controller mappings from " << file << (ok ? " loaded" : " rejected") << "\n");
    }
#endif
}

void ps2PadHostUpdate()
{
#if PS2X_PAD_GLFW
    static bool mappingsLoaded = false;
    if (!mappingsLoaded)
    {
        mappingsLoaded = true;
        loadExtraMappings();
    }

    // raylib already checks presence of four slots every frame; a quarter-second rescan is enough for hot-plugging.
    static uint32_t frame = 0;
    if ((frame++ % 15u) != 0u)
        return;

    int active = -1;
    std::string summary;
    for (int jid = 0; jid < kMaxHostPads; ++jid)
    {
        if (!glfwJoystickPresent(jid))
            continue;
        const bool mapped = glfwJoystickIsGamepad(jid) == 1;
        if (mapped && active < 0)
            active = jid;
        const char *name = glfwGetJoystickName(jid);
        const char *guid = glfwGetJoystickGUID(jid);
        summary += "slot " + std::to_string(jid) + ": \"" + (name ? name : "?") + "\" " + (guid ? guid : "?") +
                   (mapped ? " (mapped); " : " (no mapping); ");
    }
    g_activePad.store(active, std::memory_order_relaxed);

    static std::string lastSummary("\x01"); // never a real summary, so the first scan is always logged
    if (summary != lastSummary)
    {
        lastSummary = summary;
        if (summary.empty())
            RUNTIME_LOG("[pad] no controller connected; keyboard only\n");
        else if (active < 0)
            RUNTIME_LOG("[pad] " << summary << "none has a mapping; keyboard only\n");
        else
            RUNTIME_LOG("[pad] " << summary << "using slot " << active << "\n");
    }
#else
    g_activePad.store(IsGamepadAvailable(0) ? 0 : -1, std::memory_order_relaxed);
#endif
}

void ps2PadStickBytes(float x, float y, uint8_t &outX, uint8_t &outY)
{
    x = std::clamp(x, -1.0f, 1.0f);
    y = std::clamp(y, -1.0f, 1.0f);
    const float larger = std::max(std::fabs(x), std::fabs(y));
    if (larger > 0.0f)
    {
        const float scale = std::min(std::hypot(x, y), 1.0f) / larger;
        x = std::clamp(x * scale, -1.0f, 1.0f);
        y = std::clamp(y * scale, -1.0f, 1.0f);
    }
    outX = static_cast<uint8_t>(std::lround((x + 1.0f) * 127.5f));
    outY = static_cast<uint8_t>(std::lround((y + 1.0f) * 127.5f));
}

bool PSPadBackend::readState(int /*port*/, int /*slot*/, uint8_t *data, size_t size)
{
    if (!data || size < 32)
        return false;

    std::memset(data, 0, 32);
    data[0] = 0x01;
    data[1] = kPadAnalogMarker;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = data[5] = data[6] = data[7] = kPadStickCenter;

    uint16_t btns = 0xFFFFu;
    // The controller picked by ps2PadHostUpdate; the keyboard below works alongside it.
    const int kGamepad = g_activePad.load(std::memory_order_relaxed);
    const bool useGamepad = kGamepad >= 0 && IsGamepadAvailable(kGamepad);
    auto clearBit = [&btns](uint16_t mask)
    { btns &= ~mask; };

    if (useGamepad)
    {
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_LEFT_FACE_UP))
            clearBit(PAD_UP);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_LEFT_FACE_DOWN))
            clearBit(PAD_DOWN);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_LEFT_FACE_LEFT))
            clearBit(PAD_LEFT);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_LEFT_FACE_RIGHT))
            clearBit(PAD_RIGHT);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_RIGHT_FACE_DOWN))
            clearBit(PAD_CROSS);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_RIGHT_FACE_RIGHT))
            clearBit(PAD_CIRCLE);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_RIGHT_FACE_LEFT))
            clearBit(PAD_SQUARE);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_RIGHT_FACE_UP))
            clearBit(PAD_TRIANGLE);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_LEFT_TRIGGER_1))
            clearBit(PAD_L1);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_RIGHT_TRIGGER_1))
            clearBit(PAD_R1);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_LEFT_TRIGGER_2))
            clearBit(PAD_L2);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_RIGHT_TRIGGER_2))
            clearBit(PAD_R2);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_MIDDLE_RIGHT))
            clearBit(PAD_START);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_MIDDLE_LEFT))
            clearBit(PAD_SELECT);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_LEFT_THUMB))
            clearBit(PAD_L3);
        if (IsGamepadButtonDown(kGamepad, GAMEPAD_BUTTON_RIGHT_THUMB))
            clearBit(PAD_R3);

        float lx = GetGamepadAxisMovement(kGamepad, GAMEPAD_AXIS_LEFT_X);
        float ly = GetGamepadAxisMovement(kGamepad, GAMEPAD_AXIS_LEFT_Y);
        float rx = GetGamepadAxisMovement(kGamepad, GAMEPAD_AXIS_RIGHT_X);
        float ry = GetGamepadAxisMovement(kGamepad, GAMEPAD_AXIS_RIGHT_Y);
        ps2PadStickBytes(lx, ly, data[6], data[7]);
        ps2PadStickBytes(rx, ry, data[4], data[5]);
    }

    // Keyboard: always read, so it keeps working with a controller connected.
    {
        if (IsKeyDown(KEY_UP) || IsKeyDown(KEY_W))
            clearBit(PAD_UP);
        if (IsKeyDown(KEY_DOWN) || IsKeyDown(KEY_S))
            clearBit(PAD_DOWN);
        if (IsKeyDown(KEY_LEFT) || IsKeyDown(KEY_A))
            clearBit(PAD_LEFT);
        if (IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D))
            clearBit(PAD_RIGHT);
        if (IsKeyDown(KEY_X) || IsKeyDown(KEY_SPACE))
            clearBit(PAD_CROSS);
        if (IsKeyDown(KEY_C) || IsKeyDown(KEY_ESCAPE))
            clearBit(PAD_CIRCLE);
        if (IsKeyDown(KEY_Z) || IsKeyDown(KEY_KP_0))
            clearBit(PAD_SQUARE);
        if (IsKeyDown(KEY_V) || IsKeyDown(KEY_KP_1))
            clearBit(PAD_TRIANGLE);
        if (IsKeyDown(KEY_Q))
            clearBit(PAD_L1);
        if (IsKeyDown(KEY_E))
            clearBit(PAD_R1);
        if (IsKeyDown(KEY_LEFT_SHIFT))
            clearBit(PAD_L2);
        if (IsKeyDown(KEY_RIGHT_SHIFT))
            clearBit(PAD_R2);
        // Alt+Enter switches the window mode (ps2_runtime.cpp), so an Enter pressed with Alt held is not Start
        // until it is released.
        static std::atomic<bool> enterIsWindowToggle{false};
        const bool enterDown = IsKeyDown(KEY_ENTER);
        if (!enterDown)
            enterIsWindowToggle.store(false, std::memory_order_relaxed);
        else if (IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT))
            enterIsWindowToggle.store(true, std::memory_order_relaxed);
        if (enterDown && !enterIsWindowToggle.load(std::memory_order_relaxed))
            clearBit(PAD_START);
        if (IsKeyDown(KEY_TAB))
            clearBit(PAD_SELECT);
    }

    data[2] = static_cast<uint8_t>(btns & 0xFF);
    data[3] = static_cast<uint8_t>(btns >> 8);
    return true;
}
