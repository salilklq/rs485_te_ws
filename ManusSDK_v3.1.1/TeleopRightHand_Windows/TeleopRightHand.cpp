#define NOMINMAX
#define WIN32_LEAN_AND_MEAN

#include <Windows.h>

#include "ManusSDK.h"
#include "ManusSDKTypes.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{
constexpr uint16_t kRightHandSlaveAddress = 1;
constexpr uint16_t kControlRegisterStart = 0;
constexpr uint16_t kSpeedRegisterStart = 6;
constexpr uint16_t kForceRegisterStart = 12;
constexpr size_t kCommandRegisterCount = 6;

std::atomic<bool> g_Running{ true };

BOOL WINAPI ConsoleHandler(DWORD type)
{
    switch (type)
    {
    case CTRL_C_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_LOGOFF_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        g_Running = false;
        return TRUE;
    default:
        return FALSE;
    }
}

enum class CoreMode
{
    Local,
    Remote,
    Integrated
};

struct Options
{
    CoreMode mode = CoreMode::Local;
    bool enableWrite = false;
    std::string port;
    uint32_t baudRate = 115200;
    uint8_t slaveAddress = kRightHandSlaveAddress;
    double rateHz = 30.0;
    uint16_t deadband = 5;
    float smoothingAlpha = 0.35f;
    float fingerFlexMaxDeg = 180.0f;
    float thumbFlexMaxDeg = 120.0f;
    float thumbSpreadMaxDeg = 45.0f;
    bool useAbsoluteFlex = true;
    bool invertFlex = false;
    bool verbose = false;
    int speed = -1;
    int force = -1;
    bool relaxOnExit = false;
};

struct HandCommand
{
    std::array<uint16_t, kCommandRegisterCount> registers{};
};

struct SharedState
{
    std::mutex mutex;
    uint32_t rightGloveId = 0;
    bool hasErgonomics = false;
    ErgonomicsData latestErgonomics{};
};

SharedState g_State;

void PrintUsage()
{
    std::cout
        << "MANUS right-hand teleoperation demo\n"
        << "\n"
        << "Default is dry-run: it connects to MANUS Core and prints mapped registers only.\n"
        << "To write to the real right hand, pass --enable-write --port COMx.\n"
        << "\n"
        << "Usage:\n"
        << "  TeleopRightHand_Windows.exe [options]\n"
        << "\n"
        << "Options:\n"
        << "  --mode local|remote|integrated   MANUS Core mode. Default: local\n"
        << "  --port COMx                      RS485 serial port, required with --enable-write\n"
        << "  --enable-write                   Actually write Modbus-RTU frames to the hand\n"
        << "  --baud N                         Serial baud rate. Default: 115200\n"
        << "  --slave N                        Modbus slave address. Default: 1 for right hand\n"
        << "  --rate-hz N                      Command update rate. Default: 30\n"
        << "  --deadband N                     Minimum register delta before write. Default: 5\n"
        << "  --speed N                        Optionally write registers 6..11 at startup\n"
        << "  --force N                        Optionally write registers 12..17 at startup\n"
        << "  --relax-on-exit                  Write all position registers to 0 on exit\n"
        << "  --finger-flex-max DEG            Full flexion angle for fingers. Default: 180\n"
        << "  --thumb-flex-max DEG             Full flexion angle for thumb. Default: 120\n"
        << "  --thumb-spread-max DEG           Thumb opposition/spread full scale. Default: 45\n"
        << "  --signed-flex                    Use signed MANUS stretch angles instead of abs angles\n"
        << "  --invert-flex                    Invert signed flexion direction\n"
        << "  --smoothing A                    Low-pass alpha 0..1. Default: 0.35\n"
        << "  --verbose                        Print every sent command\n"
        << "  --help                           Show this text\n";
}

bool ParseInt(const char* value, int& out)
{
    if (value == nullptr) return false;
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (*value == '\0' || *end != '\0') return false;
    out = static_cast<int>(parsed);
    return true;
}

bool ParseDouble(const char* value, double& out)
{
    if (value == nullptr) return false;
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    if (*value == '\0' || *end != '\0') return false;
    out = parsed;
    return true;
}

bool ParseOptions(int argc, char** argv, Options& options)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        auto requireValue = [&](const char* name) -> const char*
        {
            if (i + 1 >= argc)
            {
                std::cerr << "Missing value for " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (arg == "--help" || arg == "-h")
        {
            PrintUsage();
            return false;
        }
        else if (arg == "--mode")
        {
            const char* value = requireValue("--mode");
            if (!value) return false;
            const std::string mode = value;
            if (mode == "local") options.mode = CoreMode::Local;
            else if (mode == "remote") options.mode = CoreMode::Remote;
            else if (mode == "integrated") options.mode = CoreMode::Integrated;
            else
            {
                std::cerr << "Unknown --mode: " << mode << "\n";
                return false;
            }
        }
        else if (arg == "--port")
        {
            const char* value = requireValue("--port");
            if (!value) return false;
            options.port = value;
        }
        else if (arg == "--enable-write")
        {
            options.enableWrite = true;
        }
        else if (arg == "--baud")
        {
            int parsed = 0;
            if (!ParseInt(requireValue("--baud"), parsed) || parsed <= 0)
            {
                std::cerr << "Invalid --baud\n";
                return false;
            }
            options.baudRate = static_cast<uint32_t>(parsed);
        }
        else if (arg == "--slave")
        {
            int parsed = 0;
            if (!ParseInt(requireValue("--slave"), parsed) || parsed <= 0 || parsed > 247)
            {
                std::cerr << "Invalid --slave\n";
                return false;
            }
            options.slaveAddress = static_cast<uint8_t>(parsed);
        }
        else if (arg == "--rate-hz")
        {
            if (!ParseDouble(requireValue("--rate-hz"), options.rateHz) || options.rateHz <= 0.0)
            {
                std::cerr << "Invalid --rate-hz\n";
                return false;
            }
        }
        else if (arg == "--deadband")
        {
            int parsed = 0;
            if (!ParseInt(requireValue("--deadband"), parsed) || parsed < 0 || parsed > 1000)
            {
                std::cerr << "Invalid --deadband\n";
                return false;
            }
            options.deadband = static_cast<uint16_t>(parsed);
        }
        else if (arg == "--speed")
        {
            if (!ParseInt(requireValue("--speed"), options.speed) || options.speed < 0 || options.speed > 1000)
            {
                std::cerr << "Invalid --speed\n";
                return false;
            }
        }
        else if (arg == "--force")
        {
            if (!ParseInt(requireValue("--force"), options.force) || options.force < 0 || options.force > 1000)
            {
                std::cerr << "Invalid --force\n";
                return false;
            }
        }
        else if (arg == "--relax-on-exit")
        {
            options.relaxOnExit = true;
        }
        else if (arg == "--finger-flex-max")
        {
            double parsed = 0.0;
            if (!ParseDouble(requireValue("--finger-flex-max"), parsed) || parsed <= 0.0)
            {
                std::cerr << "Invalid --finger-flex-max\n";
                return false;
            }
            options.fingerFlexMaxDeg = static_cast<float>(parsed);
        }
        else if (arg == "--thumb-flex-max")
        {
            double parsed = 0.0;
            if (!ParseDouble(requireValue("--thumb-flex-max"), parsed) || parsed <= 0.0)
            {
                std::cerr << "Invalid --thumb-flex-max\n";
                return false;
            }
            options.thumbFlexMaxDeg = static_cast<float>(parsed);
        }
        else if (arg == "--thumb-spread-max")
        {
            double parsed = 0.0;
            if (!ParseDouble(requireValue("--thumb-spread-max"), parsed) || parsed <= 0.0)
            {
                std::cerr << "Invalid --thumb-spread-max\n";
                return false;
            }
            options.thumbSpreadMaxDeg = static_cast<float>(parsed);
        }
        else if (arg == "--signed-flex")
        {
            options.useAbsoluteFlex = false;
        }
        else if (arg == "--invert-flex")
        {
            options.invertFlex = true;
        }
        else if (arg == "--smoothing")
        {
            double parsed = 0.0;
            if (!ParseDouble(requireValue("--smoothing"), parsed) || parsed < 0.0 || parsed > 1.0)
            {
                std::cerr << "Invalid --smoothing\n";
                return false;
            }
            options.smoothingAlpha = static_cast<float>(parsed);
        }
        else if (arg == "--verbose")
        {
            options.verbose = true;
        }
        else
        {
            std::cerr << "Unknown option: " << arg << "\n";
            PrintUsage();
            return false;
        }
    }

    if (options.enableWrite && options.port.empty())
    {
        std::cerr << "--port COMx is required when --enable-write is set.\n";
        return false;
    }

    return true;
}

bool IsHelpRequested(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h")
        {
            return true;
        }
    }
    return false;
}

std::wstring WidenAscii(const std::string& text)
{
    return std::wstring(text.begin(), text.end());
}

class SerialPort
{
public:
    ~SerialPort()
    {
        Close();
    }

    bool Open(const std::string& port, uint32_t baudRate)
    {
        Close();

        std::string devicePath = port;
        if (devicePath.rfind("\\\\.\\", 0) != 0)
        {
            devicePath = "\\\\.\\" + devicePath;
        }

        m_Handle = CreateFileW(
            WidenAscii(devicePath).c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            nullptr,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            nullptr);

        if (m_Handle == INVALID_HANDLE_VALUE)
        {
            std::cerr << "Failed to open serial port " << port << ". Win32 error " << GetLastError() << "\n";
            return false;
        }

        DCB dcb{};
        dcb.DCBlength = sizeof(DCB);
        if (!GetCommState(m_Handle, &dcb))
        {
            std::cerr << "GetCommState failed. Win32 error " << GetLastError() << "\n";
            Close();
            return false;
        }

        dcb.BaudRate = baudRate;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;
        dcb.fRtsControl = RTS_CONTROL_ENABLE;

        if (!SetCommState(m_Handle, &dcb))
        {
            std::cerr << "SetCommState failed. Win32 error " << GetLastError() << "\n";
            Close();
            return false;
        }

        COMMTIMEOUTS timeouts{};
        timeouts.ReadIntervalTimeout = 20;
        timeouts.ReadTotalTimeoutConstant = 20;
        timeouts.ReadTotalTimeoutMultiplier = 2;
        timeouts.WriteTotalTimeoutConstant = 20;
        timeouts.WriteTotalTimeoutMultiplier = 2;
        if (!SetCommTimeouts(m_Handle, &timeouts))
        {
            std::cerr << "SetCommTimeouts failed. Win32 error " << GetLastError() << "\n";
            Close();
            return false;
        }

        PurgeComm(m_Handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
        return true;
    }

    void Close()
    {
        if (m_Handle != INVALID_HANDLE_VALUE)
        {
            CloseHandle(m_Handle);
            m_Handle = INVALID_HANDLE_VALUE;
        }
    }

    bool WriteBytes(const std::vector<uint8_t>& bytes)
    {
        if (m_Handle == INVALID_HANDLE_VALUE)
        {
            return false;
        }

        DWORD written = 0;
        if (!WriteFile(m_Handle, bytes.data(), static_cast<DWORD>(bytes.size()), &written, nullptr))
        {
            std::cerr << "Serial WriteFile failed. Win32 error " << GetLastError() << "\n";
            return false;
        }

        if (written != bytes.size())
        {
            std::cerr << "Serial short write: " << written << " / " << bytes.size() << "\n";
            return false;
        }

        return true;
    }

private:
    HANDLE m_Handle = INVALID_HANDLE_VALUE;
};

uint16_t ModbusCrc16(const std::vector<uint8_t>& bytes)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t b : bytes)
    {
        crc ^= b;
        for (int i = 0; i < 8; ++i)
        {
            if (crc & 0x0001)
            {
                crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

std::vector<uint8_t> BuildWriteMultipleRegistersFrame(
    uint8_t slave,
    uint16_t startRegister,
    const std::array<uint16_t, kCommandRegisterCount>& values)
{
    std::vector<uint8_t> frame;
    frame.reserve(9 + values.size() * 2);
    frame.push_back(slave);
    frame.push_back(0x10);
    frame.push_back(static_cast<uint8_t>((startRegister >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(startRegister & 0xFF));
    frame.push_back(0x00);
    frame.push_back(static_cast<uint8_t>(values.size()));
    frame.push_back(static_cast<uint8_t>(values.size() * 2));

    for (uint16_t value : values)
    {
        frame.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
        frame.push_back(static_cast<uint8_t>(value & 0xFF));
    }

    const uint16_t crc = ModbusCrc16(frame);
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    return frame;
}

std::array<uint16_t, kCommandRegisterCount> MakeFilledRegisters(uint16_t value)
{
    std::array<uint16_t, kCommandRegisterCount> values{};
    values.fill(value);
    return values;
}

uint16_t ClampRegister(float value)
{
    if (!std::isfinite(value))
    {
        return 0;
    }
    const float clamped = std::clamp(value, 0.0f, 1000.0f);
    return static_cast<uint16_t>(std::lround(clamped));
}

float FingerFlexionDegrees(float mcp, float pip, float dip, const Options& options)
{
    if (options.useAbsoluteFlex)
    {
        return std::fabs(mcp) + std::fabs(pip) + std::fabs(dip);
    }

    float sum = mcp + pip + dip;
    if (options.invertFlex)
    {
        sum = -sum;
    }
    return std::max(0.0f, sum);
}

uint16_t FlexToRegister(float flexDeg, float maxDeg)
{
    if (maxDeg <= 0.0f)
    {
        return 0;
    }
    return ClampRegister((flexDeg / maxDeg) * 1000.0f);
}

uint16_t SpreadToRegister(float spreadDeg, const Options& options)
{
    return FlexToRegister(std::fabs(spreadDeg), options.thumbSpreadMaxDeg);
}

HandCommand MapErgonomicsToRightHandCommand(const ErgonomicsData& data, const Options& options)
{
    HandCommand command{};

    constexpr int kRightThumbBase = ErgonomicsDataType_RightFingerThumbMCPSpread;
    constexpr int kRightIndexBase = ErgonomicsDataType_RightFingerIndexMCPSpread;
    constexpr int kRightMiddleBase = ErgonomicsDataType_RightFingerMiddleMCPSpread;
    constexpr int kRightRingBase = ErgonomicsDataType_RightFingerRingMCPSpread;
    constexpr int kRightPinkyBase = ErgonomicsDataType_RightFingerPinkyMCPSpread;

    auto fingerValue = [&](int base, float maxDeg) -> uint16_t
    {
        const float mcp = data.data[base + 1];
        const float pip = data.data[base + 2];
        const float dip = data.data[base + 3];
        return FlexToRegister(FingerFlexionDegrees(mcp, pip, dip, options), maxDeg);
    };

    command.registers[0] = SpreadToRegister(data.data[kRightThumbBase], options);
    command.registers[1] = fingerValue(kRightThumbBase, options.thumbFlexMaxDeg);
    command.registers[2] = fingerValue(kRightIndexBase, options.fingerFlexMaxDeg);
    command.registers[3] = fingerValue(kRightMiddleBase, options.fingerFlexMaxDeg);
    command.registers[4] = fingerValue(kRightRingBase, options.fingerFlexMaxDeg);
    command.registers[5] = fingerValue(kRightPinkyBase, options.fingerFlexMaxDeg);

    return command;
}

HandCommand SmoothCommand(const HandCommand& current, const HandCommand& previous, bool hasPrevious, float alpha)
{
    if (!hasPrevious || alpha >= 1.0f)
    {
        return current;
    }

    if (alpha <= 0.0f)
    {
        return previous;
    }

    HandCommand smoothed{};
    for (size_t i = 0; i < smoothed.registers.size(); ++i)
    {
        const float value =
            previous.registers[i] * (1.0f - alpha) +
            current.registers[i] * alpha;
        smoothed.registers[i] = ClampRegister(value);
    }
    return smoothed;
}

bool CommandChangedEnough(const HandCommand& a, const HandCommand& b, uint16_t deadband)
{
    for (size_t i = 0; i < a.registers.size(); ++i)
    {
        const int delta = std::abs(static_cast<int>(a.registers[i]) - static_cast<int>(b.registers[i]));
        if (delta >= static_cast<int>(deadband))
        {
            return true;
        }
    }
    return false;
}

std::string CommandToString(const HandCommand& command)
{
    std::ostringstream stream;
    stream << "flip=" << std::setw(4) << command.registers[0]
           << " thumb=" << std::setw(4) << command.registers[1]
           << " index=" << std::setw(4) << command.registers[2]
           << " middle=" << std::setw(4) << command.registers[3]
           << " ring=" << std::setw(4) << command.registers[4]
           << " pinky=" << std::setw(4) << command.registers[5];
    return stream.str();
}

void OnLandscapeCallback(const Landscape* const landscape)
{
    if (!landscape)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(g_State.mutex);
    g_State.rightGloveId = 0;
    for (uint32_t i = 0; i < landscape->gloveDevices.gloveCount; ++i)
    {
        const GloveLandscapeData& glove = landscape->gloveDevices.gloves[i];
        if (glove.side == Side_Right && !glove.excluded)
        {
            g_State.rightGloveId = glove.id;
            break;
        }
    }
}

void OnErgonomicsCallback(const ErgonomicsStream* const ergonomics)
{
    if (!ergonomics)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(g_State.mutex);
    for (uint32_t i = 0; i < ergonomics->dataCount; ++i)
    {
        const ErgonomicsData& item = ergonomics->data[i];
        if (item.isUserID)
        {
            continue;
        }

        if (g_State.rightGloveId != 0 && item.id != g_State.rightGloveId)
        {
            continue;
        }

        g_State.latestErgonomics = item;
        g_State.hasErgonomics = true;
        return;
    }
}

bool InitializeManusSdk(CoreMode mode)
{
    SDKReturnCode initResult = SDKReturnCode_Error;
    if (mode == CoreMode::Integrated)
    {
        initResult = CoreSdk_InitializeIntegrated();
    }
    else
    {
        initResult = CoreSdk_InitializeCore();
    }

    if (initResult != SDKReturnCode_Success)
    {
        std::cerr << "CoreSdk initialization failed: " << static_cast<int>(initResult) << "\n";
        return false;
    }

    CoordinateSystemVUH coordinateSystem{};
    coordinateSystem.handedness = Side_Right;
    coordinateSystem.up = AxisPolarity_PositiveZ;
    coordinateSystem.view = AxisView_XFromViewer;
    coordinateSystem.unitScale = 1.0f;
    const SDKReturnCode coordResult = CoreSdk_InitializeCoordinateSystemWithVUH(coordinateSystem, true);
    if (coordResult != SDKReturnCode_Success)
    {
        std::cerr << "CoreSdk coordinate initialization failed: " << static_cast<int>(coordResult) << "\n";
        return false;
    }

    const SDKReturnCode landscapeResult = CoreSdk_RegisterCallbackForLandscapeStream(OnLandscapeCallback);
    if (landscapeResult != SDKReturnCode_Success)
    {
        std::cerr << "Register landscape callback failed: " << static_cast<int>(landscapeResult) << "\n";
        return false;
    }

    const SDKReturnCode ergoResult = CoreSdk_RegisterCallbackForErgonomicsStream(OnErgonomicsCallback);
    if (ergoResult != SDKReturnCode_Success)
    {
        std::cerr << "Register ergonomics callback failed: " << static_cast<int>(ergoResult) << "\n";
        return false;
    }

    return true;
}

bool ConnectToManusCore(CoreMode mode)
{
    const bool connectLocal = mode == CoreMode::Local;
    while (g_Running)
    {
        const SDKReturnCode lookResult = CoreSdk_LookForHosts(1, connectLocal);
        if (lookResult != SDKReturnCode_Success)
        {
            std::cout << "Looking for MANUS Core failed, retrying...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        uint32_t hostCount = 0;
        const SDKReturnCode countResult = CoreSdk_GetNumberOfAvailableHostsFound(&hostCount);
        if (countResult != SDKReturnCode_Success || hostCount == 0)
        {
            std::cout << "No MANUS Core host found, retrying...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        std::unique_ptr<ManusHost[]> hosts(new ManusHost[hostCount]);
        const SDKReturnCode hostsResult = CoreSdk_GetAvailableHostsFound(hosts.get(), hostCount);
        if (hostsResult != SDKReturnCode_Success)
        {
            std::cout << "Could not read MANUS Core host list, retrying...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        const SDKReturnCode connectResult = CoreSdk_ConnectToHost(hosts[0]);
        if (connectResult == SDKReturnCode_Success)
        {
            std::cout << "Connected to MANUS Core host " << hosts[0].hostName
                      << " at " << hosts[0].ipAddress << "\n";
            return true;
        }

        std::cout << "Could not connect to MANUS Core, retrying...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return false;
}

bool WriteRegisterBlock(
    SerialPort& serial,
    const Options& options,
    uint16_t startRegister,
    const std::array<uint16_t, kCommandRegisterCount>& values)
{
    const std::vector<uint8_t> frame =
        BuildWriteMultipleRegistersFrame(options.slaveAddress, startRegister, values);
    return serial.WriteBytes(frame);
}

bool MaybeWriteStartupRegisterBlock(SerialPort& serial, const Options& options)
{
    if (options.speed >= 0)
    {
        if (!WriteRegisterBlock(serial, options, kSpeedRegisterStart, MakeFilledRegisters(static_cast<uint16_t>(options.speed))))
        {
            std::cerr << "Failed to write speed registers.\n";
            return false;
        }
        std::cout << "Wrote speed registers 6..11 = " << options.speed << "\n";
    }

    if (options.force >= 0)
    {
        if (!WriteRegisterBlock(serial, options, kForceRegisterStart, MakeFilledRegisters(static_cast<uint16_t>(options.force))))
        {
            std::cerr << "Failed to write force registers.\n";
            return false;
        }
        std::cout << "Wrote force registers 12..17 = " << options.force << "\n";
    }

    return true;
}

} // namespace

int main(int argc, char** argv)
{
    SetConsoleCtrlHandler(ConsoleHandler, TRUE);

    if (IsHelpRequested(argc, argv))
    {
        PrintUsage();
        return 0;
    }

    Options options;
    if (!ParseOptions(argc, argv, options))
    {
        return 1;
    }

    std::cout << "MANUS right-hand teleoperation demo\n";
    if (!options.enableWrite)
    {
        std::cout << "DRY-RUN mode: no RS485 writes. Add --enable-write --port COMx to drive hardware.\n";
    }
    else
    {
        std::cout << "WRITE mode: RS485 " << options.port << " @ " << options.baudRate
                  << ", slave " << static_cast<int>(options.slaveAddress) << "\n";
    }
    std::cout << "Press Space or Ctrl+C to stop.\n";

    SerialPort serial;
    if (options.enableWrite)
    {
        if (!serial.Open(options.port, options.baudRate))
        {
            return 2;
        }
        if (!MaybeWriteStartupRegisterBlock(serial, options))
        {
            return 2;
        }
    }

    if (!InitializeManusSdk(options.mode))
    {
        return 3;
    }

    if (!ConnectToManusCore(options.mode))
    {
        CoreSdk_ShutDown();
        return 4;
    }

    HandCommand previousCommand{};
    bool hasPrevious = false;
    auto nextTick = std::chrono::steady_clock::now();
    auto lastStatus = std::chrono::steady_clock::time_point{};

    while (g_Running)
    {
        if (GetAsyncKeyState(VK_SPACE) & 0x8000)
        {
            g_Running = false;
            break;
        }

        uint32_t rightGloveId = 0;
        bool hasErgo = false;
        ErgonomicsData ergonomics{};
        {
            std::lock_guard<std::mutex> lock(g_State.mutex);
            rightGloveId = g_State.rightGloveId;
            hasErgo = g_State.hasErgonomics;
            ergonomics = g_State.latestErgonomics;
        }

        const auto now = std::chrono::steady_clock::now();
        if (rightGloveId == 0)
        {
            if (now - lastStatus > std::chrono::seconds(2))
            {
                std::cout << "Waiting for a right-hand glove in MANUS Core...\n";
                lastStatus = now;
            }
        }
        else if (!hasErgo)
        {
            if (now - lastStatus > std::chrono::seconds(2))
            {
                std::cout << "Right glove 0x" << std::hex << rightGloveId << std::dec
                          << " found. Waiting for ergonomics data...\n";
                lastStatus = now;
            }
        }
        else
        {
            HandCommand command = MapErgonomicsToRightHandCommand(ergonomics, options);
            command = SmoothCommand(command, previousCommand, hasPrevious, options.smoothingAlpha);

            const bool shouldWrite = !hasPrevious || CommandChangedEnough(command, previousCommand, options.deadband);
            if (shouldWrite)
            {
                if (options.enableWrite)
                {
                    if (!WriteRegisterBlock(serial, options, kControlRegisterStart, command.registers))
                    {
                        std::cerr << "Failed to write position registers 0..5. Stopping.\n";
                        g_Running = false;
                        break;
                    }
                }

                if (options.verbose || now - lastStatus > std::chrono::milliseconds(500))
                {
                    std::cout << (options.enableWrite ? "WRITE " : "DRY   ")
                              << "glove=0x" << std::hex << rightGloveId << std::dec
                              << " " << CommandToString(command) << "\n";
                    lastStatus = now;
                }

                previousCommand = command;
                hasPrevious = true;
            }
        }

        nextTick += std::chrono::microseconds(static_cast<int64_t>(1000000.0 / options.rateHz));
        std::this_thread::sleep_until(nextTick);
        if (std::chrono::steady_clock::now() > nextTick + std::chrono::seconds(1))
        {
            nextTick = std::chrono::steady_clock::now();
        }
    }

    if (options.enableWrite && options.relaxOnExit)
    {
        HandCommand relax{};
        WriteRegisterBlock(serial, options, kControlRegisterStart, relax.registers);
        std::cout << "Wrote relax command: all position registers = 0\n";
    }

    CoreSdk_ShutDown();
    std::cout << "Stopped.\n";
    return 0;
}
