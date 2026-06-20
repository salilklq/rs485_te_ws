// ManusKeypointStreamer
// -----------------------------------------------------------------------------
// Connects to MANUS Core, subscribes to the RAW skeleton stream (the human-hand
// skeleton from the estimation system, before any robot retargeting), assembles
// 21 MANO-ordered 3D keypoints for the left and right gloves, and streams them
// over UDP to the Python dex-retargeting teleop service.
//
// This replaces the old C++ angle->register mapping: all mapping now happens in
// Python (teleop/). This process is a thin, robust data source only.
//
// UDP packet (little-endian), one per hand per frame -- must match
// teleop/dexhand_teleop/protocol.py:
//   char[4]  'MNKP'
//   uint8    version = 1
//   uint8    side    (0 = left, 1 = right)
//   uint8    valid
//   uint8    npts = 21
//   uint32   seq
//   uint64   stamp_us
//   float32[21*3] points  (x,y,z) meters, MANUS frame, MANO order
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN

#include <WinSock2.h>
#include <WS2tcpip.h>
#include <Windows.h>

#include "ManusSDK.h"
#include "ManusSDKTypes.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#pragma comment(lib, "Ws2_32.lib")

namespace
{
constexpr int kNumPoints = 21;

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

enum class CoreMode { Local, Remote, Integrated };

struct Options
{
    CoreMode mode = CoreMode::Local;
    std::string udpHost = "127.0.0.1";
    uint16_t udpPort = 9001;
    double rateHz = 90.0;
    bool verbose = false;
};

// ---- Shared MANUS state ----------------------------------------------------
struct GloveSkeleton
{
    bool hasData = false;
    std::vector<SkeletonNode> nodes;
};

struct SharedState
{
    std::mutex mutex;
    uint32_t leftGloveId = 0;
    uint32_t rightGloveId = 0;
    std::map<uint32_t, GloveSkeleton> skeletons; // by gloveId
};

SharedState g_State;

// MANO index for a (chain, joint). Returns -1 if this node is not a MANO point.
int ManoIndexFor(ChainType chain, FingerJointType joint)
{
    auto fingerBase = [](ChainType c) -> int {
        switch (c)
        {
        case ChainType_FingerIndex:  return 5;
        case ChainType_FingerMiddle: return 9;
        case ChainType_FingerRing:   return 13;
        case ChainType_FingerPinky:  return 17;
        default: return -1;
        }
    };

    if (chain == ChainType_Hand)
    {
        return 0; // wrist
    }
    if (chain == ChainType_FingerThumb)
    {
        // thumb has no Distal: CMC, MCP, IP, TIP
        switch (joint)
        {
        case FingerJointType_Metacarpal:   return 1;
        case FingerJointType_Proximal:     return 2;
        case FingerJointType_Intermediate: return 3;
        case FingerJointType_Tip:          return 4;
        default: return -1;
        }
    }

    const int base = fingerBase(chain);
    if (base < 0)
    {
        return -1;
    }
    // finger MANO points: MCP, PIP, DIP, TIP -> Proximal, Intermediate, Distal, Tip
    switch (joint)
    {
    case FingerJointType_Proximal:     return base + 0;
    case FingerJointType_Intermediate: return base + 1;
    case FingerJointType_Distal:       return base + 2;
    case FingerJointType_Tip:          return base + 3;
    default: return -1; // Metacarpal of fingers is dropped
    }
}

// Cache of nodeId -> MANO index for each glove (built once per glove).
struct GloveNodeMap
{
    bool built = false;
    std::map<uint32_t, int> nodeIdToMano; // nodeId -> 0..20
    int wristNodeId = -1;
};

std::map<uint32_t, GloveNodeMap> g_NodeMaps;
std::mutex g_NodeMapMutex;

bool BuildNodeMap(uint32_t gloveId, GloveNodeMap& out)
{
    uint32_t count = 0;
    if (CoreSdk_GetRawSkeletonNodeCount(gloveId, count) != SDKReturnCode_Success || count == 0)
    {
        return false;
    }
    std::vector<NodeInfo> infos(count);
    if (CoreSdk_GetRawSkeletonNodeInfoArray(gloveId, infos.data(), count) != SDKReturnCode_Success)
    {
        return false;
    }

    out.nodeIdToMano.clear();
    out.wristNodeId = -1;
    for (const NodeInfo& info : infos)
    {
        const int mano = ManoIndexFor(info.chainType, info.fingerJointType);
        if (mano >= 0)
        {
            // first writer wins; wrist (0) may have several candidates -> keep first
            out.nodeIdToMano.emplace(info.nodeId, mano);
            if (mano == 0 && out.wristNodeId < 0)
            {
                out.wristNodeId = static_cast<int>(info.nodeId);
            }
        }
    }
    out.built = true;
    return !out.nodeIdToMano.empty();
}

// ---- Callbacks -------------------------------------------------------------
void OnLandscapeCallback(const Landscape* const landscape)
{
    if (!landscape)
    {
        return;
    }
    std::lock_guard<std::mutex> lock(g_State.mutex);
    g_State.leftGloveId = 0;
    g_State.rightGloveId = 0;
    for (uint32_t i = 0; i < landscape->gloveDevices.gloveCount; ++i)
    {
        const GloveLandscapeData& glove = landscape->gloveDevices.gloves[i];
        if (glove.excluded)
        {
            continue;
        }
        if (glove.side == Side_Right && g_State.rightGloveId == 0)
        {
            g_State.rightGloveId = glove.id;
        }
        else if (glove.side == Side_Left && g_State.leftGloveId == 0)
        {
            g_State.leftGloveId = glove.id;
        }
    }
}

void OnRawSkeletonStreamCallback(const SkeletonStreamInfo* const info)
{
    if (!info)
    {
        return;
    }
    std::lock_guard<std::mutex> lock(g_State.mutex);
    for (uint32_t i = 0; i < info->skeletonsCount; ++i)
    {
        RawSkeletonInfo skelInfo{};
        if (CoreSdk_GetRawSkeletonInfo(i, &skelInfo) != SDKReturnCode_Success)
        {
            continue;
        }
        GloveSkeleton& gs = g_State.skeletons[skelInfo.gloveId];
        gs.nodes.resize(skelInfo.nodesCount);
        if (CoreSdk_GetRawSkeletonData(i, gs.nodes.data(), skelInfo.nodesCount) == SDKReturnCode_Success)
        {
            gs.hasData = true;
        }
    }
}

// ---- SDK setup -------------------------------------------------------------
bool InitializeManusSdk(CoreMode mode)
{
    const SDKReturnCode initResult = (mode == CoreMode::Integrated)
        ? CoreSdk_InitializeIntegrated()
        : CoreSdk_InitializeCore();
    if (initResult != SDKReturnCode_Success)
    {
        std::cerr << "CoreSdk init failed: " << static_cast<int>(initResult) << "\n";
        return false;
    }

    CoordinateSystemVUH coord{};
    coord.handedness = Side_Right;
    coord.up = AxisPolarity_PositiveZ;
    coord.view = AxisView_XFromViewer;
    coord.unitScale = 1.0f; // meters
    if (CoreSdk_InitializeCoordinateSystemWithVUH(coord, true) != SDKReturnCode_Success)
    {
        std::cerr << "Coordinate init failed\n";
        return false;
    }

    if (CoreSdk_RegisterCallbackForLandscapeStream(OnLandscapeCallback) != SDKReturnCode_Success)
    {
        std::cerr << "Register landscape callback failed\n";
        return false;
    }
    if (CoreSdk_RegisterCallbackForRawSkeletonStream(OnRawSkeletonStreamCallback) != SDKReturnCode_Success)
    {
        std::cerr << "Register raw skeleton callback failed\n";
        return false;
    }
    return true;
}

bool ConnectToManusCore(CoreMode mode)
{
    const bool connectLocal = (mode == CoreMode::Local);
    while (g_Running)
    {
        if (CoreSdk_LookForHosts(1, connectLocal) != SDKReturnCode_Success)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        uint32_t hostCount = 0;
        if (CoreSdk_GetNumberOfAvailableHostsFound(&hostCount) != SDKReturnCode_Success || hostCount == 0)
        {
            std::cout << "No MANUS Core host found, retrying...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        std::vector<ManusHost> hosts(hostCount);
        if (CoreSdk_GetAvailableHostsFound(hosts.data(), hostCount) != SDKReturnCode_Success)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        if (CoreSdk_ConnectToHost(hosts[0]) == SDKReturnCode_Success)
        {
            std::cout << "Connected to MANUS Core " << hosts[0].hostName
                      << " @ " << hosts[0].ipAddress << "\n";
            return true;
        }
        std::cout << "Connect failed, retrying...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return false;
}

// ---- UDP -------------------------------------------------------------------
class UdpSender
{
public:
    bool Open(const std::string& host, uint16_t port)
    {
        WSADATA wsa{};
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
        {
            return false;
        }
        m_Socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (m_Socket == INVALID_SOCKET)
        {
            return false;
        }
        m_Addr.sin_family = AF_INET;
        m_Addr.sin_port = htons(port);
        inet_pton(AF_INET, host.c_str(), &m_Addr.sin_addr);
        return true;
    }

    void Send(const std::vector<uint8_t>& bytes)
    {
        if (m_Socket != INVALID_SOCKET)
        {
            sendto(m_Socket, reinterpret_cast<const char*>(bytes.data()),
                   static_cast<int>(bytes.size()), 0,
                   reinterpret_cast<sockaddr*>(&m_Addr), sizeof(m_Addr));
        }
    }

    ~UdpSender()
    {
        if (m_Socket != INVALID_SOCKET)
        {
            closesocket(m_Socket);
        }
        WSACleanup();
    }

private:
    SOCKET m_Socket = INVALID_SOCKET;
    sockaddr_in m_Addr{};
};

void AppendU32(std::vector<uint8_t>& b, uint32_t v)
{
    for (int i = 0; i < 4; ++i) b.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
}
void AppendU64(std::vector<uint8_t>& b, uint64_t v)
{
    for (int i = 0; i < 8; ++i) b.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
}
void AppendF32(std::vector<uint8_t>& b, float f)
{
    uint32_t v;
    std::memcpy(&v, &f, 4);
    AppendU32(b, v);
}

std::vector<uint8_t> BuildPacket(uint8_t side, bool valid, uint32_t seq, uint64_t stampUs,
                                 const std::array<std::array<float, 3>, kNumPoints>& pts)
{
    std::vector<uint8_t> b;
    b.reserve(24 + kNumPoints * 12);
    b.push_back('M'); b.push_back('N'); b.push_back('K'); b.push_back('P');
    b.push_back(1);                 // version
    b.push_back(side);
    b.push_back(valid ? 1 : 0);
    b.push_back(static_cast<uint8_t>(kNumPoints));
    AppendU32(b, seq);
    AppendU64(b, stampUs);
    for (const auto& p : pts)
    {
        AppendF32(b, p[0]); AppendF32(b, p[1]); AppendF32(b, p[2]);
    }
    return b;
}

// Assemble 21 MANO keypoints for a glove. Returns count of points filled.
int AssembleKeypoints(uint32_t gloveId, const GloveSkeleton& gs,
                      std::array<std::array<float, 3>, kNumPoints>& out,
                      std::array<bool, kNumPoints>& filled)
{
    filled.fill(false);
    out.fill({ 0.0f, 0.0f, 0.0f });

    GloveNodeMap nodeMap;
    {
        std::lock_guard<std::mutex> lock(g_NodeMapMutex);
        auto it = g_NodeMaps.find(gloveId);
        if (it == g_NodeMaps.end() || !it->second.built)
        {
            GloveNodeMap built;
            if (!BuildNodeMap(gloveId, built))
            {
                return 0;
            }
            g_NodeMaps[gloveId] = built;
            nodeMap = built;
        }
        else
        {
            nodeMap = it->second;
        }
    }

    int n = 0;
    for (const SkeletonNode& node : gs.nodes)
    {
        auto it = nodeMap.nodeIdToMano.find(node.id);
        if (it == nodeMap.nodeIdToMano.end())
        {
            continue;
        }
        const int mano = it->second;
        out[mano] = { node.transform.position.x, node.transform.position.y, node.transform.position.z };
        if (!filled[mano]) { filled[mano] = true; ++n; }
    }
    // Wrist fallback: if no Hand-chain node, use first node position.
    if (!filled[0] && !gs.nodes.empty())
    {
        const auto& p = gs.nodes[0].transform.position;
        out[0] = { p.x, p.y, p.z };
        filled[0] = true;
        ++n;
    }
    return n;
}

} // namespace

int main(int argc, char** argv)
{
    setvbuf(stdout, nullptr, _IONBF, 0);  // unbuffered logs (so output shows even on crash)
    SetConsoleCtrlHandler(ConsoleHandler, TRUE);

    Options options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) { std::cerr << "Missing value for " << name << "\n"; return nullptr; }
            return argv[++i];
        };
        if (arg == "--mode") { const char* v = next("--mode"); if (!v) return 1;
            std::string m = v; options.mode = (m == "remote") ? CoreMode::Remote : (m == "integrated") ? CoreMode::Integrated : CoreMode::Local; }
        else if (arg == "--udp-host") { const char* v = next("--udp-host"); if (!v) return 1; options.udpHost = v; }
        else if (arg == "--udp-port") { const char* v = next("--udp-port"); if (!v) return 1; options.udpPort = static_cast<uint16_t>(std::atoi(v)); }
        else if (arg == "--rate-hz") { const char* v = next("--rate-hz"); if (!v) return 1; options.rateHz = std::atof(v); }
        else if (arg == "--verbose") { options.verbose = true; }
        else if (arg == "--help" || arg == "-h") {
            std::cout << "ManusKeypointStreamer --udp-host H --udp-port P --rate-hz N [--mode local|remote|integrated] [--verbose]\n";
            return 0;
        }
    }

    std::cout << "ManusKeypointStreamer -> UDP " << options.udpHost << ":" << options.udpPort
              << " @ " << options.rateHz << " Hz\n";

    UdpSender udp;
    if (!udp.Open(options.udpHost, options.udpPort))
    {
        std::cerr << "UDP open failed\n";
        return 2;
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

    // Must be set AFTER connecting (it configures the live session). Optional.
    CoreSdk_SetRawSkeletonHandMotion(HandMotion_Auto);

    uint32_t seqLeft = 0, seqRight = 0;
    auto nextTick = std::chrono::steady_clock::now();
    auto lastStatus = std::chrono::steady_clock::time_point{};
    const auto period = std::chrono::microseconds(static_cast<int64_t>(1e6 / options.rateHz));

    while (g_Running)
    {
        uint32_t leftId, rightId;
        std::map<uint32_t, GloveSkeleton> snapshot;
        {
            std::lock_guard<std::mutex> lock(g_State.mutex);
            leftId = g_State.leftGloveId;
            rightId = g_State.rightGloveId;
            snapshot = g_State.skeletons;
        }

        const auto now = std::chrono::steady_clock::now();
        const uint64_t stampUs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count());

        struct Hand { uint32_t id; uint8_t side; uint32_t* seq; };
        Hand hands[2] = { { rightId, 1, &seqRight }, { leftId, 0, &seqLeft } };

        for (const Hand& h : hands)
        {
            if (h.id == 0)
            {
                continue;
            }
            auto it = snapshot.find(h.id);
            if (it == snapshot.end() || !it->second.hasData)
            {
                continue;
            }
            std::array<std::array<float, 3>, kNumPoints> pts;
            std::array<bool, kNumPoints> filled;
            const int n = AssembleKeypoints(h.id, it->second, pts, filled);
            const bool valid = (n >= kNumPoints - 2); // tolerate a couple missing
            udp.Send(BuildPacket(h.side, valid, (*h.seq)++, stampUs, pts));

            if (options.verbose && now - lastStatus > std::chrono::milliseconds(500))
            {
                std::cout << (h.side ? "R" : "L") << " glove 0x" << std::hex << h.id << std::dec
                          << " pts=" << n << "/" << kNumPoints << "\n";
            }
        }
        if (now - lastStatus > std::chrono::milliseconds(500))
        {
            if (leftId == 0 && rightId == 0)
            {
                std::cout << "Waiting for gloves in MANUS Core...\n";
            }
            lastStatus = now;
        }

        nextTick += period;
        std::this_thread::sleep_until(nextTick);
        if (std::chrono::steady_clock::now() > nextTick + std::chrono::seconds(1))
        {
            nextTick = std::chrono::steady_clock::now();
        }
    }

    CoreSdk_ShutDown();
    std::cout << "Stopped.\n";
    return 0;
}
