/**
 * play.cpp — G1 Arm7 Joystick Playback  [最终版]
 * ================================================
 * 参考: unitree_sdk2/example/g1/high_level/g1_arm7_sdk_dds_example.cpp
 *
 * 腰部: 启动时锁在初始角，全程 KP=200/KD=2
 *
 * 操作:
 *   L1     → 端矛（正向播放）
 *   L2     → 收回（反向播放回休息位）
 *   R1+R2  → 退出
 *   Ctrl+C → 退出
 *
 * LED: 绿=休息 蓝=运动中 红(255,80,80)=端矛 熄=退出
 *
 * 网卡: 自动选择 192.168.123.x 所在接口（可用 UNITREE_IFACE 环境变量覆盖）
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

using namespace unitree::robot;
using unitree_hg::msg::dds_::LowCmd_;
using unitree_hg::msg::dds_::LowState_;

// ── 关节索引 (3-DOF 腰版本) ──────────────────────────────────────────────
static constexpr int N_ARM      = 14;
static constexpr int N_TOTAL    = 17;
static constexpr int WEIGHT_IDX = 29;

static constexpr int JOINT_IDX[N_TOTAL] = {
    15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28,
    12, 13, 14
};

// ── 控制参数 ────────────────────────────────────────────────────────────
static constexpr float KP_ARM   = 60.0f;
static constexpr float KD_ARM   = 1.5f;
static constexpr float KP_WAIST = 200.0f;
static constexpr float KD_WAIST = 2.0f;
static constexpr float ENGAGE_T = 2.0f;
static constexpr float INTERP_T = 3.0f;
static constexpr int   LOOP_US  = 2000;

// ── 遥控器按键位 ─────────────────────────────────────────────────────────
static constexpr uint16_t KEY_R1 = (1 << 0);
static constexpr uint16_t KEY_L1 = (1 << 1);
static constexpr uint16_t KEY_R2 = (1 << 4);
static constexpr uint16_t KEY_L2 = (1 << 5);

static const char* ACTION_FILE = "arm7_action.dat";

// ── 全局状态 ─────────────────────────────────────────────────────────────
static LowState_          g_state{};
static std::mutex         g_state_mtx;
static std::atomic<bool>  g_running{true};
static std::atomic<bool>  g_state_received{false};

static void on_lowstate(const void* msg) {
    std::lock_guard<std::mutex> lk(g_state_mtx);
    std::memcpy(&g_state, msg, sizeof(LowState_));
    g_state_received = true;
}

static void on_signal(int) {
    std::cout << "\n[SIGNAL] 收到退出信号...\n";
    g_running = false;
}

// ── 自动挑选机器人内网网卡 (192.168.123.0/24) ───────────────────────────
static std::string pick_robot_iface() {
    if (const char* env = std::getenv("UNITREE_IFACE")) {
        if (env[0]) { std::cout << "[NET] 使用环境变量指定网卡: " << env << "\n"; return env; }
    }
    struct ifaddrs* ifs = nullptr;
    if (getifaddrs(&ifs) == 0) {
        for (auto* p = ifs; p; p = p->ifa_next) {
            if (!p->ifa_addr || p->ifa_addr->sa_family != AF_INET) continue;
            auto* sin = reinterpret_cast<sockaddr_in*>(p->ifa_addr);
            uint32_t ip = ntohl(sin->sin_addr.s_addr);
            if ((ip & 0xFFFFFF00u) == 0xC0A87B00u) {
                std::string name = p->ifa_name ? p->ifa_name : "";
                std::cout << "[NET] 自动选中网卡: " << name
                          << " (IP " << ((ip>>24)&0xff) << "." << ((ip>>16)&0xff)
                          << "." << ((ip>>8)&0xff) << "." << (ip&0xff) << ")\n";
                freeifaddrs(ifs);
                return name;
            }
        }
        freeifaddrs(ifs);
    }
    std::cerr << "[NET][WARN] 没找到 192.168.123.x 网卡，回退到 eth0\n";
    return "eth0";
}

// ── 类型别名 ─────────────────────────────────────────────────────────────
using ArmQ   = std::array<float, N_ARM>;
using TotalQ = std::array<float, N_TOTAL>;

static TotalQ get_total_q() {
    TotalQ q{};
    std::lock_guard<std::mutex> lk(g_state_mtx);
    for (int i = 0; i < N_TOTAL; ++i)
        q[i] = g_state.motor_state()[JOINT_IDX[i]].q();
    return q;
}

static ArmQ get_arm_q() {
    ArmQ q{};
    std::lock_guard<std::mutex> lk(g_state_mtx);
    for (int i = 0; i < N_ARM; ++i)
        q[i] = g_state.motor_state()[JOINT_IDX[i]].q();
    return q;
}

static uint16_t read_keys() {
    // wireless_remote 格式: [0..1]=head, [2..3]=btn(uint16), [4..]=摇杆浮点
    uint16_t k = 0;
    std::lock_guard<std::mutex> lk(g_state_mtx);
    std::memcpy(&k, &g_state.wireless_remote()[2], 2);
    return k;
}

// R1+R2 同按 → 退出（任何时刻都能触发）
static bool check_exit_combo() {
    const uint16_t k = read_keys();
    if ((k & KEY_R1) && (k & KEY_R2)) {
        std::cout << "\n[EXIT] R1+R2 退出\n";
        g_running = false;
        return true;
    }
    return false;
}

// ── cmd 写入 ─────────────────────────────────────────────────────────────
static void apply_cmd(LowCmd_& cmd,
                      const ArmQ& arm_q,
                      const TotalQ& q_init)
{
    for (int i = 0; i < N_ARM; ++i) {
        auto& mc = cmd.motor_cmd()[JOINT_IDX[i]];
        mc.q(arm_q[i]); mc.kp(KP_ARM); mc.kd(KD_ARM); mc.dq(0.0f); mc.tau(0.0f);
    }
    for (int i = N_ARM; i < N_TOTAL; ++i) {
        auto& mc = cmd.motor_cmd()[JOINT_IDX[i]];
        mc.q(q_init[i]); mc.kp(KP_WAIST); mc.kd(KD_WAIST); mc.dq(0.0f); mc.tau(0.0f);
    }
}

static void set_weight(LowCmd_& cmd, float w) {
    cmd.motor_cmd()[WEIGHT_IDX].q(std::max(0.0f, std::min(1.0f, w)));
}

static ArmQ lerp_arm(const ArmQ& a, const ArmQ& b, float t) {
    ArmQ r{};
    for (int i = 0; i < N_ARM; ++i) r[i] = a[i] + t*(b[i]-a[i]);
    return r;
}

// ── LED ──────────────────────────────────────────────────────────────────
// 重试 3 次并打印错误码，方便排查 LED 不亮的问题
static void set_led(unitree::robot::g1::AudioClient& audio,
                    uint8_t r, uint8_t g, uint8_t b) {
    for (int attempt = 0; attempt < 3; ++attempt) {
        int32_t ret = -1;
        try { ret = audio.LedControl(r, g, b); } catch (...) { ret = -999; }
        if (ret == 0) return;
        std::cerr << "[LED] LedControl(" << (int)r << "," << (int)g << ","
                  << (int)b << ") 尝试#" << (attempt+1) << " 返回 " << ret << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// ── weight ramp ──────────────────────────────────────────────────────────
static void ramp_weight_up(
    std::shared_ptr<ChannelPublisher<LowCmd_>>& pub,
    LowCmd_& cmd, float duration,
    const ArmQ& arm_hold, const TotalQ& q_init)
{
    const int steps = static_cast<int>(duration / (LOOP_US * 1e-6f));
    for (int i = 0; i < steps && g_running; ++i) {
        set_weight(cmd, static_cast<float>(i+1)/steps);
        apply_cmd(cmd, arm_hold, q_init);
        pub->Write(cmd);
        std::this_thread::sleep_for(std::chrono::microseconds(LOOP_US));
    }
}

static void ramp_down(
    std::shared_ptr<ChannelPublisher<LowCmd_>>& pub,
    LowCmd_& cmd, float duration,
    const ArmQ& arm_hold, const TotalQ& q_init)
{
    const int steps = static_cast<int>(duration / (LOOP_US * 1e-6f));
    for (int i = 0; i < steps; ++i) {
        set_weight(cmd, 1.0f - static_cast<float>(i+1)/steps);
        apply_cmd(cmd, arm_hold, q_init);
        pub->Write(cmd);
        std::this_thread::sleep_for(std::chrono::microseconds(LOOP_US));
    }
    set_weight(cmd, 0.0f);
    apply_cmd(cmd, arm_hold, q_init);
    pub->Write(cmd);
}

// 插值运动：L1/L2 触发后依次到达下一个 waypoint；R1+R2 可随时中断
static ArmQ interp_move(
    std::shared_ptr<ChannelPublisher<LowCmd_>>& pub,
    LowCmd_& cmd,
    const ArmQ& q_from, const ArmQ& q_to,
    float duration, const TotalQ& q_init)
{
    const int steps = static_cast<int>(duration / (LOOP_US * 1e-6f));
    ArmQ q_cur = q_from;
    for (int i = 0; i < steps && g_running; ++i) {
        // 每 50 步 (~100ms) 检查一次退出组合键
        if ((i % 50) == 0 && check_exit_combo()) break;
        q_cur = lerp_arm(q_from, q_to, static_cast<float>(i+1)/steps);
        apply_cmd(cmd, q_cur, q_init);
        pub->Write(cmd);
        std::this_thread::sleep_for(std::chrono::microseconds(LOOP_US));
    }
    return q_cur;
}

// ── 加载动作文件 ──────────────────────────────────────────────────────────
static std::vector<ArmQ> load_action() {
    std::ifstream f(ACTION_FILE);
    if (!f) throw std::runtime_error(
        std::string("找不到: ") + ACTION_FILE + "，请先运行 ./teach");
    int n_wp=0, n_j=0;
    f >> n_wp >> n_j;
    if (n_j != N_ARM) throw std::runtime_error("关节数不匹配，重新 ./teach");
    std::vector<ArmQ> wps(n_wp);
    for (auto& wp : wps) for (float& v : wp) f >> v;
    if (f.fail()) throw std::runtime_error("文件读取失败");
    return wps;
}

// ════════════════════════════════════════════════════════════════════════
int main() {
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    std::vector<ArmQ> waypoints;
    try { waypoints = load_action(); }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << "\n"; return 1;
    }

    std::cout << "===========================================\n"
              << "  G1 Arm7 - 回放模式\n"
              << "  已加载 " << waypoints.size() << " 个 waypoint\n"
              << "  L1 = 端矛    L2 = 收回\n"
              << "  R1+R2 = 退出    Ctrl+C = 退出\n"
              << "===========================================\n\n";

    const std::string iface = pick_robot_iface();
    ChannelFactory::Instance()->Init(0, iface);

    auto pub = std::make_shared<ChannelPublisher<LowCmd_>>("rt/arm_sdk");
    pub->InitChannel();

    auto sub = std::make_shared<ChannelSubscriber<LowState_>>("rt/lowstate");
    sub->InitChannel(on_lowstate, 10);

    unitree::robot::g1::AudioClient audio;
    audio.SetTimeout(3.0f);
    audio.Init();
    // 抢占 LED：连续熄灭几次，覆盖掉系统守护进程可能已经设定的颜色
    for (int i = 0; i < 5; ++i) {
        set_led(audio, 0, 0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }

    std::cout << "[WAIT] 等待 lowstate 数据...\n";
    while (!g_state_received && g_running)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!g_running) return 0;

    LowCmd_ cmd{};
    for (auto& mc : cmd.motor_cmd()) {
        mc.q(0.0f); mc.dq(0.0f); mc.kp(0.0f); mc.kd(0.0f); mc.tau(0.0f);
    }

    TotalQ q_init = get_total_q();
    std::cout << "[INFO] 腰部初始角 (rad): "
              << std::fixed << std::setprecision(3)
              << q_init[14] << " " << q_init[15] << " " << q_init[16] << "\n";

    ArmQ arm_init{};
    for (int i = 0; i < N_ARM; ++i) arm_init[i] = q_init[i];

    apply_cmd(cmd, arm_init, q_init);
    set_weight(cmd, 0.0f);
    pub->Write(cmd);

    std::cout << "[1/2] 接管 (weight 0→1) ...\n";
    ramp_weight_up(pub, cmd, ENGAGE_T, arm_init, q_init);

    static constexpr uint8_t RED_R = 255, RED_G = 80, RED_B = 80;

    set_led(audio, 0, 255, 0);
    std::cout << "[2/2] 接管完成 | 绿灯 | 按 L1 端矛, L2 收回\n\n";

    ArmQ  q_rest      = arm_init;
    ArmQ  q_current   = q_rest;
    bool  is_up       = false;
    bool  prev_l1     = false;
    bool  prev_l2     = false;
    int   led_refresh = 0;

    while (g_running) {
        const uint16_t keys = read_keys();

        // R1+R2 同按 → 退出（最高优先级）
        if ((keys & KEY_R1) && (keys & KEY_R2)) {
            std::cout << "[EXIT] R1+R2 退出\n";
            g_running = false;
            break;
        }

        const bool l1_cur  = (keys & KEY_L1) != 0;
        const bool l2_cur  = (keys & KEY_L2) != 0;
        const bool l1_rise = l1_cur && !prev_l1;
        const bool l2_rise = l2_cur && !prev_l2;
        prev_l1 = l1_cur;
        prev_l2 = l2_cur;

        if (l1_rise && !is_up) {
            std::cout << "[ACTION] 端矛 (L1) | 蓝灯...\n";
            set_led(audio, 0, 0, 255);

            ArmQ q_from = get_arm_q();
            for (size_t i = 0; i < waypoints.size() && g_running; ++i) {
                std::cout << "  -> wp" << i+1 << "/" << waypoints.size() << "\n";
                q_from = interp_move(pub, cmd, q_from, waypoints[i], INTERP_T, q_init);
            }
            if (!g_running) break;

            q_current = waypoints.back();
            is_up = true;

            for (int i = 0; i < 250 && g_running; ++i) {
                apply_cmd(cmd, q_current, q_init);
                pub->Write(cmd);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            { const uint16_t k = read_keys();
              prev_l1 = (k & KEY_L1) != 0;
              prev_l2 = (k & KEY_L2) != 0; }

            set_led(audio, RED_R, RED_G, RED_B);
            std::cout << "[ACTION] 端矛保持 | 红灯 | 按 L2 收回\n\n";

        } else if (l2_rise && is_up) {
            std::cout << "[ACTION] 收回 (L2) | 蓝灯...\n";
            set_led(audio, 0, 0, 255);

            ArmQ q_from = get_arm_q();
            for (int i=(int)waypoints.size()-2; i>=0 && g_running; --i) {
                std::cout << "  <- wp" << i+1 << "/" << waypoints.size() << "\n";
                q_from = interp_move(pub, cmd, q_from, waypoints[i], INTERP_T, q_init);
            }
            if (g_running) {
                std::cout << "  <- 休息位\n";
                interp_move(pub, cmd, q_from, q_rest, INTERP_T, q_init);
            }
            if (!g_running) break;

            q_current = q_rest;
            is_up = false;

            for (int i = 0; i < 250 && g_running; ++i) {
                apply_cmd(cmd, q_current, q_init);
                pub->Write(cmd);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            { const uint16_t k = read_keys();
              prev_l1 = (k & KEY_L1) != 0;
              prev_l2 = (k & KEY_L2) != 0; }

            set_led(audio, 0, 255, 0);
            std::cout << "[ACTION] 收回完成 | 绿灯 | 按 L1 端矛\n\n";

        } else {
            apply_cmd(cmd, q_current, q_init);
            pub->Write(cmd);
            // 更高频率刷 LED（~500ms 一次），抢占系统守护进程
            if (++led_refresh >= 250) {
                led_refresh = 0;
                if (is_up) set_led(audio, RED_R, RED_G, RED_B);
                else       set_led(audio, 0, 255, 0);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(LOOP_US));
        }
    }

    std::cout << "[EXIT] weight 1→0 ...\n";
    set_led(audio, 0, 0, 0);
    ArmQ q_exit = get_arm_q();
    ramp_down(pub, cmd, ENGAGE_T, q_exit, q_init);

    std::cout << "[EXIT] 完成\n";
    return 0;
}
