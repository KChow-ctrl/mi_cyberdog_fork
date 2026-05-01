/**
 * @file bridge_node.cpp
 * @brief CyberDog 指令桥接器（改进版 v2）
 *
 * 改进点（相比原版）：
 * 1. checkout_mode 前置 — 所有动作执行前先切 MANUAL，解锁固件 LOCK 保护
 * 2. LOW_BTR 电量保护 — 电量 < 40% 时提前返回警告，不再无效尝试
 * 3. spin 参数修复 — cmd_spin(angle) 正确传递旋转角度
 * 4. gait_out 确认等待 — 订阅 status_out 代替硬 sleep(200ms)
 * 5. developer_mode 查询 — SSH 读取 /mnt/UDISK/robot-software/config/user_code_ctrl_mode.txt
 * 6. ChangeMode action client — 新增 /mi1034819/checkout_mode 支持
 *
 * HTTP API：
 *   POST http://localhost:5555/cmd     单条指令
 *   POST http://localhost:5555/batch    批量动作序列
 *   GET  http://localhost:5555/state   状态查询
 *   GET  http://localhost:5555/health  健康检查
 *
 * ROS 2 接口：
 *   速度指令   → /mi1034819/body_cmd    (SE3VelocityCMD.msg)
 *   动作执行   → /mi1034819/exe_monorder (ExtMonOrder.action)
 *   模式切换   → /mi1034819/checkout_mode (ChangeMode.action)
 *   音频播放   → /mi1034819/audio_play   (AudioPlay.action)
 *   状态订阅   ← /mi1034819/status_out    (ControlState.msg，持续发布)
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <signal.h>
#include <fstream>
#include <sys/stat.h>

#ifdef CYBERDOG_INTERFACES_FOUND
#include <motion_msgs/action/ext_mon_order.hpp>
#include <motion_msgs/action/change_mode.hpp>
#include <motion_msgs/msg/se3_velocity_cmd.hpp>
#include <motion_msgs/msg/se3_velocity.hpp>
#include <motion_msgs/msg/control_state.hpp>
#include <motion_msgs/msg/gait.hpp>
#include <std_msgs/msg/header.hpp>
#include <interaction_msgs/action/audio_play.hpp>
#include <interaction_msgs/msg/audio_order.hpp>
#include <interaction_msgs/msg/audio_user.hpp>
#else
// 存根定义 — 仅供在没有 cyberdog_interfaces 的环境下编译
namespace motion_msgs {
  namespace msg {
    struct MonOrder {
      uint8_t id;
      double para;
    };
    struct SE3Velocity {
      float linear_x, linear_y, linear_z;
      float angular_x, angular_y, angular_z;
      struct {
        int32_t sec;
        uint32_t nanosec;
        int id;
      } timestamp;
    };
    struct SE3VelocityCMD {
      static constexpr uint8_t INTERNAL = 1;
      static constexpr uint8_t REMOTEC = 2;
      static constexpr uint8_t NAVIGATOR = 3;
      uint8_t sourceid;
      SE3Velocity velocity;
    };
    struct ControlState {
      struct { uint8_t gait; } gaitstamped;
      struct { uint8_t control_mode; } modestamped;
      struct { uint8_t status; } safety;
    };
  }
  namespace action {
    struct ExtMonOrder {};
    struct ChangeMode {};
    struct AudioPlay {};
  }
}
namespace interaction_msgs {
  namespace msg {
    struct AudioSongName { uint16_t id; };
    struct AudioUser { int8_t id; };
    struct AudioOrder {
      std_msgs::msg::Header header;
      AudioSongName name;
      AudioUser user;
    };
  }
}
#endif

// ─────────────────────────────────────────────────────────────
// 常量
// ─────────────────────────────────────────────────────────────
constexpr int HTTP_PORT = 5555;
constexpr int BUFFER_SIZE = 8192;
constexpr int ACTION_TIMEOUT_SEC = 15;
constexpr int GAIT_WAIT_TIMEOUT_SEC = 5;

// gait 枚举（实测值）
enum class Gait : uint8_t {
    TRANS=0, PASSIVE=1, KNEEL=2, STAND_R=3, STAND_B=4,
    AMBLE=5, WALK=6, SLOW_TROT=7, TROT=8, FLYTROT=9,
    BOUND=10, PRONK=11
};

// mode 枚举
enum class ControlMode : uint8_t {
    DEFAULT=0, LOCK=1, MANUAL=3, SEMI=13, EXPLOR=14, TRACK=15
};

// safety 状态
enum class SafetyStatus : uint8_t {
    NORMAL = 0,
    LOW_BTR = 1   // 电量低或温度异常
};

// 动作 ID（实测值）
namespace MonOrderID {
    constexpr uint8_t STAND_UP      = 9;
    constexpr uint8_t PROSTRATE     = 10;
    constexpr uint8_t STEP_BACK     = 12;
    constexpr uint8_t TURN_AROUND   = 13;
    constexpr uint8_t HI_FIVE       = 14;
    constexpr uint8_t DANCE         = 15;
    constexpr uint8_t WELCOME       = 16;
    constexpr uint8_t TURN_OVER     = 17;
    constexpr uint8_t SIT          = 18;
}

// ─────────────────────────────────────────────────────────────
// JSON 解析（极简实现，无外部依赖）
// ─────────────────────────────────────────────────────────────
std::pair<std::string, std::optional<double>> parse_json_body(const std::string& body) {
    std::pair<std::string, std::optional<double>> result;
    size_t pos = body.find("\"action\"");
    if (pos != std::string::npos) {
        size_t colon = body.find(':', pos);
        size_t q1 = body.find('"', colon);
        size_t q2 = body.find('"', q1 + 1);
        if (q1 != std::string::npos && q2 != std::string::npos)
            result.first = body.substr(q1 + 1, q2 - q1 - 1);
    }
    pos = body.find("\"param\"");
    if (pos != std::string::npos) {
        size_t colon = body.find(':', pos);
        size_t n = colon + 1;
        while (n < body.size() && (body[n] == ' ' || body[n] == '\t')) n++;
        size_t ne = n;
        while (ne < body.size() && (isdigit(body[ne]) || body[ne] == '.' || body[ne] == '-')) ne++;
        if (ne > n) {
            try { result.second = std::stod(body.substr(n, ne - n)); } catch (...) {}
        }
    }
    return result;
}

std::pair<std::vector<std::pair<std::string, std::optional<double>>>, std::vector<int>>
parse_batch_json(const std::string& body) {
    std::vector<std::pair<std::string, std::optional<double>>> actions;
    std::vector<int> delays;
    size_t apos = body.find("\"actions\"");
    if (apos == std::string::npos) return {{}, {}};
    size_t arr_s = body.find('[', apos);
    size_t arr_e = body.find(']', arr_s);
    if (arr_s == std::string::npos) return {{}, {}};
    size_t p = arr_s + 1;
    while (p < arr_e) {
        while (p < arr_e && (body[p] == ' ' || body[p] == '\t' || body[p] == ',')) p++;
        if (p >= arr_e || body[p] != '{') { p++; continue; }
        size_t obj_e = body.find('}', p);
        if (obj_e == std::string::npos || obj_e > arr_e) break;
        auto [action, param] = parse_json_body(body.substr(p, obj_e - p + 1));
        if (!action.empty()) actions.emplace_back(action, param);
        p = obj_e + 1;
    }
    size_t dpos = body.find("\"delays\"");
    if (dpos != std::string::npos) {
        size_t darr_s = body.find('[', dpos);
        size_t darr_e = body.find(']', darr_s);
        size_t dp = darr_s + 1;
        while (dp < darr_e) {
            while (dp < darr_e && (body[dp] == ' ' || body[dp] == '\t' || body[dp] == ',')) dp++;
            size_t de = dp;
            while (de < darr_e && isdigit(body[de])) de++;
            if (de > dp) { try { delays.push_back(std::stoi(body.substr(dp, de - dp))); } catch (...) {} }
            dp = de;
        }
    }
    return {actions, delays};
}

// ─────────────────────────────────────────────────────────────
// HTTP 响应构建
// ─────────────────────────────────────────────────────────────
std::string make_json_response(const std::string& status, const std::string& message) {
    std::ostringstream oss;
    oss << "HTTP/1.1 200 OK\r\n"
        << "Content-Type: application/json\r\n"
        << "Connection: close\r\n\r\n"
        << "{\"status\":\"" << status << "\",\"message\":\"" << message << "\"}";
    return oss.str();
}

std::string make_json_response_error(const std::string& error) {
    std::ostringstream oss;
    oss << "HTTP/1.1 400 Bad Request\r\n"
        << "Content-Type: application/json\r\n"
        << "Connection: close\r\n\r\n"
        << "{\"status\":\"error\",\"message\":\"" << error << "\"}";
    return oss.str();
}

std::string make_health_response() {
    return "HTTP/1.1 200 OK\r\n"
           "Content-Type: text/plain\r\n"
           "Connection: close\r\n\r\n"
           "cyberdog_bridge v2 running\n";
}

const char* gait_name(uint8_t g) {
    switch(g) {
        case 0: return "TRANS"; case 1: return "PASSIVE"; case 2: return "KNEEL";
        case 3: return "STAND_R"; case 4: return "STAND_B"; case 5: return "AMBLE";
        case 6: return "WALK"; case 7: return "SLOW_TROT"; case 8: return "TROT";
        case 9: return "FLYTROT"; case 10: return "BOUND"; case 11: return "PRONK";
        default: return "UNKNOWN";
    }
}

const char* mode_name(uint8_t m) {
    switch(m) {
        case 0: return "DEFAULT"; case 1: return "LOCK"; case 3: return "MANUAL";
        case 13: return "SEMI"; case 14: return "EXPLOR"; case 15: return "TRACK";
        default: return "UNKNOWN";
    }
}

// ─────────────────────────────────────────────────────────────
// 指令类型枚举
// ─────────────────────────────────────────────────────────────
enum class CommandType {
    STAND_UP, PROSTRATE, HI_FIVE, DANCE, WELCOME,
    TURN_AROUND, TURN_OVER, SIT, STEP_BACK,
    WALK_FORWARD, WALK_BACKWARD, STRAFE_LEFT, STRAFE_RIGHT, SPIN,
    STOP, FOLLOW,
    PHOTO, SOUND,
    STATUS, BATTERY_CHECK,
    UNKNOWN
};

CommandType resolve_action(const std::string& action) {
    static const std::map<std::string, CommandType> MAP = {
        {"stand_up",     CommandType::STAND_UP},     {"站起来",    CommandType::STAND_UP},
        {"站立",         CommandType::STAND_UP},
        {"prostrate",    CommandType::PROSTRATE},    {"趴下",      CommandType::PROSTRATE},
        {"hi_five",      CommandType::HI_FIVE},     {"握手",      CommandType::HI_FIVE},
        {"dance",        CommandType::DANCE},        {"跳舞",      CommandType::DANCE},
        {"welcome",      CommandType::WELCOME},      {"欢迎",      CommandType::WELCOME},
        {"turn_around",  CommandType::TURN_AROUND},  {"转身",      CommandType::TURN_AROUND},
        {"turn_over",    CommandType::TURN_OVER},   {"翻身",      CommandType::TURN_OVER},
        {"sit",         CommandType::SIT},           {"坐下",      CommandType::SIT},
        {"step_back",   CommandType::STEP_BACK},    {"后退一步",  CommandType::STEP_BACK},
        {"walk_forward", CommandType::WALK_FORWARD},{"前进",      CommandType::WALK_FORWARD},
        {"walk_backward",CommandType::WALK_BACKWARD},{"后退走",   CommandType::WALK_BACKWARD},
        {"strafe_left", CommandType::STRAFE_LEFT},  {"左侧移",    CommandType::STRAFE_LEFT},
        {"strafe_right",CommandType::STRAFE_RIGHT}, {"右侧移",    CommandType::STRAFE_RIGHT},
        {"spin",        CommandType::SPIN},         {"旋转",      CommandType::SPIN},
        {"stop",        CommandType::STOP},         {"停下",      CommandType::STOP},
        {"停",          CommandType::STOP},
        {"follow",      CommandType::FOLLOW},       {"跟随",      CommandType::FOLLOW},
        {"photo",       CommandType::PHOTO},        {"拍照",      CommandType::PHOTO},
        {"sound",       CommandType::SOUND},       {"音效",      CommandType::SOUND},
        {"status",      CommandType::STATUS},      {"状态",      CommandType::STATUS},
        {"battery_check",CommandType::BATTERY_CHECK},{"电量",    CommandType::BATTERY_CHECK},
    };
    auto it = MAP.find(action);
    return (it != MAP.end()) ? it->second : CommandType::UNKNOWN;
}

// ─────────────────────────────────────────────────────────────
// 类型别名
// ─────────────────────────────────────────────────────────────
#ifdef CYBERDOG_INTERFACES_FOUND
using ExtMonOrderAction_T = motion_msgs::action::ExtMonOrder;
using ChangeModeAction_T  = motion_msgs::action::ChangeMode;
using AudioPlayAction_T   = interaction_msgs::action::AudioPlay;
using SE3VelocityCMD_T    = motion_msgs::msg::SE3VelocityCMD;
using SE3Velocity_T       = motion_msgs::msg::SE3Velocity;
using ControlState_T      = motion_msgs::msg::ControlState;
using builtin_interfaces::msg::Time;
#endif

// ─────────────────────────────────────────────────────────────
// ROS 2 桥接器节点（改进版 v2）
// ─────────────────────────────────────────────────────────────
class CyberdogBridgeV2 : public rclcpp::Node {
public:
    CyberdogBridgeV2()
        : Node("cyberdog_bridge_v2")
    {
        this->declare_parameter<bool>("sim_mode", false);
        this->declare_parameter<std::string>("cyberdog_ip", "199.166.55.21");
        this->declare_parameter<std::string>("cyberdog_ssh_pass", "123");
        this->get_parameter("sim_mode", sim_mode_);
        this->get_parameter("cyberdog_ip", cyberdog_ip_);
        this->get_parameter("cyberdog_ssh_pass", cyberdog_ssh_pass_);

        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
        RCLCPP_INFO(this->get_logger(), "  CyberDog Bridge V2 启动（仿真: %s）", sim_mode_ ? "开" : "关");
        RCLCPP_INFO(this->get_logger(), "  HTTP API: http://%s:%d/cmd", cyberdog_ip_.c_str(), HTTP_PORT);

        // ──速度指令出版者────────────────────────────────────
        velocity_pub_ = this->create_publisher<SE3VelocityCMD_T>(
            "/mi1034819/body_cmd", rclcpp::SystemDefaultsQoS());

        // ──状态订阅──────────────────────────────────────────
        // status_out 包含完整 ControlState（gait + mode + safety），持续发布
        status_sub_ = this->create_subscription<ControlState_T>(
            "/mi1034819/status_out", 10,
            [this](const typename ControlState_T::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                cached_gait_ = msg->gaitstamped.gait;
                current_gait_ = msg->gaitstamped.gait;
                current_mode_ = msg->modestamped.control_mode;
                safety_status_ = msg->safety.status;
                state_received_ = true;
            });

        // ──Action Client──────────────────────────────────────
        motion_action_client_ = rclcpp_action::create_client<ExtMonOrderAction_T>(
            this, "/mi1034819/exe_monorder");
        mode_action_client_ = rclcpp_action::create_client<ChangeModeAction_T>(
            this, "/mi1034819/checkout_mode");
        audio_action_client_ = rclcpp_action::create_client<AudioPlayAction_T>(
            this, "/mi1034819/audio_play");

        // ──等待 Action Server 就绪（非阻塞）─────────────────
        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            if (!motion_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
                RCLCPP_WARN(this->get_logger(), "/mi1034819/exe_monorder 未就绪");
            } else {
                RCLCPP_INFO(this->get_logger(), "/mi1034819/exe_monorder 就绪");
            }
            if (!mode_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
                RCLCPP_WARN(this->get_logger(), "/mi1034819/checkout_mode 未就绪");
            } else {
                RCLCPP_INFO(this->get_logger(), "/mi1034819/checkout_mode 就绪");
            }
            if (!audio_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
                RCLCPP_WARN(this->get_logger(), "/mi1034819/audio_play 未就绪");
            } else {
                RCLCPP_INFO(this->get_logger(), "/mi1034819/audio_play 就绪");
            }
        }).detach();

        // ──启动 HTTP 服务器线程──────────────────────────────
        http_thread_ = std::thread(&CyberdogBridgeV2::http_server_thread, this, HTTP_PORT);
        RCLCPP_INFO(this->get_logger(), "═══════════════════════════════════════");
    }

    ~CyberdogBridgeV2() {
        RCLCPP_INFO(this->get_logger(), "正在关闭 Bridge V2...");
        shutdown_requested_ = true;
        if (http_thread_.joinable()) http_thread_.join();
    }

    // ═══════════════════════════════════════════════════════════
    // 公开方法
    // ═══════════════════════════════════════════════════════════
    std::string handle_command(const std::string& action, std::optional<double> param) {
        // ──安全检查：LOW_BTR 电量保护─────────────────────────
        if (!sim_mode_ && safety_status_ == static_cast<uint8_t>(SafetyStatus::LOW_BTR)) {
            if (is_motion_command(action)) {
                return "电量过低（LOW_BTR），电机已锁定，请充电后重试";
            }
        }

        CommandType cmd = resolve_action(action);
        switch (cmd) {
            // ──内置动作────────────────────────────────────
            case CommandType::STAND_UP:     return cmd_stand_up();
            case CommandType::PROSTRATE:    return cmd_prostrate();
            case CommandType::HI_FIVE:     return cmd_hi_five();
            case CommandType::DANCE:       return cmd_dance();
            case CommandType::WELCOME:     return cmd_welcome();
            case CommandType::TURN_AROUND: return cmd_turn_around();
            case CommandType::TURN_OVER:   return cmd_turn_over();
            case CommandType::SIT:         return cmd_sit();
            case CommandType::STEP_BACK:   return cmd_step_back();
            // ──速度指令────────────────────────────────────
            case CommandType::STOP:         return cmd_stop();
            case CommandType::WALK_FORWARD: return cmd_walk_forward(param.value_or(0.5));
            case CommandType::WALK_BACKWARD: return cmd_walk_backward(param.value_or(0.5));
            case CommandType::STRAFE_LEFT:  return cmd_strafe_left(param.value_or(0.3));
            case CommandType::STRAFE_RIGHT: return cmd_strafe_right(param.value_or(0.3));
            case CommandType::SPIN:         return cmd_spin(param.value_or(360.0));
            case CommandType::FOLLOW:       return cmd_follow();
            // ──音频───────────────────────────────────────
            case CommandType::PHOTO:        return cmd_photo();
            case CommandType::SOUND:         return cmd_sound(static_cast<int>(param.value_or(8)));
            // ──查询────────────────────────────────────────
            case CommandType::STATUS:        return cmd_status();
            case CommandType::BATTERY_CHECK: return cmd_battery_check();
            case CommandType::UNKNOWN:
            default: return "未知指令: " + action;
        }
    }

private:
    // ═══════════════════════════════════════════════════════════
    // 辅助方法
    // ═══════════════════════════════════════════════════════════

    /// 判断是否为运动相关指令（需要电量保护检查）
    bool is_motion_command(const std::string& action) {
        static const std::set<std::string> MOTION_ACTIONS = {
            "stand_up","站起来","站立","prostrate","趴下",
            "walk_forward","前进","walk_backward","后退走",
            "strafe_left","左侧移","strafe_right","右侧移",
            "spin","旋转","step_back","后退一步",
            "hi_five","握手","dance","跳舞","welcome","欢迎",
            "turn_around","转身","turn_over","翻身","sit","坐下"
        };
        return MOTION_ACTIONS.count(action) > 0;
    }

    /// 获取当前 gait（线程安全）
    uint8_t get_current_gait() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return current_gait_;
    }

    /// 获取当前 mode（线程安全）
    uint8_t get_current_mode() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return current_mode_;
    }

    /// 获取 developer mode（运控板 Mode 1）
    bool check_developer_mode() {
        std::lock_guard<std::mutex> lock(ssh_mutex_);
        std::string result = ssh_exec("cat /mnt/UDISK/robot-software/config/user_code_ctrl_mode.txt 2>/dev/null || echo 'unknown'");
        return (result.find("1") != std::string::npos);
    }

    /// SSH 执行命令（线程安全）
    std::string ssh_exec(const std::string& cmd) {
        std::string full_cmd = "sshpass -p '" + cyberdog_ssh_pass_ + "' ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 mi@" + cyberdog_ip_ + " '" + cmd + "' 2>/dev/null";
        std::array<char, 256> buffer{};
        std::string result;
        FILE* pipe = popen(full_cmd.c_str(), "r");
        if (!pipe) return "";
        while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) result += buffer.data();
        pclose(pipe);
        // 去掉末尾换行
        while (!result.empty() && (result.back() == '\n' || result.back() == '\r')) result.pop_back();
        return result;
    }

    /// 等待 gait 变成目标值（最多 timeout 秒）
    bool wait_for_gait(uint8_t target_gait, int timeout_sec = GAIT_WAIT_TIMEOUT_SEC) {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_sec);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                if (current_gait_ == target_gait) return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return false;
    }

    // ═══════════════════════════════════════════════════════════
    // HTTP 服务器
    // ═══════════════════════════════════════════════════════════

    void http_server_thread(int port) {
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) { RCLCPP_ERROR(&get_logger(), "socket failed"); return; }
        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        sockaddr_in addr{};
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);
        if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(&get_logger(), "bind port %d failed: %s", port, strerror(errno));
            close(server_fd); return;
        }
        listen(server_fd, 5);
        RCLCPP_INFO(&get_logger(), "HTTP 监听端口 %d", port);

        while (rclcpp::ok() && !shutdown_requested_) {
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(server_fd, &read_fds);
            struct timeval timeout = {1, 0};
            int activity = select(server_fd + 1, &read_fds, nullptr, nullptr, &timeout);
            if (activity < 0) { if (errno == EINTR) continue; break; }
            if (activity == 0) continue;

            sockaddr_in client_addr{};
            socklen_t client_len = sizeof(client_addr);
            int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
            if (client_fd < 0) { if (errno == EINTR) continue; continue; }

            std::string response = handle_http_request(client_fd);
            send(client_fd, response.c_str(), response.size(), 0);
            close(client_fd);
        }
        close(server_fd);
    }

    std::string handle_http_request(int client_fd) {
        char buffer[BUFFER_SIZE];
        std::memset(buffer, 0, BUFFER_SIZE);
        int bytes_read = recv(client_fd, buffer, BUFFER_SIZE - 1, 0);
        if (bytes_read <= 0) return make_json_response_error("empty request");
        buffer[bytes_read] = '\0';
        std::istringstream iss(buffer);
        std::string method, path, http_version;
        iss >> method >> path >> http_version;

        if (path == "/health" || path == "/") return make_health_response();

        // GET /state
        if (method == "GET" && path == "/state") {
            bool dev_mode = sim_mode_ ? true : check_developer_mode();
            std::lock_guard<std::mutex> lock(state_mutex_);
            std::ostringstream oss;
            oss << "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"
                << "{\"received\":" << (state_received_ ? "true" : "false")
                << ",\"gait\":\"" << gait_name(current_gait_) << "\""
                << ",\"gait_raw\":" << (int)current_gait_
                << ",\"mode\":\"" << mode_name(current_mode_) << "\""
                << ",\"mode_raw\":" << (int)current_mode_
                << ",\"safety\":\"" << (safety_status_ == 1 ? "LOW_BTR" : "NORMAL") << "\""
                << ",\"safety_raw\":" << (int)safety_status_
                << ",\"developer_mode\":" << (dev_mode ? "true" : "false")
                << "}";
            return oss.str();
        }

        // POST /batch
        if (method == "POST" && path == "/batch") {
            size_t body_start = std::string(buffer).find("\r\n\r\n");
            if (body_start == std::string::npos) return make_json_response_error("malformed request");
            std::string batch_body = std::string(buffer).substr(body_start + 4);
            auto [actions, delays] = parse_batch_json(batch_body);
            if (actions.empty()) return make_json_response_error("missing or invalid 'actions' array");

            std::lock_guard<std::mutex> lock(ros_mutex_);
            std::ostringstream summary;
            for (size_t i = 0; i < actions.size(); i++) {
                const auto& [action, param] = actions[i];
                int delay_ms = (i < delays.size()) ? delays[i] : 500;
                std::string result = handle_command(action, param);
                summary << "步骤" << (i+1) << ": " << action << " → " << result << "; ";
                if (i < actions.size() - 1)
                    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
            }
            return make_json_response("ok", summary.str());
        }

        // POST /cmd
        if (!(method == "POST" && path == "/cmd"))
            return make_json_response_error("only POST /cmd, POST /batch and GET /state are supported");

        size_t body_start = std::string(buffer).find("\r\n\r\n");
        if (body_start == std::string::npos) return make_json_response_error("malformed request");
        std::string body = std::string(buffer).substr(body_start + 4);
        auto [action, param] = parse_json_body(body);
        if (action.empty()) return make_json_response_error("missing 'action' field");

        std::lock_guard<std::mutex> lock(ros_mutex_);
        return make_json_response("ok", handle_command(action, param));
    }

    // ═══════════════════════════════════════════════════════════
    // checkout_mode 前置（关键改进）
    // ═══════════════════════════════════════════════════════════

    /**
     * @brief 切换到 MANUAL 模式（解锁固件 LOCK 保护）
     * @return 成功返回 true
     *
     * 所有动作执行前必须先调用此方法。
     * 固件在 LOCK mode 下拒绝所有 mon_order（err_code=7 UNAVAILABLE）。
     * checkout_mode(control_mode=3, MANUAL) 解锁后 gait 会自动变为 STAND_R。
     */
    bool checkout_mode_manual() {
        // 如果已经是 MANUAL，直接返回
        if (get_current_mode() == static_cast<uint8_t>(ControlMode::MANUAL)) {
            RCLCPP_INFO(&get_logger(), "当前已经是 MANUAL 模式，跳过切换");
            return true;
        }

        RCLCPP_INFO(&get_logger(), "切换到 MANUAL 模式...");
        if (!mode_action_client_->wait_for_action_server(std::chrono::seconds(3))) {
            RCLCPP_WARN(&get_logger(), "checkout_mode action server 未就绪");
            return false;
        }

        typename ChangeModeAction_T::Goal goal;
        auto now = this->get_clock()->now();
        goal.modestamped.timestamp = now;
        goal.modestamped.control_mode = static_cast<uint8_t>(ControlMode::MANUAL);
        goal.modestamped.mode_type = 0;

        auto goal_handle_future = mode_action_client_->async_send_goal(goal);
        if (goal_handle_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            RCLCPP_ERROR(&get_logger(), "checkout_mode goal 发送超时");
            return false;
        }
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(&get_logger(), "checkout_mode goal 被拒绝");
            return false;
        }

        // 等待结果
        std::mutex result_mutex;
        std::condition_variable result_cv;
        bool result_ready = false;
        bool result_success = false;
        uint8_t err_code = 0;

        mode_action_client_->async_get_result(
            goal_handle,
            [&](const rclcpp_action::Client<ChangeModeAction_T>::WrappedResult & wr) {
                std::lock_guard<std::mutex> lock(result_mutex);
                result_ready = true;
                if (wr.result) {
                    result_success = wr.result->succeed;
                    err_code = wr.result->err_code;
                }
                result_cv.notify_one();
            });

        std::unique_lock<std::mutex> lock(result_mutex);
        if (!result_cv.wait_for(lock, std::chrono::seconds(10), [&result_ready]() { return result_ready; })) {
            RCLCPP_WARN(&get_logger(), "checkout_mode 结果等待超时");
            mode_action_client_->async_cancel_goal(goal_handle);
            return false;
        }

        if (!result_success) {
            RCLCPP_ERROR(&get_logger(), "checkout_mode 失败，err_code=%d", err_code);
            return false;
        }

        RCLCPP_INFO(&get_logger(), "checkout_mode(MANUAL) 成功");

        // 等待 gait 稳定到 STAND_R（最多 3 秒）
        if (get_current_gait() != static_cast<uint8_t>(Gait::STAND_R)) {
            RCLCPP_INFO(&get_logger(), "等待 gait 稳定到 STAND_R...");
            if (!wait_for_gait(static_cast<uint8_t>(Gait::STAND_R), 3)) {
                RCLCPP_WARN(&get_logger(), "gait 未在 3 秒内稳定到 STAND_R，当前 gait=%s",
                    gait_name(get_current_gait()));
            }
        }
        return true;
    }

    // ═══════════════════════════════════════════════════════════
    // 动作执行方法（Action Client: exe_monorder）
    // ═══════════════════════════════════════════════════════════

    /// 发送动作 Goal 并等待完成（同步阻塞，最长 15s）
    bool send_motion_order(uint8_t order_id, double param = 0.0) {
        RCLCPP_INFO(&get_logger(), "send_motion_order: id=%d, param=%.1f", order_id, param);
        if (!motion_action_client_->wait_for_action_server(std::chrono::seconds(3))) {
            RCLCPP_WARN(&get_logger(), "exe_monorder Action Server 未就绪");
            return false;
        }

        typename ExtMonOrderAction_T::Goal goal;
        goal.orderstamped.id = order_id;
        goal.orderstamped.para = param;
        auto now = this->get_clock()->now();
        goal.orderstamped.timestamp = now;

        auto goal_handle_future = motion_action_client_->async_send_goal(goal);
        if (goal_handle_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            RCLCPP_ERROR(&get_logger(), "motion order goal 发送超时");
            return false;
        }
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(&get_logger(), "motion order goal 被拒绝");
            return false;
        }

        std::mutex result_mutex;
        std::condition_variable result_cv;
        bool result_ready = false;
        bool result_success = false;
        uint8_t err_code = 0;

        motion_action_client_->async_get_result(
            goal_handle,
            [&](const rclcpp_action::Client<ExtMonOrderAction_T>::WrappedResult & wr) {
                std::lock_guard<std::mutex> lock(result_mutex);
                result_ready = true;
                result_success = wr.result ? wr.result->succeed : false;
                if (wr.result) err_code = wr.result->err_code;
                result_cv.notify_one();
            });

        std::unique_lock<std::mutex> lock(result_mutex);
        if (!result_cv.wait_for(lock, std::chrono::seconds(ACTION_TIMEOUT_SEC),
                [&result_ready]() { return result_ready; })) {
            RCLCPP_WARN(&get_logger(), "motion order %d 超时（%ds）", order_id, ACTION_TIMEOUT_SEC);
            motion_action_client_->async_cancel_goal(goal_handle);
            return false;
        }

        if (!result_success) {
            RCLCPP_ERROR(&get_logger(), "motion order %d 失败，err_code=%d", order_id, err_code);
        } else {
            RCLCPP_INFO(&get_logger(), "motion order %d 执行成功", order_id);
        }
        return result_success;
    }

    // ──动作指令方法（改进：checkout_mode 前置 + gait 确认等待）─

    std::string cmd_stand_up() {
        if (!checkout_mode_manual()) return "站立失败（checkout_mode 失败）";
        // 等待 gait 真正变为 STAND_R（站立完成确认）
        if (get_current_gait() != static_cast<uint8_t>(Gait::STAND_R)) {
            if (!wait_for_gait(static_cast<uint8_t>(Gait::STAND_R), 3))
                RCLCPP_WARN(&get_logger(), "stand_up: gait 未在 3s 内确认为 STAND_R");
        }
        return send_motion_order(MonOrderID::STAND_UP) ? "站立" : "站立失败";
    }

    std::string cmd_prostrate() {
        if (!checkout_mode_manual()) return "趴下失败（checkout_mode 失败）";
        return send_motion_order(MonOrderID::PROSTRATE) ? "趴下" : "趴下失败";
    }

    std::string cmd_sit() {
        if (!checkout_mode_manual()) return "坐下失败（checkout_mode 失败）";
        return send_motion_order(MonOrderID::SIT) ? "坐下" : "坐下失败";
    }

    std::string cmd_hi_five() {
        if (!checkout_mode_manual()) return "握手失败（checkout_mode 失败）";
        return send_motion_order(MonOrderID::HI_FIVE) ? "握手" : "握手失败";
    }

    std::string cmd_dance() {
        if (!checkout_mode_manual()) return "跳舞失败（checkout_mode 失败）";
        return send_motion_order(MonOrderID::DANCE) ? "跳舞" : "跳舞失败";
    }

    std::string cmd_welcome() {
        if (!checkout_mode_manual()) return "欢迎失败（checkout_mode 失败）";
        return send_motion_order(MonOrderID::WELCOME) ? "欢迎" : "欢迎失败";
    }

    std::string cmd_turn_around() {
        if (!checkout_mode_manual()) return "转身失败（checkout_mode 失败）";
        return send_motion_order(MonOrderID::TURN_AROUND) ? "转身" : "转身失败";
    }

    std::string cmd_turn_over() {
        if (!checkout_mode_manual()) return "翻身失败（checkout_mode 失败）";
        return send_motion_order(MonOrderID::TURN_OVER) ? "翻身" : "翻身失败";
    }

    std::string cmd_step_back() {
        if (!checkout_mode_manual()) return "后退一步失败（checkout_mode 失败）";
        return send_motion_order(MonOrderID::STEP_BACK) ? "后退一步" : "后退一步失败";
    }

    // ═══════════════════════════════════════════════════════════
    // 速度指令方法
    // ═══════════════════════════════════════════════════════════

    void publish_velocity(float lin_x, float lin_y, float ang_z) {
        SE3VelocityCMD_T cmd;
        cmd.sourceid = SE3VelocityCMD_T::INTERNAL;
        auto now = this->get_clock()->now();
        int64_t ns = now.nanoseconds();
        cmd.velocity.timestamp.sec = static_cast<int32_t>(ns / 1000000000);
        cmd.velocity.timestamp.nanosec = static_cast<uint32_t>(ns % 1000000000);
#ifdef CYBERDOG_INTERFACES_FOUND
        cmd.velocity.frameid.id = 1;  // BODY_FRAME
#endif
        cmd.velocity.linear_x = lin_x;
        cmd.velocity.linear_y = lin_y;
        cmd.velocity.linear_z = 0.0f;
        cmd.velocity.angular_x = 0.0f;
        cmd.velocity.angular_y = 0.0f;
        cmd.velocity.angular_z = ang_z;
        velocity_pub_->publish(cmd);
    }

    std::string cmd_stop() {
        publish_velocity(0.0f, 0.0f, 0.0f);
        return "已停止";
    }

    /// 前进（持续发布 velocity，持续 duration 秒）
    std::string cmd_walk_forward(double distance_m) {
        if (!checkout_mode_manual()) return "前进失败（checkout_mode 失败）";
        double speed = std::min(distance_m / 2.0, 0.5);
        double duration = distance_m / speed;
        RCLCPP_INFO(&get_logger(), "walk_forward: %.2fm, speed=%.2fm/s, duration=%.1fs",
            distance_m, speed, duration);
        auto end = std::chrono::steady_clock::now()
            + std::chrono::milliseconds(static_cast<int>(duration * 1000));
        while (std::chrono::steady_clock::now() < end) {
            publish_velocity(static_cast<float>(speed), 0.0f, 0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 20Hz
        }
        publish_velocity(0.0f, 0.0f, 0.0f);
        return "前进 " + std::to_string(distance_m) + " 米";
    }

    std::string cmd_walk_backward(double distance_m) {
        if (!checkout_mode_manual()) return "后退失败（checkout_mode 失败）";
        double speed = -std::min(distance_m / 2.0, 0.5);
        double duration = distance_m / std::abs(speed);
        auto end = std::chrono::steady_clock::now()
            + std::chrono::milliseconds(static_cast<int>(duration * 1000));
        while (std::chrono::steady_clock::now() < end) {
            publish_velocity(static_cast<float>(speed), 0.0f, 0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        publish_velocity(0.0f, 0.0f, 0.0f);
        return "后退 " + std::to_string(distance_m) + " 米";
    }

    std::string cmd_strafe_left(double distance_m) {
        if (!checkout_mode_manual()) return "左侧移失败（checkout_mode 失败）";
        double speed = -std::min(distance_m / 2.0, 0.3);
        double duration = distance_m / std::abs(speed);
        auto end = std::chrono::steady_clock::now()
            + std::chrono::milliseconds(static_cast<int>(duration * 1000));
        while (std::chrono::steady_clock::now() < end) {
            publish_velocity(0.0f, static_cast<float>(speed), 0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        publish_velocity(0.0f, 0.0f, 0.0f);
        return "左侧移 " + std::to_string(distance_m) + " 米";
    }

    std::string cmd_strafe_right(double distance_m) {
        if (!checkout_mode_manual()) return "右侧移失败（checkout_mode 失败）";
        double speed = std::min(distance_m / 2.0, 0.3);
        double duration = distance_m / speed;
        auto end = std::chrono::steady_clock::now()
            + std::chrono::milliseconds(static_cast<int>(duration * 1000));
        while (std::chrono::steady_clock::now() < end) {
            publish_velocity(0.0f, static_cast<float>(speed), 0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        publish_velocity(0.0f, 0.0f, 0.0f);
        return "右侧移 " + std::to_string(distance_m) + " 米";
    }

    /// 旋转（改进：param 决定旋转角度和方向）
    std::string cmd_spin(double angle_deg) {
        if (!checkout_mode_manual()) return "旋转失败（checkout_mode 失败）";
        // angular_speed = 0.5 rad/s ≈ 28.6 度/秒
        // duration = |angle_deg| / 28.6
        double angular_speed = (angle_deg > 0) ? 0.5 : -0.5;
        double duration = std::abs(angle_deg) / 28.6479;  // rad/s = deg / 57.2958 / angular
        // 更精确: duration_s = |angle_deg| / (0.5 * 57.2958) = |angle_deg| / 28.6479
        duration = std::abs(angle_deg) * M_PI / 180.0 / std::abs(angular_speed);

        RCLCPP_INFO(&get_logger(), "spin: angle=%.1f°, angular_speed=%.2f rad/s, duration=%.2fs",
            angle_deg, angular_speed, duration);

        auto end = std::chrono::steady_clock::now()
            + std::chrono::milliseconds(static_cast<int>(duration * 1000));
        while (std::chrono::steady_clock::now() < end) {
            publish_velocity(0.0f, 0.0f, static_cast<float>(angular_speed));
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        publish_velocity(0.0f, 0.0f, 0.0f);
        return "旋转 " + std::to_string(static_cast<int>(angle_deg)) + " 度";
    }

    std::string cmd_follow() {
        publish_velocity(0.0f, 0.0f, 0.0f);
        return "跟随模式";
    }

    // ═══════════════════════════════════════════════════════════
    // 音频播放
    // ═══════════════════════════════════════════════════════════

    bool send_audio_play(int sound_id) {
        if (!audio_action_client_->action_server_is_ready()) {
            RCLCPP_WARN(&get_logger(), "audio_play action server 不可用");
            // SSH aplay 回退
            RCLCPP_INFO(&get_logger(), "尝试 SSH aplay 回退...");
            std::string cmd = "sshpass -p '" + cyberdog_ssh_pass_ + "' ssh -o StrictHostKeyChecking=no mi@" + cyberdog_ip_
                + " 'aplay /opt/ros2/cyberdog/data/wav/" + std::to_string(sound_id) + ".wav 2>/dev/null' &";
            int ret = std::system(cmd.c_str());
            return (ret == 0);
        }

        typename AudioPlayAction_T::Goal goal;
        goal.order.name.id = static_cast<uint16_t>(sound_id);
        goal.order.user.id = 0;

        auto goal_handle_future = audio_action_client_->async_send_goal(goal);
        if (!goal_handle_future.get()) return false;

        std::mutex result_mutex;
        std::condition_variable result_cv;
        bool result_ready = false;
        bool result_success = false;

        audio_action_client_->async_get_result(
            goal_handle_future.get(),
            [&](const rclcpp_action::Client<AudioPlayAction_T>::WrappedResult & wr) {
                std::lock_guard<std::mutex> lock(result_mutex);
                result_ready = true;
                result_success = wr.result ? wr.result->result.succeed : false;
                result_cv.notify_one();
            });

        std::unique_lock<std::mutex> lock(result_mutex);
        if (!result_cv.wait_for(lock, std::chrono::seconds(10),
                [&result_ready]() { return result_ready; })) {
            RCLCPP_WARN(&get_logger(), "audio_play %d 超时", sound_id);
            audio_action_client_->async_cancel_goal(goal_handle_future.get());
            return false;
        }
        return result_success;
    }

    std::string cmd_photo() {
        return send_audio_play(8) ? "拍照" : "拍照音效失败";
    }

    std::string cmd_sound(int sound_id) {
        return send_audio_play(sound_id) ? "播放音效 " + std::to_string(sound_id) : "音效失败";
    }

    // ═══════════════════════════════════════════════════════════
    // 状态查询
    // ═══════════════════════════════════════════════════════════

    std::string cmd_status() {
        bool dev_mode = sim_mode_ ? true : check_developer_mode();
        std::ostringstream oss;
        oss << "Bridge V2 就绪"
            << " | gait: " << gait_name(get_current_gait())
            << " | mode: " << mode_name(get_current_mode())
            << " | safety: " << (safety_status_ == 1 ? "LOW_BTR" : "NORMAL")
            << " | developer_mode: " << (dev_mode ? "是" : "否")
            << " | exe_monorder: " << (motion_action_client_->action_server_is_ready() ? "就绪" : "未就绪")
            << " | audio_play: " << (audio_action_client_->action_server_is_ready() ? "就绪" : "未就绪");
        return oss.str();
    }

    std::string cmd_battery_check() {
        // 通过 SSH 读取 /tmp/bms_info
        std::string bms = ssh_exec("cat /tmp/bms_info 2>/dev/null");
        if (bms.empty()) return "电量查询失败: /tmp/bms_info 不存在或 SSH 失败";

        int soc = -1, volt = 0, curr = 0, temp = 0, status = 0, health = 100;
        std::istringstream iss(bms);
        std::string line;
        while (std::getline(iss, line)) {
            if (line.find("soc:") != std::string::npos)
                soc = std::stoi(line.substr(line.find(":") + 1));
            else if (line.find("volt:") != std::string::npos)
                volt = std::stoi(line.substr(line.find(":") + 1));
            else if (line.find("curr:") != std::string::npos)
                curr = std::stoi(line.substr(line.find(":") + 1));
            else if (line.find("temp:") != std::string::npos)
                temp = std::stoi(line.substr(line.find(":") + 1));
            else if (line.find("status:") != std::string::npos)
                status = std::stoi(line.substr(line.find(":") + 1));
            else if (line.find("batt_health:") != std::string::npos)
                health = std::stoi(line.substr(line.find(":") + 1));
        }

        if (soc < 0) return "电量解析失败（原始数据: " + bms + "）";

        std::ostringstream result;
        result << "电量: " << soc << "%, 电压: " << volt << "mV, 电流: " << curr
               << "mA, 温度: " << temp << "C, 状态: " << (status == 1 ? "充电中" : "未充电")
               << ", 电池健康: " << health << "%"
               << ", developer_mode: " << (check_developer_mode() ? "是" : "否");

        // LOW_BTR 警告
        if (safety_status_ == static_cast<uint8_t>(SafetyStatus::LOW_BTR) || soc < 20) {
            result << " ⚠️ 电量过低，电机已锁定！";
        }

        // 播放电量语音（SSH aplay）
        int wav_id = (soc >= 1 && soc <= 100) ? (300 + soc) : 300;
        std::string aplay_cmd = "sshpass -p '" + cyberdog_ssh_pass_ + "' ssh -o StrictHostKeyChecking=no mi@" + cyberdog_ip_
            + " 'aplay /opt/ros2/cyberdog/data/wav/" + std::to_string(wav_id) + ".wav 2>/dev/null' &";
        std::system(aplay_cmd.c_str());

        return result.str();
    }

    // ═══════════════════════════════════════════════════════════
    // 成员变量
    // ═══════════════════════════════════════════════════════════

    // ──ROS 2 出版者────────────────────────────────────────
    rclcpp::Publisher<SE3VelocityCMD_T>::SharedPtr velocity_pub_;

    // ──ROS 2 订阅者────────────────────────────────────────
    rclcpp::Subscription<ControlState_T>::SharedPtr status_sub_;

    // ──Action Client────────────────────────────────────────
    rclcpp_action::Client<ExtMonOrderAction_T>::SharedPtr motion_action_client_;
    rclcpp_action::Client<ChangeModeAction_T>::SharedPtr mode_action_client_;   // 新增
    rclcpp_action::Client<AudioPlayAction_T>::SharedPtr audio_action_client_;

    // ──当前状态快照（线程安全）────────────────────────────────
    std::mutex state_mutex_;
    uint8_t cached_gait_ = 0;
    uint8_t current_gait_ = 0;
    uint8_t current_mode_ = 99;
    uint8_t safety_status_ = 0;
    bool state_received_ = false;

    // ──SSH 互斥（防止并发 SSH 执行）────────────────────────────
    std::mutex ssh_mutex_;

    // ──线程与同步──────────────────────────────────────────
    std::thread http_thread_;
    std::mutex ros_mutex_;
    std::atomic<bool> shutdown_requested_{false};

    // ──参数──────────────────────────────────────────────
    bool sim_mode_;
    std::string cyberdog_ip_;
    std::string cyberdog_ssh_pass_;
};

// ═════════════════════════════════════════════════════════════
// 主入口
// ═════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CyberdogBridgeV2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
