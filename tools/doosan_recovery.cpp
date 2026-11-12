// Recovery-only DRFL client. No Servo On, motion or reset requests.
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include "DRFLEx.h"

namespace {
std::atomic<bool> granted{false};
std::atomic<int> safety_state{-1};
void safety_changed(SAFETY_STATE state) {
    safety_state.store(static_cast<int>(state));
    std::cout << "safety_state=" << static_cast<int>(state)
              << " (7=RECOVERY_SOS, 8=RECOVERY_MOVE, 2=VIOLATION_STO, 3=VIOLATION_SOS)" << std::endl;
}
void state_changed(ROBOT_STATE state) {
    std::cout << "state_event=" << static_cast<int>(state) << std::endl;
}
int recovery_control(ROBOT_STATE state) {
    if (state == STATE_SAFE_STOP2) return CONTROL_RECOVERY_SAFE_STOP;
    if (state == STATE_SAFE_OFF2) return CONTROL_RECOVERY_SAFE_OFF;
    return -1;
}
void access_changed(MONITORING_ACCESS_CONTROL access) {
    granted.store(access == MONITORING_ACCESS_CONTROL_GRANT);
    std::cout << "access_event=" << static_cast<int>(access)
              << " (2=GRANT, 1=DENY, 3=LOSS)" << std::endl;
}
void print_alarm(LPLOG_ALARM alarm) {
    if (!alarm) {
        std::cout << "alarm=unavailable" << std::endl;
        return;
    }
    std::cout << "alarm level=" << static_cast<int>(alarm->_iLevel)
              << " group=" << static_cast<int>(alarm->_iGroup)
              << " index=" << alarm->_iIndex;
    for (int i = 0; i < 3; ++i)
        std::cout << " param" << i << '='
                  << std::string(alarm->_szParam[i], strnlen(alarm->_szParam[i], MAX_STRING_SIZE));
    std::cout << std::endl;
}
}

int main(int argc, char** argv) {
    std::string host, entry_method = "control";
    bool enter = false, self_test = false, force_access = false;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--host" && i+1 < argc) host = argv[++i];
        else if (arg == "--enter-recovery") enter = true;
        else if (arg == "--force-access") force_access = true;
        else if (arg == "--entry-method" && i+1 < argc) entry_method = argv[++i];
        else if (arg == "--self-test") self_test = true;
        else if (arg == "--help") {
            std::cout << "Usage: doosan_recovery --host IP [--enter-recovery [--force-access] [--entry-method control|safety-enter]]\n"
                         "Default: read state only. Close ROS bringup first.\n";
            return 0;
        } else {
            std::cerr << "Unknown/incomplete option: " << arg << '\n';
            return 2;
        }
    }
    if (entry_method != "control" && entry_method != "safety-enter") {
        std::cerr << "--entry-method must be control or safety-enter.\n";
        return 2;
    }
    if (entry_method != "control" && !enter) {
        std::cerr << "--entry-method safety-enter requires --enter-recovery.\n";
        return 2;
    }
    if (self_test) {
        for (int state = 0; state <= 15; ++state) {
            int expected = state == 9 ? 4 : state == 10 ? 5 : -1;
            if (recovery_control(static_cast<ROBOT_STATE>(state)) != expected) return 1;
        }
        std::cout << "State guards passed (no robot connection).\n";
        return 0;
    }
    if (host.empty()) {
        std::cerr << "Explicit --host IP required; use --help.\n";
        return 2;
    }
    if (force_access && !enter) {
        std::cerr << "--force-access requires --enter-recovery.\n";
        return 2;
    }
    DRAFramework::CDRFLEx robot;
    robot.set_on_monitoring_access_control(access_changed);
    robot.set_on_log_alarm(print_alarm);
    robot.set_on_monitoring_state(state_changed);
    robot.set_on_monitoring_safety_state(safety_changed);
    if (!robot.open_connection(host, 12345)) {
        std::cerr << "Connection failed.\n";
        return 1;
    }
    struct Close { DRAFramework::CDRFLEx& r; ~Close() { r.close_connection(); } } close{robot};
    SYSTEM_VERSION version{};
    if (robot.get_system_version(&version)) {
        std::cout << "controller=" << std::string(version._szController, strnlen(version._szController, MAX_SYMBOL_SIZE))
                  << " model=" << std::string(version._szRobotModel, strnlen(version._szRobotModel, MAX_SYMBOL_SIZE)) << std::endl;
    } else {
        std::cout << "controller_version=unavailable" << std::endl;
    }
    std::cout << "DRFL=" << robot.get_library_version()
              << " robot_mode=" << static_cast<int>(robot.get_robot_mode()) << std::endl;
    auto state = robot.get_robot_state();
    std::cout << "robot_state=" << static_cast<int>(state) << std::endl;
    print_alarm(robot.get_last_alarm());
    // Allow initial asynchronous safety monitoring to arrive before reporting.
    for (int i = 0; i < 20 && safety_state.load() < 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "observed_safety_state=" << safety_state.load()
              << " (-1=not received; separate from robot_state)" << std::endl;
    if (!enter || state == STATE_RECOVERY) return 0;
    if (recovery_control(state) < 0) {
        std::cerr << "Refused: recovery entry is only supported from SAFE_STOP2(9) / SAFE_OFF2(10).\n";
        return 1;
    }
    // No automatic escalation. Forced transfer requires the explicit CLI option.
    const auto access_request = force_access ? MANAGE_ACCESS_CONTROL_FORCE_REQUEST
                                             : MANAGE_ACCESS_CONTROL_REQUEST;
    std::cout << "Requesting control: " << (force_access ? "FORCE_REQUEST" : "REQUEST") << std::endl;
    if (!robot.manage_access_control(access_request)) {
        std::cerr << "Control access request rejected. No recovery request sent.\n";
        return 1;
    }
    for (int i = 0; i < 100 && !granted.load(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!granted.load()) {
        std::cerr << "Control access not granted. No recovery request sent.\n";
        return 1;
    }
    // Recheck state after the access handshake; do not act on a stale state.
    state = robot.get_robot_state();
    if (state == STATE_RECOVERY) return 0;
    int control = recovery_control(state);
    if (control < 0 || !granted.load()) {
        std::cerr << "State/access changed; no recovery request sent.\n";
        return 1;
    }
    bool accepted = false;
    if (entry_method == "safety-enter") {
        // Entry only. Never send EVENT_MOVE, servo_on, or any motion here.
        if (robot.get_robot_mode() != ROBOT_MODE_MANUAL) {
            std::cerr << "Safety entry requires MANUAL mode; no mode change requested.\n";
            return 1;
        }
        std::cout << "Sending set_safety_mode(RECOVERY, ENTER); no EVENT_MOVE or Servo On." << std::endl;
        accepted = robot.set_safety_mode(SAFETY_MODE_RECOVERY, SAFETY_MODE_EVENT_ENTER);
    } else {
        std::cout << "Sending recovery control=" << control << std::endl;
        accepted = robot.set_robot_control(static_cast<ROBOT_CONTROL>(control));
    }
    std::cout << "Recovery API returned=" << accepted << std::endl;
    if (!accepted) {
        std::cerr << "Recovery request rejected.\n";
        print_alarm(robot.get_last_alarm());
        return 1;
    }
    for (int i = 0; i < 100; ++i) {
        state = robot.get_robot_state();
        if (state == STATE_RECOVERY) {
            std::cout << "Confirmed RECOVERY(8). No motion or Servo On requested.\n";
            return 0;
        }
        if (safety_state.load() == SAFETY_STATE_RV_SOS) {
            std::cout << "Observed RECOVERY_SOS safety state(7); robot_state=" << static_cast<int>(state)
                      << ". Recovery safety stop observed; motion is not enabled by this tool.\n";
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cerr << "Recovery not confirmed; robot_state=" << static_cast<int>(state) << '\n';
    std::cerr << "observed_safety_state=" << safety_state.load() << '\n';
    print_alarm(robot.get_last_alarm());
    return 1;
}
