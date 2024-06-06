/**
 * @file basic_teleop.cpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @brief Force feedback teleoperation example
 * @date 2023-08-22
 */

#include <flexiv/omni/teleop/Robot2RobotTeleop.hpp>
#include <flexiv/omni/teleop/TeleopDefs.hpp>
#include <flexiv/omni/teleop/Scheduler.hpp>
#include <getopt.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
namespace {
constexpr std::array<double, flexiv::omni::teleop::k_jointDOF> k_preferredJntPos
    = {-0.67, -0.98, 0.89, 1.55, -0.85, 0.54, 0.46}; ///< Preferred joint position
const std::array<double, flexiv::omni::teleop::k_cartDOF> k_defaultMaxRemoteWrench
    = {80.0, 80.0, 80.0, 24.0, 24.0, 24.0}; ///< Maximum contact wrench of remote robot
std::atomic<bool> g_stop_sched = {false};   ///< Atomic signal to stop scheduler tasks
}

void printHelp()
{
    // clang-format off
    std::cout<<"Invalid program arguments"<<std::endl;
    std::cout<<"     -l     [necessary] serial number of local robot."<<std::endl;
    std::cout<<"     -r     [necessary] serial number of remote robot."<<std::endl;
    std::cout<<"     -c     [necessary] license config file path."<<std::endl;
    std::cout<<"Usage: ./basic_teleop -l Rizon4s-123456 -r Rizon4s-654321 -c <path/to/licensCfg.json>"<<std::endl;
    // clang-format on
}

struct option k_longOptions[] = {
    // clang-format off
    {"local SN",               required_argument,  0, 'l'},
    {"remote SN",              required_argument,  0, 'r'},
    {"config file of license", required_argument,  0, 'c'},
    {0,                      0,                    0,  0 }
    // clang-format on
};

/**
 * @brief  Callback function for teleop task
 */
void periodicTeleopTask(flexiv::omni::teleop::Robot2RobotTeleop& teleop1,
    flexiv::omni::teleop::Robot2RobotTeleop& teleop2)
{

    try {
        // Monitor fault on the teleop robots
        if (!teleop1.isOperational() || !teleop1.isOperational()) {
            throw std::runtime_error(
                "periodicTeleopTask: Fault occurred during teleoperation, exiting ...");
        }
        // Run teleop
        teleop1.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        g_stop_sched = true;
    }
}

int main(int argc, char* argv[])
{
    std::string remoteSN_left;
    std::string localSN_left;
    std::string remoteSN_right;
    std::string localSN_right;
    std::string licCfgPath;
    int opt = 0;
    int longIndex = 0;
    while ((opt = getopt_long_only(argc, argv, "", k_longOptions, &longIndex)) != -1) {
        switch (opt) {
            case 'r':
                remoteSN_left = std::string(optarg);
                break;
            case 'l':
                localSN_left = std::string(optarg);
                break;
            case 'R':
                remoteSN_right = std::string(optarg);
                break;
            case 'L':
                localSN_right = std::string(optarg);
                break;
            case 'c':
                licCfgPath = std::string(optarg);
                break;
            default:
                printHelp();
                return 1;
        }
    }
    if (localSN_left.empty() || remoteSN_left.empty() || localSN_right.empty()
        || remoteSN_right.empty() || licCfgPath.empty()) {
        printHelp();
        return 1;
    }

    std::cout << "Flexiv Omni-Teleop teleoperation example" << std::endl;
    std::cout << "Copyright (C) 2016-2024 Flexiv" << std::endl;
    std::cout << "All Rights Reserved." << std::endl << std::endl;

    std::cout << "left Remote SN: " + remoteSN_left << std::endl;
    std::cout << "left Local SN: " + localSN_left << std::endl;
    std::cout << "right Remote SN: " + remoteSN_right << std::endl;
    std::cout << "right Local SN: " + localSN_right << std::endl;
    std::cout << "License config file: " + licCfgPath << std::endl;

    try {

        flexiv::omni::teleop::Robot2RobotTeleop teleop_left(
            localSN_left, remoteSN_left, licCfgPath);
        flexiv::omni::teleop::Robot2RobotTeleop teleop_right(
            localSN_right, remoteSN_right, licCfgPath);

        // Enable teleop robots
        teleop_left.enable();
        teleop_right.enable();

        // Init teleop robots
        teleop_left.init();
        teleop_right.init();

        // Set preferred joint position to a better configuration
        teleop_left.setLocalNullSpacePosture(k_preferredJntPos);
        teleop_left.setRemoteNullSpacePosture(k_preferredJntPos);
        teleop_right.setLocalNullSpacePosture(k_preferredJntPos);
        teleop_right.setRemoteNullSpacePosture(k_preferredJntPos);

        // Set max remote contact wrench
        teleop_left.setRemoteMaxWrench(k_defaultMaxRemoteWrench);
        teleop_right.setRemoteMaxWrench(k_defaultMaxRemoteWrench);

        // Create real-time scheduler to run periodic tasks
        flexiv::omni::teleop::Scheduler scheduler;

        // Wait for elbow posture ready
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // Run teleop

        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.AddTask(std::bind(&periodicTeleopTask, std::ref(teleop_left)),std::ref(teleop_right)),
            "HP periodic teleop", 1, scheduler.max_priority());

        // Start all added tasks, this is by default a blocking method
        scheduler.Start();

        std::cout << "Flexiv Omni-Teleop started ... ";

        // Wait a bit for any last-second robot log message to arrive and get printed
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Block until signal received
        while (!g_stop_sched) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

    } catch (const std::exception& e) {
        std::cerr << (e.what());
        return 1;
    }

    return 0;
}