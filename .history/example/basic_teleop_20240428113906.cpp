/**
 * @file basic_teleop.cpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @brief Force feedback teleoperation example
 * @date 2023-08-22
 */

#include <flexiv/omni/teleop/Robot2RobotTeleop.hpp>
#include <flexiv/omni/teleop/TeleopDefs.hpp>
#include <getopt.h>
#include <iostream>
#include <thread>
#include <chrono>
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
void periodicTeleopTask(flexiv::omni::teleop::Robot2RobotTeleop& teleop)
{

    try {
        // Monitor fault on the teleop robots
        if (!teleop.isOperational()) {
            throw std::runtime_error(
                "periodicTeleopTask: Fault occurred during teleoperation, exiting ...");
        }
        // Run teleop
        teleop.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        g_stop_sched = true;
    }
}

/**
 * @brief Callback function for axis lock/unlock test
 */
void periodicConsoleTask(flexiv::omni::teleop::Robot2RobotTeleop& teleop)
{
    flexiv::omni::teleop::AxisLockDefs cmd;
    teleop.getLocalAxisLockState(cmd);

    while (!g_stop_sched) {

        std::string userInput;
        std::getline(std::cin, userInput);

        switch (userInput[0]) {
            case 'm':
                flexiv::base::log::info(
                    ">>> Simple command line GUI for teleop robot axis lock <<<");
                flexiv::base::log::info(
                    "- x: lock/unlock translational motion along X axis in World frame.");
                flexiv::base::log::info(
                    "- y: lock/unlock translational motion along Y axis in World frame.");
                flexiv::base::log::info(
                    "- z: lock/unlock translational motion along Z axis in World frame.");

                flexiv::base::log::info(
                    "- q: lock/unlock rotational motion along X axis in World frame.");
                flexiv::base::log::info(
                    "- w: lock/unlock rotational motion along Y axis in World frame.");
                flexiv::base::log::info(
                    "- e: lock/unlock rotational motion along Z axis in World frame.");

                flexiv::base::log::info(
                    "- X: lock/unlock translational motion along X axis in TCP frame.");
                flexiv::base::log::info(
                    "- Y: lock/unlock translational motion along Y axis in TCP frame.");
                flexiv::base::log::info(
                    "- Z: lock/unlock translational motion along Z axis in TCP frame.");

                flexiv::base::log::info(
                    "- Q: lock/unlock rotational motion along X axis in TCP frame.");
                flexiv::base::log::info(
                    "- W: lock/unlock rotational motion along Y axis in TCP frame.");
                flexiv::base::log::info(
                    "- E: lock/unlock rotational motion along Z axis in TCP frame.");
                flexiv::base::log::info("please input command >> ");
                break;
            case 'x':
                cmd.trans_axis_lock_list_[0] = !cmd.trans_axis_lock_list_[0];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_WORLD;

                break;
            case 'y':
                cmd.trans_axis_lock_list_[1] = !cmd.trans_axis_lock_list_[1];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_WORLD;
                break;
            case 'z':
                cmd.trans_axis_lock_list_[2] = !cmd.trans_axis_lock_list_[2];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_WORLD;
                break;
            case 'q':
                cmd.ori_axis_lock_list_[0] = !cmd.ori_axis_lock_list_[0];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_WORLD;
                break;
            case 'w':
                cmd.ori_axis_lock_list_[1] = !cmd.ori_axis_lock_list_[1];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_WORLD;
                break;
            case 'e':
                cmd.ori_axis_lock_list_[2] = !cmd.ori_axis_lock_list_[2];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_WORLD;
                break;

            case 'X':
                cmd.trans_axis_lock_list_[0] = !cmd.trans_axis_lock_list_[0];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_TCP;
                break;
            case 'Y':
                cmd.trans_axis_lock_list_[1] = !cmd.trans_axis_lock_list_[1];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_TCP;
                break;
            case 'Z':
                cmd.trans_axis_lock_list_[2] = !cmd.trans_axis_lock_list_[2];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_TCP;
                break;
            case 'Q':
                cmd.ori_axis_lock_list_[0] = !cmd.ori_axis_lock_list_[0];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_TCP;
                break;
            case 'W':
                cmd.ori_axis_lock_list_[1] = !cmd.ori_axis_lock_list_[1];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_TCP;
                break;
            case 'E':
                cmd.ori_axis_lock_list_[2] = !cmd.ori_axis_lock_list_[2];
                cmd.coord = flexiv::omni::teleop::CoordType::CD_TCP;
                break;

            default:
                flexiv::base::log::warn("Invalid command, please enter \'m\' for help \n");
                break;
        }
        teleop.setLocalAxisLockCmd(cmd);
        flexiv::base::log::info(" Axis [X] in [{}] frame locking status : [{}]",
            flexiv::omni::teleop::CoordTypeStr[cmd.coord], cmd.trans_axis_lock_list_[0]);
        flexiv::base::log::info(" Axis [Y] in [{}] frame locking status : [{}]",
            flexiv::omni::teleop::CoordTypeStr[cmd.coord], cmd.trans_axis_lock_list_[1]);
        flexiv::base::log::info(" Axis [Z] in [{}] frame locking status : [{}]",
            flexiv::omni::teleop::CoordTypeStr[cmd.coord], cmd.trans_axis_lock_list_[2]);
        flexiv::base::log::info(" Axis [Rx] in [{}] frame locking status : [{}]",
            flexiv::omni::teleop::CoordTypeStr[cmd.coord], cmd.ori_axis_lock_list_[0]);
        flexiv::base::log::info(" Axis [Ry] in [{}] frame locking status : [{}]",
            flexiv::omni::teleop::CoordTypeStr[cmd.coord], cmd.ori_axis_lock_list_[1]);
        flexiv::base::log::info(" Axis [Rz] in [{}] frame locking status : [{}]",
            flexiv::omni::teleop::CoordTypeStr[cmd.coord], cmd.ori_axis_lock_list_[2]);
    }
    return;
}

int main(int argc, char* argv[])
{
    std::string remoteSN;
    std::string localSN;
    std::string licCfgPath;
    bool isBlocking = true;
    int opt = 0;
    int longIndex = 0;
    while ((opt = getopt_long_only(argc, argv, "", k_longOptions, &longIndex)) != -1) {
        switch (opt) {
            case 'r':
                remoteSN = std::string(optarg);
                break;
            case 'l':
                localSN = std::string(optarg);
                break;
            case 'c':
                licCfgPath = std::string(optarg);
                break;
            default:
                printHelp();
                return 1;
        }
    }
    if (localSN.empty() || remoteSN.empty() || licCfgPath.empty()) {
        printHelp();
        return 1;
    }

    std::cout << "Force feedback teleoperation example" << std::endl;
    std::cout << "Copyright (C) 2016-2024 Flexiv" << std::endl;
    std::cout << "All Rights Reserved." << std::endl << std::endl;

    std::cout << "Remote SN: " + remoteSN << std::endl;
    std::cout << "Local SN: " + localSN << std::endl;
    std::cout << "License config file: " + licCfgPath << std::endl;

    try {

        try {

            flexiv::omni::teleop::Robot2RobotTeleop teleop(localSN, remoteSN, licCfgPath);

            // Enable teleop robots
            teleop.enable();

            // Init teleop robots
            teleop.init();

            // Set preferred joint position to a better configuration
            teleop.setLocalNullSpacePosture(k_preferredJntPos);
            teleop.setRemoteNullSpacePosture(k_preferredJntPos);

            // Set max remote contact wrench
            teleop.setRemoteMaxWrench(k_defaultMaxRemoteWrench);

            // Create real-time scheduler to run periodic tasks
            flexiv::omni::teleop::Scheduler scheduler;

            // Wait for elbow posture ready
            std::this_thread::sleep_for(std::chrono::seconds(3));

            // Run teleop
            flexiv::base::log::info("Omni-Teleop will run in background ... ");

            // Add periodic task with 1ms interval and highest applicable priority
            scheduler.AddTask(std::bind(&periodicTeleopTask, std::ref(teleop)),
                "HP periodic teleop", 1, scheduler.max_priority());

            scheduler.AddTask(std::bind(&periodicConsoleTask, std::ref(teleop)),
                "LP nonPeriodic console", 1000, scheduler.min_priority());

            // Start all added tasks, this is by default a blocking method
            scheduler.Start();

            // Wait a bit for any last-second robot log message to arrive and get printed
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // Block until signal received
            while (!g_stop_sched) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

        } catch (const std::exception& e) {
            flexiv::base::log::error(e.what());
            return 1;
        }

        return 0;
    }
