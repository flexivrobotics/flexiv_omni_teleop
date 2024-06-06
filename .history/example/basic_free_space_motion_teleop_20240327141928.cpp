/**
 * @file basic_medical_teleop.cpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 * @brief example program for medical teleoperation
 * @date 2023-08-22
 */

// Flexiv
#include <flexiv/omni/teleop/Robot2RobotTeleop.hpp>
#include <flexiv/base/StdLogger.hpp>
#include <flexiv/omni/teleop/TeleopDefs.hpp>
#include <getopt.h>
#include <iostream>
#include <thread>
#include <chrono>
namespace {
// Preferred joint position
constexpr std::array<double, flexiv::omni::teleop::k_jointDOF> k_preferredJntPos
    = {-0.67, -0.98, 0.89, 1.55, -0.85, 0.54, 0.46};
}

void printHelp()
{
    // clang-format off
    flexiv::base::log::error("Invalid program arguments");
    flexiv::base::log::info("     -l     [necessary] serial number of local robot.");
    flexiv::base::log::info("     -r     [necessary] serial number of remote robot.");
    flexiv::base::log::info("     -c     [necessary] license config file path.");
    flexiv::base::log::info("Usage: ./test_flexiv_omni_teleop -l Rizon4s-123456 -r Rizon4s-654321 -c <path/to/licensCfg.json>");
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
 * @brief test function for axis locking. This will unlock only one axis every 5 seconds
 * @param[in] teleop Omni Teleop instance
 * @param[in] AxisCmd Commands contain reference frame and axis to be locked
 * @param[in] startTimeSec Start time point
 */
void testAixsLock(flexiv::omni::teleop::Robot2RobotTeleop& teleop,
    flexiv::omni::teleop::AxisLockDefs& AxisCmd,
    const std::chrono::time_point<std::chrono::system_clock>& startTimeSec)
{
    const auto currentTimeSec = std::chrono::system_clock::now();
    auto elapsedTimeSec
        = std::chrono::duration_cast<std::chrono::seconds>(currentTimeSec - startTimeSec);

    if (elapsedTimeSec <= std::chrono::seconds(5)) {
        // Only floating X in TCP frame in first 5 seconds
        AxisCmd.coord = flexiv::omni::teleop::CD_TCP;
        AxisCmd.ori_axis_lock_list_ = {true, true, true};
        AxisCmd.trans_axis_lock_list_ = {false, true, true};
        teleop.setLocalAxisLockCmd(AxisCmd);

    } else if (elapsedTimeSec > std::chrono::seconds(5)
               && elapsedTimeSec <= std::chrono::seconds(10)) {
        // Only floating Y in TCP frame
        AxisCmd.coord = flexiv::omni::teleop::CD_TCP;
        AxisCmd.ori_axis_lock_list_ = {true, true, true};
        AxisCmd.trans_axis_lock_list_ = {true, false, true};
        teleop.setLocalAxisLockCmd(AxisCmd);
    } else if (elapsedTimeSec > std::chrono::seconds(10)
               && elapsedTimeSec <= std::chrono::seconds(15)) {
        // Only floating Z in TCP frame
        AxisCmd.coord = flexiv::omni::teleop::CD_TCP;
        AxisCmd.ori_axis_lock_list_ = {true, true, true};
        AxisCmd.trans_axis_lock_list_ = {true, true, false};
        teleop.setLocalAxisLockCmd(AxisCmd);
    } else if (elapsedTimeSec > std::chrono::seconds(15)
               && elapsedTimeSec <= std::chrono::seconds(20)) {
        // Only floating Rx in TCP frame
        AxisCmd.coord = flexiv::omni::teleop::CD_TCP;
        AxisCmd.ori_axis_lock_list_ = {false, true, true};
        AxisCmd.trans_axis_lock_list_ = {true, true, true};
        teleop.setLocalAxisLockCmd(AxisCmd);
    } else if (elapsedTimeSec > std::chrono::seconds(20)
               && elapsedTimeSec <= std::chrono::seconds(25)) {
        // Only floating Ry in TCP frame
        AxisCmd.coord = flexiv::omni::teleop::CD_TCP;
        AxisCmd.ori_axis_lock_list_ = {true, false, true};
        AxisCmd.trans_axis_lock_list_ = {true, true, true};
        teleop.setLocalAxisLockCmd(AxisCmd);
    } else if (elapsedTimeSec > std::chrono::seconds(25)
               && elapsedTimeSec <= std::chrono::seconds(30)) {
        // Only floating Rz in TCP frame
        AxisCmd.coord = flexiv::omni::teleop::CD_TCP;
        AxisCmd.ori_axis_lock_list_ = {true, true, false};
        AxisCmd.trans_axis_lock_list_ = {true, true, true};
        teleop.setLocalAxisLockCmd(AxisCmd);
    } else if (elapsedTimeSec > std::chrono::seconds(30)
               && elapsedTimeSec <= std::chrono::seconds(35)) {
        // Unlock all axes
        AxisCmd.coord = flexiv::omni::teleop::CD_TCP;
        AxisCmd.ori_axis_lock_list_ = {false, false, false};
        AxisCmd.trans_axis_lock_list_ = {false, false, false};
        teleop.setLocalAxisLockCmd(AxisCmd);
    } else if (elapsedTimeSec > std::chrono::seconds(35)
               && elapsedTimeSec <= std::chrono::seconds(40)) {
        // Only floating X in World frame
        AxisCmd.coord = flexiv::omni::teleop::CD_WORLD;
        AxisCmd.ori_axis_lock_list_ = {true, true, true};
        AxisCmd.trans_axis_lock_list_ = {false, true, true};
        teleop.setLocalAxisLockCmd(AxisCmd);
    } else if (elapsedTimeSec > std::chrono::seconds(40)
               && elapsedTimeSec <= std::chrono::seconds(45)) {
        // Only floating Y in World frame
        AxisCmd.coord = flexiv::omni::teleop::CD_WORLD;
        AxisCmd.ori_axis_lock_list_ = {true, true, true};
        AxisCmd.trans_axis_lock_list_ = {true, false, true};
        teleop.setLocalAxisLockCmd(AxisCmd);
    } else if (elapsedTimeSec > std::chrono::seconds(45)
               && elapsedTimeSec <= std::chrono::seconds(50)) {
        // Only floating Z in World frame
        AxisCmd.coord = flexiv::omni::teleop::CD_WORLD;
        AxisCmd.ori_axis_lock_list_ = {true, true, true};
        AxisCmd.trans_axis_lock_list_ = {true, true, false};
        teleop.setLocalAxisLockCmd(AxisCmd);
    } else if (elapsedTimeSec > std::chrono::seconds(50)
               && elapsedTimeSec <= std::chrono::seconds(55)) {
        // Only floating Rx in World frame
        AxisCmd.coord = flexiv::omni::teleop::CD_WORLD;
        AxisCmd.ori_axis_lock_list_ = {false, true, true};
        AxisCmd.trans_axis_lock_list_ = {true, true, true};
        teleop.setLocalAxisLockCmd(AxisCmd);
    } else if (elapsedTimeSec > std::chrono::seconds(55)
               && elapsedTimeSec <= std::chrono::seconds(60)) {
        // Only floating Ry in World frame
        AxisCmd.coord = flexiv::omni::teleop::CD_WORLD;
        AxisCmd.ori_axis_lock_list_ = {true, false, true};
        AxisCmd.trans_axis_lock_list_ = {true, true, true};
        teleop.setLocalAxisLockCmd(AxisCmd);
    } else if (elapsedTimeSec > std::chrono::seconds(60)
               && elapsedTimeSec <= std::chrono::seconds(65)) {
        // Only floating Rz in World frame
        AxisCmd.coord = flexiv::omni::teleop::CD_WORLD;
        AxisCmd.ori_axis_lock_list_ = {true, true, false};
        AxisCmd.trans_axis_lock_list_ = {true, true, true};
        teleop.setLocalAxisLockCmd(AxisCmd);
    } else {
        // Unlock all axes
        AxisCmd.coord = flexiv::omni::teleop::CD_WORLD;
        AxisCmd.ori_axis_lock_list_ = {false, false, false};
        AxisCmd.trans_axis_lock_list_ = {false, false, false};
        teleop.setLocalAxisLockCmd(AxisCmd);
    }
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
                flexiv::base::log::info("Remote SN: " + remoteSN);
                break;
            case 'l':
                localSN = std::string(optarg);
                flexiv::base::log::info("Local SN: " + localSN);
                break;
            case 'c':
                licCfgPath = std::string(optarg);
                flexiv::base::log::info("License config file: " + licCfgPath);
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

    try {

        flexiv::omni::teleop::Robot2RobotTeleop teleop(localSN, remoteSN, licCfgPath);

        // Enable teleop robots
        teleop.enable();

        // Init teleop robots
        teleop.init();

        // Set preferred joint position to a better configuration
        teleop.setLocalNullSpacePosture(k_preferredJntPos);
        teleop.setRemoteNullSpacePosture(k_preferredJntPos);

        // Wait for elbow posture ready
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // Run teleop
        flexiv::base::log::info("Omni-Teleop will run in background ... ");
        teleop.run(isBlocking);

    } catch (const std::exception& e) {
        flexiv::base::log::error(e.what());
        return 1;
    }

    return 0;
}
