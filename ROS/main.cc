/*
 * Copyright (C) 2026 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "fusion.h"

#include "lvio/common/logging.h"
#include "lvio/common/timecost.h"

#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <unistd.h>

#include <absl/strings/numbers.h>

void sigintHandler(int sig);
void checkStateThread(std::shared_ptr<Fusion> fusion);

int main(int argc, char *argv[]) {
#if ROS_AVAILABLE == 1
    // ROS节点初始化
    ros::init(argc, argv, "lvio_node", ros::InitOption::NoSigintHandler | ros::InitOption::AnonymousName);

    if (argc < 2) {
        std::cout << "Usage: pa_lvio_ros <config_file> [logtostderr]" << std::endl;
        return -1;
    }

    // 配置文件路径
    string config_file(argv[1]);

    // 命令行参数控制日志输出到标准输出
    bool logtostderr = false;
    if (argc >= 3) {
        int res;
        if (absl::SimpleAtoi(argv[2], &res)) {
            logtostderr = (res == 1);
        }
    }

    auto fusion = std::make_shared<Fusion>();
#elif ROS_AVAILABLE == 2
    // ROS 2节点初始化
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
    auto node         = std::make_shared<rclcpp::Node>("lvio_node");

    string config_file;
    bool logtostderr = false;
    if (non_ros_args.size() >= 2) {
        config_file = non_ros_args[1];
        if (non_ros_args.size() >= 3) {
            int res;
            if (absl::SimpleAtoi(non_ros_args[2], &res)) {
                logtostderr = (res == 1);
            }
        }
    } else {
        node->declare_parameter<string>("config_file", "");
        node->declare_parameter<bool>("logtostderr", false);
        node->get_parameter("config_file", config_file);
        node->get_parameter("logtostderr", logtostderr);
    }

    if (config_file.empty()) {
        std::cout << "Usage: pa_lvio_ros <config_file> [logtostderr]" << std::endl;
        rclcpp::shutdown();
        return -1;
    }

    auto fusion = std::make_shared<Fusion>(node);
#endif

    // 注册信号处理函数
    std::signal(SIGINT, sigintHandler);

    std::cout << "Fusion process is started..." << std::endl;

    // 退出检测线程
    std::thread check_thread(checkStateThread, fusion);

    // 进入消息循环
    TimeCost timecost;
    fusion->run(config_file, logtostderr);
    check_thread.join();
    LOGI << "LVIO process costs " << timecost.costInSecond() << " seconds";

    // 关闭日志
    Logging::shutdownLogging();

    return 0;
}

void sigintHandler(int sig) {
    std::cout << "Terminate by Ctrl+C " << sig << std::endl;
    global_finished = true;
}

void checkStateThread(std::shared_ptr<Fusion> fusion) {
    std::cout << "Check thread is started..." << std::endl;

    auto fusion_ptr = std::move(fusion);
    while (!global_finished) {
        sleep(1);
    }

    // 退出LVIO处理线程
    fusion_ptr->setFinished();
    std::cout << "LVIO has been shutdown ..." << std::endl;

    // 关闭ROS
#if ROS_AVAILABLE == 1
    ros::shutdown();
#elif ROS_AVAILABLE == 2
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
#endif
    std::cout << "ROS node has been shutdown ..." << std::endl;
}
