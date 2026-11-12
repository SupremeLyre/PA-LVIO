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

#ifndef FUSION_H
#define FUSION_H

#include "ROS/ros_compat.h"
#include "lidar/lidar_converter.h"

#include "common/types.h"
#include "lvio/lvio.h"

#include <atomic>
#include <memory>
#include <queue>

extern std::atomic<bool> global_finished;

class Fusion {

public:
    Fusion() = default;
#if ROS_AVAILABLE == 2
    explicit Fusion(const rclcpp::Node::SharedPtr &node)
        : node_(node) {
    }
#endif

    void run(const string &config_file, bool logtostderr = true);

    void setFinished();

private:
    void imuCallback(const ImuMsgConstPtr &imumsg);

    void imageCallback(const ImageMsgConstPtr &imagemsg);
    void imageCallback(const CompressedImageMsgConstPtr &imagemsg);

    void livoxCallback(const LivoxCustomMsgConstPtr &lidarmsg);
    void pointCloudCallback(const PointCloud2MsgConstPtr &lidarmsg);

    void addImuData(const IMU &imu);

#if ROS_AVAILABLE == 1
    void processSubscribe(const string &imu_topic, const string &image_topic, const string &lidar_topic,
                          ros::NodeHandle &nh);
#elif ROS_AVAILABLE == 2
    void processSubscribe(const string &imu_topic, const string &image_topic, const string &lidar_topic,
                          const rclcpp::Node::SharedPtr &node);
#endif
    void processRead(const string &imu_topic, const string &image_topic, const string &lidar_topic,
                     const string &bagfile);

private:
#if ROS_AVAILABLE == 2
    rclcpp::Node::SharedPtr node_;
#endif
    std::shared_ptr<LVIO> lvio_;

    bool is_use_visual_{false};
    bool is_use_lidar_{false};
    bool is_use_lidar_depth_{false};

    IMU imu_{.time = 0}, imu_pre_{.time = 0};
    double imu_data_dt_{0.005};

    bool use_compressed_image_{false};
    bool resize_image_to_half_{false};

    std::queue<IMU> imu_buffer_;
    std::queue<VisualFrame::Ptr> visual_frame_buffer_;

    LidarConverter::Ptr lidar_converter_;
    int lidar_type_;
};

#endif // FUSION_ROS_H
