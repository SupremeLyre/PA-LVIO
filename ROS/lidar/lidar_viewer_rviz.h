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

#ifndef VIEWER_RVIZ_H
#define VIEWER_RVIZ_H

#include "ROS/ros_compat.h"
#include "lidar/lidar_viewer.h"

#include <condition_variable>
#include <thread>

class LidarViewerRviz : public lidar::LidarViewer {

public:
#if ROS_AVAILABLE == 1
    explicit LidarViewerRviz(ros::NodeHandle &nh);
#elif ROS_AVAILABLE == 2
    explicit LidarViewerRviz(const rclcpp::Node::SharedPtr &node);
#endif

    void run() override;
    void setFinished() override;

    void updateMapPointCloud(PointCloudPtr pointcloud) override;
    void updateCurrentPointCloud(PointCloudPtr pointcloud) override;
    void updatePlanePointCloud(PointCloudPtr pointcloud) override;
    void updateCurrentPose(const Pose &pose) override;
    void updateRGBMapPointCloud(RGBPointCloudPtr pointcloud) override;

private:
#if ROS_AVAILABLE == 1
    ros::Time currentRosStamp() const {
        return ros::Time::now();
    }
#elif ROS_AVAILABLE == 2
    builtin_interfaces::msg::Time currentRosStamp() const {
        return node_->now();
    }
#endif

    void publishOdometry();
    void publishCurrentPointCloud();
    void publishMapPointCloud();
    void publishRGBMapPointCloud();
    void publishPlanePointCloud();

private:
    std::string frame_id_;

    Pose pose_;
    PathMsg path_;

#if ROS_AVAILABLE == 1
    ros::Publisher path_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher current_pub_;
    ros::Publisher plane_pub_;
    ros::Publisher map_pub_;
    ros::Publisher rgb_map_pub_;
#elif ROS_AVAILABLE == 2
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<PathMsg>::SharedPtr path_pub_;
    rclcpp::Publisher<OdometryMsg>::SharedPtr pose_pub_;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr current_pub_;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr plane_pub_;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr map_pub_;
    rclcpp::Publisher<PointCloud2Msg>::SharedPtr rgb_map_pub_;
#endif

    std::condition_variable update_sem_;
    std::mutex update_mutex_;
    std::mutex map_mutex_;

    std::atomic<bool> is_finished_{false};
    std::atomic<bool> is_map_pointcloud_{false};
    std::atomic<bool> is_rgb_map_pointcloud_{false};
    std::atomic<bool> is_current_pointcloud_{false};
    std::atomic<bool> is_plane_pointcloud_{false};
    std::atomic<bool> is_current_pose_{false};
};

#endif // VIEWER_RVIZ_H