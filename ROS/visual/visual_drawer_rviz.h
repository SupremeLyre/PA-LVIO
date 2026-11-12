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

#ifndef VISUAL_DRAWER_RVIZ_H
#define VISUAL_DRAWER_RVIZ_H

#include "ROS/ros_compat.h"
#include "visual/visual_drawer.h"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>

using std::string;
using std::vector;

class VisualDrawerRviz : public visual::VisualDrawer {

public:
#if ROS_AVAILABLE == 1
    explicit VisualDrawerRviz(ros::NodeHandle &nh);
#elif ROS_AVAILABLE == 2
    explicit VisualDrawerRviz(const rclcpp::Node::SharedPtr &node);
#endif

    void run() override;

    void setFinished() override;

    // 地图
    void updateNewFixedMappoints(vector<Vector3d> points) override;
    void updateCurrentMappoints(vector<Vector3d> points) override;
    void updateMap(const Pose &pose) override;

    // 跟踪图像
    void updateFrame(visual::VisualFrame::Ptr frame) override;
    void updateTrackedMapPoints(vector<cv::Point2f> map, vector<cv::Point2f> matched,
                                vector<visual::MapPointType> mappoint_type) override;
    void updateTrackedRefPoints(vector<cv::Point2f> ref, vector<cv::Point2f> cur) override;

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

    void publishTrackingImage();

    void publishMapPoints();

    void publishOdometry();

private:
    // 多线程
    std::condition_variable update_sem_;
    std::mutex update_mutex_;
    std::mutex map_mutex_;
    std::mutex image_mutex_;

    // 标志
    std::atomic<bool> is_finished_;
    std::atomic<bool> isframerdy_;
    std::atomic<bool> ismaprdy_;

    // 跟踪
    visual::VisualFrame::Ptr frame_;
    cv::Mat raw_image_;
    cv::Mat track_image_;
    vector<Vector3d> fixed_mappoints_;
    vector<Vector3d> current_mappoints_;

    Pose pose_;
    PathMsg path_;

#if ROS_AVAILABLE == 1
    ros::Publisher path_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher track_image_pub_;
    ros::Publisher raw_image_pub_;
    ros::Publisher fixed_points_pub_;
    ros::Publisher current_points_pub_;
#elif ROS_AVAILABLE == 2
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<PathMsg>::SharedPtr path_pub_;
    rclcpp::Publisher<OdometryMsg>::SharedPtr pose_pub_;
    rclcpp::Publisher<ImageMsg>::SharedPtr track_image_pub_;
    rclcpp::Publisher<ImageMsg>::SharedPtr raw_image_pub_;
    rclcpp::Publisher<PointCloudMsg>::SharedPtr fixed_points_pub_;
    rclcpp::Publisher<PointCloudMsg>::SharedPtr current_points_pub_;
#endif

    string frame_id_;
};

#endif // VISUAL_DRAWER_RVIZ_H
