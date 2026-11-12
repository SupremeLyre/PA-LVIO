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

#include "lidar_viewer_rviz.h"

#include "common/logging.h"
#include "common/rotation.h"

#include <pcl/filters/random_sample.h>
#include <pcl_conversions/pcl_conversions.h>

#if ROS_AVAILABLE == 1
LidarViewerRviz::LidarViewerRviz(ros::NodeHandle &nh)
    : is_finished_(false) {
    frame_id_ = "world";

    pose_pub_    = nh.advertise<OdometryMsg>("/lidar/pose", 10);
    path_pub_    = nh.advertise<PathMsg>("/lidar/path", 10);
    current_pub_ = nh.advertise<PointCloud2Msg>("/lidar/current", 10);
    plane_pub_   = nh.advertise<PointCloud2Msg>("/lidar/plane", 10);
    map_pub_     = nh.advertise<PointCloud2Msg>("/lidar/map", 10);
    rgb_map_pub_ = nh.advertise<PointCloud2Msg>("/lidar/rgb_map", 10);

    map_pointcloud_ = PointCloudPtr(new PointCloud);
}
#elif ROS_AVAILABLE == 2
LidarViewerRviz::LidarViewerRviz(const rclcpp::Node::SharedPtr &node)
    : node_(node)
    , is_finished_(false) {
    frame_id_ = "world";

    pose_pub_    = node_->create_publisher<OdometryMsg>("/lidar/pose", 10);
    path_pub_    = node_->create_publisher<PathMsg>("/lidar/path", 10);
    current_pub_ = node_->create_publisher<PointCloud2Msg>("/lidar/current", 10);
    plane_pub_   = node_->create_publisher<PointCloud2Msg>("/lidar/plane", 10);
    map_pub_     = node_->create_publisher<PointCloud2Msg>("/lidar/map", 10);
    rgb_map_pub_ = node_->create_publisher<PointCloud2Msg>("/lidar/rgb_map", 10);

    map_pointcloud_ = PointCloudPtr(new PointCloud);
}
#endif

void LidarViewerRviz::setFinished() {
    is_finished_ = true;
    update_sem_.notify_one();
}

void LidarViewerRviz::run() {

    LOGI << "Lidar viewer thread is started";
    while (!is_finished_) {
        std::unique_lock<std::mutex> lock(update_mutex_);
        update_sem_.wait(lock);

        if (is_current_pose_) {
            publishOdometry();
            is_current_pose_ = false;
        }

        if (is_current_pointcloud_) {
            publishCurrentPointCloud();
            is_current_pointcloud_ = false;
        }

        if (is_plane_pointcloud_) {
            publishPlanePointCloud();
            is_plane_pointcloud_ = false;
        }

        if (is_map_pointcloud_) {
            publishMapPointCloud();
            is_map_pointcloud_ = false;
        }

        if (is_rgb_map_pointcloud_) {
            publishRGBMapPointCloud();
            is_rgb_map_pointcloud_ = false;
        }
    }

    LOGI << "Lidar viewer thread is exited";
}

void LidarViewerRviz::updateRGBMapPointCloud(RGBPointCloudPtr pointcloud) {
    std::unique_lock<std::mutex> lock(map_mutex_);
    rgb_map_pointcloud_ = std::move(pointcloud);

    is_rgb_map_pointcloud_ = true;
    update_sem_.notify_one();
}

void LidarViewerRviz::updateMapPointCloud(PointCloudPtr pointcloud) {
    std::unique_lock<std::mutex> lock(map_mutex_);

    // 随机降采样 1/10
    pcl::RandomSample<PointType> random_sample;
    random_sample.setSample(pointcloud->size() / 10);
    random_sample.setInputCloud(pointcloud);
    random_sample.filter(*map_pointcloud_);

    is_map_pointcloud_ = true;
    update_sem_.notify_one();
}

void LidarViewerRviz::updateCurrentPointCloud(PointCloudPtr pointcloud) {
    std::unique_lock<std::mutex> lock(map_mutex_);
    current_pointcloud_    = std::move(pointcloud);
    is_current_pointcloud_ = true;

    update_sem_.notify_one();
}

void LidarViewerRviz::updatePlanePointCloud(PointCloudPtr pointcloud) {
    std::unique_lock<std::mutex> lock(map_mutex_);
    plane_pointcloud_    = std::move(pointcloud);
    is_plane_pointcloud_ = true;

    update_sem_.notify_one();
}

void LidarViewerRviz::updateCurrentPose(const Pose &pose) {
    std::unique_lock<std::mutex> lock(map_mutex_);
    pose_            = pose;
    is_current_pose_ = true;

    update_sem_.notify_one();
}

void LidarViewerRviz::publishOdometry() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    OdometryMsg odometry;

    auto quaternion = Rotation::matrix2quaternion(pose_.R);
    auto stamp      = currentRosStamp();

    // Odometry
    odometry.header.stamp            = stamp;
    odometry.header.frame_id         = frame_id_;
    odometry.pose.pose.position.x    = pose_.t.x();
    odometry.pose.pose.position.y    = pose_.t.y();
    odometry.pose.pose.position.z    = pose_.t.z();
    odometry.pose.pose.orientation.x = quaternion.x();
    odometry.pose.pose.orientation.y = quaternion.y();
    odometry.pose.pose.orientation.z = quaternion.z();
    odometry.pose.pose.orientation.w = quaternion.w();
    publishRosMsg(pose_pub_, odometry);

    // Path
    PoseStampedMsg pose_stamped;
    pose_stamped.header.stamp    = stamp;
    pose_stamped.header.frame_id = frame_id_;
    pose_stamped.pose            = odometry.pose.pose;

    path_.header.stamp    = stamp;
    path_.header.frame_id = frame_id_;
    path_.poses.push_back(pose_stamped);
    publishRosMsg(path_pub_, path_);
}

void LidarViewerRviz::publishCurrentPointCloud() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    auto stamp = currentRosStamp();

    PointCloud2Msg current_msg;
    pcl::toROSMsg(*current_pointcloud_, current_msg);

    // toROSMsg() will overwrite these parameters
    current_msg.header.stamp    = stamp;
    current_msg.header.frame_id = frame_id_;

    publishRosMsg(current_pub_, current_msg);
}

void LidarViewerRviz::publishPlanePointCloud() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    auto stamp = currentRosStamp();

    PointCloud2Msg plane_msg;
    pcl::toROSMsg(*plane_pointcloud_, plane_msg);

    // toROSMsg() will overwrite these parameters
    plane_msg.header.stamp    = stamp;
    plane_msg.header.frame_id = frame_id_;

    publishRosMsg(plane_pub_, plane_msg);
}

void LidarViewerRviz::publishMapPointCloud() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    auto stamp = currentRosStamp();

    PointCloud2Msg map_msg;
    pcl::toROSMsg(*map_pointcloud_, map_msg);

    // toROSMsg() will overwrite these parameters
    map_msg.header.stamp    = stamp;
    map_msg.header.frame_id = frame_id_;

    publishRosMsg(map_pub_, map_msg);
}

void LidarViewerRviz::publishRGBMapPointCloud() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    auto stamp = currentRosStamp();

    PointCloud2Msg rgb_map_msg;
    pcl::toROSMsg(*rgb_map_pointcloud_, rgb_map_msg);

    // toROSMsg() will overwrite these parameters
    rgb_map_msg.header.stamp    = stamp;
    rgb_map_msg.header.frame_id = frame_id_;

    publishRosMsg(rgb_map_pub_, rgb_map_msg);
}
