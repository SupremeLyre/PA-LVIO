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

#ifndef ROS_COMPAT_H
#define ROS_COMPAT_H

#include <cstdint>

#if ROS_AVAILABLE == 1
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

using ImuMsg                    = sensor_msgs::Imu;
using ImuMsgConstPtr            = sensor_msgs::ImuConstPtr;
using ImageMsg                  = sensor_msgs::Image;
using ImageMsgConstPtr          = sensor_msgs::ImageConstPtr;
using CompressedImageMsg        = sensor_msgs::CompressedImage;
using CompressedImageMsgConstPtr = sensor_msgs::CompressedImageConstPtr;
using PointCloudMsg             = sensor_msgs::PointCloud;
using PointCloud2Msg            = sensor_msgs::PointCloud2;
using PointCloud2MsgConstPtr    = sensor_msgs::PointCloud2ConstPtr;
using OdometryMsg               = nav_msgs::Odometry;
using PathMsg                   = nav_msgs::Path;
using Point32Msg                = geometry_msgs::Point32;
using PoseStampedMsg            = geometry_msgs::PoseStamped;
using LivoxCustomMsg            = livox_ros_driver::CustomMsg;
using LivoxCustomMsgConstPtr    = livox_ros_driver::CustomMsgConstPtr;
using LivoxCustomPoint          = livox_ros_driver::CustomPoint;

template <typename StampT>
inline double stampToSec(const StampT &stamp) {
    return stamp.toSec();
}

template <typename StampT>
inline uint64_t stampToNanosec(const StampT &stamp) {
    return stamp.toNSec();
}

template <typename PubT, typename MsgT>
inline void publishRosMsg(PubT &pub, const MsgT &msg) {
    pub.publish(msg);
}

#elif ROS_AVAILABLE == 2
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <livox_ros_driver/msg/custom_msg.hpp>
#include <livox_ros_driver/msg/custom_point.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using ImuMsg                    = sensor_msgs::msg::Imu;
using ImuMsgConstPtr            = sensor_msgs::msg::Imu::ConstSharedPtr;
using ImageMsg                  = sensor_msgs::msg::Image;
using ImageMsgConstPtr          = sensor_msgs::msg::Image::ConstSharedPtr;
using CompressedImageMsg        = sensor_msgs::msg::CompressedImage;
using CompressedImageMsgConstPtr = sensor_msgs::msg::CompressedImage::ConstSharedPtr;
using PointCloudMsg             = sensor_msgs::msg::PointCloud;
using PointCloud2Msg            = sensor_msgs::msg::PointCloud2;
using PointCloud2MsgConstPtr    = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
using OdometryMsg               = nav_msgs::msg::Odometry;
using PathMsg                   = nav_msgs::msg::Path;
using Point32Msg                = geometry_msgs::msg::Point32;
using PoseStampedMsg            = geometry_msgs::msg::PoseStamped;
using LivoxCustomMsg            = livox_ros_driver::msg::CustomMsg;
using LivoxCustomMsgConstPtr    = livox_ros_driver::msg::CustomMsg::ConstSharedPtr;
using LivoxCustomPoint          = livox_ros_driver::msg::CustomPoint;
using LivoxCustomMsg2           = livox_ros_driver2::msg::CustomMsg;
using LivoxCustomMsg2ConstPtr   = livox_ros_driver2::msg::CustomMsg::ConstSharedPtr;
using LivoxCustomPoint2         = livox_ros_driver2::msg::CustomPoint;

template <typename StampT>
inline double stampToSec(const StampT &stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

template <typename StampT>
inline uint64_t stampToNanosec(const StampT &stamp) {
    return static_cast<uint64_t>(stamp.sec) * 1000000000ULL + static_cast<uint64_t>(stamp.nanosec);
}

template <typename PubT, typename MsgT>
inline void publishRosMsg(PubT &pub, const MsgT &msg) {
    pub->publish(msg);
}
#else
#error "ROS_AVAILABLE must be defined as 1 (ROS 1) or 2 (ROS 2)"
#endif

#endif // ROS_COMPAT_H
