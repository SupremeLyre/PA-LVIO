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

#include "visual_drawer_rviz.h"

#include "common/logging.h"
#include "common/rotation.h"

#if ROS_AVAILABLE == 1
VisualDrawerRviz::VisualDrawerRviz(ros::NodeHandle &nh)
    : is_finished_(false)
    , isframerdy_(false)
    , ismaprdy_(false) {

    frame_id_ = "world";

    pose_pub_           = nh.advertise<OdometryMsg>("/visual/pose", 2);
    path_pub_           = nh.advertise<PathMsg>("/visual/path", 2);
    track_image_pub_    = nh.advertise<ImageMsg>("/visual/tracking", 2);
    raw_image_pub_      = nh.advertise<ImageMsg>("/visual/image", 2);
    fixed_points_pub_   = nh.advertise<PointCloudMsg>("/visual/fixed", 2);
    current_points_pub_ = nh.advertise<PointCloudMsg>("/visual/current", 2);
}
#elif ROS_AVAILABLE == 2
VisualDrawerRviz::VisualDrawerRviz(const rclcpp::Node::SharedPtr &node)
    : is_finished_(false)
    , isframerdy_(false)
    , ismaprdy_(false)
    , node_(node) {

    frame_id_ = "world";

    pose_pub_           = node_->create_publisher<OdometryMsg>("/visual/pose", 2);
    path_pub_           = node_->create_publisher<PathMsg>("/visual/path", 2);
    track_image_pub_    = node_->create_publisher<ImageMsg>("/visual/tracking", 2);
    raw_image_pub_      = node_->create_publisher<ImageMsg>("/visual/image", 2);
    fixed_points_pub_   = node_->create_publisher<PointCloudMsg>("/visual/fixed", 2);
    current_points_pub_ = node_->create_publisher<PointCloudMsg>("/visual/current", 2);
}
#endif

void VisualDrawerRviz::setFinished() {
    is_finished_ = true;
    update_sem_.notify_one();
}

void VisualDrawerRviz::run() {

    LOGI << "Visual drawer thread is started";
    while (!is_finished_) {
        // 等待绘图更新信号
        std::unique_lock<std::mutex> lock(update_mutex_);
        update_sem_.wait(lock);

        // 发布跟踪的图像
        if (isframerdy_) {
            publishTrackingImage();

            isframerdy_ = false;
        }

        // 发布轨迹和地图点
        if (ismaprdy_) {
            publishOdometry();

            publishMapPoints();

            ismaprdy_ = false;
        }
    }
    LOGI << "Visual drawer thread is exited";
}

void VisualDrawerRviz::updateFrame(visual::VisualFrame::Ptr frame) {
    std::unique_lock<std::mutex> lock(image_mutex_);

    frame_ = frame;

    isframerdy_ = true;
    update_sem_.notify_one();
}

void VisualDrawerRviz::updateTrackedMapPoints(vector<cv::Point2f> map, vector<cv::Point2f> matched,
                                              vector<visual::MapPointType> mappoint_type) {
    std::unique_lock<std::mutex> lock(image_mutex_);
    pts2d_map_     = std::move(map);
    pts2d_matched_ = std::move(matched);
    mappoint_type_ = std::move(mappoint_type);
}

void VisualDrawerRviz::updateTrackedRefPoints(vector<cv::Point2f> ref, vector<cv::Point2f> cur) {
    std::unique_lock<std::mutex> lock(image_mutex_);
    pts2d_ref_ = std::move(ref);
    pts2d_cur_ = std::move(cur);
}

void VisualDrawerRviz::publishTrackingImage() {
    std::unique_lock<std::mutex> lock(image_mutex_);

    frame_->rawImage().copyTo(raw_image_);
    frame_->image().copyTo(track_image_);

    cv::Mat drawed;
    drawTrackingImage(track_image_, drawed);

    // OpenCV RGB format is saved as BGR

    // Tracked image
    ImageMsg image;
    image.header.stamp    = currentRosStamp();
    image.header.frame_id = frame_id_;
    image.encoding        = sensor_msgs::image_encodings::BGR8;
    image.height          = drawed.rows;
    image.width           = drawed.cols;

    size_t size = image.height * image.width * 3;
    image.step  = image.width * 3;
    image.data.resize(size);
    memcpy(image.data.data(), drawed.data, size);

    publishRosMsg(track_image_pub_, image);

    // Raw image
    image.header.stamp    = currentRosStamp();
    image.header.frame_id = frame_id_;
    image.height          = raw_image_.rows;
    image.width           = raw_image_.cols;

    if (raw_image_.channels() == 1) {
        image.encoding = sensor_msgs::image_encodings::MONO8;

        size       = image.height * image.width;
        image.step = image.width;
    } else if (raw_image_.channels() == 3) {
        image.encoding = sensor_msgs::image_encodings::BGR8;

        size       = image.height * image.width * 3;
        image.step = image.width * 3;
    }
    image.data.resize(size);
    memcpy(image.data.data(), raw_image_.data, size);

    publishRosMsg(raw_image_pub_, image);
}

void VisualDrawerRviz::publishMapPoints() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    auto stamp = currentRosStamp();

    // 发布窗口内的路标点
    PointCloudMsg current_pointcloud;

    current_pointcloud.header.stamp    = stamp;
    current_pointcloud.header.frame_id = frame_id_;

    // 获取当前点云
    for (const auto &current : current_mappoints_) {
        Point32Msg point;
        point.x = static_cast<float>(current.x());
        point.y = static_cast<float>(current.y());
        point.z = static_cast<float>(current.z());

        current_pointcloud.points.push_back(point);
    }
    publishRosMsg(current_points_pub_, current_pointcloud);

    // 发布新的地图点
    PointCloudMsg fixed_pointcloud;

    fixed_pointcloud.header.stamp    = stamp;
    fixed_pointcloud.header.frame_id = frame_id_;

    for (const auto &pts : fixed_mappoints_) {
        Point32Msg point;
        point.x = static_cast<float>(pts.x());
        point.y = static_cast<float>(pts.y());
        point.z = static_cast<float>(pts.z());

        fixed_pointcloud.points.push_back(point);
    }
    publishRosMsg(fixed_points_pub_, fixed_pointcloud);

    fixed_mappoints_.clear();
}

void VisualDrawerRviz::publishOdometry() {
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

void VisualDrawerRviz::updateCurrentMappoints(vector<Vector3d> points) {
    std::unique_lock<std::mutex> lock(map_mutex_);

    current_mappoints_ = std::move(points);
}

void VisualDrawerRviz::updateNewFixedMappoints(vector<Vector3d> points) {
    std::unique_lock<std::mutex> lock(map_mutex_);

    fixed_mappoints_ = std::move(points);
}

void VisualDrawerRviz::updateMap(const Pose &pose) {
    std::unique_lock<std::mutex> lock(map_mutex_);

    pose_ = pose;

    ismaprdy_ = true;
    update_sem_.notify_one();
}