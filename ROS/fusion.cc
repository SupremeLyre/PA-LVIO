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
#include "lidar/lidar_viewer_rviz.h"
#include "visual/visual_drawer_rviz.h"

#include "common/logging.h"
#include "common/misc.h"
#include "lidar/lidar_frame.h"
#include "visual/visual_frame.h"

#include <absl/strings/str_format.h>
#include <absl/time/clock.h>
#include <absl/time/time.h>

#include <filesystem>
#include <set>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#if ROS_AVAILABLE == 1
#include <rosbag/bag.h>
#include <rosbag/view.h>
#elif ROS_AVAILABLE == 2
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#endif

std::atomic<bool> global_finished = false;

void Fusion::setFinished() {
    if (lvio_) {
        lvio_->setFinished();
    }
}

void Fusion::run(const string &config_file, bool logtostderr) {
#if ROS_AVAILABLE == 1
    // ROS Handle
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
#elif ROS_AVAILABLE == 2
    if (!node_) {
        node_ = std::make_shared<rclcpp::Node>("lvio_node");
    }
#endif

    // 加载配置
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_file);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file " << config_file << std::endl;
        return;
    }
    auto outputpath        = config["outputpath"].as<string>();
    auto is_make_outputdir = config["is_make_outputdir"].as<bool>();

    // 如果文件夹不存在, 尝试创建
    std::error_code ec;
    if (!outputpath.empty() && !std::filesystem::is_directory(outputpath, ec)) {
        std::filesystem::create_directories(outputpath, ec);
    }
    if (outputpath.empty() || !std::filesystem::is_directory(outputpath, ec)) {
        std::cout << "Failed to open outputpath" << std::endl;
        return;
    }

    if (is_make_outputdir) {
        absl::CivilSecond cs = absl::ToCivilSecond(absl::Now(), absl::LocalTimeZone());
        absl::StrAppendFormat(&outputpath, "/T%04d%02d%02d%02d%02d%02d", cs.year(), cs.month(), cs.day(), cs.hour(),
                              cs.minute(), cs.second());
        std::filesystem::create_directory(outputpath);
    }
    string log_file = outputpath + "/lvio.log";
    Logging::initialization(logtostderr, true, log_file);

    // message topic
    string imu_topic, image_topic, lidar_topic;
    imu_topic   = config["ros"]["imu_topic"].as<string>();
    image_topic = config["ros"]["image_topic"].as<string>();
    lidar_topic = config["ros"]["lidar_topic"].as<string>();
    if (config["ros"]["use_compressed_image"]) {
        use_compressed_image_ = config["ros"]["use_compressed_image"].as<bool>();
    }
    if (config["ros"]["resize_image_to_half"]) {
        resize_image_to_half_ = config["ros"]["resize_image_to_half"].as<bool>();
    }

    // 读取ROS包
    bool is_read_bag = config["ros"]["is_read_bag"].as<bool>();
    string bag_file  = config["ros"]["bag_file"].as<string>();

    // ROS具有高优先级, 重置配置文件的ROS读取配置
#if ROS_AVAILABLE == 1
    bool is_read_bag_ros;
    if (pnh.param<bool>("is_read_bag", is_read_bag_ros, false)) {
        is_read_bag = is_read_bag_ros;
        string bag_file_ros;
        pnh.param<string>("bagfile", bag_file_ros, "");
        if (bag_file_ros.compare("") != 0) {
            bag_file = bag_file_ros;
        }
    }
#elif ROS_AVAILABLE == 2
    if (!node_->has_parameter("is_read_bag")) {
        node_->declare_parameter<bool>("is_read_bag", is_read_bag);
    }
    if (!node_->has_parameter("bagfile")) {
        node_->declare_parameter<string>("bagfile", "");
    }
    bool is_read_bag_ros = false;
    if (node_->get_parameter("is_read_bag", is_read_bag_ros)) {
        string bag_file_ros;
        node_->get_parameter("bagfile", bag_file_ros);
        if (!bag_file_ros.empty()) {
            is_read_bag = is_read_bag_ros;
            bag_file    = bag_file_ros;
        } else if (is_read_bag_ros != is_read_bag) {
            is_read_bag = is_read_bag_ros;
        }
    }
#endif

    double imu_data_rate = config["imu"]["imudatarate"].as<double>();
    imu_data_dt_         = 1.0 / imu_data_rate;

    is_use_visual_      = config["visual"]["is_use_visual"].as<bool>();
    is_use_lidar_depth_ = config["visual"]["is_use_lidar_depth"].as<bool>();

    // LiDAR参数
    is_use_lidar_            = config["lidar"]["is_use_lidar"].as<bool>();
    lidar_type_              = config["lidar"]["lidar_type"].as<int>();
    int scan_line            = config["lidar"]["scan_line"].as<int>();
    double nearest_distance  = config["lidar"]["nearest_distance"].as<double>();
    double farthest_distance = config["lidar"]["farthest_distance"].as<double>();
    double frame_rate        = config["lidar"]["frame_rate"].as<double>();

    if (!is_use_visual_ && !is_use_lidar_) {
        LOGE << "Unsupported configuration without visual and lidar";
        return;
    }

    if (is_use_lidar_ || (is_use_visual_ && is_use_lidar_depth_)) {
        // lidar数据转换对象
        lidar_converter_ =
            std::make_shared<LidarConverter>(frame_rate, scan_line, 1, nearest_distance, farthest_distance);
    }

    // ROS传感器数据
    LOGI << "Process ROS IMU messages";
    if (is_use_visual_) {
        LOGI << "Process camera messages";
    }
    if (is_use_lidar_ || (is_use_visual_ && is_use_lidar_depth_)) {
        if (lidar_type_ == Livox) {
            LOGI << "Process livox lidar messages";
        } else if (lidar_type_ == Velodyne) {
            LOGI << "Process velodyne lidar messages";
        } else if (lidar_type_ == Ouster) {
            LOGI << "Process ouster lidar messages";
        } else if (lidar_type_ == Hesai) {
            LOGI << "Process hesai lidar messages";
        } else {
            LOGE << "Unsupported lidar type";
            return;
        }
    }

    // 创建LVIO
    VisualDrawerRviz::Ptr visual_drawer = nullptr;
    LidarViewerRviz::Ptr lidar_viewer   = nullptr;
    bool is_use_visualization_          = config["is_use_visualization"].as<bool>();
    if (is_use_visualization_) {
        if (is_use_visual_) {
#if ROS_AVAILABLE == 1
            visual_drawer = std::make_shared<VisualDrawerRviz>(nh);
#elif ROS_AVAILABLE == 2
            visual_drawer = std::make_shared<VisualDrawerRviz>(node_);
#endif
        }
        if (is_use_lidar_) {
#if ROS_AVAILABLE == 1
            lidar_viewer = std::make_shared<LidarViewerRviz>(nh);
#elif ROS_AVAILABLE == 2
            lidar_viewer = std::make_shared<LidarViewerRviz>(node_);
#endif
        }
    }
    lvio_ = std::make_shared<LVIO>(config_file, outputpath, visual_drawer, lidar_viewer);

    // check is initialized
    if (!lvio_->isRunning()) {
        LOGE << "Fusion ROS terminate";
        return;
    }

    // 处理ROS数据
    if (is_read_bag) {
        LOGI << "Start to read ROS bag file";
        processRead(imu_topic, image_topic, lidar_topic, bag_file);
        LOGI << "Finish to read ROS bag file";

        // 结束处理
        global_finished = true;
    } else {
#if ROS_AVAILABLE == 1
        processSubscribe(imu_topic, image_topic, lidar_topic, nh);
#elif ROS_AVAILABLE == 2
        processSubscribe(imu_topic, image_topic, lidar_topic, node_);
#endif
    }
}

#if ROS_AVAILABLE == 1
void Fusion::processSubscribe(const string &imu_topic, const string &image_topic, const string &lidar_topic,
                              ros::NodeHandle &nh) {
    // IMU
    ros::Subscriber imu_sub;
    imu_sub = nh.subscribe<ImuMsg>(imu_topic, 200, &Fusion::imuCallback, this);

    // Visual image
    ros::Subscriber image_sub;
    if (is_use_visual_) {
        if (use_compressed_image_) {
            image_sub = nh.subscribe<CompressedImageMsg>(image_topic, 20, &Fusion::imageCallback, this);
        } else {
            image_sub = nh.subscribe<ImageMsg>(image_topic, 20, &Fusion::imageCallback, this);
        }
    }

    // Livox lidar
    ros::Subscriber lidar_sub;
    if (is_use_lidar_ || (is_use_visual_ && is_use_lidar_depth_)) {
        if (lidar_type_ == Livox) {
            lidar_sub = nh.subscribe<LivoxCustomMsg>(lidar_topic, 10, &Fusion::livoxCallback, this);
        } else if ((lidar_type_ == Velodyne) || (lidar_type_ == Ouster) || (lidar_type_ == Hesai)) {
            lidar_sub = nh.subscribe<PointCloud2Msg>(lidar_topic, 10, &Fusion::pointCloudCallback, this);
        }
    }

    LOGI << "Waiting ROS message...";

    // Enter message loopback
    ros::spin();
}
#elif ROS_AVAILABLE == 2
void Fusion::processSubscribe(const string &imu_topic, const string &image_topic, const string &lidar_topic,
                              const rclcpp::Node::SharedPtr &node) {
    // IMU
    auto imu_sub = node->create_subscription<ImuMsg>(
        imu_topic, 200, [this](const ImuMsgConstPtr msg) { imuCallback(msg); });

    // Visual image
    rclcpp::Subscription<CompressedImageMsg>::SharedPtr compressed_image_sub;
    rclcpp::Subscription<ImageMsg>::SharedPtr image_sub;
    if (is_use_visual_) {
        if (use_compressed_image_) {
            compressed_image_sub = node->create_subscription<CompressedImageMsg>(
                image_topic, 20, [this](const CompressedImageMsgConstPtr msg) { imageCallback(msg); });
        } else {
            image_sub = node->create_subscription<ImageMsg>(
                image_topic, 20, [this](const ImageMsgConstPtr msg) { imageCallback(msg); });
        }
    }

    // Livox / PointCloud2 lidar
    rclcpp::Subscription<LivoxCustomMsg>::SharedPtr livox_sub;
    rclcpp::GenericSubscription::SharedPtr livox_generic_sub;
    rclcpp::TimerBase::SharedPtr livox_type_timer;
    rclcpp::Subscription<PointCloud2Msg>::SharedPtr pointcloud_sub;
    if (is_use_lidar_ || (is_use_visual_ && is_use_lidar_depth_)) {
        if (lidar_type_ == Livox) {
            livox_sub = node->create_subscription<LivoxCustomMsg>(
                lidar_topic, 10, [this](const LivoxCustomMsgConstPtr msg) { livoxCallback(msg); });
            livox_type_timer = node->create_wall_timer(
                std::chrono::milliseconds(100),
                [this, node, lidar_topic, &livox_sub, &livox_generic_sub, &livox_type_timer]() {
                    const auto pubs = node->get_publishers_info_by_topic(lidar_topic);
                    for (const auto &pub : pubs) {
                        const auto &type = pub.topic_type();
                        if (type == "livox_ros_driver/msg/CustomMsg") {
                            livox_type_timer->cancel();
                            return;
                        }
                        if (type == "livox_ros_driver2/msg/CustomMsg" ||
                            type == "livox_interfaces/msg/CustomMsg") {
                            livox_sub.reset();
                            livox_generic_sub = node->create_generic_subscription(
                                lidar_topic, type, 10,
                                [this](std::shared_ptr<rclcpp::SerializedMessage> serialized) {
                                    auto msg = std::make_shared<LivoxCustomMsg>();
                                    rclcpp::Serialization<LivoxCustomMsg>().deserialize_message(
                                        serialized.get(), msg.get());
                                    livoxCallback(msg);
                                });
                            livox_type_timer->cancel();
                            return;
                        }
                    }
                });
        } else if ((lidar_type_ == Velodyne) || (lidar_type_ == Ouster) || (lidar_type_ == Hesai)) {
            pointcloud_sub = node->create_subscription<PointCloud2Msg>(
                lidar_topic, 10, [this](const PointCloud2MsgConstPtr msg) { pointCloudCallback(msg); });
        }
    }

    LOGI << "Waiting ROS message...";

    // Enter message loopback
    rclcpp::spin(node);
}
#endif

void Fusion::processRead(const string &imu_topic, const string &image_topic, const string &lidar_topic,
                         const string &bagfile) {
    // 消息列表
    vector<string> topics;
    topics.emplace_back(imu_topic);
    topics.emplace_back(image_topic);
    topics.emplace_back(lidar_topic);

    bool is_read_lidar = is_use_lidar_ || (is_use_visual_ && is_use_lidar_depth_);
    LOGI << "Read bag with topics: " << imu_topic << ", " << image_topic << ", " << lidar_topic;

#if ROS_AVAILABLE == 1
    // 遍历ROS包
    rosbag::Bag bag(bagfile);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for (rosbag::MessageInstance const &msg : view) {
        // 等待数据处理完毕
        while (!global_finished && !lvio_->canAddData()) {
            usleep(100);
        }

        // 强制退出信号
        if (global_finished) {
            return;
        }

        // 按数据包内消息的实际类型分发, 避免配置与数据包类型不一致时实例化为空
        if (msg.getTopic() == imu_topic) {
            auto imumsg = msg.instantiate<ImuMsg>();
            imuCallback(imumsg);
        } else if (is_use_visual_ && (msg.getTopic() == image_topic)) {
            if (use_compressed_image_) {
                auto imagemsg = msg.instantiate<CompressedImageMsg>();
                imageCallback(imagemsg);
            } else {
                auto imagemsg = msg.instantiate<ImageMsg>();
                imageCallback(imagemsg);
            }
        } else if (is_read_lidar && (msg.getTopic() == lidar_topic)) {
            if (lidar_type_ == Livox) {
                auto livox_ptr = msg.instantiate<LivoxCustomMsg>();
                livoxCallback(livox_ptr);
            } else if ((lidar_type_ == Velodyne) || (lidar_type_ == Ouster) || (lidar_type_ == Hesai)) {
                auto points_ptr = msg.instantiate<PointCloud2Msg>();
                pointCloudCallback(points_ptr);
            }
        }
    }
#elif ROS_AVAILABLE == 2
    rosbag2_cpp::Reader reader;
    reader.open(bagfile);

    rosbag2_storage::StorageFilter filter;
    filter.topics = topics;
    reader.set_filter(filter);

    rclcpp::Serialization<ImuMsg> imu_serializer;
    rclcpp::Serialization<CompressedImageMsg> compressed_image_serializer;
    rclcpp::Serialization<ImageMsg> image_serializer;
    rclcpp::Serialization<LivoxCustomMsg> livox_serializer;
    rclcpp::Serialization<PointCloud2Msg> pointcloud_serializer;

    while (reader.has_next()) {
        // 等待数据处理完毕
        while (!global_finished && !lvio_->canAddData()) {
            usleep(100);
        }

        // 强制退出信号
        if (global_finished) {
            return;
        }

        auto serialized_msg = reader.read_next();
        rclcpp::SerializedMessage extracted_serialized_msg(*serialized_msg->serialized_data);

        if (serialized_msg->topic_name == imu_topic) {
            auto imumsg = std::make_shared<ImuMsg>();
            imu_serializer.deserialize_message(&extracted_serialized_msg, imumsg.get());
            imuCallback(imumsg);
        } else if (is_use_visual_ && (serialized_msg->topic_name == image_topic)) {
            if (use_compressed_image_) {
                auto imagemsg = std::make_shared<CompressedImageMsg>();
                compressed_image_serializer.deserialize_message(&extracted_serialized_msg, imagemsg.get());
                imageCallback(imagemsg);
            } else {
                auto imagemsg = std::make_shared<ImageMsg>();
                image_serializer.deserialize_message(&extracted_serialized_msg, imagemsg.get());
                imageCallback(imagemsg);
            }
        } else if (is_read_lidar && (serialized_msg->topic_name == lidar_topic)) {
            if (lidar_type_ == Livox) {
                auto livox_ptr = std::make_shared<LivoxCustomMsg>();
                livox_serializer.deserialize_message(&extracted_serialized_msg, livox_ptr.get());
                livoxCallback(livox_ptr);
            } else if ((lidar_type_ == Velodyne) || (lidar_type_ == Ouster) || (lidar_type_ == Hesai)) {
                auto points_ptr = std::make_shared<PointCloud2Msg>();
                pointcloud_serializer.deserialize_message(&extracted_serialized_msg, points_ptr.get());
                pointCloudCallback(points_ptr);
            }
        }
    }
#endif

    // 等待数据处理结束
    int sec_cnts = 0;
    while (!lvio_->isBufferEmpty()) {
        sleep(1);
        if (sec_cnts++ > 20) {
            LOGW << "Waiting lvio processing timeout";
            break;
        }
    }
}

void Fusion::imuCallback(const ImuMsgConstPtr &imumsg) {
    if (!imumsg) {
        return;
    }
    imu_pre_ = imu_;

    // Time
    imu_.time = stampToSec(imumsg->header.stamp);
    // delta time
    imu_.dt = imu_.time - imu_pre_.time;

    // IMU measurements, Front-Right-Down
    imu_.dtheta[0] = imumsg->angular_velocity.x * imu_.dt;
    imu_.dtheta[1] = imumsg->angular_velocity.y * imu_.dt;
    imu_.dtheta[2] = imumsg->angular_velocity.z * imu_.dt;
    imu_.dvel[0]   = imumsg->linear_acceleration.x * imu_.dt;
    imu_.dvel[1]   = imumsg->linear_acceleration.y * imu_.dt;
    imu_.dvel[2]   = imumsg->linear_acceleration.z * imu_.dt;

    // 数据未准备好
    if (imu_pre_.time == 0) {
        return;
    }

    addImuData(imu_);
}

void Fusion::addImuData(const IMU &imu) {
    imu_buffer_.push(imu);
    while (!imu_buffer_.empty()) {
        auto temp = imu_buffer_.front();

        // add new IMU
        if (lvio_->addNewImu(temp)) {
            imu_buffer_.pop();
        } else {
            // thread lock failed, try next time
            break;
        }
    }
}

void Fusion::imageCallback(const CompressedImageMsgConstPtr &imagemsg) {
    if (!imagemsg) {
        return;
    }
    cv::Mat image;

    vector<uint8_t> buffer(imagemsg->data);
    image = cv::imdecode(buffer, cv::IMREAD_COLOR);

    if (resize_image_to_half_) {
        cv::resize(image, image, image.size() / 2);
    }

    // Add new Image to LVIO
    auto latest_frame = visual::VisualFrame::createFrame(stampToSec(imagemsg->header.stamp), image);

    visual_frame_buffer_.push(latest_frame);
    while (!visual_frame_buffer_.empty()) {
        auto frame = visual_frame_buffer_.front();
        if (lvio_->addNewVisualFrame(frame)) {
            visual_frame_buffer_.pop();
        } else {
            break;
        }
    }
}

void Fusion::imageCallback(const ImageMsgConstPtr &imagemsg) {
    if (!imagemsg) {
        return;
    }
    cv::Mat image;

    // 构造图像
    if (imagemsg->encoding == sensor_msgs::image_encodings::MONO8) {
        // Gray
        image = cv::Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC1);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width);
    } else if (imagemsg->encoding == sensor_msgs::image_encodings::BGR8) {
        // BGR8

        // OpenCV color format is RGB
        image = cv::Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC3);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width * 3);
    } else if (imagemsg->encoding == sensor_msgs::image_encodings::BAYER_RGGB8) {
        // BayerBG8
        image = cv::Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC1);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width);

        // OpenCV color format is RGB
        cv::cvtColor(image, image, cv::COLOR_BayerBG2RGB);
    } else if (imagemsg->encoding == sensor_msgs::image_encodings::BAYER_BGGR8) {
        // BayerRG8
        image = cv::Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC1);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width);

        // OpenCV color format is RGB
        cv::cvtColor(image, image, cv::COLOR_BayerRG2RGB);
    }

    if (resize_image_to_half_) {
        cv::resize(image, image, image.size() / 2);
    }

    // Add new Image to LVIO
    auto latest_frame = visual::VisualFrame::createFrame(stampToSec(imagemsg->header.stamp), image);

    visual_frame_buffer_.push(latest_frame);
    while (!visual_frame_buffer_.empty()) {
        auto frame = visual_frame_buffer_.front();
        if (lvio_->addNewVisualFrame(frame)) {
            visual_frame_buffer_.pop();
        } else {
            break;
        }
    }

    LOG_EVERY_N(INFO, 20) << "Raw data time " << Logging::doubleData(imu_.time) << ", "
                          << Logging::doubleData(latest_frame->stamp());
}

void Fusion::livoxCallback(const LivoxCustomMsgConstPtr &lidarmsg) {
    if (!lidarmsg) {
        return;
    }
    PointCloudCustomPtr pointcloud_ds  = PointCloudCustomPtr(new PointCloudCustom);
    PointCloudCustomPtr pointcloud_raw = PointCloudCustomPtr(new PointCloudCustom);
    double start, end;

    lidar_converter_->livoxPointCloudConvertion(lidarmsg, pointcloud_raw, pointcloud_ds, start, end);

    // 无有效点云
    if (pointcloud_ds->points.empty()) {
        return;
    }

    if (is_use_lidar_) {
        // Create lidar frame
        auto frame = LidarFrame::createFrame(pointcloud_ds->points.front().time, pointcloud_ds->points.back().time,
                                             pointcloud_ds);
        lvio_->addNewLidarFrame(frame);
    }
    if (is_use_visual_ && is_use_lidar_depth_) {
        // 激光深度增强点云
        lvio_->addNewPointCloud(pointcloud_raw);
    }
}

void Fusion::pointCloudCallback(const PointCloud2MsgConstPtr &lidarmsg) {
    if (!lidarmsg) {
        return;
    }
    PointCloudCustomPtr pointcloud = PointCloudCustomPtr(new PointCloudCustom);
    double start = 0, end = 0;

    // Velodyne自定义格式
    if (lidar_type_ == Velodyne) {
        lidar_converter_->velodynePointCloudConvertion(lidarmsg, pointcloud, start, end);
    } else if (lidar_type_ == Ouster) {
        lidar_converter_->ousterPointCloudConvertion(lidarmsg, pointcloud, start, end);
    } else if (lidar_type_ == Hesai) {
        lidar_converter_->hesaiPointCloudConvertion(lidarmsg, pointcloud, start, end);
    }

    if (is_use_lidar_) {
        // Create lidar frame
        auto frame = LidarFrame::createFrame(start, end, pointcloud);
        lvio_->addNewLidarFrame(frame);
    }
    if (is_use_visual_ && is_use_lidar_depth_) {
        // 激光深度增强点云
        lvio_->addNewPointCloud(pointcloud);
    }
}
