#include <glog/logging.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <iostream>

#include "ros2_bag_reader.h"

namespace meskernel {

void BagReader::registerImuCallback(const ImuCallBack& callback) {
  imu_callback_ = callback;
}

void BagReader::registerLivoxCallback(const PointCloudCallback& callback) {
  point_cloud_callback_ = callback;
}

void BagReader::init(const std::string& bag_file,
                     const std::vector<std::string>& topics, bool flag) {

  bag_file_ = bag_file;
  lidar_topic_ = topics[0];
  imu_topic_ = topics[1];

  if (topics.size() == 3) {
    img_topic_0 = topics[2];
  }

  else if (topics.size() == 4) {
    img_topic_0 = topics[2];
    img_topic_1 = topics[3];
  }

  is_compressed_ = flag;

  exit_.store(false);
}

void BagReader::start() {
  if (running_) {
    LOG(WARNING) << "BagReader is running already";
    return;
  }

  process_thread_ =
      std::make_shared<std::thread>(&BagReader::processLoop, this);
  running_.store(true);
}

void BagReader::stop() {
  exit_.store(true);
  running_.store(false);
  if (process_thread_->joinable()) {
    process_thread_->join();
  }
}

void BagReader::processLoop() {
  // rosbag2_cpp::StorageOptions storage_options; // foxy
  rosbag2_storage::StorageOptions storage_options;   //humble
  storage_options.uri = bag_file_;
  storage_options.storage_id = "sqlite3";
  rosbag2_cpp::ConverterOptions converter_opt;
  converter_opt.input_serialization_format = "cdr";
  converter_opt.output_serialization_format = "cdr";

  rosbag2_cpp::readers::SequentialReader reader;
  reader.open(storage_options, converter_opt);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> lidar_serialization;
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
  rclcpp::Serialization<sensor_msgs::msg::Image> img_serialization;
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage>
      compressed_img_serialization;

  int64_t last_msg_timestamp = 0;
  while (!exit_ && reader.has_next()) {

    std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg;
    msg = reader.read_next();
    
    const auto msg_timestamp = msg->time_stamp;
    if (last_msg_timestamp != 0) {
      const auto msg_time_diff = msg_timestamp - last_msg_timestamp;
      std::this_thread::sleep_for(std::chrono::nanoseconds(msg_time_diff / 2));
    }

    if (msg->topic_name == lidar_topic_) {
      if (point_cloud_callback_ != nullptr) {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        sensor_msgs::msg::PointCloud2::SharedPtr lidar_ros_msg =
            std::make_shared<sensor_msgs::msg::PointCloud2>();
        lidar_serialization.deserialize_message(&serialized_msg,
                                                lidar_ros_msg.get());

        point_cloud_callback_(lidar_ros_msg);
      }

    } else if (msg->topic_name == imu_topic_) {
      if (imu_callback_ != nullptr) {
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        sensor_msgs::msg::Imu::SharedPtr imu_ros_msg =
            std::make_shared<sensor_msgs::msg::Imu>();
        imu_serialization.deserialize_message(&serialized_msg,
                                              imu_ros_msg.get());
        imu_callback_(imu_ros_msg);
      }
    } else if (msg->topic_name == img_topic_0) {

      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      if (is_compressed_ && compressed_img_callback0 != nullptr) {

        sensor_msgs::msg::CompressedImage::SharedPtr img_ros_msg =
            std::make_shared<sensor_msgs::msg::CompressedImage>();
        compressed_img_serialization.deserialize_message(&serialized_msg,
                                                         img_ros_msg.get());

        compressed_img_callback0(img_ros_msg);

      } else {
        if (img_callback0 != nullptr) {
          rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
          sensor_msgs::msg::Image::SharedPtr img_ros_msg =
              std::make_shared<sensor_msgs::msg::Image>();
          img_serialization.deserialize_message(&serialized_msg,
                                                img_ros_msg.get());
          img_callback0(img_ros_msg);
        }
      }
    }
    else if (msg->topic_name == img_topic_1) {
      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
      if (is_compressed_ && compressed_img_callback1 != nullptr) {

        sensor_msgs::msg::CompressedImage::SharedPtr img_ros_msg =
            std::make_shared<sensor_msgs::msg::CompressedImage>();
        compressed_img_serialization.deserialize_message(&serialized_msg,
                                                         img_ros_msg.get());

        compressed_img_callback1(img_ros_msg);

      } else {
        if (img_callback1 != nullptr) {
          rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
          sensor_msgs::msg::Image::SharedPtr img_ros_msg =
              std::make_shared<sensor_msgs::msg::Image>();
          img_serialization.deserialize_message(&serialized_msg,
                                                img_ros_msg.get());
          img_callback1(img_ros_msg);
        }
      }
    }

    last_msg_timestamp = msg_timestamp;
  }
  LOG(WARNING) << "bag reading finished...";
}

void BagReader::registerImageCallback0(const ImageCallback& callback) {
  img_callback0 = callback;
}

void BagReader::registerImageCallback1(const ImageCallback& callback) {
  img_callback1 = callback;
}

void BagReader::registerCompressedImageCallback0(
    const CompressedImageCallback& callback) {
  compressed_img_callback0 = callback;
}

void BagReader::registerCompressedImageCallback1(
    const CompressedImageCallback& callback) {
  compressed_img_callback1 = callback;
}

void BagReader::registerImageCallback(
    const ImageCallback& callback0, const ImageCallback& callback1) {
  img_callback0 = callback0;
  img_callback1 = callback1;
}

void BagReader::registerCompressedmageCallback(
    const CompressedImageCallback& callback0,
    const CompressedImageCallback& callback1) {
  compressed_img_callback0 = callback0;
  compressed_img_callback1 = callback1;
}

}  // namespace meskernel