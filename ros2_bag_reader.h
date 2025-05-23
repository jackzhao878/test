#pragma once
#include <atomic>
#include <functional>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <thread>

namespace meskernel {

class BagReader {
 public:
  BagReader() = default;
  ~BagReader() = default;

  using ImuCallBack =
      std::function<void(const sensor_msgs::msg::Imu::ConstSharedPtr)>;

  using PointCloudCallback =
      std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr)>;

  using ImageCallback =
      std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr)>;

  using CompressedImageCallback = std::function<void(
      const sensor_msgs::msg::CompressedImage::ConstSharedPtr)>;

  void registerImuCallback(const ImuCallBack& callback);

  void registerLivoxCallback(const PointCloudCallback& callback);

  void registerImageCallback(const ImageCallback& callback0,
                             const ImageCallback& callback1 = nullptr);

  void registerCompressedmageCallback(
      const CompressedImageCallback& callback0,
      const CompressedImageCallback& callback1 = nullptr);

  void registerImageCallback0(const ImageCallback& callback);
  void registerImageCallback1(const ImageCallback& callback);

  void registerCompressedImageCallback0(
      const CompressedImageCallback& callback);
  void registerCompressedImageCallback1(
      const CompressedImageCallback& callback);

  void init(const std::string& bag_file, const std::vector<std::string>& topics,
            bool flag = false);

  void start();

  void stop();


  void processLoop();

 private:
  std::string bag_file_;
  std::string lidar_topic_;
  std::string imu_topic_;
  std::string img_topic_0, img_topic_1;  //left--0 right--1

  ImuCallBack imu_callback_;
  PointCloudCallback point_cloud_callback_;
  ImageCallback img_callback0, img_callback1;
  CompressedImageCallback compressed_img_callback0, compressed_img_callback1;
  bool is_compressed_ = false;

  std::shared_ptr<std::thread> process_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> exit_{false};
};

}  // namespace meskernel