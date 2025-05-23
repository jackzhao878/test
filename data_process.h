#pragma once

#include "ros2_bag_reader.h"

#include <filesystem>

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>

namespace meskernel {

enum ImageSaveType {
  KNotSaveImg = 0,
  KSaveVideo,
  KSaveSingleImg,
  KSaveImgs,
  KNotSaveCompressedImg,
  KSaveCompressedVideo,
  KSaveCompressedSingleImg,
  KSaveCompressedImgs
};

enum PCDSaveType { KNotSavePCD = 0, KSaveSinglePCD, KSaveGlobalPCD };

enum TimeStampSaveType { KNotSaveTime = 0, KSaveTime };

using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloud_ptr = pcl::PointCloud<PointT>::Ptr;

class DataProcess {
 public:
  DataProcess(const std::string& bag_file,
              const std::vector<std::string>& topics,
              ImageSaveType img_save_type = KNotSaveImg,
              PCDSaveType pcd_save_type = KNotSavePCD,
              TimeStampSaveType time_save_type = KNotSaveTime);

  void imuDataCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void imgDataCallback0(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void imgDataCallback1(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void lidarDataCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void compressedImgDataCallback0(
      sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  void compressedImgDataCallback1(
      sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);

  void set_video_format(int w, int h, int fps);

  void start();
  
  void stop();
 protected:
  void init();
  void createFolder(const std::string& path);
  void saveTimeStamp(const std::ofstream& f, const rclcpp::Time& time_stamp);

 private:
  std::string bag_path_;
  std::vector<std::string> topics_;
  ImageSaveType image_save_type_;
  PCDSaveType pcd_save_type_;
  TimeStampSaveType time_save_type_;

  std::ofstream f_lidar, f_imu, f_img_0, f_img_1;  // for save timestamps
  PointCloud_ptr cloud_curr_, cloud_global_;
  int left_id_ = 0, right_id_ = 0, cloud_id_ = 0;
  cv::VideoWriter left_writer_, right_writer_;
  bool is_compressed_ = false;
  bool is_set_video_format_ = false;
  int w_, h_, fps_;

  std::shared_ptr<BagReader> bag_reader_;
  std::string img_dir0_, img_dir1_, pcd_dir_, time_stamp_dir_;
  cv::Mat left_img_, right_img_;
};
}  // namespace meskernel