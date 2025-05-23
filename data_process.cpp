
#include <cv_bridge/cv_bridge.h>
#include "data_process.h"

namespace meskernel {
DataProcess::DataProcess(const std::string& bag_file,
                         const std::vector<std::string>& topics,
                         ImageSaveType img_save_type, PCDSaveType pcd_save_type,
                         TimeStampSaveType time_save_type)
    : bag_path_(bag_file),
      topics_(topics),
      image_save_type_(img_save_type),
      pcd_save_type_(pcd_save_type),
      time_save_type_(time_save_type) {

  img_dir0_ = bag_path_ + "/left_image";
  img_dir1_ = bag_path_ + "/right_image";
  pcd_dir_ = bag_path_ + "/pcd";
  time_stamp_dir_ = bag_path_ + "/time_stamp";
}

void DataProcess::init() {

  bag_reader_.reset(new BagReader());

  if (pcd_save_type_ != KNotSavePCD) {
    cloud_curr_ = pcl::make_shared<PointCloud>();
    cloud_global_ = pcl::make_shared<PointCloud>();
    if (pcd_save_type_ == KSaveSinglePCD) {
      createFolder(pcd_dir_);
    }
  }

  if (image_save_type_ == KSaveVideo || image_save_type_ == KSaveCompressedVideo) {
    if (is_set_video_format_) {
      int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
      left_writer_.open(bag_path_ + "/left.mp4", fourcc, fps_,
                        cv::Size(w_, h_));
      right_writer_.open(bag_path_ + "/right.mp4", fourcc, fps_,
                         cv::Size(w_, h_));
    }
  }

  if (image_save_type_ == KSaveImgs ||
      image_save_type_ == KSaveCompressedImgs) {
    createFolder(img_dir0_);
    createFolder(img_dir1_);
  }

  if (time_save_type_ != KNotSaveTime) {

    createFolder(time_stamp_dir_);
    f_imu.open(time_stamp_dir_ + "/imu.txt");
    f_lidar.open(time_stamp_dir_ + "/lidar.txt");
    f_img_0.open(time_stamp_dir_ + "/img_l.txt");
    f_img_1.open(time_stamp_dir_ + "/img_r.txt");

    f_lidar << std::fixed << std::setprecision(6);
    f_imu << std::fixed << std::setprecision(6);
    f_img_0 << std::fixed << std::setprecision(6);
    f_img_1 << std::fixed << std::setprecision(6);
  }

  if (image_save_type_ > 4) {
    is_compressed_ = true;
    bag_reader_->registerCompressedImageCallback0(std::bind(
        &DataProcess::compressedImgDataCallback0, this, std::placeholders::_1));

    bag_reader_->registerCompressedImageCallback1(std::bind(
        &DataProcess::compressedImgDataCallback1, this, std::placeholders::_1));


  } else if (image_save_type_ > 0 && image_save_type_ < 4)

  {
    bag_reader_->registerImageCallback0(
        std::bind(&DataProcess::imgDataCallback0, this, std::placeholders::_1));

    bag_reader_->registerImageCallback1(
        std::bind(&DataProcess::imgDataCallback1, this, std::placeholders::_1));


  }

  bag_reader_->registerImuCallback(
      std::bind(&DataProcess::imuDataCallback, this, std::placeholders::_1));
  bag_reader_->registerLivoxCallback(
      std::bind(&DataProcess::lidarDataCallback, this, std::placeholders::_1));

  bag_reader_->init(bag_path_, topics_, is_compressed_);
}


void DataProcess::stop()
{
  bag_reader_->stop();
}
void DataProcess::start() {
  init();

  bag_reader_->processLoop();

  if(time_save_type_ != KNotSaveTime) {
    f_lidar.close();
    f_imu.close(); 
    f_img_0.close();
    f_img_1.close();
  }

   if (pcd_save_type_ == KSaveGlobalPCD) {
    pcl::io::savePCDFileBinary(bag_path_+"global.pcd",*cloud_global_);
   }

    if(left_writer_.isOpened()) {
      left_writer_.release();
    }

     if(right_writer_.isOpened()) {
      right_writer_.release();
    }

}
void DataProcess::imuDataCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  if (time_save_type_ == KSaveTime) {
    rclcpp::Time time_stamp = msg->header.stamp;
    f_imu << time_stamp.seconds() << std::endl;
  }
}

void DataProcess::imgDataCallback0(
    sensor_msgs::msg::Image::ConstSharedPtr msg) {
  if (time_save_type_ == KSaveTime) {
    rclcpp::Time time_stamp = msg->header.stamp;
    f_img_0 << time_stamp.seconds() << std::endl;
  }
  cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  if (image_save_type_ == KSaveSingleImg) {
    if (left_img_.empty()) {
      left_img_ = cv_ptr->image;
      std::string save_file = bag_path_ + "/left.png";
      cv::imwrite(save_file, left_img_);
    }
  } else if (image_save_type_ == KSaveImgs) {
    std::string path = img_dir0_ + "/" + std::to_string(left_id_) + ".png";
    cv::imwrite(path, cv_ptr->image);
    left_id_++;
  } else if (image_save_type_ == KSaveVideo) {
    if (is_set_video_format_) {
      left_writer_.write(cv_ptr->image);
    }
  }
}

void DataProcess::imgDataCallback1(
    sensor_msgs::msg::Image::ConstSharedPtr msg) {
  if (time_save_type_ == KSaveTime) {
    rclcpp::Time time_stamp = msg->header.stamp;
    f_img_1 << time_stamp.seconds() << std::endl;
  }

  cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  if (image_save_type_ == KSaveSingleImg) {
    if (right_img_.empty()) {
      right_img_ = cv_ptr->image;
      std::string save_file = bag_path_ + "/right.png";
      cv::imwrite(save_file, right_img_);
    }
  } else if (image_save_type_ == KSaveImgs) {
    std::string path = img_dir1_ + "/" + std::to_string(right_id_) + ".png";
    cv::imwrite(path, cv_ptr->image);
    right_id_++;
  } else if (image_save_type_ == KSaveVideo) {
    if (is_set_video_format_) {
      right_writer_.write(cv_ptr->image);
    }
  }
}

void DataProcess::lidarDataCallback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  if (time_save_type_ == KSaveTime) {
    rclcpp::Time time_stamp = msg->header.stamp;
    f_lidar << time_stamp.seconds() << std::endl;
  }

  cloud_curr_ = pcl::make_shared<PointCloud>();
  pcl::fromROSMsg(*msg, *cloud_curr_);

  if (pcd_save_type_ == KSaveGlobalPCD) {
    *cloud_global_ += *cloud_curr_;
  } else if (pcd_save_type_ == KSaveSinglePCD) {
    std::string path = pcd_dir_ + "/" + std::to_string(cloud_id_) + ".pcd";
    pcl::io::savePCDFileBinary(path, *cloud_curr_);
    cloud_id_++;
  }
}

void DataProcess::compressedImgDataCallback0(
    sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {

  if (time_save_type_ == KSaveTime) {
    rclcpp::Time time_stamp = msg->header.stamp;
    f_img_0 << time_stamp.seconds() << std::endl;
  }

  if (image_save_type_ == KSaveCompressedSingleImg) {

    if (left_img_.empty()) {

      left_img_ = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
      std::string save_file = bag_path_ + "/left.png";
      cv::imwrite(save_file, left_img_);
    }
  } else if (image_save_type_ == KSaveCompressedImgs) {
    left_img_ = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    std::string path = img_dir0_ + "/" + std::to_string(left_id_) + ".png";
    cv::imwrite(path, left_img_);
    left_id_++;
  } else if (image_save_type_ == KSaveCompressedVideo) {
    if (is_set_video_format_) {
      left_img_ = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
      left_writer_.write(left_img_);
    }
  }
}



void DataProcess::compressedImgDataCallback1(
    sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {

  if (time_save_type_ == KSaveTime) {
    rclcpp::Time time_stamp = msg->header.stamp;
    f_img_1 << time_stamp.seconds() << std::endl;
  }

  if (image_save_type_ == KSaveCompressedSingleImg) {

    if (right_img_.empty()) {

      right_img_ = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
      std::string save_file = bag_path_ + "/right.png";
      cv::imwrite(save_file, right_img_);
    }
  } else if (image_save_type_ == KSaveCompressedImgs) {
    right_img_ = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    std::string path = img_dir1_ + "/" + std::to_string(right_id_) + ".png";
    cv::imwrite(path, right_img_);
    right_id_++;
  } else if (image_save_type_ == KSaveCompressedVideo) {
    if (is_set_video_format_) {
      right_img_ = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
      right_writer_.write(right_img_);
    }
  }
}

void DataProcess::createFolder(const std::string& path) {

  if (!std::filesystem::exists(path)) {
    std::string cmd_mkdir_result_dir = "mkdir -p " + path;
    int cmd_status = system(cmd_mkdir_result_dir.c_str());
    if (cmd_status != 0) {
      std::cerr << cmd_mkdir_result_dir << " failed." << std::endl;
    }
  }
}

void DataProcess::set_video_format(int w, int h, int fps) {
  w_ = w, h_ = h, fps_ = fps;
  is_set_video_format_ = true;
}
}  // namespace meskernel