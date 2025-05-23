#include "data_process.h"
const std::string lidar_topic = "/livox/lidar";
const std::string imu_topic = "/livox/imu";
const std::string img_topic0 = "/camera/left";
const std::string img_topic1 = "/camera/right";
const std::string compressed_img_topic0 = "/compressed_camera/left";
const std::string compressed_img_topic1 = "/compressed_camera/right";

std::shared_ptr<meskernel::DataProcess> process;

void CtrlCHandler(int signum)
{
  process->stop();
}
int main(int argc, char** argv) {

  if (argc < 2) {
    std::cout << "Useage rosrun package_name mid360_save_pcd_node "
                 "save_dir(string)\n";
    return 0;
  }

  std::string data_dir = std::string(argv[1]);
  signal(SIGINT, CtrlCHandler);
  rclcpp::init(argc, argv);

  std::vector<std::string> topics{lidar_topic, imu_topic,
                                  img_topic1};

  process =
      std::make_shared<meskernel::DataProcess>(
          data_dir, topics, meskernel::ImageSaveType::KNotSaveImg,
          meskernel::PCDSaveType::KSaveGlobalPCD,
          meskernel::TimeStampSaveType::KNotSaveTime);

  process->start();

  return 0;
}
