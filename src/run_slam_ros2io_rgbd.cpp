#ifdef USE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#elif USE_SOCKET_PUBLISHER
#include <socket_publisher/publisher.h>
#endif

#include <openvslam/system.h>
#include <openvslam/config.h>
#include <openvslam/util/yaml.h>
#include <openvslam_ros2io.hpp>

#include <chrono>

#include <numeric>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <popl.hpp>

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

void tracking()
{
  std::shared_ptr<openvslam_ros::system> ros;

  ros = std::make_shared<openvslam_ros::rgbd>();

  auto& SLAM = *(ros->SLAM_);

  // startup the SLAM process
  SLAM.startup();

  rclcpp::Rate rate(50);
  ros->setParams();
  while (rclcpp::ok()) {
    ros->exec_.spin_some();
    rate.sleep();
  }

  // shutdown the SLAM process
  SLAM.shutdown();

}

int main(int argc, char* argv[]) {

#ifdef USE_STACK_TRACE_LOGGER
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
#endif

  rclcpp::init(argc, argv);

  // run tracking

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

  tracking();

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

  return EXIT_SUCCESS;
}
