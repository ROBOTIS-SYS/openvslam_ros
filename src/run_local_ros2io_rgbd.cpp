#ifdef USE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#elif USE_SOCKET_PUBLISHER
#include <socket_publisher/publisher.h>
#endif

#include <openvslam/system.h>
#include <openvslam/config.h>
#include <openvslam/util/yaml.h>
#include <openvslam_ros2io.hpp>

#include <iostream>
#include <chrono>
#include <numeric>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif

void localization() {
    std::shared_ptr<openvslam_ros::system> ros;

        ros = std::make_shared<openvslam_ros::rgbd>();

    auto& SLAM = *(ros->SLAM_);
    // load the prebuilt map
    RCLCPP_WARN(ros->node_->get_logger(),"ORB_vocab_file_path: %s", ros->vocab_file_path_.c_str());
    RCLCPP_WARN(ros->node_->get_logger(),"Map database path: %s", ros->map_db_path_.c_str());
    SLAM.load_map_database(ros->map_db_path_);
    // startup the SLAM process (it does not need initialization of a map)
    bool mapping = false;
    SLAM.startup(false);
    // select to activate the mapping module or not
    if (mapping) {
        SLAM.enable_mapping_module();
    }
    else {
        SLAM.disable_mapping_module();
    }

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
    rclcpp::uninstall_signal_handlers();

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStart("slam.prof");
#endif

    localization();

#ifdef USE_GOOGLE_PERFTOOLS
    ProfilerStop();
#endif

    return EXIT_SUCCESS;
}
