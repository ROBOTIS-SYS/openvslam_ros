#ifndef OPENVSLAM_ROS_H
#define OPENVSLAM_ROS_H

#include <openvslam/system.h>
#include <openvslam/config.h>
#include <openvslam/util/stereo_rectifier.h>
// #include <openvslam/publish/frame_publisher.h>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <opencv2/core/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace openvslam_ros {
class system {
public:
    system();
    ~system();
    void publish_pose(const Eigen::Matrix4d& cam_pose_wc, const rclcpp::Time& stamp);
    void setParams();
    std::shared_ptr<openvslam::system> SLAM_;
    std::shared_ptr<openvslam::config> cfg_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::executors::SingleThreadedExecutor exec_;
    rmw_qos_profile_t custom_qos_;
    cv::Mat mask_;
    std::vector<double> track_times_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pose_pub_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>>
        init_pose_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> map_to_odom_broadcaster_;
    std::string odom_frame_, map_frame_, base_link_, camera_link_;
    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    bool publish_tf_;
    double transform_tolerance_;

    // newly added
    std::string cfg_file_path_;
    std::string vocab_file_path_;
    std::string mask_img_path_;
    bool enable_debug_;
    bool eval_log_;
    std::string map_db_path_;

    // newly added for the ros topic visualization
    std::string mat_type2encoding(int mat_type);
    void publish_debug_img(const std::shared_ptr<openvslam::publish::frame_publisher>& frame_publisher);
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> debug_img_pub_;
    const std::shared_ptr<openvslam::publish::frame_publisher> frame_publisher_;

private:
    void init_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

};

class mono : public system {
public:
    mono();
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    image_transport::Subscriber sub_;
};

class stereo : public system {
public:
    stereo();
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& left, const sensor_msgs::msg::Image::ConstSharedPtr& right);

    std::shared_ptr<openvslam::util::stereo_rectifier> rectifier_;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sf_, right_sf_;
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_;
};

class rgbd : public system {
public:
    rgbd();
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& color, const sensor_msgs::msg::Image::ConstSharedPtr& depth);

    message_filters::Subscriber<sensor_msgs::msg::Image> color_sf_, depth_sf_;
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_;
};

} // namespace openvslam_ros

#endif // OPENVSLAM_ROS_H
