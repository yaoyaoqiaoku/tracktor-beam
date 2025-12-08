#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class ArucoTrackerNode : public rclcpp::Node
{
public:
	ArucoTrackerNode();

private:
	void loadParameters();

	void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
	void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
	void annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target, int id);

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_large_pub;
	// 新增：小码位姿发布者
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_small_pub;

	std::unique_ptr<cv::aruco::ArucoDetector> _detector;
	cv::Mat _camera_matrix;
	cv::Mat _dist_coeffs;

    // 新增：双二维码参数
    int _param_aruco_id_large;   // 大码ID（原_param_aruco_id复用）
    int _param_aruco_id_small;   // 小码ID（新增)
	int _param_dictionary;
    double _param_marker_size_large; // 大码尺寸（原参数复用）
    double _param_marker_size_small; // 小码尺寸（新增）
};

