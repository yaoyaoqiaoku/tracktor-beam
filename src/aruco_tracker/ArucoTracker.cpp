#include "ArucoTracker.hpp"		// 包含ArucoTrackerNode类的声明
#include <sstream>		// 用于字符串流操作

// 构造函数：初始化节点，加载参数，创建检测器、订阅者和发布者
ArucoTrackerNode::ArucoTrackerNode()
	: Node("aruco_tracker_node")		// 初始化ROS 2节点，节点名为"aruco_tracker_node"
{
	loadParameters();	// 加载参数

	// 初始化ArUco检测器参数（当前使用默认参数，可根据需求调整）
	// See: https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html
	auto detectorParams = cv::aruco::DetectorParameters();

	// 加载预定义的ArUco字典（字典决定了标记的样式，如4x4格子的250个标记）
	// See: https://docs.opencv.org/4.x/d1/d21/aruco__dictionary_8hpp.html
	auto dictionary = cv::aruco::getPredefinedDictionary(_param_dictionary);

	// 创建ArUco检测器实例（用字典和参数初始化）
	_detector = std::make_unique<cv::aruco::ArucoDetector>(dictionary, detectorParams);

	// 设置QoS（服务质量）：图像传输适合用"best_effort"（允许丢包，优先实时性）
	auto qos = rclcpp::QoS(1).best_effort();

	// 创建图像订阅者：订阅"/camera"话题的图像消息，回调函数为image_callback
	_image_sub = create_subscription<sensor_msgs::msg::Image>(
			     "/camera", qos, std::bind(&ArucoTrackerNode::image_callback, this, std::placeholders::_1));

	// 创建相机信息订阅者：订阅"/camera_info"话题的内参消息，回调函数为camera_info_callback
	_camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
				   "/camera_info", qos, std::bind(&ArucoTrackerNode::camera_info_callback, this, std::placeholders::_1));

	// 创建发布者：发布处理后的图像（带标记标注）到"/image_proc"话题
	_image_pub = create_publisher<sensor_msgs::msg::Image>("/image_proc", qos);
	// 发布目标标记的位姿到"/target_pose"话题
	_target_pose_large_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose_large", qos);
	// 新增：创建小码位姿发布者
	_target_pose_small_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose_small", qos);
}

// 加载并初始化ROS参数（从参数服务器或启动文件中获取）
void ArucoTrackerNode::loadParameters()
{
	// 声明参数（若未设置，使用默认值）
	declare_parameter<int>("aruco_id_large", 49);	// 要跟踪的ArUco标记ID(大码)
	declare_parameter<int>("aruco_id_small", 50);    // 小码ID
	declare_parameter<int>("dictionary", 7); // ArUco字典类型（默认2对应DICT_4X4_250）
	declare_parameter<double>("marker_size_large", 0.25);	// 标记的实际尺寸（单位：米，默认0.5米）
	declare_parameter<double>("marker_size_small", 0.05);    // 小码尺寸
	//	整数参数值	           字典枚举名	                             含义
	//	   0	            DICT_4X4_50	                  4x4 格子，50 个标记（ID 0 - 49）
	//	   1				DICT_4X4_100				  4x4 格子，100 个标记（ID 0 - 99）
	//	   2	 			DICT_4X4_250				  4x4 格子，250 个标记（ID 0 - 249）
	//	   3	            DICT_4X4_1000	              4x4 格子，1000 个标记（ID 0 - 999）
	//	   4	            DICT_5X5_50	                  5x5 格子，50 个标记（ID 0 - 49）
	//	   5	            DICT_5X5_100	              5x5 格子，100 个标记（ID 0 - 99）
	//	   6	            DICT_5X5_250	              5x5 格子，250 个标记（ID 0 - 249）
	//	   7	            DICT_5X5_1000	              5x5 格子，1000 个标记（ID 0 - 999）
	//	   8	            DICT_6X6_50	                  6x6 格子，50 个标记（ID 0 - 49）
	//	   9	            DICT_6X6_100	              6x6 格子，100 个标记（ID 0 - 99）
	//	   10	            DICT_6X6_250	              6x6 格子，250 个标记（ID 0 - 249）
	//	   11	            DICT_6X6_1000	              6x6 格子，1000 个标记（ID 0 - 999）
	//	   12	            DICT_7X7_50	                  7x7 格子，50 个标记（ID 0 - 49）
	//	   13	            DICT_7X7_100	              7x7 格子，100 个标记（ID 0 - 99）
	//	   14	            DICT_7X7_250	              7x7 格子，250 个标记（ID 0 - 249）
	//	   15	            DICT_7X7_1000	              7x7 格子，1000 个标记（ID 0 - 999）
	//	   16	            DICT_ARUCO_ORIGINAL	          原始 ArUco 标记集，特定样式的标记
	//	   17	            DICT_APRILTAG_16H5	          兼容 AprilTag 16h5 类型标记
	//	   18	            DICT_APRILTAG_25H9	          兼容 AprilTag 25h9 类型标记
	//	   19	            DICT_APRILTAG_36H10	          兼容 AprilTag 36h10 类型标记
	//	   20	            DICT_APRILTAG_36H11	          兼容 AprilTag 36h11 类型标记 

	// 从参数服务器获取参数值，存储到成员变量 
	get_parameter("aruco_id_large", _param_aruco_id_large);
	get_parameter("aruco_id_small", _param_aruco_id_small);
	get_parameter("dictionary", _param_dictionary);
	get_parameter("marker_size_large", _param_marker_size_large);
	get_parameter("marker_size_small", _param_marker_size_small);
}

// 图像消息回调：处理相机图像，检测ArUco标记并计算位姿
void ArucoTrackerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	try {
		// 1. 将ROS图像消息转换为OpenCV格式（BGR8编码，适合OpenCV处理）
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		// 2. 检测图像中的ArUco标记
		std::vector<int> ids;		// 存储检测到的标记ID
		std::vector<std::vector<cv::Point2f>> corners;		// 存储每个标记的四个角点坐标（图像像素坐标）
		_detector->detectMarkers(cv_ptr->image, corners, ids);	// 执行检测
		cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);		// 在图像上绘制检测到的标记（边框+ID）

		// 3. 若已获取相机内参和畸变系数，继续计算位姿
		if (!_camera_matrix.empty() && !_dist_coeffs.empty()) {

			// 3.1 对标记角点进行去畸变（消除镜头畸变影响，提高位姿计算精度）
			std::vector<std::vector<cv::Point2f>> undistortedCorners;	// 去畸变后的角点
			for (const auto& corner : corners) {
				std::vector<cv::Point2f> undistortedCorner;
				// 去畸变函数：输入原始角点、相机内参、畸变系数，输出校正后的角点
				cv::undistortPoints(corner, undistortedCorner, _camera_matrix, _dist_coeffs, cv::noArray(), _camera_matrix);
				undistortedCorners.push_back(undistortedCorner);
			}

			// 3.2 遍历检测到的标记，找到目标ID（_param_aruco_id）并计算位姿
            for (size_t i = 0; i < ids.size(); i++) {
                double marker_size = 0.0;
                rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub = nullptr;

                // 匹配大码/小码，设置对应尺寸和发布者
                if (ids[i] == _param_aruco_id_large) {
                    marker_size = _param_marker_size_large;
                    pub = _target_pose_large_pub;
                } else if (ids[i] == _param_aruco_id_small) {
                    marker_size = _param_marker_size_small;
                    pub = _target_pose_small_pub;
                } else {
                    continue; // 忽略其他ID
                }

				// 3.3 定义标记的3D模型坐标（以标记中心为原点，右手坐标系）
				float half_size = marker_size / 2.0f;	// 标记边长的一半
				std::vector<cv::Point3f> objectPoints = {
					cv::Point3f(-half_size,  half_size, 0),  // 左上角（相对于中心）
					cv::Point3f(half_size,  half_size, 0),   // 右上角
					cv::Point3f(half_size, -half_size, 0),   // 右下角
					cv::Point3f(-half_size, -half_size, 0)   // 左下角
				};

				// 3.4 使用PnP算法（ Perspective-n-Point ）计算位姿
				// 输入：3D模型点、去畸变后的2D图像点、相机内参；输出：旋转向量（rvec）和平移向量（tvec）
				cv::Vec3d rvec, tvec;	// rvec：旋转向量（轴角表示）；tvec：平移向量（单位：米，相机坐标系下）
				cv::solvePnP(objectPoints, undistortedCorners[i], _camera_matrix, cv::noArray(), rvec, tvec);
				// 3.5 在图像上绘制坐标轴（可视化标记的位姿）
				// 参数：图像、相机内参、旋转向量、平移向量、坐标轴长度（用标记尺寸作为参考）
				cv::drawFrameAxes(cv_ptr->image, _camera_matrix, cv::noArray(), rvec, tvec, marker_size);

				// 3.6 将旋转向量转换为四元数（ROS中常用四元数表示方向）
				cv::Mat rot_mat;	// 旋转矩阵（3x3）
				cv::Rodrigues(rvec, rot_mat);	// 旋转向量→旋转矩阵（罗德里格斯变换）
				cv::Quatd quat = cv::Quatd::createFromRotMat(rot_mat).normalize();	// 旋转矩阵→四元数并归一化

				// 3.7 构造位姿消息并发布
				geometry_msgs::msg::PoseStamped pose_msg;
				pose_msg.header.stamp = msg->header.stamp;	// 时间戳与图像一致
				pose_msg.header.frame_id = "camera_frame";	// 位姿的参考坐标系（相机坐标系）
				// 平移分量（x,y,z：标记中心在相机坐标系下的位置，单位：米）
				pose_msg.pose.position.x = tvec[0];
				pose_msg.pose.position.y = tvec[1];
				pose_msg.pose.position.z = tvec[2];
				// 旋转分量（四元数：标记相对于相机的姿态）
				pose_msg.pose.orientation.x = quat.x;
				pose_msg.pose.orientation.y = quat.y;
				pose_msg.pose.orientation.z = quat.z;
				pose_msg.pose.orientation.w = quat.w;

				pub->publish(pose_msg);	// 发布位姿消息

				// 3.8 在图像上标注位姿信息（x,y,z坐标）
				annotate_image(cv_ptr, tvec, ids[i]);
			}

		} else {
			// 若未获取相机内参，输出错误日志
			RCLCPP_ERROR(get_logger(), "Missing camera calibration");
		}

		// 4. 发布处理后的图像（带标记和坐标轴标注）
		cv_bridge::CvImage out_msg;
		out_msg.header = msg->header;	// 保持原图像的时间戳和坐标系
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;	// 编码格式
		out_msg.image = cv_ptr->image;	// 处理后的图像
		_image_pub->publish(*out_msg.toImageMsg().get());	// 转换为ROS消息并发布

	} catch (const cv_bridge::Exception& e) {
		// 捕获cv_bridge转换异常并输出日志
		RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
	}
}

// 相机信息回调：获取相机内参和畸变系数（用于后续位姿计算）
void ArucoTrackerNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
	// 从CameraInfo消息中提取相机内参矩阵（K）和畸变系数（d），并深拷贝到OpenCV的Mat中
	// 相机内参矩阵K是3x3矩阵：[fx, 0, cx; 0, fy, cy; 0, 0, 1]，其中fx/fy是焦距，cx/cy是主点坐标
	_camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();   // 畸变系数d是向量（通常5个参数：k1,k2,p1,p2,k3），用于校正镜头畸变
	_dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();   // Use clone to ensure a deep copy

	// 打印相机内参矩阵（用于调试，确认参数是否正确）
	RCLCPP_INFO(get_logger(), "Camera matrix updated:\n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
		    _camera_matrix.at<double>(0, 0), _camera_matrix.at<double>(0, 1), _camera_matrix.at<double>(0, 2),
		    _camera_matrix.at<double>(1, 0), _camera_matrix.at<double>(1, 1), _camera_matrix.at<double>(1, 2),
		    _camera_matrix.at<double>(2, 0), _camera_matrix.at<double>(2, 1), _camera_matrix.at<double>(2, 2));
	RCLCPP_INFO(get_logger(), "Camera Matrix: fx=%f, fy=%f, cx=%f, cy=%f",
		    _camera_matrix.at<double>(0, 0), // fx：x方向焦距
		    _camera_matrix.at<double>(1, 1), // fy：y方向焦距
		    _camera_matrix.at<double>(0, 2), // cx：图像x方向主点（光轴与图像平面交点）
		    _camera_matrix.at<double>(1, 2)  // cy：图像y方向主点
		   );

	// 检查焦距是否有效（若为0则异常）
	if (_camera_matrix.at<double>(0, 0) == 0) {
		RCLCPP_ERROR(get_logger(), "Focal length is zero after update!");

	} else {
		RCLCPP_INFO(get_logger(), "Updated camera intrinsics from camera_info topic.");
		// 成功获取内参后，取消订阅相机信息话题（只需获取一次）
		RCLCPP_INFO(get_logger(), "Unsubscribing from camera info topic");
		_camera_info_sub.reset();	// 取消订阅，释放资源
	}
}

// 在图像上标注目标的位置信息（x,y,z坐标）
void ArucoTrackerNode::annotate_image(cv_bridge::CvImagePtr image, const cv::Vec3d& target, int id)
{
	// 格式化文本：保留2位小数，显示x,y,z坐标
	std::ostringstream stream;
	stream << std::fixed << std::setprecision(2);
	stream << "ID:" << id << " X:" << target[0] << " Y:" << target[1] << " Z:" << target[2];
	std::string text = stream.str();

	// 设置文本属性（字体、大小、粗细等）
	int fontFace = cv::FONT_HERSHEY_SIMPLEX;	// 字体类型
	double fontScale = 0.8;		// 字体缩放比例
	int thickness = 2;		// 字体粗细
	// 计算文本尺寸（用于确定绘制位置）
    cv::Point textOrg(10, id == _param_aruco_id_large ? 30 : 60); // 大码在上，小码在下
    cv::Scalar color = (id == _param_aruco_id_large) ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 255, 0); // 大码黄色，小码绿色
    cv::putText(image->image, text, textOrg, fontFace, fontScale, color, thickness, 8);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoTrackerNode>());
	rclcpp::shutdown();
	return 0;
}
