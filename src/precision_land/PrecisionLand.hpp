#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>		// 用于发送命令给飞控，新增
#include <px4_msgs/msg/vehicle_status.hpp>    // 用于获取飞控状态，新增
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>  // 用于距离传感器数据

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <vector>
#include <limits> // 包含std::numeric_limits所需
#include <mutex>  // 新增：互斥锁头文件
#include <deque>   // 新增：双端队列头文件

// 新增常量定义（参考模板，确保与PX4一致）
const uint16_t VEHICLE_CMD_NAV_LAND = 21; // 降落命令ID

class PrecisionLand : public px4_ros2::ModeBase
{
public:
	explicit PrecisionLand(rclcpp::Node& node);

	// 回调函数
	void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);	//大码位姿回调函数
	// 新增：小码位姿回调函数
	void targetPoseSmallCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

	void laserHeightCallback(const px4_msgs::msg::DistanceSensor::SharedPtr msg);  // 激光高度回调函数

	// 模式生命周期函数
	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	struct ArucoTag {
		// Initialize position with NaN values directly in the struct
		Eigen::Vector3d position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
		Eigen::Quaterniond orientation;
		rclcpp::Time timestamp;

		// 修正：valid() 应判断位置是否有效（而非时间戳），与cpp中逻辑一致
		bool valid() const { 
			return !std::isnan(position.x()) && !std::isnan(position.y()) && !std::isnan(position.z()); 
		};
	};

	// 核心修复：函数声明中设置默认参数（.cpp中不再重复）
    void send_vehicle_command(uint16_t command, 
                             float param1 = 0.0f, 
                             float param2 = 0.0f,
                             float param3 = 0.0f, 
                             float param4 = 0.0f, 
                             double param5 = 0.0,
                             double param6 = 0.0, 
                             float param7 = 0.0f);
    void execute_land();

	// 参数加载
	void loadParameters();

	// 坐标系转换：将 tag 位姿从相机光学坐标系转换到世界坐标系（NED）
	ArucoTag getTagWorld(const ArucoTag& tag);

	Eigen::Vector2f calculateVelocitySetpointXY(const ArucoTag& current_tag);	// 计算XY速度设定值,传入当前 tag
	bool checkTargetTimeout(const ArucoTag& tag);	// 传入 tag，检查是否超时
	bool positionReached(const Eigen::Vector3f& target) const;

	double fuseHeight(double visual_height, double laser_height);	// 融合视觉高度和激光高度

	enum class State {
		Idle,		// 空闲状态
		Search, 	// 搜索目标
		Approach, 	// 接近目标，保持高度
		Descend, 	// 下降
		Finished	// 降落完成
	};

	void switchToState(State state);
	std::string stateName(State state);

	// 新增：双二维码参数
    int _param_aruco_id_large = 49;
    int _param_aruco_id_small = 50;
    double _height_threshold_switch = 2.0; // 切换高度：2米（大码→小码）
    double _height_threshold_recover = 2.0; // 小码丢失后回升上限
    double _height_threshold_low = 0.2;    // 最终降落高度
	double _land_mode_switch_height = 2.0;       // 允许切换到Land模式的最低高度

	// 新增：高度融合参数
	double _param_visual_weight = 0.5;    // 视觉高度权重
	double _param_laser_weight = 0.5;     // 激光高度权重
	double _laser_min_valid_distance = 0.02; // 激光最小有效距离
	double _laser_max_valid_distance = 12.0; // 激光最大有效距离
	double _fused_height = 0.0;           // 融合后的高度
	double _last_laser_height = std::numeric_limits<double>::quiet_NaN();     // 上一帧激光高度
	bool _laser_data_valid = false;       // 激光数据有效性标志

	// ros2
	rclcpp::Node& _node;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_large_sub;	// 大码位姿订阅和存储
	// 新增：小码位姿订阅和存储
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_small_sub;	// 小码位姿订阅和存储
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;		// 降落检测订阅
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_world_pub;	// 发布世界系下的 tag 位姿
	// 核心修复：自定义VehicleStatus订阅者（不使用父类私有成员）
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub_custom; // 自定义订阅者
	rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr _laser_height_sub;		// 激光高度订阅
	px4_msgs::msg::VehicleStatus current_vehicle_status_; // 存储PX4状态
    std::mutex state_mutex_; // 保护状态变量的互斥锁
	std::mutex height_mutex_; // 保护高度数据的互斥锁
	std::deque<double> _laser_height_buffer; // 激光高度缓冲区
	const size_t _laser_buffer_size = 5; // 激光高度缓冲区大小
	double _last_debug_print_time;        // 上一次打印调试信息的时间
	
	// px4接口对象
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;	// 本地位置接口
	std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;		// 姿态接口
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;		// 轨迹 setpoint 接口

	// Data
	State _state = State::Search;
	bool _search_started = false;

	//二维码数据（世界系）
	ArucoTag _tag_large;	// 大码世界系数据
	ArucoTag _tag_small;	// 小码世界系数据

	float _approach_altitude = {};

	// Land detection
	bool _land_detected = false;
	bool _target_lost_prev = true;
	// 新增：状态变量
    bool _small_tag_lost = false;
    rclcpp::Time _small_tag_lost_time;
    double _param_small_tag_recover_timeout = 5.0; // 小码丢失后回升等待时间
	
	// Waypoints for Search pattern
	std::vector<Eigen::Vector3f> _search_waypoints;
	// Search pattern generation
	void generateSearchWaypoints();
	// Search pattern index
	int _search_waypoint_index = 0;

	// Parameters
	float _param_descent_vel = {};
	float _param_vel_p_gain = {};
	float _param_vel_i_gain = {};
	float _param_max_velocity = {};
	float _param_target_timeout = {};
	float _param_delta_position = {};
	float _param_delta_velocity = {};

	float _vel_x_integral {};
	float _vel_y_integral {};
};

