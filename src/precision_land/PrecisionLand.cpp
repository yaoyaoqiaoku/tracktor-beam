#include "PrecisionLand.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

// -------------------------- 自定义模式定义 ------------------------
// 模式名称：用于PX4识别该自定义模式
static const std::string kModeName = "PrecisionLandCustom";
// 调试输出开关：true表示启用调试日志
static const bool kEnableDebugOutput = true;

// 引入px4_ros2命名空间的字面量（简化单位相关代码，如时间单位等）
using namespace px4_ros2::literals;

PrecisionLand::PrecisionLand(rclcpp::Node& node)
	: ModeBase(node, kModeName)	// 继承ModeBase基类，传入节点和模式名称
	, _node(node)
	, _last_debug_print_time(0.0)  // 新增：初始化调试打印时间戳
//-------------------------- 自定义模式定义 ------------------------

{
	// 创建 setpoint / 状态封装对象（与 px4_ros2 框架交互）
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

	// 订阅视觉目标（Aruco）位姿，QoS 使用 best_effort（丢包可接受）,订阅大码位姿
	_target_pose_large_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose_large",
			   rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::targetPoseCallback, this, std::placeholders::_1));

	// 新增：订阅小码位姿
    _target_pose_small_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose_small",
               rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::targetPoseSmallCallback, this, std::placeholders::_1));

	// 新增：创建vehicle_command发布者（QoS使用默认可靠传输，确保命令被PX4接收）
    _vehicle_command_pub = _node.create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command",
				rclcpp::QoS(10));
    			RCLCPP_INFO(_node.get_logger(), "成功创建vehicle_command发布者");

	// 核心修复：订阅PX4状态（使用自定义订阅者_vehicle_status_sub_custom）
	rclcpp::QoS px4_qos(10);
	px4_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
	px4_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

	_vehicle_status_sub_custom = _node.create_subscription<px4_msgs::msg::VehicleStatus>(
		"/fmu/out/vehicle_status",
		px4_qos,
		[this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
			std::lock_guard<std::mutex> lock(state_mutex_);
			current_vehicle_status_ = *msg;
		}
	);

	// 订阅飞控的降落检测信息
	_vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",
				     rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::vehicleLandDetectedCallback, this, std::placeholders::_1));

	// 新增：订阅激光高度数据
	// 新增：订阅激光高度话题
	_laser_height_sub = _node.create_subscription<px4_msgs::msg::DistanceSensor>(
		"/fmu/out/distance_sensor",
		rclcpp::QoS(10).best_effort(),
		std::bind(&PrecisionLand::laserHeightCallback, this, std::placeholders::_1)
	);
	if (_laser_height_sub) { // 判断订阅者是否创建成功
	RCLCPP_INFO(_node.get_logger(), "成功订阅激光高度话题 /fmu/out/distance_sensor");
		} else {
			RCLCPP_ERROR(_node.get_logger(), "激光高度话题订阅失败！");
		}

	// 发布 world-frame 的 tag pose（用于可视化）
	_target_pose_world_pub = _node.create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose_world", rclcpp::QoS(1).best_effort());

	// 读取参数（默认值在 loadParameters 中声明）
	loadParameters();

	// 指示该模式不需要手柄（manual_control）
	modeRequirements().manual_control = false;
}

void PrecisionLand::loadParameters()
{
	// 声明参数并设置默认值
	_node.declare_parameter<float>("descent_vel", 1.0);		//下降速度
	_node.declare_parameter<float>("vel_p_gain", 1.5);		//速度P增益
	_node.declare_parameter<float>("vel_i_gain", 0.0);		//速度I增益
	_node.declare_parameter<float>("max_velocity", 3.0);		//最大速度
	_node.declare_parameter<float>("target_timeout", 3.0);		//目标超时
	_node.declare_parameter<float>("delta_position", 0.25);		//位置到达阈值
	_node.declare_parameter<float>("delta_velocity", 0.25);		//速度到达阈值
	// 新增参数声明（可选，从配置文件读取）
    _node.declare_parameter<int>("aruco_id_large", 49);
    _node.declare_parameter<int>("aruco_id_small", 50);
    _node.declare_parameter<double>("height_threshold_switch", 2.0);
    _node.declare_parameter<double>("small_tag_recover_timeout", 5.0);
	_node.declare_parameter<double>("height_threshold_low", 0.2);      // 最终精控高度
    _node.declare_parameter<double>("height_threshold_recover", 2.5);  // 小码丢失回升高度
	_node.declare_parameter<double>("land_mode_switch_height", 2.0); // 新增参数声明
	// 新增：高度融合参数声明
	_node.declare_parameter<double>("param_visual_weight", 0.5);    // 视觉高度权重
	_node.declare_parameter<double>("param_laser_weight", 0.5);     // 激光高度权重

	// 读取参数到成员变量
	_node.get_parameter("descent_vel", _param_descent_vel);
	_node.get_parameter("vel_p_gain", _param_vel_p_gain);
	_node.get_parameter("vel_i_gain", _param_vel_i_gain);
	_node.get_parameter("max_velocity", _param_max_velocity);
	_node.get_parameter("target_timeout", _param_target_timeout);
	_node.get_parameter("delta_position", _param_delta_position);
	_node.get_parameter("delta_velocity", _param_delta_velocity);
	// 新增：读取双码参数
	_node.get_parameter("aruco_id_large", _param_aruco_id_large);
	_node.get_parameter("aruco_id_small", _param_aruco_id_small);
	_node.get_parameter("height_threshold_switch", _height_threshold_switch);
	_node.get_parameter("small_tag_recover_timeout", _param_small_tag_recover_timeout);
	_node.get_parameter("height_threshold_low", _height_threshold_low);
	_node.get_parameter("height_threshold_recover", _height_threshold_recover);
	_node.get_parameter("land_mode_switch_height", _land_mode_switch_height); // 新增参数读取
	// 新增：读取高度融合参数
	_node.get_parameter("param_visual_weight", _param_visual_weight);
	_node.get_parameter("param_laser_weight", _param_laser_weight);

	// 打印关键参数，方便调试
	RCLCPP_INFO(_node.get_logger(), "descent_vel: %f", _param_descent_vel);
	RCLCPP_INFO(_node.get_logger(), "vel_i_gain: %f", _param_vel_i_gain);
	RCLCPP_INFO(_node.get_logger(), "vel_p_gain: %f", _param_vel_p_gain);
	RCLCPP_INFO(_node.get_logger(), "max_velocity: %f", _param_max_velocity);
	RCLCPP_INFO(_node.get_logger(), "target_timeout: %f", _param_target_timeout);
	RCLCPP_INFO(_node.get_logger(), "delta_position: %f", _param_delta_position);
	RCLCPP_INFO(_node.get_logger(), "delta_velocity: %f", _param_delta_velocity);
	RCLCPP_INFO(_node.get_logger()," aruco_id_large: %d", _param_aruco_id_large);
	RCLCPP_INFO(_node.get_logger()," aruco_id_small: %d", _param_aruco_id_small);
	RCLCPP_INFO(_node.get_logger()," height_threshold_switch: %f", _height_threshold_switch);
	RCLCPP_INFO(_node.get_logger()," small_tag_recover_timeout: %f", _param_small_tag_recover_timeout);
	RCLCPP_INFO(_node.get_logger()," height_threshold_low: %f", _height_threshold_low);
	RCLCPP_INFO(_node.get_logger()," height_threshold_recover: %f", _height_threshold_recover);
	RCLCPP_INFO(_node.get_logger()," land_mode_switch_height: %f", _land_mode_switch_height);
	RCLCPP_INFO(_node.get_logger()," param_visual_weight: %f", _param_visual_weight);
	RCLCPP_INFO(_node.get_logger()," param_laser_weight: %f", _param_laser_weight);
}

void PrecisionLand::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	// 更新降落检测标志（来自飞控）
	_land_detected = msg->landed;
}

void PrecisionLand::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	// 只有当搜索已开始时才处理视觉消息，防止激活前误处理
	if (_search_started) {
		auto tag = ArucoTag {
			.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
			// 注意 PoseStamped 的四元数顺序：w, x, y, z
			.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
			.timestamp = _node.now(),
		};

		// 将 tag（光学相机坐标）转换为 world（NED）坐标
		_tag_large = getTagWorld(tag);

		// 发布世界系下的 tag pose 用于可视化（frame_id="map"）
		geometry_msgs::msg::PoseStamped world_pose_msg;
		world_pose_msg.header.stamp = _node.now();
		world_pose_msg.header.frame_id = "map";
		world_pose_msg.pose.position.x = tag.position.x();
		world_pose_msg.pose.position.y = tag.position.y();
		world_pose_msg.pose.position.z = tag.position.z();
		world_pose_msg.pose.orientation.w = tag.orientation.w();
		world_pose_msg.pose.orientation.x = tag.orientation.x();
		world_pose_msg.pose.orientation.y = tag.orientation.y();
		world_pose_msg.pose.orientation.z = tag.orientation.z();
		_target_pose_world_pub->publish(world_pose_msg);
	}

}

// 新增：小码位姿回调函数
void PrecisionLand::targetPoseSmallCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	// 只有当搜索已开始时才处理视觉消息，防止激活前误处理
	if (_search_started) {
		auto tag = ArucoTag {
			.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
			// 注意 PoseStamped 的四元数顺序：w, x, y, z
			.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
			.timestamp = _node.now(),
		};

		// 将 tag（光学相机坐标）转换为 world（NED）坐标
		_tag_small = getTagWorld(tag);
		// 发布世界系下的 tag pose 用于可视化（frame_id="map"）
		geometry_msgs::msg::PoseStamped world_pose_msg;
		world_pose_msg.header.stamp = _node.now();
		world_pose_msg.header.frame_id = "map";
		world_pose_msg.pose.position.x = _tag_small.position.x();
		world_pose_msg.pose.position.y = _tag_small.position.y();
		world_pose_msg.pose.position.z = _tag_small.position.z();
		world_pose_msg.pose.orientation.w = _tag_small.orientation.w();
		world_pose_msg.pose.orientation.x = _tag_small.orientation.x();
		world_pose_msg.pose.orientation.y = _tag_small.orientation.y();
		world_pose_msg.pose.orientation.z = _tag_small.orientation.z();
		_target_pose_world_pub->publish(world_pose_msg);

		//小码找回，重置丢失状态
		if (_small_tag_lost) {
			_small_tag_lost = false;
			RCLCPP_INFO(_node.get_logger(), "小码已重新识别");
		}
	}
}

// 新增：发送PX4命令函数实现（参数含默认值）
void PrecisionLand::send_vehicle_command(uint16_t command, 
                                         float param1, 
                                         float param2,
                                         float param3, 
                                         float param4, 
                                         double param5,
                                         double param6, 
                                         float param7)
{
	auto msg = std::make_unique<px4_msgs::msg::VehicleCommand>();
	msg->timestamp = _node.now().nanoseconds() / 1000; // 纳秒转微秒
	msg->param1 = param1;
	msg->param2 = param2;
	msg->param3 = param3;
	msg->param4 = param4;
	msg->param5 = param5;
	msg->param6 = param6;
	msg->param7 = param7;
	msg->command = command;
	msg->target_system = 1;
	msg->target_component = 1;
	msg->source_system = 1;
	msg->source_component = 1;
	msg->confirmation = 0;
	msg->from_external = true;

	_vehicle_command_pub->publish(std::move(msg));
	RCLCPP_INFO(_node.get_logger(), "发送PX4命令: command=%u, param1=%.1f", command, param1);
}

//新增：执行降落命令函数
void PrecisionLand::execute_land()
{
    // 1. 获取当前无人机状态（解锁状态）
    bool armed = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        armed = current_vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    }

    // 2. 检查是否已解锁（未解锁则不发送降落命令）
    if (!armed) {
        RCLCPP_WARN(_node.get_logger(), "无人机未解锁，无法发送降落命令");
        return;
    }

    // 3. 发送降落命令（VEHICLE_CMD_NAV_LAND=21，参数与模板一致）
    send_vehicle_command(VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(_node.get_logger(), "降落命令已发送,等待PX4执行");
}

//新增：激光高度回调函数
void PrecisionLand::laserHeightCallback(const px4_msgs::msg::DistanceSensor::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(height_mutex_);
    
    // 检查激光高度数据的有效性
    if (msg->type == msg->MAV_DISTANCE_SENSOR_LASER && 
        msg->orientation == msg->ROTATION_DOWNWARD_FACING &&
        msg->current_distance >= msg->min_distance && 
        msg->current_distance <= msg->max_distance) {
        
        // 1. 将有效数据加入滑动缓冲区
        _laser_height_buffer.push_back(msg->current_distance);
        
        // 2. 保持缓冲区大小不超过设定的窗口（例如5帧）
        if (_laser_height_buffer.size() > static_cast<size_t>(_laser_buffer_size)) {
            _laser_height_buffer.pop_front();
        }
        
        // 3. 计算滑动平均值（避免单帧数据跳变）
        double sum = 0.0;
        for (const auto& h : _laser_height_buffer) {
            sum += h;
        }
        _last_laser_height = sum / _laser_height_buffer.size();
        
        // 4. 标记数据有效并打印日志（改用DEBUG级别避免刷屏）
        _laser_data_valid = true;
        // RCLCPP_DEBUG(_node.get_logger(), 
        //              "激光高度更新 [滑动平均]: 原始=%.2f m, 滤波后=%.2f m (缓冲区大小=%ld)",
        //              msg->current_distance, _last_laser_height, _laser_height_buffer.size());
                     
    } else {
        // 无效数据处理：标记无效，清空缓冲区避免残留错误值
        _laser_data_valid = false;
        _laser_height_buffer.clear(); // 清空缓冲区，防止无效数据参与后续计算
        RCLCPP_WARN_THROTTLE(_node.get_logger(), *(_node.get_clock()), 1000, 
                             "激光数据无效 - 类型:%d, 朝向:%d, 距离:%.3f (有效范围:%.3f~%.3f)",
                             msg->type, msg->orientation, msg->current_distance, 
                             msg->min_distance, msg->max_distance);
    }
}


//新增：高度融合函数实现
double PrecisionLand::fuseHeight(double visual_height, double laser_height)
{
	std::lock_guard<std::mutex> lock(height_mutex_);
	// 权重归一化
	double total_weight = _param_visual_weight + _param_laser_weight;
	double visual_w = _param_visual_weight / total_weight;
	double laser_w = _param_laser_weight / total_weight;
	// 数据有效性检查
	bool visual_valid = !std::isnan(visual_height) && visual_height > 0.0;
	bool laser_valid = _laser_data_valid && laser_height >= _laser_min_valid_distance && laser_height <= _laser_max_valid_distance;

	double fused = 0.0;
	if (std::isnan(laser_height)) {
		laser_valid = false; // 激光高度为NaN时视为无效
	}
	
	if (visual_valid && laser_valid) {
		// 双源数据有效，按权重融合
		fused = visual_w * visual_height + laser_w * laser_height;
		//RCLCPP_DEBUG(_node.get_logger(), "高度融合: 视觉=%.2f m, 激光=%.2f m, 融合=%.2f m", visual_height, laser_height, fused);
	} else if (visual_valid) {
		// 仅视觉数据有效
		fused = visual_height;
		//RCLCPP_DEBUG(_node.get_logger(), "仅视觉高度有效: %.2f m", fused);
	} else if (laser_valid) {
		// 仅激光数据有效
		fused = laser_height;
		//RCLCPP_DEBUG(_node.get_logger(), "仅激光高度有效: %.2f m", fused);
	} else {
		// 均无效:使用无人机本地位置高度
		fused = -_vehicle_local_position->positionNed().z();
		//RCLCPP_WARN(_node.get_logger(), "视觉和激光高度数据均无效，使用本地位置高度: %.2f m", fused);
	}
	_fused_height = fused;
	return fused;
}

PrecisionLand::ArucoTag PrecisionLand::getTagWorld(const ArucoTag& tag)
{
	// ********** 坐标系转换 **********
	// 光学相机坐标（Optical） --> NED
	// 你的代码注释：
	// Optical: X right, Y down, Z away from lens
	// NED: X forward, Y right, Z away from viewer
	// 使用一个变换矩阵 R，把 optical 旋转到 NED
	Eigen::Matrix3d R;
	R << 0, -1, 0,
	     1,  0, 0,
	     0,  0, 1;
	Eigen::Quaterniond quat_NED(R); // 四元数表示的旋转（optical->NED）

	// 读取无人机当前在 NED（本地坐标）的位姿
	auto vehicle_position = Eigen::Vector3d(_vehicle_local_position->positionNed().cast<double>());
	auto vehicle_orientation = Eigen::Quaterniond(_vehicle_attitude->attitude().cast<double>());

	// 形成仿射变换（注意乘法顺序：平移 * 旋转）
	// drone_transform：机体在世界的位置+姿态
	Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_position) * vehicle_orientation;

	// camera_transform：相机相对于机体的偏置（这里写死了 0,0,-0.1）和 optical->NED 旋转
	// 注意：实际应用中相机偏置应由标定给出
	Eigen::Affine3d camera_transform = Eigen::Translation3d(0, 0, -0.1) * quat_NED;

	// tag_transform：相机检测到的 tag 在相机坐标系的位置+姿态
	Eigen::Affine3d tag_transform = Eigen::Translation3d(tag.position) * tag.orientation;

	// tag 在 world（NED）下的变换
	Eigen::Affine3d tag_world_transform = drone_transform * camera_transform * tag_transform;

	ArucoTag world_tag = {
		.position = tag_world_transform.translation(),
		.orientation = Eigen::Quaterniond(tag_world_transform.rotation()),
		.timestamp = tag.timestamp,
	};

	return world_tag;
}

void PrecisionLand::onActivate()
{
	// 激活时生成搜索航点并开始搜索
	generateSearchWaypoints();
	_search_started = true;
	_last_debug_print_time = _node.now().seconds(); // 新增：重置打印时间，确保激活后立即打印一次
	switchToState(State::Search);
}

void PrecisionLand::onDeactivate()
{
	// 空（可以在这里做清理）
}

void PrecisionLand::updateSetpoint(float dt_s)
{
	// 获取当前无人机高度（NED坐标系：Z轴向下，故高度为 -position.z()）
    double visual_height = -_vehicle_local_position->positionNed().z();	// 视觉高度
	double laser_height = _last_laser_height;				// 激光高度
	double current_height = fuseHeight(visual_height, laser_height);	// 融合高度
    bool target_lost = false;
    ArucoTag current_tag; // 当前依赖的二维码位姿


	// 根据当前高度决定使用大码还是小码
	if (current_height > _height_threshold_switch) {
		// 使用大码
		current_tag = _tag_large;
		target_lost = checkTargetTimeout(_tag_large);	// 检查大码是否丢失
		_small_tag_lost = false; // 重置小码丢失标志
	} else if (current_height > _height_threshold_low) {
		// 使用小码
		target_lost = checkTargetTimeout(_tag_small);	// 检查小码是否丢失
		current_tag = _tag_small;


		// 小码丢失处理：回升到一定高度寻找，超时则返回大码模式
		if (target_lost) {
			if (!_small_tag_lost) {
				_small_tag_lost = true;
				_small_tag_lost_time = _node.now();
				RCLCPP_INFO(_node.get_logger(), "小码丢失，上升高度以重新搜索");
			} 

			// 回升到一定高度（NED坐标系：Z轴向下，目标Z=-_height_threshold_recover）
			_trajectory_setpoint->updatePosition(Eigen::Vector3f(
				_vehicle_local_position->positionNed().x(),	// 保持当前位置X
				_vehicle_local_position->positionNed().y(),	// 保持当前位置Y
				-1.0 * _height_threshold_recover));

			// 检查是否超时未找到小码,返回一定高度后继续搜索大码
			double recover_duration = _node.now().seconds() - _small_tag_lost_time.seconds();
			if (recover_duration > _param_small_tag_recover_timeout) {
				RCLCPP_INFO(_node.get_logger(), "小码恢复超时，切换到大码模式");
				_small_tag_lost = false;
				_search_waypoint_index = 0;
                		switchToState(State::Search);
                		return;
            }
			return;   //回升阶段不执行其他控制
		} 
	} else {
		// 最终降落阶段，继续使用小码
		current_tag = _tag_small;
		target_lost = checkTargetTimeout(current_tag);
		// 小码丢失后的处理：满足高度条件则发送LAND命令，否则模式失败
		if (target_lost) {
			// 打印当前高度和切换阈值，方便调试
			RCLCPP_INFO(_node.get_logger(), "最终降落阶段小码丢失！当前高度: %.2f m, 切换Land模式阈值: %.2f m", 
						current_height, _land_mode_switch_height);

			// 切换Land模式的条件：当前高度 ≤ 配置阈值
			if (current_height <= _land_mode_switch_height) {
				RCLCPP_INFO(_node.get_logger(), "满足切换条件,执行PX4原生降落命令");
				
				// 调用封装的降落命令执行函数（与模板逻辑一致）
				execute_land();

				// 标记当前模式成功完成（已切换到PX4原生Land模式）
				ModeBase::completed(px4_ros2::Result::Success);
				switchToState(State::Idle);

			} else {
				// 不满足切换条件（高度过高），按原逻辑终止模式
				RCLCPP_ERROR(_node.get_logger(), "不满足Land模式切换条件(当前高度%.2f > 阈值%.2f)，紧急停止降落！",
							current_height, _land_mode_switch_height);
				ModeBase::completed(px4_ros2::Result::ModeFailureOther);
				switchToState(State::Idle);
			}
			return;
		}
    }

	// 打印目标丢失/找回日志（仅边沿触发）
	if (target_lost && !_target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Target lost (current: %s, 融合高度: %.2f m)", 
			current_height > _height_threshold_switch ? "large tag" : "small tag", current_height);		//打印当前依赖的二维码
	} else if (!target_lost && _target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Target acquired (current: %s, 融合高度: %.2f m)",
			current_height > _height_threshold_switch ? "large tag" : "small tag", current_height);		//打印当前依赖的二维码
	}
	_target_lost_prev = target_lost;	// 更新上次丢失状态

	//新增：调试信息打印控制
	double current_time = _node.now().seconds();
	bool need_print = (kEnableDebugOutput &&
		(current_time - _last_debug_print_time >= 0.5)); // 每0.5秒打印一次

	// 打印调试信息（0.5秒一次）
	if (need_print) {
		// 准备调试信息
		std::string current_tag_type = current_height > _height_threshold_switch ? "大码(" + std::to_string(_param_aruco_id_large) + ")" : "小码(" + std::to_string(_param_aruco_id_small) + ")";
		bool is_precise_control = (current_height <= _height_threshold_low);
		bool is_land_complete = (_state == State::Finished) || _land_detected;
		
		// 格式化输出激光高度（处理NaN情况）
		std::string laser_height_str = std::isnan(laser_height) ? "无效" : std::to_string(laser_height);
		
		// 打印调试信息
		RCLCPP_INFO(_node.get_logger(), 
			"\n===== 精准降落调试信息 =====\n"
			"视觉高度：%.2f m\t激光高度：%s m\t融合高度：%.2f m\n"
			"当前使用二维码：%s\n"
			"是否进入精控：%s\n"
			"是否完成降落：%s\n"
			"============================",
			visual_height,
			laser_height_str.c_str(),
			current_height,
			current_tag_type.c_str(),
			is_precise_control ? "是" : "否",
			is_land_complete ? "是" : "否");
		
		_last_debug_print_time = current_time; // 更新最后打印时间
	}

	// 状态机分支
	// 2. 原状态机逻辑修改：使用当前依赖的二维码（current_tag）
	switch (_state) {
	case State::Idle: {
		// 空转
		break;
	}

	case State::Search: {
		// 如果检测到 tag（position 非 NaN），记录当前高度作为 approach altitude 并进入 Approach
		// 检测到当前依赖的二维码，进入Approach
		if (!std::isnan(current_tag.position.x())) {	// position.x() 非 NaN 表示检测到
			RCLCPP_INFO(_node.get_logger(), "目标已识别,切换到Approach");
			_approach_altitude = _vehicle_local_position->positionNed().z();
			switchToState(State::Approach);
			break;
		}

		// 否则按航点搜索
		auto waypoint_position = _search_waypoints[_search_waypoint_index];
		_trajectory_setpoint->updatePosition(waypoint_position); // 位置 setpoint

		// 判断是否到达该航点，若到达则切换到下一个航点（循环）
		if (positionReached(waypoint_position)) {
			_search_waypoint_index++;

			if (_search_waypoint_index >= static_cast<int>(_search_waypoints.size())) {
				_search_waypoint_index = 0;
			}
		}

		break;
	}

	case State::Approach: {
		// 阶段1（大码）丢失：原逻辑退出；阶段2（小码）丢失已在上面处理
		if (target_lost && current_height > _height_threshold_switch) {
			RCLCPP_INFO(_node.get_logger(), "大码丢失，模式失败退出");
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Idle);
			return;
		}

		// 目标位置：使用当前tag的XY，保持approach_altitude
		auto target_position = Eigen::Vector3f(current_tag.position.x(), current_tag.position.y(), _approach_altitude);
		_trajectory_setpoint->updatePosition(target_position);

		// 到达后进入下降
		if (positionReached(target_position)) {
			RCLCPP_INFO(_node.get_logger(), "到达目标上方,切换到Descend");
			switchToState(State::Descend);
		}

		break;
	}

	case State::Descend: {

		// 按高度判断依赖的二维码，计算速度控制
            if (current_height > _height_threshold_low) {
                Eigen::Vector2f vel = calculateVelocitySetpointXY(current_tag); // 传入当前tag
                _trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel), std::nullopt,
                    px4_ros2::quaternionToYaw(current_tag.orientation));
            } else {
                // 最终精控降落阶段，速度减慢
                Eigen::Vector2f vel = calculateVelocitySetpointXY(current_tag);
                _trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel * 0.5), std::nullopt,
                    px4_ros2::quaternionToYaw(current_tag.orientation));
            }

		// 由飞控的 land_detected 决定是否已着陆，若已着陆则完成
		if (_land_detected) {
			RCLCPP_INFO(_node.get_logger(), "飞控检测到已着陆");
			switchToState(State::Finished);
		}

		break;
	}

	case State::Finished: {
		// 通知上层模式完成
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	} // end switch/case
}

Eigen::Vector2f PrecisionLand::calculateVelocitySetpointXY(const ArucoTag& current_tag)
{
	float p_gain = _param_vel_p_gain;
	float i_gain = _param_vel_i_gain;

	// P component: 当前机体位置减去 tag 位置
	float delta_pos_x = _vehicle_local_position->positionNed().x() - current_tag.position.x();
	float delta_pos_y = _vehicle_local_position->positionNed().y() - current_tag.position.y();

	// I component（简单累积）
	_vel_x_integral += delta_pos_x;
	_vel_y_integral += delta_pos_y;

	// 限制积分以避免 wind-up（这里用了 max_velocity 作为 clamp 上限）
	float max_integral = _param_max_velocity;
	_vel_x_integral = std::clamp(_vel_x_integral, -1.f * max_integral, max_integral);
	_vel_y_integral = std::clamp(_vel_y_integral, -1.f * max_integral, max_integral);

	// 计算 P 与 I 分量
	float Xp = delta_pos_x * p_gain;
	float Xi = _vel_x_integral * i_gain;
	float Yp = delta_pos_y * p_gain;
	float Yi = _vel_y_integral * i_gain;

	// P + I，注意取反是因为 delta = vehicle - target，速度应朝向 target
	float vx = -1.f * (Xp + Xi);
	float vy = -1.f * (Yp + Yi);

	// 限制速度（最小/最大），此处只做了 clamp 到 ±_param_max_velocity
	vx = std::clamp(vx, -1.f * _param_max_velocity, _param_max_velocity);
	vy = std::clamp(vy, -1.f * _param_max_velocity, _param_max_velocity);

	return Eigen::Vector2f(vx, vy);
}

bool PrecisionLand::checkTargetTimeout(const ArucoTag& tag)
{
	// 若 tag 本身无效，认为已丢失
	if (!tag.valid()) {
		return true;
	}

	// 若当前时间与 tag 时间差超过阈值，认为已丢失
	if (_node.now().seconds() - tag.timestamp.seconds() > _param_target_timeout) {
		return true;
	}

	return false;
}

void PrecisionLand::generateSearchWaypoints()
{
	// 生成一个“螺旋”搜索路径（NED）
	double start_x = 0.0;
	double start_y = 0.0;
	double current_z = -_fused_height; // 融合高度转NED（Z向下）
	auto min_z = -1.0; // 目标最低高度（NED 约定）
	double max_radius = 2.0;
	double layer_spacing = 0.5;
	int points_per_layer = 16;
	std::vector<Eigen::Vector3f> waypoints;

	// 计算需要的层数（注意符号/四舍五入）
	int num_layers = (static_cast<int>((min_z - current_z) / layer_spacing) / 2) < 1 ? 1 : (static_cast<int>((
				 min_z - current_z) / layer_spacing) / 2);

	// 为每一层产生外螺旋与内螺旋（并改变高度）
	for (int layer = 0; layer < num_layers; ++layer) {
		std::vector<Eigen::Vector3f> layer_waypoints;

		// 从中心向外螺旋
		double radius = 0.0;
		for (int point = 0; point < points_per_layer + 1; ++point) {
			double angle = 2.0 * M_PI * point / points_per_layer;
			double x = start_x + radius * cos(angle);
			double y = start_y + radius * sin(angle);
			double z = current_z;
			layer_waypoints.push_back(Eigen::Vector3f(x, y, z));
			radius += max_radius / points_per_layer; // 逐步增大半径
		}

		// 将该层的外螺旋加入总航点
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// 向内螺旋过渡时降低高度
		current_z += layer_spacing;

		// 内螺旋：将顺序反过来，然后设置 z，加入到主航点
		std::reverse(layer_waypoints.begin(), layer_waypoints.end());
		for (auto& waypoint : layer_waypoints) {
			waypoint.z() = current_z;
		}
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// 为下一层继续降低高度
		current_z += layer_spacing;
	}

	_search_waypoints = waypoints;
}

bool PrecisionLand::positionReached(const Eigen::Vector3f& target) const
{
	// 读取当前位置和速度
	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	const auto delta_pos = target - position;
	// 到达条件：位置误差小于阈值 && 速度小于阈值
	return (delta_pos.norm() < _param_delta_position) && (velocity.norm() < _param_delta_velocity);
}

std::string PrecisionLand::stateName(State state)
{
	switch (state) {
	case State::Idle:
		return "Idle";
	case State::Search:
		return "Search";
	case State::Approach:
		return "Approach";
	case State::Descend:
		return "Descend";
	case State::Finished:
		return "Finished";
	default:
		return "Unknown";
	}
}

void PrecisionLand::switchToState(State state)
{
	RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	// NodeWithMode 会把 PrecisionLand 包装成一个 ROS2 node 并处理 mode 的生命周期（例如周期性调用 updateSetpoint）
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<PrecisionLand>>(kModeName, kEnableDebugOutput));
	rclcpp::shutdown();
	return 0;
}
