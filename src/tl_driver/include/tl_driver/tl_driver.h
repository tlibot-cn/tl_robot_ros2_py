#ifndef TL_DRIVER__TL_DRIVER_H_
#define TL_DRIVER__TL_DRIVER_H_

// common
#include <algorithm>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// msg
#include "tl_ros2_interface/msg/arm_status.hpp"
#include "tl_ros2_interface/msg/move_command.hpp"
#include "tl_ros2_interface/msg/job_insert_move.hpp"
#include "tl_ros2_interface/msg/cartesian_pose.hpp"
#include "tl_ros2_interface/msg/tool_param.hpp"
#include "tl_ros2_interface/msg/modbus_tcp_param.hpp"
#include "tl_ros2_interface/msg/modbus_rtu_param.hpp"
#include "tl_ros2_interface/msg/modbus_master_param.hpp"
#include "tl_ros2_interface/msg/robot_dh_param.hpp"
#include "tl_ros2_interface/msg/job_file_name.hpp"

// srv
#include "tl_ros2_interface/srv/set_speed.hpp"
#include "tl_ros2_interface/srv/jogging.hpp"
#include "tl_ros2_interface/srv/set_drag_mode.hpp"
#include "tl_ros2_interface/srv/track_save.hpp"
#include "tl_ros2_interface/srv/track_playback.hpp"
#include "tl_ros2_interface/srv/set_tool_param.hpp"
#include "tl_ros2_interface/srv/set_user_coord.hpp"
#include "tl_ros2_interface/srv/set_axis_zero_pos.hpp"
#include "tl_ros2_interface/srv/set_current_coord.hpp"
#include "tl_ros2_interface/srv/get_coord_num.hpp"
#include "tl_ros2_interface/srv/tool_hand_calib.hpp"
#include "tl_ros2_interface/srv/set_digital_output.hpp"
#include "tl_ros2_interface/srv/get_digital_input_output.hpp"
#include "tl_ros2_interface/srv/modbus_write.hpp"
#include "tl_ros2_interface/srv/modbus_read.hpp"
#include "tl_ros2_interface/srv/coord_transform.hpp"
#include "tl_ros2_interface/srv/get_pos_reachable.hpp"
#include "tl_ros2_interface/srv/get_dh_param.hpp"
#include "tl_ros2_interface/srv/get_all_job_file_name.hpp"
#include "tl_ros2_interface/srv/job_run.hpp"
#include "tl_ros2_interface/srv/set_global_pos.hpp"
#include "tl_ros2_interface/srv/get_global_pos.hpp"
#include "tl_ros2_interface/srv/open_servo_j.hpp"
#include "tl_ros2_interface/srv/set_current_mode.hpp"
#include "tl_ros2_interface/srv/queue_motion_set_status.hpp"
#include "tl_ros2_interface/srv/queue_motion_move_j.hpp"

// lib
#include "nrc_interface.h"
#include "nrc_job_operate.h"
#include "nrc_track.h"
#include "nrc_io.h"
#include "nrc_modbus.h"
#include "nrc_queue_operate.h"

class TL_Arm : public rclcpp::Node
{
public:
  TL_Arm();
  ~TL_Arm();

  bool connect();
  bool disconnect();
  bool power_on();
  bool power_off();
  bool init();
  bool is_connected();
  bool is_powered();

  // 服务
  void handle_connect_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_disconnect_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_poweron_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_poweroff_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_clear_error_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_set_speed_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetSpeed::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetSpeed::Response> response);
  
  void handle_start_jogging_service(
      const std::shared_ptr<tl_ros2_interface::srv::Jogging::Request> request,
      std::shared_ptr<tl_ros2_interface::srv::Jogging::Response> response);
  
  void handle_stop_jogging_service(
    const std::shared_ptr<tl_ros2_interface::srv::Jogging::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::Jogging::Response> response);

  void handle_set_drag_mode_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetDragMode::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetDragMode::Response> response);

  void handle_get_drag_status_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_track_save_service(
    const std::shared_ptr<tl_ros2_interface::srv::TrackSave::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::TrackSave::Response> response);

  void handle_track_playback_service(
    const std::shared_ptr<tl_ros2_interface::srv::TrackPlayback::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::TrackPlayback::Response> response);

  void handle_set_tool_param_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetToolParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetToolParam::Response> response);

  void handle_set_user_coord_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetUserCoord::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetUserCoord::Response> response);

  void handle_set_axis_zero_pos_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetAxisZeroPos::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetAxisZeroPos::Response> response);

  void handle_set_current_coord_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetCurrentCoord::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetCurrentCoord::Response> response);

  void handle_get_coord_num_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetCoordNum::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetCoordNum::Response> response);

  void handle_tool_hand_calib_service(
    const std::shared_ptr<tl_ros2_interface::srv::ToolHandCalib::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::ToolHandCalib::Response> response);

  void handle_set_digital_output_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetDigitalOutput::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetDigitalOutput::Response> response);

  void handle_get_digital_input_output_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetDigitalInputOutput::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetDigitalInputOutput::Response> response);
  
  void handle_modbus_write_service(
    const std::shared_ptr<tl_ros2_interface::srv::ModbusWrite::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::ModbusWrite::Response> response);

  void handle_modbus_read_service(
    const std::shared_ptr<tl_ros2_interface::srv::ModbusRead::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::ModbusRead::Response> response);
  
  void handle_coord_transform_service(
    const std::shared_ptr<tl_ros2_interface::srv::CoordTransform::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::CoordTransform::Response> response);

  void handle_get_pos_reachable_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetPosReachable::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetPosReachable::Response> response);

  void handle_get_dh_param_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetDHParam::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetDHParam::Response> response);

  void handle_get_all_job_filename_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetAllJobFileName::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetAllJobFileName::Response> response);

  void handle_job_run_service(
    const std::shared_ptr<tl_ros2_interface::srv::JobRun::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::JobRun::Response> response);

  void handle_job_delete_service(
    const std::shared_ptr<tl_ros2_interface::srv::JobRun::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::JobRun::Response> response);

  void handle_set_global_pos_service( 
    const std::shared_ptr<tl_ros2_interface::srv::SetGlobalPos::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetGlobalPos::Response> response);

  void handle_get_global_pos_service(
    const std::shared_ptr<tl_ros2_interface::srv::GetGlobalPos::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::GetGlobalPos::Response> response);

  void handle_set_current_mode_service(
    const std::shared_ptr<tl_ros2_interface::srv::SetCurrentMode::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::SetCurrentMode::Response> response);

  void handle_open_servoj_service(
    const std::shared_ptr<tl_ros2_interface::srv::OpenServoJ::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::OpenServoJ::Response> response);

  void handle_close_servoj_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handle_queue_motion_set_status_service(
    const std::shared_ptr<tl_ros2_interface::srv::QueueMotionSetStatus::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::QueueMotionSetStatus::Response> response);

  void handle_queue_motion_movej_service(
    const std::shared_ptr<tl_ros2_interface::srv::QueueMotionMoveJ::Request> request,
    std::shared_ptr<tl_ros2_interface::srv::QueueMotionMoveJ::Response> response);
  
  void handle_queue_motion_stop_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // 话题
  void handle_movej_topic(
    const tl_ros2_interface::msg::MoveCommand::SharedPtr msg);

  void handle_job_insert_movej_topic(
    const tl_ros2_interface::msg::JobInsertMove::SharedPtr msg);

  void handle_movel_topic(
    const tl_ros2_interface::msg::MoveCommand::SharedPtr msg);

  void handle_job_insert_movel_topic(
    const tl_ros2_interface::msg::JobInsertMove::SharedPtr msg);

  void handle_set_servoj_pos_topic(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  void publish_arm_state();
  void publish_joint_pose(const std::vector<double> & joint_pose);
  void publish_tcp_pose(const std::vector<double> & tcp_pose);
  void publish_running_status();

private:
  std::string arm_ip_;
  std::string arm_port_;
  std::string arm_port_aux_;
  std::string arm_type_;

  int socket_fd_ {0};
  int socket_fd_aux_ {0};
  bool is_connected_ {false};           // 机械臂是否连接
  bool is_powered_ {false};             // 机械臂是否上电(示教模式)

  std::vector<std::string> arm_joints_;
  double publish_rate_ {100.0};
  int ndof_ {6};

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr poweron_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr poweroff_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_error_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetSpeed>::SharedPtr set_speed_service_;
  rclcpp::Service<tl_ros2_interface::srv::Jogging>::SharedPtr start_jogging_service_;
  rclcpp::Service<tl_ros2_interface::srv::Jogging>::SharedPtr stop_jogging_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetDragMode>::SharedPtr set_drag_mode_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_drag_status_service_;
  rclcpp::Service<tl_ros2_interface::srv::TrackSave>::SharedPtr track_save_service_;
  rclcpp::Service<tl_ros2_interface::srv::TrackPlayback>::SharedPtr track_playback_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetToolParam>::SharedPtr set_tool_param_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetUserCoord>::SharedPtr set_user_coord_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetAxisZeroPos>::SharedPtr set_axis_zero_pos_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetCurrentCoord>::SharedPtr set_current_coord_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetCoordNum>::SharedPtr get_coord_num_service_;
  rclcpp::Service<tl_ros2_interface::srv::ToolHandCalib>::SharedPtr tool_hand_calib_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetDigitalOutput>::SharedPtr set_digital_output_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetDigitalInputOutput>::SharedPtr get_digital_input_output_service_;
  rclcpp::Service<tl_ros2_interface::srv::ModbusWrite>::SharedPtr modbus_write_service_;
  rclcpp::Service<tl_ros2_interface::srv::ModbusRead>::SharedPtr modbus_read_service_;
  rclcpp::Service<tl_ros2_interface::srv::CoordTransform>::SharedPtr coord_transform_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetPosReachable>::SharedPtr get_pos_reachable_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetDHParam>::SharedPtr get_dh_param_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetAllJobFileName>::SharedPtr get_all_job_filename_service_;
  rclcpp::Service<tl_ros2_interface::srv::JobRun>::SharedPtr job_run_service_;
  rclcpp::Service<tl_ros2_interface::srv::JobRun>::SharedPtr job_delete_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetGlobalPos>::SharedPtr set_global_pos_service_;
  rclcpp::Service<tl_ros2_interface::srv::GetGlobalPos>::SharedPtr get_global_pos_service_;
  rclcpp::Service<tl_ros2_interface::srv::SetCurrentMode>::SharedPtr set_current_mode_service_;
  rclcpp::Service<tl_ros2_interface::srv::OpenServoJ>::SharedPtr open_servoj_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_servoj_service_;
  rclcpp::Service<tl_ros2_interface::srv::QueueMotionSetStatus>::SharedPtr queue_motion_set_status_service_;
  rclcpp::Service<tl_ros2_interface::srv::QueueMotionMoveJ>::SharedPtr queue_motion_movej_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr queue_motion_stop_service_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::CartesianPose>::SharedPtr tcp_pose_pub_;
  rclcpp::Publisher<tl_ros2_interface::msg::ArmStatus>::SharedPtr running_status_pub_;

  rclcpp::Subscription<tl_ros2_interface::msg::MoveCommand>::SharedPtr movej_sub_;
  rclcpp::Subscription<tl_ros2_interface::msg::JobInsertMove>::SharedPtr job_insert_movej_sub_;
  rclcpp::Subscription<tl_ros2_interface::msg::MoveCommand>::SharedPtr movel_sub_;
  rclcpp::Subscription<tl_ros2_interface::msg::JobInsertMove>::SharedPtr job_insert_movel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr set_servoj_pos_sub_;

  rclcpp::TimerBase::SharedPtr state_publish_timer_;
};

#endif  // TL_DRIVER__TL_DRIVER_H_