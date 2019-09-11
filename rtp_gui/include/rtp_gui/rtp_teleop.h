#ifndef RTP_GUI_RTP_TELEOP_H
#define RTP_GUI_RTP_TELEOP_H

#include <ros/ros.h>
#include <vector>
#include <mutex>
#include <cstdlib>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
//#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include "rtp_msgs/SetInt16.h"
#include "rtp_msgs/SetFloat64.h"
#include "rtp_msgs/SetString.h"
#include "rtp_gui/rtp_gui_const.h"
#include "rtp_msgs/RobotMoveCommand.h"
#include "rtp_msgs/RobotMove.h"
#include "robot_movement_interface/CommandList.h"


namespace rtp_gui {
class RtpTeleop;
}//end namespace rtp_gui

/**
 * @brief implementation of simulation mode and bring up mode
 */
class RtpTeleop
{
public:
  RtpTeleop(moveit::planning_interface::MoveGroupInterface *group, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor);

  /**
   * @brief set velocity scaling
   * @param velocity scaling
   */
  void set_velocity_scaling(double data);

private:

  //callbacks of corresponding service
  bool joint_teleop_cb(rtp_msgs::SetInt16::Request &req, rtp_msgs::SetInt16::Response &resp);
  bool cart_teleop_cb(rtp_msgs::SetInt16::Request &req, rtp_msgs::SetInt16::Response &resp);
  bool home_teleop_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
  bool stop_teleop_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
  bool move_teleop_cb(rtp_msgs::RobotMove::Request &req, rtp_msgs::RobotMove::Response &resp);
  bool mode_teleop_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
  bool move_voice_control_cb(rtp_msgs::SetInt16::Request &req, rtp_msgs::SetInt16::Response &resp);

  /**
   * @brief execute a joint motion in blocking mode
   * @param a rtp_msgs::RobotMoveCommand cmd to be executed
   * @return true if path plans ok, otherwise false
   * useless in this case
   */
  bool move_joint(const rtp_msgs::RobotMoveCommand &cmd);

  /**
   * @brief execute a cart motion in blocking mode
   * @param a rtp_msgs::RobotMoveCommand cmd to be executed
   * @return true if path plans ok, otherwise false
   * useless in this case
   */
  bool move_cart(const rtp_msgs::RobotMoveCommand &cmd);

  /**
   * @brief create robot_movement_interface::CommandList from rtp_msgs::RobotMoveCommands
   * @param rtp_msgs::RobotMoveCommands cmds to be transformed
   * @param a robot_movement_interface::CommandList cmd_list
   */
  void creat_cmd_list(const rtp_msgs::RobotMoveCommands &cmds, robot_movement_interface::CommandList &cmd_list);

  /**
   * @brief transform a rtp_msgs::RobotMoveCommand to a robot_movement_interface::Command
   * @param rtp_msgs::RobotMoveCommand to be transformed
   * @return robot_movement_interface::Command data
   */
  robot_movement_interface::Command pose_to_rmi_command(const rtp_msgs::RobotMoveCommand &robot_move_cmd);

  /**
   * @brief transform a pose along an axis in defined angle
   * @param pose_stamped The pose to be transformed
   * @param axis  Define the axis
   * @param angle The rotation angle along axis
   */
  void pose_stamped_rotation(geometry_msgs::PoseStamped &pose_stamped, const tf::Vector3 &axis, double angle);

  /**
   * @brief Check if an point is too close to the goal point. If so, we use goal point instead
   *
   * When button pressed in home mode,trajectory points will be produced in defined increment.
   * Last point may not obey this incremnt, so we may use goal point instead.
   * @param position_goal  Like Zero Position [0,0,0,0,0,0]
   * @param calculation_point
   * @param speed  Speed for each point
   * @param time_increment time increment for consecutive two points
   * @return
   */
  bool check_points_in_home_mode(const std::vector<double> &position_goal, const std::vector<double> &calculation_point, std::vector<double> &speed, double time_increment);

  /**
   * @brief re-specify a trajectory's speed
   * @param trajectory to be velocity scaled
   * @param scale value
   */
  void trajectory_scaling(moveit_msgs::RobotTrajectory &trajectory, double scale);

  /**
   * @brief simulate motion in fake controller
   * @param request commands
   * @param response message
   * @return
   */
  bool move_simulation(rtp_msgs::RobotMove::Request &req, rtp_msgs::RobotMove::Response &resp);

  /**
   * @brief send robot_movement_interface::CommandList to rmi_driver
   * @param request commands
   * @param response message
   * @return
   */
  bool move_bringup(rtp_msgs::RobotMove::Request &req, rtp_msgs::RobotMove::Response &resp);

  /**
   * @brief encapsulation function to compute cartesian path
   * @param waypoints
   * @param trajectory
   * @param eef_step
   * @param maxtries
   * @param jump_threshold
   * @return
   */
  bool cartesian_path_plan(const std::vector<geometry_msgs::Pose> &waypoints, moveit_msgs::RobotTrajectory &trajectory ,const double eef_step, int maxtries=30, const double jump_threshold=0);


  /**
   * @brief voice control wrapper function in simulation
   * @param command type parsed from voice speech
   * @param velocity_scaling(0~1)
   * @return
   */
  bool voice_control_simulation(const int cmd_type, const double velocity_scaling);

  /**
   * @brief voice control wrapper function in bringup
   * @param command type parsed from voice speech
   * @param velocity_scaling(0~1)
   * @return
   */
  bool voice_control_bringup(const int cmd_type, const double velocity_scaling);

private:

  //moveit stuff
  moveit::planning_interface::MoveGroupInterface *group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;//useless here
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  ros::NodeHandle teleop_nh_;

  //pubs and subs
  ros::ServiceServer joint_teleop_server_;
  ros::ServiceServer cart_teleop_server_;
  ros::ServiceServer home_teleop_server_;
  ros::ServiceServer stop_teleop_server_;
  ros::ServiceServer move_teleop_server_;
  ros::ServiceServer mode_teleop_server_;
  ros::ServiceServer move_voice_control_server_;
  ros::Publisher pub_rmi_;//topic to rmi_driver


  /**
   * @brief angle increment when joint jogging
   */
  double resolution_angle_;

  /**
   * @brief linear increment when cart jogging
   */
  double resolution_linear_;

  /**
   * @brief actual velocity scaling in %
   * modified by ui
   */
  double velocity_scaling_;

  /**
   * @brief actual speed (rad/s)
   */
  std::vector<double> joint_speed_;

  /**
   * @brief max speed ratio under jogging mode
   */
  double jogging_speed_ratio_default_;

  /**
   * @brief joint speed limit according to urdf
   * modify this part yourself
   */
  std::vector<double> joint_speed_limit_;

  /**
   * @brief use to specify time consumption in between 2 cart poses when jogging under different speed
   */
  double cart_duration_; // in second

  /**
   * @brief time consumption between 2 points when 100%
   * use to check for next possible point when jogging cartpose
   */
  double cart_duration_default_;//when 100% speed , time consumed between 2 points

  /**
   * @brief new end effector, can be set by user
   */
  std::string end_link_;

  /**
   * @brief new planning frame, can be set by user
   */
  std::string reference_link_;

  /**
   * @brief default end effector according to urdf
   */
  std::string default_tip_link_;

  /**
   * @brief default planning frame according to urdf
   */
  std::string root_link_;

  //tf stuff
  tf::TransformListener tf_listener_;
  tf::StampedTransform transform_rootToRef_;
  tf::StampedTransform transform_tipToEnd_;

  /**
   * @brief motion mode
   * false if simulation mode, true if bring up mode
   */
  bool mode_;// false for simulation mode, true for bring up mode

  /**
   * @brief user's command id to push back into robot_movement_interface::Command
   */
  int cmd_id_;

  /**
   * @brief default test point using in speech control case
   */
  std::vector<double> defalut_test_point;

};

#endif // RTP_GUI_RTP_TELEOP_H
