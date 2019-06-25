#ifndef RTP_GUI_RTP_MANAGER_H
#define RTP_GUI_RTP_MANAGER_H


#include <string>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>
#include "rtp_gui/RtpDynamicReconfigureConfig.h"
#include "rtp_teleop.h"


namespace rtp_gui {
class RtpManager;
}//end namespace rtp_gui

/**
 * @brief aggregates velocity dynamic reconfigure server and motion implementation(bring up mode and simulation mode)
 */
class RtpManager
{
public:
  /**
   * @brief default constructor
   * @param MoveGroupInterface
   * @param PlanningSceneMonitorPtr
   */
  RtpManager(moveit::planning_interface::MoveGroupInterface *group, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor);
  ~RtpManager();

  /**
   * @brief dynamic reconfigure server callback in velocity setting
   * @param config data from dynamic reconfigure client
   * @param level
   */
  void dynamic_reconfigure_cb(rtp_gui::RtpDynamicReconfigureConfig &config, uint32_t level);

  /**
   * @brief sets velocity scaling value to RtpTeleop
   * @param velocity scaling
   */
  void set_velocity_scaling(double data);

private:

  //moveit stuff
  moveit::planning_interface::MoveGroupInterface *group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  ros::NodeHandle local_nh_;

  //pubs
  ros::Publisher ref_link_name_pub_;
  ros::Publisher end_link_name_pub_;

  //tf listerner
  tf::TransformListener tf_listener_;

  /**
   * @brief implementation of simulation mode and bring up mode
   */
  RtpTeleop *teleop_api_;

  /**
   * @brief velocity scaling value setted by gui through dynamic reconfigure
   */
  double velocity_scaling_;

  /**
   * @brief newest planning frame
   * @todo
   */
  std_msgs::String ref_link_name_msg_;

  /**
   * @brief newest end effector link
   * @todo
   */
  std_msgs::String end_link_name_msg_;

  /**
   * @brief dynamic reconfiguration server in gui velocity setting
   */
  dynamic_reconfigure::Server<rtp_gui::RtpDynamicReconfigureConfig> dynamic_reconfigure_sever_;

};

#endif // RTP_GUI_RTP_MANAGER_H
