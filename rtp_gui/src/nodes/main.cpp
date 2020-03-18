#include "rtp_gui/rtp_gui.h"
#include <angles/angles.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtp_node", ros::init_options::AnonymousName);
  ros::CallbackQueue move_group_cb_queue;

  std::string move_group_name;
  std::string move_group_desc = "robot_description";
  ros::NodeHandle move_group_nh;
  move_group_nh.setCallbackQueue(&move_group_cb_queue);

  move_group_nh.getParam("move_group_name", move_group_name);//eg."arm_er7" or "arm_er12"

  moveit::planning_interface::MoveGroupInterface::Options move_group_options(move_group_name, move_group_desc, move_group_nh);
  moveit::planning_interface::MoveGroupInterface move_group(move_group_options);

  ros::AsyncSpinner move_group_spinner(2, &move_group_cb_queue);
  move_group_spinner.start();

  move_group.startStateMonitor();
  move_group.getCurrentJointValues();

  ros::AsyncSpinner common_spinner(1);
  common_spinner.start();

  boost::shared_ptr<tf::Transformer> tf_ptr(new tf::Transformer());
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf_ptr));

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startStateMonitor();
  planning_scene_monitor->startWorldGeometryMonitor();

  RtpManager rtp_manager(&move_group, planning_scene_monitor);

  ros::waitForShutdown();

  return 0;
}
