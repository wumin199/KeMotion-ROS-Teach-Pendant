#include <ros/ros.h>
#include <QApplication>
#include "mainwindow.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtp_gui_node");
  ros::NodeHandle gui_nh;
  ros::CallbackQueue move_group_cb_queue;
  std::string move_group_name = "arm_er7";
  std::string move_group_desc = "robot_description";
  gui_nh.setCallbackQueue(&move_group_cb_queue);

  moveit::planning_interface::MoveGroupInterface::Options move_group_options(move_group_name, move_group_desc, gui_nh);
  moveit::planning_interface::MoveGroupInterface move_group(move_group_options);

  ros::AsyncSpinner move_group_spinner(2, &move_group_cb_queue);
  move_group_spinner.start();

  move_group.startStateMonitor();
  move_group.getCurrentJointValues();

  ros::AsyncSpinner common_spinner(1);
  common_spinner.start();

  QApplication a(argc,argv);
  MainWindow w;
  w.initMoveGroup(&move_group);
  w.show();

  return a.exec();

}
