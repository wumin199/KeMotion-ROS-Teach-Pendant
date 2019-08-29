#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QApplication>
#include <QPushButton>
#include <QButtonGroup>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include "rtp_msgs/SetInt16.h"
#include "rtp_msgs/RobotMoveCommands.h"
#include "rtp_msgs/RobotMove.h"
#include <dynamic_reconfigure/client.h>
#include "rtp_gui/RtpDynamicReconfigureConfig.h"
#include "rtp_gui/rtp_data_type.h"
#include "rtp_msgs/RobotMoveCommands.h"
#include "rtp_msgs/RobotMoveCommand.h"
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

  /**
   * @brief init Qt slots with signals
   */
  void initQtConnection();

  /**
   * @brief init GUI elements
   */
  void initGui();

  /**
   * @brief initializes a MoveGroupInterface for this gui
   */
  void initMoveGroup(moveit::planning_interface::MoveGroupInterface *group);

  /**
   * @brief initializes some pre-defined points for this case
   */
  void initPointList();


private slots:
  /**
   * @brief send joint jog button ID to serverServer
   */
  void joint_jog();

  /**
   * @brief send cart jog button ID to serverServer
   */
  void cart_jog();

  /**
   * @brief show velocity line_edit while changing QSlider
   */
  void show_velocity();

  /**
   * @brief conduct robot homing through calling a server
   */
  void home();

  /**
   * @brief conduct robot stopping through calling a server
   */
  void stop();


  /**
   * @brief fk calculation
   */
  void on_button_fk_clicked();
\
  /**
   * @brief ik calculation: ap -> cp(rpy)
   */
  void on_button_ik_1_clicked();

  /**
   * @brief ik calculation: ap -> cp(quaternion)
   */
  void on_button_ik_2_clicked();

  /**
   * @brief radio button slot functions
   */
  void refresh_position();

  /**
   * @brief refresh motion mode in between simulation or bringup
   */
  void refresh_mode();

  /**
   * @brief teach and save a point
   */
  void on_button_teach_save_clicked();


  /**
   * @brief execute pointlist
   */
  void on_button_execute_clicked();

  /**
   * @brief start voice control
   * speech recognizer starts working
   */
  void start_voice_control();

  /**
   * @brief stop voice control
   * speech recognizer stops working
   */
  void stop_voice_control();

private:

  /**
   * @brief update reply information
   * @param resp
   */
  void update_reply_show(rtp_msgs::SetInt16::Response &resp);

  /**
   * @brief update ui joint value
   */
  void ros_timer_callback();

  /**
   * @brief show point position in specified point
   * @param point id
   */
  void show_point_position(int point_id);

  /**
   * @brief record thread
   */
  void record_thread();

  /**
   * @brief display voice recognition results
   * @param msg
   */
  void speech_status(const std_msgs::String::ConstPtr &msg);

private:

  Ui::MainWindow *ui;

  //services
  ros::ServiceClient joint_teleop_client_;
  ros::ServiceClient cart_teleop_client_;
  ros::ServiceClient home_teleop_client_;
  ros::ServiceClient stop_teleop_client_;
  ros::ServiceClient move_client_;//continous move : lin -> ptp -> lin
  ros::ServiceClient mode_teleop_client_;
  ros::ServiceClient voice_control_client_;//start speech recording
  ros::ServiceClient move_voice_control_client_;//start moving according to speech recognition result

  //pubs and subs
  //ros::Publisher voice_control_pub_;//voice control
  ros::Subscriber enable_robot_sub;
  ros::Subscriber fault_robot_sub;
  ros::Subscriber end_link_name_sub;
  ros::Subscriber reference_link_sub;
  ros::Subscriber speech_sub_;//get speech recognition result

  //timer used to update ui
  ros::Timer ros_timer_;


  //links
  std::string end_link_;
  std::string reference_link_;

  tf::TransformListener tf_listener_;

  //velocity reconfigure
  dynamic_reconfigure::Client<rtp_gui::RtpDynamicReconfigureConfig>* dynamic_reconfigure_client_;

  //local stuff
  ros::NodeHandle local_gui_node_;
  std::vector<rtp_gui::MotionRequestData> motion_request_data_;
  moveit::planning_interface::MoveGroupInterface *group_;

  //radioButton group
  QButtonGroup *button_point_group_;
  QButtonGroup *button_mode_group_;

  //time point used to prevent button release too quickly
  std::chrono::system_clock::time_point time_button_pressed;
  std::chrono::system_clock::time_point time_button_released;
  std::chrono::system_clock::time_point record_button_pressed;
  std::chrono::system_clock::time_point record_button_released;

  bool bflag = false;
  std::thread record_thread_;
};

#endif // MAINWINDOW_H
