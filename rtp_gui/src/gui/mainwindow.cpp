#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QApplication>
#include <QDebug>
#include <QList>
#include <QString>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <exception>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow),
  local_gui_node_("rtp_node/")
{
  ui->setupUi(this);

  initGui();
  initQtConnection();
  initPointList();
  show_point_position(1);

  joint_teleop_client_ = local_gui_node_.serviceClient<rtp_msgs::SetInt16>("joint_teleop");
  cart_teleop_client_ = local_gui_node_.serviceClient<rtp_msgs::SetInt16>("cart_teleop");
  home_teleop_client_ = local_gui_node_.serviceClient<std_srvs::SetBool>("home_teleop");
  stop_teleop_client_ = local_gui_node_.serviceClient<std_srvs::SetBool>("stop_teleop");
  mode_teleop_client_ = local_gui_node_.serviceClient<std_srvs::SetBool>("mode_teleop");

  //move_client_ = local_gui_node_.serviceClient<std::vector<rtp_msgs::RobotMoveCommand>>("move_teleop");
  move_client_ = local_gui_node_.serviceClient<rtp_msgs::RobotMove>("move_teleop");
  dynamic_reconfigure_client_ =
      new dynamic_reconfigure::Client<rtp_gui::RtpDynamicReconfigureConfig>("rtp_node/");

  move_voice_control_client_ = local_gui_node_.serviceClient<rtp_msgs::SetInt16>("move_voice_control");
  voice_control_client_ = local_gui_node_.serviceClient<std_srvs::SetBool>("voice_control");
  speech_sub_ = local_gui_node_.subscribe("speech_status", 1000, &MainWindow::speech_status, this);

  ros_timer_ = local_gui_node_.createTimer(ros::Duration(0.1), boost::bind(&MainWindow::ros_timer_callback, this));


}

void MainWindow::initQtConnection()
{

  QList<QPushButton *> joint_jog_buttons = ui->widget_joint_jog->findChildren<QPushButton *>();
  for (int var = 0; var < joint_jog_buttons.count(); ++var)
  {
    this->connect(joint_jog_buttons[var], &QPushButton::pressed,this, &MainWindow::joint_jog);
    this->connect(joint_jog_buttons[var], &QPushButton::released, this, &MainWindow::stop);
  }

  QList<QPushButton *> joint_pose_buttons = ui->widget_pose_jog->findChildren<QPushButton *>();
  for (int var = 0; var < joint_pose_buttons.count(); ++var)
  {
    this->connect(joint_pose_buttons[var], &QPushButton::pressed,this, &MainWindow::cart_jog);
    this->connect(joint_pose_buttons[var], &QPushButton::released, this, &MainWindow::stop);
  }

  this->connect(ui->slider_velocity, &QSlider::valueChanged, this, &MainWindow::show_velocity);
  this->connect(ui->button_home, &QPushButton::pressed, this, &MainWindow::home);
  this->connect(ui->button_home, &QPushButton::released, this, &MainWindow::stop);
  this->connect(ui->button_stop, &QPushButton::pressed, this, &MainWindow::stop);

  button_point_group_ = new QButtonGroup(this); //as herited from this, no need to delete it cz qt has done this for users
  button_point_group_->addButton(ui->rb_point1, 1);
  button_point_group_->addButton(ui->rb_point2, 2);
  button_point_group_->addButton(ui->rb_point3, 3);
  button_point_group_->addButton(ui->rb_point4, 4);
  this->connect(ui->rb_point1, &QRadioButton::clicked, this, &MainWindow::refresh_position);
  this->connect(ui->rb_point2, &QRadioButton::clicked, this, &MainWindow::refresh_position);
  this->connect(ui->rb_point3, &QRadioButton::clicked, this, &MainWindow::refresh_position);
  this->connect(ui->rb_point4, &QRadioButton::clicked, this, &MainWindow::refresh_position);

  button_mode_group_ = new QButtonGroup(this);
  button_mode_group_->addButton(ui->rb_simulation, 1);
  button_mode_group_->addButton(ui->rb_bringup, 2);
  this->connect(ui->rb_simulation, &QRadioButton::clicked, this, &MainWindow::refresh_mode);
  this->connect(ui->rb_bringup, &QRadioButton::clicked, this, &MainWindow::refresh_mode);

  //this->connect(ui->button_voice_control_start, &QPushButton::clicked, this, &MainWindow::start_voice_control);
  //this->connect(ui->button_voice_control_stop, &QPushButton::clicked, this, &MainWindow::stop_voice_control);
  this->connect(ui->button_voice_control, &QPushButton::pressed, this, &MainWindow::start_voice_control);
  this->connect(ui->button_voice_control, &QPushButton::released, this, &MainWindow::stop_voice_control);
}

void MainWindow::initGui()
{
  this->setWindowTitle("KeMotion-ROS Teach Pendant");
  ui->label_16->hide();
  ui->lineedit_reply_show->hide();

  setWindowFlags(this->windowFlags() &~ Qt::WindowMaximizeButtonHint);
  setFixedSize(this->width(), this->height());

  ui->tabWidget->setCurrentIndex(0);

  //hide Transformation Tab
  ui->tabWidget->setTabEnabled(1, false);
  ui->tabWidget->setStyleSheet("QTabBar::tab::disabled {width: 0; color: transparent;}");
}

void MainWindow::joint_jog()
{
  time_button_pressed = std::chrono::system_clock::now();

  //identify joint jog button ID as request_data
  //button_j1_p as ID:1  button_j2_m as ID:-1
  short request_data = 0;

  QString button_object_name = sender()->objectName();

  bool sign = button_object_name.endsWith("p");

  if (sign)
  {
    request_data = button_object_name.mid(button_object_name.count()-3, 1).toShort();
  }
  else
  {
    request_data = - button_object_name.mid(button_object_name.count()-3, 1).toShort();
  }

  ROS_INFO("GUI button ID is  %d",request_data);

  //connect to server
  rtp_msgs::SetInt16 srv;
  srv.request.data = request_data;

  if (!joint_teleop_client_.call(srv))
  {
    ROS_INFO("ui joint jog button request failed");
  }
  else
  {
    std_msgs::String str;
    str.data = srv.response.message;
    ROS_INFO("result is  %s", str.data.c_str());
  }
}

void MainWindow::cart_jog()
{
  time_button_pressed = std::chrono::system_clock::now();

  //identify cart jog button ID as request_data
  //X+/-,Y+/-,Z+/-,RX+/-,RY+/-,RZ+/- to (1/-1, 2/-2, ..., 6/-6
  short request_data = 0;
  QString button_name = sender()->objectName();
  bool sign = button_name.endsWith("p");

  if(sign)
  {
    request_data = button_name.mid(7,1).toShort();
  }
  else
  {
    request_data = - button_name.mid(7,1).toShort();
  }

  rtp_msgs::SetInt16 srv;
  srv.request.data = request_data;
  //qDebug()<<"gui cart req button is "<< srv.request.data;
  if (!cart_teleop_client_.call(srv))
  {
    ROS_INFO("ui cart jog button request failed");
  }
  else
  {
    std_msgs::String str;
    str.data = srv.response.message;
    ROS_INFO("gui get result %s", str.data.c_str());
  }
}

void MainWindow::show_velocity()
{
  int pos =  ui->slider_velocity->value();
  QString str = QString("%1").arg(pos);
  ui->lineedit_velocity->setText(str);

  rtp_gui::RtpDynamicReconfigureConfig config;
  config.velocity_scaling = static_cast<float>(pos);
  dynamic_reconfigure_client_->setConfiguration(config);
}

void MainWindow::home()
{  
  time_button_pressed = std::chrono::system_clock::now();

  std_srvs::SetBool srv;
  srv.request.data = true;
  if (!home_teleop_client_.call(srv))
  {
    ROS_INFO("ui home button request failed");
  }
}

void MainWindow::stop()
{
  std_srvs::SetBool srv;
  srv.request.data = true;

  time_button_released = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_button_released - time_button_pressed).count();
  //std::cout << "time consumes : " << duration << std::endl;

  //in case button release too quickly
  if (static_cast<long>(duration) < 500)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
  }

  if (!stop_teleop_client_.call(srv))
  {
    ROS_INFO("ui stop request failed");
  }
}

MainWindow::~MainWindow()
{
  delete ui;
  delete dynamic_reconfigure_client_;
}

void MainWindow::update_reply_show(rtp_msgs::SetInt16::Response &resp)
{

}

void MainWindow::initMoveGroup(moveit::planning_interface::MoveGroupInterface *group)
{
  group_ = group;
  end_link_ = group_->getEndEffectorLink();
  reference_link_ = group_->getPlanningFrame();
}

void MainWindow::ros_timer_callback()
{
  std::vector<std::string> joint_name = group_->getJointNames();
  std::vector<double> joint_value = group_->getCurrentJointValues();

  if (joint_name.size()==6 and joint_value.size()==6)
  {
    ui->lineedit_j1->setText(QString::number((joint_value[0]/M_PI)*180,10,2));
    ui->lineedit_j2->setText(QString::number((joint_value[1]/M_PI)*180,10,2));
    ui->lineedit_j3->setText(QString::number((joint_value[2]/M_PI)*180,10,2));
    ui->lineedit_j4->setText(QString::number((joint_value[3]/M_PI)*180,10,2));
    ui->lineedit_j5->setText(QString::number((joint_value[4]/M_PI)*180,10,2));
    ui->lineedit_j6->setText(QString::number((joint_value[5]/M_PI)*180,10,2));
  }

  tf::StampedTransform transform;
  while (ros::ok())
  {
    try
    {
      tf_listener_.waitForTransform(reference_link_, end_link_, ros::Time(0), ros::Duration(100));
      tf_listener_.lookupTransform(reference_link_, end_link_, ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }
  ui->lineedit_x->setText(QString::number(transform.getOrigin().getX()*1000,10,2));//m->mm
  ui->lineedit_y->setText(QString::number(transform.getOrigin().getY()*1000,10,2));
  ui->lineedit_z->setText(QString::number(transform.getOrigin().getZ()*1000,10,2));
  tf::Quaternion tf_q = transform.getRotation();
  double roll, pitch, yaw;
  tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  ui->lineedit_R->setText(QString::number(roll*180/M_PI,10,2));
  ui->lineedit_P->setText(QString::number(pitch*180/M_PI,10,2));
  ui->lineedit_Y->setText(QString::number(yaw*180/M_PI,10,2));
}

void MainWindow::on_button_fk_clicked()//ap-->cp
{
  robot_state::RobotStatePtr kinematic_state_ptr = group_->getCurrentState();
  robot_state::RobotState kinematic_state = *kinematic_state_ptr;
  const robot_state::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());

  std::vector<double> joint_values(6);//degree
  joint_values[0] = ui->lineedit_fk_j1->text().toDouble()*(M_PI/180);
  joint_values[1] = ui->lineedit_fk_j2->text().toDouble()*(M_PI/180);
  joint_values[2] = ui->lineedit_fk_j3->text().toDouble()*(M_PI/180);
  joint_values[3] = ui->lineedit_fk_j4->text().toDouble()*(M_PI/180);
  joint_values[4] = ui->lineedit_fk_j5->text().toDouble()*(M_PI/180);
  joint_values[5] = ui->lineedit_fk_j6->text().toDouble()*(M_PI/180);

  kinematic_state.setJointGroupPositions(joint_model_group, joint_values);
  const Eigen::Affine3d &end_effector_state = kinematic_state.getGlobalLinkTransform("tool0");

  tf::Pose pose;
  tf::poseEigenToTF(end_effector_state,pose);

  double x, y, z;//mm
  x = pose.getOrigin()[0]*1000;
  y = pose.getOrigin()[1]*1000;
  z = pose.getOrigin()[2]*1000;

  double q_x, q_y, q_z, q_w;
  q_x = pose.getRotation().x();
  q_y = pose.getRotation().y();
  q_z = pose.getRotation().z();
  q_w = pose.getRotation().w();

  double roll, pitch, yaw;//degree
  tf::Matrix3x3 mat(pose.getRotation());
  mat.getRPY(roll, pitch, yaw);
  roll *= (180/M_PI);
  pitch *= (180/M_PI);
  yaw *= (180/M_PI);

  //xyz + rpy
  QString pose_rpy;
  pose_rpy.append("x: ");
  pose_rpy.append(QString::number(x, 10, 2));
  pose_rpy.append("  y: ");
  pose_rpy.append(QString::number(y, 10, 2));
  pose_rpy.append("  z: ");
  pose_rpy.append(QString::number(z, 10, 2));
  pose_rpy.append(";    r: ");
  pose_rpy.append(QString::number(roll, 10, 2));
  pose_rpy.append("  p: ");
  pose_rpy.append(QString::number(pitch, 10, 2));
  pose_rpy.append("  y: ");
  pose_rpy.append(QString::number(yaw, 10, 2));
  ui->lineedit_fk_xyzrpy->setText(pose_rpy);

  //xyz + xyzw
  QString pose_quartenion;
  pose_quartenion.append("x: ");
  pose_quartenion.append(QString::number(x, 10, 2));
  pose_quartenion.append("  y: ");
  pose_quartenion.append(QString::number(y, 10, 2));
  pose_quartenion.append("  z: ");
  pose_quartenion.append(QString::number(z, 10, 2));
  pose_quartenion.append(";    q_x: ");
  pose_quartenion.append(QString::number(q_x, 10, 2));
  pose_quartenion.append("  q_y: ");
  pose_quartenion.append(QString::number(q_y, 10, 2));
  pose_quartenion.append("  q_z: ");
  pose_quartenion.append(QString::number(q_z, 10, 2));
  pose_quartenion.append("  q_w: ");
  pose_quartenion.append(QString::number(q_w, 10, 2));
  ui->lineedit_fk_trans_quat->setText(pose_quartenion);

  //matrix
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation());
}

void MainWindow::on_button_ik_1_clicked()//cp-->ap
{
  geometry_msgs::Pose pose;
  pose.position.x = ui->lineedit_ik1_x->text().toDouble()/1000;
  pose.position.y = ui->lineedit_ik1_y->text().toDouble()/1000;
  pose.position.z = ui->lineedit_ik1_z->text().toDouble()/1000;
  pose.orientation.x = ui->lineedit_ik1_qx->text().toDouble();
  pose.orientation.y = ui->lineedit_ik1_qy->text().toDouble();
  pose.orientation.z = ui->lineedit_ik1_qz->text().toDouble();
  pose.orientation.w = ui->lineedit_ik1_qw->text().toDouble();

  Eigen::Affine3d affine_pose;
  tf::Pose tf_pose;
  tf::poseMsgToTF(pose, tf_pose);
  tf::poseTFToEigen(tf_pose, affine_pose);

  //print matrix
  ROS_INFO_STREAM("Translation: \n" << affine_pose.translation());
  ROS_INFO_STREAM("Rotation: \n" << affine_pose.rotation());

  //xyz rpy
  tf::Quaternion tf_quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3 mat(tf_quat);
  mat.getRPY(roll, pitch, yaw);
  roll *= (180/M_PI);
  pitch *= (180/M_PI);
  yaw *= (180/M_PI);

  QString xyzrpy;
  xyzrpy.append("x: ");
  xyzrpy.append(ui->lineedit_ik1_x->text());
  xyzrpy.append("  y: ");
  xyzrpy.append(ui->lineedit_ik1_y->text());
  xyzrpy.append("  z: ");
  xyzrpy.append(ui->lineedit_ik1_z->text());
  xyzrpy.append(";    r: ");
  xyzrpy.append(QString::number(roll, 10, 2));
  xyzrpy.append("  p: ");
  xyzrpy.append(QString::number(pitch, 10, 2));
  xyzrpy.append("  y: ");
  xyzrpy.append(QString::number(yaw, 10, 2));
  ui->lineedit_ik_1_xyzrpy->setText(xyzrpy);

  //ik
  robot_state::RobotStatePtr kinematic_state_ptr = group_->getCurrentState();
  robot_state::RobotState kinematic_state = *kinematic_state_ptr;
  const robot_state::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());
  bool found_ik = kinematic_state.setFromIK(joint_model_group, pose, 10, 0.2);
  if (found_ik)
  {
    std::vector<double> joint_values(6);
    kinematic_state.copyJointGroupPositions(joint_model_group, joint_values);

    QString joints;
    joints.append("J1: ");
    joints.append(QString::number(joint_values[0]*(180/M_PI), 10, 2));
    joints.append("; J2: ");
    joints.append(QString::number(joint_values[1]*(180/M_PI), 10, 2));
    joints.append("; J3: ");
    joints.append(QString::number(joint_values[2]*(180/M_PI), 10, 2));
    joints.append("; J4: ");
    joints.append(QString::number(joint_values[3]*(180/M_PI), 10, 2));
    joints.append("; J5: ");
    joints.append(QString::number(joint_values[4]*(180/M_PI), 10, 2));
    joints.append("; J6: ");
    joints.append(QString::number(joint_values[5]*(180/M_PI), 10, 2));
    ui->lineedit_ik_1_joint->setText(joints);
  }
  else
  {
    ROS_INFO("ik found error");
  }
}

void MainWindow::on_button_ik_2_clicked()//cp-->ap
{
  double roll, pitch, yaw;
  roll = ui->lineedit_ik2_r->text().toDouble()*(M_PI/180);
  pitch = ui->lineedit_ik2_p->text().toDouble()*(M_PI/180);
  yaw = ui->lineedit_ik2_y_2->text().toDouble()*(M_PI/180);

  tf::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);

  geometry_msgs::Pose pose;
  pose.position.x = ui->lineedit_ik2_x->text().toDouble()/1000;
  pose.position.y = ui->lineedit_ik2_y->text().toDouble()/1000;
  pose.position.z = ui->lineedit_ik2_z->text().toDouble()/1000;
  pose.orientation.x = quaternion.getX();
  pose.orientation.y = quaternion.getY();
  pose.orientation.z = quaternion.getZ();
  pose.orientation.w = quaternion.getW();

  Eigen::Affine3d affine_pose;
  tf::Pose tf_pose;
  tf::poseMsgToTF(pose, tf_pose);
  tf::poseTFToEigen(tf_pose, affine_pose);

  //xyz xyzw
  QString xyzq;
  xyzq.append("x: ");
  xyzq.append(ui->lineedit_ik2_x->text());
  xyzq.append("  y: ");
  xyzq.append(ui->lineedit_ik2_y->text());
  xyzq.append("  z: ");
  xyzq.append(ui->lineedit_ik2_z->text());
  xyzq.append(";    x: ");
  xyzq.append(QString::number(pose.orientation.x, 10, 2));
  xyzq.append("  y: ");
  xyzq.append(QString::number(pose.orientation.y, 10, 2));
  xyzq.append("  z: ");
  xyzq.append(QString::number(pose.orientation.z, 10, 2));
  xyzq.append("  w: ");
  xyzq.append(QString::number(pose.orientation.w, 10, 2));
  ui->lineedit_ik_2_xyzq->setText(xyzq);

  //matrix
  ROS_INFO_STREAM("Translation: \n" << affine_pose.translation());
  ROS_INFO_STREAM("Rotation: \n" << affine_pose.rotation());

  //ik
  robot_state::RobotStatePtr kinematic_state_ptr = group_->getCurrentState();
  robot_state::RobotState kinematic_state = *kinematic_state_ptr;
  const robot_state::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());
  bool found_ik = kinematic_state.setFromIK(joint_model_group, pose, 10, 0.2);
  if (found_ik)
  {
    std::vector<double> joint_values(6);
    kinematic_state.copyJointGroupPositions(joint_model_group, joint_values);

    QString joints;
    joints.append("J1: ");
    joints.append(QString::number(joint_values[0]*(180/M_PI), 10, 2));
    joints.append("; J2: ");
    joints.append(QString::number(joint_values[1]*(180/M_PI), 10, 2));
    joints.append("; J3: ");
    joints.append(QString::number(joint_values[2]*(180/M_PI), 10, 2));
    joints.append("; J4: ");
    joints.append(QString::number(joint_values[3]*(180/M_PI), 10, 2));
    joints.append("; J5: ");
    joints.append(QString::number(joint_values[4]*(180/M_PI), 10, 2));
    joints.append("; J6: ");
    joints.append(QString::number(joint_values[5]*(180/M_PI), 10, 2));
    ui->lineedit_ik_2_joint->setText(joints);
  }
  else
  {
    ROS_INFO("ik found error");
  }
}

void MainWindow::initPointList()
{
  geometry_msgs::Pose pose;
  std::vector<double> joint_values(6);
  std::vector<double> rpy(3);

  //point1
  rtp_gui::MotionRequestData point_1("point1", rtp_gui::PTP, 0.4);

  pose.position.x = 0.4245;//m
  pose.position.y = 0.0;
  pose.position.z = 0.69489;
  pose.orientation.x = 0.97;
  pose.orientation.y = 0;
  pose.orientation.z = 0.26;
  pose.orientation.w = 0;
  point_1.setPose(pose);

  joint_values = {0, 0, 0, 0, -60, 0};
  std::for_each(joint_values.begin(), joint_values.end(), [](double &value){value *= (M_PI/180);});
  point_1.setJointValues(joint_values);

  rpy= {180, -30, 0};
  std::for_each(rpy.begin(), rpy.end(), [](double &value){value *= (M_PI/180);});
  point_1.setRPY(rpy);

  motion_request_data_.push_back(point_1);

  //point2
  rtp_gui::MotionRequestData point_2("point2", rtp_gui::PTP, 0.4);

  pose.position.x = 0.50671;
  pose.position.y = 0.0;
  pose.position.z = 0.54832;
  pose.orientation.x = 1;
  pose.orientation.y = 0;
  pose.orientation.z = 0.09;
  pose.orientation.w = 0;
  point_2.setPose(pose);

  joint_values = {0, -20, 0, 0, -60, 0};
  std::for_each(joint_values.begin(), joint_values.end(), [](double& value){value *= (M_PI/180);});
  point_2.setJointValues(joint_values);

  rpy ={180, -10, 0};
  std::for_each(rpy.begin(), rpy.end(), [](double &value){value *= (M_PI/180);});
  point_2.setRPY(rpy);

  motion_request_data_.push_back(point_2);


  //point3
  rtp_gui::MotionRequestData point_3("point3", rtp_gui::Line, 0.4);

  pose.position.x = 0.51567;
  pose.position.y = -0.20834;
  pose.position.z = 0.64029;
  pose.orientation.x = 0.97;
  pose.orientation.y = 0.02;
  pose.orientation.z = 0.21;
  pose.orientation.w = 0.09;
  point_3.setPose(pose);

  joint_values = {-22, -24, 20, 0, -60, -24};
  std::for_each(joint_values.begin(), joint_values.end(), [](double& value){value *= (M_PI/180);});
  point_3.setJointValues(joint_values);

  rpy ={168.78, -23.61,  4.35};
  std::for_each(rpy.begin(), rpy.end(), [](double &value){value *= (M_PI/180);});
  point_3.setRPY(rpy);

  motion_request_data_.push_back(point_3);


  //point4
  rtp_gui::MotionRequestData point_4("point4", rtp_gui::PTP, 0.4);

  pose.position.x = 0.46700;
  pose.position.y = 0;
  pose.position.z = 0.76850;
  pose.orientation.x = 0.71;
  pose.orientation.y = 0.00;
  pose.orientation.z = 0.71;
  pose.orientation.w = -0.00;
  point_4.setPose(pose);

  joint_values = {0, 0, 0, 0, 0, 0};
  std::for_each(joint_values.begin(), joint_values.end(), [](double& value){value *= (M_PI/180);});
  point_4.setJointValues(joint_values);

  rpy ={135.00, -90.00, 45.00};
  std::for_each(rpy.begin(), rpy.end(), [](double &value){value *= (M_PI/180);});
  point_4.setRPY(rpy);

  motion_request_data_.push_back(point_4);
}

void MainWindow::show_point_position(int point_id)
{
  if (point_id > motion_request_data_.size())
    return;

  QString joint_values;
  joint_values += "J1:";
  joint_values += QString::number(motion_request_data_[point_id-1].getJointValues()[0]*(180/M_PI), 10, 2);
  joint_values += "  J2:";
  joint_values += QString::number(motion_request_data_[point_id-1].getJointValues()[1]*(180/M_PI), 10, 2);
  joint_values += "  J3:";
  joint_values += QString::number(motion_request_data_[point_id-1].getJointValues()[2]*(180/M_PI), 10, 2);
  joint_values += "  J4:";
  joint_values += QString::number(motion_request_data_[point_id-1].getJointValues()[3]*(180/M_PI), 10, 2);
  joint_values += "  J5:";
  joint_values += QString::number(motion_request_data_[point_id-1].getJointValues()[4]*(180/M_PI), 10, 2);
  joint_values += "  J6:";
  joint_values += QString::number(motion_request_data_[point_id-1].getJointValues()[5]*(180/M_PI), 10, 2);
  ui->lineedit_joint_values->setText(joint_values);

  double speed;
  speed = motion_request_data_[point_id-1].getSpeed();
  ui->lineedit_speed->setText(QString::number(speed*100, 10, 1));

  QString cart_values;
  cart_values += "x:";
  cart_values += QString::number(motion_request_data_[point_id-1].getPose().position.x * 1000, 10, 2);
  cart_values += "  y:";
  cart_values += QString::number(motion_request_data_[point_id-1].getPose().position.y * 1000, 10, 2);
  cart_values += "  z:";
  cart_values += QString::number(motion_request_data_[point_id-1].getPose().position.z * 1000, 10, 2);
  cart_values += ";  R:";
  cart_values += QString::number(motion_request_data_[point_id-1].getRPY()[0] * (180/M_PI), 10, 2);
  cart_values += "  P:";
  cart_values += QString::number(motion_request_data_[point_id-1].getRPY()[1] * (180/M_PI), 10, 2);
  cart_values += "  Y:";
  cart_values += QString::number(motion_request_data_[point_id-1].getRPY()[2] * (180/M_PI), 10, 2);
  ui->lineedit_cart_values->setText(cart_values);
}

void MainWindow::refresh_position()
{
  show_point_position(button_point_group_->checkedId());
}

void MainWindow::on_button_teach_save_clicked()
{
  int point_id = button_point_group_->checkedId();
  rtp_gui::MotionRequestData req_data(motion_request_data_[point_id-1]); //inherit PTP or Line

  double speed = ui->lineedit_speed->text().toDouble();
  if (speed<=0 || speed >100)
  {
    speed = 40.0;
  }
  req_data.setSpeed(speed/100.0);

  geometry_msgs::Pose pose;
  pose.position = group_->getCurrentPose().pose.position;
  pose.orientation = group_->getCurrentPose().pose.orientation;
  req_data.setPose(pose);

  std::vector<double> joint_values(group_->getCurrentJointValues());
  req_data.setJointValues(joint_values);

  std::vector<double> rpy(group_->getCurrentRPY());
  req_data.setRPY(rpy);

  motion_request_data_[point_id-1] = req_data;
  refresh_position();
}

void MainWindow::on_button_execute_clicked()
{
  rtp_msgs::RobotMove srv;
  rtp_msgs::RobotMoveCommand cmd;

  for (int i=0; i < motion_request_data_.size(); i++)
  {
    cmd.rpy = motion_request_data_[i].getRPY();
    cmd.pose = motion_request_data_[i].getPose();
    cmd.speed = motion_request_data_[i].getSpeed();
    cmd.point_name = motion_request_data_[i].getPointName();
    cmd.motion_type = motion_request_data_[i].getMotionType();
    cmd.joints_valus = motion_request_data_[i].getJointValues();
    srv.request.cmd.commands.push_back(cmd);
  }

  if (!move_client_.call(srv))
  {
    ROS_INFO("execute requests failed");
  }
  else
  {
    std_msgs::String str;
    str.data = srv.response.message;
    ROS_INFO("gui get result %s", str.data.c_str());
  }
}

void MainWindow::refresh_mode()
{
  if (button_mode_group_->checkedId() == 2)
  {
    std_srvs::SetBool srv;
    srv.request.data = true;
    if (!mode_teleop_client_.call(srv))
    {
      ROS_INFO("bring up mode setting failed");
    }
  }
  else
  {
    std_srvs::SetBool srv;
    srv.request.data = false;
    if (!mode_teleop_client_.call(srv))
    {
      ROS_INFO("simulation mode setting failed");
    }
  }
}

void MainWindow::start_voice_control()
{
  std_srvs::SetBool srv;
  srv.request.data = true;
  if (!voice_control_client_.call(srv))
  {
    ui->lineedit_echo_words->setText("voice control start request failed");
  }
  else
  {
    //record_button_pressed = std::chrono::system_clock::now();
    //bflag = true;
    //record_thread_ = std::thread(&MainWindow::record_thread, this);
    //record_thread_.detach();
    ui->lineedit_echo_words->setText("recording...");
  }
}

void MainWindow::stop_voice_control()
{
  std_srvs::SetBool srv;
  srv.request.data = false;

  if (!voice_control_client_.call(srv))
  {
    ui->lineedit_echo_words->setText("voice control stop request failed");
  }
  //record_button_released = std::chrono::system_clock::now();
  //bflag = false;
  ui->lineedit_echo_words->setText("recording completed");
}

void MainWindow::record_thread()
{
  ros::Rate loop_rate(10);
  while (ros::ok() && bflag)
  {
    ROS_INFO("test");
    loop_rate.sleep();
  }

  ROS_INFO("break");

}

void MainWindow::speech_status(const std_msgs::String::ConstPtr &msg)
{
  QString data = QString::fromStdString(msg->data);
  ui->lineedit_echo_words->setText(data);
  ROS_INFO("speech data is [%s]", msg->data.c_str());

  bool rst = false;

  short request_data = 0;
  rtp_msgs::SetInt16 srv;

  //parse Chinese speech data


  //parse English speech data
  if (data.length() < 3 || data.length() > 15)
  {
    ROS_INFO("data length [%d]", data.length());
    return;
  }
  rst = (data.contains("up.",Qt::CaseInsensitive) || data.contains("上", Qt::CaseInsensitive));
  if (rst)
  {
    ROS_INFO("move up");
    request_data = 1;
  }

  rst = (data.contains("down.",Qt::CaseInsensitive) || data.contains("下", Qt::CaseInsensitive));
  if (rst)
  {
    ROS_INFO("move down");
    request_data = 2;
  }

  rst = (data.contains("left.",Qt::CaseInsensitive) || data.contains("左", Qt::CaseInsensitive));
  if (rst)
  {
    ROS_INFO("move left");
    request_data = 3;
  }

  rst = (data.contains("right.",Qt::CaseInsensitive) || data.contains("右", Qt::CaseInsensitive));
  if (rst)
  {
    ROS_INFO("move right");
    request_data = 4;
  }

  rst = (data.contains("forward.",Qt::CaseInsensitive) || data.contains("前", Qt::CaseInsensitive));
  if (rst)
  {
    ROS_INFO("move forward");
    request_data = 5;
  }

  rst = (data.contains("backward.",Qt::CaseInsensitive) || data.contains("后", Qt::CaseInsensitive));
  if (rst)
  {
    ROS_INFO("move backward");
    request_data = 6;
  }

  rst = (data.contains("stop.",Qt::CaseInsensitive) || data.contains("停止", Qt::CaseInsensitive));
  if (rst)
  {
    ROS_INFO("robot stop");
    request_data = 7;
  }

  rst = (data.contains("point.",Qt::CaseInsensitive) || data.contains("测试", Qt::CaseInsensitive)); //test point
  if (rst)
  {
    ROS_INFO("move to test point");
    request_data = 8;
  }

  if (request_data == 0)
    return;


  //connect to server
  srv.request.data = request_data;
  if (!move_voice_control_client_.call(srv))
  {
    ROS_INFO("request failed");
  }

}
