#include "rtp_gui/rtp_teleop.h"

RtpTeleop::RtpTeleop(moveit::planning_interface::MoveGroupInterface *group, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor):
  group_(group), planning_scene_monitor_(planning_scene_monitor), teleop_nh_("rtp_node/"), mode_(false)
{
  joint_teleop_server_ = teleop_nh_.advertiseService("joint_teleop", &RtpTeleop::joint_teleop_cb, this);
  cart_teleop_server_ = teleop_nh_.advertiseService("cart_teleop", &RtpTeleop::cart_teleop_cb, this);
  home_teleop_server_ = teleop_nh_.advertiseService("home_teleop", &RtpTeleop::home_teleop_cb, this);
  stop_teleop_server_ = teleop_nh_.advertiseService("stop_teleop", &RtpTeleop::stop_teleop_cb, this);
  move_teleop_server_ = teleop_nh_.advertiseService("move_teleop", &RtpTeleop::move_teleop_cb, this);
  mode_teleop_server_ = teleop_nh_.advertiseService("mode_teleop", &RtpTeleop::mode_teleop_cb, this);
  move_voice_control_server_ = teleop_nh_.advertiseService("move_voice_control", &RtpTeleop::move_voice_control_cb, this);

  pub_rmi_ = teleop_nh_.advertise<robot_movement_interface::CommandList>("command_list", 1);

  // parameter for normal teleop
  jogging_speed_ratio_default_ = rtp_gui::JOGGING_SPEED_DEFAULT_RATIO_CONST;//0.2
  joint_speed_limit_ = rtp_gui::JOINT_SPEED_LIMIT_CONST;
  cart_duration_default_ = rtp_gui::CART_DURATION_DEFAULT_CONST;//0.05s
  resolution_angle_ = 0.02;//rad
  resolution_linear_ = 0.005;//m

  joint_speed_.resize(rtp_gui::JOINT_SPEED_LIMIT_CONST.size());
  velocity_scaling_ = 0.4;//ui default velocity scaling
  for (int i =0; i<rtp_gui::JOINT_SPEED_LIMIT_CONST.size(); i++)
  {
    joint_speed_[i] = rtp_gui::JOINT_SPEED_LIMIT_CONST[i] * velocity_scaling_;
  }

  //new end effector and planning frame if user resets them
  end_link_ = group_->getEndEffectorLink();
  reference_link_ = group_->getPlanningFrame(); //eg. "/base_link"

  //default end effector and planning frame
  default_tip_link_ = group_->getEndEffectorLink();
  root_link_ = group_->getPlanningFrame();

  defalut_test_point = {0,-20,0,0,-60,0};//degree
  std::for_each(defalut_test_point.begin(), defalut_test_point.end(), [](double &value){value *= (M_PI/180);});
}

bool RtpTeleop::joint_teleop_cb(rtp_msgs::SetInt16::Request &req, rtp_msgs::SetInt16::Response &resp)
{
  //check button ID: req equals button ID
  if (req.data == 0 || abs(req.data) > group_->getJointNames().size())
  {
    resp.success = false;
    resp.message = "wrong joint teleop data";
    return true;
  }

  int joint_num = abs(req.data);
  std::vector<double> position_currrent = group_->getCurrentJointValues();
  std::vector<double> position_goal = position_currrent;//ideal goal
  std::vector<double> position_actual = position_currrent;//actual goal considering colision and distance
  double joint_current_position = position_currrent[joint_num-1];
  std::string direction = group_->getJointNames()[joint_num - 1];
  double sign;

  if(joint_num == req.data)
  {
    //axis move to upper
    position_goal[joint_num-1] = group_->getRobotModel()->getURDF()->getJoint(group_->getJointNames()[joint_num - 1])->limits->upper;
    direction.append("+");
    sign = 1;
  }
  else
  {
    //axis move to lower
    position_goal[joint_num-1] = group_->getRobotModel()->getURDF()->getJoint(group_->getJointNames()[joint_num - 1])->limits->lower;
    direction.append("-");
    sign = -1;
  }

  if(fabs(position_goal[joint_num-1] - joint_current_position) < 0.00873)//0.5 degree
  {
    resp.success = false;
    std::string result = "robot can't move in ";
    result.append(direction);
    result.append(" direction any more");
    resp.message = result;
    return true;
  }

  trajectory_msgs::JointTrajectoryPoint point_tmp;

  robot_state::RobotStatePtr kinematic_state_ptr = group_->getCurrentState();
  robot_state::RobotState kinematic_state = *kinematic_state_ptr;
  const robot_state::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());

  planning_scene_monitor_->updateFrameTransforms();
  planning_scene::PlanningSceneConstPtr plan_scene = planning_scene_monitor_->getPlanningScene();

  std::vector<double> position_tmp = position_currrent;
  bool collision_flag = false;

  int loop_num =1;

  //while((fabs(position_goal[joint_num-1] - position_tmp[joint_num-1]) / joint_speed_[joint_num-1]) > 0.1)
  while(fabs(position_goal[joint_num-1] - position_tmp[joint_num-1]) > 0.00873)//0.5 degree
  {
    //position_tmp[joint_num-1] += joint_speed_[joint_num-1] * 0.1 * sign;
    position_tmp[joint_num-1] += 0.01745 * sign;//1 degree per inscrement
    kinematic_state.setJointGroupPositions(joint_model_group, position_tmp);
    if(plan_scene->isStateColliding(kinematic_state, group_->getName()))
    {
      if(loop_num == 1)
      {
        resp.success = false;
        std::string result = "robot can't move in ";
        result.append(direction);
        result.append( "direction any longer, it will collide");
        resp.message = result;
        return true;
      }
      collision_flag = true;
      break;
    }
    position_actual[joint_num-1] = position_tmp[joint_num-1];
    loop_num++;
  }

  if(!collision_flag) // deal with last point
  {
    kinematic_state.setJointGroupPositions(joint_model_group, position_goal);
    if(!plan_scene->isStateColliding(kinematic_state, group_->getName()))
    {
      position_actual = position_goal;
    }
  }

  ROS_INFO("joint%d actual goal value is %f",joint_num-1,position_actual[joint_num-1]);

  group_->setStartStateToCurrentState();

  group_->setJointValueTarget(position_actual);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = group_->plan(my_plan);

  if (success)
  {
    trajectory_scaling(my_plan.trajectory_, jogging_speed_ratio_default_ * velocity_scaling_);
    moveit::planning_interface::MoveItErrorCode error;

    error = group_->asyncExecute(my_plan);
    if (error != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_INFO_STREAM("moveitErrorCode"<<error);
    }

    resp.success = true;
    std::string result = "robot is moving in ";
    result.append(direction);
    result.append(" direction");
    resp.message = result;
    return true;
  }
  else
  {
    ROS_INFO("joint ptp error");
    resp.success = false;
    resp.message = "joint ptp planning error";
    return true;
  }
}

bool RtpTeleop::cart_teleop_cb(rtp_msgs::SetInt16::Request &req, rtp_msgs::SetInt16::Response &resp)
{
  //req.data X+/-,Y+/-,Z+/-,RX+/-,RY+/-,RZ+/- to (1/-1, 2/-2, ..., 6/-6
  if(req.data == 0 || abs(req.data) > group_->getJointNames().size())
  {
    resp.success = false;
    resp.message = "wrong cart teleop data";
    return true;
  }

  int operation_num = abs(req.data);
  int sign;
  if (operation_num == req.data)
    sign = 1;
  else
    sign = -1;

  ros::Rate r(100);
  int counter = 0;

  while(ros::ok())
  {
    try
    {                                      //new one       default one
      tf_listener_.waitForTransform(reference_link_, root_link_, ros::Time(0), ros::Duration(10.0));
      tf_listener_.lookupTransform(reference_link_, root_link_, ros::Time(0), transform_rootToRef_);
      break;
    }
    catch (tf::TransformException &ex)
    {
      r.sleep();
      counter++;
      if(counter>200)
      {
        ROS_ERROR("%s", ex.what());
        resp.success = false;
        resp.message = "Can't get pose of reference frame";
        return true;
      }
      continue;
    }
  }

  counter = 0;
  while(ros::ok())
  {
    try
    {                                    //new one   default one
      tf_listener_.waitForTransform(end_link_, default_tip_link_, ros::Time(0), ros::Duration(10.0));
      tf_listener_.lookupTransform(end_link_, default_tip_link_, ros::Time(0), transform_tipToEnd_);
      break;
    }
    catch (tf::TransformException &ex)
    {
      r.sleep();
      counter++;
      if(counter>200)
      {
        ROS_ERROR("%s", ex.what());
        resp.success = false;
        resp.message = "Can't get pose of teleop frame";
        return true;
      }
      continue;
    }
  }

  Eigen::Affine3d affine_rootToRef, affine_refToRoot;
  tf::transformTFToEigen(transform_rootToRef_, affine_rootToRef);
  affine_refToRoot = affine_rootToRef.inverse();

  Eigen::Affine3d affine_tipToEnd;
  tf::transformTFToEigen(transform_tipToEnd_, affine_tipToEnd);

  geometry_msgs::PoseStamped current_pose = group_->getCurrentPose(end_link_);
  tf::Pose tf_pose_tmp;
  Eigen::Affine3d affine_pose_tmp;

  tf::poseMsgToTF(current_pose.pose, tf_pose_tmp);
  tf::poseTFToEigen(tf_pose_tmp, affine_pose_tmp);

  Eigen::Affine3d affine_current_pose = affine_rootToRef * affine_pose_tmp;
  tf::poseEigenToTF(affine_current_pose, tf_pose_tmp);

  tf::poseTFToMsg(tf_pose_tmp, current_pose.pose);//current_pose is now end_link_ to Ref_ at the jogging mode
  std::vector<double> current_joint_state = group_->getCurrentJointValues();

  trajectory_msgs::JointTrajectoryPoint planning_point;
  //first point
  planning_point.positions.resize(group_->getJointNames().size());
  planning_point.positions = group_->getCurrentJointValues();
  planning_point.velocities.resize(group_->getJointNames().size());
  planning_point.accelerations.resize(group_->getJointNames().size());
  ros::Duration duration(0);
  planning_point.time_from_start = duration;

  trajectory_msgs::JointTrajectory planning_trajectory;
  planning_trajectory.header.seq = 0;
  planning_trajectory.joint_names.resize(group_->getJointNames().size());
  for (int i=0; i < group_->getJointNames().size(); i++)
  {
    planning_trajectory.joint_names[i] = group_->getJointNames()[i];
  }
  planning_trajectory.points.push_back(planning_point);//push back current point as first one

  tf::Vector3 x_axis(1, 0, 0);
  tf::Vector3 y_axis(0, 1, 0);
  tf::Vector3 z_axis(0, 0, 1);

  double resolution_alpha = resolution_angle_;
  double resolution_delta = resolution_linear_;

  robot_state::RobotStatePtr kinematic_state_ptr = group_->getCurrentState();
  robot_state::RobotState kinematic_state = *kinematic_state_ptr;
  const robot_state::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());

  planning_scene_monitor_->updateFrameTransforms();
  planning_scene::PlanningSceneConstPtr plan_scene = planning_scene_monitor_->getPlanningScene();

  std::string direction;
  bool ik_have_result = true;

  for(int i=0; i<100; i++)
  {
    switch (operation_num){
    case 1:
        current_pose.pose.position.x += sign * resolution_delta;
        if (sign==1)
          direction = "X+";
        else
          direction = "X-";
      break;
    case 2:
        current_pose.pose.position.y += sign * resolution_delta;
        if (sign==1)
          direction = "Y+";
        else
          direction = "Y-";
      break;
    case 3:
        current_pose.pose.position.z += sign * resolution_delta;
        if (sign==1)
          direction = "Z+";
        else
          direction = "Z-";
      break;
    case 4:
        pose_stamped_rotation(current_pose, x_axis, sign*resolution_alpha);
        if (sign == 1)
          direction = "Rx+";
        else
          direction = "Rx-";
      break;
    case 5:
        pose_stamped_rotation(current_pose, y_axis, sign*resolution_alpha);
        if (sign == 1)
          direction = "Ry+";
        else
          direction = "Ry-";
      break;
    case 6:
        pose_stamped_rotation(current_pose, z_axis, sign*resolution_alpha);
        if (sign == 1)
          direction = "Rz+";
        else
          direction = "Rz-";
      break;
    default:
      break;
    }

    tf::poseMsgToTF(current_pose.pose, tf_pose_tmp);
    tf::poseTFToEigen(tf_pose_tmp, affine_pose_tmp);

    affine_current_pose = affine_refToRoot * affine_pose_tmp * affine_tipToEnd;//renew each time

    ik_have_result = kinematic_state.setFromIK(joint_model_group, affine_current_pose,default_tip_link_);
    if (ik_have_result)
    {
      if (planning_trajectory.points.size() != (i+1))
        break;

      bool position_validation = true;

      for (int j=0; j < group_->getJointNames().size(); j++)
      {
        planning_point.positions[j] = *kinematic_state.getJointPositions(group_->getJointNames()[j]); // value after IK

        double shift_tmp = fabs(planning_trajectory.points[i].positions[j] -  planning_point.positions[j]);
        if (shift_tmp > cart_duration_ * rtp_gui::JOINT_SPEED_LIMIT_CONST[j])
          position_validation = false;
      }

      if (position_validation == false || plan_scene->isStateColliding(kinematic_state, group_->getName()))
      {
        ROS_INFO_STREAM("position validattion false or collision occurs, plan breaks in "<<i);
        break;
      }

      ros::Duration dur((i+1)*cart_duration_);
      planning_point.time_from_start = dur;
      planning_trajectory.points.push_back(planning_point);
    }
    else
    {
      ROS_INFO_STREAM("no ik solution, break in plan point ID: "<<i);
      break;
    }
  }


  if (planning_trajectory.points.size() == 1)
  {
    resp.success=false;
    std::string result="robot can't move in ";
    result.append(direction);
    result.append(" direction any more");
    resp.message=result;
    return true;
  }

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //init my_plan
  my_plan.trajectory_.joint_trajectory.header.seq = 0;
  my_plan.trajectory_.joint_trajectory.header.frame_id = group_->getPlanningFrame();// "/base_link"
  my_plan.trajectory_.joint_trajectory.joint_names.resize(group_->getJointNames().size());
  for (int i=0; i < group_->getJointNames().size(); i++)
  {
    my_plan.trajectory_.joint_trajectory.joint_names[i] = group_->getJointNames()[i];
  }

  my_plan.trajectory_.joint_trajectory.points.resize(planning_trajectory.points.size());
  for (int i=0; i < planning_trajectory.points.size(); i++)
  {
    my_plan.trajectory_.joint_trajectory.points[i].time_from_start = planning_trajectory.points[i].time_from_start;
    my_plan.trajectory_.joint_trajectory.points[i].velocities.resize(planning_trajectory.joint_names.size());
    my_plan.trajectory_.joint_trajectory.points[i].accelerations.resize(planning_trajectory.joint_names.size());
    my_plan.trajectory_.joint_trajectory.points[i].positions.resize(planning_trajectory.joint_names.size());
    for (int j=0; j < planning_trajectory.points[i].positions.size(); j++)
    {
      my_plan.trajectory_.joint_trajectory.points[i].positions[j] = planning_trajectory.points[i].positions[j];
    }
  }

  resp.success=true;
  std::string result="robot is moving in ";
  result.append(direction);
  result.append(" direction");
  resp.message=result;
  group_->asyncExecute(my_plan);
  return true;
}

bool RtpTeleop::home_teleop_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
  std::vector<double> position_current = group_->getCurrentJointValues();
  std::vector<double> position_goal;
  position_goal.resize(joint_speed_.size());// {0,0,0,0,0,0} represents zero position

  group_->setStartStateToCurrentState();

  group_->setJointValueTarget(position_goal);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = group_->plan(my_plan);

  if (success)
  {
    trajectory_scaling(my_plan.trajectory_, velocity_scaling_);
    group_->asyncExecute(my_plan);
    resp.success = true;
    std::string result = "robot is moving to home position";
    resp.message = result;
    return true;
  }
  else
  {
    ROS_INFO("homing error");
    resp.success = false;
    resp.message = "homing planning error";
    return true;
  }
}

bool RtpTeleop::stop_teleop_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
  group_->stop();
  resp.success = true;
  resp.message = "stop moving";
  //ROS_INFO("button release");
  return true;
}

void RtpTeleop::set_velocity_scaling(double data)
{
  this->velocity_scaling_ = data/100.0;//data:1~100
  for (int i =0; i<rtp_gui::JOINT_SPEED_LIMIT_CONST.size(); i++)
  {
    joint_speed_[i] = rtp_gui::JOINT_SPEED_LIMIT_CONST[i] * velocity_scaling_;
  }
  cart_duration_ = cart_duration_default_ / velocity_scaling_; //0.05/
  ROS_INFO("new speed is %f",velocity_scaling_ );
  //group_->setMaxVelocityScalingFactor(velocity_scaling_);
}

void RtpTeleop::pose_stamped_rotation(geometry_msgs::PoseStamped &pose_stamped, const tf::Vector3 &axis, double angle)
{
  tf::Quaternion q_1(pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                     pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w);
  tf::Quaternion q_2(axis, angle);
  tf::Matrix3x3 m(q_1);
  tf::Matrix3x3 m_2(q_2);
  m_2.operator *= (m);

  double r, p, y;
  m_2.getRPY(r,p,y);
  q_2.setRPY(r,p,y);
  pose_stamped.pose.orientation.x = q_2.getX();
  pose_stamped.pose.orientation.y = q_2.getY();
  pose_stamped.pose.orientation.z = q_2.getZ();
  pose_stamped.pose.orientation.w = q_2.getW();
}

bool RtpTeleop::check_points_in_home_mode(const std::vector<double> &position_goal, const std::vector<double> &calculation_point, std::vector<double> &speed, double time_increment)
{
  bool rst = true;
  for (int i=0; i<position_goal.size(); i++)
  {
    if (fabs(position_goal[i]-calculation_point[i])/speed[i] > time_increment)
      rst = false;
  }
  return rst;
}

void RtpTeleop::trajectory_scaling(moveit_msgs::RobotTrajectory &trajectory, double scale)
{
  if(scale<=0 || scale >1)
  {
    return;
  }
  for(int i=0; i<trajectory.joint_trajectory.points.size(); i++)
  {
    trajectory.joint_trajectory.points[i].time_from_start.operator*=(1.0/scale);
    for (int j=0; j<trajectory.joint_trajectory.points[i].velocities.size(); j++)
    {
      trajectory.joint_trajectory.points[i].velocities[j] *= scale;
    }
    for (int j=0; j<trajectory.joint_trajectory.points[i].accelerations.size(); j++)
    {
      trajectory.joint_trajectory.points[i].accelerations[j] *= (scale*scale);
    }
  }
}

bool RtpTeleop::move_teleop_cb(rtp_msgs::RobotMove::Request &req, rtp_msgs::RobotMove::Response &resp)
{
  if (mode_ == false)
  {
    //simulation mode
    bool rst = move_simulation(req, resp);
    return true;
  }
  else
  {
    //bring up mode
    bool rst = move_bringup(req, resp);
    return true;
  }
}

bool RtpTeleop::mode_teleop_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
  if (req.data == true)
  {
    system("gnome-terminal -e \"rosnode kill /joint_state_publisher\"");
    system("gnome-terminal -e \"roslaunch rmi_driver rmi_driver.launch\"");
    mode_ = true;
    resp.message = "Mode: bring up";
  }
  else
  {
    system("gnome-terminal -e \"roslaunch rtp_gui joint_state_publisher.launch\"");
    system("gnome-terminal -e \"rosnode kill /rmi_driver\"");
    mode_ = false;
    resp.message = "Mode : simulation";
  }

  resp.success = true;
  return true;
}

bool RtpTeleop::move_joint(const rtp_msgs::RobotMoveCommand &cmd)
{
  bool rst = true;
  std::vector<double> joint_position(cmd.joints_valus);
  group_->setJointValueTarget(joint_position);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = group_->plan(my_plan);
  if (success)
  {
    trajectory_scaling(my_plan.trajectory_, cmd.speed);
    group_->execute(my_plan);
  }
  else
  {
    rst = false;
  }
  return rst;
}

bool RtpTeleop::move_cart(const rtp_msgs::RobotMoveCommand &cmd)
{
  bool rst = true;

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose pose;
  pose = cmd.pose;
  waypoints.push_back(pose);
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold =0.0;  //0.0
  const double eef_step = 0.05;//m
  double fraction = group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  if (fraction == 1)
  {
    my_plan.trajectory_ = trajectory;
    trajectory_scaling(my_plan.trajectory_, cmd.speed);
    group_->execute(my_plan);
  }
  else
  {
    rst = false;
  }
  return rst;
}

bool RtpTeleop::move_simulation(rtp_msgs::RobotMove::Request &req, rtp_msgs::RobotMove::Response &resp)
{
  group_->setStartStateToCurrentState();
  robot_state::RobotState kinematic_state = *group_->getCurrentState();
  const robot_state::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  for (int i = 0; i < group_->getJointNames().size(); i++)
  {
    my_plan.trajectory_.joint_trajectory.joint_names.push_back(group_->getJointNames()[i]);
  }
  my_plan.trajectory_.joint_trajectory.header.seq = 0;
  my_plan.trajectory_.joint_trajectory.header.frame_id = reference_link_;//"/base_link"

  for (int i = 0; i < req.cmd.commands.size(); i++)
  {
    if (req.cmd.commands[i].motion_type == 1) //PTP
    {
      moveit::planning_interface::MoveGroupInterface::Plan my_plan_tmp;
      group_->setJointValueTarget(req.cmd.commands[i].joints_valus);
      moveit::planning_interface::MoveItErrorCode success = group_->plan(my_plan_tmp);
      if (success)
      {
        if (i == 0)
        {
          trajectory_scaling(my_plan_tmp.trajectory_, req.cmd.commands[i].speed);
          my_plan.trajectory_ = my_plan_tmp.trajectory_;
        }
        else
        {
          //remove first position and modify time_from_start
          trajectory_scaling(my_plan_tmp.trajectory_, req.cmd.commands[i].speed);
          ros::Duration dur((my_plan.trajectory_.joint_trajectory.points.end() - 1)->time_from_start);
          std::for_each(my_plan_tmp.trajectory_.joint_trajectory.points.begin()+1, my_plan_tmp.trajectory_.joint_trajectory.points.end(),
                        [=, &my_plan](trajectory_msgs::JointTrajectoryPoint &point)
                                      {
                                        point.time_from_start += dur;
                                        my_plan.trajectory_.joint_trajectory.points.push_back(point);
                                      });
        }
      }
      else
      {
        resp.success = false;
        resp.message = "point ";
        resp.message += (i+1);
        resp.message += " : plan error";
        return false;
      }
    }
    else if (req.cmd.commands[i].motion_type == 2)//Line
    {
      std::vector<geometry_msgs::Pose> waypoints;
      geometry_msgs::Pose pose;
      const double jump_threshold =0.0;  //0.0
      const double eef_step = 0.01;//m
      double fraction = 0;
      int maxtries = 20;
      int attempts = 0;
      moveit_msgs::RobotTrajectory trajectory;

      if (i == 0)
      {
        pose.position = group_->getCurrentPose().pose.position;
        pose.orientation = group_->getCurrentPose().pose.orientation;
        waypoints.push_back(pose);
        pose = req.cmd.commands[i].pose;
        waypoints.push_back(pose);

        while (fraction < 1.0 && attempts < maxtries)
        {
          fraction = group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
          attempts += 1;
          ROS_INFO("still trying after  %d attempts...", attempts);
          ROS_INFO("Visualizing cartesian path (%.2f%% acheived)", fraction * 100.0);
        }

        if (fraction == 1)
        {
          trajectory_scaling(trajectory, req.cmd.commands[i].speed);
          my_plan.trajectory_ = trajectory;
        }
        else
        {
          resp.success = false;
          resp.message = "point ";
          resp.message += std::to_string(i+1);
          resp.message += " : plan error, last fraction is ";
          resp.message += std::to_string(fraction);
          return false;
        }
      }
      else
      {
        waypoints.push_back(req.cmd.commands[i-1].pose);
        waypoints.push_back(req.cmd.commands[i].pose);

        while (fraction < 1.0 && attempts < maxtries)
        {
          fraction = group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
          attempts += 1;
          ROS_INFO("still trying after  %d attempts...", attempts);
          ROS_INFO("Visualizing cartesian path (%.2f%% acheived)", fraction * 100.0);
        }

        if (fraction == 1)
        {
          trajectory_scaling(trajectory, req.cmd.commands[i].speed);
          ros::Duration dur((my_plan.trajectory_.joint_trajectory.points.end() - 1)->time_from_start);
          std::for_each(trajectory.joint_trajectory.points.begin()+1, trajectory.joint_trajectory.points.end(),
                        [=, &my_plan](trajectory_msgs::JointTrajectoryPoint &point)
                                      {
                                        point.time_from_start += dur;
                                        my_plan.trajectory_.joint_trajectory.points.push_back(point);
                                      });
        }
        else
        {
          resp.success = false;
          resp.message = "point ";
          resp.message += std::to_string(i+1);
          resp.message += " : plan error, last fraction is ";
          resp.message += std::to_string(fraction);
          return false;
        }
      }
    }
    else
    {
      resp.success = false;
      resp.message = "wrong motion type";
      return false;
    }

    kinematic_state.setJointGroupPositions(joint_model_group, req.cmd.commands[i].joints_valus);
    group_->setStartState(kinematic_state);
  }

  group_->asyncExecute(my_plan);
  resp.success = true;
  resp.message = "plan done, moving ...";
  return true;
}

bool RtpTeleop::move_bringup(rtp_msgs::RobotMove::Request &req, rtp_msgs::RobotMove::Response &resp)
{
  robot_movement_interface::CommandList cmd_list;
  creat_cmd_list(req.cmd, cmd_list);

  pub_rmi_.publish(cmd_list);

  resp.success = true;
  return true;
}

void RtpTeleop::creat_cmd_list(const rtp_msgs::RobotMoveCommands &cmds, robot_movement_interface::CommandList &cmd_list)
{
  cmd_id_ = 0;
  for(auto &&pt : cmds.commands)
  {
    cmd_list.commands.emplace_back(pose_to_rmi_command(pt));
  }
}

robot_movement_interface::Command RtpTeleop::pose_to_rmi_command(const rtp_msgs::RobotMoveCommand &robot_move_cmd)
{
  robot_movement_interface::Command cmd;
  cmd.header.frame_id = "base_link";

  switch (robot_move_cmd.motion_type)
  {
  case 1:
    cmd.command_type = "PTP";
    cmd.command_id = cmd_id_++;
    cmd.pose_type = "JOINTS";

    for (int i=0; i<6; i++)
    {
      cmd.pose.push_back(robot_move_cmd.joints_valus[i]);
    }

    cmd.blending_type = "OVLREL"; //= %
    cmd.blending.push_back(80.0);

    cmd.velocity_type = "DYN";

    for (int i = 0; i < 4; i++)
    {
      cmd.velocity.push_back(robot_move_cmd.speed * 100);//velAxis, accAxis, decAxis, jerkAxis
    }

    cmd.velocity.push_back(1000.0);//vel
    cmd.velocity.push_back(5000.0);//acc
    cmd.velocity.push_back(5000.0);//dec
    cmd.velocity.push_back(100000.0);//jerk

    cmd.velocity.push_back(1000.0);//velOri
    cmd.velocity.push_back(5000.0);//accOri
    cmd.velocity.push_back(5000.0);//decOri
    cmd.velocity.push_back(100000.0);//jerkOri

    break;

  case 2:
    cmd.command_type = "LIN";
    cmd.command_id = cmd_id_++;
    cmd.pose_type = "QUATERNION";
    cmd.pose.push_back(robot_move_cmd.pose.position.x);
    cmd.pose.push_back(robot_move_cmd.pose.position.y);
    cmd.pose.push_back(robot_move_cmd.pose.position.z);
    cmd.pose.push_back(robot_move_cmd.pose.orientation.w);
    cmd.pose.push_back(robot_move_cmd.pose.orientation.x);
    cmd.pose.push_back(robot_move_cmd.pose.orientation.y);
    cmd.pose.push_back(robot_move_cmd.pose.orientation.z);

    cmd.blending_type = "OVLREL";
    cmd.blending.push_back(80.0);

    cmd.velocity_type = "DYN";
    for (int i = 0; i < 4; i++)
    {
      cmd.velocity.push_back(100); //velAxis, accAxis, decAxis, jerkAxis
    }

    cmd.velocity.push_back(1000.0 * robot_move_cmd.speed);//vel
    cmd.velocity.push_back(5000.0 * robot_move_cmd.speed);//acc
    cmd.velocity.push_back(5000.0 * robot_move_cmd.speed);//dec
    cmd.velocity.push_back(100000.0 * robot_move_cmd.speed);//jerk

    cmd.velocity.push_back(1000.0 * robot_move_cmd.speed);//velOri
    cmd.velocity.push_back(5000.0 * robot_move_cmd.speed);//accOri
    cmd.velocity.push_back(5000.0 * robot_move_cmd.speed);//decOri
    cmd.velocity.push_back(100000.0 * robot_move_cmd.speed);//jerkOri

    break;

  default :
    break;
  }

  return cmd;

}

bool RtpTeleop::move_voice_control_cb(rtp_msgs::SetInt16::Request &req, rtp_msgs::SetInt16::Response &resp)
{
  double velocity_scale = 0.1;
  if (mode_ == false)//simulation
  {
    group_->stop();

    bool rst = voice_control_simulation(req.data, velocity_scale);
    if (rst)
    {
      resp.success = true;
      return true;
    }
    else
    {
      resp.success = false;
      return true;
    }
  }
  else//bring up
  {
    bool rst = voice_control_bringup(req.data, velocity_scale);
    if (rst)
    {
      resp.success = true;
      return true;
    }
    else
    {
      resp.success = false;
      return true;
    }
  }

}

bool RtpTeleop::cartesian_path_plan(const std::vector<geometry_msgs::Pose> &waypoints, moveit_msgs::RobotTrajectory &trajectory, const double eef_step, int maxtries, const double jump_threshold)
{
  double fraction = 0;
  int attempts = 0;
  while (fraction < 1.0 && attempts < maxtries)
  {
    fraction = group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    attempts += 1;
  }

  if (fraction == 1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool RtpTeleop::voice_control_simulation(const int cmd_type, const double velocity_scaling)
{
  if (cmd_type == 7)//stop robot
    return true;

  if (cmd_type == 8)//move to test point
    {
      group_->setJointValueTarget(defalut_test_point);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      moveit::planning_interface::MoveItErrorCode success = group_->plan(my_plan);
      if (success)
      {
        trajectory_scaling(my_plan.trajectory_, velocity_scaling);
        group_->asyncExecute(my_plan);
        return true;
      }
      else
      {
        return false;
      }
    }


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  for (int i = 0; i < group_->getJointNames().size(); i++)
  {
    my_plan.trajectory_.joint_trajectory.joint_names.push_back(group_->getJointNames()[i]);
  }
  my_plan.trajectory_.joint_trajectory.header.seq = 0;
  my_plan.trajectory_.joint_trajectory.header.frame_id = reference_link_;//"/base_link"

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose current_pose;//first point
  current_pose.position = group_->getCurrentPose().pose.position;
  current_pose.orientation = group_->getCurrentPose().pose.orientation;
  waypoints.push_back(current_pose);

  moveit_msgs::RobotTrajectory trajectory;

  geometry_msgs::Pose pose(current_pose);//second point

  double increment = 0.1;//10cm

  if (cmd_type == 1) //move up:z+
  {
    pose.position.z += increment;
  }

  if (cmd_type == 2)//move down:z-
  {
    pose.position.z -= increment;
  }

  if (cmd_type == 3)//move left:y+
  {
    pose.position.y += increment;
  }

  if (cmd_type == 4)//move left:y-
  {
    pose.position.y -= increment;
  }

  if (cmd_type == 5)//move forward:x+
  {
    pose.position.x += increment;
  }

  if (cmd_type == 6)//move backward:x-
  {
    pose.position.x -= increment;
  }

  waypoints.push_back(pose);


  bool rst = cartesian_path_plan(waypoints, trajectory,0.005, 30, 0);
  if (rst)
  {
    trajectory_scaling(trajectory, velocity_scaling);
    my_plan.trajectory_ = trajectory;
    group_->asyncExecute(my_plan);
    return true;
  }
  else
  {
    pose = current_pose;
    if (cmd_type == 1) //move up:z+
    {
      pose.position.z += increment/2;
    }

    if (cmd_type == 2)//move down:z-
    {
      pose.position.z -= increment/2;
    }

    if (cmd_type == 3)//move left:y+
    {
      pose.position.y += increment/2;
    }

    if (cmd_type == 4)//move left:y-
    {
      pose.position.y -= increment/2;
    }

    if (cmd_type == 5)//move forward:x+
    {
      pose.position.x += increment/2;
    }

    if (cmd_type == 6)//move backward:x-
    {
      pose.position.x -= increment/2;
    }

    waypoints.back() = pose;

    rst = cartesian_path_plan(waypoints, trajectory,0.005, 30, 0);
    if (rst)
    {
      trajectory_scaling(trajectory, velocity_scaling);
      my_plan.trajectory_ = trajectory;
      group_->asyncExecute(my_plan);
      return true;
    }
    else
    {
      return false;
    }
  }

}

bool RtpTeleop::voice_control_bringup(const int cmd_type, const double velocity_scaling)
{

  if (cmd_type == 7)//stop robot
  {
    robot_movement_interface::Command cmd;
    robot_movement_interface::CommandList cmd_list;
    cmd.command_id = 1;
    cmd.command_type = "ABORT";
    cmd_list.commands.emplace_back(cmd);
    pub_rmi_.publish(cmd_list);
    return true;
  }

  if (cmd_type == 8)//move to test point
  {
    robot_movement_interface::Command cmd;
    robot_movement_interface::CommandList cmd_list;
    cmd.command_id = 1;
    cmd.command_type = "ABORT";
    cmd_list.commands.emplace_back(cmd);

    cmd.command_id = 2;
    cmd.command_type = "PTP";
    cmd.pose_type = "JOINTS";
    for (int i=0; i<6; i++)
    {
      cmd.pose.push_back(defalut_test_point[i]);
    }

    cmd.blending_type = "OVLREL";
    cmd.blending.push_back(80.0);
    cmd.velocity_type = "DYN";

    for (int i = 0; i < 4; i++)
    {
      cmd.velocity.push_back(velocity_scaling * 100);//velAxis, accAxis, decAxis, jerkAxis
    }

    cmd.velocity.push_back(1000.0);//vel
    cmd.velocity.push_back(5000.0);//acc
    cmd.velocity.push_back(5000.0);//dec
    cmd.velocity.push_back(100000.0);//jerk

    cmd.velocity.push_back(1000.0);//velOri
    cmd.velocity.push_back(5000.0);//accOri
    cmd.velocity.push_back(5000.0);//decOri
    cmd.velocity.push_back(100000.0);//jerkOri

    cmd_list.commands.emplace_back(cmd);
    pub_rmi_.publish(cmd_list);
    return true;
  }

  robot_movement_interface::Command cmd;
  robot_movement_interface::CommandList cmd_list;

  cmd.command_id = 1;
  cmd.command_type = "ABORT";
  cmd_list.commands.emplace_back(cmd);

  cmd.command_id = 2;
  cmd.command_type = "LIN";
  cmd.pose_type = "QUATERNION";
  cmd.blending_type = "OVLREL";
  cmd.blending.push_back(0.0);

  cmd.velocity_type = "DYN";
  for (int i = 0; i < 4; i++)
  {
    cmd.velocity.push_back(100); //velAxis, accAxis, decAxis, jerkAxis
  }

  cmd.velocity.push_back(1000.0 * velocity_scaling);//vel
  cmd.velocity.push_back(5000.0 * velocity_scaling);//acc
  cmd.velocity.push_back(5000.0 * velocity_scaling);//dec
  cmd.velocity.push_back(100000.0 * velocity_scaling);//jerk

  cmd.velocity.push_back(1000.0 * velocity_scaling);//velOri
  cmd.velocity.push_back(5000.0 * velocity_scaling);//accOri
  cmd.velocity.push_back(5000.0 * velocity_scaling);//decOri
  cmd.velocity.push_back(100000.0 * velocity_scaling);//jerkOri

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  for (int i = 0; i < group_->getJointNames().size(); i++)
  {
    my_plan.trajectory_.joint_trajectory.joint_names.push_back(group_->getJointNames()[i]);
  }
  my_plan.trajectory_.joint_trajectory.header.seq = 0;
  my_plan.trajectory_.joint_trajectory.header.frame_id = reference_link_;//"/base_link"

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose current_pose;//first point
  current_pose.position = group_->getCurrentPose().pose.position;
  current_pose.orientation = group_->getCurrentPose().pose.orientation;
  waypoints.push_back(current_pose);

  moveit_msgs::RobotTrajectory trajectory;

  geometry_msgs::Pose pose(current_pose);//second point

  double increment = 0.1;//10cm

  if (cmd_type == 1) //move up:z+
  {
    pose.position.z += increment;
  }

  if (cmd_type == 2)//move down:z-
  {
    pose.position.z -= increment;
  }

  if (cmd_type == 3)//move left:y+
  {
    pose.position.y += increment;
  }

  if (cmd_type == 4)//move left:y-
  {
    pose.position.y -= increment;
  }

  if (cmd_type == 5)//move forward:x+
  {
    pose.position.x += increment;
  }

  if (cmd_type == 6)//move backward:x-
  {
    pose.position.x -= increment;
  }

  waypoints.push_back(pose);


  bool rst = cartesian_path_plan(waypoints, trajectory,0.005, 30, 0);
  if (rst)
  {
    cmd.pose.push_back(pose.position.x);
    cmd.pose.push_back(pose.position.y);
    cmd.pose.push_back(pose.position.z);
    cmd.pose.push_back(pose.orientation.w);
    cmd.pose.push_back(pose.orientation.x);
    cmd.pose.push_back(pose.orientation.y);
    cmd.pose.push_back(pose.orientation.z);

    cmd_list.commands.emplace_back(cmd);
    pub_rmi_.publish(cmd_list);
    return true;
  }
  else
  {
    pose = current_pose;
    if (cmd_type == 1) //move up:z+
    {
      pose.position.z += increment/2;
    }

    if (cmd_type == 2)//move down:z-
    {
      pose.position.z -= increment/2;
    }

    if (cmd_type == 3)//move left:y+
    {
      pose.position.y += increment/2;
    }

    if (cmd_type == 4)//move left:y-
    {
      pose.position.y -= increment/2;
    }

    if (cmd_type == 5)//move forward:x+
    {
      pose.position.x += increment/2;
    }

    if (cmd_type == 6)//move backward:x-
    {
      pose.position.x -= increment/2;
    }

    waypoints.back() = pose;

    rst = cartesian_path_plan(waypoints, trajectory,0.005, 30, 0);
    if (rst)
    {
      cmd.pose.push_back(pose.position.x);
      cmd.pose.push_back(pose.position.y);
      cmd.pose.push_back(pose.position.z);
      cmd.pose.push_back(pose.orientation.w);
      cmd.pose.push_back(pose.orientation.x);
      cmd.pose.push_back(pose.orientation.y);
      cmd.pose.push_back(pose.orientation.z);

      cmd_list.commands.emplace_back(cmd);
      pub_rmi_.publish(cmd_list);
      return true;
    }
    else
    {
      return false;
    }
  }
}
