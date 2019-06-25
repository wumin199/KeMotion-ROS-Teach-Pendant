#include "rtp_gui/rtp_manager.h"


RtpManager::RtpManager(moveit::planning_interface::MoveGroupInterface *group, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor):
    group_(group), planning_scene_monitor_(planning_scene_monitor), local_nh_("rtp_node/")
{
  teleop_api_ = new RtpTeleop(group, planning_scene_monitor);

  dynamic_reconfigure_sever_.setCallback(boost::bind(&RtpManager::dynamic_reconfigure_cb, this, _1, _2));

  ref_link_name_pub_ = local_nh_.advertise<std_msgs::String>("reference_link_name", 1, true);
  end_link_name_pub_ = local_nh_.advertise<std_msgs::String>("end_link_name", 1, true);

  ref_link_name_msg_.data = group_->getPlanningFrame();
  end_link_name_msg_.data = group_->getEndEffectorLink();

  ref_link_name_pub_.publish(ref_link_name_msg_);
  end_link_name_pub_.publish(end_link_name_msg_);
}

RtpManager::~RtpManager()
{
  if (teleop_api_ != nullptr)
    delete teleop_api_;
}

void RtpManager::dynamic_reconfigure_cb(rtp_gui::RtpDynamicReconfigureConfig &config, uint32_t level)
{
  set_velocity_scaling(config.velocity_scaling);
}

void RtpManager::set_velocity_scaling(double data)
{
  velocity_scaling_ = data;//1~100
  teleop_api_->set_velocity_scaling(velocity_scaling_);
}
