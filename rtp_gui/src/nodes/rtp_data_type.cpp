#include "rtp_gui/rtp_data_type.h"


bool rtp_gui::MotionRequestData::setJointValues(const std::vector<double> &joint_value)
{
    if (joint_value.size()<1)
      return false;

    this->joint_values_.resize(joint_value.size());
    this->joint_values_ = joint_value;
    return  true;
}

bool rtp_gui::MotionRequestData::setRPY(const std::vector<double> &rpy)
{
  if (rpy.size()!=3)
    return false;
  this->rpy_values_.resize(3);
  this->rpy_values_ = rpy;
  return true;
}
