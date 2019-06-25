#ifndef RTP_DATA_TYPE_H
#define RTP_DATA_TYPE_H

#include <vector>
#include <string>
#include <geometry_msgs/Pose.h>

namespace rtp_gui {

/**
 * @brief Enumeration of motion types in this gui demo
 */
enum MotionType
{
  PTP =1,
  Line =2
};

/**
 * @brief Data type that used in between gui and rtp_node
 * it encapsulates a point's related values
 */
class MotionRequestData
{
public:
  /**
   * @brief defalut constructor
   */
  MotionRequestData(){}

  /**
   * @brief constructor
   * @param define a point's name
   */
  MotionRequestData(std::string point_name) : point_name_(point_name){}

  /**
   * @brief constructor
   * @param point_name define a point's name
   * @param type motion type for this point
   * @param speed motion speed
   */
  MotionRequestData(std::string point_name, rtp_gui::MotionType type, double speed)
    : point_name_(point_name), motion_tpye_(type), speed_(speed){}

  /**
   * @brief constructor
   * @param point_name define a point's name
   * @param type motion type for this point
   * @param speed motion speed
   * @param pose point's pose value
   */
  MotionRequestData(std::string point_name, rtp_gui::MotionType type, double speed, const geometry_msgs::Pose &pose)
    : point_name_(point_name), motion_tpye_(type), speed_(speed)
  {
    this->setPose(pose);
  }

  /**
   * @brief constructor
   * @param other another MotionRequestData param
   */
  MotionRequestData(const MotionRequestData &other)
  {
    this->pose_ = other.pose_;
    this->speed_ = other.speed_;
    this->point_name_ = other.point_name_;
    this->rpy_values_ = other.rpy_values_;
    this->motion_tpye_ = other.motion_tpye_;
    this->joint_values_ = other.joint_values_;
  }

  /**
   * @brief = operator implementation
   * @param another MotionRequestData param
   * @return
   */
  MotionRequestData& operator = (const MotionRequestData &other)
  {
    if (this != &other)
    {
      pose_ = other.pose_;
      speed_ = other.speed_;
      point_name_ = other.point_name_;
      rpy_values_ = other.rpy_values_;
      motion_tpye_ = other.motion_tpye_;
      joint_values_ = other.joint_values_;
    }
    return *this;
  }

  ~MotionRequestData(){}

  /**
   * @brief returns the point's pose value
   * @return pose of the point
   */
  geometry_msgs::Pose getPose() const
  {
    return this->pose_;
  }

  /**
   * @brief sets a point with a pose value
   * @param pose
   */
  void setPose(const geometry_msgs::Pose &pose)
  {
    this->pose_ = pose;
  }

  /**
   * @brief gets the point's name
   * @return point's name
   */
  std::string getPointName() const
  {
    return this->point_name_;
  }

  /**
   * @brief sets a point with a point's name
   * @param point's name
   */
  void setPointName(std::string name)
  {
    this->point_name_ = name;
  }

  /**
   * @brief sets a point with a motion type
   * @param motion type
   */
  void setMotionType(rtp_gui::MotionType mh)
  {
    this->motion_tpye_ = mh;
  }

  /**
   * @brief gets the point's motion type
   * @return motion type
   */
  rtp_gui::MotionType getMotionType() const
  {
    return this->motion_tpye_;
  }

  /**
   * @brief sets a point with joint values
   * @param joint values
   * @return true if value set, otherwise false(joint values empty)
   */
  bool setJointValues(const std::vector<double> &joint_value);

  /**
   * @brief gets the point's joint values
   * @return joint values
   */
  std::vector<double> getJointValues()
  {
    return  this->joint_values_;
  }

  /**
   * @brief gets the joint's speed
   * @return speed
   */
  double getSpeed() const
  {
    return this->speed_;
  }

  /**
   * @brief sets a point with speed
   * @param speed
   */
  void setSpeed(double speed)
  {
    this->speed_ = speed;
  }

  /**
   * @brief returns the point's rpy
   * @return RPY values
   */
  std::vector<double> getRPY() const
  {
    return this->rpy_values_;
  }

  /**
   * @brief sets a point with RPY
   * @param RPY values
   * @return true if value set, otherwise false(RPY size not equal to 3)
   */
  bool setRPY(const std::vector<double> &rpy);

private:

  /**
   * @brief point's pose value
   * typically, position in meter
   */
  geometry_msgs::Pose pose_;

  /**
   * @brief point's joint values
   * typically, in ras/s
   */
  std::vector<double> joint_values_;

  /**
   * @brief point's roll pitch yaw
   * typically, in rad
   */
  std::vector<double> rpy_values_;

  /**
   * @brief point's motion type
   * 1 for ptp movement, 2 for line movement
   */
  rtp_gui::MotionType motion_tpye_;

  /**
   * @brief point's name
   */
  std::string point_name_;

  /**
   * @brief point's motion speed
   * typically, varies from 0 to 1
   */
  double speed_;


};

}//end namespace rtp_gui
#endif // RTP_DATA_TYPE_H
