#ifndef RTP_GUI_CONST_H
#define RTP_GUI_CONST_H

#include <vector>

namespace rtp_gui {
//const double JOINT_SPEED_LIMIT_CONST = 1.57; //rad/s (maxin)
//const double JOINT_SPEED_DEFAULT_CONST = 0.78; //rad/s (default)


const std::vector<double> JOINT_SPEED_LIMIT_CONST{6.632,6.109,6.981,7.854,7.854,13.963}; // rad/s for each joint according to urdf
const double JOGGING_SPEED_DEFAULT_RATIO_CONST = 0.2;//default joint jogging axis speed
const double CART_DURATION_DEFAULT_CONST = 0.05; //second
} //end namespace rtp_gui

#endif // RTP_GUI_CONST_H
