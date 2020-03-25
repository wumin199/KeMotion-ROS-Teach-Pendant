#ifndef RTP_GUI_CONST_H
#define RTP_GUI_CONST_H

#include <vector>

namespace rtp_gui {
//const double JOINT_SPEED_LIMIT_CONST = 1.57; //rad/s (maxin)
//const double JOINT_SPEED_DEFAULT_CONST = 0.78; //rad/s (default)

#define er7 //#define er12

#ifdef er7
//er7
const std::vector<double> JOINT_SPEED_LIMIT_CONST{6.632,6.109,6.981,7.854,7.854,13.963}; // rad/s for each joint according to urdf
const std::vector<double> LIN_DYNAMICS_CONST{3000.0, 18000.0, 18000.0, 540000.0, 360.0, 720.0, 720.0, 7200.0};//velCart(mm/s), accCart(mm/s2), decCart(mm/s2), jerkCart(mm/s3), velOri(degree/s), accOri(degree/s2), decOri(degree/s2), jerkOri(degree/s3)
const std::vector<double> DEFAULT_VOICE_TEST_POINT{0,-0.349,0,0,-1.047,0};//rad
#endif

#ifdef er12
//er12
const std::vector<double> JOINT_SPEED_LIMIT_CONST{6.9813,6.9813,4.7123889,4.712,12.217,13.614}; // rad/s for each joint according to urdf
const std::vector<double> LIN_DYNAMICS_CONST{3000.0, 10000.0, 10000.0, 200000.0, 360.0, 1800.0, 1800.0, 100000.0};//velCart(mm/s), accCart(mm/s2), decCart(mm/s2), jerkCart(mm/s3), velOri(degree/s), accOri(degree/s2), decOri(degree/s2), jerkOri(degree/s3)
const std::vector<double> DEFAULT_VOICE_TEST_POINT{0,0.349,0,0,1.047,0};//rad
#endif

const double JOGGING_SPEED_DEFAULT_RATIO_CONST = 0.2;//default joint jogging axis speed
const double CART_DURATION_DEFAULT_CONST = 0.05; //second
} //end namespace rtp_gui

#endif // RTP_GUI_CONST_H
