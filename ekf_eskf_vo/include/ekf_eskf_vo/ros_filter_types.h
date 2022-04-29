#ifndef ROBOT_LOCALIZATION_ROS_FILTER_TYPES_H
#define ROBOT_LOCALIZATION_ROS_FILTER_TYPES_H

#include "ekf_eskf_vo/ros_filter.h"
#include "ekf_eskf_vo/ekf.h"
#include "ekf_eskf_vo/ukf.h"

namespace RobotLocalization
{

    typedef RosFilter<Ekf> RosEkf;
    typedef RosFilter<Ukf> RosUkf;

}

#endif
