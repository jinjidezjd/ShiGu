#ifndef ROBOT_LOCALIZATION_ROS_FILTER_UTILITIES_H
#define ROBOT_LOCALIZATION_ROS_FILTER_UTILITIES_H

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Dense>

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#define RF_DEBUG(msg)        \
    if (filter_.getDebug())  \
    {                        \
        debugStream_ << msg; \
    }

// Handy methods for debug output
std::ostream &operator<<(std::ostream &os, const tf2::Vector3 &vec);
std::ostream &operator<<(std::ostream &os, const tf2::Quaternion &quat);
std::ostream &operator<<(std::ostream &os, const tf2::Transform &trans);
std::ostream &operator<<(std::ostream &os, const std::vector<double> &vec);

namespace RobotLocalization
{
    namespace RosFilterUtilities
    {

        double getYaw(const tf2::Quaternion quat);

        //! @brief Method for safely obtaining transforms.
        //! @param[in] buffer - tf buffer object to use for looking up the transform
        //! @param[in] targetFrame - The target frame of the desired transform
        //! @param[in] sourceFrame - The source frame of the desired transform
        //! @param[in] time - The time at which we want the transform
        //! @param[in] timeout - How long to block before falling back to last transform
        //! @param[out] targetFrameTrans - The resulting transform object
        //! @param[in] silent - Whether or not to print transform warnings
        //! @return Sets the value of @p targetFrameTrans and returns true if successful,
        //! false otherwise.
        //!
        //! This method attempts to obtain a transform from the @p sourceFrame to the @p
        //! targetFrame at the specific @p time. If no transform is available at that time,
        //! it attempts to simply obtain the latest transform. If that still fails, then the
        //! method checks to see if the transform is going from a given frame_id to itself.
        //! If any of these checks succeed, the method sets the value of @p targetFrameTrans
        //! and returns true, otherwise it returns false.
        //!
        bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
                                 const std::string &targetFrame,
                                 const std::string &sourceFrame,
                                 const ros::Time &time,
                                 const ros::Duration &timeout,
                                 tf2::Transform &targetFrameTrans,
                                 const bool silent = false);

        //! @brief Method for safely obtaining transforms.
        //! @param[in] buffer - tf buffer object to use for looking up the transform
        //! @param[in] targetFrame - The target frame of the desired transform
        //! @param[in] sourceFrame - The source frame of the desired transform
        //! @param[in] time - The time at which we want the transform
        //! @param[out] targetFrameTrans - The resulting transform object
        //! @param[in] silent - Whether or not to print transform warnings
        //! @return Sets the value of @p targetFrameTrans and returns true if successful,
        //! false otherwise.
        //!
        //! This method attempts to obtain a transform from the @p sourceFrame to the @p
        //! targetFrame at the specific @p time. If no transform is available at that time,
        //! it attempts to simply obtain the latest transform. If that still fails, then the
        //! method checks to see if the transform is going from a given frame_id to itself.
        //! If any of these checks succeed, the method sets the value of @p targetFrameTrans
        //! and returns true, otherwise it returns false.
        //!
        bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
                                 const std::string &targetFrame,
                                 const std::string &sourceFrame,
                                 const ros::Time &time,
                                 tf2::Transform &targetFrameTrans,
                                 const bool silent = false);

        //! @brief Utility method for converting quaternion to RPY
        //! @param[in] quat - The quaternion to convert
        //! @param[out] roll - The converted roll
        //! @param[out] pitch - The converted pitch
        //! @param[out] yaw - The converted yaw
        //!
        void quatToRPY(const tf2::Quaternion &quat, double &roll, double &pitch, double &yaw);

        //! @brief Converts our Eigen state vector into a TF transform/pose
        //! @param[in] state - The state to convert
        //! @param[out] stateTF - The converted state
        //!
        void stateToTF(const Eigen::VectorXd &state, tf2::Transform &stateTF);

        //! @brief Converts a TF transform/pose into our Eigen state vector
        //! @param[in] stateTF - The state to convert
        //! @param[out] state - The converted state
        //!
        void TFtoState(const tf2::Transform &stateTF, Eigen::VectorXd &state);

    } // namespace RosFilterUtilities
} // namespace RobotLocalization

#endif // ROBOT_LOCALIZATION_ROS_FILTER_UTILITIES_H