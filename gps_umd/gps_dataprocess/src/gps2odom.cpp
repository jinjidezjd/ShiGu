#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>

using namespace gps_common;
using namespace std;

static ros::Publisher odom_pub;
std::string frame_id = "odom"; //父帧
// std::string child_frame_id = "gps_Link"; //子帧，仿真中
std::string child_frame_id = "imu_link"; //子帧，kitti数据集
double rot_cov = 99999.0;                //协方差，表明获取到的角度不可信
bool append_zone = false;
double northing_start, easting_start, altitude_start; //起点的坐标，作为基准

void callback(const sensor_msgs::NavSatFixConstPtr &fix)
{
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX)
  {
    ROS_DEBUG_THROTTLE(60, "GPS ");
    return;
  }
  if (fix->header.stamp == ros::Time(0))
  {
    return;
  }
  static int is_start = 0;

  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);
  //获取起点坐标
  if (is_start == 0)
  {
    is_start = 1;
    northing_start = northing;
    easting_start = easting;
    altitude_start = fix->altitude;
  }

  if (odom_pub)
  {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;
    //自己修改
    // odom.pose.pose.position.x = northing-northing_start;
    // odom.pose.pose.position.y = -(easting-easting_start);
    //原始
    odom.pose.pose.position.x = (easting - easting_start);
    odom.pose.pose.position.y = (northing - northing_start);

    odom.pose.pose.position.z = fix->altitude - altitude_start;

    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;

    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{fix->position_covariance[0],
                                            fix->position_covariance[1],
                                            fix->position_covariance[2],
                                            0, 0, 0,
                                            fix->position_covariance[3],
                                            fix->position_covariance[4],
                                            fix->position_covariance[5],
                                            0, 0, 0,
                                            fix->position_covariance[6],
                                            fix->position_covariance[7],
                                            fix->position_covariance[8],
                                            0, 0, 0,
                                            0, 0, 0, rot_cov, 0, 0,
                                            0, 0, 0, 0, rot_cov, 0,
                                            0, 0, 0, 0, 0, rot_cov}};
    odom.pose.covariance = covariance;

    odom_pub.publish(odom);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps2odom");
  ros::NodeHandle node;
  odom_pub = node.advertise<nav_msgs::Odometry>("/gps/odom", 10);
  ros::Subscriber fix_sub = node.subscribe("/navsat/fix", 10, callback);
  ros::spin();
}
