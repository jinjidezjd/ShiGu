// 订阅rviz的/initialpose话题，发布map->odom的静态变换

#include <iostream>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

geometry_msgs::TransformStamped transformStamped;
geometry_msgs::PoseWithCovarianceStamped temp;
void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    temp = *msg;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "initial_localization");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, callback);

    tf2_ros::TransformBroadcaster br;

    //初始位置
    // transformStamped.transform.translation.x = 0;
    // transformStamped.transform.translation.y = 0;
    // transformStamped.transform.translation.z = 0;
    // transformStamped.transform.rotation.x = 0;
    // transformStamped.transform.rotation.y = 0;
    // transformStamped.transform.rotation.z = 0;
    // transformStamped.transform.rotation.w = 1;

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        ros::spinOnce();
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "odom";
        transformStamped.transform.translation.x = temp.pose.pose.position.x;
        transformStamped.transform.translation.y = temp.pose.pose.position.y;
        transformStamped.transform.translation.z = temp.pose.pose.position.z;
        transformStamped.transform.rotation = temp.pose.pose.orientation;
        br.sendTransform(transformStamped);
    }
}