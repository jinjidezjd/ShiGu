// 将EU数据集话题时间戳进行转换,转换到当前时间戳
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

static ros::Publisher cloud_pub;
static ros::Publisher laser_pub;
static ros::Publisher imu_pub;
void callback_cloud(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    sensor_msgs::PointCloud2 data(*msg);
    data.header.stamp = ros::Time::now();
    cloud_pub.publish(data);
}
void callback_scan(const sensor_msgs::LaserScanConstPtr &msg)
{
    sensor_msgs::LaserScan data(*msg);
    data.header.stamp = ros::Time::now();
    laser_pub.publish(data);
}
void callback_imu(const sensor_msgs::ImuConstPtr &msg)
{
    sensor_msgs::Imu data(*msg);
    std::cout << "im here!" << std::endl;
    data.header.stamp = ros::Time::now();
    data.header.frame_id = "imu_Link";
    imu_pub.publish(data);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dataset_tampstamp_transform");
    ros::NodeHandle node;
    //点云数据
    cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/cloud/new", 10);
    ros::Subscriber cloud_sub = node.subscribe("/cloud", 10, callback_cloud);
    //激光数据
    laser_pub = node.advertise<sensor_msgs::LaserScan>("/scan/new", 10);
    ros::Subscriber laser_sub = node.subscribe("/scan", 10, callback_scan);
    //imu数据
    imu_pub = node.advertise<sensor_msgs::Imu>("/imu/data_new", 10);
    ros::Subscriber imu_sub = node.subscribe("/imu/data", 10, callback_imu);
    ros::spin();
}