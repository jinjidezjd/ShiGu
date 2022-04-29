// 将离线三维点转换为点云数据

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <string>
#include <iostream>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
string frame_id = "map"; //所在帧
string file;
float zero_x, zero_y, zero_z; //原点的绝对坐标

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_conversion");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::Publisher pub = nh.advertise<PointCloud>("points2", 1);
    nh_priv.param("file_name", file, string("/home/sjtu/zjd/ShiGu/src/shigu_gazebo/pcl/result.txt"));
    ifstream myfile(file, ios::in); //只读
    string temp;
    if (!myfile)
    {
        cout << "未成功打开三维地图文件" << file << endl;
        return 0;
    }

    PointCloud::Ptr msg(new PointCloud);
    msg->header.frame_id = frame_id;
    msg->height = 1; //点云是无序的
    int len = 0;
    while (getline(myfile, temp)) //一次只读取一行
    {
        len++;
        string x, y, z;
        x = temp.substr(0, temp.find_first_of(','));
        y = temp.substr(temp.find_first_of(',') + 1, temp.find_last_of(',') - temp.find_first_of(',') - 1);
        z = temp.substr(temp.find_last_of(',') + 1);
        float point_x, point_y, point_z;
        point_x = stof(x) / 1000;
        point_y = stof(y) / 1000;
        point_z = stof(z) / 1000;
        if (len == 1)
        { //将第一个点作为坐标原点
            zero_x = point_x;
            zero_y = point_y;
            zero_z = point_z;
        }
        //获取相对坐标
        point_x = point_x - zero_x;
        point_y = point_y - zero_y;
        point_z = point_z - zero_z;
        //封装
        msg->points.push_back(pcl::PointXYZ(point_x, point_y, point_z));
        // cout << point_x << " " << point_y << " " << point_z << endl;
    }
    myfile.close(); //关闭文件
    msg->width = len;

    ros::Rate loop_rate(4);
    while (nh.ok())
    {
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}