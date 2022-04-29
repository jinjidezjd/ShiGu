// 将点云数据转为栅格地图

#include <iostream>
#include <fstream>
#include <stdint.h>
#include <math.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;

using namespace std;

/* 全局变量 */
PointCloud::ConstPtr currentPC;
bool newPointCloud = false;

double cellResolution; //栅格的分辨率,单位m
double high_threshold; //最高阈值，单位m
double low_threshold;  //最低阈值，单位m
int buffer_size;       //每个栅格中超过高度阈值的数量
ros::Publisher pub;

void calcSize(double *xMax, double *yMax, double *xMin, double *yMin)
{
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D(*currentPC, minPt, maxPt);

    *xMax = maxPt.x;
    *yMax = maxPt.y;
    *xMin = minPt.x;
    *yMin = minPt.y;
}

//得到栅格地图
void computeGrid(std::vector<signed char> &ocGrid, double xMin, double yMin, int xCells, int yCells)
{
    // cout << "开始计算法线" << endl;
    //计算点云的法线
    NormalCloud::Ptr cloud_normals(new NormalCloud);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(currentPC);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.06);
    ne.compute(*cloud_normals);

    // cout << "判断法线是否垂直" << endl;

    int size = xCells * yCells;
    std::vector<int> countGrid(size);

    //判断每个点云的法向量是否垂直
    for (size_t i = 0; i < currentPC->size(); i++)
    {
        double x = currentPC->points[i].x;
        double y = currentPC->points[i].y;
        double z = currentPC->points[i].z;
        // cout << x << " " << y << " " << z << endl;
        int xc = (int)((x - xMin) / cellResolution); //取整后默认按照cellResolution将点分配到ｃｅｌｌ
        int yc = (int)((y - yMin) / cellResolution);

        if (z > high_threshold || z < low_threshold) //如果高于阈值或者低于阈值
        {
            countGrid[yc * xCells + xc]++; //统计一个ｃｅｌｌ中垂直方向满足条件的点数
            // cout << "不满足要求" << endl;
        }
    }

    // cout << "计算占据概率" << endl;

    //根据阈值计算占据概率
    for (int i = 0; i < size; i++) // size:xCells * yCells
    {
        if (countGrid[i] < buffer_size && countGrid[i] > 0)
            ocGrid[i] = 0;
        else if (countGrid[i] > buffer_size)
            ocGrid[i] = 100;
        else if (countGrid[i] == 0)
            ocGrid[i] = 0; // TODO Should be -1
        // cout << countGrid[i] << endl;
    }
}

void updateGrid(nav_msgs::OccupancyGridPtr grid, int xCells, int yCells,
                double originX, double originY, std::vector<signed char> *ocGrid)
{
    static int seq = 0;

    grid->header.frame_id = "map";
    grid->header.seq = seq++;
    grid->header.stamp.sec = ros::Time::now().sec;
    grid->header.stamp.nsec = ros::Time::now().nsec;
    grid->info.map_load_time = ros::Time::now();
    grid->info.resolution = cellResolution;
    grid->info.width = xCells;
    grid->info.height = yCells;
    grid->info.origin.position.x = originX; // minx
    grid->info.origin.position.y = originY; // miny
    grid->info.origin.position.z = 0;
    grid->info.origin.orientation.w = 1;
    grid->info.origin.orientation.x = 0;
    grid->info.origin.orientation.y = 0;
    grid->info.origin.orientation.z = 0;
    grid->data = *ocGrid;
}

void callback(const PointCloud::ConstPtr &msg)
{
    currentPC = msg;
    // ROS_INFO_STREAM("Convertor节点——接收到点云");

    /*计算点云的最大和最小值*/
    double xMax = 0, yMax = 0, xMin = 0, yMin = 0;
    calcSize(&xMax, &yMax, &xMin, &yMin);

    // cout << "极值：" << xMax << " " << yMax << " " << xMin << " " << yMin << " " << endl;

    /* 确定栅格地图的长和宽 */
    int xCells = ((int)((xMax - xMin) / cellResolution)) + 1;
    int yCells = ((int)((yMax - yMin) / cellResolution)) + 1;

    // cout << "地图大小：" << xCells << " " << yCells << endl;

    /*计算栅格地图*/
    std::vector<signed char> ocGrid(yCells * xCells); //存储每个ｃｅｌｌ的值　　０或者１００
    computeGrid(ocGrid, xMin, yMin, xCells, yCells);

    // cout << "成功计算得到栅格地图" << endl;

    //发布地图消息
    nav_msgs::OccupancyGridPtr grid(new nav_msgs::OccupancyGrid);
    updateGrid(grid, xCells, yCells, xMin, yMin, &ocGrid);
    pub.publish(grid);
    ROS_INFO_STREAM("map_conversion节点——发布栅格地图");
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "map_conversion");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::Subscriber sub = nh.subscribe<PointCloud>("/points2", 1, callback);
    pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    nh_priv.param("cellResolution", cellResolution, 0.3);
    nh_priv.param("high_threshold", high_threshold, 0.4);
    nh_priv.param("low_threshold", low_threshold, 0.0);
    nh_priv.param("buffer_size", buffer_size, 1);

    //构造占据网格消息
    nav_msgs::OccupancyGridPtr grid(new nav_msgs::OccupancyGrid);
    grid->header.seq = 1;
    grid->header.frame_id = "map"; //父坐标系
    grid->info.origin.position.z = 0;
    grid->info.origin.orientation.w = 1;
    grid->info.origin.orientation.x = 0;
    grid->info.origin.orientation.y = 0;
    grid->info.origin.orientation.z = 0;

    // ROS_INFO_STREAM("Convertor节点初始化完成");
    ros::Rate loop_rate(0.2);
    ros::Duration t(10);
    while (ros::ok())
    {
        ros::spinOnce();

        t.sleep();
    }
}