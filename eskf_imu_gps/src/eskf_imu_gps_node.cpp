#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include "../include/eskf.h"

class ESKF_Fusion
{
public:
    ESKF_Fusion(ros::NodeHandle nh, ros::NodeHandle nh_priv)
    {
        //加速度噪声,磁力计噪声,加速度偏置噪声,磁力计偏置噪声
        double acc_n, gyr_n, acc_w, gyr_w;
        nh_priv.param("acc_noise", acc_n, 1e-2);
        nh_priv.param("gyr_noise", gyr_n, 1e-4);
        nh_priv.param("acc_bias_noise", acc_w, 1e-6);
        nh_priv.param("gyr_bias_noise", gyr_w, 1e-8);
        //是否发布tf
        nh_priv.param("publish_tf", publish_tf, false);
        // gps坐标系在imu坐标系下的位置
        double x, y, z;
        nh_priv.param("p_I_GNSS_x", x, 0.);
        nh_priv.param("p_I_GNSS_y", y, 0.);
        nh_priv.param("p_I_GNSS_z", z, 0.);
        const Eigen::Vector3d p_I_GNSS(x, y, z);
        //封装
        eskf_ptr_ = std::make_shared<ESKF>(acc_n, gyr_n, acc_w, gyr_w, p_I_GNSS);

        // 订阅\发布话题
        imu_sub_ = nh.subscribe("/imu/data", 10, &ESKF_Fusion::imu_callback, this);
        gnss_sub_ = nh.subscribe("/fix", 10, &ESKF_Fusion::gnss_callback, this);

        path_pub_ = nh.advertise<nav_msgs::Path>("eskf_path", 10);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("eskf_odom", 10);

        // log files
        file_gnss_.open("/home/rick/gnss.csv");
        file_state_.open("/home/rick/fused_state.csv");

        std::cout << "[ ESKF ] Start." << std::endl;
    }
    ~ESKF_Fusion()
    {
        file_gnss_.close();
        file_state_.close();
    }
    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
    void gnss_callback(const nav_msgs::OdometryConstPtr &gnss_msg);
    void publish_save_state(void);

private:
    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_sub_;
    ros::Publisher path_pub_;
    ros::Publisher odom_pub_;
    nav_msgs::Path nav_path_;
    bool publish_tf;
    tf::Transform transform;
    tf::TransformBroadcaster odom_broadcaster_;
    ESKFPtr eskf_ptr_;

    // log files
    std::ofstream file_gnss_;
    std::ofstream file_state_;
};

void ESKF_Fusion::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    // imu数据回调函数
    //数据封装
    IMUDataPtr imu_data_ptr = std::make_shared<IMUData>();
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    //获得加速度和角速度
    imu_data_ptr->acc[0] = imu_msg->linear_acceleration.x;
    imu_data_ptr->acc[1] = imu_msg->linear_acceleration.y;
    imu_data_ptr->acc[2] = imu_msg->linear_acceleration.z;
    imu_data_ptr->gyro[0] = imu_msg->angular_velocity.x;
    imu_data_ptr->gyro[1] = imu_msg->angular_velocity.y;
    imu_data_ptr->gyro[2] = imu_msg->angular_velocity.z;
    //执行预测环节
    if (!eskf_ptr_->process_IMU_Data(imu_data_ptr))
        return;
}

void ESKF_Fusion::gnss_callback(const nav_msgs::OdometryConstPtr &gnss_msg)
{
    //数据封装
    GNSSDataPtr gnss_data_ptr = std::make_shared<GNSSData>();
    gnss_data_ptr->timestamp = gnss_msg->header.stamp.toSec();
    //经纬度
    gnss_data_ptr->point[0] = gnss_msg->pose.pose.position.x;
    gnss_data_ptr->point[1] = gnss_msg->pose.pose.position.y;
    gnss_data_ptr->point[2] = gnss_msg->pose.pose.position.z;
    gnss_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gnss_msg->pose.covariance.data()).block<3, 3>(0, 0);
    //执行观测更新环节
    if (!eskf_ptr_->process_GNSS_Data(gnss_data_ptr))
        return;
    file_gnss_ << std::fixed << std::setprecision(15)
               << gnss_data_ptr->timestamp << ", "
               << gnss_data_ptr->point[0] << ", "
               << gnss_data_ptr->point[1] << ", "
               << gnss_data_ptr->point[2] << std::endl;
    //如果速度超过阈值,初始化滤波器，但gps起点不变
    if (abs(eskf_ptr_->state_ptr_->v_G_I(0, 0)) >= 20 || abs(eskf_ptr_->state_ptr_->v_G_I(1, 0)) >= 20 || abs(eskf_ptr_->state_ptr_->v_G_I(2, 0)) >= 20)
    {
        eskf_ptr_->initialize();                     //初始化滤波器
        eskf_ptr_->process_GNSS_Data(gnss_data_ptr); //重新执行观测更新环节
        return;
    }
    //发布话题
    publish_save_state();
}

void ESKF_Fusion::publish_save_state(void)
{
    //发布话题
    // 里程计odometry
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.header.stamp = ros::Time::now();
    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    T_wb.linear() = eskf_ptr_->state_ptr_->R_G_I;
    T_wb.translation() = eskf_ptr_->state_ptr_->p_G_I;
    tf::poseEigenToMsg(T_wb, odom_msg.pose.pose);
    tf::vectorEigenToMsg(eskf_ptr_->state_ptr_->v_G_I, odom_msg.twist.twist.linear);
    Eigen::Matrix3d P_pp = eskf_ptr_->state_ptr_->cov.block<3, 3>(0, 0); // position covariance
    Eigen::Matrix3d P_po = eskf_ptr_->state_ptr_->cov.block<3, 3>(0, 6); // position rotation covariance
    Eigen::Matrix3d P_op = eskf_ptr_->state_ptr_->cov.block<3, 3>(6, 0); // rotation position covariance
    Eigen::Matrix3d P_oo = eskf_ptr_->state_ptr_->cov.block<3, 3>(6, 6); // rotation covariance
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> P_imu_pose = Eigen::Matrix<double, 6, 6>::Zero();
    P_imu_pose << P_pp, P_po, P_op, P_oo;
    for (int i = 0; i < 36; i++)
        odom_msg.pose.covariance[i] = P_imu_pose.data()[i];
    odom_pub_.publish(odom_msg);

    // 路径path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg.header;
    pose_stamped.pose = odom_msg.pose.pose;
    nav_path_.header = pose_stamped.header;
    nav_path_.poses.push_back(pose_stamped);
    path_pub_.publish(nav_path_);

    //发布tf
    //发布tf
    if (publish_tf)
    {
        tf::Vector3 tf_pose;
        transform.setOrigin(tf::Vector3(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z));
        tf::Quaternion q;
        tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, q);
        q.normalized();
        transform.setRotation(q);
        odom_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
    }

    // 保存经纬度
    Eigen::Vector3d point;
    convert_enu_to_point(eskf_ptr_->init_point_, eskf_ptr_->state_ptr_->p_G_I, &point);
    Eigen::Quaterniond q_G_I(eskf_ptr_->state_ptr_->R_G_I);
    file_state_ << std::fixed << std::setprecision(15) << eskf_ptr_->state_ptr_->timestamp << ", "
                << eskf_ptr_->state_ptr_->p_G_I[0] << ", " << eskf_ptr_->state_ptr_->p_G_I[1] << ", " << eskf_ptr_->state_ptr_->p_G_I[2] << ", "
                << q_G_I.x() << ", " << q_G_I.y() << ", " << q_G_I.z() << ", " << q_G_I.w() << ", "
                << point[0] << ", " << point[1] << ", " << point[2]
                << std::endl;
    // std::cout << "bias of acc: " << eskf_ptr_->state_ptr_->acc_bias.transpose() << std::endl;
    // std::cout << "bias of gyr: " << eskf_ptr_->state_ptr_->gyro_bias.transpose() << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eskf_imu_gps_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ESKF_Fusion node(nh, nh_priv); //类对象实例化
    ros::spin();
}
