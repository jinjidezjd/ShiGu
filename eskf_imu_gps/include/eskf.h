#ifndef ESKF_H
#define ESKF_H

#include "../include/utils.h"

class ESKF
{
public:
    ESKF(double acc_n, double gyr_n, double acc_w, double gyr_w, Eigen::Vector3d p_IMU_GNSS);
    StatePtr state_ptr_;                //保存的状态
    Eigen::Vector3d p_I_GNSS_;          // gps坐标系在imu坐标系下的位置
    Eigen::Vector3d init_point_;        //初始坐标点
    bool initialized_;                  //是否被初始化
    std::deque<IMUDataPtr> imu_buffer_; //保存imu信息
    IMUDataPtr last_imu_ptr_;           //最新的imu信息

    bool process_IMU_Data(IMUDataPtr imu_data_ptr);                //处理imu信息
    bool process_GNSS_Data(GNSSDataPtr gnss_data_ptr);             //处理gps信息
    bool initialize(void);                                         //初始化
    void predict(IMUDataPtr last_imu_ptr, IMUDataPtr cur_imu_ptr); //预测环节
    void update(GNSSDataPtr gnss_data_ptr);                        //更新环节

private:
    double acc_noise_;       //加速度噪声
    double gyro_noise_;      //角速度噪声
    double acc_bias_noise_;  //加速度偏置噪声
    double gyro_bias_noise_; //角速度偏置噪声
};
using ESKFPtr = std::shared_ptr<ESKF>;

#endif // ESKF_H
