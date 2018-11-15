/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file topics.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089@qq.com>
 *
 ***************************************************************************/

#include "topics.h"

params_t _params = {
    .accel_offset[0] = 0.6281335353333333f,
    .accel_offset[1] = 0.216268003f,
    .accel_offset[2] = -0.00599384308f,
    .accel_scale[1] = 1.0f,
    .accel_scale[0] = 1.0f,
    .accel_scale[2] = 1.0f,

    .gyro_offset[0] = -0.0022095706466667f,
    .gyro_offset[1] = 0.0047049589133333f,
    .gyro_offset[2] = 0.0001629428613333333f,
    .gyro_scale[0] = 1.0f,
    .gyro_scale[1] = 1.0f,
    .gyro_scale[2] = 1.0f,

    //0->wheel 1  1->wheel 2  2->wheel 3  3-> wheel 4
    .velocity_p[0] = 50.9f,//3.0f
    .velocity_p[1] = 50.9f,
    .velocity_p[2] = 50.9f,
    .velocity_p[3] = 50.9f,

    .velocity_i[0] = 0.0f,//0.3f
    .velocity_i[1] = 0.0f,
    .velocity_i[2] = 0.0f,
    .velocity_i[3] = 0.0f,

    .velocity_d[0] = 0.0f,//0.1f
    .velocity_d[1] = 0.0f,
    .velocity_d[2] = 0.0f,
    .velocity_d[3] = 0.0f,

    // pitch
    .att_p[1] = 1.2f,  //20.0f
    .rate_p[1] = 1.0f,//0.24f 1.00
    .rate_i[1] = 0.0f,  //0.5f
	.rate_d[1] = 0.0f,  //0.4f

    // yaw
    .att_p[2] =  0.0f,//15
    .rate_p[2] = 0.0f,//0.4f
    .rate_i[2] = 0.0f,//0.1f
	.rate_d[2] = 0.0f,// 0.0f

    // 底盘跟随
    .rotate_p = 5.0f, //7.5
    .rotate_i = 0.0f,
    .rotate_d = 0.001f,

    .tri_angle_p = 0.0f,
    .tri_angle_d = 0.0f,
    .tri_rate_p = 2.10f,
    .tri_rate_i = 0.0f,
    .tri_rate_d = 0.0f,
    .tri_rate_i_max = 0.0f,

    .gimbalfollow_p = 0.3f,
    .gimbalfollow_i = 0.11f,
    .gimbalfollow_d = 0.8f,
    .gimbalfollow_i_max = 1.0f,

};

//加速度
accel_t _accel = { 0 };

//陀螺仪
gyro_t _gyro = { 0 };

//遥控器原始接收数据
radio_raw_t _radio_raw = { 0 };

//遥控器最终数据
radio_t _radio = { 0 };

//车轮电机反馈速度
wheel_velocity_t _wheel_velocity = { 0 };

//车轮电机反馈电流
wheel_current_t _wheel_current = { 0 };

//云台电机反馈角度
gimbal_angle_t _gimbal_angle = { 0 };

//姿态最终数据
attitude_t _attitude = { 0 };

//激励值
actuator_t _actuator = { 0 };

//pid
pid_t _pid = { 0 };

motor_t _motor_trigger = { 0 };

tri_rates_t _tri_rates = { 0 };
