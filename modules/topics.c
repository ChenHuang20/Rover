/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file topics.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 *
 ***************************************************************************/

#include "topics.h"

params_t _params = {
    .accel_offset[0] = -0.0716258511,
    .accel_offset[1] = -0.0958640128,
    .accel_offset[2] = -0.216609955,
    .accel_scale[0] = 1.0f,
    .accel_scale[1] = 1.0f,
    .accel_scale[2] = 1.0f,

    .gyro_offset[0] = -0.00843819138,
    .gyro_offset[1] = 0.0334871039,
    .gyro_offset[2] = -0.0306651108,
    .gyro_scale[0] = 1.0f,
    .gyro_scale[1] = 1.0f,
    .gyro_scale[2] = 1.0f,

    // pitch
    .att_p[1] = 0.0f,
    .rate_p[1] = 0.28f,
    .rate_d[1] = 0.7f,
    .rate_i[1] = 0.0f,

    // yaw
    .att_p[2] = 0.0f,
    .rate_p[2] = 0.17f,
    .rate_d[2] = 0.7f,
    .rate_i[2] = 0.0f,
};

accel_t _accel = { 0 };
gyro_t _gyro = { 0 };
radio_raw_t _radio_raw = { 0 };
radio_t _radio = { 0 };
wheel_velocity_t _wheel_velocity = { 0 };
attitude_t _attitude = { 0 };
actuator_t _actuator = { 0 };
