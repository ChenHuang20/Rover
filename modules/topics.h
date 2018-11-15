/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file topics.h
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089@qq.com>
 *
 ***************************************************************************/

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float accel_offset[3];
    float accel_scale[3];
    float gyro_offset[3];
    float gyro_scale[3];

    float velocity_p[4];
    float velocity_i[4];
    float velocity_d[4];

    float att_p[3];
    float rate_p[3];
    float rate_d[3];
    float rate_i[3];
    float rate_i_max[3];

    float rotate_p;
    float rotate_i;
    float rotate_d;
    float rotate_i_max;

    float tri_angle_p;
    float tri_angle_d;
    float tri_rate_p;
    float tri_rate_d;
    float tri_rate_i;
    float tri_rate_i_max;

    float rorate_std;

    float gimbalfollow_p;
    float gimbalfollow_i;
    float gimbalfollow_d;
    float gimbalfollow_i_max;
    
    float maxpower;
} params_t;

typedef struct {
    uint64_t timestamp;
    float temperature;
    int16_t x_raw, y_raw, z_raw;
    float x, y, z;
} accel_t;

typedef struct {
    uint64_t timestamp;
    float temperature;
    int16_t x_raw, y_raw, z_raw;
    float x, y, z;
} gyro_t;

typedef struct {
    uint64_t timestamp;

    struct {
        int16_t ch0;
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
        uint8_t s1;
        uint8_t s2;
    } rc;

    struct {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    struct {
        uint16_t v;
    } key;

	uint32_t lastUpdate;

    float quality;
} radio_raw_t;

typedef struct {
    uint64_t timestamp;
    float x;
    float y;
    float pitch;
    float yaw;
    uint8_t mode;
    uint8_t shoot;
} radio_t;

typedef struct {
    uint64_t timestamp;
    float euler[3];
    float rates[3];
} attitude_t;

typedef struct {
    uint64_t timestamp;
    float gimbal[3];
    float wheel[4];
	float rotate;
    float trigger;
} actuator_t;

typedef struct {
    uint64_t timestamp;
    float wheel[4];
} wheel_velocity_t;

typedef struct {
    uint64_t timestamp;
    float trigger;
} tri_rates_t;

typedef struct {
    uint64_t timestamp;
    float wheel[4];
    float power_sum;
    float max_sum;
} wheel_current_t;

typedef struct {
    uint64_t timestamp;
    float gimbal[2];
} gimbal_angle_t;

typedef struct
{
    int16_t  speed_rpm;

    int16_t  given_current;
    uint8_t  hall;

    uint16_t offset_angle;
    int32_t round_cnt;
    uint32_t msg_cnt;
    int32_t total_ecd;
    int32_t total_angle;

    uint16_t angle; //abs angle range:[0,8191]
    uint16_t last_angle; //abs angle range:[0,8191]

} motor_t;


typedef struct {
    uint64_t timestamp;
    float _euler_sp[3];
    float _rates_sp[3];
    float _velocity_sp[4];
    float _rotate_sp;
    float _tri_angle_sp;
    float _tri_rates_sp;
    float _gimbalfollow_angle_sp;
    float _gimbalfollow_rate_sp;

    float _euler_prev[3];
    float _rates_prev[3];
    float _velocity_prev[3];
    float _rotate_prev;
    float _tri_angle_prev;
    float _tri_rates_prev;
    float _gimbalfollow_prev;

    float _rates_int[3];
    float _euler_int[3];
    float _velocity_int[3];
    float _rotate_int;
    float _tri_rates_int;
    float _tri_angle_int;
    float _gimbalfollow_int;
    
    float err_wheel_now[4];
    float err_wheel_prev[4];
    float err_wheel_pprev[4];
    float err_gimbal_now[4];
    float err_gimbal_prev[4];
    float err_gimbal_pprev[4];
    float err_rotate_now;
    float err_rotate_prev;
    float err_rotate_pprev;
    float err_tri_rates_now;
    float err_tri_tates_prev;
    float err_tri_rates_pprev;
    float err_gimbalfollow_now;
    float err_gimbalfollow_prev;
    float err_gimbalfollow_pprev;

    float delta_wheel[4];
    float output_wheel[4];
    float output_gimbal[3];
    float output_rotate;
    float output_trigger;
    float output_tri_angle;
    float output_gimbalfollow;
} pid_t;

extern pid_t _pid;
extern gyro_t _gyro;
extern accel_t _accel;
extern radio_t _radio;
extern params_t _params;
extern actuator_t _actuator;
extern attitude_t _attitude;
extern radio_raw_t _radio_raw;
extern gimbal_angle_t _gimbal_angle;
extern wheel_current_t _wheel_current;
extern wheel_velocity_t _wheel_velocity;

extern motor_t _motor_trigger;
extern tri_rates_t _tri_rates;

#ifdef __cplusplus
}
#endif
