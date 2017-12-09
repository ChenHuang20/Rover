/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file topics.h
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
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

    float att_p[3];
    float rate_p[3];
    float rate_d[3];
    float rate_i[3];
    float rate_i_max[3];
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
    uint8_t armed;
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
} actuator_t;

typedef struct {
    uint64_t timestamp;
    float wheel[4];
} wheel_velocity_t;

extern params_t _params;
extern accel_t _accel;
extern gyro_t _gyro;
extern radio_t _radio;
extern radio_raw_t _radio_raw;
extern attitude_t _attitude;
extern wheel_velocity_t _wheel_velocity;
extern actuator_t _actuator;

#ifdef __cplusplus
}
#endif
