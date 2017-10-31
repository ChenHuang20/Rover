/*      厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file topics.h
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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

extern radio_raw_t _radio_raw;

typedef struct {
    uint64_t timestamp;
    float x;
    float y;
    float pitch;
    float yaw;
    uint8_t armed;
    uint8_t shoot;
} radio_t;

extern radio_t _radio;

typedef struct {
    uint64_t timestamp;
    float wheel[4];
} wheel_velocity_t;

extern wheel_velocity_t _wheel_velocity;

typedef struct {
    uint64_t timestamp;
    float wheel[4];
} wheel_velocity_setpoint_t;

extern wheel_velocity_setpoint_t _wheel_velocity_setpoint;

#ifdef __cplusplus
}
#endif
