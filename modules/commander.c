/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file commander.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089@qq.com>
 *
 ***************************************************************************/

#include "commander.h"

#include <math.h>
#include <string.h>
#include <stdint.h>

#include "scheduler.h"
#include "topics.h"
#include "flash.h"
//#include "led.h"
//#include "adc.h"

#ifndef M_ONE_G
#define M_ONE_G 9.80665f
#endif

static uint8_t _calibrate = 0;

float _acc_filter;
float _battery_filter;

// mode: < -1 | -1 | 0           | 1          | > 1
// leds: off  | on | quick flash | slow flash | flash n times
void led_indicate(int mode)
{
    static int times = 0;
    static uint64_t t_start = 0;
    static uint64_t t_prev = 0;

    uint64_t t = time();

    uint64_t t_on, t_off, t_delay;

    char path[3] = "led";
    uint8_t led[4];

    if (times != mode) {
        times = mode;
        t_start = t;
        t_prev = t;
    }

    if (times < -1) {
        t_on = 0;
        t_off = 0;
        t_delay = UINT32_MAX;

    } else if (times < 0) {
        t_on = 0;
        t_off = 0;
        t_delay = 0;

    } else if (times < 2) {
        t_on = times ? 1.2e6 : 6e4;
        t_off = t_on;
        t_delay = t_off << 1;

    } else {
        t_on = 1.2e5;
        t_off = t_on * (times << 1);
        t_delay = t_off + 1.5e6;
    }

    if (t > t_start + t_delay) {
        // turn on
        led[0] = 1;
        led[1] = 1;
        led[2] = 1;
        led[3] = 1;

        write(path, led, sizeof(led));

        t_start = t;
        t_prev = t;

    } else if (t > t_start + t_off) {
        // turn on
        led[0] = 0;
        led[1] = 0;
        led[2] = 0;
        led[3] = 0;

        write(path, led, sizeof(led));

    } else if (t > t_prev + t_on) {
        // toggle
        led[0] = 2;
        led[1] = 2;
        led[2] = 2;
        led[3] = 2;

        write(path, led, sizeof(led));

        t_prev = t;
    }
}

static void calibrate()
{
    static int count = 0;
    static float accel_sample[3] = { 0.0f };
    static float gyro_sample[3] = { 0.0f };

    const int samples = 300;

    if (_calibrate) {
        if (!count) {
            gyro_sample[0] = _gyro.x;
            gyro_sample[1] = _gyro.y;
            gyro_sample[2] = _gyro.z;

            accel_sample[0] = _accel.x;
            accel_sample[1] = _accel.y;
            accel_sample[2] = _accel.z;

            count++;

        } else {
            
            gyro_sample[0] += _gyro.x;
            gyro_sample[1] += _gyro.y;
            gyro_sample[2] += _gyro.z;

            accel_sample[0] += _accel.x;
            accel_sample[1] += _accel.y;
            accel_sample[2] += _accel.z;

            count++;

            if (count == samples) {
                for (int i = 0; i < 3; i++) {
                    gyro_sample[i] /= (float)samples;
                    accel_sample[i] /= (float)samples;
                }

                accel_sample[2] += M_ONE_G;

                _params.gyro_offset[0] += gyro_sample[0];
                _params.gyro_offset[1] += gyro_sample[1];
                _params.gyro_offset[2] += gyro_sample[2];

                _params.accel_offset[0] += accel_sample[0];
                _params.accel_offset[1] += accel_sample[1];
                _params.accel_offset[2] += accel_sample[2];

                count = 0;

                _calibrate = 0;

                // save parameters to flash
                flash_save();
            }
        }
    }
}

static void commander()
{
    uint64_t t = time();
    
    static uint64_t count = 0;
    
    count++;

    // calibration
    static uint64_t calibrate_timestamp = 0;

    if (!_calibrate && _radio.pitch > 0.9f && _radio.yaw < -0.9f && _radio.y < -0.9f && _radio.x < -0.9f) {
        if (t > calibrate_timestamp + 500000) {
            _calibrate = 1;
        }

    } else {
        calibrate_timestamp = t;
    }

//    if(count == 40)
//        _calibrate = 1;

    calibrate();
}

void commander_task()
{
    commander();
}
