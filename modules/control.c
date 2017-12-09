/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file control.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 *
 ***************************************************************************/

#include "control.h"

#include <math.h>
#include <string.h>

#include "scheduler.h"
#include "topics.h"

#ifndef M_DEG_TO_RAD
#define M_DEG_TO_RAD    0.01745329251994329576923690768489f
#endif

#ifndef M_RAD_TO_DEG
#define M_RAD_TO_DEG    57.295779513082320876798154814105f
#endif

float _att_sp[3];
float _rates_prev[3];
float _rates_sp[3];
float _rates_sp_prev[3];
float _rates_int[3];

float _att_control[3];

const float rc_ctrl_rate[3] = { 2.0f, 2.0f, 2.0f };

void gimbal_control()
{
    uint64_t t = time();

    static uint64_t t_prev = 0;

    float dt = t_prev > 0 ? (t - t_prev) * 1e-6f : 4e-3f;
    dt = dt < 0.002f ? 0.002f : dt > 0.02f ? 0.02f : dt;

    t_prev = t;

    float att[3] = { _attitude.euler[0], _attitude.euler[1], _attitude.euler[2] };
    float rates[3] = { _attitude.rates[0], _attitude.rates[1], _attitude.rates[2] };

    if (fabsf(_radio.pitch) > 0.05f) {
        _att_sp[1] = att[1];
        _rates_sp[1] = _radio.pitch * rc_ctrl_rate[1];

    } else {
        _rates_sp[1] = (_att_sp[1] - att[1]) * _params.att_p[1];
    }

    if (fabsf(_radio.yaw) > 0.05f) {
        _att_sp[2] = att[2];
        _rates_sp[2] = _radio.yaw * rc_ctrl_rate[2];

    } else {
        _rates_sp[2] = (_att_sp[2] - att[2]) * _params.att_p[2];
    }

    float output[3];

    for (int i = 0; i < 3; i++) {

        float err = _rates_sp[i] - rates[i];

        output[i] = err * _params.rate_p[i] + _rates_int[i] + (_rates_prev[i] - rates[i]) * _params.rate_d[i];
        output[i] = fmaxf(-1.0f, fminf(1.0f, output[i]));

        _rates_prev[i] = rates[i];

        _rates_int[i] += err * _params.rate_i[i];
        _rates_int[i] = fmaxf(-_params.rate_i_max[i], fminf(_params.rate_i_max[i], _rates_int[i]));
    }

    _actuator.timestamp = t;

    for (int i = 0; i < 3; i++) {
        _actuator.gimbal[i] = output[i] * 5000.0f;
    }
}

void control_task()
{
    gimbal_control();
}
