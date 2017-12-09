/***************************************************************************
 *
 *  Copyright (c) 2017 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file estimator.c
 *
 * @author zwh <zwh@raaworks.com>
 *
 * @data 2017.10.27
 */

#include <math.h>
#include <stdint.h>

static const float accel_gain = 0.2f;
static const float gyro_bias_gain = 0.1f;

static float q[4];
static float gyro_bias[3];

static uint8_t inited = 0;

/*
 * brief: generate quaternion from body acceleration
 *
 * @param accel: body acceleration in m/s^2
 */
static uint8_t init(const float accel[])
{
    float length = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

    float z[3] = { -accel[0] / length, -accel[1] / length, -accel[2] / length };
    float x[3] = { sqrt(1.0f - z[0] * z[0]), 0.0f, -z[0] };
    float y[3] = {
        z[1] * x[2] - z[2] * x[1],
        z[2] * x[0] - z[0] * x[2],
        z[0] * x[1] - z[1] * x[0],
    };

    float R[3][3] = {
        { x[0], x[1], x[2] },
        { y[0], y[1], y[2] },
        { z[0], z[1], z[2] }
    };

    float tr = R[0][0] + R[1][1] + R[2][2];

    if (tr > 0.0f) {
        float s = sqrtf(tr + 1.0f);
        q[0] = s * 0.5f;
        s = 0.5f / s;
        q[1] = (R[2][1] - R[1][2]) * s;
        q[2] = (R[0][2] - R[2][0]) * s;
        q[3] = (R[1][0] - R[0][1]) * s;

    } else {
        int i = 0;

        for (int n = 1; n < 3; n++) {
            if (R[n][n] > R[i][i]) {
                i = n;
            }
        }

        int j = (i + 1) % 3;
        int k = (i + 2) % 3;
        float s = sqrtf(R[i][i] - R[j][j] - R[k][k] + 1.0f);

        q[i + 1] = s * 0.5f;
        s = 0.5f / s;
        q[j + 1] = (R[i][j] + R[j][i]) * s;
        q[k + 1] = (R[k][i] + R[i][k]) * s;
        q[0] = (R[k][j] - R[j][k]) * s;
    }

    length = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] /= length;
    q[1] /= length;
    q[2] /= length;
    q[3] /= length;

    gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0.0f;

    uint8_t ret = 0;

    if (isfinite(q[0]) && isfinite(q[1]) && isfinite(q[2]) && isfinite(q[3])
        && length > 0.95f && length < 1.05f) {

        ret = 1;
    }

    return ret;
}

/*
 * brief: attitude estimator base on quaternion
 *
 * @param dt   : delta time in seconds
 * @param accel: body acceleration in m/s^2
 * @param gyro : body rates in rad/s
 */
static void update(const float dt, const float accel[], const float gyro[])
{
    if (!inited) {
        inited = init(accel);
        return;
    }

    float scale;

    scale = 1.0f / sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    float a[3] = {accel[0] * scale, accel[1] * scale, accel[2] * scale};

    float b[3] = {
        2.0f * (q[1] * q[3] - q[0] * q[2]),
        2.0f * (q[2] * q[3] + q[0] * q[1]),
        q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
    };

    float c[3] = {
        b[1] * a[2] - b[2] * a[1],
        b[2] * a[0] - b[0] * a[2],
        b[0] * a[1] - b[1] * a[0],
    };

    for (int i = 0; i < 3; i++) {
        c[i] *= accel_gain;

        gyro_bias[i] += c[i] * gyro_bias_gain * dt;
        gyro_bias[i] = gyro_bias[i] < -0.05f ? -0.05f : gyro_bias[i] > 0.05f ? 0.05f : gyro_bias[i];

        c[i] += gyro[i] + gyro_bias[i];
    }

    float q_prev[4] = {q[0], q[1], q[2], q[3]};

    q[0] += (c[0] * -q_prev[1] + c[1] * -q_prev[2] + c[2] * -q_prev[3]) * 0.5f * dt;
    q[1] += (c[0] *  q_prev[0] + c[1] * -q_prev[3] + c[2] *  q_prev[2]) * 0.5f * dt;
    q[2] += (c[0] *  q_prev[3] + c[1] *  q_prev[0] + c[2] * -q_prev[1]) * 0.5f * dt;
    q[3] += (c[0] * -q_prev[2] + c[1] *  q_prev[1] + c[2] *  q_prev[0]) * 0.5f * dt;

    scale = 1.0f / sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= scale;
    q[1] *= scale;
    q[2] *= scale;
    q[3] *= scale;

    if (!isfinite(q[0]) || !isfinite(q[1]) || !isfinite(q[2]) || !isfinite(q[3])) {
        q[0] = q_prev[0];
        q[1] = q_prev[1];
        q[2] = q_prev[2];
        q[3] = q_prev[3];
        gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0.0f;
    }
}

// interface

#include "estimator.h"
#include "scheduler.h"
#include "topics.h"

static uint64_t t_prev = 0;

void estimator_task()
{
    uint64_t t = time();

    float dt = t_prev > 0 ? (t - t_prev) * 1e-6f : 1e-6f;
    dt = dt < 0.0001f ? 0.0001f : dt > 0.02f ? 0.02f : dt;

    t_prev = t;

    // acceleration in m/s^s
    float a[3] = { _accel.x, _accel.y, _accel.z };

    // angular speed in rad/s
    float g[3] = { _gyro.x, _gyro.y, _gyro.z };

    // update attitude by quaternion
    update(dt, a, g);

    // generate rotation matrix from quaternion

    float sq[4] = { q[0] * q[0], q[1] * q[1], q[2] * q[2], q[3] * q[3] };

    // publish attitude
    _attitude.timestamp = t;

    _attitude.euler[0] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    _attitude.euler[1] = asinf(2.0f * (q[0] * q[2] - q[3] * q[1]));
    _attitude.euler[2] = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));

    _attitude.rates[0] = g[0] + gyro_bias[0];
    _attitude.rates[1] = g[1] + gyro_bias[1];
    _attitude.rates[2] = g[2] + gyro_bias[2];
}
