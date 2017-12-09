/***************************************************************************
 *
 *                          厦门大学海韵机器人队
 *
 * @2017 all rights reserved
 *
 * @file sbus.c
 *
 * @author zwh <zwh@raaworks.com>
 *         hc <450801089.qq.com>
 *
 ***************************************************************************/

#include "sbus.h"
#include "scheduler.h"
#include "topics.h"

static char path[3] = { 'u', 1, 0 };

void sbus()
{
    uint8_t buf[18] = { 0 };

    if (read(path, buf, 18) == 0) {
        if (buf[0] == 0 && buf[1] == 0 && buf[2] == 0 && buf[3] == 0 && buf[4] == 0 && buf[5] == 0) {
            return;

        } else {
            _radio_raw.timestamp = time();

            _radio_raw.rc.ch0 = ((buf[0] | (int16_t)buf[1] << 8) & 0x07FF) - 1024;
            _radio_raw.rc.ch1 = ((buf[1] >> 3 | (int16_t)buf[2] << 5) & 0x07FF) - 1024;
            _radio_raw.rc.ch2 = ((buf[2] >> 6 | (int16_t)buf[3] << 2 | (int16_t)buf[4] << 10) & 0x07FF) - 1024;
            _radio_raw.rc.ch3 = ((buf[4] >> 1 | (int16_t)buf[5] << 7) & 0x07FF) - 1024;

            _radio_raw.rc.s1 = ((buf[5] >> 4) & 0x000C) >> 2;
            _radio_raw.rc.s2 = ((buf[5] >> 4) & 0x0003);

            _radio_raw.mouse.x = buf[6] | ((int16_t)buf[7] << 8);
            _radio_raw.mouse.y = buf[8] | ((int16_t)buf[9] << 8);
            _radio_raw.mouse.z = buf[10] | ((int16_t)buf[11] << 8);

            _radio_raw.mouse.press_l = buf[12];
            _radio_raw.mouse.press_r = buf[13];

            _radio_raw.key.v = buf[14] | buf[15] << 8;

            _radio.timestamp = time();
            _radio.x = _radio_raw.rc.ch1 / 660.0f;
            _radio.y = _radio_raw.rc.ch0 / 660.0f;
            _radio.pitch = -_radio_raw.rc.ch3 / 660.0f;
            _radio.yaw = _radio_raw.rc.ch2 / 660.0f;
            _radio.armed = _radio_raw.rc.s2;
            _radio.shoot = _radio_raw.rc.s1;
        }
    }
}
