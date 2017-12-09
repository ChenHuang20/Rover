/***************************************************************************
 *
 *  Copyright (c) 2017 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file low_pass_filter.c
 *
 * @author zwh <zwh@raaworks.com>
 */

#include <math.h>

#include "low_pass_filter.h"

#ifndef M_PI_F
#define M_PI_F  3.14159265358979323846f
#endif

void lpf_set_cutoff_frequency(low_pass_filter_t *p, float sample_freq, float cutoff_freq)
{
    p->_cutoff_freq = cutoff_freq;

    if (p->_cutoff_freq <= 0.0f) {
        p->_a1 = 0.0f;
        p->_a2 = 0.0f;
        p->_b0 = 0.0f;

        p->_delay_element_1 = 0.0f;
        p->_delay_element_2 = 0.0f;

    } else {
        float fr = sample_freq / cutoff_freq;
        float ohm = tanf(M_PI_F / fr);
        float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;

        p->_b0 = ohm * ohm / c;
        p->_a1 = 2.0f * (ohm * ohm - 1.0f) / c;
        p->_a2 = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;

        p->_delay_element_1 = 0.0f;
        p->_delay_element_2 = 0.0f;
    }
}

float lpf_allpy(low_pass_filter_t *p, float sample)
{
    if (p->_cutoff_freq <= 0.0f) {
        // no filtering
        return sample;
    }

    // do filtering
    float delay_element_0 = sample - p->_delay_element_1 * p->_a1 - p->_delay_element_2 * p->_a2;

    if (!isfinite(delay_element_0)) {
        // don't allow bad values to propagate via the filter
        delay_element_0 = sample;
    }

    float output = delay_element_0 * p->_b0 + p->_delay_element_1 * 2.0f * p->_b0 + p->_delay_element_2 * p->_b0;

    p->_delay_element_2 = p->_delay_element_1;
    p->_delay_element_1 = delay_element_0;

    // return the value. should be no need to check limits
    return output;
}

float lpf_reset(low_pass_filter_t *p, float sample)
{
    float dval = sample / (p->_b0 + 2.0f * p->_b0 + p->_b0);

    p->_delay_element_1 = dval;
    p->_delay_element_2 = dval;

    return lpf_allpy(p, sample);
}
