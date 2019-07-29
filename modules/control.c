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

int START=1;

const float rc_ctrl_rate[3] = { 2.0f, 4.0f, 2.5f };
uint8_t start_flag = 0;

void wheel_control()
{
    uint64_t t = time();

    static uint64_t t_prev = 0;

    float dt = t_prev > 0 ? (t - t_prev) * 1e-6f : 4e-3f;
    dt = dt < 0.002f ? 0.002f : dt > 0.02f ? 0.02f : dt;

    t_prev = t;

    _pid._velocity_sp[0] = + _radio.y + _radio.x + _radio.yaw;
    _pid._velocity_sp[1] = + _radio.y - _radio.x - _radio.yaw;
    _pid._velocity_sp[2] = + _radio.y - _radio.x + _radio.yaw;
    _pid._velocity_sp[3] = + _radio.y + _radio.x - _radio.yaw;

    for(int i = 0; i < 4; i++) {
        _pid.err_wheel_now[i] = _pid._velocity_sp[i] - _wheel_velocity.wheel[i];

        _pid.delta_wheel[i] =  _params.velocity_p[i] * (_pid.err_wheel_now[i] - _pid.err_wheel_prev[i])
                                 +_params.velocity_i[i] * (_pid.err_wheel_now[i])
                                 +_params.velocity_d[i] * (_pid.err_wheel_now[i] - 2*_pid.err_wheel_prev[i] + _pid.err_wheel_pprev[i]);

        _pid.err_wheel_pprev[i] = _pid.err_wheel_prev[i];
        _pid.err_wheel_prev[i] = _pid.err_wheel_now[i];

        _pid.output_wheel[i] +=_pid.delta_wheel[i]; 
    }

    _actuator.timestamp = t;

    for (int i = 0; i < 4; i++) {

    _actuator.wheel[i] = _pid.output_wheel[i];

    }
}

void gimbal_follow() {
        _pid._gimbalfollow_rate_sp = (0.90f - _gimbal_angle.gimbal[0]) * _params.att_p[2];
        //��ǰ���ٶ����=�������ٶ�-��ǰ���ٶ�(�ڻ�)
        _pid.err_gimbalfollow_now = _pid._gimbalfollow_rate_sp - _attitude.rates[2];
        //�ڻ����=P��+I��+D��
        _pid.output_gimbalfollow = _pid.err_gimbalfollow_now * _params.rate_p[2]
                              + _pid._rates_int[2]
                              + (_pid._rates_prev[2] - _attitude.rates[2]) * _params.rate_d[2];
        _pid.output_gimbalfollow = fmaxf(-1.0f, fminf(1.0f, _pid.output_gimbalfollow));

        _pid._rates_prev[2] = _attitude.rates[2];

        _pid._rates_int[2] += _pid.err_gimbal_now[2] * _params.rate_i[2];
        _pid._rates_int[2] = fmaxf(-_params.rate_i_max[2], fminf(_params.rate_i_max[2], _pid._rates_int[2]));

//        _actuator.gimbal[2] = _pid.output_gimbalfollow;
}

void gimbal_control()
{
    uint64_t t = time();

    static uint64_t t_prev = 0;

    float dt = t_prev > 0 ? (t - t_prev) * 1e-6f : 4e-3f;
    dt = dt < 0.002f ? 0.002f : dt > 0.02f ? 0.02f : dt;

    t_prev = t;
    //�⻷����ǶȺͽ��ٶ�
    float euler[3] = { _attitude.euler[0], _attitude.euler[1], _attitude.euler[2] };
    float rates[3] = { _attitude.rates[0], _attitude.rates[1], _attitude.rates[2] };
    //��ң�����ƶ�
    if (fabsf(_radio.pitch) > 0.05f) {
    //pitch�����Ƕ�
        _pid._euler_sp[1] = euler[1];
    //pitch�������ٶ�
        _pid._rates_sp[1] = _radio.pitch * rc_ctrl_rate[1];
    //��ң�������ƶ�
    } else {
        _pid._rates_sp[1] = (_pid._euler_sp[1] - euler[1]) * _params.att_p[1];
    }
    //��ң�����ƶ�
    if (fabsf(_radio.yaw) > 0.05f) {
    //yaw�����Ƕ�
        _pid._euler_sp[2] = euler[2];
        start_flag = 1;
    //yaw�������ٶ�
        _pid._rates_sp[2] = _radio.yaw * rc_ctrl_rate[2];
    //��ң�������ƶ�
	} else {
        _pid._rates_sp[2] = (_pid._euler_sp[2] - euler[2]) * _params.att_p[2];
    }

    for (int i = 0; i < 3; i++) {
        //��ǰ���ٶ����=�������ٶ�-��ǰ���ٶ�(�ڻ�)
        _pid.err_gimbal_now[i] = _pid._rates_sp[i] - rates[i];
        //�ڻ����=P��+I��+D��
        _pid.output_gimbal[i] = _pid.err_gimbal_now[i] * _params.rate_p[i]
                              + _pid._rates_int[i]
                              + (_pid._rates_prev[i] - rates[i]) * _params.rate_d[i];
        _pid.output_gimbal[i] = fmaxf(-1.0f, fminf(1.0f, _pid.output_gimbal[i]));

        _pid._rates_prev[i] = rates[i];

        _pid._rates_int[i] += _pid.err_gimbal_now[i] * _params.rate_i[i];
        _pid._rates_int[i] = fmaxf(-_params.rate_i_max[i], fminf(_params.rate_i_max[i], _pid._rates_int[i]));
    }

    _actuator.timestamp = t;

    for (int i = 0; i < 3; i++) {
        _actuator.gimbal[i] = _pid.output_gimbal[i];
    }
}

void chassis_follow()
{
    _params.rotate_i_max = 1.0f;
    //��������   У׼��ǰ��̨λ��
    if(_radio.mode == 1) {
        _params.rorate_std = _gimbal_angle.gimbal[0];
        START = 1;
    }
    
    if(_radio.mode == 3) {
        _pid._rotate_sp = _params.rorate_std;
        START = 1;
    }
    
    if(_radio.mode == 2) {
        if(START == 1)  {
            _pid._rotate_sp = _params.rorate_std + 0.1f;
        }
        if(_gimbal_angle.gimbal[0] >= (_params.rorate_std+0.1f)) {
            _pid._rotate_sp = _params.rorate_std - 0.1f;
        }
        if(_gimbal_angle.gimbal[0] <= (_params.rorate_std-0.1f)) {
            _pid._rotate_sp = _params.rorate_std - 0.1f;
        }
    }
    uint64_t t = time();

    static uint64_t t_prev = 0;

    float dt = t_prev > 0 ? (t - t_prev) * 1e-6f : 4e-3f;
    dt = dt < 0.002f ? 0.002f : dt > 0.02f ? 0.02f : dt;

    t_prev = t;
    //���νǶ�ֵ֮��
	_pid.err_rotate_now = 0.386f - _gimbal_angle.gimbal[0];
    //����
    if(_pid.err_rotate_now >= -0.002f && _pid.err_rotate_now <= 0.002f)
        _pid.err_rotate_now = 0.0f;

    //�������ǰ��λ�ÿ�Խ0~8191��Ӱ��
    if	(_pid.err_rotate_now >= 0.33f )	{
        _pid.err_rotate_now -= 1;
    }
    if	(_pid.err_rotate_now <= -0.33f)	{
        _pid.err_rotate_now += 1;
    }

    _pid.output_rotate = _pid.err_rotate_now * _params.rotate_p
                                             + _pid._rotate_int
                                             + (_pid.err_rotate_prev - _gimbal_angle.gimbal[0]) * _params.rotate_d;

    _pid.err_rotate_prev = _gimbal_angle.gimbal[0];

    _pid._rotate_int += _pid.err_rotate_now * _params.rotate_i;
    _pid._rotate_int = fmaxf(- _params.rotate_i_max, fminf(_params.rotate_i_max, _pid._rotate_int));

    _pid._velocity_sp[0] = + _radio.y + _radio.x + _pid.output_rotate * 1;
    _pid._velocity_sp[1] = + _radio.y - _radio.x - _pid.output_rotate * 1;
    _pid._velocity_sp[2] = + _radio.y - _radio.x + _pid.output_rotate * 1;
    _pid._velocity_sp[3] = + _radio.y + _radio.x - _pid.output_rotate * 1;

    for(int i = 0; i < 4; i++) {
    _pid.err_wheel_now[i] = _pid._velocity_sp[i] - _wheel_velocity.wheel[i];

    _pid.delta_wheel[i] =  _params.velocity_p[i] * (_pid.err_wheel_now[i] - _pid.err_wheel_prev[i])
                          +_params.velocity_i[i] * (_pid.err_wheel_now[i])
                          +_params.velocity_d[i] * (_pid.err_wheel_now[i] - 2*_pid.err_wheel_prev[i] + _pid.err_wheel_pprev[i]);

    _pid.err_wheel_pprev[i] = _pid.err_wheel_prev[i];
    _pid.err_wheel_prev[i] = _pid.err_wheel_now[i];

    _pid.output_wheel[i] +=_pid.delta_wheel[i]; 
    }

    _actuator.timestamp = t;

    for (int i = 0; i < 4; i++) {
        _actuator.wheel[i] = _pid.output_wheel[i];
    }

}

void power_limit()
{
    float ABSCurrent[4] = { //_wheel_current.wheel[0]
        (_wheel_current.wheel[0] > 0 ? _wheel_current.wheel[0] : -_wheel_current.wheel[0]),
        (_wheel_current.wheel[1] > 0 ? _wheel_current.wheel[1] : -_wheel_current.wheel[1]),
        (_wheel_current.wheel[2] > 0 ? _wheel_current.wheel[2] : -_wheel_current.wheel[2]),
        (_wheel_current.wheel[3] > 0 ? _wheel_current.wheel[3] : -_wheel_current.wheel[3]),
    };
     
    //���ʷ���
    _wheel_current.power_sum = ABSCurrent[0] + ABSCurrent[1] + ABSCurrent[2] + ABSCurrent[3];
    
    //ȫ����ģʽ
    if(_wheel_current.power_sum > _wheel_current.max_sum)
    {
        _actuator.wheel[0] = _wheel_current.max_sum * ABSCurrent[0] / _wheel_current.power_sum;
        _actuator.wheel[1] = _wheel_current.max_sum * ABSCurrent[1] / _wheel_current.power_sum;
        _actuator.wheel[2] = _wheel_current.max_sum * ABSCurrent[2] / _wheel_current.power_sum;
        _actuator.wheel[3] = _wheel_current.max_sum * ABSCurrent[3] / _wheel_current.power_sum;
    }
    else
    {
        _actuator.wheel[0] = _wheel_current.max_sum / 4;
        _actuator.wheel[1] = _wheel_current.max_sum / 4;
        _actuator.wheel[2] = _wheel_current.max_sum / 4;
        _actuator.wheel[3] = _wheel_current.max_sum / 4;
    }
    
    //�ɵ���ģʽ
        if(_wheel_current.power_sum > _wheel_current.max_sum)
    {
        _actuator.wheel[0] *= _wheel_current.max_sum / _wheel_current.power_sum;
        _actuator.wheel[1] *= _wheel_current.max_sum / _wheel_current.power_sum;
        _actuator.wheel[2] *= _wheel_current.max_sum / _wheel_current.power_sum;
        _actuator.wheel[3] *= _wheel_current.max_sum / _wheel_current.power_sum;
    }

}

#define FRIC_RUN  1

void trigger_control()
{
    uint64_t t = time();

    //��ǰ�Ƕ�
    float tri_angle = _motor_trigger.total_angle;

    //ת��ʱ
    if(_radio.shoot == 1 && _radio.mode == 1) {

    //�����Ƕ� = ��ǰ�Ƕ� + 40
    _pid._tri_angle_sp = 40 + tri_angle;

    //�������ٶ� = �����ǶȺ͵�ǰ�Ƕ���PID = λ�û���� = �ڻ�����
    _pid._tri_rates_sp = (_pid._tri_angle_sp - tri_angle) * _params.tri_angle_p
                       + _pid._tri_rates_int
                       + (_pid._tri_angle_prev - tri_angle) * _params.tri_angle_d;
    }

    //��ת��ʱ
    else {
    _pid._tri_rates_sp = (_pid._tri_angle_sp - tri_angle) * _params.tri_angle_p;
    }

    //��ǰ���ٶ����=�������ٶ�-��ǰ���ٶ�(�ڻ�)
    _pid.err_tri_rates_now = _pid._tri_rates_sp - _tri_rates.trigger;
    //�ڻ����=P��+I��+D��
    _pid.output_trigger = _pid.err_tri_rates_now * _params.tri_rate_p
                        + _pid._tri_rates_int
                        + (_pid._tri_rates_prev - _tri_rates.trigger) * _params.tri_rate_d;
//    _pid.output_trigger = fmaxf(-1.0f, fminf(1.0f, _pid.output_trigger));

    _pid._tri_rates_prev = _tri_rates.trigger;

//    _pid._tri_rates_int += _pid.err_tri_rates_now * _params.tri_rate_i;
//    _pid._tri_rates_int = fmaxf(-_params.tri_rate_i_max, fminf(_params.tri_rate_i_max, _pid._tri_rates_int));

    _actuator.trigger = _pid.output_trigger;

}

void control_task()
{

//    //��У׼ģʽ��
//    if(_radio.mode != 3) {
//        // ���̿���
//        if(_radio.shoot == 2) {
        wheel_control();
//        }
//        // ��̨����
        gimbal_control();

//        // ���̸���
//        
//        chassis_follow();
//}
//    //��������
//    power_limit();
//    trigger_control();
}
