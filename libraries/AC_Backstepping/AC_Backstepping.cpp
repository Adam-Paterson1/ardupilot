#include "AC_Backstepping.h"

extern const AP_HAL::HAL &hal;

AC_Backstepping::AC_Backstepping(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                                 const AP_Motors& motors, AC_AttitudeControl& attitude_control,
                                 const AP_FakeSensor& fs) :
                                 _ahrs(ahrs),
                                 _inav(inav),
                                 _motors(motors),
                                 _attitude_control(attitude_control),
                                 _fs(fs)
{
    // init variables
    _pos_target_y = 0.0f;
    _pos_target_z = 0.0f;

    _dt = 0.0025f;

    _angle_transition_counter = 0;
    _thrust_transition_counter = 0;
    // init flags
    flags.snapshot_angle = false;
    flags.snapshot_thrust = false;
    flags.angle_transition_completed = false;
    flags.thrust_transition_completed = false;
}

float AC_Backstepping::_angle_transition(float BS_roll_out)
{
    float angle_out;
    // static uint8_t counter = 0;
    // counter++;
    // if (counter > 20) {
    //     counter = 0;
    //     hal.uartE->printf("$ Angle Trans %f %f\n", _last_mode_roll_out, _angle_transition_counter);
    // }
    if (flags.snapshot_angle) _last_mode_roll_out = _motors.get_roll();

    flags.snapshot_angle = false;

    // 400 counts = 1s
    if (_angle_transition_counter < switch_time)
    {
        angle_out = (switch_time - _angle_transition_counter)*_last_mode_roll_out/switch_time + _angle_transition_counter*BS_roll_out/switch_time;
        _angle_transition_counter++;
    }
    else
    {
        flags.angle_transition_completed = true;
        angle_out = BS_roll_out;
    }
    return angle_out;
}

void AC_Backstepping::write_log()
{
    // write log to dataflash
    DataFlash_Class::instance()->Log_Write("BS", "TimeUS,KFY,KFZ,VELY,VELZ,ROLL_D, YD, ZD",
                                           "smmmmnn-mmnn-", "F000000000000", "Qffffffffffff",
                                           AP_HAL::micros64(),
                                           (double) _pos.y,
                                           (double) _pos.z,
                                           (double) _pos.vy,
                                           (double) _pos.vz,
                                           (double) _target_roll,
                                           (double) _pos_target_y,
                                           (double) _pos_target_z);
}

void AC_Backstepping::reset_mode_switch()
{
    flags.angle_transition_completed = false;
    flags.thrust_transition_completed = false;
    flags.snapshot_angle = true;
    flags.snapshot_thrust = true;
    flags.switched_mode = false;
}

float AC_Backstepping::_throttle_transition(float BS_thr_out)
{
    float thr_out;

    if (flags.snapshot_thrust)  _last_mode_thr_out = _motors.get_throttle();

    flags.snapshot_thrust = false;

    // 400 counts = 1s
    if (_thrust_transition_counter < switch_time)
    {
        thr_out = (switch_time - _thrust_transition_counter)*_last_mode_thr_out/switch_time + _thrust_transition_counter*BS_thr_out/switch_time;
        _thrust_transition_counter++;
    }
    else
    {
        flags.thrust_transition_completed = true;
        thr_out = BS_thr_out;
    }
    thr_out = _limit_thrust(thr_out);
    return thr_out;
}

void AC_Backstepping::pos_update(position_t pos)
{
    _pos.y  = pos.y;
    _pos.z  = pos.z;
    // static uint8_t counter = 0;
    // counter++;
    // if (counter > 20) {
    //     counter = 0;
    //     hal.uartE->printf("$ Pos up %f %f\n", _pos.y, _pos.z);
    // }
}


void AC_Backstepping::set_target_pos(position_t pos)
{
    _pos_target_y = pos.y;
    _pos_target_z = pos.z;
    // static uint8_t counter = 0;
    // counter++;
    // if (counter > 20) {
    //     counter = 0;
    //     hal.uartE->printf("$ Target up %f %f\n", _pos_target_y, _pos_target_z);
    // }
    
}

void AC_Backstepping::reset_integral()
{
    _pos.iez = 0;
    _pos.iey = 0;
}

float AC_Backstepping::_limit_thrust(float thr)
{
    if (thr > 0.45f)        return 0.45f;
    else if (thr < 0.05f)   return 0.05f;
    else                return thr;
}

float AC_Backstepping::_limit_value(float value, float threshold)
{
    if (value > threshold)         return threshold;
    else if (value < -threshold)   return -threshold;
    else                           return value;
}

float AC_Backstepping::_rad2cdeg(float in)
{
    return in*180/M_PI*100.0f;
}

//  PID controller
float AC_Backstepping::update_PID_lateral_controller(float roll_max)
{
    // position error in mm as centidegrees are very small
    float ey = (_pos_target_y - _pos.y);

    // i term, restrict integral windup
    if (_pid.iy > 0) {
        _pid_iey += ey * _dt;
        _pid_iey = _limit_value(_pid_iey, PID_IYTERM_MAX / _pid.iy);
    }

    perr.pterm_y = _pid.py * ey;
    perr.iterm_y = _pid.iy * _pid_iey;
    // d term is error getting better or worse
    perr.dterm_y = _limit_value(_pid.dy * (ey - _prev_ey), PID_DYTERM_MAX);

    _prev_ey = _prev_ey + (ey - _prev_eys[index_y]) / 90;
    _prev_eys[index_y] = ey;
    index_y = (index_y + 1) % 90;

    _target_roll = _limit_value(perr.pterm_y + perr.iterm_y + perr.dterm_y, roll_max);

    // check for manual override
    if (!flags.angle_transition_completed)   _target_roll = _angle_transition(_target_roll);
    return _target_roll;
}
    /*
    Above
     -10 -> -5 => 5
     -5 -> -10 => -5
    Below
     10 -> 5 => -5
     5 -> 10 => 5
    */
float AC_Backstepping::update_PID_vertical_controller()
{
    // position error in m
    float ez = (_pos_target_z - _pos.z) / 1000.0f;

    // i term, restrict integral
    if (_pid.iz > 0) {
        _pid_iez += ez * _dt;
        _pid_iez = _limit_value(_pid_iez, PID_IZTERM_MAX / _pid.iz);
    }
    
    perr.pterm_z = _pid.pz * ez;
    perr.iterm_z = _pid.iz*_pid_iez;
    // d term is error getting better or worse
    perr.dterm_z = _limit_value(_pid.dz * (ez - _prev_ez), PID_DZTERM_MAX);
    // Low pass prev_ez
    _prev_ez = _prev_ez + (ez - _prev_ezs[index_z]) / 90;
    _prev_ezs[index_z] = ez;
    index_z = (index_z + 1) % 90;
    // static uint8_t counter = 0;
    // counter++;
    // if (counter >5) {
    //     counter = 0;
    //     hal.uartE->printf("$ Pos up %f %f %f %f\n", _pid.dz, ez, _prev_ez, perr.dterm_z);
    // }
    _thr_out = _limit_thrust(THROTTLE_HOVER_FOR_US + perr.pterm_z + perr.iterm_z + perr.dterm_z);

    // mode transition throttle ramping, 0.5s
    if (!flags.thrust_transition_completed)   _thr_out = _throttle_transition(_thr_out);
    return _thr_out;
}

void AC_Backstepping::reset_PID_integral()
{
    _pid_iey = 0;
    _pid_iez = 0;
}

void AC_Backstepping::set_PID_gains(gains_t gains)
{
    _pid.py = gains.py;
    _pid.iy = gains.iy;
    _pid.dy = gains.dy;
    _pid.pz = gains.pz;
    _pid.iz = gains.iz;
    _pid.dz = gains.dz;
    // static uint8_t counter = 0;
    // counter++;
    // if (counter > 20) {
    //     counter = 0;
    //     hal.uartE->printf("$ Gains %f %f %f %f %f %f\n", _pid.py, _pid.iy, _pid.dy, _pid.pz, _pid.iz, _pid.dz);
    // }
}
