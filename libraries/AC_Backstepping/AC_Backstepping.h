#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include "AP_FakeSensor/AP_FakeSensor.h"
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash.h>

#define G                                   -9.81f  // gravity
#define DEFAULT_IMAX                        10
#define BACKSTEPPING_THROTTLE_CUTOFF_FREQ   2.0f    // low-pass filter on accel error (unit: hz)
#define BACKSTEPPING_VEL_ERROR_CUTOFF_FREQ  5.0f
#define ACCEL_SCALE_FACTOR                  0.04f   // to scale desired acceleration to program output: 0.04 is best
#define POS_ERROR_THRESHOLD                 1.5f    // in m, max allowed change in position
#define THROTTLE_TRANSITION_TIME            1.0f    // second
#define MANUAL_OVERRIDE_TIME                1.5f    // second
#define THROTTLE_HOVER_FOR_US               0.2025f
// defines for PID controller
#define PID_DYTERM_MAX                      2000    // 2 degree
#define PID_IYTERM_MAX                      1000    // 10 degree
#define PID_DZTERM_MAX                      0.07    // 20% throttle
#define PID_IZTERM_MAX                      0.065    // 10% throttle
#define PID_DYAWTERM_MAX                    2000 
class AC_Backstepping
{
public:
    // variables
    struct backstepping_flags
    {
        bool switched_mode;
        bool snapshot_angle;
        bool snapshot_thrust;
        bool angle_transition_completed;
        bool thrust_transition_completed;
        bool roll_override;
    }flags;

    pos_error_t perr;
    float switch_time = (float) 400.0f*THROTTLE_TRANSITION_TIME;
    // functions
    AC_Backstepping(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                    const AP_Motors& motors, AC_AttitudeControl& attitude_control,
                    const AP_FakeSensor& fs);

    void pos_update(position_t pos);
    void get_imax(float imax_y, float imax_z);
    void get_gains(float yk1, float yk2, float yk3, float zk1, float zk2, float zk3);
    void set_target_pos(position_t pos);
    void reset_integral();
    void reset_mode_switch();
    void write_log();

    // PID lateral controller
    float update_PID_lateral_controller(float angle_max);
    float update_PID_vertical_controller();
    float update_PID_yaw_controller();
    float get_PID_alt_climb_rate();    // cm/s
    void set_PID_gains(gains_t gains);
    void reset_PID_integral();

private:
    // references to inertial nav and ahrs libraries
    const AP_AHRS_View &        _ahrs;
    const AP_InertialNav&       _inav;
    const AP_Motors&            _motors;
    AC_AttitudeControl&         _attitude_control;
    const AP_FakeSensor&        _fs;

    struct controller_gains_t
    {
        float k1_y;
        float k2_y;
        float k3_y;
        float k1_z;
        float k2_z;
        float k3_z;
    }_gains;

    position_t _pos;
    float _dt;
    float _prev_ey;
    int num_prev_eys = 100;
    float _prev_eys[100] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int   index_y = 0;
    float _prev_ez;
    int num_prev_ezs = 100;
    float _prev_ezs[100] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int   index_z = 0;
    float _prev_e_yaw;
    int num_prev_e_yaws = 100;
    float _prev_e_yaws[100] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int   index_yaw = 0;
    float _pos_target_z;
    float _pos_target_y;
    int   _prev_nset;

    float _last_mode_thr_out;
    float _last_mode_roll_out;
    float _angle_transition_counter;
    float _thrust_transition_counter;
    float _thr_out;    // thrust output for motor, range from 0-1
    float _target_roll;     // desired roll to the attitude controller
    float _target_yaw;
    float _pilot_roll;      // pilot roll input
    float _roll_max;        // max roll angle
    float _yaw_max = 4000.0f;

    float _angle_transition(float target_roll);
    float _throttle_transition(float BS_thr_out);
    float _limit_value(float value, float threshold);
    float _limit_thrust(float thr);
    float _limit_sin_phi(float sp);
    float _rad2cdeg(float in);

    // PID lateral controller
    struct pid_gains_t
    {
        float py;
        float iy;
        float dy;
        float pz;
        float iz;
        float dz;
        float pyaw;
        float dyaw;
    }_pid;

    float _pid_iey;
    float _pid_iez;
};
