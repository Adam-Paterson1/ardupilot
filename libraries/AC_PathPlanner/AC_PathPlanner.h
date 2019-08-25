#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_FakeSensor/AP_FakeSensor.h"
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash.h>

#define WAYPOINT_TIME_INTERVAL  10000*1000  // ms*1000
#define ad                      0.1         // m/s^2, desired accleration
#define DIST_THRES              0.1         // m
#define PITCH_OSCILLATION       2           // degree - for experiment only
#define PITCH_PERIOD            1           // second

class AC_PathPlanner
{
public:

    AC_PathPlanner();
    position_t run_setpoint();
    position_t run_diagonal_trajectory();
    position_t run_circular_trajectory();
    position_t get_target_pos();
    void get_current_pos(position_t pos);
    void get_default_target(float yd, float zd);

    // for experiment only
    float pitch_oscillator(float pilot_pitch);

private:
    struct flags_t
    {
        bool atGoal = false;
        bool start_flight = false;
        bool yGoal = false;
        bool zGoal = false;

    }_flags;

    position_t _pos;
    position_t _pos_d;   // target position

    float _ftimer = 0;
    uint64_t _timer = 0;    // ms * 10
    uint64_t _t0;

    // circular trajectory parameters
    float _cir_radius = 0.45;            // circle radius
    float _cir_height_offset = 0.8;     // height offset
    float _w = 0.2;                    // time constant/frequency : small = slow

    // *** for setpoint tracking ***
    // square
    //float _wp_y[6] = {0  , 0.6, 0.6, -0.6, -0.6, 0};
    //float _wp_z[6] = {0.6, 0.6, 1.7,  1.7,  0.6, 0.6};

    // X
    //float _wp_y[6] = {0, -0.55, 0.55, -0.55, 0.55, 0};
    //float _wp_z[6] = {0.6, 0.6, 1.7, 1.7, 0.6, 0.6};

    // up down
    //float _wp_y[3] = {0, 0, 0};
    //float _wp_z[3] = {0.6, 1.6, 0.6};

    // step up <> down
    //float _wp_y[7] = {0, 0, 0, 0, 0, 0, 0}; // middle
    //float _wp_y[7] = {0.55,0.55,0.55,0.55,0.55,0.55,0.55};
    //float _wp_z[7] = {0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.7};

    float _wp_y[4] = {0.55,0.55,0.55,0.55};
    float _wp_z[4] = {0.4, 0.8, 1.2, 1.6};

    uint16_t _wp_idx = 0;   // current waypoint index;
    uint16_t _nwp;          // number of waypoints

    // *** for trajectory tracking ***


    void _check_flight_init();
};
