#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <stdio.h>
#include <cstdio>
#include <vector>
#include <ctype.h>

#define ODROID_BAUDRATE 921600
#define N_MSG_VARIABLE  5
#define DATA_BUF_SIZE 24 // 6 int variables
#define FAR_THRESHOLD 2000 // mm

using namespace std;

union int_num
{
    unsigned char buf[4];
    int num;
};

union float_num
{
    unsigned char buf[4];
    float num;
};

enum FakeSensor_status
{
    Good,
    FarFromOrigin,
    OutOfTunnel,
    Bad,
    NotConnected
};

struct rc_channel_t
{
    int roll  = 1500;
    int pitch = 1500;
    int yaw   = 1500;
    int thr   = 0;
    int aux5  = 0;
    int aux6  = 0;
    int aux7  = 0;
    int aux8  = 0;
    int aux9  = 0;

};

// control output
struct pos_error_t
{
    float ey;
    float dterm_y;
    float iterm_y;
    float ez;
    float dterm_z;
    float iterm_z;
};


struct position_t
{
    float y;        // m
    float z;        // m
    float yd;       // target y
    float zd;       // target z
    float vy;       // m/s
    float vz;       // m/s
    int nset = 0;

    // controller stuff
    float vel_y_err = 0;
    float vel_z_err = 0;

    float prev_ey = 0;    // previous pos y error
    float prev_ez = 0;    // previous pos z error

    float iey = 0;   // error integral of pos y
    float iez;   // error integral of pos z
};

struct FakeSensor_data_t
{
    position_t pos;
    position_t KF_pos;

    // Pixhawk 2 info
    float roll;
    float pitch;
    float yaw;

    rc_channel_t ch;    // rc channels

    uint32_t ts;    // in ms

    enum FakeSensor_status status;

    // controller output
    float AC_alt_target;
    float AC_cr;
    float dist_err;
    float target_rangefinder_alt;
    pos_error_t perr;
    float mthrust_out[3];   // motor thrust output
    float u1;               // Backstepping thrust output
};



class AP_FakeSensor
{
public:
    FakeSensor_data_t data;

    AP_FakeSensor();

    void init();
    void update();
    void get_AHRS(AP_AHRS_View* ahrs);
    void get_motors(AP_MotorsMulticopter* motors);
    bool data_is_ok();
    void get_KF_pos(position_t p);  // get kalman filter output;
    void write_log();

    // get controller info
    void read_controller(pos_error_t perr, float u1);

private:
    AP_HAL::UARTDriver *_uart = nullptr;
    char _linebuf[DATA_BUF_SIZE];
    uint8_t _buflen = 0;
    uint8_t _brokenlen = 0;
    position_t _prev_pos;
    bool _flag_init = false;

    AP_MotorsMulticopter*   _motors;
    AP_AHRS_View*           _ahrs;

    // messenger
    void _get_pos();
    void _read_radio();
    void _read_AHRS();
    vector<unsigned char> _msg_encoder();
    void _msg_sender(vector<unsigned char>  msg);

    // helper
    float _limit_thr(float thr);    // restrict throttle from 0-1 (mainly noise issue?)
    vector<unsigned char> _int2byte(vector<unsigned char> in, int value);
    vector<unsigned char> _float2byte(vector<unsigned char> in, float value);
    int _byte2int(char* buffer, int position);
    float _byte2float(char* buffer, int position);

};
