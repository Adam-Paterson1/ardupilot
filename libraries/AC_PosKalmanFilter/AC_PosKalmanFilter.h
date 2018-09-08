#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <DataFlash/DataFlash.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_FakeSensor/AP_FakeSensor.h>

#define POS_FILTER_CUTOFF_FREQ  2.0f
#define R                       0.0025f     // measurement noise cov
#define Q                       0.0225f     // process noise cov
//#define dt_KF                   0.005f         // @400Hz
//#define dt_KF2                  dt_KF*dt_KF // dt^2
#define G_KF                    9.81f        // gravity for kalman filter

class AC_PosKalmanFilter
{
public:
    AC_PosKalmanFilter(const AP_AHRS_View& ahrs,
                       const AP_InertialNav& inav,
                       const AP_FakeSensor& fs);
    void run();

    position_t get_pos();
    void print_shit();
    void write_log();

private:
    // references to inertial nav and ahrs libraries
    const AP_AHRS_View &        _ahrs;
    const AP_InertialNav&       _inav;
    const AP_FakeSensor&        _fs;

    vector<float> _Xp;  // state prediction
    vector<float> _Xe;  // state estimate
    vector<float> _Pp;  // error covariance prediction
    vector<float> _Pe;   // error covariance estimate
    vector<float> _K;   // Kalman gain

    float dt_KF;
    float dt_KF2;
    uint64_t t0;

    position_t _pos;

    int _prev_data_set;

    // KF matrix operation
    void _init();
    vector<float> _Kk(vector<float> P);
    vector<float> _P_estimate(vector<float> P, vector<float> K);
    vector<float> _P_predict(vector<float> P);
    vector<float> _X_estimate(vector<float> K, vector<float> Xp, position_t pos);
    vector<float> _X_predict(vector<float> Xe, float ay, float az);
};
