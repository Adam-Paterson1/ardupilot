#include "AC_PosKalmanFilter.h"

extern const AP_HAL::HAL &hal;

AC_PosKalmanFilter::AC_PosKalmanFilter(const AP_AHRS_View& ahrs,
                                       const AP_InertialNav& inav,
                                       const AP_FakeSensor& fs) :
                                       _ahrs(ahrs),
                                       _inav(inav),
                                       _fs(fs)
{
    hal.uartA->begin(115200); // debug

    _init();

    _a_bias.x = 0;
    _a_bias.y = 0;
    _a_bias.z = 0;

    _dt_KF = 0.0025f;
    _dt_KF2 = _dt_KF*_dt_KF;

    _t0 = AP_HAL::micros64();
}

void AC_PosKalmanFilter::_init()
{
    _Xp.reserve(4);
    _Xe.reserve(4);
    _Pp.reserve(8);
    _Pe.reserve(8);
    _K.reserve(4);

    for (int i=0;i<4;i++)
    {
        _Xp.push_back(0);
        _Xe.push_back(0);
    }

    _Pp.push_back(1);
    _Pp.push_back(0);
    _Pp.push_back(0);
    _Pp.push_back(1);
    _Pp.push_back(1);
    _Pp.push_back(0);
    _Pp.push_back(0);
    _Pp.push_back(1);

    _Pe =_Pp;
}

void AC_PosKalmanFilter::run()
{
    _dt_KF = (float) (AP_HAL::micros64()-_t0)*0.001f*0.001f; // second
    _dt_KF2 = _dt_KF*_dt_KF;

    // get accelerations
    const Vector3f &accel = _ahrs.get_accel_ef_blended();

    // check if need to reset velocity drift
    _check_onGround_reset();

    float ay = accel.y;
    float az = -(accel.z + G_KF);

    // prediction - 400Hz
     _Xp = _X_predict(_Xe, ay, az);

     _Pp = _P_predict(_Pe);

     _Xe = _Xp;

    //  correction - 30Hz / as soon as new lidar data arrives
    if (_prev_data_set != _fs.data.pos.nset)
    {
        _K = _Kk(_Pp);

        _Xe = _X_estimate(_K, _Xp, _fs.data.pos);
        _Pe  = _P_estimate(_Pp, _K);
    }

    _pos.y  = _Xe[0];
    _pos.vy = _Xe[1];
    _pos.z  = _Xe[2];
    _pos.vz = _Xe[3];

    _prev_data_set = _fs.data.pos.nset;
    _pos.yaw = _fs.data.pos.yaw;

    _t0 = AP_HAL::micros64();

    //int abc = 0;

    //if (_flags.reset_vel) abc = 1;

    //hal.uartA->printf("ay %4.5f, by %4.5f, vy %4.5f abc:%d\n", ay, _a_bias.y, _pos.vy, abc);

    //print_shit();
}

vector<float> AC_PosKalmanFilter::_X_predict(vector<float> Xe, float ay, float az)
{
    vector<float> xp;
    xp.reserve(4);

    // y
    xp.push_back( Xe[0] + Xe[1]*_dt_KF + 0.5f*ay*_dt_KF2 );
    xp.push_back( Xe[1] + ay*_dt_KF                     );

    // z
    xp.push_back( Xe[2] + Xe[3]*_dt_KF + 0.5f*az*_dt_KF2 );
    xp.push_back( Xe[3] + az*_dt_KF                     );

    return xp;
}

vector<float> AC_PosKalmanFilter::_X_estimate(vector<float> K, vector<float> Xp, position_t pos)
{
    vector<float> xe;
    xe.reserve(4);

    // y
    xe.push_back( Xp[0] + K[0]*(pos.y - Xp[0]) );
    xe.push_back( Xp[1] + K[1]*(pos.y - Xp[0]) );

    // z
    xe.push_back( Xp[2] + K[2]*(pos.z - Xp[2]) );
    xe.push_back( Xp[3] + K[3]*(pos.z - Xp[2]) );

    return xe;
}

vector<float> AC_PosKalmanFilter::_P_predict(vector<float> P)
{
    vector<float> pp;
    pp.reserve(8);

    // y
    pp.push_back( P[0] + Q + _dt_KF*P[2] + _dt_KF*(P[1] + _dt_KF*P[3]) );
    pp.push_back( P[1] + _dt_KF * P[3]                               );
    pp.push_back( P[2] + _dt_KF * P[3]                               );
    pp.push_back( P[3] + Q                                          );

    // z
    pp.push_back( P[4] + Q + _dt_KF*P[6] + _dt_KF*(P[5] + _dt_KF*P[7]) );
    pp.push_back( P[5] + _dt_KF * P[7]                               );
    pp.push_back( P[6] + _dt_KF * P[7]                               );
    pp.push_back( P[7] + Q                                          );

    return pp;
}

vector<float> AC_PosKalmanFilter::_P_estimate(vector<float> P, vector<float> K)
{
    vector<float> pe;
    pe.reserve(8);

    // y
    pe.push_back( -(K[0] - 1) * (P[0] + Q + _dt_KF * P[2] + _dt_KF * (P[1] + _dt_KF * P[3]))                   );
    pe.push_back( -(P[1] + _dt_KF * P[3]) * (K[0] - 1)                                                       );
    pe.push_back( P[2] - K[1] * (P[0] + Q + _dt_KF * P[2] + _dt_KF * (P[1] + _dt_KF * P[3])) + _dt_KF * P[3]    );
    pe.push_back( P[3] + Q - K[1] * (P[1] + _dt_KF * P[3])                                                   );

    // z
    pe.push_back( -(K[2] - 1) * (P[4] + Q + _dt_KF * P[6] + _dt_KF * (P[5] + _dt_KF * P[7]))                   );
    pe.push_back( -(P[5] + _dt_KF * P[7]) * (K[2] - 1)                                                       );
    pe.push_back( P[6] - K[3] * (P[4] + Q + _dt_KF * P[6] + _dt_KF * (P[5] + _dt_KF * P[7])) + _dt_KF * P[7]    );
    pe.push_back( P[7] + Q - K[3] * (P[5] + _dt_KF * P[7])                                                   );

    return pe;
}

vector<float> AC_PosKalmanFilter::_Kk(vector<float> P)
{
    vector<float> k;
    k.reserve(4);

    // y
    k.push_back( P[0] / (R + P[0]) );
    k.push_back( P[2] / (R + P[0]) );

    // z
    k.push_back( P[4] / (R + P[4]) );
    k.push_back( P[6] / (R + P[4]) );

    return k;
}

void AC_PosKalmanFilter::_check_onGround_reset()
{
    _is_onGround(); // check if system is idle on the ground

    if (_flags.reset_vel)
    {
        _Xe[1] = 0;
        _Xe[3] = 0;
    }
}

position_t AC_PosKalmanFilter::get_pos()
{
    return _pos;
}

void AC_PosKalmanFilter::_is_onGround()
{
    int thr = rc().channel(CH_3)->get_radio_in();

    if (thr > 1100)
    {
        _flags.idle = false;
        _flags.reset_vel = false;
        _ground_timer = 0;
    }
    else
    {
        _flags.idle = true;

        if (_ground_timer > 10) // if rest for 10s, start sampling IMU bias
        {
            _flags.reset_vel = true;
        }

        _ground_timer += _dt_KF;

        if (_ground_timer > 20)
        {
            _ground_timer = 0;
            _flags.reset_vel = false;
        }
    }

}

void AC_PosKalmanFilter::print_shit()
{
    hal.uartA->printf("pos_z %f, KF_z %f\n", _fs.data.pos.z, _pos.z);
}

void AC_PosKalmanFilter::write_log()
{
    // write log to dataflash
    const Vector3f &accel = _ahrs.get_accel_ef_blended();

    DataFlash_Class::instance()->Log_Write("PKF", "TimeUS,Y,Z,KFY,KFZ,VELY,VELZ,ACCY,ACCZ,NSET",
                                           "smmmmnnoo-", "F000000000", "Qffffffffi",
                                           AP_HAL::micros64(),
                                           (double) _fs.data.pos.y,
                                           (double) _fs.data.pos.z,
                                           (double) _pos.y,
                                           (double) _pos.z,
                                           (double) _pos.vy,
                                           (double) _pos.vz,
                                           (double) accel.y,
                                           (double) accel.z,
                                           (int) _fs.data.pos.nset);
}
