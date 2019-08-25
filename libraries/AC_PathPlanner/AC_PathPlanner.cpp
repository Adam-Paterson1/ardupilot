#include "AC_PathPlanner.h"

extern const AP_HAL::HAL &hal;

AC_PathPlanner::AC_PathPlanner()
{
    _nwp = sizeof(_wp_y)/sizeof(*_wp_y);
    _wp_idx = 0;
    _timer = 0;

    _pos_d.ay = 0;
    _pos_d.az = 0;
    _pos_d.vy = 0;
    _pos_d.vz = 0;
    _pos_d.y = 0;
    _pos_d.z = 1;

    _flags.atGoal = false;
    _flags.start_flight = false;
}

position_t AC_PathPlanner::run_setpoint()
{
    _check_flight_init();

    _flags.start_flight = false; // only run a single setpoint

    if (_flags.start_flight && !_flags.atGoal)
    {
        // check if reached goal
        if (_wp_idx >= _nwp)
        {
            _flags.atGoal = true;
        }
        else
        {
            _pos_d.y = _wp_y[_wp_idx];
            _pos_d.z = _wp_z[_wp_idx];
        }

        uint32_t dt = AP_HAL::micros64() - _t0;
        _timer += dt;


        if (_timer >= WAYPOINT_TIME_INTERVAL)
        {
            _wp_idx++;
            _timer = 0;
        }
    }
    else
    {
        _timer = 0;
        _wp_idx = 0;

        if (!_flags.atGoal)
        {
            _pos_d.y = _wp_y[0];
            _pos_d.z = _wp_z[0];
        }
    }

    _t0 = AP_HAL::micros64();
    //hal.uartA->printf("yd %f, zd %f, timer %d\n", _pos_d.y, _pos_d.z , _timer);
    return _pos_d;
}

position_t AC_PathPlanner::run_circular_trajectory()
{
    _check_flight_init();

    if (_flags.start_flight && !_flags.atGoal)
    {
        float dt = (float) (AP_HAL::micros64() - _t0)*0.001f*0.001f;

        if (_wp_idx == 0)
        {
            // position
            _pos_d.y = 0;
            _pos_d.z = _cir_height_offset + _cir_radius;

            // velocity
            _pos_d.vy = 0;//_cir_radius * _w * M_PI;
            _pos_d.vz = 0;

            // check if reached the starting point
            if (fabs(_pos_d.y - _pos.y) < DIST_THRES && fabs(_pos_d.z - _pos.z) < DIST_THRES)
            {
                _wp_idx++;
            }

        }
        else if (_wp_idx == 1)
        {
            // position
            _pos_d.y = _cir_radius * sin(_w*M_PI*_ftimer);
            _pos_d.z = _cir_height_offset + _cir_radius * cos(_w*M_PI*_ftimer);

            // velocity
            _pos_d.vy = _cir_radius * _w * M_PI * cos(_w*M_PI*_ftimer);
            _pos_d.vz = - _cir_radius * _w * M_PI * sin(_w*M_PI*_ftimer);

            // acceleration
            _pos_d.ay = -_cir_radius * _w * _w * M_PI * M_PI * sin(_w*M_PI*_ftimer);
            _pos_d.az = -_cir_radius * _w * _w * M_PI * M_PI * cos(_w*M_PI*_ftimer);

            _ftimer += dt;
        }


    }

    _t0 = AP_HAL::micros64();
    //hal.uartA->printf("yd %f, zd %f, vyd %f, vzd %f, y %f, z %f\n", _pos_d.y, _pos_d.z,_pos_d.vy, _pos_d.vz, _pos.y, _pos.z);
    return _pos_d;
}


position_t AC_PathPlanner::run_diagonal_trajectory()
{
    _check_flight_init();

    if (_flags.start_flight && !_flags.atGoal)
    {
        float dt = (float) (AP_HAL::micros64() - _t0)*0.001f*0.001f;

        switch (_wp_idx)
        {
            case 0:
            {
                if (_pos_d.y < 0)
                {
                    _pos_d.vy += ad * dt;
                    _pos_d.y  += _pos_d.vy * dt;
                }
                else if (_pos_d.y >= 0 && _pos_d.y < 0.5-DIST_THRES)
                {
                    _pos_d.vy += -ad * dt;
                    _pos_d.y  += _pos_d.vy * dt;
                }
                else
                {
                    _flags.yGoal = true;
                }

                if (_pos_d.z < 1.1)
                {
                    _pos_d.vz += ad * dt;
                    _pos_d.z  += _pos_d.vz * dt;
                }
                else if (_pos_d.z >= 1.1 && _pos_d.z < 1.6-DIST_THRES)
                {
                    _pos_d.vz += -ad * dt;
                    _pos_d.z  += _pos_d.vz * dt;
                }
                else
                {
                    _flags.zGoal = true;
                }
                break;
            }
            case 1:
            {
                // y from 0.5 to -0.5
                if (_pos_d.y > 0)
                {
                    _pos_d.vy += -ad * dt;
                    _pos_d.y  += _pos_d.vy * dt;
                }
                else if (_pos_d.y <= 0 && _pos_d.y > -0.5+DIST_THRES)
                {
                    _pos_d.vy += ad * dt;
                    _pos_d.y  += _pos_d.vy * dt;
                }
                else
                {
                    _flags.yGoal = true;
                }

                // constant z
                _flags.zGoal = true;
                break;
            }
            case 2:
            {
                // y from -0.5 to 0.5
                if (_pos_d.y > 0)
                {
                    _pos_d.vy += -ad * dt;
                    _pos_d.y  += _pos_d.vy * dt;
                }
                else if (_pos_d.y <= 0 && _pos_d.y > -0.5+DIST_THRES)
                {
                    _pos_d.vy += ad * dt;
                    _pos_d.y  += _pos_d.vy * dt;
                }
                else
                {
                    _flags.yGoal = true;
                }

                // z from 1.6 to 0.6
                if (_pos_d.z > 1.1)
                {
                    _pos_d.vz += -ad * dt;
                    _pos_d.z  += _pos_d.vz * dt;
                }
                else if (_pos_d.z <= 1.1 && _pos_d.z > 0.6+DIST_THRES)
                {
                    _pos_d.vz += ad * dt;
                    _pos_d.z  += _pos_d.vz * dt;
                }
                else
                {
                    _flags.zGoal = true;
                }
                break;
            }
        }

        if (_flags.yGoal && _flags.zGoal)
        {
            _wp_idx += 2;
            _flags.yGoal = false;
            _flags.zGoal = false;
        }

        if (_wp_idx >= 3)
        {
            _flags.atGoal = true;
            _wp_idx = 0;
        }

        _ftimer += dt;
    }
    else
    {
        _wp_idx = 0;
        _timer = 0;
        _pos_d.vy = 0;
        _pos_d.vz = 0;
    }

    _t0 = AP_HAL::micros64();
    //hal.uartA->printf("yd %f, zd %f, vyd %f, vzd %f, timer %f, idx %d\n", _pos_d.y, _pos_d.z,_pos_d.vy, _pos_d.vz,_ftimer, _wp_idx);
    return _pos_d;
}

float AC_PathPlanner::pitch_oscillator(float pilot_pitch)
{
    _check_flight_init();

    float target_pitch = 0;

    float manual_pitch_trim = rc().channel(CH_7)->get_radio_in() - 1500; // range: 1000-2000

    // in centidegree
    if (_flags.start_flight)
    {
        target_pitch = PITCH_OSCILLATION * sin(2*M_PI/PITCH_PERIOD * _ftimer) * 100 + manual_pitch_trim;
    }
    else
    {
        target_pitch = pilot_pitch;
    }
    
    return target_pitch;
}


void AC_PathPlanner::get_default_target(float yd, float zd)
{
    _wp_y[0] = yd;
    _wp_z[0] = zd;

    _pos_d.y = yd;
    _pos_d.z = zd;
}

void AC_PathPlanner::_check_flight_init()
{
    int ch7 = rc().channel(CH_7)->get_radio_in();

    if (ch7 > 1500)
    {
        _flags.start_flight = true;
    }
    else
    {
        _flags.start_flight = false;
        _flags.atGoal = false;
        _flags.yGoal = false;
        _flags.zGoal = false;
        _pos_d.y = _wp_y[0];
        _pos_d.z = _wp_z[0];
        _timer = 0;
        _ftimer = 0;
        _wp_idx = 0;
    }
}

void AC_PathPlanner::get_current_pos(position_t pos)
{
    _pos = pos;
}
