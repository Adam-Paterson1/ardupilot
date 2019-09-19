#include "AP_FakeSensor.h"

extern const AP_HAL::HAL &hal;

AP_FakeSensor::AP_FakeSensor()
{
    data.pos.y    = 0;
    data.pos.z    = 0;
    data.target_pos.y = 0;
    data.target_pos.z = 0;
    data.status   = NotConnected;

}

void AP_FakeSensor::get_AHRS(AP_AHRS_View* ahrs)
{
    _ahrs = ahrs;
}

void AP_FakeSensor::get_motors(AP_MotorsMulticopter* motors)
{
    _motors = motors;
}

void AP_FakeSensor::init()
{
    _flag_init = false;
    _uart = hal.uartE;  // using GPS2 port
    if (_uart == nullptr)    {return;}
    _uart->begin(ODROID_BAUDRATE);
    gcs().send_text(MAV_SEVERITY_INFO, "Initialising fake sensor communication with Odroid...\n");
}

void AP_FakeSensor::update()
{
    // get position data from Odroid
    _get_pos();

    // update radio values
    _read_radio();

    // update AHRS
    _read_AHRS();

    // get motor thrust outputs
    _motors->get_motor_thrust_output(data.mthrust_out);

    // assign timestamp to data
    data.ts = AP_HAL::millis();

    //_uart->printf("$hello\n");
    // send Pixhawk 2 data back to Odroid
    static uint8_t counter = 0;
    counter++;
    // if (counter > 20) {
    //     counter = 0;
    //     _uart->printf("%lu %f %f %f %f %f %f\n", 
    //             data.ts, data.roll, data.pitch, data.yaw, data.mthrust_out[0], data.mthrust_out[1], data.mthrust_out[2]);
    // }
    if (counter > 7) {
        counter = 0;
        _uart->printf("%lu %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n", 
                data.ts, data.KF_pos.y, data.KF_pos.z, data.mthrust_out[0], data.roll, data.pitch, data.yaw, 
                data.target_thrust, data.target_roll, data.target_pitch, data.target_yaw_rate, 
                data.perr.pterm_y, data.perr.iterm_y, data.perr.dterm_y,
                data.perr.pterm_z, data.perr.iterm_z, data.perr.dterm_z);
    }
}

void AP_FakeSensor::_read_AHRS()
{
    data.roll = _ahrs->roll;
    data.pitch = _ahrs->pitch;
    data.yaw = _ahrs->yaw;
}


void AP_FakeSensor::_get_pos()
{
    if (_uart == nullptr)    {return;}

    int16_t nbytes = _uart->available();
    if (nbytes) _prev_pos = data.pos;   // storing the previous data for comparison

    _buflen = 0;
    // If we last saw a v that means we care about this number and those following until a \n
    while (_uart->available() > 0) {
        char c = _uart->read();
        if (c == 'v') {
            // Reset
            i = 0;
            partial_buffer = true;
        } else if (partial_buffer) {
            // Not V and not in progress its garbage
            // Not V and we are in progress, read it
            if (c != '#') {
                _linebuf[i] = c;
                i++;
            } else {
                //End of transmission
                // _uart->printf("Running");
                _linebuf[i] = '\0';
                int j = 0;
                char delim[] = ",";
                float buff[10];
                char *ptr = strtok(_linebuf, delim);
                while(ptr != NULL)
                {
                    buff[j] = atof(ptr);
                    ptr = strtok(NULL, delim);
                    j++;
                }
                // _uart->printf("$");
                // for (j = 0; j < 2; j++)
                // {
                //     _uart->printf("%f ", buff[j]);
                // }
                // _uart->printf("\n");
                // Update position
                data.pos.y = buff[0];
                data.pos.z = buff[1];
                data.target_pos.y = buff[2];
                data.target_pos.z = buff[3];
                data.gains.py = buff[4];
                data.gains.iy = buff[5];
                data.gains.dy = buff[6];
                data.gains.pz = buff[7];
                data.gains.iz = buff[8];
                data.gains.dz = buff[9];
                data.pos.nset = _prev_pos.nset + 1;
                // Reset line buffer
                for (i = 0; i <DATA_BUF_SIZE; i++) {
                    _linebuf[i] = ' ';
                }
                // Reset flags
                i = 0;
                partial_buffer = false;
            }
        }
    }    
    // position update
}

void AP_FakeSensor::get_KF_pos(position_t p)
{
    data.KF_pos = p;
}

void AP_FakeSensor::get_error_terms(pos_error_t errs)
{
    data.perr.pterm_z = errs.pterm_z;
    data.perr.iterm_z = errs.iterm_z;
    data.perr.dterm_z = errs.dterm_z;
    data.perr.pterm_y = errs.pterm_y;
    data.perr.iterm_y = errs.iterm_y;
    data.perr.dterm_y = errs.dterm_y;
    // static uint8_t counter = 0;
    // counter++;
    // if (counter >5) {
    //     counter = 0;
    //     hal.uartE->printf("$ Yar %f \n", errs.dterm_z);
    // }
}

void AP_FakeSensor::get_target_things(float thrust, float roll, float pitch, float yaw_rate)
{
    data.target_thrust = thrust;
    data.target_roll = roll;
    data.target_pitch = pitch;
    data.target_yaw_rate = yaw_rate;
}

bool AP_FakeSensor::data_is_ok()
{
    if (data.status == Bad)    return false;
    else                        return true;
}

void AP_FakeSensor::_read_radio()
{
    data.ch.roll    = rc().channel(CH_1)->get_radio_in();
    data.ch.pitch   = rc().channel(CH_2)->get_radio_in();
    data.ch.thr     = rc().channel(CH_3)->get_radio_in();
    data.ch.yaw     = rc().channel(CH_4)->get_radio_in();
    data.ch.aux5    = rc().channel(CH_5)->get_radio_in();
    data.ch.aux6    = rc().channel(CH_6)->get_radio_in();
    data.ch.aux7    = rc().channel(CH_7)->get_radio_in();
    data.ch.aux8    = rc().channel(CH_8)->get_radio_in();
    data.ch.aux9    = rc().channel(CH_9)->get_radio_in();
}

void AP_FakeSensor::write_log()
{
    // write log to dataflash
    DataFlash_Class::instance()->Log_Write("FAKE", "TimeUS,ALTD,CR,eDIST,RNGALT",
                                           "smmmm", "F0000", "Qffff",
                                           AP_HAL::micros64(),
                                           (double) data.AC_alt_target,
                                           (double) data.AC_cr,
                                           (double) data.dist_err,
                                           (double) data.target_rangefinder_alt);
}
