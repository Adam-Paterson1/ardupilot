#include "Copter.h"

extern const AP_HAL::HAL &hal;
/*
 * Init and run calls for althold, flight mode 26
 */

// althold_init - initialise althold controller
bool Copter::ModeTunnelPID::init(bool ignore_checks)
{
    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    backstepping->reset_PID_integral();

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Copter::ModeTunnelPID::run()
{
    // ======================
    // do not need to touch
    // ======================

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    // ======================
    // do not need to touch
    // ======================


    // reset integral if on the ground
    if (!motors->armed() || !motors->get_interlock())
    {
        backstepping->reset_PID_integral();
    }

    backstepping->get_PID_gains(g.alt_hold_p, g.TUNLpid_y_p, g.TUNLpid_y_i, g.TUNLpid_y_d);

    backstepping->get_pilot_lean_angle_input(target_roll, copter.aparm.angle_max);

    // generate waypoint
    pp.get_default_target(g.BS_yd, g.BS_zd);

    pp.get_current_pos(pkf->get_pos());

    //position_t target_pos = pp.run_circular_trajectory();
    //position_t target_pos = pp.run_diagonal_trajectory();
    position_t target_pos = pp.run_setpoint();

    // update position and position target
    backstepping->get_target_pos(target_pos.y, target_pos.z);
    backstepping->get_target_vel(target_pos.vy, target_pos.vz);

    backstepping->pos_update(pkf->get_pos());

    // adjust climb rate using rangefinder
    float target_climb_rate = backstepping->get_PID_alt_climb_rate();

    // call position controller
    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    pos_control->update_z_controller();

    target_roll = backstepping->update_PID_lateral_controller();

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

}
