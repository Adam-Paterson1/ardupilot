#include "Copter.h"


/*
 * Init and run calls for althold, flight mode 24
 */

// althold_init - initialise althold controller
bool Copter::ModeTunnel::init(bool ignore_checks)
{
    backstepping->reset_integral();
    backstepping->reset_mode_switch();

    return true;
}

// Tunnel_run - runs the backstepping controller
// should be called at 100hz or more
void Copter::ModeTunnel::run()
{
    // get gains for backstepping
    // backstepping->get_gains(g.BS_y_k1,g.BS_y_k2,g.BS_y_k3,g.BS_z_k1,g.BS_z_k2,g.BS_z_k3);

    // // apply SIMPLE mode transform to pilot inputs
    // update_simple_mode();

    // // get pilot desired lean angles
    // float target_roll, target_pitch;
    // get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // backstepping->get_pilot_lean_angle_input(target_roll, copter.aparm.angle_max);

    // // get pilot's desired yaw rate
    // float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // // reset integral if on the ground
    // if (!motors->armed() || !motors->get_interlock())
    // {
    // //    backstepping->reset_integral();
    //     backstepping->reset_mode_switch();
    // }

    // motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // // generate waypoint
    // pp.get_default_target(g.BS_yd, g.BS_zd);

    // pp.get_current_pos(pkf->get_pos());
    // position_t target_pos = pp.run_circular_trajectory();
    // //position_t target_pos = pp.run_diagonal_trajectory();
    // //position_t target_pos = pp.run_setpoint();

    // // call backstepping controller
    // backstepping->get_target_pos(target_pos.y, target_pos.z);
    // backstepping->get_target_vel(target_pos.vy, target_pos.vz);
    // backstepping->get_target_accel(target_pos.ay, target_pos.az);

    // backstepping->pos_update(pkf->get_pos());

    // backstepping->update_alt_controller();

    // target_roll = backstepping->update_lateral_controller();

    // // call attitude controller
    // attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // // get control outputs
    // pos_sensor.read_controller(backstepping->perr, backstepping->get_u1());


    //pkf->write_log();
    //pkf->print_shit();
}
