#include "Copter.h"

extern const AP_HAL::HAL &hal;
/*
 * Init and run calls for althold, flight mode 26
 */

bool roll_override = false;
bool prev_roll_override = false;
bool thrust_override = false;
bool prev_thrust_override = false;
float _dt = 0.0025f;
int FULL_TIME = 600; //1.5 seconds
int ROLL_TIME = 50; //125ms
int roll_override_timer = 0;
int thrust_override_timer = 0;
float prev_pilot_thrust = 0;
int roll_transition_counter = 0;
int thrust_transition_counter = 0;
int roll_transition_range = 20; //50ms
int thrust_transition_range = 100; //250ms

float transitionCoefficient (float from, float to, int counter, int range) {
  float dist = to - from;
  float step = dist / range; // 200 for 0.5s transition
  return from + step * (range - counter);
}
// althold_init - initialise althold controller
bool Copter::ModeTunnelPID::init(bool ignore_checks)
{
    backstepping->reset_PID_integral();
    backstepping->reset_mode_switch();
    prev_pilot_thrust = get_pilot_desired_throttle(channel_throttle->get_control_in());
    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Copter::ModeTunnelPID::run()
{
    // SETUP ----------------------------------------------------------------
    float target_roll, target_pitch, target_thrust, target_yaw_rate;
    float pilot_roll, pilot_pitch, pilot_thrust, pilot_yaw_rate;

       // reset integral if on the ground
    if (!motors->armed() || !motors->get_interlock())
    {
        backstepping->reset_PID_integral();
        zero_throttle_and_relax_ac();
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // PILOT ----------------------------------------------------------------
    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(pilot_roll, pilot_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    pilot_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_thrust = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // CONTROL ----------------------------------------------------------------
    // Set PID Gains for dynamic tuning
    backstepping->set_PID_gains(pos_sensor.data.gains);
    // Set target
    backstepping->set_target_pos(pos_sensor.data.target_pos);
    // Update current position
    backstepping->pos_update(pkf->get_pos());
    // Calculate outputs
    target_roll = backstepping->update_PID_lateral_controller(copter.aparm.angle_max);
    target_thrust = backstepping->update_PID_vertical_controller();
    target_yaw_rate = backstepping->update_PID_yaw_controller();
    pos_sensor.get_error_terms(backstepping->perr);
    // OVERRIDES ----------------------------------------------------------------
    // If pilot wiggling thrust or roll, let them take over for 1.5 seconds
    if ((pilot_roll > 0.1 * copter.aparm.angle_max) || (pilot_roll < -0.1 * copter.aparm.angle_max)) {
        roll_override = true;
        roll_override_timer = ROLL_TIME;
    }
    if (((pilot_thrust - prev_pilot_thrust) > 0.009) || ((pilot_thrust - prev_pilot_thrust) < -0.009)) {
        thrust_override = true;
        thrust_override_timer = FULL_TIME;
    }
    // If timed out reset to copter control
    if (roll_override_timer <= 0) {
        roll_override = false;
    }
    if (thrust_override_timer <= 0) {
        thrust_override = false;
    }
    // If they just took over or lost control provide a smooth ramp between their signal and controller taking over
    if (roll_override != prev_roll_override) {
        roll_transition_counter = roll_transition_range;
    }
    if (thrust_override != prev_thrust_override) {
        thrust_transition_counter = thrust_transition_range;
    }
    //Apply mixing to get target roll + thrust
    if (roll_override == true) {
        target_roll = transitionCoefficient(1, 0.25, roll_transition_counter, roll_transition_range)*target_roll + transitionCoefficient(0, 0.75, roll_transition_counter, roll_transition_range)*pilot_roll;
        roll_override_timer--;
    } else {
        target_roll = transitionCoefficient(0.25, 1, roll_transition_counter, roll_transition_range)*target_roll + transitionCoefficient(0.75, 0, roll_transition_counter, roll_transition_range)*pilot_roll;
    }

    if (thrust_override == true) {
        target_thrust = transitionCoefficient(1, 0.3, thrust_transition_counter, thrust_transition_range)*target_thrust + transitionCoefficient(0, 0.7, thrust_transition_counter, thrust_transition_range)*pilot_thrust;
        thrust_override_timer--;
    } else {
        target_thrust = transitionCoefficient(0.3, 1, thrust_transition_counter, thrust_transition_range)*target_thrust + transitionCoefficient(0.7, 0, thrust_transition_counter, thrust_transition_range)*pilot_thrust;
    }

    // Decrement counters and update whether pilot took control this cycle
    if (roll_transition_counter > 0) {
        roll_transition_counter--;
    }
    if (thrust_transition_counter > 0) {
        thrust_transition_counter--;
    }
    prev_roll_override = roll_override;
    prev_thrust_override = thrust_override_timer;

    //Pass through pilot control
    prev_pilot_thrust = pilot_thrust;
    if ((pilot_yaw_rate > 200) || (pilot_yaw_rate < -200)) {
        target_yaw_rate = pilot_yaw_rate;
    }
    // target_yaw_rate = pilot_yaw_rate;
    target_pitch = pilot_pitch;
    // OUTPUT ----------------------------------------------------------------
    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(target_thrust, true, g.throttle_filt);

    // update position and position target
    pos_sensor.get_target_things(target_thrust, target_roll, target_pitch, target_yaw_rate);
    // static uint8_t counter = 0;
    // counter++;
    // if (counter > 40) {
    //     counter = 0;
    //     hal.uartE->printf("%lu %f %f %f %f %f %f %f %f\n", 
    //             pos_sensor.data.ts, pos_sensor.data.mthrust_out[0], pos_sensor.data.roll, pos_sensor.data.pitch, pos_sensor.data.yaw, target_thrust, target_roll, target_pitch, target_yaw_rate);
    // }
    // static uint8_t counter = 0;
    // counter++;
    // if (counter > 20) {
    //     counter = 0;
        // if (roll_override == true) {
        //     hal.uartE->printf("$ Pilot override");
        // }
    //     //hal.uartE->printf("$ Pilot %f %f %f %f", pilot_thrust, pilot_roll, pilot_pitch, pilot_yaw_rate);
    //     hal.uartE->printf("$ ITS WORKING %f %f %f %f", target_thrust, target_roll, target_pitch, target_yaw_rate);
    // }
    
}
