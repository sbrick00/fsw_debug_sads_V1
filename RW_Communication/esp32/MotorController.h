#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>

// Minimal torque -> speed integrator.
//
// This is intentionally simple for bring-up: it integrates commanded torque
// using a constant wheel inertia J, producing a new commanded speed (RPM).

class MotorController
{
public:
    MotorController();
    MotorController(float Kp_, float Ki_, float Kd_);

    float calculate_omega(float torque_cmd_Nm, float speed_cmd_last_rpm);

    // (Reserved) speed-loop PID – not currently used
    float calculate_input(float speed_setpt_rpm, float speed_actual_rpm);

    void set_gains_SPI(float Kp_NEW, float Ki_NEW, float Kd_NEW);

private:
    float Kp, Ki, Kd;

    const float J;                 // wheel inertia [kg*m^2]
    unsigned long last_update_ms;
    float omega_rad_s;             // internal wheel speed state [rad/s]
};

#endif // MOTORCONTROLLER_H
