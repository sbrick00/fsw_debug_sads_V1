#include <Arduino.h>

#include "MotorController.h"

MotorController::MotorController()
    : Kp(0.0f), Ki(0.0f), Kd(0.0f),
      J(0.001712f),
      last_update_ms(0),
      omega_rad_s(0.0f)
{
}

MotorController::MotorController(float Kp_, float Ki_, float Kd_)
    : Kp(Kp_), Ki(Ki_), Kd(Kd_),
      J(0.001712f),
      last_update_ms(0),
      omega_rad_s(0.0f)
{
}

float MotorController::calculate_omega(float torque_cmd_Nm, float speed_cmd_last_rpm)
{
    unsigned long now_ms = millis();

    // First call: initialize internal omega from last commanded RPM
    if (last_update_ms == 0)
    {
        omega_rad_s    = speed_cmd_last_rpm * (2.0f * PI / 60.0f); // RPM -> rad/s
        last_update_ms = now_ms;
        return speed_cmd_last_rpm;
    }

    float dt_s = (now_ms - last_update_ms) / 1000.0f;
    last_update_ms = now_ms;

    // Guardrails
    if (dt_s <= 0.0f) dt_s = 0.001f;
    if (dt_s > 0.5f)  dt_s = 0.5f;

    // Angular acceleration [rad/s^2]
    float alpha = torque_cmd_Nm / J;

    // Integrate [rad/s]
    omega_rad_s += alpha * dt_s;

    // Clamp to a physical limit (tune)
    const float omega_max_rad_s = 2.0f * PI * 2000.0f / 60.0f;
    if (omega_rad_s >  omega_max_rad_s) omega_rad_s =  omega_max_rad_s;
    if (omega_rad_s < -omega_max_rad_s) omega_rad_s = -omega_max_rad_s;

    // rad/s -> RPM
    return omega_rad_s * (60.0f / (2.0f * PI));
}

float MotorController::calculate_input(float, float)
{
    // Speed-loop PID not implemented for bring-up.
    return 0.0f;
}

void MotorController::set_gains_SPI(float Kp_NEW, float Ki_NEW, float Kd_NEW)
{
    Kp = Kp_NEW;
    Ki = Ki_NEW;
    Kd = Kd_NEW;
}
