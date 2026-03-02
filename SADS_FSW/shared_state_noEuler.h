#pragma once

#include <array>
#include <cstdint>
#include <mutex>

#include "mti03_driver.h"

static constexpr int NUM_WHEELS = 3;

struct WheelState {
    bool present = false;     // discovered at runtime
    float rpm = 0.0f;
    uint32_t faults = 0;
};

// Single source of truth shared across tasks.
struct SharedState
{
    mutable std::mutex mtx;

    // Raw measurement (IMU task @ 100 Hz)
    ImuState meas_imu{};

    // "Estimator" output (currently: identity copy)
    ImuState est_imu{};

    // Wheels
    std::array<WheelState, NUM_WHEELS> wheels{};
    std::array<float, NUM_WHEELS> wheel_torque_cmd_nm{}; // commanded torque (Nm)

    // Time bookkeeping (steady-clock microseconds)
    uint64_t last_meas_us = 0;
    uint64_t last_est_us  = 0;
    uint64_t last_ctrl_us = 0;
    uint64_t last_rw_us   = 0;

    struct Snapshot
    {
        ImuState est_imu;
        std::array<WheelState, NUM_WHEELS> wheels;
        std::array<float, NUM_WHEELS> wheel_torque_cmd_nm;
        uint64_t t_us = 0;
    };

    Snapshot snapshot() const
    {
        std::lock_guard<std::mutex> lock(mtx);
        Snapshot s;
        s.est_imu = est_imu;
        s.wheels = wheels;
        s.wheel_torque_cmd_nm = wheel_torque_cmd_nm;
        s.t_us = last_meas_us;
        return s;
    }
};
