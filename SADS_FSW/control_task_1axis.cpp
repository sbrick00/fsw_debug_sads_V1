#include "shared_state.h"
#include "timing.h"
#include "rw_bus.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cmath>

// Must match ESP32 register map
static constexpr uint8_t REG_TORQUE_CMD = 0x00;

void control_task(SharedState& shared,
                  std::atomic<bool>& stop_flag,
                  std::array<RwBus, NUM_WHEELS>& rws)
{
    using namespace std::chrono;
    const auto period = 200ms; // 5 Hz
    const float period_sec = 0.200; // [seconds]

    std::array<float, 4> quat{};
    float yaw = 0.0f;
    float yaw_rate = 0.0f;
    float u = 0;
    float Kp = 0.006;
    float Kd = 0.015;

    std::array<float, NUM_WHEELS> torque_cmd{};
    torque_cmd.fill(0.0f);

    auto next = steady_clock::now();
    while (!stop_flag.load())
    {
        next += period;

        {
            std::lock_guard<std::mutex> lock(shared.mtx);

            quat = { shared.est_imu.q[0],
                    shared.est_imu.q[1],
                    shared.est_imu.q[2],
                    shared.est_imu.q[3] };

            yaw_rate = shared.est_imu.omega[2];
        }

        float qw = quat[0];
        float qx = quat[1];
        float qy = quat[2];
        float qz = quat[3];

        // Optional but highly recommended: renormalize
        float norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (norm > 1e-6f) {
            qw /= norm;
            qx /= norm;
            qy /= norm;
            qz /= norm;
        }

        // Yaw (Z-axis rotation)
        float siny_cosp = 2.0f * (qw*qz + qx*qy);
        float cosy_cosp = 1.0f - 2.0f * (qy*qy + qz*qz);

        float yaw = std::atan2(siny_cosp, cosy_cosp);

        u = Kp*yaw + Kd*yaw_rate;

        // Constrain torque command to max limits of 0.05 Nm in either direction
        if (u > 0.05f || u < -0.02f) u = (u > 0) ? 0.05f : -0.02f;
        torque_cmd.fill(u);

        // Publish commanded torque for telemetry (even if wheel absent)
        {
            std::lock_guard<std::mutex> lock(shared.mtx);
            shared.wheel_torque_cmd_nm = torque_cmd;
            shared.last_ctrl_us = now_us();
        }

        // Command and readback
        for (int i = 0; i < NUM_WHEELS; ++i)
        {
            if (!rws[i].ok) continue;

            (void)rws[i].bus.setSlave(rws[i].addr);

            if (std::fabs(torque_cmd[i]) > 1e-5f)
                if (!rws[i].bus.writeFloat(REG_TORQUE_CMD, torque_cmd[i]))
                {
                    std::fprintf(stderr,
                        "[control] RW%d write torque failed (addr 0x%02X) — disabling\n",
                        i, rws[i].addr);
                    rws[i].ok = false;

                    std::lock_guard<std::mutex> lock(shared.mtx);
                    shared.wheels[i].present = false;
                    continue;
                }

            float rpm = 0.0f; uint32_t faults = 0;
            if (rws[i].bus.readRPMandFaults(rpm, faults))
            {
                std::lock_guard<std::mutex> lock(shared.mtx);
                shared.wheels[i].present = true;
                shared.wheels[i].rpm = rpm;
                shared.wheels[i].faults = faults;
                shared.last_rw_us = now_us();
            }
        }

        sleep_until(next);
    }
}