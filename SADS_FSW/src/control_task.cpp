#include "shared_state.h"
#include "timing.h"
#include "rw_bus.h"
#include "sads_balance_cubemodel.h" // --- NEW: Simulink Header ---

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
    // --- UPDATED: Switched from 200ms (5Hz) to 100ms (10Hz) ---
    const auto period = 100ms; 
    const float period_sec = 0.100; 

    // --- NEW: Instantiate and Initialize Simulink Model ---
    sads_balance_cubemodelModelClass simulink_model;
    simulink_model.initialize();

    // Existing RW PID Variables
    std::array<float, 3> eul{};
    float yaw = 0.0f;
    float yaw_rate = 0.0f;
    float u = 0;
    float Wn = 0.3f; 
    float Izz = 0.14; 
    float zeta = 0.7f; 
    float Kp = Izz * Wn * Wn; 
    float Kd = 2.0f * zeta * Wn * Izz; 

    std::array<float, NUM_WHEELS> torque_cmd{};
    torque_cmd.fill(0.0f);

    auto next = steady_clock::now();
    while (!stop_flag.load())
    {
        next += period;

        // Take a thread-safe snapshot of the current state
        auto snap = shared.snapshot();

        if (snap.current_mode == ControlMode::STEPPER_SIMULINK) 
        {
            // --- NEW: SIMULINK / STEPPER MODE ---

            // 1. Feed the C++ snapshot into the Simulink Inputs (rtU)
            simulink_model.rtU.roll   = snap.est_euler.roll_rad;
            simulink_model.rtU.pitch  = snap.est_euler.pitch_rad;
            simulink_model.rtU.yaw    = snap.est_euler.yaw_rad;
            simulink_model.rtU.p_rate = snap.est_imu.omega[0];
            simulink_model.rtU.q_rate = snap.est_imu.omega[1];
            simulink_model.rtU.r_rate = snap.est_imu.omega[2];

            // 2. Run the 10Hz calculation
            simulink_model.step();

            // 3. Write outputs back to the shared state
            std::lock_guard<std::mutex> lock(shared.mtx);
            shared.stepper_cmds[0] = simulink_model.rtY.stepper1_cmd;
            shared.stepper_cmds[1] = simulink_model.rtY.stepper2_cmd;
            shared.stepper_cmds[2] = simulink_model.rtY.stepper3_cmd;
            shared.last_ctrl_us = now_us();
        }
        else if (snap.current_mode == ControlMode::RW_PID) 
        {
            // --- EXISTING: REACTION WHEEL PID MODE ---
            
            yaw = snap.est_euler.yaw_rad;
            yaw_rate = snap.est_imu.omega[2];
            u = Kp*yaw + Kd*yaw_rate;

            if (u > 0.05f || u < -0.02f) u = (u > 0) ? 0.05f : -0.02f;
            torque_cmd.fill(u);

            {
                std::lock_guard<std::mutex> lock(shared.mtx);
                shared.wheel_torque_cmd_nm = torque_cmd;
                shared.last_ctrl_us = now_us();
            }

            // Command and readback to ESP32s
            for (int i = 0; i < NUM_WHEELS; ++i)
            {
                if (!rws[i].ok) continue;
                (void)rws[i].bus.setSlave(rws[i].addr);

                if (std::fabs(torque_cmd[i]) > 1e-5f) {
                    if (!rws[i].bus.writeFloat(REG_TORQUE_CMD, torque_cmd[i])) {
                        std::fprintf(stderr, "[control] RW%d write torque failed...\n", i);
                        rws[i].ok = false;
                        std::lock_guard<std::mutex> lock(shared.mtx);
                        shared.wheels[i].present = false;
                        continue;
                    }
                }
                float rpm = 0.0f; uint32_t faults = 0;
                if (rws[i].bus.readRPMandFaults(rpm, faults)) {
                    std::lock_guard<std::mutex> lock(shared.mtx);
                    shared.wheels[i].present = true;
                    shared.wheels[i].rpm = rpm;
                    shared.wheels[i].faults = faults;
                    shared.last_rw_us = now_us();
                }
            }
        }
        else 
        {
            // --- IDLE / SAFESTATE MODE ---
            std::lock_guard<std::mutex> lock(shared.mtx);
            shared.wheel_torque_cmd_nm.fill(0.0f);
            shared.stepper_cmds = {0, 0, 0};
        }

        sleep_until(next);
    }
}