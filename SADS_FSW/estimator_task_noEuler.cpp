#include "shared_state.h"
#include "timing.h"

#include <atomic>
#include <chrono>

void estimator_task(SharedState& shared, std::atomic<bool>& stop_flag)
{
    using namespace std::chrono;
    const auto period = 10ms; // 100 Hz (once per measurement)

    auto next = steady_clock::now();
    while (!stop_flag.load())
    {
        next += period;

        {
            std::lock_guard<std::mutex> lock(shared.mtx);
            shared.est_imu = shared.meas_imu; // identity estimator
            shared.last_est_us = now_us();
        }

        sleep_until(next);
    }
}
