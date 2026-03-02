#include "imu/mti03_driver.h"
#include <iostream>
#include <unistd.h>

int main() {
    MTi03Driver imu("/dev/ttyUSB0");

    if (!imu.start()) {
        std::cerr << "Failed to start IMU driver\n";
        return 1;
    }

    while (true) {
        ImuState s = imu.getState();

        if (s.valid) {
            std::cout
                << "Quat: "
                << s.q[0] << " "
                <<  s.q[1] << " "
                <<  s.q[2] << " "
                <<  s.q[3]
                << " | Rates (rad/s): "
                <<  s.omega[0] << " "
                <<  s.omega[1] << " "
                <<  s.omega[2]
                << " | Mag: "
                << s.mag[0] << " "
                << s.mag[1] << " "
                << s.mag[2]
                << " | T: "
                << s.timestamp
                << "\n";
        }

        usleep(10000); // 100 Hz consumer loop
    }
}