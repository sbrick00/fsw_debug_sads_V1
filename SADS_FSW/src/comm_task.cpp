#include "shared_state.h"
#include "timing.h"

#include <atomic>
#include <chrono>
#include <iostream>
#include <cstring>

// Linux networking headers
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

void comm_task(SharedState& shared, std::atomic<bool>& stop_flag)
{
    using namespace std::chrono;
    
    // ***IMPORTANT***: MAKE THIS MATCH THE SIMULINK MODEL FIXED STEP SIZE
    const auto period = 20ms; 

    // Socket setup
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "[comm_task] Failed to create UDP socket\n";
        return;
    }

    struct sockaddr_in dest_addr;
    std::memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(5000);                   // Port for Simulink to listen on
    dest_addr.sin_addr.s_addr = inet_addr("100.89.134.72"); // Send to external

    // Init a flat array tothe 10 float values (40 bytes tot)
    std::array<float, 10> udp_payload{};

    auto next = steady_clock::now();
    while (!stop_flag.load())
    {
        next += period;

        // Get a thread snapshot of the mutex board
        auto s = shared.snapshot();

        // --- 3. Pack the payload ---
        // Quaternion *eta first smh*
        udp_payload[0] = s.est_imu.q[0];
        udp_payload[1] = s.est_imu.q[1];
        udp_payload[2] = s.est_imu.q[2];
        udp_payload[3] = s.est_imu.q[3];
        
        // Body rates [rad/s]
        udp_payload[4] = s.est_imu.omega[0];
        udp_payload[5] = s.est_imu.omega[1];
        udp_payload[6] = s.est_imu.omega[2];
        
        // Euler ang [rad]
        udp_payload[7] = s.est_euler.roll_rad;
        udp_payload[8] = s.est_euler.pitch_rad;
        udp_payload[9] = s.est_euler.yaw_rad;

        // Send the UDP packet
        // Cast to a void pointer and specify the byte size (10*4 = 40)
        sendto(sock, udp_payload.data(), sizeof(udp_payload), 0,
               (struct sockaddr*)&dest_addr, sizeof(dest_addr));

        // Sleep until the next cycle. Keeps timing rigid
        sleep_until(next);
    }

    // Clean up
    close(sock);
    std::puts("[comm_task] UDP socket closed.");
}