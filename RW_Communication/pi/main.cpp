#include <chrono>
#include <csignal>
#include <cstdio>
#include <thread>

#include "i2c_rw.h"

// Must match ESP32 register map
static constexpr uint8_t REG_TORQUE_CMD     = 0x00;
static constexpr uint8_t REG_RPM_AND_FAULTS = 0x04;
static constexpr uint8_t REG_FAULTS         = 0x08;

static volatile bool g_run = true;

static void onSigInt(int)
{
    g_run = false;
}

int main(int argc, char** argv)
{
    // Usage: ./rw_i2c <i2c_addr_hex>  (default 0x20)
    uint8_t addr = 0x20;
    if (argc >= 2)
    {
        unsigned int tmp = 0;
        if (std::sscanf(argv[1], "%x", &tmp) == 1)
            addr = static_cast<uint8_t>(tmp & 0xFF);
    }

    std::signal(SIGINT, onSigInt);

    I2CRW bus("/dev/i2c-1");
    if (!bus.open())
    {
        std::perror("open /dev/i2c-1");
        return 1;
    }
    if (!bus.setSlave(addr))
    {
        std::perror("ioctl I2C_SLAVE");
        return 1;
    }

    std::printf("Talking to RW node at I2C addr 0x%02X\n", addr);

    // Example: step a torque command while reading back RPM
    float torque = 0.00f;
    int counter = 0;

    while (g_run)
    {
        // Send torque command (N*m)
        if (!bus.writeFloat(REG_TORQUE_CMD, torque))
        {
            std::perror("writeFloat");
        }

        float rpm = 0.0f;
        uint32_t faults = 0;
        if (bus.readRPMandFaults(rpm, faults))
        {
            std::printf("tq=%.4f N*m | rpm=%8.2f | faults=0x%08X\n", torque, rpm, faults);
        }
        else
        {
            std::perror("readRPMandFaults");
        }

        /*
        if(1 <= counter <= 50) {torque = 0.001; counter++;}
        else if (51 <= counter <= 100) {torque = -0.001; counter ++;}
        else counter = 1;
        */

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::puts("Exiting.");
    return 0;
}
