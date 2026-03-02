#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "Shares.h"
#include "Driver.h"
#include "MotorController.h"

//================= CONFIG =================
#ifndef RW_I2C_ADDR
#define RW_I2C_ADDR 0x20   // unique per wheel (set per ESP32 build)
#endif

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ    100000

#define MOTOR_CTRL_HZ 500
//==========================================

//------------- I2C register map -------------
static constexpr uint8_t REG_TORQUE_CMD = 0x00; // write 4 bytes float torque [N*m]
static constexpr uint8_t REG_RPM_AND_FAULTS = 0x04; // read 8 bytes: float rpm + uint32 faults
static constexpr uint8_t REG_FAULTS = 0x08; // read 4 bytes: uint32 faults

// Shared vars (defined here, declared in Shares.h)
Share<float>    torque_cmd("Torque Command");
Share<float>    speed_actual("Speed Actual");
Share<float>    speed_cmd_last("Speed Cmd Last");
Share<uint32_t> rw_fault_flags("RW Fault Flags");
Queue<uint32_t> edge_time(4, "FGOUT Rising Edge Timestamp");

// I2C "current register" pointer set by the last master write
static volatile uint8_t reg_ptr = 0;

// Hardware singletons
Driver          Peripheral;
MotorController Controller;

//--------------- Helpers ---------------
static inline void wireWriteU32(uint32_t v)
{
    Wire.write(reinterpret_cast<uint8_t*>(&v), sizeof(v));
}

static inline void wireWriteF32(float v)
{
    Wire.write(reinterpret_cast<uint8_t*>(&v), sizeof(v));
}

//================ I2C CALLBACKS ================
// Master write pattern supported:
//  1) write 1 byte register pointer
//  2) optionally followed by payload
//
// Torque write:
//  write: [REG_TORQUE_CMD][4 bytes float]
void onI2CReceive(int nbytes)
{
    if (nbytes <= 0) return;

    reg_ptr = Wire.read();
    nbytes--;

    if (reg_ptr == REG_TORQUE_CMD && nbytes >= 4)
    {
        uint8_t buf[4];
        for (int i = 0; i < 4; i++) buf[i] = Wire.read();

        float tq;
        memcpy(&tq, buf, sizeof(float));

        torque_cmd.put(tq);
    }

    // Drain anything else
    while (Wire.available()) (void)Wire.read();
}

// Master read pattern supported:
//  write: [REG_xx]      (no payload)
//  read:  N bytes
void onI2CRequest()
{
    if (reg_ptr == REG_RPM_AND_FAULTS)
    {
        const float rpm = speed_actual.get();
        const uint32_t faults = rw_fault_flags.get();
        wireWriteF32(rpm);
        wireWriteU32(faults);
        return;
    }

    if (reg_ptr == REG_FAULTS)
    {
        const uint32_t faults = rw_fault_flags.get();
        wireWriteU32(faults);
        return;
    }

    // Default: return zero (4 bytes)
    float zero = 0.0f;
    wireWriteF32(zero);
}

//==================== Task: compute RPM from FGOUT ====================
// edge_time queue receives rising-edge timestamps from the Driver FGOUT ISR.
void task_readActual(void* parameters)
{
    (void)parameters;
    uint32_t last_time = 0;

    for (;;)
    {
        const uint32_t current_time = edge_time.get();  // blocks until an edge timestamp is available

        if (last_time != 0)
        {
            const uint32_t dt_us = current_time - last_time;
            if (dt_us > 0)
            {
                const float dt_s = dt_us / 1.0e6f;
                const float freq_hz = 1.0f / dt_s;

                // NOTE: Your earlier empirical scaling
                const float rpm = freq_hz * 15.0f;
                speed_actual.put(rpm);
            }
        }

        last_time = current_time;
    }
}

//==================== Task: motor control loop ====================
// Reads commanded torque and converts it to a speed command.
// For bring-up we run open-loop speed via PWM CLKIN.
void motorTask(void* arg)
{
    (void)arg;

    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / MOTOR_CTRL_HZ);

    for (;;)
    {
        const float tq = torque_cmd.get();
        const float last_spd = speed_cmd_last.get();

        // Convert torque -> speed command
        const float omega_cmd_rpm = Controller.calculate_omega(tq, last_spd);

        Peripheral.cmd_speed_PWM(omega_cmd_rpm);
        speed_cmd_last.put(omega_cmd_rpm);

        // One-shot torque command: clear after consuming
        torque_cmd.put(0.0f);

        vTaskDelayUntil(&last, period);
    }
}

//==================== SETUP ====================
void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("SACS Reaction Wheel Controller starting...");

    // Initialize shares so .get() does not block forever on first use
    torque_cmd.put(0.0f);
    speed_actual.put(0.0f);
    speed_cmd_last.put(0.0f);
    rw_fault_flags.put(0);

    // I2C slave init
    Wire.begin(RW_I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ);
    Wire.onReceive(onI2CReceive);
    Wire.onRequest(onI2CRequest);

    // Driver init (SPI, PWM, FGOUT ISR)
    Peripheral.begin();

    // Tasks
    xTaskCreate(task_readActual, "RPM Calc", 4096, nullptr, 9, nullptr);
    xTaskCreate(motorTask, "MotorControl", 4096, nullptr, 3, nullptr);

    Serial.printf("RW node ready | I2C addr = 0x%02X\n", RW_I2C_ADDR);
}

//==================== LOOP ====================
// All work is done in tasks; loop only prints debug.
void loop()
{
    static uint32_t last_ms = 0;
    const uint32_t now = millis();

    if (now - last_ms >= 250)
    {
        last_ms = now;

        const float rpm = speed_actual.get();
        const float last_cmd = speed_cmd_last.get();
        const uint32_t faults = rw_fault_flags.get();

        Serial.print("rpm="); Serial.print(rpm, 2);
        Serial.print(" | cmd_rpm="); Serial.print(last_cmd, 2);
        Serial.print(" | faults=0x"); Serial.println(faults, HEX);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}
