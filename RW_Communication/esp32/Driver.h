#ifndef DRIVER_H
#define DRIVER_H

#include <Arduino.h>
#include <SPI.h>

// Low-level DRV8308 interface + FGOUT interrupt timestamping.
//
// - begin(): configures pins, attaches FGOUT ISR, starts VSPI, sets up CLKIN PWM
// - handleISR(): pushes micros() timestamps into edge_time queue (declared in Shares.h)

class Driver
{
public:
    Driver();

    void begin();

    // Chip-select helpers (active LOW)
    void scs_begin() { digitalWrite(PIN_SCS, LOW); }
    void scs_end()   { digitalWrite(PIN_SCS, HIGH); }

    // Basic controls
    void enable()  { digitalWrite(PIN_EN, HIGH); }
    void disable() { digitalWrite(PIN_EN, LOW); }

    void brake()   { digitalWrite(PIN_BRAKE, HIGH); }
    void unbrake() { digitalWrite(PIN_BRAKE, LOW); }

    void set_dir(bool cw) { digitalWrite(PIN_DIR, cw ? HIGH : LOW); }

    // DRV8308 SPI register access (7-bit addr)
    void drv_write(uint8_t addr7, uint16_t message);
    uint16_t drv_read(uint8_t addr7);

    // Speed commands
    void cmd_speed_SPI(float speed_rpm);
    void cmd_speed_PWM(float speed_rpm);

    // ISR trampoline
    static void IRAM_ATTR ISR_wrapper();
    void handleISR();

private:
    // SPI
    uint8_t PIN_SCLK;
    uint8_t PIN_MISO;
    uint8_t PIN_MOSI;

    // DRV8308 pins
    uint8_t PIN_SCS;
    uint8_t PIN_EN;
    uint8_t PIN_CLKIN;
    uint8_t PIN_FGOUT;
    uint8_t PIN_FAULTn;
    uint8_t PIN_LOCKn;
    uint8_t PIN_RESET;
    uint8_t PIN_BRAKE;
    uint8_t PIN_DIR;

    // ISR
    volatile uint32_t _lastEdgeTime;

    static Driver* _instance;
};

#endif // DRIVER_H
