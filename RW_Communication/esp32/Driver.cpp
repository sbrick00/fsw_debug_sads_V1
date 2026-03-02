#include <Arduino.h>
#include <SPI.h>

#include "Driver.h"
#include "Shares.h"

// Use VSPI on ESP32
static SPIClass vspi(VSPI);

Driver* Driver::_instance = nullptr;

Driver::Driver()
{
    _lastEdgeTime = 0;

    // VSPI pins
    PIN_SCLK = 18;
    PIN_MISO = 19;
    PIN_MOSI = 23;

    // DRV8308 pins (adjust to match your PCB)
    PIN_SCS    = 5;
    PIN_EN     = 13;
    PIN_CLKIN  = 4;
    PIN_FGOUT  = 34;
    PIN_FAULTn = 35;
    PIN_LOCKn  = 32;
    PIN_RESET  = 27;
    PIN_BRAKE  = 14;
    PIN_DIR    = 26;
}

void Driver::begin()
{
    // GPIO
    pinMode(PIN_SCS, OUTPUT);
    digitalWrite(PIN_SCS, HIGH); // idle high (inactive)

    pinMode(PIN_EN, OUTPUT);
    pinMode(PIN_RESET, OUTPUT);
    pinMode(PIN_BRAKE, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);

    pinMode(PIN_FGOUT, INPUT);
    pinMode(PIN_FAULTn, INPUT);
    pinMode(PIN_LOCKn, INPUT);

    // Keep reset asserted briefly
    digitalWrite(PIN_RESET, LOW);
    delay(5);
    digitalWrite(PIN_RESET, HIGH);

    // Default: not braking
    digitalWrite(PIN_BRAKE, LOW);

    // Attach FGOUT interrupt
    _instance = this;
    attachInterrupt(digitalPinToInterrupt(PIN_FGOUT), ISR_wrapper, RISING);

    // Enable driver
    enable();

    // SPI
    vspi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);

    // PWM for CLKIN (channel 0)
    ledcSetup(0, 20000, 8);      // default 20 kHz, 8-bit
    ledcAttachPin(PIN_CLKIN, 0);
}

void Driver::drv_write(uint8_t addr7, uint16_t message)
{
    vspi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    scs_begin();
    delayMicroseconds(1);

    vspi.transfer((0u << 7) | (addr7 & 0x7F));
    vspi.transfer((uint8_t)(message >> 8));
    vspi.transfer((uint8_t)(message & 0xFF));

    delayMicroseconds(1);
    scs_end();
    vspi.endTransaction();
    delayMicroseconds(5);
}

uint16_t Driver::drv_read(uint8_t addr7)
{
    vspi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    scs_begin();
    delayMicroseconds(1);

    vspi.transfer((1u << 7) | (addr7 & 0x7F));
    uint8_t msb_r = vspi.transfer(0x00);
    uint8_t lsb_r = vspi.transfer(0x00);

    delayMicroseconds(1);
    scs_end();
    vspi.endTransaction();
    delayMicroseconds(5);

    return ((uint16_t)msb_r << 8) | lsb_r;
}

void Driver::cmd_speed_SPI(float speed_rpm)
{
    // Map RPM into DRV8308 SPEED register (12-bit). Tune MAX_RPM empirically.
    if (speed_rpm <= 0.0f)
    {
        drv_write(0x0B, 0x0000);
        return;
    }

    const float MAX_RPM = 2760.0f;
    const uint16_t MAX_RAW = 0x0FFF;

    float scaled = (speed_rpm / MAX_RPM) * (float)MAX_RAW;
    if (scaled < 0.0f) scaled = 0.0f;
    if (scaled > (float)MAX_RAW) scaled = (float)MAX_RAW;

    drv_write(0x0B, (uint16_t)scaled);
}

void Driver::cmd_speed_PWM(float speed_rpm)
{
    // Your earlier mapping: FGOUT [Hz] = RPM/15. Using same for CLKIN.
    if (speed_rpm <= 0.0f)
    {
        ledcWriteTone(0, 0);
        return;
    }

    const double freq_hz = speed_rpm / 15.0;
    ledcWriteTone(0, freq_hz);
}

void IRAM_ATTR Driver::ISR_wrapper()
{
    if (_instance) _instance->handleISR();
}

void Driver::handleISR()
{
    // Keep ISR minimal: capture timestamp and enqueue.
    _lastEdgeTime = micros();
    edge_time.put(_lastEdgeTime);
}
