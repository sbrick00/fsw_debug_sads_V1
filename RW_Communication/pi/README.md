# Raspberry Pi ↔ ESP32 Reaction Wheel Node (I2C bring-up)

## ESP32 register map (must match firmware)
- `0x00` TORQUE_CMD (write 4 bytes float, N*m)
- `0x04` RPM_AND_FAULTS (read 8 bytes: float rpm, uint32 faults)
- `0x08` FAULTS (read 4 bytes: uint32 faults)

## Build (on Raspberry Pi)
```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake i2c-tools

# Enable I2C in raspi-config if needed
# sudo raspi-config  -> Interface Options -> I2C -> Enable

mkdir -p build
cd build
cmake ..
cmake --build . -j
```

## Run
```bash
# Example: talk to addr 0x20
./rw_i2c 20
```

## Wiring notes
- Pi SDA/SCL are **3.3V**. Ensure ESP32 I2C pins are also 3.3V.
- Use a shared ground.
- Add pullups (2.2k–4.7k) on SDA/SCL if not already on your board.
