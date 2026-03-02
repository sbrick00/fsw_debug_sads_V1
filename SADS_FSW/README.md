# SADS FSW (First Draft)

Threads:
- IMU: 100 Hz (MTi-03 over USB serial)
- Estimator: 100 Hz (identity copy)
- Control: 20 Hz (zero commanded torque)
- Telemetry: 5 Hz (HTTP/AJAX)

Run:
```bash
mkdir -p build
cd build
cmake ..
make -j4
./sads_fsw
```

Open:
- http://localhost:8080
- or http://<pi-ip>:8080
