# ESP32 WiFi Micro Quadcopter (v3)

A lightweight flight controller and web-based telemetry/control interface for an ESP32-based micro quadcopter running brushed 8520 coreless motors.

The firmware implements a 500 Hz cascade PID control loop (angle -> rate) with yaw damping, I2C signal filtering, a desaturating motor mixer, and an integrated FreeRTOS web server for real-time telemetry and tuning over Wi-Fi.

## Architecture & Implementation Details

### Control Loop Structure
- **Inner Loop (Rate PID)**: Runs at 500 Hz on FreeRTOS Core 1. Tracks desired angular rates directly using filtered gyro feedback.
- **Outer Loop (Angle P)**: Converts pitch and roll attitude errors into rate setpoints clamped to +/-220 deg/s.
- **Yaw Damper**: Rate-only loop targeting 0 deg/s to counteract reaction torque and motor imbalances.
- **Filters**:
  - Gyro low-pass: PT1 filter at 110 Hz.
  - D-term low-pass: PT1 filter at 70 Hz.
  - Glitch detection: Rejects single-sample rate spikes exceeding 250 deg/s.
  - Dynamic attitude filter: Decays accelerometer weight when net acceleration deviates more than +/-0.30g from 1g.
- **Mixer**:
  - Dynamically shifts base throttle to prevent differential thrust collapse when stabilization commands hit limits.
  - Enforces a 6% idle spin floor while armed to prevent motor stall.
  - Anti-windup reduces integrators when saturation occurs.

### Task Isolation
- **Core 1 (`ctl`)**: Sensor acquisition (I2C), cascade PID calculations, and 20 kHz LEDC PWM updates (priority 4).
- **Core 0 (`web`)**: Wi-Fi SoftAP and HTTP web server (priority 2), preventing network traffic from blocking the real-time flight task.

## Hardware Configuration

### Bill of Materials
- **Microcontroller**: ESP32 NodeMCU-32S
- **IMU**: MPU6050 (I2C address: `0x68` or `0x69`)
- **Barometer (Optional)**: BMP280 / BME280 (I2C address: `0x76` or `0x77`)
- **Motors**: 4x 8520 coreless brushed DC motors
- **Propellers**: 55mm - 65mm (2x CW, 2x CCW)
- **Motor Drivers**: 4x AO3400A N-channel MOSFETs with pulldown resistors and flyback diodes
- **Power**: 1S LiPo (3.7V, 500-650mAh, >=25C)
- **Frame**: 3D-printed 120mm frame (STL models included in repository)

### Pinout

#### Motor Outputs (20 kHz PWM)
| Motor | Position | Direction | GPIO |
| :--- | :--- | :--- | :--- |
| M1 | Front-Left | CW | GPIO 16 |
| M2 | Rear-Left | CCW | GPIO 17 |
| M3 | Rear-Right | CW | GPIO 18 |
| M4 | Front-Right | CCW | GPIO 19 |

*Note: Orientation has the board antenna facing Right, with the USB/CMD end pointing forward.*

#### Sensors (I2C Bus)
| Signal | ESP32 GPIO | MPU6050 | BMP280 (Optional) |
| :--- | :--- | :--- | :--- |
| SDA | GPIO 21 | SDA | SDA |
| SCL | GPIO 22 | SCL | SCL |
| 3.3V | 3V3 | VCC | VCC |
| GND | GND | GND | GND |

## 3D Models

The repository includes STL files for 3D printing:
- `120_final_nocam.stl`: 120mm motor-to-motor frame with integrated 8.5mm motor mounts.
- `base.stl`: Electronics and battery mounting deck.

Recommended material: PLA+, PETG, or Nylon. 100% infill recommended for motor arm rigidity.

## Building and Flashing

### Using PlatformIO (Recommended)
1. Install [PlatformIO Core](https://docs.platformio.org/en/latest/core/index.html) or the PlatformIO IDE extension for VS Code.
2. Clone the repository:
   ```bash
   git clone https://github.com/himanshusangwan250-hash/esp32-wifi-quadcopter.git
   cd esp32-wifi-quadcopter
   ```
3. Connect the ESP32 via USB and flash:
   ```bash
   pio run -t upload
   ```
4. Open the serial console at 115200 baud:
   ```bash
   pio device monitor -b 115200
   ```

### Using Arduino IDE
1. Install the ESP32 board support package (by Espressif).
2. Select board `ESP32 Dev Module`.
3. Open `main.cpp` (or rename to a `.ino` sketch).
4. Verify baud rate is set to 115200 and upload.

## Operation & Safety

### Network Credentials
- **Access Point SSID**: `MicroQuad`
- **Password**: `flyquad123`
- **Web UI Address**: `http://192.168.4.1`

### Pre-Flight Sequence
1. Place quadcopter on a flat, stationary surface before powering on.
2. The IMU auto-calibrates gyro and accelerometer offsets on boot (takes ~2 seconds).
3. Connect your device to the `MicroQuad` Wi-Fi network and open `http://192.168.4.1`.
4. Verify real-time sensor readouts in the telemetry box.
5. If the quad was moved during boot, place it level and click **LEVEL (flat & still)**.
6. Use **Bench Mode** (props off) to test control response and motor directions by tilting the frame by hand.

### Safety Cutoffs
- **Physical Emergency Stop**: Hold the ESP32 `BOOT` button (GPIO0) to latch an immediate hard shutdown.
- **Web Interface Kill**: Red **KILL** button immediately sets all PWM outputs to 0. Must be cleared via **RESET KILL** before re-arming.
- **Tilt Protection**: Disarms automatically if roll or pitch exceeds +/-60 degrees.
- **Failsafe Timeout**: Automatically cuts throttle if the browser connection heartbeat is lost for more than 3000 ms.
- **Arming Interlock**: Arming is rejected if tilted >20 degrees or if the boot calibration detected excessive initial tilt.

## Gain Configuration

Default tuning parameters (stored in non-volatile flash via ESP32 `Preferences`):

| Parameter | Default | Range | Description |
| :--- | :--- | :--- | :--- |
| `KP_ANGLE` | 4.0 | 0.0 - 20.0 | Outer attitude loop P gain (target dps per degree error) |
| `KP_RATE` | 0.15 | 0.0 - 2.0 | Inner rate loop P gain (% PWM per dps error) |
| `KI_RATE` | 0.10 | 0.0 - 2.0 | Inner rate loop I gain (active above 20% base throttle) |
| `KD_RATE` | 0.00 | 0.0 - 1.0 | Inner rate loop D gain (PT1 filtered at 70 Hz) |
| `KP_YAW` | 0.25 | 0.0 - 3.0 | Yaw rate damping P gain |
| `KI_YAW` | 0.20 | 0.0 - 3.0 | Yaw rate damping I gain |

Values can be modified in real time from the web interface and committed to flash memory with the **Save to flash** button.

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
