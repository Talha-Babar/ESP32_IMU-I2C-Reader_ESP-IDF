# ESP32 I2C-Based IMU Data Reader (ESP-IDF Framework)

This project demonstrates using the ESP32 with the I2C protocol to interface with an Inertial Measurement Unit (IMU) sensor (e.g., BMI323). The ESP32 initializes the sensor, configures its features, and continuously reads acceleration and gyroscope data.

---

## Features

- **I2C Communication**: Interfaces with the IMU sensor using the I2C protocol.
- **Sensor Initialization**: Configures the IMU for acceleration and gyroscope data readings.
- **Real-Time Data Acquisition**:
  - Acceleration on X, Y, and Z axes.
  - Gyroscope readings on X, Y, and Z axes.
- **Reset Functionality**: Uses a GPIO pin to reset the IMU.

---

## Prerequisites

- ESP32 development board.
- Compatible IMU sensor (e.g., BMI323).
- ESP-IDF installed and configured.
- Proper hardware connections for I2C and reset pin.

---

## How It Works

1. **I2C Initialization**:

   - The ESP32 configures the I2C master with the following parameters:
     - SCL Pin: GPIO 22
     - SDA Pin: GPIO 21
     - Frequency: 400 kHz
   - The reset pin is configured as GPIO 5.

2. **IMU Configuration**:

   - The sensor is configured using I2C writes to specific registers:
     - Enable engine.
     - Configure IO and sensor features.
     - Set up the accelerometer and gyroscope.

3. **Data Acquisition**:

   - The ESP32 reads acceleration and gyroscope data from the IMU registers:
     - Acceleration on X, Y, and Z axes.
     - Gyroscope readings on X, Y, and Z axes.
   - The data is printed to the serial monitor in real time.

4. **Reset Mechanism**:
   - The reset pin is toggled at the start to ensure the IMU is properly initialized.
