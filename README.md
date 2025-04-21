# Satellite Tracking and Data Logging System

This project uses various sensors (BMP180, MPU6050, HMC5883L, GPS) and the ESP32 module to build a satellite tracking system. The system logs data to an SD card and sends it to another device using the ESP-NOW protocol. Additionally, when the satellite stops, the buzzer activates, and all data is continuously recorded.

## Features
- BMP180 Barometer: Measures altitude, temperature, and pressure.
- MPU6050 Accelerometer and Gyroscope: Measures acceleration and rotation.
- HMC5883L Magnetometer: Measures magnetic field.
- NEO-6M GPS: Collects GPS data.
- SD Card: Logs data to an SD card.
- ESP-NOW: Sends data to another ESP32 device.

## Libraries Used
This project uses the following libraries:

- `Adafruit_BMP085`: [Adafruit BMP085 Library](https://github.com/adafruit/Adafruit-BMP085-Unified)
- `Adafruit_MPU6050`: [Adafruit MPU6050 Library](https://github.com/adafruit/Adafruit_MPU6050)
- `TinyGPSPlus`: [TinyGPSPlus Library](https://github.com/mikalhart/TinyGPSPlus)
- `HMC5883L`: [HMC5883L Library](https://github.com/ArduPilot/Arduino-HMC5883L)
- `SD`: Arduino SD Library
- `ESP-NOW`: ESP32's native wireless communication library

## Usage

### Hardware Connections:
- **BMP180 Barometer**: Connected via I2C.
- **MPU6050 Accelerometer and Gyroscope**: Connected via I2C.
- **HMC5883L Magnetometer**: Connected via I2C.
- **NEO-6M GPS**: Connected via UART1.
- **SD Card**: Connected via SPI.
- **Buzzer**: Connected to a digital pin (pin 4).

### Circuit Diagram:
The connections are clearly explained in the code. However, if there is any ambiguity regarding the connections, feel free to contact me.

### Running the Code:
1. Open Arduino IDE and select the appropriate ESP32 board.
2. Install the required libraries.
3. Upload the code to the ESP32.
4. Open the Serial Monitor to check if the data is being received and transmitted correctly.

## License

MIT License (MIT)

Copyright (c) 2025 Efe Özer

This software and associated documentation files are provided "as is", without warranty of any kind, express or implied, including but not limited to the warranties of merchantability, fitness for a particular purpose, and noninfringement. In no event shall the authors or copyright holders be liable for any claim, damages, or other liability, whether in an action of contract, tort, or otherwise, arising from, out of, or in connection with the software or the use or other dealings in the software.

