# Bridge SHM Firmware (ESP32-S3)

This directory contains the edge signal processing firmware for the ESP32-S3 microcontroller.

## Hardware Configuration & Pinout

| Component | Pin Name | ESP32-S3 GPIO | Description |
| :--- | :--- | :--- | :--- |
| **MPU9250** | SDA | **GPIO 41** | I2C Data Line (400 kHz Fast Mode) |
| **MPU9250** | SCL | **GPIO 42** | I2C Clock Line |
| **SW-420** | DOUT | **GPIO 47** | Auxiliary Vibration Event Trigger |
| **Power** | VCC | **3.3V** | Regulated Logic Power |
| **Power** | GND | **GND** | Common Ground |

## Required Arduino Libraries
Install the following libraries via the Arduino Library Manager:
1. `arduinoFFT` by Enrique Condes (v1.6.x API)
2. `MPU9250_asukiaaa` by Asuki Kono
3. `Wire` (Built-in ESP32 core)

## Arduino IDE Board Settings
- **Board**: `ESP32S3 Dev Module`
- **USB CDC On Boot**: `Enabled`
- **Flash Mode**: `QIO 80MHz`
- **Flash Size**: `8MB (or 4MB)`
- **Upload Speed**: `921600`
- **Baud Rate**: `115200`
