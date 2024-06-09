# ESP32 with ILI9341 and LVGL with freeRTOS

> [!Note]
> Syaifullah Hilmi Ma'arij | 22/497775/TK/54568

This source code is created to fulfill the final project assignment for the  _Sistem Berbasis Mikroprosesor_ (_Microprocessor-Based Systems_)  course. The focus of the project is to create a program using:
- ESP32 Microcontroller
- ILI9341 Display (LCD TFT SPI 2.4/2.8/3.2 inch)
- FreeRTOS
- Arduino/ESP-IDF Framework
- LVGL Library is recommended

## Table of Contents

- [Introduction](#introduction)
- [Components](#components)
- [Pin Connections](#pin-connections)
- [Installation](#installation)
- [Usage](#usage)
- [Acknowledgements](#acknowledgements)

## Introduction

This project demonstrates the integration of multiple sensors with an ESP32 microcontroller. The sensor data is displayed on an ILI9341 TFT screen using the LVGL library. The touch functionality is handled using an XPT2046 touchscreen controller. The project uses FreeRTOS for managing tasks efficiently.

## Components

- **ESP32 Microcontroller**
- **ILI9341 TFT Display**
- **XPT2046 Touchscreen Controller**
- **DHT22 Sensor** (Temperature and Humidity)
- **HC-SR04 Sensor** (Ultrasonic Distance)

## Pin Connections
> [!TIP]
> Match the pins below to simplify your work.

### ESP32 to ILI9341 TFT Display

| ILI9341 Pin | ESP32 Pin   |
|-------------|-------------|
| VCC         | 3.3V        |
| GND         | GND         |
| CS          | GPIO 15     |
| RESET       | GPIO 12     |
| DC/RS       | GPIO 2      |
| SDI(MOSI)   | GPIO 13     |
| SCK         | GPIO 14     |
| LED         | 3.3V        |
| SDO(MISO)   | GPIO 16     |
| T_CLK       | GPIO 25     |
| T_CS        | GPIO 33     |
| T_DIN       | GPIO 32     |
| T_OUT       | GPIO 39     |
| T_IRQ       | GPIO 36     |

### ESP32 to DHT22 Sensor

| DHT22 Pin   | ESP32 Pin   |
|-------------|-------------|
| VCC         | 3.3V        |
| DATA        | GPIO 19     |
| GND         | GND         |

### ESP32 to HC-SR04 Sensor

| HC-SR04 Pin | ESP32 Pin   |
|-------------|-------------|
| VCC         | 5V          |
| TRIG        | GPIO 5      |
| ECHO        | GPIO 18     |
| GND         | GND         |

#### References
> - [My Reference about TFT ILI9341 Display](https://randomnerdtutorials.com/esp32-tft-touchscreen-display-2-8-ili9341-arduino/)
> - [Learn more about ESP32 Pinout Reference](https://lastminuteengineers.com/esp32-pinout-reference/)


## Installation
> [!WARNING]
> It's important to have a good understanding of how the TFT ILI9341 pins work. All these pins are defined in the `User_Setup.h` file.
> If you're using the same pins as mentioned above, then you should be fine. However, please note that the touch pins (TFT) are defined separately in `main.cpp` due to the use of the XPT2046 library for the touchscreen part.

1. **Clone the repository or download ZIP:**

    ```sh
    git clone https://github.com/saaip7/SBM-Project_2.git
    ```

2. **Install the required libraries:**

    The necessary libraries are already included in the repository. These libraries include:

    - Adafruit Unified Sensor
    - LVGL (version 8.3.11)
    - TFT_eSPI
    - XPT2046_Touchscreen
    - UI (exported from SquareLine Studio for LVGL)

3. **Configure the TFT_eSPI PIN:**
   If you want to configure the TFT ILI9341 pins by yourself:
    - Navigate to `TFT_eSPI/User_Setup.h`
    - Modify the pin connections to match your own pins.

## Usage

1. **Open the project in PlatformIO IDE (VSCode):**

    - Open the project folder in PlatformIO IDE (VSCode).

2. **Build and upload the code to ESP32:**

    - Ensure that the correct board and port are selected in the `platformio.ini` file. (Sometimes you don't have to)
    - Click the Upload button in PlatformIO IDE to compile and upload the code to the ESP32.

3. **Run the project:**

    - Once the code is uploaded, the ESP32 will start reading sensor data and display it on the ILI9341 TFT screen using the LVGL library. The data will be updated periodically.

## Acknowledgements

> - **LVGL (Light and Versatile Graphics Library):** Used for creating the graphical user interface. The layout was designed using SquareLine Studio, which simplifies LVGL-based GUI design.
> - **Arduino Framework:** Used for developing and uploading the code.
> - **FreeRTOS:** Used for task management and scheduling.
> - **SquareLine Studio:** A helpful tool for designing and managing LVGL-based GUIs.

---

Feel free to contribute to this project by opening issues or submitting pull requests.
