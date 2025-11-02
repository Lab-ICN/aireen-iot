# ICN Prototype - 24

> [!NOTE]
> This project was created as part of a research initiative on hydroponic smart agriculture at the Information Centric Networking (ICN) Laboratory in 2025. It is currently a work in progress and may be subject to changes in the near future.

> [!WARNING]
> The project is developed using PlatformIO IDE. To contribute or run the project, please **use Visual Studio Code (VSCode)** and **install the required extensions** as specified in this repository. 

## About
This research project explores the implementation of smart agriculture in a hydroponic plant system. 
Its primary objective is to minimize human intervention by leveraging IoT-based automation. 
Sensors continuously monitor environmental and water-quality parameters, and decisions are made automatically to maintain optimal conditions for plant growth.
This repository contains the IoT-related firmware, hardware interface code, and automation logic. 
It is part of a larger initiative to build a self-sustaining hydroponic farming system.

## System Design
![image](https://github.com/user-attachments/assets/240896ac-fae4-4765-8057-cbfc79a9f6a5)

## Pinout Reference
The table below lists the default ESP32 pin assignments as defined in `include/config.h`. Adjust as needed.

| ESP32 Pin | Sensor/Actuator |
| :--- | :--- |
| 3.3v | LDR sensor primary pin |
| | TDS sensor VCC pin |
| GND | LDR sensor secondary pin (via resistor) |
| | TDS sensor GND pin |
| A0 (GPIO 2) | LDR sensor secondary pin |
| A1 (GPIO 4) | TDS sensor signal pin |

## Software Dependencies
This code is using some third party libraries provided by Arduino, such as follows.
- `PubSubClient.h` for MQTT client implementation  

## Future Works
- [x] Interfacing LDR sensor
- [x] Interfacing TDS sensor
- [x] Implementing MQTT-based data publishing to a remote server
- [ ] Interfacing Analog pH meter
- [ ] Integrating relay control for pumps and other actuators

## Known Issues
- [ ] LDR sensor readings may be unreliable in low-light environments, potentially due to unsuitable resistor selection or calibration issues.
