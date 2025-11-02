# Aireen

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
<img width="1630" height="1320" alt="image" src="https://github.com/user-attachments/assets/f193976f-f28b-44bb-b88a-077a928ec0e3" />

## Pinout Reference

The table below lists the default ESP32 pin assignments as defined in `include/config.h`. Adjust as needed.

| ESP32 Pin | Sensor/Actuator                         |
|-----------|-----------------------------------------|
| 3.3v      | LDR sensor primary pin                  |
| 3.3v      | TDS sensor VCC pin                      |
| GND       | LDR sensor secondary pin (via resistor) |
| GND       | TDS sensor GND pin                      |
| GPIO35    | LDR sensor secondary pin                |
| GPIO36    | TDS sensor signal pin                   |

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































