# Portable-weather-station

## Overview

This project is a **battery-powered portable weather station** capable of monitoring and logging localised environmental data in real-time. Built around the **Particle Photon** microcontroller and connected via **Bluetooth Low Energy (BLE)**, it interfaces with a custom **mobile app (Blynk platform)** for seamless data viewing. Designed with portability, accuracy and efficiency in mind. The system measures temperature, humidity, pressure, wind speed, wind direction and rainfall - ideal for applications in agriculture, aviation, construction etc.

---

## Features

- **Weather Parameters Monitored:**
  - Temperature
  - Relative Humidity
  - Atmospheric Pressure
  - Wind Speed and Direction
  - Rainfall

- **Mobile App:**
  - Real-time display (via Blynk)
  - Historical data trends (charts)
  - Sensor status monitoring
  - Battery status and notifications

- **Communication:**
  - Bluetooth Low Energy (JDY-08 Module)
  - No GSM/Wi-Fi needed (low power)

- **Data Logging:**
  - MicroSD card logging in CSV format
  - Timestamped entries
  - Log toggle via mobile app

- **Power System:**
  - Rechargeable 3.7V 1050mAh Li-ion battery
  - 16+ hours operation on full charge
  - Safe charging via integrated Adafruit Li-ion charge controller

---

## Hardware Components

| Component                     | Purpose                                       |
|------------------------------|-----------------------------------------------|
| Particle Photon              | Microcontroller Unit                          |
| Bosch BME280                 | Temperature, Pressure & Humidity Sensor       |
| Davis 6410                  | Wind Speed & Direction Sensor (reconditioned) |
| BGT WS-601ABS2              | Tipping Bucket Rain Sensor                    |
| JDY-08 BLE Module            | Bluetooth Communication                       |
| MicroSD Card Module          | Data Logging                                  |
| 3.7V 1050mAh Li-ion Battery | Power Supply                                  |
| Adafruit Charger (MCP73833) | Safe Charging Circuitry                       |

---

## Mobile App Interface (Blynk)

The app is organized into 3 tabs:
1. **Live Measurements** - Real-time sensor readings and logger toggle.
2. **Trends** - Visualize data over time using super charts.
3. **General** - Battery status, Bluetooth connection, and notifications.

> *Note: The app requires an internet connection to access Blynk's authentication servers during login, even though hardware communicates locally via BLE.*

---

## Power Optimization

- Wi-Fi disabled on the controller to reduce power consumption
- Custom sleep mode logic active when no wind or rain sensors are connected
- Low power LED indicators for charging state
- Battery voltage and status monitored via ADC

---


## Technical Highlights

- Uses **interrupt-driven** logic for wind and rain sensors
- Sensor disconnection detection & status feedback
- Time-stamped data logging with SD card detection
- Designed for **South African** climate extremes
- Fully custom **PCB design** using EasyEDA
- Modular port system using RJ45 connectors
- Custom enclosure with labelled ports and battery mount

---


## References

Detailed references and explanations are available in the included report (/Docs/Portable Weather Station_Project Report.pdf).

---
