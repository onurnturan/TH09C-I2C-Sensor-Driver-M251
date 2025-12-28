# TH09C-I2C-Sensor-Driver-M251
**Overview**
This repository contains a low-level I2C driver for the TH09C digital temperature and humidity sensor, implemented on the Nuvoton M251 (Cortex-M23) microcontroller. The project demonstrates high-precision environmental data acquisition and robust data validation using a custom CRC7 checksum algorithm.
**Key Features**
Precision Sensing: Reads high-resolution raw data for both Temperature and Humidity.
Data Validation (CRC7): Implements a manual 7-bit Cyclic Redundancy Check to ensure data integrity over the I2C bus.
Multi-Unit Conversion: Automatically converts raw sensor values into Kelvin, Celsius, and Fahrenheit.
I2C Protocol Implementation: Utilizes register-based I2C communication with error handling and status checks.
Non-Blocking Logic: Designed with appropriate delays and timing for sensor stabilization.
**Signal Flow**
The system initializes the I2C0 peripheral, triggers a measurement by writing to the SENS_START register, waits for the conversion time, and then reads 6 bytes of data (3 bytes for Temperature, 3 bytes for Humidity) sequentially.
**Hardware Components**
MCU: Nuvoton M251 series.
Sensor: TH09C (Digital T/H Sensor).
Interface: I2C (Inter-Integrated Circuit) @ 100kHz.
