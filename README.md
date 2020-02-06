# esp32-adt7410
## Introduction
Basic ESP-IDF library for Analog Devices ADT7410 I2C temperature sensor.

## Usage
Please initialize I2C driver prior to calling `adt7410_init()`.

## Features and Limitations
* User selected 16-bit and 13-bit measurement resolution
* Only supports continuous conversion

## Roadmap
* Implement Temperature Set Point functionality in library
* Implement conversions other than continuous conversion
