# Ventilator Control System Prototype

This repository contains a basic prototype framework for a Ventilator Control System developed to test hardware components. The project includes both embedded C code for the microcontroller and a Python GUI application for real-time data visualization.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Getting Started](#getting-started)
  - [Embedded Code](#embedded-code)
  - [Python GUI Application](#python-gui-application)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Introduction

The Ventilator Control System prototype is designed to interface with pressure sensors, flow sensors, and control valves to simulate the operation of a ventilator. The embedded code runs on an STM32 microcontroller, while the Python application provides a graphical user interface (GUI) for monitoring real-time data.

This project serves as a foundation for testing hardware components and developing further functionalities for a ventilator system.

## Features

- **Embedded C Code:**
  - Reads pressure data from ADC.
  - Controls valves using GPIO pins.
  - Communicates with a host computer via UART.
  - Utilizes DMA for efficient data transfer.
  - Follows AUTOSAR coding standards.

- **Python GUI Application:**
  - Real-time plotting of pressure and volume data.
  - Serial communication with the microcontroller.
  - Calculates volume using flow integration.
  - Displays maximum pressure and volume values.
  - User-friendly interface with Matplotlib and Tkinter.

## Hardware Requirements

- **Microcontroller:**
  - STM32 NUCLEO-F439ZI development board (or compatible STM32 board).

- **Sensors and Actuators:**
  - Pressure sensor connected to ADC input.
  - Flow sensor connected to appropriate interface.
  - Valves connected to GPIO pins.

- **Communication Interface:**
  - Serial connection (e.g., USB-to-Serial adapter) between the microcontroller and the host computer.

## Software Requirements

- **Embedded Development:**
  - STM32CubeIDE or equivalent IDE for STM32 development.
  - STM32CubeMX for generating initialization code (optional).

- **Python Environment:**
  - Python 3.x
  - Required Python packages:
    - `numpy`
    - `matplotlib`
    - `tkinter` (usually included with Python)
    - `pyserial`

## Getting Started

### Embedded Code

1. **Setup the Development Environment:**
   - Install STM32CubeIDE or your preferred IDE.
   - Ensure all necessary drivers and tools for STM32 development are installed.

2. **Clone the Repository:**
   ```bash
   git clone https://github.com/yourusername/ventilator-control-system.git
