# Ventilator Control System Prototypes

This repository contains three prototype implementations for ventilator control systems, each designed to test different approaches and hardware configurations. The project includes embedded C code for STM32 microcontrollers and a Python GUI application for real-time monitoring.

## Prototypes

### 1. Volume/Pressure Control Prototype
Located in `ventilator_volume_pressure/`, this implementation focuses on precise volume and pressure control using:
- Flow sensor (SFM3000) for volume measurement
- Honeywell pressure sensor for pressure monitoring
- Stepper motor control for volume delivery
- Real-time pressure and volume monitoring

### 2. Motor Control Prototype
Located in `ventilator_motor_control/`, this implementation emphasizes precise motor control for:
- Tidal volume delivery using stepper motors
- Position-based volume control
- TMCL motor driver integration
- State machine implementation for breathing cycles

### 3. CPAP Prototype
Located in `ventilator_cpap/`, this implementation provides:
- Continuous positive airway pressure control
- Dual valve control system
- Real-time pressure monitoring
- Simple pressure-based control loop

## Hardware Requirements

### Microcontroller
- STM32F4 series (NUCLEO-F439ZI recommended)

### Sensors
- SFM3000 flow sensor (I2C interface)
- Honeywell pressure sensor (I2C interface)
- Limit switches for motor positioning

### Actuators
- Stepper motor with driver (TMCL 1260 supported)
- Control valves (for CPAP prototype)
- GPIO-controlled indicators

### Communication
- USB-to-Serial adapter for GUI communication
- I2C bus for sensor communication
- Motor control interfaces (as needed)

## Software Requirements

### Development Environment
- STM32CubeIDE (or equivalent)
- Python 3.x for GUI application
- ARM GCC toolchain

### Python Dependencies
- numpy
- matplotlib
- tkinter
- pyserial

### STM32 Libraries
- STM32 HAL
- Custom BSP layer (provided)
- Sensor drivers (provided)
