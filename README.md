# Gear Vibration and Fault Detection using STM32, MPU6050, and NanoEdge AI

## Overview
This project implements a real-time fault detection system for a gear connected to a DC motor using STM32 and NanoEdge AI. By analyzing the vibration data from the MPU6050 IMU, the system can detect issues such as external contact, internal contact, and balance problems. The detected status is displayed on an SSD1306 OLED screen, and the data is transferred to a PC via a USB virtual COM port.

## Features
- **Fault Detection**: Detects three types of issues—external contact, internal contact, and balance problems.
- **Machine Learning Integration**: Uses NanoEdge AI to process vibration patterns and classify the system's state (faulty, stopped, or running normally).
- **Real-time Display**: Displays the system's status on an SSD1306 OLED screen.
- **Data Transfer**: Provides real-time data transfer via USB virtual COM to a PC for further analysis.
  
  ![Sample](https://github.com/Emrecanbl/STM32-NonoEdge-AI-Fault-Classification-/blob/main/file.jpg?raw=true)
  
## Hardware Components
- **STM32**: Main controller for data acquisition, processing, and communication.
- **MPU6050 IMU**: Used to measure vibration and acceleration data from the motor and gear assembly.
- **SSD1306 OLED Display**: Displays the current system state (e.g., "Normal Operation", "Balance Problem", "External Contact").
- **USB Virtual COM Port**: Transfers the processed data to a PC for logging or further analysis.

## Software Components
- **CubeIDE**: Used for firmware development.
- **NanoEdge AI Studio**: Machine learning model trained using vibration patterns to classify system states.
- **I2C Communication**: Interface used for communication between STM32 and both the MPU6050 and SSD1306.

![Sample](https://github.com/Emrecanbl/STM32-NonoEdge-AI-Fault-Classification-/blob/main/Bencmark.png?raw=true)

## How It Works
1. **Data Acquisition**: Vibration data is continuously captured from the MPU6050 IMU.
2. **Feature Extraction**: The vibration data is processed using a pre-trained NanoEdge AI model to detect anomalies or patterns that indicate faults.
3. **Fault Classification**: The system identifies whether the motor and gear system is running normally, has stopped, or has an issue (e.g., external contact, balance problem).
4. **User Interface**: The detected status is shown on the SSD1306 OLED display.
5. **Data Logging**: The system can transfer the fault data over USB to a connected PC for further diagnostics.

![Sample](https://github.com/Emrecanbl/STM32-NonoEdge-AI-Fault-Classification-/blob/main/94vgid.gif?raw=true)

## Getting Started
### Prerequisites
- STM32CubeIDE
- NanoEdge AI Studio
- STM32 Nucleo Board (or compatible STM32 microcontroller)
- MPU6050 IMU Sensor
- SSD1306 OLED Display
Run the project and observe the fault detection on the OLED screen.

## Usage
- Normal Operation: The OLED displays "Normal Operation".
- Fault Detected: The OLED shows specific messages like "External Contact" or "Balance Problem".
- Data Transfer: Use a serial terminal to view real-time data from the virtual COM port.

## Future Enhancements
- Add support for wireless data transmission via BLE.
- Expand fault detection capabilities with additional sensors.
- Optimize the NanoEdge AI model for even faster detection.
