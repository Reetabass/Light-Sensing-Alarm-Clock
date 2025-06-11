# Light-Sensing Alarm Clock  
Embedded Systems Final Project — IES 40434

---

## Overview

This project implements a fully embedded **Light-Sensing Alarm Clock** on the ATmega328P microcontroller. It uses multiple AVR peripherals to create a real-time alarm system that triggers based on environmental light conditions, with gesture-based dismissal using an ultrasonic sensor.

All modules are written in low-level AVR C, fully interrupt-driven, and do not use any Arduino libraries.

---

## Features

- Light-based alarm activation using ADC.
- Gesture-based alarm dismissal using ultrasonic sonar (AC + Input Capture).
- Multiple operational modes: **Day**, **Night**, **Off**.
- Mode switching and calibration via external interrupts.
- Buzzer output using PWM.
- Temperature monitoring via ADC.
- Serial debugging using USART.
- Multiple timers integrated for PWM, ADC timing, sonar measurements, and system clocks.
- Full integration of required embedded modules.

---

## Hardware Components

- ATmega328P Microcontroller (16MHz external crystal)
- Light Sensor (photoresistor or photodiode circuit)
- Ultrasonic Sensor (trigger/echo)
- Buzzer
- Push Buttons (mode control & calibration)
- Temperature Sensor (optional)
- Serial-to-USB adapter (for USART debug output)
- Various resistors, capacitors, wiring

---

## Pin Mapping

| Peripheral       | AVR Pin |
|-------------------|---------|
| Light Sensor (ADC)| PC0 (ADC0) |
| Temperature Sensor| PC1 (ADC1) |
| Sonar Trigger     | PD4 |
| Sonar Echo (IC)   | PD6 (ICP1) |
| Buzzer (PWM)      | PD5 (OC0B) |
| Mode Button       | PD2 (INT0) |
| Logic Flip Button | PD7 |
| Calibrate Button  | PB4 |
| USART TX/RX       | PD0 / PD1 |

---

## Software Modules

### ADC (Analog-to-Digital Converter)
- Reads light and temperature sensors.
- Interrupt-driven conversion.

### Analog Comparator (AC)
- Works with Input Capture for sonar echo detection.

### Input Capture (IC)
- Measures sonar echo pulse width for distance calculation.

### USART
- Serial debugging output for monitoring system behavior.

### Timers

- **Timer0**: Fast PWM for buzzer control.
- **Timer1**: Input Capture for sonar timing.
- **Timer2**: CTC for periodic system triggers (ADC sampling, overflow timing).

### External Interrupts
- Mode changes and calibration via INT0 and INT1.

### Sonar Module
- Calculates distance using ultrasonic pulses.
- Allows gesture-based alarm dismissal.

---

## System Logic

- **Day Mode**: Alarm triggers if light drops below threshold.
- **Night Mode**: Alarm triggers if light rises above threshold.
- **Off Mode**: Alarm disabled.
- If alarm is active, user can disable it by waving hand in front of sonar.

---
