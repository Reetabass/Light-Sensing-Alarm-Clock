Light-Sensing Alarm Clock
Final Embedded Systems Project (IES 40434)

Overview
This project implements a Light-Sensing Alarm Clock built entirely using AVR C and ATmega328P microcontroller peripherals. The system operates in multiple modes (Day/Night/Off), triggers alarms based on light intensity, and allows the user to disable alarms using hand gestures via a sonar distance sensor. It integrates multiple hardware modules and peripherals to demonstrate a fully embedded real-time system.

Features
Light-based alarm triggering using ADC light sensor readings

Temperature monitoring with ADC (optional)

Gesture-based alarm dismissal using ultrasonic distance measurement via Input Capture and Analog Comparator

Multiple operational modes (Day, Night, Off) toggled with external interrupts

USART serial communication for debugging and monitoring

Buzzer alarm controlled via PWM

Timers used for PWM, ADC triggering, and overflow management

Fully interrupt-driven design for responsive real-time behavior

Hardware Components
ATmega328P Microcontroller

Light Sensor (e.g. photoresistor or photodiode + resistor divider)

Ultrasonic Sensor (custom trigger/echo pins)

Buzzer (PWM output)

Buttons (for mode switching and calibration)

Temperature Sensor (optional)

Serial-to-USB adapter for USART monitoring

16 MHz Crystal Oscillator

Pull-up resistors and passive components

Pin Configuration
Peripheral	AVR Pin
Light Sensor (ADC)	ADC0 (PC0)
Temperature Sensor	ADC1 (PC1)
Sonar Trigger	PD4
Sonar Echo (IC)	PD6 (ICP1)
Buzzer (PWM)	PD5 (OC0B or OC0A)
Mode Button	PD2 (INT0)
Logic Flip Button	PD7
Calibrate Button	PB4
USART TX/RX	PD0 / PD1

Software Modules
1. Analog-to-Digital Converter (ADC)
Used for light and temperature measurements.

Interrupt-driven conversion completion (ADC_vect).

2. Analog Comparator (AC)
Used for accurate sonar echo detection in conjunction with Input Capture.

3. Input Capture (IC)
Captures sonar echo pulse widths on ICP1 (PD6).

Calculates distance using Timer1.

4. USART Communication
Sends debug info: mode status, light levels, temperature, sonar distance, and alarm status.

5. Timers/Counters
Timer0: Fast PWM mode for buzzer sound.

Timer1: Input Capture for sonar echo timing.

Timer2: CTC mode for periodic ADC triggering and system timekeeping.

6. External Interrupts
INT0 (PD2) used for mode switching.

INT1 (PD3) used for manual override or calibration.

7. Sonar Distance Measurement
Triggered manually via sonar module.

Calculates object distance to determine hand wave for alarm dismissal.

System Logic
Day Mode: Alarm triggers if light level drops below threshold.

Night Mode: Alarm triggers if light level exceeds threshold.

Off Mode: Alarm disabled.

Once alarm is active, waving hand in front of the sonar sensor disables it.

