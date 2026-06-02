# PID-Speed-Control-Robot-KL46Z
Embedded systems project implementing PID-based motor speed control on the FRDM-KL46Z using encoder feedback, TPM PWM, interrupts, and real-time control algorithms in C.

Project Overview

This project implements a closed-loop PID control system for a two-wheel robot using encoder feedback and TPM-generated PWM signals. The robot maintains stable motor speed across different surfaces by continuously measuring wheel rotation and dynamically adjusting PWM duty cycles in real time.

Features

* PID-based closed-loop motor speed control
* Encoder pulse measurement using GPIO interrupts
* TPM-generated PWM motor control
* Real-time timer interrupt handling
* Differential drive robot platform
* Stable speed regulation across multiple surfaces
* Oscilloscope verification of encoder outputs
* Register-level peripheral configuration

Hardware Platform

* FRDM-KL46Z Development Board
* DC Motors with Quadrature Encoders
* TB6612FNG Motor Driver
* TPM0 / TPM2 Modules
* GPIO Interrupt System
* Oscilloscope-based encoder validation

Technologies Used

* Embedded C
* ARM Cortex-M0+
* PID Control Systems
* TPM (Timer/PWM Module)
* GPIO Interrupt Handling
* Encoder Feedback Systems
* MCUXpresso IDE

Control System

The robot measures wheel speed using encoder pulse counts collected during fixed timer intervals. A PID controller calculates the error between the desired speed and measured speed, then adjusts PWM duty cycles to maintain stable motion under changing load conditions.

Learning Outcomes

This project demonstrates practical experience with:

* Real-time embedded control systems
* PID algorithm implementation
* Interrupt-driven programming
* Encoder feedback processing
* PWM motor control
* Hardware/software integration
* Oscilloscope signal analysis


## Video Demonstrations

- [Carpet Surface Test](https://rumble.com/v76ua66-motor-carpet.html)

- [Flat Surface Test](https://rumble.com/v76u9v4-flat-surface.html)
