# Pololu DC motor position PID control using Arduino
This project implements a closed loop PID control on a Pololu 70:1 metal gearmotor (4754) using an Arduino UNO R4 WiFi and a Pololu Dual G2 High-Power Motor Shield 24v14 (M1). A rotary encoder is used as the user interface: turning the knob adjusts a “dialed” target, and pressing the knob latches that value as the setpoint. The motor’s built-in quadrature encoder supplies feedback so the controller always knows where the output shaft is.

The controller is a position-based PID written in Arduino C++. Loop timing uses micros() to compute Δt. Encoder values are read as follows: 1x on the motor’s A channel rising edge, with direction read from B. The motor encoder is 16 counts/rev at the motor shaft for a single rising edge; through the 70:1 gearbox that becomes 1120 (70x16) counts/rev at the output. A constant ROT_STEP = 1120 makes one knob “step” equal exactly one output-shaft revolution

The PID output is mapped to the Pololu driver’s setM1Speed(-400…+400) command. A configurable PWM_MAX (e.g., 300) limits the drive to a sensible range, and a small deadband (e.g., 19) is implemented. 

For tuning and visualization, the sketch streams setpoint and position (normalized to revolutions) over serial, and MATLAB plots them with a fixed, zero-based time axis. This makes it easy to compare runs, spot overshoot or steady-state error, and adjust gains. 

## Dependencies

This project uses Pololu’s Dual G2 High-Power Motor Shield library.  
Code for the motor driver is maintained here:  
[pololu/dual-g2-high-power-motor-shield](https://github.com/pololu/dual-g2-high-power-motor-shield)



![Tracking demo](assets/DCMotorPWM.png)

#### Video
[https://github.com/arijeetnath12/DC-Motor-Position-PID-control/blob/main/assets/Timeline_.mp4](https://github.com/arijeetnath12/DC-Motor-Position-PID-control/issues/1)


