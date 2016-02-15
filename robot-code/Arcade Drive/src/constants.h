#pragma once

#ifndef SRC_CONSTANTS_H_
#define SRC_CONSTANTS_H_

// DRIVER JOYSTICK BUTTONS
const int SHOOTER_BUTTON=1;

//PWMs
const int DRIVE_RIGHT_PWM = 0;
const int DRIVE_LEFT_PWM = 1;
const int TURRET=2;
const int SHOOTER=3;

//DIO
const int LEFT_ENCODER_A_DIO = 0;
const int LEFT_ENCODER_B_DIO = 1;
const int RIGHT_ENCODER_A_DIO = 2;
const int RIGHT_ENCODER_B_DIO = 3;

//joystick ports
const int DRIVER_JOYSTICK = 0;
const int OP_JOYSTICK = 1;

//solenoids
const int COMPRESSOR = 0;
const int HIGH_SHIFT_SOLENOID = 0;
const int LOW_SHIFT_SOLENOID = 1;

// analog channels
const int GYRO_ANALOG_CHANNEL = 1;


#endif /* SRC_CONSTANTS_H_ */
