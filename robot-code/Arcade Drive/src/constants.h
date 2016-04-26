#pragma once

#ifndef SRC_CONSTANTS_H_
#define SRC_CONSTANTS_H_

//#define PRACTICE_ROBOT 1

#if PRACTICE_ROBOT
#define PRESSURE_SWITCH_INPUT	14
#define COMPRESSOR_RELAY		1
#else
const int COMPRESSOR = 0;
#endif

//joystick buttons
const int HI_SHIFT_BUTTON=1;
const int SHOOTER_BUTTON=3;
const int PREP_SHOOT_BUTTON = 4;
const int PICKUP_BALL = 5;
const int LIFT_SCALE_ARM = 6;
const int LOWER_SCALE_ARM = 7;
const int CLIMBER_BUTTON=8;
const int LOW_GOAL=9;


// opstick
const int REV_UP_SHOOTER_BUTTON=2;
const int CAMERA_BUTTON=4;
const int ROLLER_IN_BUTTON=6;
const int ROLLER_OUT_BUTTON=7;
const int ARM_DOWN_BUTTON=8;
const int RECENTER_BUTTON = 9;
const int SHOOTER_REV_BUTTON=10; //reverse
const int SHOOTER_FWD_BUTTON=11; //forward


#if PRACTICE_ROBOT
//limits turret
const float MAX_TURRET_POS=2.8;
const float MIN_TURRET_POS=1.7;
const float TURRET_V_PER_DEGREES=0.0387;
const float TURRET_TOLERANCE = 0.05;
const float TURRET_TURN_SPEED = 0.5;
const float SHOOTER_SPEED =0.95;

//limits camera
const float CAMERA_FOV = 53.079;
const float CAMERA_FOV_OFFSET = 0.8;
const float CORRECT_PIX_POS = 69;
const float MAX_PIX_POS = 71;
const float MIN_PIX_POS=65;

//limits pickup
const float MAX_PICKUP_POS=4.7;
const float MIN_PICKUP_POS=0.9;
const float MAX_PICKUP_POWER = 1;
const float MIN_PICKUP_POWER = 0.1;
const float PREP_SHOOT_PICKUP_POS = 4.2;

#else
//limits turret
//const float MAX_TURRET_POS=2.5; old range
//const float MIN_TURRET_POS=1.3; old range
const float MAX_TURRET_POS=2.58;
const float MIN_TURRET_POS=1.25;
const float TURRET_V_PER_DEGREES=0.0387;
const float TURRET_TOLERANCE = 0.05;
const float TURRET_TURN_SPEED = 0.5;
const float SHOOTER_SPEED =0.90;
//limits camera
const float CAMERA_FOV = 53.079;
const float CAMERA_FOV_OFFSET = 0.8;
const float CORRECT_PIX_POS = 78;
const float MAX_PIX_POS = CORRECT_PIX_POS+3;
const float MIN_PIX_POS=CORRECT_PIX_POS-3;
//limits pickup
const float MAX_PICKUP_POS=4.4;
const float MIN_PICKUP_POS=0.7;
const float MAX_PICKUP_POWER = 1;
const float MIN_PICKUP_POWER = 0.1;
const float PREP_SHOOT_PICKUP_POS = 3.37;
const float UNJAM_PICKUP_POS = 4.3;

#endif

//PWMs
#if PRACTICE_ROBOT
const int DRIVE_RIGHT_PWM = 1; //Right Side Drive Motor
const int DRIVE_LEFT_PWM = 2; //Left Side Drive Motor
const int PICKUP_ROLLER_PWM=3; //Pickup Roller Motor
const int TURRET_PWM=4; //Turret Rotation Motor
const int SHOOTER_PWM=5; //Shooter Motor
const int PICKUP_PWM=6;  //Pickup up/down Motor
const int CLIMBER_PWM=7; //Climber Motor
#else
const int DRIVE_RIGHT_PWM = 0; //Right Side Drive Motor
const int DRIVE_LEFT_PWM = 1; //Left Side Drive Motor
const int PICKUP_ROLLER_PWM=2; //Pickup Roller Motor
const int TURRET_PWM=3; //Turret Rotation Motor
const int SHOOTER_PWM=4; //Shooter Motor
const int PICKUP_PWM=5;  //Pickup up/down Motor
const int CLIMBER_PWM=6; //Climber Motor
#endif

#if PRACTICE_ROBOT
//DIO
const int LEFT_ENCODER_A_DIO = 1;
const int LEFT_ENCODER_B_DIO = 2;
const int RIGHT_ENCODER_A_DIO = 3;
const int RIGHT_ENCODER_B_DIO = 4;
#else
//DIO
const int LEFT_ENCODER_A_DIO = 0;
const int LEFT_ENCODER_B_DIO = 1;
const int RIGHT_ENCODER_A_DIO = 2;
const int RIGHT_ENCODER_B_DIO = 3;
#endif

//joystick ports
#if PRACTICE_ROBOT
const int DRIVER_JOYSTICK = 1;
const int OP_JOYSTICK = 2;
#else
const int DRIVER_JOYSTICK = 0;
const int OP_JOYSTICK = 1;
#endif

#if PRACTICE_ROBOT
const float fEncoderTicksPerFoot = -1936.f;
#else
const float fEncoderTicksPerFoot = -1936.f;
#endif

//solenoids
#if PRACTICE_ROBOT
const int HIGH_SHIFT_SOLENOID = 2;
const int LOW_SHIFT_SOLENOID = 4;
const int LIFT_SCALE_ARMS_SOLENOID = 3;
const int LOWER_SCALE_ARMS_SOLENOID = 1;
#else
const int HIGH_SHIFT_SOLENOID = 1;
const int LOW_SHIFT_SOLENOID = 3;
const int LIFT_SCALE_ARMS_SOLENOID = 2;
const int LOWER_SCALE_ARMS_SOLENOID = 0;
#endif

// analog channels
#if PRACTICE_ROBOT
const int GYRO_ANALOG_CHANNEL = 2;
const int PICKUP_POT = 3;

const int TURRET_POT=1;
#else
const int GYRO_ANALOG_CHANNEL = 1;
const int PICKUP_POT = 2;

const int TURRET_POT=0;
#endif
//const int TURRET_CONTROLLER=3;


//const int PICKUP_CONTROLLER=5;



#endif /* SRC_CONSTANTS_H_ */
