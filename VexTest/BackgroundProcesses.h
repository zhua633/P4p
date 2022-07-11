#pragma once

#include <Windows.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string> 
#include <iomanip>

#include "extApi.h"


//macros for SENSORS
#define MidLight	 	0
#define LeftLight 		1
#define RightLight	 	2
#define RightEnc 		5
#define LeftEnc 		6
#define LowArmLimit 	7
#define HighArmLimit 	8
#define EncoderArm		9
#define sonarSensor		10
#define rightLED		11
#define leftLED			12

//macros for MOTORS
#define LeftMotor 		0
#define RightMotor 		1
#define ArmMotor 		2

//macros Timers
#define T_1 0 //T1
#define T_2 1 //T2
#define T_3 2 //T3
#define T_4 3 //T4

void InitialliseSensors(simxInt ClientId, simxInt LightSensHandles[], simxInt SonarSensHandle);
void Initialise(simxInt ClientId);
void motorPower(int motorSelect, int power);
int getMotorPower(int motorSelect);
float readSensor(int sensorSelect);
void setSensor(int sensorSelect, int sValue);
void resetTimer(int timerName);
int readTimer(int timerName);
float saturate(float input, float lower, float upper);
void armUp(float percentPower);
void armDown(float percentPower);
void delay(int time);

