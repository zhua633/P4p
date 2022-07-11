// FinalCoppela.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#define _USE_MATH_DEFINES

#include <Windows.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string>
#include <iomanip>
#include <cstdio>
#include <math.h>


#include "DataLogging.h"
#include "BackgroundProcesses.h"

#include "extApi.h"
#include "extApiPlatform.h"
#include "extApi.c"
#include "extApiPlatform.c"
using std::cout;

void armTime(float armPercentage, int duration);
void driveTime(float leftPercentage, float rightPercentage, int duration);
void driveDistance(int travelDistance, float percentagePower);
float count_to_mm(float wheelenc);
void armmove(int target);
void lineFollowing();
void action();
void sonar(float distance);
void turning(int angle);

int main()
{
	int clientID = 0;
	simxFinish(-1);                                                     //! Close any previously unfinished business
	clientID = simxStart((simxChar*)"127.0.0.1", 19000, TRUE, TRUE, 5000, 5);  //!< Main connection to COPPELIASIM
	Sleep(1);
	if (clientID != -1)
	{
		Initialise(clientID);
		printf("Finished Initialisation\n");
		//Code Goes Here
		/*
		* 		while (1) {
			float L = readSensor(1); ///left
			float M = readSensor(0); //middle
			float R = readSensor(2); //right
			printf("Left - ");
			cout << L;
			printf("Mid - ");
			cout << M;
			printf("Right - ");
			cout << R;
			delay(1000);
		}
		*/
		//FULL RUN
		//action();
		
		// Testing line follow
		//lineFollowing();
		armmove(90);
		// Testing picking up can
		//armTime(-70, 500);
		//driveTime(70, 70, 5000);
		//armTime(70,500);

		//driveDistance(100, 100);
		//sonar(424);
	}
	else
	{
		printf(" Connection status to COPPELIASIM: FAILED\n");
	};
	datalogClose();
	printf("Finished");
	simxFinish(clientID);
	return clientID;
}

float mm_to_count(float distance) {
	//convert mm to encoder value
	float enc;
	enc = distance * 360 / 53 * 1 / M_PI;
	return enc;
}
float count_to_mm(float wheelenc) {
	//convert envoder value to mm
	float dis;
	dis = wheelenc * M_PI * 53 / 360;
	return dis;
}
void driveDistance(int distance, float percpwr) {
	float u, error = 1000, time, integral, prevErrors = 0, Kp = 1, Ki = 0.1, leftResult, rightResult, pos;
	//set encoder counts to zero.
	setSensor(LeftEnc, 0);
	setSensor(RightEnc, 0);
	//saturate the motor power between 0-100%.
	percpwr = saturate(percpwr, 0, 100);
	//reset timer counts.
	resetTimer(T_1);
	//Read the encoder counts as variabls.
	leftResult = readSensor(LeftEnc);
	rightResult = readSensor(RightEnc);
	pos = count_to_mm(leftResult);
	//repeats until distance reached
	//while distance is positive
	while (pos < distance && pos >= 0) {
		//make sure the two wheels drive the same distance
		error = rightResult - leftResult;
		time = readTimer(T_1);
		//PI controller
		integral = prevErrors + (error * time / 1000);

		u = Kp * error + Ki * integral;

		//refresh encoder count
		leftResult = readSensor(LeftEnc);
		rightResult = readSensor(RightEnc);
		//send power to motor
		motorPower(LeftMotor, percpwr + u);
		motorPower(RightMotor, percpwr - u);
		delay(100);
		pos = count_to_mm(leftResult);
	}
	//while distance is negative
	while (pos > distance && pos <= 0) {
		//makes sure the wheels drives the same distance
		error = rightResult - leftResult;
		time = readTimer(T_1);
		integral = prevErrors + (error * time / 1000);
		u = Kp * error + Ki * integral;

		//refresh encoder count
		leftResult = readSensor(LeftEnc);
		rightResult = readSensor(RightEnc);
		//allocate the motor power

		motorPower(LeftMotor, -(percpwr - u));
		motorPower(RightMotor, -(percpwr + u));
		delay(100);
		pos = count_to_mm(leftResult);
	}

	motorPower(LeftMotor, 0);
	motorPower(RightMotor, 0);
}
void driveTime(float LeftPower, float RightPower, int Duration) {
	float error, u, Kp = 1, pwr;
	// Limit user input between -100 and 100 percent.
	LeftPower = saturate(LeftPower, -100, 100);
	RightPower = saturate(RightPower, -100, 100);

	error = LeftPower - RightPower;
	u = Kp * error;
	pwr = saturate(u, 0, 100 - u);

	// Turn on motor according to input powers.
	motorPower(LeftMotor, LeftPower - pwr);
	motorPower(RightMotor, RightPower + pwr);
	delay(Duration);
	motorPower(LeftMotor, 0);
	motorPower(RightMotor, 0);
}
void armTime(float Power, int duration) {

	// Limit user input between -100 and 100 percent.
	Power = (saturate(Power, -100, 100));

	// Raise the arm according to input power percentage.
	motorPower(ArmMotor, Power);
	delay(duration);
	motorPower(ArmMotor, 0);
}
int armangle(float enc) {
	//converts potentiometer value to angle
	float angle;
	angle = (enc - 3200) / -20;
	return angle;
}
void turning(int angle) {
	//robot turns according to the arc length
	float finalEnc, error, distance, du;
	float  Kp, u;
	//converting angle turned to distance travelled
	distance = angle * 2.186;
	finalEnc = mm_to_count(distance);
	setSensor(RightEnc, 0);
	setSensor(LeftEnc, 0);
	Kp = 0.5;
	//decide on which way the robot turns
	if (angle > 0) {
		u = 50;
	}
	else {
		u = -50;
	}

	do {
		//make sure the 2 wheels turns the same amount
		error = readSensor(RightEnc) + readSensor(LeftEnc);
		du = Kp * error;

		//turn until arclength is reached
		driveTime(u + du, -u - du, 5);
	} while (abs(readSensor(RightEnc)) < abs(finalEnc));
}
void armmove(int target) {

	float angle, error = 0;
	float Kp, u = 0, Ki;
	//flag variable
	int flag = 0;
	//initialise armmove PI controller
	Ki = 0.1;
	Kp = 0.1;
	do {
		//continue moving unti within +- 3 degrees of the angle specified
		angle = armangle(readSensor(EncoderArm));
		cout << angle;
		cout << "\n";
		error = target - angle;
		//PI controller
		u = Kp * error + (Ki * error * abs(u));
		armTime(u, 5);
		if (angle<target + 3 && angle>target - 3) {
			flag = 1;
		}

	} while (flag == 0);
}
void lineFollowing() {
	//set each sensor and set flag as variable
	int flag = 0, brown = 410;
	float leftResult, rightResult, midResult;
	float error, u, Kp = 3, pwr, LeftPower = 30, RightPower = 30;
	//set encorders to 0.
	setSensor(LeftEnc, 0);
	setSensor(RightEnc, 0);

	//set the result variable to light sensors
	leftResult = readSensor(LeftLight);
	midResult = readSensor(MidLight);
	rightResult = readSensor(RightLight);

	while (flag == 0) {
		//when only the left light sensor is detecting brown.
		if (leftResult >= brown && midResult < brown && rightResult < brown) {
			//Turning 1.5 degrees anticlockwise.
			turning(-1.5);
		}

		//when only the right light sensor is detecting brown.
		else if (leftResult < brown && midResult < brown && rightResult >= brown) {
			// Turning 1.5 degrees clockwise.
			turning(1.5);
		}

		//when only the middle light sensor is detecting brown.
		else if (leftResult < brown && midResult >= brown && rightResult < brown) {
			error = LeftPower - RightPower;
			u = Kp * error;
			pwr = saturate(u, 0, 100 - u);

			// Turn on motor according to input powers.
			motorPower(LeftMotor, LeftPower - pwr);
			motorPower(RightMotor, RightPower + pwr);
			delay(100);
			motorPower(LeftMotor, 0);
			motorPower(RightMotor, 0);
		}


		//when left and middle light sensors are detecting brown.
		else if (leftResult >= brown && midResult >= brown && rightResult < brown) {
			// Turning 1 degree anticlockwise.
			turning(-1);
		}

		// when middle and right sensors are detecting brown
		else if (leftResult < brown && midResult >= brown && rightResult >= brown) {
			// Turning 1 degree clockwise.
			turning(1);
		}

		// when only left and right sensors are detecting brown
		else if (leftResult >= brown && midResult < brown && rightResult >= brown) {
			// Turning 1 degree anticlockwise.
			turning(-1);
		}

		// When all the sensors are detecting brown.
		else if (1250 > leftResult && leftResult >= brown && 1250 > midResult && midResult >= brown && 1250 > rightResult && rightResult >= brown) {
			error = LeftPower - RightPower;
			u = Kp * error;
			pwr = saturate(u, 0, 100 - u);

			// Turn on motor according to input powers.
			motorPower(LeftMotor, LeftPower - pwr);
			motorPower(RightMotor, RightPower + pwr);
			delay(100);
			motorPower(LeftMotor, 0);
			motorPower(RightMotor, 0);
		}

		//when none of the sensors are detecting brown, the robot will stop.
		else if (leftResult < brown && midResult < brown && rightResult < brown) {
			flag = 1;
		}

		//when all three sensors are detecting black.
		else if (1250 < leftResult && 1250 < midResult && 1250 < rightResult) {
			// reached the black line and stop.
			flag = 1;
		}

		// When the middle is very dark, but the left and right sensors are lighter, the robot is on a 90 degree turn.
		else if (brown < leftResult && 1250 < midResult && brown < rightResult) {
			// reached the black line and stop.
			turning(10);
		}

		leftResult = readSensor(LeftLight);
		midResult = readSensor(MidLight);
		rightResult = readSensor(RightLight);
	}
	driveDistance(30, 50);
}
void sonar(float distance) {
	//set a variable for each sensors, and components of the PI controller
	float leftResult, rightResult, error, time, Kp = 1.8, Ki = 0.2, u, integral, prevErrors = 0;

	//reset timer counts.
	resetTimer(T_1);

	//Set encoder counts to zero.
	setSensor(LeftEnc, 0);
	setSensor(RightEnc, 0);

	while (distance >= 0) {
		cout << readSensor(sonarSensor);
		cout << "\n";
		if (readSensor(sonarSensor) > 0) {
			if (readSensor(sonarSensor) <= distance) {
				//If something is detected within the threshold range.
				break;
			}

			//If nothing is detected within the threshold range.
			//Read the encoder counts as variables.
			leftResult = readSensor(LeftEnc);
			rightResult = readSensor(RightEnc);

			//calculate teh difference between the left and right encoders
			error = rightResult - leftResult;
			//read timer.
			time = readTimer(T_1);
			//Calculate the integral component of the PI Controller.
			integral = prevErrors + (error * time / 1000);
			//Calculate the error.
			u = Kp * error + Ki * integral;

			// Store the integral value in the prevErrors variable.
			prevErrors = integral;
			//Read the encoders again.
			leftResult = readSensor(LeftEnc);
			rightResult = readSensor(RightEnc);

			//Set the adjusted powers for left and right wheel.
			motorPower(LeftMotor, 70 + u);
			motorPower(RightMotor, 70 - u);
			// delay for one tenth of a second.
			delay(1);
		}
	}
	motorPower(LeftMotor, 0);
	motorPower(RightMotor, 0);
}
void action() {
	// Move arm into a vertical position.
	//armmove(90);
	armTime(70, 500);
	// Wait for 1 second.
	//delay(1000);
	// Using sonar to drive forward until an object within 424 mm is detected.
	sonar(424);
	// Lower the arm into horizontal position.
	//armmove(0);
	cout << "arm time";
	armTime(-70, 500);
	// Wait for 1 second.
	//delay(1000);
	// Drive forward for 30mm.
	cout << "drive distance";
	driveDistance(30, 50);
	// Raise the arm up 70 degrees.
	//armmove(70);
	cout << "arm time";
	armTime(-70, 500);
	// Drive backwards for 420 mm.
	driveDistance(-420, 70);
	// Turn clockwise.
	turning(80);
	// Drive forward for 320 mm.
	driveDistance(320, 70);
	// Wait for 1 second.
	//delay(1000);
	// Start following the line.
	lineFollowing();
	// Turn anticlockwise for 1.5 degrees to aim for the finishing target.
	//turning(-1.5);


	// Using sonar to drive forward until an object within 280mm is detected.
	sonar(280);
	// Wait for one second
	//delay(1000);

	turning(-10);
	// Lower the arm into a horizontal position.
	//armmove(0);
	armTime(70, 500);
	// Wait for one second.
	//delay(1000);
	turning(10);


	// Drive backwards for 400 mm.
	driveDistance(-400, 70);
	// Turn clockwise.
	turning(75);
	// Drive forward for 1300mm.
	driveDistance(1300, 70);
	// Turn anticlockwise.
	turning(-75);
	// drive into the finishing zone.
	driveDistance(550, 70);
}


