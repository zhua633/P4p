#include <Windows.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string>
#include <iomanip>
#include <cstdio>
#include <ctime>

#include "extApi.h"


#define PI 3.14

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

int clientID = 0;
int _leftmotor = 0;
int _rightmotor = 0;
int _Sonar_Sensor = 0;
int _Left_Sensor = 0;
int _Mid_Sensor = 0;
int _Right_Sensor = 0;
int _Vision_Sensor = 0;
int _arm_motor = 0;
int _Dummy_Pivot = 0;
int LeftPower = 0;
int rightPower = 0;
int armPower = 0;
float SetLeft = 0;
float SetRight = 0;
float SetArm = 0;
int T1 = 0;
int T2 = 0;
int T3 = 0;
int T4 = 0;

void InitialliseSensors(simxInt ClientId, simxInt LightSensHandles[], simxInt SonarSensHandle) {
    float* auxValues = NULL;
    int* auxValuesCount = 0;
    float SensorVals[3] = { 0 };
    int reading_flag_light = 0;
    int reading_flag_sonar = 0;
    for (int i = 0; i < 3; i++) { //First call of sensors
        simxReadVisionSensor(ClientId, LightSensHandles[i], NULL, &auxValues, &auxValuesCount, simx_opmode_streaming);
    }
    while (reading_flag_light != 1) { //Continue calling the sensors until an ok reading
        for (int i = 0; i < 3; i++) {
            if (simxReadVisionSensor(ClientId, LightSensHandles[i], NULL, &auxValues, &auxValuesCount, simx_opmode_buffer) == simx_return_ok) {
                reading_flag_light = 1; //Sensors have started reading
            }
        }
    }

    //First call of sensor
    simxReadProximitySensor(ClientId, SonarSensHandle, NULL, SensorVals, NULL, NULL, simx_opmode_streaming);

    while (reading_flag_sonar != 1) { //Continue calling the sensor until an ok reading
        if (simxReadProximitySensor(ClientId, SonarSensHandle, NULL, SensorVals, NULL, NULL, simx_opmode_buffer) == simx_return_ok) {
            reading_flag_sonar = 1; //Sensors have started reading
        }

    }
}

void Initialise(simxInt ClientID) { //Intialisation of the remote api 
    printf(" Connection status to VREP: SUCCESS\n");
    simxInt syncho = simxSynchronous(ClientID, 1);
    int start = simxStartSimulation(ClientID, simx_opmode_oneshot_wait);
    int TEST1 = simxGetObjectHandle(ClientID, "Motor1", &_leftmotor, simx_opmode_blocking);
    int TEST2 = simxGetObjectHandle(ClientID, "Motor2", &_rightmotor, simx_opmode_blocking);
    int TEST3 = simxGetObjectHandle(ClientID, "Sonar_Sensor", &_Sonar_Sensor, simx_opmode_blocking);
    int TEST4 = simxGetObjectHandle(ClientID, "Left_Sensor", &_Left_Sensor, simx_opmode_blocking);
    int TEST5 = simxGetObjectHandle(ClientID, "Mid_Sensor", &_Mid_Sensor, simx_opmode_blocking);
    int TEST6 = simxGetObjectHandle(ClientID, "Right_Sensor", &_Right_Sensor, simx_opmode_blocking);
    int TEST7 = simxGetObjectHandle(ClientID, "ArmMotor", &_arm_motor, simx_opmode_blocking);
    int TEST9 = simxGetObjectHandle(ClientID, "Dummy_Pivot", &_Dummy_Pivot, simx_opmode_blocking);

    if (TRUE)
    {
        printf("Computed object handle: %d  %d\n",TEST1,_leftmotor);
        printf("Computed object handle: %d  %d\n", TEST1, _rightmotor);
        printf("Computed object handle: %d  %d\n", TEST1, _Sonar_Sensor);
        printf("Computed object handle: %d  %d\n", TEST1, _Left_Sensor);
        printf("Computed object handle: %d  %d\n", TEST1, _Mid_Sensor);
        printf("Computed object handle: %d  %d\n", TEST1, _Right_Sensor);
        printf("Computed object handle: %d  %d\n", TEST1, _arm_motor);
        printf("Computed object handle: %d  %d\n", TEST1, _Dummy_Pivot);
    }

    int Lightsensors[3] = { _Left_Sensor, _Mid_Sensor, _Right_Sensor };
    simxSetJointTargetVelocity(ClientID, _leftmotor, 0.0, simx_opmode_oneshot_wait);
    simxSetJointTargetVelocity(ClientID, _rightmotor, 0.0, simx_opmode_oneshot_wait);
    simxSetJointTargetVelocity(ClientID, _arm_motor, 0.0, simx_opmode_oneshot_wait);
    InitialliseSensors(ClientID, Lightsensors, _Sonar_Sensor);

    clientID = ClientID;
	
}

float saturate(float input, float lower, float upper) {
    if (input > upper) {
        return upper;
    }
    else if (input < lower) {
        return lower;
    }
    return input;
}

void motorPower(int motorSelect, int power) {
    int outPower = (int)saturate(power, -127, 127);
    float velocity;
    if (power > 15) { //Conversion from motor power input to rad/s 
        velocity = 0.02 * power;// -1.5;
    }
    else if (power < -15) {
        velocity = 0.02 * power;// +1.5;
    }
    else {
        velocity = 0;
    }
    float CurrAngle = 0;
    switch (motorSelect) {
    case 0:
        LeftPower = power;
        simxSetJointTargetVelocity(clientID, _leftmotor, velocity, simx_opmode_oneshot_wait);
        break;
    case 1:
        rightPower = power;
        simxSetJointTargetVelocity(clientID, _rightmotor, velocity, simx_opmode_oneshot_wait);
        break;
    case 2:
        simxGetJointPosition(clientID, _arm_motor, &CurrAngle, simx_opmode_oneshot_wait);
        if ((CurrAngle == 5 * (PI / 180))&&(power>0)) {
            simxSetJointTargetVelocity(clientID, _arm_motor, 0, simx_opmode_oneshot_wait);
        }
        else if ((CurrAngle == -1 * (PI / 180))&&(power<0)) {
            simxSetJointTargetVelocity(clientID, _arm_motor, 0, simx_opmode_oneshot_wait);
        }
        else {
            armPower = power;
            simxSetJointTargetVelocity(clientID, _arm_motor, velocity, simx_opmode_oneshot_wait);
        }
        break;
    default:
        printf("Unrecognised Motor Selected\n");
        break;
    }
}

int getMotorPower(int motorSelect) { //No equivalent in Coppelia
    int PowerValue = 0; 
    switch (motorSelect) {
    case 0:
        PowerValue = LeftPower;
        break;
    case 1:
        PowerValue = rightPower;
        break;
    case 2:
        PowerValue =  armPower;
        break;
    default:
        printf("Unrecognised Motor Selected\n");
        break;
    }
    return PowerValue;
}


float readSensor(int sensorSelect) { //Conversions
    float result = 0;
    float* auxValues = NULL;
    int* auxValuesCount = 0;
    float Angle = 0;
    float SensorVals[3] = { 0 };
    switch (sensorSelect) {
    case 0:
        simxReadVisionSensor(clientID, _Mid_Sensor, NULL, &auxValues, &auxValuesCount, simx_opmode_buffer);
        result = (int)((1 - auxValues[10])*1857 + 152); //data[10] is the average intensity, Then the conversion
        break;
    case 1:
        simxReadVisionSensor(clientID, _Left_Sensor, NULL, &auxValues, &auxValuesCount, simx_opmode_buffer);
        result = (int)((1 - auxValues[10])*1857 + 152); //data[10] is the average intensity
        break;
    case 2:
        simxReadVisionSensor(clientID, _Right_Sensor, NULL, &auxValues, &auxValuesCount, simx_opmode_buffer);
        result = (int)((1 - auxValues[10])*1857 + 152); //data[10] is the average intensity
		break;
    case 5:
        simxGetJointPosition(clientID, _rightmotor, &Angle, simx_opmode_oneshot_wait);
        result = (int)(Angle - SetRight) * (int)(450 / PI); //900 counts per revolution 
        break;
    case 6:
        simxGetJointPosition(clientID, _leftmotor, &Angle, simx_opmode_oneshot_wait);
        result = (int)(Angle - SetLeft) * (int)(450 / PI); //900 counts per revolution 
        break;
    case 9:
        simxGetJointPosition(clientID, _arm_motor, &Angle, simx_opmode_oneshot_wait);
        if ((SetArm < 0) && (Angle > SetArm)) {
            result = (float)((abs(SetArm) - Angle) * (180.0 / PI) * 900 * 7 / 360);
        }
        else {
            result = (float)((Angle - SetArm) * (180.0 / PI) * 900 * 7 / 360);
        } 
        break;
    case 10:
        simxReadProximitySensor(clientID, _Sonar_Sensor, NULL, SensorVals, NULL, NULL, simx_opmode_buffer);
        result = (float)SensorVals[2]*1000; // Convert from m to mm
        break;
    default:
        printf("Unrecognised Sensor Selected\n");
        break;
    }
    return result;
}

void setSensor(int sensorSelect, int sValue) {
    float Angle = 0;
    switch (sensorSelect) {
    case 5:
        simxGetJointPosition(clientID, _rightmotor, &Angle, simx_opmode_oneshot_wait);
        SetRight = Angle - sValue;
        break;
    case 6:
        simxGetJointPosition(clientID, _leftmotor, &Angle, simx_opmode_oneshot_wait);
        SetLeft = Angle - sValue;
        break;
    case 9:
        simxGetJointPosition(clientID, _arm_motor, &Angle, simx_opmode_oneshot_wait);
        printf("SetArm: %f", Angle);
        SetArm = Angle - sValue;
        break;
    default:
        printf("Unrecognised Sensor Selected\n");
        break;
    }
}

void resetTimer(int timerName) { //Stores the time of reset into the global variable associated with the iputted timer
    switch (timerName) {
    case 0:
        T1 = clock();
        break;
    case 1:
        T2 = clock();
        break;
    case 2:
        T3 = clock();
        break;
    case 3:
        T4 = clock();
        break;
    default:
        printf("Unrecognised Timer Selected\n");
        break;
    }
   
}

int readTimer(int timerName) { //returns the value of the selected timer !May need to convert the time as it is the processor clock
    int timereturn = 0;
    switch (timerName) {
    case 0:
        timereturn = clock() - T1; //Need to minus the reset time
        break;
    case 1:
        timereturn = clock() - T2;
        break;
    case 2:
        timereturn = clock() - T3;
        break;
    case 3:
        timereturn = clock() - T4;
        break;
    default:
        printf("Unrecognised Timer Selected\n");
        break;
    }
    return timereturn;
}

void armUp(float percentPower) {
    float velocity;
    int power = abs(round(percentPower / 100 * 127));
    if (power > 127) {
        power = 127;
    }
    if (power > 15) { //Conversion from motor power input to rad/s 
        velocity = 0.02 * power;// -1.5;
    }
    else if (power < -15) {
        velocity = 0.02 * power;// +1.5;
    }
    else {
        velocity = 0;
    }
    simxSetJointTargetVelocity(clientID, _arm_motor, velocity, simx_opmode_oneshot_wait);
    float CurrAngle;
    simxGetJointPosition(clientID, _arm_motor, &CurrAngle, simx_opmode_oneshot_wait);
    while (CurrAngle < 50*(3.14/180)) { 
        simxGetJointPosition(clientID, _arm_motor, &CurrAngle, simx_opmode_oneshot_wait);
    }
	if (CurrAngle >= 50*(3.14 / 180)) {
		printf("Upper Limit Switch Hit! (armUp)\n");
	}
    simxSetJointTargetVelocity(clientID, _arm_motor, 0, simx_opmode_oneshot_wait);
    Sleep(1000);
}


void armDown(float percentPower) {
    float velocity;
    int power = abs(round(percentPower / 100 * 127)); 
    if (power > 127) {
        power = 127;
    }
    if (power > 15) { //Conversion from motor power input to rad/s 
        velocity = 0.2 * power;// -1.5;
    }
    else if (power < -15) {
        velocity = 0.2 * power;// -1.5;
    }
    else {
        velocity = 0;
    }
    simxSetJointTargetVelocity(clientID, _arm_motor, -velocity, simx_opmode_oneshot_wait);
    float CurrAngle;
    simxGetJointPosition(clientID, _arm_motor, &CurrAngle, simx_opmode_oneshot_wait);
    while (CurrAngle > -10 * (3.14 / 180)) { //May need to change the angle to match the real robot
        simxGetJointPosition(clientID, _arm_motor, &CurrAngle, simx_opmode_oneshot_wait);
    }
	if (CurrAngle <= 50 * (3.14 / 180)) {
		printf("Lower Limit Switch Hit! (armDown)\n");
	}
    simxSetJointTargetVelocity(clientID, _arm_motor, 0, simx_opmode_oneshot_wait);
    Sleep(1000);
}

void delay(int time) {
    Sleep(time);
}