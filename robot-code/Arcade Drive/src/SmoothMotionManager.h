/*
 * SmoothMotionManager.h
 *
 *  Created on: Oct 6, 2016
 *      Author: rebotics-user
 */

#ifndef SRC_SMOOTHMOTIONMANAGER_H_
#define SRC_SMOOTHMOTIONMANAGER_H_

#include <deque>
#include <stdio.h>
#include "WPILib.h"

using namespace std;
float blendNum(float x, float y, float blendToY, float max, float min);

struct timeDistance
{
  timeDistance(float time, float distance) : time(time), distance(distance) {};
  float time;
  float distance;
};

float getSpeedOfPoints(const deque<timeDistance>& data);

class SmoothMotionManager
{
	float lastPos;
	float lastTime;
	float lastSpeed;
	float initTime;
	deque<timeDistance> recordedData;

public:
	SmoothMotionManager(float initalSensorReading, float initalTimeSeconds);

	// MechModel: a quick class so that callers to tick() can represent a speed->motorpower "model".
	// input: the speed you want to achieve
	// output: the motor power that achieves that speed in steady-state driving.
	// tip: use the SMoothMotionCalibrationSensors class and the SmoothMotionManager::Calibrate() function to help you build this model
	class MechModel
	{
		public:
			virtual float GetMotorPowerForSpeed(float targetSpeed) = 0;
			//returns how many seconds it takes for a motor command to happen in real life
			virtual float GetMechanicalLag() = 0;
	};
	class SmoothMotionCalibrationSensors
	{
	public:
		virtual float GetSensorReading() = 0;
		virtual bool ShouldContinueCalibration() = 0;
		virtual void SetMotorPower(float power) = 0;
		virtual float GetTestSpeed(int step) = 0;
	};

	void calibrate(SmoothMotionCalibrationSensors* calibrateSensors);

	float computeCurrentSpeed();
	void recordData(float time, float distance);

	//tick returns true when you've gotten where you want to be
	bool tick(const float currentPos,
					const float targetPos,
					const float powerIfUnder,
					const float powerIfOver,
					const float accelRate,
					const float currentTime,
					const float stoppingSpeedTolerance,
					const float stoppingDistanceTolerance,
					const float blendGap,
					float* powerOutput,
					MechModel* mechModel);

};



#endif /* SRC_SMOOTHMOTIONMANAGER_H_ */
