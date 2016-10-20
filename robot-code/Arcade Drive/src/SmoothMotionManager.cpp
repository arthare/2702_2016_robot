/*
 * SmoothMotionManager.cpp
 *
 *  Created on: Oct 6, 2016
 *      Author: rebotics-user
 */
#include "SmoothMotionManager.h"
#include <deque>
#include <math.h>

using namespace std;
float blendNum(float x, float y, float blendToY, float max, float min)
{
	float ret = (1-blendToY)*x + blendToY*y;
	if (ret < min)
	{
		ret = min;
	}
	if (ret > max)
	{
		ret = max;
	}
	return ret;
}




float getSpeedOfPoints(const deque<timeDistance>& data)
{
  //using https://en.wikipedia.org/wiki/Simple_linear_regression

  float timeAvg = 0;
  float distAvg = 0;

  for(deque<timeDistance>::const_iterator i = data.begin(); i != data.end(); i++){
    const timeDistance pt = *i;
    timeAvg += pt.time;
    distAvg += pt.distance;
  }

  timeAvg = timeAvg / data.size();
  distAvg = distAvg / data.size();

  float top = 0;
  float bottom = 0;

  for(deque<timeDistance>::const_iterator i = data.begin(); i != data.end(); i++){
    const timeDistance pt = *i;
    top += (pt.distance - distAvg)*(pt.time - timeAvg);
    bottom += pow((pt.time - timeAvg),2);
  }

  float beta = top/bottom;

  return beta;

}

	SmoothMotionManager::SmoothMotionManager(float initialPos, float time):lastPos(initialPos),
													lastTime(time),
													lastSpeed(0),
													initTime(time)
	{

	}



	void SmoothMotionManager::calibrate(SmoothMotionCalibrationSensors* calibrateSensors){
		for(int i = 0; i < 5 && calibrateSensors->ShouldContinueCalibration(); i++){
			float motorPower = calibrateSensors->GetTestSpeed(i);
			float sensorReadingBefore = calibrateSensors->GetSensorReading();
			calibrateSensors->SetMotorPower(motorPower);

			const int secondsToTest = 2;
			int startTime = GetFPGATime();
			int endTime = GetFPGATime() + secondsToTest * 1000000;
			while(calibrateSensors->ShouldContinueCalibration() && GetFPGATime() < endTime) {
				calibrateSensors->SetMotorPower(motorPower);
				Wait(0.01);
			}

			float sensorReadingAfter = calibrateSensors->GetSensorReading();
			float sensorDelta = sensorReadingAfter-sensorReadingBefore;
			float sensorRate = sensorDelta/2;
			printf("%.2f \t %.2f \n", motorPower, sensorRate);
		}
		calibrateSensors->SetMotorPower(0);
	}

	float SmoothMotionManager::computeCurrentSpeed()
	{
		float slope = getSpeedOfPoints(recordedData);
		return slope;
	}
	void SmoothMotionManager::recordData(float time, float distance)
	{
		const float keepTime = 0.05;
		recordedData.push_back(timeDistance(time, distance));
		while(recordedData.size() > 0 && recordedData.front().time < time-keepTime){
			recordedData.pop_front();
		}
	}
	bool SmoothMotionManager::tick(const float currentPos,
					const float targetPos,
					const float powerIfUnder,
					const float powerIfOver,
					const float accelRate,
					const float currentTime,
					const float stoppingSpeedTolerance,
					const float stoppingDistanceTolerance,
					const float blendGap,
					float* powerOutput,
					MechModel* mechModel)
	{
		const float deltaTime = currentTime - lastTime;
		lastTime = currentTime;

		const float distToTarget = fabs(targetPos - currentPos);

		if (deltaTime == 0){
			return false;
		}
		const float curSpeed = computeCurrentSpeed();

		recordData(currentTime, currentPos);
		lastPos = currentPos;

		const float absCurSpeed = fabs(curSpeed);
		if (distToTarget < stoppingDistanceTolerance && absCurSpeed < stoppingSpeedTolerance)
		{
			// achieved target: tell the caller we're done
			return true;
		}

		const float maxa = accelRate;
		float futurePos = currentPos + (mechModel->GetMechanicalLag()*curSpeed);
		float futureDistToTarget = targetPos-futurePos;
		float targetSpeedForDecel = sqrt(2*maxa*abs(futureDistToTarget));
		float secPassed = currentTime-initTime;
		float targetSpeedForAcel = maxa*secPassed;
		float targetSpeed = min(targetSpeedForDecel, targetSpeedForAcel);

		printf("%08.2f  %08.2f  %08.2f  %08.2f \n",targetSpeedForDecel,secPassed,targetSpeedForAcel,targetSpeed);

		// In-progress: attempting to keep wheels from spinning
		//targetSpeed = max(targetSpeed,lastSpeed-maxa*deltaTime);
		//targetSpeed = min(targetSpeed,lastSpeed+maxa*deltaTime);
		const float gap = blendGap;
		if(futurePos > targetPos)
		{
			targetSpeed = targetSpeed * -1;
		}

		if(curSpeed < targetSpeed-gap)
		{
			*powerOutput = powerIfUnder;
		}
		else if(curSpeed >= targetSpeed-gap && curSpeed <= targetSpeed )
		{
			const float min = targetSpeed - gap;
			const float max = targetSpeed;
			float blend = (curSpeed-min)/(max-min);
			//blending from max power and the movement model
			*powerOutput = blendNum(powerIfUnder,mechModel->GetMotorPowerForSpeed(targetSpeed),blend,powerIfUnder,powerIfOver);
		}
		else if(curSpeed <= targetSpeed+gap && curSpeed > targetSpeed )
		{
			const float min = targetSpeed;
			const float max = targetSpeed + gap;
			float blend = (curSpeed-min)/(max-min);
			//blending from max power and the movement model
			*powerOutput = blendNum(mechModel->GetMotorPowerForSpeed(targetSpeed),powerIfOver,blend,powerIfUnder,powerIfOver);
		}
		else
		{
			*powerOutput = powerIfOver;
		}
		/*printf("%.2f \t %.2f \t %.2f \t %.2f \t %.2f  %.2f \n",
					*powerOutput,
					currentTime,
					curSpeed,
					currentPos,
					targetSpeed,
					futurePos);*/
		lastSpeed = curSpeed;
		return false;
	}




