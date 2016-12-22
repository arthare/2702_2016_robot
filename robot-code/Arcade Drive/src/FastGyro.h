/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef FastGyro_H_
#define FastGyro_H_

#include "SensorBase.h"
#include "PIDSource.h"
#include "LiveWindow/LiveWindowSendable.h"

class AnalogInput;

/**
 * Use a rate FastGyro to return the robots heading relative to a starting position.
 * The FastGyro class tracks the robots heading based on the starting position. As the robot
 * rotates the new heading is computed by integrating the rate of rotation returned
 * by the sensor. When the class is instantiated, it does a short calibration routine
 * where it samples the FastGyro while at rest to determine the default offset. This is
 * subtracted from each sample to determine the heading. This FastGyro class must be used
 * with a channel that is assigned one of the Analog accumulators from the FPGA. See
 * AnalogChannel for the current accumulator assignments.
 */
class FastGyro : public SensorBase
{
public:
	static const uint32_t kOversampleBits = 8;
	static const uint32_t kAverageBits = 0;

	static constexpr float kSamplesPerSecond = 200.0;
	static constexpr float kCalibrationSampleTime = 5.0;
	static constexpr float kDefaultVoltsPerDegreePerSecond = 0.007;

	FastGyro(uint8_t moduleNumber, uint32_t channel);
	explicit FastGyro(uint32_t channel);
	explicit FastGyro(AnalogInput *channel);
	explicit FastGyro(AnalogInput &channel);
	virtual ~FastGyro();
	virtual float GetAngle();
	virtual double GetRate();
	void SetSensitivity(float voltsPerDegreePerSecond);
	virtual void Reset();


private:
	void InitFastGyro();

	AnalogInput *m_analog;
	float m_voltsPerDegreePerSecond;
	float m_offset;
	bool m_channelAllocated;
	uint32_t m_center;

	ITable *m_table;
};
#endif
