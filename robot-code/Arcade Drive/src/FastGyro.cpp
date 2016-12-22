/*
 * FastGyro.cpp
 *
 *  Created on: Nov 10, 2016
 *      Author: rebotics-user
 */
/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "FastGyro.h"
#include "AnalogInput.h"
#include "Timer.h"
#include "WPIErrors.h"
#include "LiveWindow/LiveWindow.h"

const uint32_t FastGyro::kOversampleBits;
const uint32_t FastGyro::kAverageBits;

constexpr float FastGyro::kSamplesPerSecond;
constexpr float FastGyro::kCalibrationSampleTime;
constexpr float FastGyro::kDefaultVoltsPerDegreePerSecond;

/**
 * Initialize the gyro.
 * Calibrate the gyro by running for a number of samples and computing the center value for this
 * part. Then use the center value as the Accumulator center value for subsequent measurements.
 * It's important to make sure that the robot is not moving while the centering calculations are
 * in progress, this is typically done when the robot is first turned on while it's sitting at
 * rest before the competition starts.
 */
void FastGyro::InitFastGyro()
{
	m_table = NULL;
	if (!m_analog->IsAccumulatorChannel())
	{
		wpi_setWPIErrorWithContext(ParameterOutOfRange,
				"moduleNumber and/or channel (must be accumulator channel)");
		if (m_channelAllocated)
		{
			delete m_analog;
			m_analog = NULL;
		}
		return;
	}

	m_voltsPerDegreePerSecond = kDefaultVoltsPerDegreePerSecond;
	m_analog->SetAverageBits(kAverageBits);
	m_analog->SetOversampleBits(kOversampleBits);
	float sampleRate = kSamplesPerSecond *
		(1 << (kAverageBits + kOversampleBits));
	m_analog->SetSampleRate(sampleRate);
	Wait(1.0);

	m_analog->InitAccumulator();
	Wait(kCalibrationSampleTime);

	long long value;
	uint32_t count;
	m_analog->GetAccumulatorOutput(value, count);

	m_center = (uint32_t)((float)value / (float)count + .5);

	m_offset = ((float)value / (float)count) - (float)m_center;

	m_analog->SetAccumulatorCenter(m_center);
	m_analog->SetAccumulatorDeadband(0); ///< TODO: compute / parameterize this
	m_analog->ResetAccumulator();

}

/**
 * Gyro constructor given a slot and a channel.
 *
 * @param moduleNumber The analog module the gyro is connected to (1).
 * @param channel The analog channel the gyro is connected to (1 or 2).
 */
FastGyro::FastGyro(uint8_t moduleNumber, uint32_t channel)
{
	m_analog = new AnalogInput(channel);
	m_channelAllocated = true;
	InitFastGyro();
}

/**
 * Gyro constructor with only a channel.
 *
 * Use the default analog module slot.
 *
 * @param channel The analog channel the gyro is connected to.
 */
FastGyro::FastGyro(uint32_t channel)
{
	m_analog = new AnalogInput(channel);
	m_channelAllocated = true;
	InitFastGyro();
}

/**
 * Gyro constructor with a precreated analog channel object.
 * Use this constructor when the analog channel needs to be shared. There
 * is no reference counting when an AnalogChannel is passed to the gyro.
 * @param channel The AnalogChannel object that the gyro is connected to.
 */
FastGyro::FastGyro(AnalogInput *channel)
{
	m_analog = channel;
	m_channelAllocated = false;
	if (channel == NULL)
	{
		wpi_setWPIError(NullParameter);
	}
	else
	{
		InitFastGyro();
	}
}

FastGyro::FastGyro(AnalogInput &channel)
{
	m_analog = &channel;
	m_channelAllocated = false;
	InitFastGyro();
}

/**
 * Reset the gyro.
 * Resets the gyro to a heading of zero. This can be used if there is significant
 * drift in the gyro and it needs to be recalibrated after it has been running.
 */
void FastGyro::Reset()
{
	m_analog->ResetAccumulator();
}

/**
 * Delete (free) the accumulator and the analog components used for the gyro.
 */
FastGyro::~FastGyro()
{
	if (m_channelAllocated)
		delete m_analog;
}

/**
 * Return the actual angle in degrees that the robot is currently facing.
 *
 * The angle is based on the current accumulator value corrected by the oversampling rate, the
 * gyro type and the A/D calibration values.
 * The angle is continuous, that is can go beyond 360 degrees. This make algorithms that wouldn't
 * want to see a discontinuity in the gyro output as it sweeps past 0 on the second time around.
 *
 * @return the current heading of the robot in degrees. This heading is based on integration
 * of the returned rate from the gyro.
 */
float FastGyro::GetAngle( void )
{
	long long rawValue;
	uint32_t count;
	m_analog->GetAccumulatorOutput(rawValue, count);

	long long value = rawValue - (long long)((float)count * m_offset);

	double scaledValue = value * 1e-9 * (double)m_analog->GetLSBWeight() * (double)(1 << m_analog->GetAverageBits()) /
		(m_analog->GetSampleRate() * m_voltsPerDegreePerSecond);

	return (float)scaledValue;
}


/**
 * Return the rate of rotation of the gyro
 *
 * The rate is based on the most recent reading of the gyro analog value
 *
 * @return the current rate in degrees per second
 */
double FastGyro::GetRate( void )
{
	return (m_analog->GetAverageValue() - ((double)m_center + m_offset)) * 1e-9 * m_analog->GetLSBWeight()
			/ ((1 << m_analog->GetOversampleBits()) * m_voltsPerDegreePerSecond);
}


/**
 * Set the gyro type based on the sensitivity.
 * This takes the number of volts/degree/second sensitivity of the gyro and uses it in subsequent
 * calculations to allow the code to work with multiple gyros.
 *
 * @param voltsPerDegreePerSecond The type of gyro specified as the voltage that represents one degree/second.
 */
void FastGyro::SetSensitivity( float voltsPerDegreePerSecond )
{
	m_voltsPerDegreePerSecond = voltsPerDegreePerSecond;
}







