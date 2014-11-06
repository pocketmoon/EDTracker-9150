////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "RTIMU.h"
#include "RTIMUSettings.h"
#include "CalLib.h"

//  this sets the learning rate for compass running average calculation


//  this defines the accelerometer noise level

#define RTIMU_FUZZY_GYRO_ZERO           (RTFLOAT)0.20

#define RTIMU_FUZZY_GYRO_ZERO_SQUARED   (RTIMU_FUZZY_GYRO_ZERO * RTIMU_FUZZY_GYRO_ZERO)

//  this defines the accelerometer noise level

#define RTIMU_FUZZY_ACCEL_ZERO          (RTFLOAT)0.05

#define RTIMU_FUZZY_ACCEL_ZERO_SQUARED   (RTIMU_FUZZY_ACCEL_ZERO * RTIMU_FUZZY_ACCEL_ZERO)


#if defined(MPU9150_68) || defined(MPU9150_69)
#include "RTIMUMPU9150.h"
#endif

#if defined(LSM9DS0_6a) || defined(LSM9DS0_6b)
#include "RTIMULSM9DS0.h"
#endif

#if defined(GD20HM303D_6a) || defined(GD20HM303D_6b)
#include "RTIMUGD20HM303D.h"
#endif

#if defined(GD20M303DLHC_6a) || defined(GD20M303DLHC_6b)
#include "RTIMUGD20M303DLHC.h"
#endif

RTIMU *RTIMU::createIMU(RTIMUSettings *settings)
{
#if defined(MPU9150_68) || defined(MPU9150_69)
    return new RTIMUMPU9150(settings);
#endif
#if defined(LSM9DS0_6a) || defined(LSM9DS0_6b)
    return new RTIMULSM9DS0(settings);
#endif
#if defined(GD20HM303D_6a) || defined(GD20HM303D_6b)
    return new RTIMUGD20HM303D(settings);
#endif
#if defined(GD20M303DLHC_6a) || defined(GD20M303DLHC_6b)
    return new RTIMUGD20M303DLHC(settings);
#endif
}


RTIMU::RTIMU(RTIMUSettings *settings)
{
    m_settings = settings;

    m_calibrationMode = false;
    m_calibrationValid = false;
    m_gyroBiasValid = false;
}

RTIMU::~RTIMU()
{
}

void RTIMU::setCalibrationData()
{
    float maxDelta = -1;
    float delta;
    CALLIB_DATA calData;

    m_calibrationValid = false;

    if (calLibRead(0, &calData)) {
        if (calData.magValid != 1) {
            return;
        }

        //  find biggest range

        for (int i = 0; i < 3; i++) {
            if ((calData.magMax[i] - calData.magMin[i]) > maxDelta)
                maxDelta = calData.magMax[i] - calData.magMin[i];
        }
        if (maxDelta < 0) {
            return;
        }
        maxDelta /= 2.0f;                                       // this is the max +/- range

        for (int i = 0; i < 3; i++) {
            delta = (calData.magMax[i] - calData.magMin[i]) / 2.0f;
            m_compassCalScale[i] = maxDelta / delta;            // makes everything the same range
            m_compassCalOffset[i] = (calData.magMax[i] + calData.magMin[i]) / 2.0f;
        }
        m_calibrationValid = true;
    }
}

void RTIMU::gyroBiasInit()
{
    m_gyroAlpha = 2.0f / m_sampleRate;
    m_gyroSampleCount = 0;
}

void RTIMU::handleGyroBias()
{
    RTVector3 deltaAccel = m_previousAccel;
    deltaAccel -= m_accel;   // compute difference
    m_previousAccel = m_accel;

    if ((deltaAccel.squareLength() < RTIMU_FUZZY_ACCEL_ZERO_SQUARED) && 
                (m_gyro.squareLength() < RTIMU_FUZZY_GYRO_ZERO_SQUARED)) {
        // what we are seeing on the gyros should be bias only so learn from this
        m_gyroBias.setX((1.0 - m_gyroAlpha) * m_gyroBias.x() + m_gyroAlpha * m_gyro.x());
        m_gyroBias.setY((1.0 - m_gyroAlpha) * m_gyroBias.y() + m_gyroAlpha * m_gyro.y());
        m_gyroBias.setZ((1.0 - m_gyroAlpha) * m_gyroBias.z() + m_gyroAlpha * m_gyro.z());

        if (m_gyroSampleCount < (5 * m_sampleRate)) {
            m_gyroSampleCount++;

            if (m_gyroSampleCount == (5 * m_sampleRate)) {
                m_gyroBiasValid = true;
            }
        }
    }

    m_gyro -= m_gyroBias;
}

//http://www-users.cs.york.ac.uk/~fisher/cgi-bin/mkfscript

#define GAIN75HZ   0.00009544251

float RTIMU::filterp75hz (float *a, float *b)
{
  return  (  (a[0] + a[3]) + 3.0 * (a[1] + a[2])
             + (  0.8281462754 * b[0]) + ( -2.6404834928 * b[1])
             + (  2.8115736773 * b[2]));
}

//#define COMPASS_ALPHA                   (RTFLOAT)0.025
#define COMPASS_ALPHA                   (RTFLOAT)0.05
#define LPF1
void RTIMU::calibrateAverageCompass()
{
    //  calibrate if required

    if (!m_calibrationMode && m_calibrationValid) {
        m_compass.setX((m_compass.x() - m_compassCalOffset[0]) * m_compassCalScale[0]);
        m_compass.setY((m_compass.y() - m_compassCalOffset[1]) * m_compassCalScale[1]);
        m_compass.setZ((m_compass.z() - m_compassCalOffset[2]) * m_compassCalScale[2]);
    

    //  update running average
	#ifdef LPF1
	
	m_compassAverage.setX(m_compass.x() * COMPASS_ALPHA + m_compassAverage.x() * (1.0 - COMPASS_ALPHA));
    m_compassAverage.setY(m_compass.y() * COMPASS_ALPHA + m_compassAverage.y() * (1.0 - COMPASS_ALPHA));
    m_compassAverage.setZ(m_compass.z() * COMPASS_ALPHA + m_compassAverage.z() * (1.0 - COMPASS_ALPHA));
	
	#else
	
	 for (int i = 0; i < 3; i++)
        {
          ax[i] = ax[i + 1];
          bx[i] = bx[i + 1];
          ay[i] = ay[i + 1];
          by[i] = by[i + 1];
          az[i] = az[i + 1];
          bz[i] = bz[i + 1];
        }

        ax[3] = m_compass.x() * GAIN75HZ;
        ay[3] = m_compass.y() * GAIN75HZ;
        az[3] = m_compass.z() * GAIN75HZ;

        bx[3] = filterp75hz(ax, bx);
		m_compassAverage.setX(bx[3]);
		
        by[3] = filterp75hz(ay, by);
		m_compassAverage.setY(by[3]);

		bz[3] = filterp75hz(az, bz);
		m_compassAverage.setZ(bz[3]);		
		
#endif
    m_compass = m_compassAverage;
	}
}

bool RTIMU::IMUGyroBiasValid()
{
    return m_gyroBiasValid;
}


