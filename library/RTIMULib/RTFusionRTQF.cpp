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

#include "RTFusionRTQF.h"

//  The QVALUE affects the gyro response.
//0.001f
//#define RTQF_QVALUE	0.001f
#define RTQF_QVALUE	0.0001f

//  The RVALUE controls the influence of the accels and compass.
//  The bigger the value, the more sluggish the response.

//0.0010f  //25 smooth but laggy. 0.0020 is good
//0.005f    id slightly laged
#define RTQF_RVALUE	0.0005f  


RTFusionRTQF::RTFusionRTQF()
{
    m_Q = RTQF_QVALUE;
    m_R = RTQF_RVALUE;
    reset();
}

RTFusionRTQF::~RTFusionRTQF()
{
}

void RTFusionRTQF::reset()
{
    m_firstTime = true;
    m_fusionPose = RTVector3();
    m_fusionQPose.fromEuler(m_fusionPose);
    m_measuredPose = RTVector3();
    m_measuredQPose.fromEuler(m_measuredPose);
}

void RTFusionRTQF::newIMUData(const RTVector3& gyro, const RTVector3& accel, const RTVector3& compass, unsigned long timestamp)
{
    if (m_firstTime) {
        m_lastFusionTime = timestamp;
        calculatePose(accel, compass);

        //  initialize the poses

        m_fusionQPose.fromEuler(m_measuredPose);
        m_fusionPose = m_measuredPose;
        m_firstTime = false;
    } else {
        m_timeDelta = (RTFLOAT)(timestamp - m_lastFusionTime) / (RTFLOAT)1000;
        m_lastFusionTime = timestamp;
        if (m_timeDelta <= 0)
            return;

        calculatePose(accel, compass);

//      predict();

        RTFLOAT x2, y2, z2;
        RTFLOAT qs, qx, qy,qz;

        qs = m_fusionQPose.scalar();
        qx = m_fusionQPose.x();
        qy = m_fusionQPose.y();
        qz = m_fusionQPose.z();

        x2 = gyro.x() * (RTFLOAT)0.5;
        y2 = gyro.y() * (RTFLOAT)0.5;
        z2 = gyro.z() * (RTFLOAT)0.5;

        // Predict new state

        m_fusionQPose.setScalar(qs + (-x2 * qx - y2 * qy - z2 * qz) * m_timeDelta);
        m_fusionQPose.setX(qx + (x2 * qs + z2 * qy - y2 * qz) * m_timeDelta);
        m_fusionQPose.setY(qy + (y2 * qs - z2 * qx + x2 * qz) * m_timeDelta);
        m_fusionQPose.setZ(qz + (z2 * qs + y2 * qx - x2 * qy) * m_timeDelta);

//      update();

        m_stateQError = m_measuredQPose - m_fusionQPose;

        // make new state estimate

        RTFLOAT qt = m_Q * m_timeDelta;

        m_fusionQPose += m_stateQError * (qt / (qt + m_R));

        m_fusionQPose.normalize();

        m_fusionQPose.toEuler(m_fusionPose);
    }
}

void RTFusionRTQF::calculatePose(const RTVector3& accel, const RTVector3& mag)
{
    RTQuaternion m;
    RTQuaternion q;

    accel.accelToEuler(m_measuredPose);

//  q.fromEuler(m_measuredPose);

    RTFLOAT cosX2 = cos(m_measuredPose.x() * 0.5f);
    RTFLOAT sinX2 = sin(m_measuredPose.x() * 0.5f);
    RTFLOAT cosY2 = cos(m_measuredPose.y() * 0.5f);
    RTFLOAT sinY2 = sin(m_measuredPose.y() * 0.5f);

    q.setScalar(cosX2 * cosY2);
    q.setX(sinX2 * cosY2);
    q.setY(cosX2 * sinY2);
    q.setZ( - sinX2 * sinY2);
 //   normalize();

    m.setScalar(0);
    m.setX(mag.x());
    m.setY(mag.y());
    m.setZ(mag.z());

    m = q * m * q.conjugate();
    m_measuredPose.setZ(-atan2(m.y(), m.x()));

    m_measuredQPose.fromEuler(m_measuredPose);

    //  check for quaternion aliasing. If the quaternion has the wrong sign
    //  the kalman filter will be very unhappy.

    int maxIndex = -1;
    RTFLOAT maxVal = -1000;

    for (int i = 0; i < 4; i++) {
        if (fabs(m_measuredQPose.data(i)) > maxVal) {
            maxVal = fabs(m_measuredQPose.data(i));
            maxIndex = i;
        }
    }

    //  if the biggest component has a different sign in the measured and kalman poses,
    //  change the sign of the measured pose to match.

    if (((m_measuredQPose.data(maxIndex) < 0) && (m_fusionQPose.data(maxIndex) > 0)) ||
            ((m_measuredQPose.data(maxIndex) > 0) && (m_fusionQPose.data(maxIndex) < 0))) {
        m_measuredQPose.setScalar(-m_measuredQPose.scalar());
        m_measuredQPose.setX(-m_measuredQPose.x());
        m_measuredQPose.setY(-m_measuredQPose.y());
        m_measuredQPose.setZ(-m_measuredQPose.z());
        m_measuredQPose.toEuler(m_measuredPose);
    }
}
