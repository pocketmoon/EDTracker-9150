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

#ifndef _RTIMULIBDEFS_H
#define	_RTIMULIBDEFS_H

#include "RTMath.h"

//  IMU enable defs - only one should be enabled, the rest commented out

#define MPU9150_68                      // MPU9150 at address 0x68
//#define MPU9150_69                      // MPU9150 at address 0x69
//#define LSM9DS0_6a                      // LSM9DS0 at address 0x6a
//#define LSM9DS0_6b                      // LSM9DS0 at address 0x6b
//#define GD20HM303D_6a                   // GD20H + M303D at address 0x6a
//#define GD20HM303D_6b                   // GD20H + M303D at address 0x6b
//#define GD20M303DLHC_6a                 // GD20 + M303DLHC at address 0x6a
//#define GD20M303DLHC_6b                 // GD20 + M303DLHC at address 0x6b

//  IMU type codes

#define RTIMU_TYPE_MPU9150                  1                   // InvenSense MPU9150
#define RTIMU_TYPE_LSM9DS0                  2                   // STM LSM9DS0 (eg Sparkfun IMU)
#define RTIMU_TYPE_GD20HM303D               3                   // STM L3GD20H/LSM303D (Pololu Altimu)
#define RTIMU_TYPE_GD20M303DLHC             4                   // STM L3GD20/LSM303DHLC (Adafruit IMU)

#endif // _RTIMULIBDEFS_H
