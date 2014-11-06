/* ============================================
EDTracker device code is placed under the MIT License

Copyright (c) 2014 Rob James

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/


const char  infoString []  = "EDTrackerMCal V1.0.1";

#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "CalLib.h"
#include <EEPROM.h>

enum outputModeType {
  OFF,
  DBG,
  UI
};

outputModeType outputMode = OFF;

RTIMU *imu;                                           // the IMU object
RTIMUSettings settings;                               // the settings object
CALLIB_DATA magCalData;                                  // the calibration data


//  1Hz   0.00021960621
// 0.75hz  0.00009544251
// 0.5hz  0.00002914649
#define GAIN50HZ    0.00002914649
#define GAIN75HZ   0.00009544251
#define GAIN1HZ    0.00021960621


float filter1hz (float *xv, float *yv)
{
  return  (xv[0] + xv[3]) + 3.0 * (xv[1] + xv[2]) + ( 0.7776385602 * yv[0]) + ( -2.5282312191 * yv[1]) + (  2.7488358092 * yv[2]);
}

float filterp75hz (float *xv, float *yv)
{
  return  (  (xv[0] + xv[3]) + 3.0 * (xv[1] + xv[2])
             + (  0.8281462754 * yv[0]) + ( -2.6404834928 * yv[1])
             + (  2.8115736773 * yv[2]));
}

float filterp50hz (float *xv, float *yv)
{
  return  ( (xv[0] + xv[3]) + 3.0 * (xv[1] + xv[2])
            + (  0.8818931306 * yv[0]) + ( -2.7564831952 * yv[1])
            + (  2.8743568927 * yv[2]));
}


void setup()
{
  Serial.begin(115200);
  Wire.begin();
  delay(1000);

  calLibRead(0, &magCalData);
  magCalData.magValid = false;

  for (int i = 0; i < 3; i++)
  {
    magCalData.magMin[i] = 9999;
    magCalData.magMax[i] = -9999;
  }

  imu = RTIMU::createIMU(&settings);
  imu->IMUInit();
  imu->setCalibrationMode(true); // so we get raw values...
  Serial.print("M\tFound device: ");
  Serial.println(imu->IMUName());

}

float headRaw=0.0;
float headSimple=0.0;
unsigned long lastReport=0;
float headLPF=0.0;


float xv[4];
float yv[4];

float xw[4];
float yw[4];

void loop()
{
  boolean changed;
  RTVector3 mag;

  if (imu->IMURead())
  {
    changed = false;
    mag = imu->getCompass();

    for (int i = 0; i < 3; i++)
    {
      if (mag.data(i) < magCalData.magMin[i])
      {
        magCalData.magMin[i] = mag.data(i);
        changed = true;
      }
      if (mag.data(i) > magCalData.magMax[i]) {
        magCalData.magMax[i] = mag.data(i);
        changed = true;
      }
    }

    parseInput();
    
 /*   float ox =  mag.data(0)-27.0;
    float oy=  mag.data(1)-8.0;
    
     for (int i = 0; i < 3; i++)
        {
          xv[i] = xv[i + 1];
          yv[i] = yv[i + 1];
          xw[i] = xw[i + 1];
          yw[i] = yw[i + 1];
        }

        xv[3] = ox * GAIN75HZ;
        xw[3] = oy * GAIN75HZ;

        yv[3] = filterp75hz(xv, yv);
        yw[3] = filterp75hz(xw, yw);
        headLPF = (atan2(yw[3], yv[3]));//-magCentre;
    
    headRaw = atan2(oy,ox);
    headSimple = headSimple * 0.97 + atan2(oy,ox)*0.03;
    
    //raw
    Serial.print(ox,4);
      Serial.print("\t");
      Serial.print(oy,4);
      Serial.print("\t");
      
      
      Serial.print(headRaw,4);
      Serial.print("\t");
      
      //basic filter
      Serial.print(headSimple,4);
      Serial.print("\t");

      //LPF
      Serial.print(headLPF,4);
      Serial.println("");
*/
    if (changed  && outputMode == UI)
    {
      lastReport = millis()+1000;
     reportMagCal();
    }
//    else
//    {
//      Serial.print(mag.data(0),4);
//      Serial.print("\t");
//      Serial.print(mag.data(1),4);
//      Serial.print("\t");
//      Serial.println(mag.data(2),4); 
//    }
  }
}

void reportMagCal()
{
   Serial.print("$\t");
      Serial.print(magCalData.magMin[0]);
      Serial.print("\t");
      Serial.print(magCalData.magMax[0]);
      Serial.print("\t");
      Serial.print(magCalData.magMin[1]);
      Serial.print("\t");
      Serial.print(magCalData.magMax[1]);
      Serial.print("\t");
      Serial.print(magCalData.magMin[2]);
      Serial.print("\t");
      Serial.println(magCalData.magMax[2]);
}
void parseInput()
{
  if (Serial.available() > 0)
  {
    byte command = Serial.read();
    if (command == 'S')
    {
      outputMode = OFF;
      Serial.println("S"); //silent
    }
    else if (command == 'H')
    {
      Serial.println("H"); // Hello
    }
    else if (command == 'V')
    {
      Serial.println("V"); //verbose
      sendInfo();
      outputMode = UI;
    }
    else if (command == 'I')
    {
      sendInfo();
    }
    else if (command == 'H')
    {
      Serial.println("H"); // Hello
    }
    else if (command == '$')
    {
      magCalData.magValid = true;
      calLibWrite(0, &magCalData);
      Serial.print("M\tMag cal data saved for device ");
      Serial.println(imu->IMUName());
    }
  }
}

void sendInfo()
{
  Serial.print("I\t");
  Serial.print(infoString);
  Serial.print(":");
  Serial.println(imu->IMUName());
  reportMagCal();
}



