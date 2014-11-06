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

//10-Oct-2014  First Magnetometer base line sketch


const char  infoString []  = "EDTrackerMag V1.0.1";


#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h"
#include "CalLib.h"
#include <EEPROM.h>

RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;

float cx, cy, cz;
#define BUTTON_PIN 10
#define LED_PIN 17 // (Arduino is 13, Teensy is 11, Teensy++ is 6)


/* EEPROM Offsets for config and calibration stuff*/
#define EE_VERSION 0
// these are now longs (4 bytes)
#define EE_XGYRO 1
#define EE_YGYRO 5
#define EE_ZGYRO 9
#define EE_XACCEL 13
#define EE_YACCEL 17
#define EE_ZACCEL 21
// 1 byte
#define EE_ORIENTATION 25
// 2 bytes
#define EE_XDRIFTCOMP 26

//0 for linear, 1 for exponential
#define EE_EXPSCALEMODE 28

//2x 1 byte  in 6:2   0.25 steps should be ok
#define EE_YAWSCALE 29
#define EE_PITCHSCALE 30
#define EE_YAWEXPSCALE 31
#define EE_PITCHEXPSCALE 32

//single bytes
#define EE_POLLMPU 33
#define EE_AUTOCENTRE 34


long gBias[3], aBias[3], fBias[3];
boolean recentre = false;

//anually apply these later
static byte gyro_orients[6] =
{
  B10001000, // Z Up X Forward
  B10000101, // X right
  B10101100, // X Back
  B10100001, // X Left
  B01010100, // left ear
  B01110000  // right ear
}; //ZYX

enum outputModeType {
  OFF,
  DBG,
  UI
};
outputModeType outputMode = OFF;
int samples = 0;
//Running count of samples - used when recalibrating
boolean calibrated = false;
boolean blinkState = false;

byte orientation = 1;

//Need some helper funct to read/write integers
void writeIntEE(int address, int value) {
  EEPROM.write(address + 1, value >> 8); //upper byte
  EEPROM.write(address, value & 0xff); // write lower byte
}

int readIntEE(int address) {
  return (EEPROM.read(address + 1) << 8 | EEPROM.read(address));
}

void writeLongEE(int address,  long value) {
  for (int i = 0; i < 4; i++)
  {
    EEPROM.write(address++, value & 0xff); // write lower byte
    value = value >> 8;
  }
}

long readLongEE(int address) {
  return ((long)EEPROM.read(address + 3) << 24 |
          (long)EEPROM.read(address + 2) << 16 |
          (long)EEPROM.read(address + 1) << 8 |
          (long)EEPROM.read(address));
}
unsigned long lastReport = 0;

boolean expScaleMode = 0;
float   yawScale = 1.0;
float   pitchScale = 1.0;
//unsigned char revision;

void blink()
{
   blinkState = !blinkState;
   digitalWrite(LED_PIN, blinkState);  
}

void setup()
{
  int errcode;
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);
  Wire.begin();

  imu = RTIMU::createIMU(&settings);
  cx = cy = cz = 0.0;

  Serial.println(imu->IMUName());

  if ((errcode = imu->IMUInit()) < 0) {
    Serial.print("Failed to initialise MPU");
    Serial.println(errcode);
  }

  if (imu->getCalibrationValid())
    Serial.println("M\tCompass calibration OK");
  else
    Serial.println("M\tCompass calibration data Invalid");

  lastDisplay = lastRate = millis();
  sampleCount = 0;


  orientation = constrain(EEPROM.read(EE_ORIENTATION), 0, 5);

  expScaleMode = EEPROM.read(EE_EXPSCALEMODE);
  getScales();

  // mpu_read_6050_accel_bias(fBias);
  //delay(100);
  //loadBiases();
  //delay(100);
    Serial.println("M\tKeep EDTracker stationary to calibrate.");

  while (imu->IMUGyroBiasValid()==false)
  {
    if (millis()> lastReport)
    {
    lastReport=millis()+100;
    blink();
  }
     if (imu->IMURead()) 
     {             
        fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
     }
  }
      Serial.println("M\tCalibration complete.");


}


float
expFunc (float newX, float scale)
{
  return (0.000122076 * newX * newX * scale) * (newX / fabs(newX));
}

RTVector3 v, a;

TrackState_t joySt;

unsigned long lastUpdate = 0;
void loop()
{
  unsigned long now = millis();
  unsigned long delta;

  if (imu->IMURead()) 
  {
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    sampleCount++;

    v = fusion.getFusionPose();

    float newX = v.z() * 10430.06;
    float newY = v.y() * 10430.06;
    float newZ = v.x() * 10430.06;

    if (digitalRead(BUTTON_PIN) == LOW  || recentre)
    {
      cx = newX;
      cy = newY;
      cz = newZ;
      recentre = false;
    }

    newX = newX - cx;

    // this should take us back to zero BUT we may have wrapped so ..
    if (newX < -32768.0)
      newX += 65536.0;

    if (newX > 32768.0)
      newX -= 65536.0 ;

    newY = newY - cy;
    newZ = newZ - cz;

    newX = constrain(newX, -16383.0, 16383.0);
    newY = constrain(newY, -16383.0, 16383.0);
    newZ = constrain(newZ, -16383.0, 16383.0);

    long  iX ;
    long  iY ;
    long  iZ ;

    if (expScaleMode) {
      iX = expFunc(newX, yawScale);
      iY = expFunc(newY, pitchScale);
      iZ = expFunc(newZ, pitchScale);
    }
    else
    {
      // and scale to out target range plus a 'sensitivity' factor;
      iX = (newX * yawScale );
      iY = (newY * pitchScale );
      iZ = (newZ * pitchScale );
    }

    // clamp after scaling to keep values within 16 bit range
    iX = constrain(iX, -32767, 32767);
    iY = constrain(iY, -32767, 32767);
    iZ = constrain(iZ, -32767, 32767);

    // Do it to it.
    joySt.xAxis = iX ;
    joySt.yAxis = iY ;
    joySt.zAxis = iZ ;

    Tracker.setState(&joySt);
    samples++;

    unsigned long    nowMillis = millis();

    if (outputMode == UI &&  nowMillis > lastReport)
    {
      Serial.print(iX ); // Yaw
      Serial.print("\t");
      Serial.print(iY); // Pitch
      Serial.print("\t");
      Serial.print(iZ); // Roll
      Serial.print("\t");

      a = imu->getAccel();
      short tmp[3];
      tmp[0] = a.x() * 1000.0;
      tmp[1] = a.y() * 1000.0;
      tmp[2] = a.z() * 1000.0;

      tripple(tmp);
      a = imu->getGyro();

      tmp[0] = a.x() * 1000.0;
      tmp[1] = a.y() * 1000.0;
      tmp[2] = a.z() * 1000.0;

      tripple(tmp);
      Serial.println("");
      lastReport = nowMillis + 25;
    }
    
    if (nowMillis > lastUpdate)
    {
      blink();
      lastUpdate = nowMillis + 1000;
      //Serial.println(samples);
      samples = 0;
    }
    parseInput();

  
  }
}


void parseInput()
{
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    byte command = Serial.read();

    bool scale = false;
    if (command == 'c')
    {
      yawScale += 0.25;
      scale = true;
    }
    if (command == 'C')
    {
      yawScale += 1.0;
      scale = true;
    }

    if (command == 'd')
    {
      yawScale -= 0.25;
      scale = true;
    }
    if (command == 'G')
    {
      yawScale -= 1.0;
      scale = true;
    }

    if (command == 'e')
    {
      pitchScale += 0.25;
      scale = true;
    }
    if (command == 'E')
    {
      pitchScale += 1.0;
      scale = true;
    }

    if (command == 'f')
    {
      pitchScale -= 0.25;
      scale = true;
    }
    if (command == 'F')
    {
      pitchScale -= 1.0;
      scale = true;
    }
    if (scale)
    {
      setScales();
      scl();
    }

    //    }
    //else
    if (command == 'S')
    {
      outputMode = OFF;
      Serial.println("S"); //silent
      //dmp_set_fifo_rate(DEFAULT_MPU_HZ);

    }
    else if (command == 'H')
    {
      Serial.println("H"); // Hello
    }
    else if (command == 't')
    {
      //toggle linear/expoinential mode
      expScaleMode = !expScaleMode;
      getScales();
      EEPROM.write(EE_EXPSCALEMODE, expScaleMode);
      scl();
    }
    else if (command == 'V')
    {
      Serial.println("V"); //verbose
      //Serial.print("I\t");
      //Serial.println(infoString);
      sendInfo();

      scl();

      outputMode = UI;


    }
    else if (command == 'I')
    {
      //Serial.print("I\t");
      //Serial.println(infoString);
      sendInfo();

      //Serial.println("M\t----------------");

      //Serial.print("O\t");
      //Serial.println(orientation);
      //sendByte('O', orientation);

      //mess("M\tGyro Bias ", gBias);
      //mess("M\tAccel Bias ", aBias);
      //mess("M\tFact Bias ", fBias);

      //Serial.print("M\tMPU Revision ");
      //Serial.println(revision);
      scl();
      //polling();
      // sendByte('p', 0);

      //      Serial.print("O\t");
      //      Serial.println(orientation);
      sendByte('O', orientation);
      //sendByte('#', 0);

    }
    else if (command == 'P')
    {
      //mpu_set_dmp_state(0);
      orientation = (orientation + 1) % 6; //0 to 5
      //dmp_set_orientation(gyro_orients[orientation]);
      // mpu_set_dmp_state(1);
      //Serial.print("O\t");
      //Serial.println(orientation);
      sendByte('O', orientation);
      EEPROM.write(EE_ORIENTATION, orientation);
    }
    else if (command == 'R')
    {
      //recalibrate offsets
      recentre = true;
    }
    else if (command == 'H')
    {
      Serial.println("H"); // Hello
    }


  }
}








//
void loadBiases() {

  for (int i = 0; i < 3; i++)
  {
    gBias[i] = readLongEE (EE_XGYRO  + i * 4);
    aBias[i] = readLongEE (EE_XACCEL + i * 4);
  }

  //mpu_set_gyro_bias_reg(gBias);
  //mpu_set_accel_bias_6050_reg(aBias, true);

  return ;
}





void tripple(short *v)
{
  for (int i = 0; i < 3; i++)
  {
    Serial.print(v[i] ); //
    Serial.print("\t");
  }
}

void mess(char *m, long*v)
{
  Serial.print(m);
  Serial.print(v[0]); Serial.print(" / ");
  Serial.print(v[1]); Serial.print(" / ");
  Serial.println(v[2]);
}



void getScales()
{
  if (expScaleMode)
  {
    yawScale = (float)EEPROM.read(EE_YAWEXPSCALE) / 4.0;
    pitchScale = (float)EEPROM.read(EE_PITCHEXPSCALE) / 4.0;

    if (yawScale == 0 || yawScale > 60)
      yawScale = 14.0;

    if (pitchScale == 0 || pitchScale > 60)
      pitchScale = 14.0;
  }
  else
  {
    yawScale = (float)EEPROM.read(EE_YAWSCALE) / 4.0;
    pitchScale = (float)EEPROM.read(EE_PITCHSCALE) / 4.0;

    if (yawScale == 0 || yawScale > 60)
      yawScale = 5.0;

    if (pitchScale == 0 || pitchScale > 60)
      pitchScale = 5.0;
  }
}


void setScales()
{
  if (expScaleMode)
  {
    EEPROM.write(EE_YAWEXPSCALE, (int)(yawScale * 4));
    EEPROM.write(EE_PITCHEXPSCALE, (int)(pitchScale * 4));
  }
  else
  {
    EEPROM.write(EE_YAWSCALE, (int)(yawScale * 4));
    EEPROM.write(EE_PITCHSCALE, (int)(pitchScale * 4));
  }
}

void sendBool(char x, boolean v)
{
  Serial.print(x);
  Serial.print("\t");
  Serial.println(v);
}

void sendByte(char x, byte b)
{
  Serial.print(x);
  Serial.print("\t");
  Serial.println(b);
}

void sendInfo()
{
  Serial.print("I\t");
  Serial.print(infoString);
  Serial.print(":");
  Serial.println(imu->IMUName());
}

void
scl()
{
  Serial.print("s\t");
  Serial.print(expScaleMode);
  Serial.print("\t");
  Serial.print(yawScale);
  Serial.print("\t");
  Serial.println(pitchScale);
}
