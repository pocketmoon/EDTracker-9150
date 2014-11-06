# RTIMULib-Arduino - a versatile 9-dof IMU library for the Arduino

RTIMULib-Arduino is the simplest way to connect a 9-dof IMU to an Arduino and obtain full fused quaternion or Euler angle pose data.

Check out www.richards-tech.com for more details, updates and news.

## Release history

Note that any older release can be obtained via the Releases tab on the repo's GitHub page.

### October 4 2014 - 2.2.0

Added support for the L3GD20 + LSM303DLHC IMU combo and the L3GD20H + LSM303D combo. As a result, there are an increased number of #defines in RTIMULibDefs.h, only one of which should be un-commented in order to select the IMU in use. By default, it now looks like:

	#define MPU9150_68                      // MPU9150 at address 0x68
	//#define MPU9150_69                      // MPU9150 at address 0x69
	//#define LSM9DS0_6a                      // LSM9DS0 at address 0x6a
	//#define LSM9DS0_6b                      // LSM9DS0 at address 0x6b
	//#define GD20HM303D_6a                   // GD20H + M303D at address 0x6a
	//#define GD20HM303D_6b                   // GD20H + M303D at address 0x6b
	//#define GD20M303DLHC_6a                 // GD20 + M303DLHC at address 0x6a
	//#define GD20M303DLHC_6b                 // GD20 + M303DLHC at address 0x6b

Changed the gyro bias calculation to run automatically when the IMU is detected as being stable. This means
that the IMU no longer needs to be kept still for 5 seconds and gyro bias is continually tracked. IMUGyroBiasValid can be called to check if enough stable samples have been obtained for a reasonable bias calculation to be made. If the IMU is stable, this will normally occur within 1 second. If it never indicates a valid bias, the #defines RTIMU_FUZZY_GYRO_ZERO and/or RTIMU_FUZZY_ACCEL_ZERO may need to be increased if the gyro bias or accelerometer noise is unusually high. These should be set to be greater than the readings observed when the IMU is completely stable.

Updated I2Cdev to the latest version.

### September 26 2014 - 2.1.0

Added a new sketch - ArduinoAccel. This shows how to subtract a rotated gravity vector using quaternions
in order to obtain the residual accelerations. It's basically used in exactly the same way as ArduinoIMU but
displays the residual accelerations instead. Note that there's no accel calibration at the moment so there 
will be some residual accelerations indicated that aren't real. Also, as usual (!), it's essential to perform
magnetometer calibration or else results will be useless.

### September 26 2014 - 2.0.0

There have been significant changes in this version (V2) to reduce the memory footprint. Instead of the previous
autodetection system, the IMU and address is now selected using a #define in RTIMULibDefs.h in the libraries
subdirectory. By default it looks like this:

	#define MPU9150_68                      // MPU9150 at address 0x68
	//#define MPU9150_69                      // MPU9150 at address 0x69
	//#define LSM9DS0_6a                      // LSM9DS0 at address 0x6a
	//#define LSM9DS0_6b                      // LSM9DS0 at address 0x6b

Change the commenting as required to select the device and address. All other functionality has remained the same. V1 is still available via the GitHub repo release tab.

### May 5 2014 - 1.0.0

Fixed bug in MPU-9150 compass initialization - changed incorrect writeBytes to readBytes to get fuse ROM data.

### April 22 2014 - 0.9.0

#### First release

This version supports the InvenSense MPU-9150 and STM LSM9DS0 single chip IMUs. The fusion filter is RTFusionRTQF.

(Pre-V2 only) The software will automatically discover the IMU type in use and also the address being used. This can be overridden in RTIMUSettings.cpp if desired.

Two sketches are included. ArduinoMagCal can be used to store magnetometer calibration data. Load the sketch and waggle the IMU around, making sure all axes reach their minima and maxima. The display will stop updating when this occurs. Then, enter 's' followed by enter into the IDE serial monitor to save the data.

ArduinoIMU is the main demo program. It configures the IMU based on settings in RTIMUSettings.cpp. Change these to alter any of the parameters. By default, it runs at 50 (MPU-9150) or 95 (LSM9DS0) gyro and accel samples per second. The display is updated only 3 times per second regardless of IMU sample rate.

Note that, prior to version 2.2.0, the gyro bias is being calculated during the first 5 seconds. If the IMU is moved during this period, the bias calculation may be incorrect and the code will need to be restarted. Starting at version 2.2.0 this is no longer a problem and gyro bias will be reported as valid after the required number of stable samples have been obtained.
