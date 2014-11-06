
Tested with latest Arduino Beta  1.5.8

You'll need to install patched 'core' files into <your arduino 158 folder>\hardware\arduino\avr\cores\arduino

These sketches use a fork of Richard Barnett's sensor fusion library (originals available here https://github.com/richards-tech/RTIMULib-Arduino)

N.B. Sketch uses 'auto-bias on startup' method of calibration so keep still for a few seconds when pluggin in.



Values to play with on RTIMULib (see comments in code);


RTFusionRTQ.cpp

	RTQF_QVALUE
	RTQF_RVALUE


RTIMU.cpp
	
	COMPASS_ALPHA  - controls low pass filtering on compass value

