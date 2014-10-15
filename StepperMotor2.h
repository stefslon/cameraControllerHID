/**
	Class to control stepper motor movement
	
	S.Slonevskiy
	May 2014
	
**/

#ifndef StepperMotor2_h
#define StepperMotor2_h

#include "Arduino.h"

#define SM2_FULLSTEP 	0
#define SM2_HALFSTEP 	1

#define SM2_MAX_DELAY 	100000 	// us
#define SM2_MIN_DELAY 	1000 	// us
#define SM2_CAL_SPEED_DPS 25		// degrees/second -- eventually want to max out!

#define NELEMS(x)  (sizeof(x) / sizeof(x[0]))

const int __steppingHalf[] = {1, 3, 2, 6, 4, 12, 8, 9};
const int __steppingFull[] = {3, 6, 12, 9};

class StepperMotor2
{
	public:
		StepperMotor2(int pin1, int pin2, int pin3, int pin4, int calPin, int maxAngleDeg, int stepStyle = SM2_FULLSTEP);
		
		/* Motion related functions */
		void step(int direction);
		void run(int directionSpeedDPS);
		void stop();
		void gotoPosition(int newPosition, unsigned int msecToComplete);
		void recal();
		
		/* Informational functions */
		int getPosition() { return _currentPosition; }
		bool isCalMode() { return _isCalibrationMode; }
		
		/* This function is to be called from the loop() */
		void move();
		
		/* Misc routines */
		void debugInternalParams();

	private:
		unsigned long _previousStepMicros;
		
		int _currentPosition; 	// current position (pointing angle essentially)
		unsigned int _maxPosition; // 360 deg = 512 * 4 or 8 positions
		unsigned long _udegreesStepSize; // size of a microstep in units of microdegrees, ie udegrees/step
		
		bool _gotoActive;		// flagger for go-to function
		unsigned int _gotoMsecToComplete; 	// time to complete go-to operation
		int _gotoPosition;		// requested go-to position
		int _gofromPosition; 	// position before go-to was requested
		
		int _pins[4];
		int _calPin; // pin hooked up to hall effect sensor
		
		unsigned long _stepDelay; 	// us delay between steps
		int _stepDirection; 		// +1 or -1 (or 0) to signify step state advance direction)
		
		int _currentStepState; // curent step in the step table
		int _stepStates[8]; // store selected step values (stored as integers, but really binary values is what we are looking at)
		int _numStepStates; // just to know how many steps are in the selected technique

		bool _isCalibrationMode;
		
		/* Internal/private functions */
		void setupCal();
		void writeStepToPins();
		unsigned long sineEaseInOut(int p, int c, unsigned int d);
		unsigned long isqrt(unsigned long x);
};

#endif
