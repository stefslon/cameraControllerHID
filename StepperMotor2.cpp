/**
	Class to control stepper motor movement
	
	S.Slonevskiy
	May 2014
	
**/

#include "Arduino.h"
#include "StepperMotor2.h"


/*
	Constructor
*/
StepperMotor2::StepperMotor2(int pin1, int pin2, int pin3, int pin4, int calPin, int maxAngleDeg, int stepStyle)
{
	// Assign pins
	_pins[0] = pin1;
	_pins[1] = pin2;
	_pins[2] = pin3;
	_pins[3] = pin4;
	_calPin  = calPin;
	
	// Init pins to digital output
	for (int count = 0; count < 4; count++) {
		pinMode(_pins[count], OUTPUT);
	}
	
	// Init calibration pin
	pinMode(_calPin, INPUT_PULLUP);
	
	// Define stepping technique
	if (stepStyle==SM2_HALFSTEP) { 
		_numStepStates = NELEMS(__steppingHalf);
		for (int ii=0; ii<_numStepStates; ii++) {
			_stepStates[ii] = __steppingHalf[ii];
		}
	}
	else { // if (stepStyle==SM2_FULLSTEP) {
		_numStepStates = NELEMS(__steppingFull);
		for (int ii=0; ii<_numStepStates; ii++) {
			_stepStates[ii] = __steppingFull[ii];
		}
	}
	
	/* WARNING! DO NOT PUT ANY Serial.print STATEMENTS IN THIS SECTION! 	*/
	/* At the moment of construction of this function, serial interface is	*/
	/* not initialized yet, so it hangs the entire Teensy 					*/

	// Init internal delay counter
	_previousStepMicros = 0;
	
	// Init go-to location as false
	_gotoActive = false;
	_gotoPosition = 0;
	
	// Init motor characteristcs
	// Note: 512 * _numStepStates = 360 degrees 
	_maxPosition = (unsigned long)512 * (unsigned long)_numStepStates * (unsigned long)maxAngleDeg / 360;
	_udegreesStepSize = (unsigned long)360 * (unsigned long)1000000 / ( (unsigned long)512 * (unsigned long)_numStepStates );

	// Setup calibration mode
	setupCal();
}

/*
	Use in debugging...
*/
void StepperMotor2::debugInternalParams()
{
	Serial.print("SM2 Init: _maxPosition ");
	Serial.println(_maxPosition);
	
	Serial.print("SM2 Init: _udegreesStepSize ");
	Serial.println(_udegreesStepSize);
	
	Serial.print("SM2 Init: _stepDelay ");
	Serial.println(_stepDelay);
}


/*
	Start continuous motion
	int directionSpeedDPS 	sets motion speed in degrees/second, sign signifies direction
*/
void StepperMotor2::run(int directionSpeedDPS)
{
	if (!_isCalibrationMode) {
		_gotoActive = false;
		if (directionSpeedDPS==0) {
			_stepDirection = 0;
		}
		else {
			_stepDirection = (directionSpeedDPS>0) ? +1 : -1;
			_stepDelay = _udegreesStepSize / abs(directionSpeedDPS); //SM2_MAX_DELAY-abs(speedDirection);
			//Serial.print("Step delay at ");
			//Serial.print(_stepDelay);
			//Serial.println();
		}
	}
}


/*
	Move stepper one semi-step, no delay, no repetitive motion
	Stops any continuous runs currently on
	int direction 		sets direction of the step, magnitude of the parameter is ignored
*/
void StepperMotor2::step(int direction)
{
	if (!_isCalibrationMode) {
		_gotoActive = false;
		_stepDirection = 0;
		//_currentStepState = _currentStepState + 1;
		if (direction<0 || direction>0) {
			_currentPosition += (direction>0) ? +1 : -1;
			writeStepToPins();
			//Serial.print("StepperMotor2: stepped to position ");
			//Serial.println(_currentPosition);
		}
	}
}

/*
	Stop any current motion, except calibration
*/
void StepperMotor2::stop()
{
	if (!_isCalibrationMode) {
		_stepDirection = 0;
	}
}


/*
	Goto a sepcified position in the specified amount of time
	int newPosition 				step number to go to
	unsigned int msecToComplete 	mseconds to complete this operation
*/
void StepperMotor2::gotoPosition(int newPosition, unsigned int msecToComplete)
{
	_gotoActive = true;
	_gotoPosition = newPosition;
	_gofromPosition = _currentPosition;
	_gotoMsecToComplete = msecToComplete;
	if (newPosition>_currentPosition) {
		_stepDirection = +1;
	}
	else if (newPosition<_currentPosition) {
		_stepDirection = -1;
	}
	else {
		_stepDirection = 0;
		_gotoActive = false;
	}
	
	//requestedSpeedDPS = (_gotoPosition-_currentPosition) * _udegreesStepSize / (timeToComplete*1e6);
	//_stepDelay = _udegreesStepSize / requestedSpeedDPS;
	
	// _stepDelay is in units of usec / step
	_stepDelay = ((unsigned long)msecToComplete*1e3) / (unsigned long)abs(_gotoPosition-_currentPosition);
	
	Serial.print("StepperMotor2: go to position ");
	Serial.print(_gotoPosition);
	Serial.print(" with step delay of ");
	Serial.print(_stepDelay);
	Serial.print(" usec, from current position of ");
	Serial.print(_currentPosition);
	Serial.print(", with step direction ");
	Serial.println(_stepDirection);
}

/*
	Invoke recalibration mode
*/
void StepperMotor2::recal()
{
	setupCal();
}

/*
	Move stepper motor one step (when elapsed delay time passed)
	Must be called from the loop() function
*/
void StepperMotor2::move()
{
	unsigned long currentMicros = micros();
	if((currentMicros - _previousStepMicros > _stepDelay) && (_stepDirection!=0)) {
		_previousStepMicros = currentMicros;
		
		//_currentStepState = _currentStepState + _stepDirection;
		_currentPosition += _stepDirection;

		writeStepToPins();
	}
	
	if (_isCalibrationMode) {
		// Check calibration pin
		// If there, then define this as position 0, and go to position where we started
		bool calState = digitalRead(_calPin);	
		if (!calState) {
			// When cal pin goes low we are there
			_gotoPosition = -_currentPosition;
			_currentPosition = 0;
			_isCalibrationMode = false;
			gotoPosition(_gotoPosition,1500);

			Serial.println("StepperMotor2: calibration finished.");
		}
	}
	else {
		// Goto location logic
		if (_gotoActive && _gotoPosition==_currentPosition) {
			_gotoActive = false;
			_stepDirection = 0;
			Serial.println("StepperMotor2: go to position reached");
		}
		else {
			//_stepDelay = ((unsigned long)msecToComplete*1e3) / (unsigned long)abs(_gotoPosition-_currentPosition);
			if (_gotoActive) {
				_stepDelay = sineEaseInOut(abs(_gotoPosition - _currentPosition), abs(_gotoPosition - _gofromPosition), _gotoMsecToComplete);
				if (_stepDelay<SM2_MIN_DELAY) { _stepDelay = SM2_MIN_DELAY; Serial.println("    _stepDelay limiting!"); }
				if (_stepDelay>SM2_MAX_DELAY) { _stepDelay = SM2_MAX_DELAY; Serial.println("    _stepDelay limiting!"); }
				//Serial.print("    _stepDelay update ");
				//Serial.println(_stepDelay);
			}
		}
	}
}

/*
	PRIVATE
*/

/*
	Current step number is translated to the high/low state on four stepper wires
	Rotation edges are enfored here
*/
void StepperMotor2::writeStepToPins()
{
	/*
	if (_currentStepState>_numStepStates-1) {
		_currentStepState = 0;
		_currentPosition++;

		//Serial.print("StepperMotor2: position ");
		//Serial.println(_currentPosition);

	}
	if (_currentStepState<0) {
		_currentStepState = _numStepStates-1;
		_currentPosition--;

		//Serial.print("StepperMotor2: position ");
		//Serial.println(_currentPosition);

	}
	*/
	
	if (_currentPosition>=0) {
		_currentStepState = _currentPosition % _numStepStates;
	}
	else {
		_currentStepState = _numStepStates - ((-_currentPosition) % _numStepStates);
	}
	
	if (!_isCalibrationMode && ((_currentPosition<=0) || (_currentPosition>=_maxPosition))) {
		// Cannot move anywhere we are at the either edge so stop
		stop();
		
		// If edge was overjumped, then reset
		if (_currentPosition<=0) _currentPosition=0;
		if (_currentPosition>=_maxPosition) _currentPosition=_maxPosition;

		Serial.print("StepperMotor2: reached the edge, position ");
		Serial.println(_currentPosition);

		return;
	}
	
	unsigned int mask = 0001;
	for (int count = 0; count < 4; count++) {
		if (_stepStates[_currentStepState] & mask) {
			digitalWrite(_pins[count], HIGH);
		}
		else {
			digitalWrite(_pins[count], LOW);
		}
		mask <<= 1;
	}
}

/*
	Setup parameters required for calibration
*/
void StepperMotor2::setupCal()
{
	_currentPosition = 0;
	_stepDirection = -1; // counter-clockwise turn until calibration event is triggered
	_stepDelay = _udegreesStepSize / SM2_CAL_SPEED_DPS;	// calibration mode can be pretty fast
	_isCalibrationMode = true; // make sure cal mode is on
}

/*
	Sine in/out easing used in gotoPosition function
	int p 			remaining steps
	int c			total number of steps to make
	unsigned int d 	time to complete operation
	return 			delay time in microseconds
*/
unsigned long StepperMotor2::sineEaseInOut(int p, int c, unsigned int d)
{

	long aa = (1000-2*((long)p*1000)/(long)c);
	unsigned long bb = 1000000-(unsigned long)(aa*aa);
	unsigned long cc = isqrt(bb);
	//unsigned long dt = 1000000*(unsigned long)d/cc*2/c/3;
	unsigned long dt = 1000000/cc*(unsigned long)d*2/c/3;
	
	return dt;
}

/*
	Integer square root
	unsigned long x 	integer to take a square root of
	return				square root of a number
*/
unsigned long StepperMotor2::isqrt(unsigned long x)
{  
    unsigned long op, res, one;  
    
    op = x;  
    res = 0;  
  
    /* "one" starts at the highest power of four <= than the argument. */  
    one = (unsigned long)1 << 30;  /* second-to-top bit set */  
    while (one > op) one >>= 2;  
  
    while (one != 0) {  
        if (op >= res + one) {  
            op -= res + one;  
            res += one << 1;  // <-- faster than 2 * one  
        }  
        res >>= 1;  
        one >>= 2;  
    }  
    return res;  
}
