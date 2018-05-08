/*
  ClearPathMotorSD.h - Library for interfacing with Clearpath-SD motors using an Arduino- Version 1
  Teknic 2017 Brendan Flosenzier 

  This library is free software; you can redistribute it and/or
  modify it.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

/* 
  
  A ClearPathMotorSD is activated by creating an instance of the ClearPathMotorSD class.

  There can several instances of ClearPathMotorSD however each must be attached to different pins.

  This class is used in conjuntion with the ClearPathStepGen class which manages and sends step pulses to each motor.
  
  Note: Each attached motor must have its direction/B pin connected to one of pins 8-13 on an arduino UNO (PORTB) 
  in order to work with the ClearPathStepGen object.  Other devices can be connected to pins 8-13 as well.
  If you are using another Arduino besides the UNO, the ClearPathStepGen must be modifyed to use a different port.

  The functions for a ClearPathMotorSD are:

   ClearPathMotorSD - default constructor for initializing the motor
   
   attach() - Attachs pins to this motor, and declares them as input/outputs

   stopMove()  - Interupts the current move, the motor may abruptly stop

   move() - sets the maximum veloctiy

   disable() - disables the motor

   enable() - enables the motor

   getCommandedPosition() - Returns the absolute cmomanded position where the position on enable=0

   readHLFB() - Returns the value of the motor's HLFB Pin

   setMaxVel() - sets the maximum veloctiy

   setMaxAccel() - sets the acceleration

   commandDone() - returns wheter or not there is a valid current command
   
 */
#include "Arduino.h"
#include "ClearPathMotorSD.h"


/*		
	This is an internal Function used by ClearPathStepGen to calculate how many pulses to send to each motor.
	It tracks the current command, as well as how many steps have been sent, and calculates how many steps
	to send in the next ISR.
*/
int ClearPathMotorSD::calcSteps()
{  
	_TX++;//increment time
	if(!Enabled)
		return 0;

	// Process current move state.
	switch(moveStateX){
		case 3: // IdleState state, executed only once.

 if(CommandX == 0) //If no/finished command/, do nothing set everything to 0
{
                _PX=0;
                _VX=0;
                _OPX=0;
                _TX=0;
                _TX1=0;
                _TX2=0;
                _TX3=0;
				_BurstX=0;
}
else
{
			// Compute Move parameters
			_ZX = CommandX<<_bitshift;
			_ZX2 = abs(_ZX>>1);
			if(_ZX > 0)
				_AXS = _AXMX;
			else
				_AXS = -_AXMX;
			_AX = _AXS;
			// Do immediate move if half move length <= maximum acceleration.
			if(_ZX2 <= _AXMX) {
				_AX = 0;
				_VX = 0;
				_PX = _ZX;
				moveStateX = 3;	//Set to Move Idle
                CommandX=0;		//Zero command
				break;
			}
			// Otherwise, execute move and go to Phase1
			_PX = _PX + _VX;// + (_AX>>1);
	                _VX = _VX + _AX;
			moveStateX = 1;
}
			break;

		case 1:		//Phase 1 first half of move
			
			// Execute move
			_PX = _PX + _VX + (_AX>>1);
	                _VX = _VX + _AX;

			// Check position.
			if(abs(_PX) >= _ZX2) {
				// If half move reached, compute time parameters and go to PXhase2

				if(_flag)		//This makes sure you go one step past half in order to make sure Phase 2 goes well
				{
				if(_TX1 == 0) {
					_TX1 = _TX;
				}
				if(_TX2 == 0) {
					_TX2 = _TX;
				}
				_AX = -_AX;					//Set deccelleration
				_TX3 = (_TX2<<1) - _TX1;	//compute time params
				_TAUX = _TX2<<1;
				moveStateX = 2;			//Start Phase 2
				}
				_flag=true;
				
			}
			else {
				// Otherwise, check velocity.
				if(labs(_VX) >= _VMX) {
					// If maximum velocity reached, compute TX1 and set AX = 0, and _VX=_VMX.
					if(_TX1 == 0) {
						_AX = 0;
						_TX1 = _TX;
						if(_VX > 0)
                          _VX=_VMX;
						else
                          _VX=-_VMX;
					}
				}
			}
			break;

		case 2:		//Phase 2 2nd half of move
			// Execute move
			_PX = _PX + _VX + (_AX>>1);
	                _VX = _VX + _AX;

			// Check time.
			if(_TX >= _TX3) {
				// If beyond TX3, wait for done condition.
				_AX = -_AXS;
				if((_TX > _TAUX) || (labs(_PX) > labs(_ZX)) || (_VX*_AX > 0)) {
            	// If done, enforce final position.
					_AX = 0;
					_VX = 0;
					_PX = _ZX;
					moveStateX = 3;
					CommandX=0;
				}
			}
			break;
	}
	// Compute burst value
	_BurstX = (_PX - _OPX)>>_bitshift;
	// Update accumulated integer position
	_OPX += (long)(_BurstX)<<_bitshift;

	//check which direction, and incement absPosition
	if(_direction)
		AbsPosition+=_BurstX;
	else
		AbsPosition-=_BurstX;
	return _BurstX;

}


/*		
	This is the default constructor.  This intializes the variables.
*/
ClearPathMotorSD::ClearPathMotorSD()
{
  moveStateX=3;
  PinA=0;
  PinB=0;
  PinE=0;
  PinH=0;
  Enabled=false;
_VMX=0;					
_AXMX=0;
_PX=0;				
_OPX=0;				
 _VX=0;				
_AX=0;					
_TX=0;					
_TX1=0;				
 _TX2=0;				
_TX3=0;			
_TAUX=0;					
_flag=0;
_AXS=0;					
 _ZX=0;				
_ZX2=0;					
CommandX=0;
_bitshift=10;
_BurstX=0;
AbsPosition=0;
}

/*		
	This is the one pin attach function.  It asociates the passed number, as this motors Step Pin
*/
void ClearPathMotorSD::attach(int BPin)
{
  PinA=0;
  PinB=BPin;
  PinE=0;
  PinH=0;
  pinMode(PinB,OUTPUT);
}

/*		
	This is the two pin attach function.  
	It asociates the 1st number, as this motors Direction Pin
	and the 2nd number with the Step Pin
*/
void ClearPathMotorSD::attach(int APin, int BPin)
{
  PinA=APin;
  PinB=BPin;
  PinE=0;
  PinH=0;
  pinMode(PinA,OUTPUT);
  pinMode(PinB,OUTPUT);
}

/*		
	This is the three pin attach function.  
	It asociates the 1st number, as this motors Direction Pin,
	the 2nd number with the Step Pin,
	and the 3rd number with the Enable Pin
*/
void ClearPathMotorSD::attach(int APin, int BPin, int EPin)
{
  PinA=APin;
  PinB=BPin;
  PinE=EPin;
  PinH=0;
  pinMode(PinA,OUTPUT);
  pinMode(PinB,OUTPUT);
  pinMode(PinE,OUTPUT);
}

/*		
	This is the four pin attach function.  
	It asociates the 1st number, as this motors Direction Pin,
	the 2nd number with the Step Pin,
	the 3rd number with the Enable Pin,
	and the 4th number as the HLFB Pin
*/
void ClearPathMotorSD::attach(int APin, int BPin, int EPin, int HPin)
{
  PinA=APin;
  PinB=BPin;
  PinE=EPin;
  PinH=HPin;
  pinMode(PinA,OUTPUT);
  pinMode(PinB,OUTPUT);
  pinMode(PinE,OUTPUT);
  pinMode(PinH,INPUT_PULLUP);
}

/*		
	This function clears the current move, and puts the motor in a
	move idle state, without disabling it, or clearing the position.

	This may cause an abrupt stop.
*/
void ClearPathMotorSD::stopMove()
{
	cli();
	_PX=0;
	_VX=0;
     _OPX=0;
     _TX=0;
     _TX1=0;
     _TX2=0;
     _TX3=0;
	 _BurstX=0;
	 moveStateX = 3;
     CommandX=0;
	 sei();
}

/*		
	This function commands a directional move
	The move cannot be longer than 2,000,000 counts
	If there is a current move, it will NOT be overwritten

	The function will return true if the move was accepted
*/
boolean ClearPathMotorSD::move(long dist)
{
  if(CommandX==0)
  {
	  if(dist<0)
	  {
		  if(PinA!=0)
		  {
			  digitalWrite(PinA,HIGH);
			  _direction=true;
		  }
		  CommandX=-dist;
	  }
	  else
	  {
		  if(PinA!=0)
		  {
			  digitalWrite(PinA,LOW);
			  _direction=false;
		  }
		CommandX=dist;
	  }
	  return true;
  }
  else
	  return false;

}

/*		
	This function sets the velocity in Counts/sec assuming the ISR frequency is 2kHz.
	The maximum value for velMax is 100,000, the minimum is 2
*/
void ClearPathMotorSD::setMaxVel(long velMax)
{
	int n = velMax/2000;
	if(n<51)
		_VMX=(velMax*(1<<_bitshift))/2000;
	else
		_VMX=50*(1<<_bitshift);

}
/*		
	This function sets the acceleration in Counts/sec/sec assuming the ISR frequency is 2kHz.
	The maximum value for accelMax is 2,000,000, the minimum is 4,000
*/
void ClearPathMotorSD::setMaxAccel(long accelMax)
{
  _AXMX=(accelMax*(1<<_bitshift))/4000000;
}


/*		
	This function returns the absolute commanded position
*/
long ClearPathMotorSD::getCommandedPosition()
{
		return AbsPosition;
}

/*		
	This function returns true if there is no current command
	It returns false if there is a current command
*/
boolean ClearPathMotorSD::commandDone()
{

	if(CommandX==0)
		return true;
	else
		return false;
}


/*		
	This function returns the value of the HLFB Pin
*/
boolean ClearPathMotorSD::readHLFB()
{
	if(PinH!=0)
		return !digitalRead(PinH);
	else
		return false;
}

/*		
	This function enables the motor
*/
void ClearPathMotorSD::enable()
{

	if(PinE!=0)
		digitalWrite(PinE,HIGH);
	AbsPosition=0;
	Enabled=true;
}

/*		
	This function returns zeros out the current command, and digitally writes the enable pin LOW
	If the motor was not attached with an enable pin, then it just zeros the command
*/
void ClearPathMotorSD::disable()
{

	if(PinE!=0)
		digitalWrite(PinE,LOW);
	_PX=0;
    Enabled=false;
	_VX=0;
     _OPX=0;
     _TX=0;
     _TX1=0;
     _TX2=0;
     _TX3=0;
	 _BurstX=0;
	 moveStateX = 3;
     CommandX=0;
}
