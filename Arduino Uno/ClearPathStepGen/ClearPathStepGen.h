/*
  ClearPathStepGen.h - Interrupt driven library for controlling Clearpath motors using an Arduino- Version 1
  Teknic 2017 Brendan Flosenzier

  This library is free software; you can redistribute it and/or
  modify it.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

/* 
  
  A ClearPathStepGen is activated by creating an instance of the ClearPathStepGen class, and passing the constructor references to the motors it will control.
  The motors are pulsed in the background according to move information stored within each motor.
  Only motors of type; ClearPathMotorSD may be used.

  There can only be one instance of Step Controller at any time.

  This class uses Timer2, so other functions and classes which use timer 2 will not work correctly ie: tone(), MsTimer2() etc.

  The ISR is set to 2KHz, nominally

  Note: Each attached motor must have its direction/B pin connected to one of pins 8-13

  other devices can be connected to pins 8-13 as well

 
   Start(time)     - gets Direction pins for all connected motors (make sure all motors have been attached before this is called
						Configures the ISR to run at 2kHz
   Stop() - disables the ISR in this class
   
 */
#ifndef ClearPathStepGen_h
#define ClearPathStepGen_h

#include "Arduino.h"
#include "ClearPathMotorSD.h"

class ClearPathStepGen
{
  public:
  ClearPathStepGen(ClearPathMotorSD* motor1);
  ClearPathStepGen(ClearPathMotorSD* motor1, ClearPathMotorSD* motor2);
  ClearPathStepGen(ClearPathMotorSD* motor1, ClearPathMotorSD* motor2, ClearPathMotorSD* motor3);
  ClearPathStepGen(ClearPathMotorSD* motor1, ClearPathMotorSD* motor2, ClearPathMotorSD* motor3, ClearPathMotorSD* motor4);
  ClearPathStepGen(ClearPathMotorSD* motor1, ClearPathMotorSD* motor2, ClearPathMotorSD* motor3, ClearPathMotorSD* motor4, ClearPathMotorSD* motor5);
  ClearPathStepGen(ClearPathMotorSD* motor1, ClearPathMotorSD* motor2, ClearPathMotorSD* motor3, ClearPathMotorSD* motor4, ClearPathMotorSD* motor5, ClearPathMotorSD* motor6);
  void Start();
  void Stop();
  int getsum();


};
#endif
