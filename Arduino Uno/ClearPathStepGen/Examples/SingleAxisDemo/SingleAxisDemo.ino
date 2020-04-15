/*
  SingleAxisDemo
  Runs a Teknic ClearPath SDSK or SDHP motor, back and forth
 
  Copyright (c) 2017 Teknic Inc. This work is free to use, copy and distribute under the terms of the standard
  MIT permissive software license which can be found at https://opensource.org/licenses/MIT
 */

//Import Required libraries
#include <ClearPathMotorSD.h>
#include <ClearPathStepGen.h>

// initialize a ClearPathMotorSD Motor
ClearPathMotorSD X;

//initialize the controller and pass the reference to the motor we are controlling
ClearPathStepGen machine(&X);

// the setup routine runs once when you press reset:
void setup()
{  
  //Begin Serial Communication // NOTE: If communication lags, consider increasing baud rate
  Serial.begin(9600);
  
//X.attach(9);                //attach motor so Step/B is connected to pin 9
//X.attach(8,9);              //Direction/A is pin 8, Step/B is pin 9
//X.attach(8,9,6);            //Direction/A is pin 8, Step/B is pin 9, Enable is pin 6
X.attach(8,9,6,4);          //Direction/A is pin 8, Step/B is pin 9, Enable is pin 6, HLFB is pin 4

// Set max Velocity.  Parameter can be between 2 and 100,000 steps/sec
  X.setMaxVel(100000);
  
// Set max Acceleration.  Parameter can be between 4000 and 2,000,000 steps/sec/sec
  X.setMaxAccel(2000000);
  
// Enable motor, reset the motor position to 0
X.enable();

delay(100);

// Set up the ISR to constantly update motor position.  All motor(s) must be attached, and enabled before this function is called.
machine.Start();

 
}

// the loop routine runs over and over again forever:
void loop()
{  
 // Move the motor forward 10,000 steps
   X.move(10000);
   Serial.println("Move Start");
   
// wait until the command is finished and The motor's HLFB asserts
   while(!X.commandDone()||!X.readHLFB()) //just use command done if not using motor feedback
   { }
   Serial.println("Move Done");
   delay(1000);
  
// Move the motor backwards 10,000 steps
   X.move(-10000);
   Serial.println("Negative Move Begins");
   
// wait until the command is finished and The motor's HLFB asserts  
   while(!X.commandDone()||!X.readHLFB()) //just use command done if not using motor feedback
   { }
   Serial.println("Move Done");
   delay(1000);
   
   
}