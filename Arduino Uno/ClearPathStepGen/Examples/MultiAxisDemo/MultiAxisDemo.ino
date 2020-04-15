/*
  Multi Axis Axample
  Runs 3 a Teknic ClearPath SDSK or SDHP motors
  
  the motors cycle forward taking turns starting with X, then both X and Y, then X, Y, and Z
 
 
 Copyright (c) 2017 Teknic Inc. This work is free to use, copy and distribute under the terms of the standard
  MIT permissive software license which can be found at https://opensource.org/licenses/MIT
 */



//Import Required libraries
#include <ClearPathMotorSD.h>
#include <ClearPathStepGen.h>

// initialize a ClearPathMotorSD Motors
ClearPathMotorSD X,Y,Z;

//initialize the controller and pass the references to the motors we are controlling
ClearPathStepGen machine(&X,&Y,&Z);

// the setup routine runs once when you press reset:
void setup()
{  
  
  //Begin Serial Communication // NOTE: WHEN GOING FAST, communication may lag
  Serial.begin(9600);
  
  //Setup pins, In this example all motors share the same enable signal
  X.attach(8,9,6,4);     //Direction/A is pin 8, Step/B is pin 9, Enable is pin 6, HLFB is pin 4
  Y.attach(10,11,6,5);   //Direction/A is pin 10, Step/B is pin 11, Enable is pin 6, HLFB is pin 5
  Z.attach(12,13,6,7);   //Direction/A is pin 12, Step/B is pin 13, Enable is pin 6, HLFB is pin 7
  
  //Set velocity and Acceleration in steps/sec, and steps/sec/sec
  X.setMaxVel(100000);
  X.setMaxAccel(200000);
  Y.setMaxVel(10000);
  Y.setMaxAccel(20000);
  Z.setMaxVel(100000);
  Z.setMaxAccel(4000);
  
// Enable motors, reset each motors position to 0  
X.enable();
Y.enable();
Z.enable();

delay(100);  

// Set up the ISR to constantly update motor position.
machine.Start();

 
}

// the loop routine runs over and over again forever:
void loop()
{  
// Move the X motor forward 50,000 counts
   X.move(50000);
   Serial.println("X Motor Move");
   
// wait until the command is finished and the motor's HLFB asserts
   while(!X.commandDone()||!X.readHLFB()) //just use command done if not using motor feedback
   { }

// Move the Y motor forward 100,000 counts
   Y.move(100000);
   Serial.println("Y Motor Move");
   // Move the X motor back 50,000 counts
   X.move(50000);
   Serial.println("X Motor Move");
   
// wait until the command is finished and both motor's HLFB asserts
   while(!Y.commandDone()||!Y.readHLFB()||!X.commandDone()||!X.readHLFB()) 
   { }
   
   // Move the Y motor back 50,000 counts
   Y.move(-50000);
   Serial.println("Y Motor Move");
   // Move the X motor back 50,000 counts
   X.move(-50000);
   Serial.println("X Motor Move");
   // Move the Z motor forward 100,000 counts  
   Z.move(100000);
   Serial.println("Z Motor Move");
   
// wait until the command is finished and The motor's HLFB asserts
   while(!Z.commandDone()||!Z.readHLFB()||!Y.commandDone()||!Y.readHLFB()||!X.commandDone()||!X.readHLFB()) 
   { }
   
   
   // Move the Y motor back 50,000 counts
   Y.move(-50000);
   Serial.println("Y Motor Move");
   // Move the X motor back 50,000 counts
   X.move(-50000);
   Serial.println("X Motor Move");
   // Move the Z motor forward 100,000 counts  
   Z.move(-100000);
   Serial.println("Z Motor Move");
   
// wait until the command is finished and The motor's HLFB asserts
   while(!Z.commandDone()||!Z.readHLFB()||!Y.commandDone()||!Y.readHLFB()||!X.commandDone()||!X.readHLFB())
   { }
   
   
   
}
