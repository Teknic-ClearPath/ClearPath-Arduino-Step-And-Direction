This Library sends pulsed step and Direction signals to ClearPath motors to precisly control position, and velocity of up to 6 motors simulateously up to 4000RPM.  The methodology used to create the pulse step and direction signals is not compatible with stepper motors, or with other servo systems.  The pulsed step and direction also requires a RAS setting of at least 16 ms on all ClearPath motors, a longer RAS will results in quieter motion.

This library defines two classes used to manage and control the motors:ClearPathMotorSD, and ClearPathStepGen.

The ClearPathMotorSD class is used to manage an individual motor including setting up pins, commanding moves, setting velocity/acceleration etc.  The following is a list of commands in this class:

--- ClearPathMotorSD - default constructor for initializing the motor
   
   
--- attach() - Attachs pins to this motor, and declares them as input/outputs

   
--- stopMove()  - Interupts the current move, the motor may abruptly stop

   
--- move() - sets the maximum veloctiy

   
--- disable() - disables the motor

   
--- enable() - enables the motor

   
--- getCommandedPosition() - Returns the absolute cmomanded position where the position on enable=0

   
--- readHLFB() - Returns the value of the motor's HLFB Pin

   
--- setMaxVel() - sets the maximum veloctiy

   
--- setMaxAccel() - sets the acceleration

   
--- commandDone() - returns wheter or not there is a valid current command
   


The ClearPathStepGen class is the class which manages the sending of the pulsed step and direction signals to all motors.  This is accomplished by setting up a Timer based ISR at around 2kHz (using Timer2), and directly writing to the I/O register Port B.  Becuase only PORTB is used to send step signals, the B input of the ClearPath motors must be connected to pins 8-13 on an Arduino Uno.  Unused pins on PORTB may be used for other function without interfereing with this library.

NOTE: If you are using another type of arduino besides the UNO, the ClearPathStepGen must be modified to output on a different PORT and/or the Contrustor function must be modified to correctly relate the pin numbers to the bits of the I/O Resister you would like to use.  For example,

In an Arduino Mega, PORTA refers to pins, 22-29, so to modify this library to use a Mega simply:

1) Do a find/replace all replacing "PORTB" with "PORTA"  (case sensitive, ignore quotes).

2) Do a find/replace all replacing "->getDirPin()-8" with "->getDirPin()-22" (case sensitive, ignore quotes).

After this, you could use an Arduino Mega and connect input B to pins 22-29


