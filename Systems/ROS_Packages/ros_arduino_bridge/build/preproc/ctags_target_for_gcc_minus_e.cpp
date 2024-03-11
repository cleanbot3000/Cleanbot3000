# 1 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
/*********************************************************************
 *  ROSArduinoBridge
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */

   /* The Pololu VNH5019 dual motor driver shield */
   //#define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   //#define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   //#define ROBOGAIA

   /* Encoders directly attached to Arduino board */


   /* BLD120A Motor driver*/



//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h


/* Serial port baud rate */


/* Maximum PWM signal */



# 80 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 2




/* Include definition of serial commands */
# 86 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 2

/* Sensor functions */
# 89 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 2

/* Include servo support if required */






  /* Motor driver function definitions */
# 99 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 2

  /* Encoder driver function definitions */
# 102 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 2

  /* PID parameters and functions */
# 105 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 2

  /* Run the PID loop at 30 times per second */

  bool LED_STATE = true;

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / 30 /* Hz*/;

  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */

  long lastMotorCommand = 2000;


/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = 
# 144 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3 4
       __null
# 144 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
           ;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
  case 'b':
    Serial.println(57600);
    break;
  case 'a':
    Serial.println(analogRead(arg1));
    break;
  case 'd':
    Serial.println(digitalRead(arg1));
    break;
  case 'x':
    analogWrite(arg1, arg2);
    Serial.println("OK");
    break;
  case 'w':
    if (arg2 == 0) digitalWrite(arg1, 0x0);
    else if (arg2 == 1) digitalWrite(arg1, 0x1);
    Serial.println("OK");
    break;
  case 'c':
    if (arg2 == 0) pinMode(arg1, 0x0);
    else if (arg2 == 1) pinMode(arg1, 0x1);
    Serial.println("OK");
    break;
  case 'p':
    Serial.println(Ping(arg1));
    break;
# 200 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
  case 'e':
    Serial.print(readEncoder(0));
    Serial.print(" ");
    Serial.println(readEncoder(1));
    break;
   case 'r':
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case 'm':
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK");
    break;
  case 'o':
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    resetPID();
    moving = 0; // Sneaky way to temporarily disable the PID
    setMotorSpeeds(arg1, arg2);
    Serial.println("OK");
    break;
  case 'u':
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;

  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  pinMode(2, 0x1);
  pinMode(4, 0x1);
  pinMode(5 /*changed from 3 for testing purposes*/, 0x1);
  pinMode(8, 0x1);

  pinMode(18 /*PC4  //pin A4 //TBD*/, 0x0);
  pinMode(19 /*PC5   //pin A5 //TBD*/, 0x0);

  // pinMode(ISR, OUTPUT);
  // digitalWrite(ISR, LOW);

  // cli();                      //stop interrupts for till we make the settings
  // /*1. First we reset the control register to make sure we start with everything disabled.*/
  // TCCR1A = 0;                 // Reset entire TCCR1A to 0 
  // TCCR1B = 0;                 // Reset entire TCCR1B to 0

  // /*2. We set the prescalar to the desired value by changing the CS10 CS12 and CS12 bits. */  
  // TCCR1B |= B00000100;        //Set CS12 to 1 so we get prescalar 256  

  // /*3. We enable compare match mode on register A*/
  // TIMSK1 |= B00000010;        //Set OCIE1A to 1 so we enable compare match A 

  // /*4. Set the value of register A to 31250*/
  // OCR1A = 62500;             //Finally we set compare register A to this value  
  // sei();  

  pinMode(20 /*PD2  //pin 2 //TBD*/, 0x0);
  pinMode(21 /*PD3  //pin 3 //TBD*/, 0x0);

  attachInterrupt(((20 /*PD2  //pin 2 //TBD*/) == 2 ? 0 : ((20 /*PD2  //pin 2 //TBD*/) == 3 ? 1 : ((20 /*PD2  //pin 2 //TBD*/) >= 18 && (20 /*PD2  //pin 2 //TBD*/) <= 21 ? 23 - (20 /*PD2  //pin 2 //TBD*/) : -1))), left_isr, 3);
  // attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B), left_isr, RISING);

  attachInterrupt(((18 /*PC4  //pin A4 //TBD*/) == 2 ? 0 : ((18 /*PC4  //pin A4 //TBD*/) == 3 ? 1 : ((18 /*PC4  //pin A4 //TBD*/) >= 18 && (18 /*PC4  //pin A4 //TBD*/) <= 21 ? 23 - (18 /*PC4  //pin A4 //TBD*/) : -1))), right_isr, 3);
  // attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_B), right_isr, RISING);
  // pinMode(LEFT_ENC_INDEX, INPUT);
  // pinMode(RIGHT_ENC_INDEX, INPUT);



  Serial.begin(57600);

// Initialize the motor controller if used */




  initMotorController();
  resetPID();


/* Attach servos if used */
# 311 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = 
# 325 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3 4
                                  __null
# 325 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                                      ;
      else if (arg == 2) argv2[index] = 
# 326 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3 4
                                       __null
# 326 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                                           ;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[index] = 
# 335 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino" 3 4
                      __null
# 335 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino"
                          ;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

// If we are using base control, run a PID calculation at the appropriate intervals

  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // // Check to see if we have exceeded the auto-stop interval
  // if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
  //   setMotorSpeeds(0, 0);
  //   moving = 0;
  // }


// Sweep servos






}
# 1 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/encoder_driver.ino"
/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
# 31 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/encoder_driver.ino"
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;

  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  void left_isr(){
  int stateB = digitalRead(21 /*PD3  //pin 3 //TBD*/);

//   Determine the direction of rotation
  if (stateB == 1) {
    left_enc_pos--;
  } else {
    left_enc_pos++;
  }
}

  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  void right_isr(){
  // Read the state of the two encoder channels
  // int stateA = digitalRead(RIGHT_ENC_PIN_A);
  int stateB = digitalRead(19 /*PC5   //pin A5 //TBD*/);
  // delayMicroseconds(100);
  // digitalWrite(ISR, HIGH);
  //Determine the direction of rotation
  if (stateB == 1) {
    right_enc_pos++;

  } else {
    right_enc_pos--;
  }
  // digitalWrite(ISR, LOW);
}

  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == 0) {
      return left_enc_pos;

    } else {
      return right_enc_pos;
    }
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == 0){
      left_enc_pos=0L;
      return;
    } else {
      right_enc_pos=0L;
      return;
    }
  }




// ISR(TIMER1_COMPA_vect){
//   TCNT1  = 0;                  //First, set the timer back to 0 so it resets for next interrupt
//   LED_STATE = !LED_STATE;      //Invert LED state
//   Serial.print(readEncoder(LEFT));
//   Serial.print(" ");
//   Serial.println(readEncoder(RIGHT));
//   digitalWrite(13,LED_STATE);  //Write new state to the LED on pin D5
//   resetEncoders();
// }

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(0);
  resetEncoder(1);
}
# 1 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/motor_driver.ino"
/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/
# 59 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/motor_driver.ino"
  void initMotorController() {
    digitalWrite(13 /*TBD*/, 0x1);
    digitalWrite(13 /*TBD*/, 0x1);
  }

  void setMotorSpeed(int i, int spd) {// i = L/R motor
    unsigned char reverse = 0;

    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;

      if (i == 0) { //if left motor
        if (reverse == 0) { analogWrite(4, spd); digitalWrite(2, 0x0); }
        else if (reverse == 1) { analogWrite(4, spd); digitalWrite(2, 0x1); }
      }
      else /*if (i == RIGHT) //no need for condition*/ {
        if (reverse == 1) { analogWrite(8, spd); digitalWrite(5 /*changed from 3 for testing purposes*/, 0x0); }
        else if (reverse == 0) { analogWrite(8, spd); digitalWrite(5 /*changed from 3 for testing purposes*/, 0x1); }
      }
  }

  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(0, leftSpeed);
    setMotorSpeed(1, rightSpeed);
  }
# 1 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/servos.ino"
/***************************************************************
   Servo Sweep - by Nathaniel Gallinger

   Sweep servos one degree step at a time with a user defined
   delay in between steps.  Supports changing direction 
   mid-sweep.  Important for applications such as robotic arms
   where the stock servo speed is too fast for the strength
   of your system.

 *************************************************************/
