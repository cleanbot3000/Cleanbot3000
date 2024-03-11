#line 1 "/home/zelbot/ws/src/ros_arduino_bridge/ROSArduinoBridge/motor_driver.h"
/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef BLD120A
  #define RIGHT_MOTOR_SPEED 8
  #define LEFT_MOTOR_SPEED  4
  #define RIGHT_MOTOR_DIR 5 //changed from 3 for testing purposes
  #define LEFT_MOTOR_DIR 2
  #define RIGHT_MOTOR_ENABLE 13 //TBD
  #define LEFT_MOTOR_ENABLE 13  //TBD
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
