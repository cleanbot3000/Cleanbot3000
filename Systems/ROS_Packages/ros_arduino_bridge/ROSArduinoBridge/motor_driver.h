/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef BLD120A
  #define RIGHT_MOTOR_SPEED 8
  #define LEFT_MOTOR_SPEED  4
  #define RIGHT_MOTOR_DIR 3
  #define LEFT_MOTOR_DIR 2
  #define RIGHT_MOTOR_ENABLE 13 //TBD
  #define LEFT_MOTOR_ENABLE 13  //TBD
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
