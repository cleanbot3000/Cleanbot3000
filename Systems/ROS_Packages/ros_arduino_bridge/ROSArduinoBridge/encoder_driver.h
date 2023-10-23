/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A 22 //PD2  //pin 2 //TBD
  #define LEFT_ENC_PIN_B 23 //PD3  //pin 3 //TBD
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A 26 //PC4  //pin A4 //TBD
  #define RIGHT_ENC_PIN_B 27 //PC5   //pin A5 //TBD
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

