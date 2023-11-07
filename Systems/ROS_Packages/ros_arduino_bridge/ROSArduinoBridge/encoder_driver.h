/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A 20 //PD2  //pin 2 //TBD
  #define LEFT_ENC_PIN_B 21 //PD3  //pin 3 //TBD
  
  // #define LEFT_ENC_INDEX 34
  // #define RIGHT_ENC_INDEX 13

  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A 18 //PC4  //pin A4 //TBD
  #define RIGHT_ENC_PIN_B 19 //PC5   //pin A5 //TBD
  // #define ISR 22
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
void left_isr();
void right_isr();

