/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
  volatile long left_enc_pos = 0;
  volatile long right_enc_pos = 0;
    
  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  void left_isr(){
  // Read the state of the two encoder channels
  int stateA = digitalRead(LEFT_ENC_PIN_A);
  int stateB = digitalRead(LEFT_ENC_PIN_B);

//   Determine the direction of rotation
  if (stateA != stateB) {
    left_enc_pos++;
  } else {
    left_enc_pos--;
  }
}
  
  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  void right_isr(){
  // Read the state of the two encoder channels
  // int stateA = digitalRead(RIGHT_ENC_PIN_A);
  int stateB = digitalRead(RIGHT_ENC_PIN_B);

//   Determine the direction of rotation
  if (stateB != 1) {
    right_enc_pos--;
  } else {
    right_enc_pos++;
  }
}
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) {
      return left_enc_pos;
      
    } else {
      return right_enc_pos; 
    }


  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif
