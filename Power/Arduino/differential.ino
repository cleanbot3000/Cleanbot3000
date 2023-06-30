/*
cleanbot demo code 1.0
this code does not change any of the output
frequencies of the pins
this code does not integrate/use the lcd keypad
attachment
the goal of the code below is to demonstarte
the control of two motors through cytron hbridge
the intended output (or coded behavior of robot)
is to:
 drive forward
 pause
 drive backward
 pause
 spin right
 pause
 spin left
 pause
 turn right
 pause
 turn left
 pause
 then repeat
*/
// Pin for analog keypad
#define diy_pwm A2
// PWM output pin --for motor1
#define pwm1 4
// Motor direction pin --for motor1
#define dir1 2
// PWM output pin --for motor2
#define pwm2 8
// Motor direction pin --for motor2
#define dir2 3
void setup() {
 // put your setup code here, to run once:
 // Define Pins - setting pins to output
 pinMode(pwm1,OUTPUT);
 pinMode(dir1,OUTPUT);
 pinMode(pwm2,OUTPUT);
 pinMode(dir2,OUTPUT);
}
void loop() {
 // code inside "void loop" runs repeatedly
 //assuming digital write for forward direction
 // robot drives forward
 analogWrite(pwm1,10);
 analogWrite(pwm2,10);
 delay(1000); //delay
 //change direction of both motors
 digitalWrite(dir1,!digitalRead(dir1));
 digitalWrite(dir2,!digitalRead(dir2));
 delay(200);
//robot drives backward
 analogWrite(pwm1,10);
 analogWrite(pwm2,10);
 delay(2000);
 //change direction of motor1
 digitalWrite(dir1,!digitalRead(dir1));
 delay(200);
 //robot spins left
 analogWrite(pwm1,10);
 analogWrite(pwm2,10);
 delay(3000);
 //change direction of both motors
 digitalWrite(dir1,!digitalRead(dir1));
 digitalWrite(dir2,!digitalRead(dir2));
 delay(200);
 //robot spins right
 analogWrite(pwm1,250);
 analogWrite(pwm2,250);
 delay(3000);

 //make direction of both motors forward
 //by changing direction of motor2 back toforward
 digitalWrite(dir2,!digitalRead(dir2));
 //or digitalWrite(dir2, __) .. unknown int val touse
 delay(200);
 //robot turns left : motor1 (left wheel) set tolower speed
 analogWrite(pwm1,1);
 analogWrite(pwm2,10);
 delay(3000);
 //robot turns right : motor2 (right wheel) set tolower speed
 analogWrite(pwm1,10);
 analogWrite(pwm2,1);
 delay(3000);

}