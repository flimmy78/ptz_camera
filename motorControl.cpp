#include <Arduino.h>
#include "motorControl.h"
#include "SPWM_SVPWM_Lookup_Table.h"

int currentStepA;
int currentStepB;
int currentStepC;
int potVal = 0;
int sineArraySize;
int increment = 0;
boolean direct = 1; // direction true=forward, false=backward

MotorControl::MotorControl(){
}

MotorControl::~MotorControl(){
}

void MotorControl::setup(){
  if (SerialDebug){
    Serial.println("Motor Setup...");
  }

#if 1
  TCCR1A = 1; // TCCR1A configuration: 00-Channel A disabled D9, 00-Channel B disabled D10, 00-Channel C disabled D11, 01-Fast PWM 8 bit (pg 131)
  TCCR1B = 0x09; // TCCR1B configuration: Clock mode and Fast PWM 8 bit (pg 133) & 62500 Hz frequency.....greater than audible sound
  TCCR1C = 0;  // TCCR1C configuration
  DDRB |= 0b11100000; // Set D9, D10 and D11 as output
  TCCR1A |= 0b10101000; // Activate D9, D10 and D11 PWM

  TCCR3B = TCCR3B & 0b11111001; // Configure D5 use PWM with 32kHz.....greater than audible sound
  
  TCCR4B = TCCR4B & 0b11111001; // Configure D6 and D13 to use PWM with 32kHz.....greater than audible sound
  
  pinMode(PWM5, OUTPUT);
  pinMode(PWM6, OUTPUT);
  pinMode(PWM13, OUTPUT);

  pinMode(EN1A, OUTPUT);
  pinMode(EN2A, OUTPUT); 
  pinMode(EN3A, OUTPUT); 

  digitalWrite(EN1A, HIGH);
  digitalWrite(EN2A, HIGH);
  digitalWrite(EN3A, HIGH);

  pinMode(EN1B, OUTPUT);
  pinMode(EN2B, OUTPUT); 
  pinMode(EN3B, OUTPUT); 

  digitalWrite(EN1B, HIGH);
  digitalWrite(EN2B, HIGH);
  digitalWrite(EN3B, HIGH);

  float phaseShift = FULL_CIRCLE_DEGREE / 3.0;         // Find phase shift and initial A, B C phase values
  currentStepA_[MOTORID_A] = 0.0;
  currentStepB_[MOTORID_A] = currentStepA_[0] + phaseShift;
  currentStepC_[MOTORID_A] = currentStepB_[0] + phaseShift;
  currentStepA_[MOTORID_B] = 0.0;
  currentStepB_[MOTORID_B] = currentStepA_[0] + phaseShift;
  currentStepC_[MOTORID_B] = currentStepB_[0] + phaseShift;

/*
  sineArraySize = sizeof(pwmSin)/sizeof(int); // Find lookup table size
  int phaseShift = sineArraySize / 3;         // Find phase shift and initial A, B C phase values
  currentStepA = 0;
  currentStepB = currentStepA + phaseShift;
  currentStepC = currentStepB + phaseShift;
*/
 
  sineArraySize--; // Convert from array Size to last PWM array number
#else
  TCCR1A = 1; // TCCR1A configuration: 00-Channel A disabled D9, 00-Channel B disabled D10, 00-Channel C disabled D11, 01-Fast PWM 8 bit (pg 131)
  TCCR1B = 0x09; // TCCR1B configuration: Clock mode and Fast PWM 8 bit (pg 133) & 62500 Hz frequency.....greater than audible sound
  TCCR1C = 0;  // TCCR1C configuration
  DDRB |= 0b11100000; // Set D9, D10 and D11 as output
  TCCR1A |= 0b10101000; // Activate D9, D10 and D11 PWM

  TCCR3B = TCCR3B & 0b11111001; // Configure D5 use PWM with 32kHz.....greater than audible sound
  
  TCCR4B = TCCR4B & 0b11111001; // Configure D6 and D13 to use PWM with 32kHz.....greater than audible sound
  
  pinMode(PWM5, OUTPUT);
  pinMode(PWM6, OUTPUT);
  pinMode(PWM13, OUTPUT);
  
  pinMode(EN1A, OUTPUT); 
  pinMode(EN2A, OUTPUT); 
  pinMode(EN3A, OUTPUT); 
  
  digitalWrite(EN1A, HIGH);
  digitalWrite(EN2A, HIGH);
  digitalWrite(EN3A, HIGH);

  pinMode(EN1B, OUTPUT); 
  pinMode(EN2B, OUTPUT); 
  pinMode(EN3B, OUTPUT); 
  
  digitalWrite(EN1B, HIGH);
  digitalWrite(EN2B, HIGH);
  digitalWrite(EN3B, HIGH);
   
  sineArraySize = sizeof(pwmSin)/sizeof(int); // Find lookup table size
  int phaseShift = sineArraySize / 3;         // Find phase shift and initial A, B C phase values
  currentStepA = 0;
  currentStepB = currentStepA + phaseShift;
  currentStepC = currentStepB + phaseShift;
 
  sineArraySize--; // Convert from array Size to last PWM array number
#endif

  if (SerialDebug){
    Serial.println("Motor Setup Done");
  }
}

void MotorControl::set_DigitalPin(bool enable){
  if (enable){    
    digitalWrite(EN1A, HIGH);
    digitalWrite(EN2A, HIGH);
    digitalWrite(EN3A, HIGH);   
    //
    digitalWrite(EN1B, HIGH);
    digitalWrite(EN2B, HIGH);
    digitalWrite(EN3B, HIGH);
  }
  else{
    digitalWrite(EN1A, LOW);
    digitalWrite(EN2A, LOW);
    digitalWrite(EN3A, LOW);
    //
    digitalWrite(EN1B, LOW);
    digitalWrite(EN2B, LOW);
    digitalWrite(EN3B, LOW);    
  }
}

void MotorControl::moveDelay(){
  // Control speed using Potentiometer - only for testing
/*
  potVal_ = analogRead(potPin);    
  potVal_ = map(potVal_, 0, 1023, 0, 10);
  delay(potVal_);
*/
}

void MotorControl::stepMove(int motor_id, int direction){
  if (direction == DIRECTION_FORWARD){
    angle_inc_ = 1;
  }
  else if (direction == DIRECTION_BACKWARD){
    angle_inc_ = -1;
  }

  if (motor_id == MOTORID_A){
    // For L6234A
    PWM9 = (int)(AMPLITUDE*sin(currentStepA_[motor_id]/RADIUS_TO_DEGREE)/2.0 + 127.0);
    PWM10 = (int)(AMPLITUDE*sin(currentStepB_[motor_id]/RADIUS_TO_DEGREE)/2.0 + 127.0);
    PWM11 = (int)(AMPLITUDE*sin(currentStepC_[motor_id]/RADIUS_TO_DEGREE)/2.0 + 127.0);
  }  
  else if (motor_id == MOTORID_B){
    // For L6234B
    analogWrite(PWM5, (int)(AMPLITUDE*sin(currentStepA_[motor_id]/RADIUS_TO_DEGREE)/2.0 + 127.0));
    analogWrite(PWM6, (int)(AMPLITUDE*sin(currentStepB_[motor_id]/RADIUS_TO_DEGREE)/2.0 + 127.0));
    analogWrite(PWM13, (int)(AMPLITUDE*sin(currentStepC_[motor_id]/RADIUS_TO_DEGREE)/2.0 + 127.0));
  }

  currentStepA_[motor_id] = currentStepA_[motor_id] + angle_inc_;
  currentStepB_[motor_id] = currentStepB_[motor_id] + angle_inc_;
  currentStepC_[motor_id] = currentStepC_[motor_id] + angle_inc_;

  //Check for lookup table overflow and return to opposite end if necessary
  if(currentStepA_[motor_id] >= FULL_CIRCLE_DEGREE)  currentStepA_[motor_id] = 0;
  if(currentStepA_[motor_id] < 0)  currentStepA_[motor_id] = FULL_CIRCLE_DEGREE;
 
  if(currentStepB_[motor_id] >= FULL_CIRCLE_DEGREE)  currentStepB_[motor_id] = 0;
  if(currentStepB_[motor_id] < 0)  currentStepB_[motor_id] = FULL_CIRCLE_DEGREE;
 
  if(currentStepC_[motor_id] >= FULL_CIRCLE_DEGREE)  currentStepC_[motor_id] = 0;
  if(currentStepC_[motor_id] < 0) currentStepC_[motor_id] = FULL_CIRCLE_DEGREE;

  if (0){
  //if (SerialDebug){
    Serial.print("motor_id: "); Serial.println(motor_id);
    Serial.print("stepA: "); Serial.println((int)(AMPLITUDE*sin(currentStepA_[motor_id]/RADIUS_TO_DEGREE)/2.0 + 127.0));
    Serial.print("stepB: "); Serial.println((int)(AMPLITUDE*sin(currentStepB_[motor_id]/RADIUS_TO_DEGREE)/2.0 + 127.0));
    Serial.print("stepC: "); Serial.println((int)(AMPLITUDE*sin(currentStepC_[motor_id]/RADIUS_TO_DEGREE)/2.0 + 127.0));
  }

  moveDelay();
}

float MotorControl::get_angle_inc_(){
  return angle_inc_;
}

void MotorControl::test_loop() {
#if 0
  // For L6234A
  PWM9 = pwmSin[currentStepA];
  PWM10 = pwmSin[currentStepB];
  PWM11 = pwmSin[currentStepC];  

  // For L6234B
  analogWrite(PWM5, pwmSin[currentStepA]);
  analogWrite(PWM6, pwmSin[currentStepB]);
  analogWrite(PWM13, pwmSin[currentStepC]);
   
  if (direct==true) increment = 1;
  else increment = -1;     
 
  currentStepA = currentStepA + increment;
  currentStepB = currentStepB + increment;
  currentStepC = currentStepC + increment;
 
  //Check for lookup table overflow and return to opposite end if necessary
  if(currentStepA > sineArraySize)  currentStepA = 0;
  if(currentStepA < 0)  currentStepA = sineArraySize;
 
  if(currentStepB > sineArraySize)  currentStepB = 0;
  if(currentStepB < 0)  currentStepB = sineArraySize;
 
  if(currentStepC > sineArraySize)  currentStepC = 0;
  if(currentStepC < 0) currentStepC = sineArraySize;
#else
  stepMove(MOTORID_A, DIRECTION_BACKWARD);
  stepMove(MOTORID_B, DIRECTION_BACKWARD);
#endif
  delay(50);
}