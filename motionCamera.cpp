#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "motionCamera.h"

MotorControl motor_ctrl_;

int i2c_buf_[4];
int i2c_pos_ = 0;
int reg_pitch_move_f_ = 0;
int reg_pitch_move_b_ = 0;
int reg_pitch_move_s_ = 0;
int reg_roll_move_f_ = 0;
int reg_roll_move_b_ = 0;
int reg_roll_move_s_ = 0;

void receiveEvent(int howMany) {
  i2c_pos_ = 0;
  while (Wire.available() > 0) {
    i2c_buf_[i2c_pos_] = Wire.read();
    Serial.println(i2c_buf_[i2c_pos_]);
    i2c_pos_++;
  }
  delay(100);
}
  
/*
// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
byte y = 0;
void requestEvent() {
  Wire.write("okok"); // respond with message of 6 bytes // as expected by master
  Wire.write(y);
  Wire.write("\n");
  y = (y+1)%10;
}
*/

MotionCamera::MotionCamera(){
}

MotionCamera::~MotionCamera(){
}

void MotionCamera::setup(){  
  Wire.begin(8);                // join i2c bus with address #8
  TWBR = 12;          // 400 kbit/sec I2C speed
  //Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent);

  Serial.begin(115200);
  Serial1.begin(9600);
  delay(2000);  // wait till serial ready

  motor_ctrl_.setup();
}

void MotionCamera::process(){
  float new_pitch, new_roll;
  Serial.println("program ready");

  int pid_test = 0;
  int test_debug = 0;
  while (1){
    if (Serial1.available()) {
      int x = Serial1.read();
      Serial.println(x);
    }
  
    motor_ctrl_.test_loop();
#if 0
    delay(100);
    if (i2c_pos_ >= 4){
      if (i2c_buf_[0] == 0){   // only deal with write now
        switch(i2c_buf_[1]){
          case REG_PITCH_MOVE_F:
            reg_pitch_move_f_ = (i2c_buf_[2]<<8) | i2c_buf_[3];
            break;
          case REG_PITCH_MOVE_B:
            reg_pitch_move_b_ = (i2c_buf_[2]<<8) | i2c_buf_[3];
            break;
          case REG_PITCH_MOVE_S:
            reg_pitch_move_s_ = (i2c_buf_[2]<<8) | i2c_buf_[3];
            break;
          case REG_ROLL_MOVE_F:
            reg_roll_move_f_ = (i2c_buf_[2]<<8) | i2c_buf_[3];
            break;
          case REG_ROLL_MOVE_B:
            reg_roll_move_b_ = (i2c_buf_[2]<<8) | i2c_buf_[3];
            break;
          case REG_ROLL_MOVE_S:
            reg_roll_move_s_ = (i2c_buf_[2]<<8) | i2c_buf_[3];
            break;
          default:
            break;
        }
      }
    }
    i2c_pos_ = 0;

    if (reg_pitch_move_f_){
      Serial.println("reg_pitch_move_f_ ");
      for (int i=0; i<5; i++){
        motor_ctrl_.stepMove(motor_ctrl_.MOTORID_A, motor_ctrl_.DIRECTION_FORWARD);
      }
      reg_pitch_move_f_ = 0;
    }
    else if (reg_pitch_move_b_){
      Serial.println("reg_pitch_move_b_");
      for (int i=0; i<5; i++){
        motor_ctrl_.stepMove(motor_ctrl_.MOTORID_A, motor_ctrl_.DIRECTION_BACKWARD);
      }
      reg_pitch_move_b_ = 0;
    }
    else if (reg_pitch_move_s_){
      Serial.println("reg_pitch_move_s_");
      reg_pitch_move_f_ = 0;
      reg_pitch_move_b_ = 0;
      reg_pitch_move_s_ = 0;
    }
    else if (reg_roll_move_f_){
      Serial.println("reg_roll_move_f_");
      for (int i=0; i<5; i++){
        motor_ctrl_.stepMove(motor_ctrl_.MOTORID_A, motor_ctrl_.DIRECTION_FORWARD);
      }
      else{
        reg_roll_move_f_ = 0;
      }
    }
    else if (reg_roll_move_b_){
      Serial.println("reg_roll_move_b_");
      for (int i=0; i<5; i++){
        motor_ctrl_.stepMove(motor_ctrl_.MOTORID_A, motor_ctrl_.DIRECTION_BACKWARD);
      }
      else{
        reg_roll_move_b_ = 0;
      }
    }
    else if (reg_roll_move_s_){
      Serial.println("reg_roll_move_s_");
      reg_roll_move_f_ = 0;
      reg_roll_move_b_ = 0;
      reg_roll_move_s_ = 0;
    }
#endif    
  }
}
