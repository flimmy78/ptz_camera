#ifndef MOTOR_CONTROL_H_

#include "pinMapping.h"

// analogWrite() function configures PWM11 with Timer0. However, Timer0 is also used for 
// time keeping functions like millis(), delay() etc., therefore changing Timer0 settings will
// change these functions, rendering them unusable as original. PWM11 internally can also  
// be configured using Timer1, along with PWM9 and PWM10. PWM values will then be written directly  
// to their respective OCRnA/B/C registers. Therefore, this sketch does that below.
// Refer: http://r6500.blogspot.ca/2014/12/fast-pwm-on-arduino-leonardo.html

// Same can be done for PWM5, PWM6 and PWM13. However, to use them to drive one motor,
// their frequency need to be the same and since they are configured using timers that use 
// different master clocks, it is not convenient. Using analogWrite() function however allows them 
// to have have same frequency, therefore, this sketch uses it.

// After setting the Timers, PWM values are derived by sinusoidal pulse width modulation (SPWM) 
// and space vector pulse width modulation (SVPWM). Either can be used.
// Refer: http://www.berryjam.eu/2015/04/driving-bldc-gimbals-at-super-slow-speeds-with-arduino/

#define AMPLITUDE    255.0

class MotorControl
{
public:
  enum MOTORID {
    MOTORID_A = 0,
    MOTORID_B,
  };

  enum DIRECTION {
    DIRECTION_FORWARD = 0,
    DIRECTION_BACKWARD,
  };

  MotorControl();
  virtual ~MotorControl();

  void setup();
  void stepMove(int motor_id, int direction);

  float get_angle_inc_();
  void set_DigitalPin(bool enable);
  
  void test_loop();

protected:
  void moveDelay();

private:
  // [0] - A, [1] - B
  float currentStepA_[2];
  float currentStepB_[2];
  float currentStepC_[2];

  int potVal_ = 0;     // variable to store the value coming from the potentiometer
  int sineArraySize_;

  float angle_inc_;
};

#endif  // MOTOR_CONTROL_H_
