#ifndef PIN_MAPPING_H_

#define SerialDebug true  // Set to true to get Serial output for debugging

#define FULL_CIRCLE_DEGREE    360.0
#define RADIUS_TO_DEGREE      57.296  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
#define RADIUS_TO_DEGREE_NEG  -57.296  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians

#define LED 17

// Direct PWM change variables
#define PWM9    OCR1A // Uses Timer 1, D9 - To IN1 of L6234A
#define PWM10   OCR1B // Uses Timer 1, D10 - To IN2 of L6234A
#define PWM11   OCR1C // Uses Timer 1, D11 - To IN3 of L6234A

#define PWM5     5 // Uses Timer 3, D5 - To IN1 of L6234B
#define PWM6     6 // Uses Timer 4, D6 - To IN2 of L6234B
#define PWM13    13 // Uses Timer 4, D13 - To IN3 of L6234B

#define EN1A     4 // D4 - To EN1 of L6234A
#define EN2A     7 // D7 - To EN2 of L6234A
#define EN3A     8 // D8 - To EN3 of L6234A

#define EN1B     14 // D14 - To EN1 of L6234B
#define EN2B     15 // D15 - To EN2 of L6234B
#define EN3B     16 // D16 - To EN3 of L6234B

#define potPin  A0 // select the input pin for the potentiometer, D18

#endif  // PIN_MAPPING_H_
