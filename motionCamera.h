#ifndef MOTION_CAMERA_H_

#include "motorControl.h"

#define REG_PITCH_MOVE_F     0
#define REG_PITCH_MOVE_B     1
#define REG_PITCH_MOVE_S     2
#define REG_ROLL_MOVE_F      3
#define REG_ROLL_MOVE_B      4
#define REG_ROLL_MOVE_S      5

class MotionCamera{
public:
  MotionCamera();
  virtual ~MotionCamera();
  void setup();
  void process();

protected:

private:
  
};

#endif  // MOTION_CAMERA_H_
