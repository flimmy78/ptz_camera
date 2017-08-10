#include "motionCamera.h"

MotionCamera MotionCamera_inst;

void setup()
{
  MotionCamera_inst.setup();
}

void loop()
{
  MotionCamera_inst.process();
}
