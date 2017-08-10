#ifndef DYNAMIXEL_MOTOR_H_
#define DYNAMIXEL_MOTOR_H_

#include <string>
#include <map>

typedef struct dynamixel_regs_s {
  uint16_t byte_length;
  bool write;
  int min;
  int max;
  std::string name;;
} dynamixel_regs_t;

typedef std::map<uint16_t, dynamixel_regs_t> dynamixel_regs_map;

class DynamixelMotor{
public:
  DynamixelMotor();
  virtual ~DynamixelMotor();
  
  void print_reg_list();
  int update_buf(int read, int reg, int val, uint8_t *p);

protected:
  const dynamixel_regs_map dynamixel_regs
  {
    //{0,   {1, false,  0,      0,      "Model Number"}},  
    {2,   {1, false,  0,      0,      "Version of Firmware"}},
    {3,   {1, true,   0,      252,    "ID"}},
    {4,   {1, true,   0,      252,    "Baud Rate"}},
    {5,   {1, true,   0,      254,    "Return Delay Time"}},
    {6,   {2, true,   0,      4095,   "CW Angle Limit"}},
    {8,   {2, true,   0,      4095,   "CCW Angle Limit"}},
    {11,  {1, true,   0,      99,     "The Highest Limit Temperature"}},
    {12,  {1, true,   50,     250,    "The Lowest Limit Voltage"}},
    {13,  {1, true,   50,     250,    "The Highest Limit Voltage"}},
    {14,  {2, true,   0,      1023,   "Max Torque"}},
    {17,  {1, true,   36,     36,     "Alarm LED"}},
    {18,  {1, true,   36,     36,     "Alarm Shutdown"}},
    {20,  {2, true,   -26624, 26624,  "Multi turn offset"}},
    {22,  {1, true,   1,      255,    "Resolution divider"}},
    {24,  {1, true,   0,      1,      "Torque Enable"}},
    {25,  {1, true,   0,      1,      "LED"}},
    {26,  {1, true,   0,      254,    "D Gain"}},
    {27,  {1, true,   0,      254,    "I Gain"}},
    {28,  {1, true,   0,      254,    "P Gain"}},
    {30,  {2, true,   -28672, 28672,  "Goal Position"}},
    {32,  {1, true,   0,      1023,   "Moving Speed"}},
    {34,  {1, true,   0,      1023,   "Torque Limit"}},
    {36,  {1, false,  0,      0,      "Preset Position"}},
    {38,  {1, false,  0,      0,      "Preset Speed"}},
    {40,  {1, false,  0,      0,      "Preset Load"}},
    {42,  {1, false,  0,      0,      "Preset Voltage"}},
    {43,  {1, false,  0,      0,      "Preset Temperature"}},
    {44,  {1, false,  0,      0,      "Registered Instruction"}},
    {46,  {1, false,  0,      0,      "Moving"}},
    {47,  {1, true,   0,      1,      "Lock"}},
    {48,  {1, true,   0,      1023,   "Punch"}},
    {73,  {1, true,   0,      254,    "Goal Acceleration"}},
  };

private:
  void Send_Byte_Mx106(uint8_t ID,uint8_t RW,uint8_t Address,uint8_t Data); //02-Read 03-Write
  void Send_Word_Mx106(uint8_t ID,uint8_t RW,uint8_t Address,uint8_t Low_Byte,uint8_t High_Byte); //02-Read 03-Write

  uint8_t buf_[16];
  uint8_t buf_size_;
};

#endif    // DYNAMIXEL_MOTOR_H_


