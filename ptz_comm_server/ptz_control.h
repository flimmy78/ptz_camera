#ifndef PTZ_CONTROL_H_
#define PTZ_CONTROL_H_

#include <string>
#include <map>

typedef struct ptz_regs_s {
  std::string name;
} ptz_regs_t;

typedef std::map<uint16_t, ptz_regs_t> ptz_regs_map;

class PTZControl{
public:
  PTZControl();
  virtual ~PTZControl();

  void print_reg_list();

protected:
  const ptz_regs_map ptz_regs
  {
    {0,  {"REG_W_PITCH"}},  //target pitch
    {1,  {"REG_W_ROLL"}},   //target roll
    {2,  {"REG_R_PITCH"}},  //real time pitch 
    {3,  {"REG_R_ROLL"}},   //real time roll
    {4,  {"REG_PITCH_MOVE"}},
    {5,  {"REG_ROLL_MOVE"}},
    {6,  {"REG_MOVE"}},
    {7,  {"REG_TOLERANCE"}},   
  };

private:

};

#endif    // PTZ_CONTROL_H_


