#include <iostream>
#include "ptz_control.h"

PTZControl::PTZControl(){
}

PTZControl::~PTZControl(){
}

void PTZControl::print_reg_list(){
  std::cout << "----------" << std::endl;
  std::cout << "PTZ registers:" << std::endl;
  for (auto iter = ptz_regs.begin(); iter != ptz_regs.end(); iter++){
    std::cout << iter->first << "," << iter->second.name << std::endl;
  }
  std::cout << "----------" << std::endl;
}
