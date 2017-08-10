#include <iostream>
#include <string.h>
#include "fcb_camera.h"

FCBCamera::FCBCamera(){
}

FCBCamera::~FCBCamera(){
}

int FCBCamera::update_buf(int read, int reg, int val, uint8_t *p){
  auto iter =camera_regs.find(reg);
  if (iter != camera_regs.end()){
    memcpy(p, iter->second.data, iter->second.byte_length);
    return iter->second.byte_length;
  }
  return 0;
}

void FCBCamera::print_reg_list(){
  std::cout << "----------" << std::endl;
  std::cout << "Camera registers:" << std::endl;
  for (auto iter =camera_regs.begin(); iter != camera_regs.end(); iter++){
    std::cout << iter->first << "," << iter->second.name << "," << (int)iter->second.byte_length << std::endl;
    for (int i=0; i<(int)iter->second.byte_length; i++){
      printf("0x%x,",iter->second.data[i]);
    }
    printf("\n");
  }
  std::cout << "----------" << std::endl;
}
