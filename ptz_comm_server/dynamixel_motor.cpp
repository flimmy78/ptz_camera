#include <iostream>
#include <string.h>
#include "dynamixel_motor.h"

DynamixelMotor::DynamixelMotor(){
}

DynamixelMotor::~DynamixelMotor(){
}

void DynamixelMotor::Send_Byte_Mx106(uint8_t ID,uint8_t RW,uint8_t Address,uint8_t Data)//02-Read 03-Write
{
	uint8_t CheckSum,i;

	CheckSum=0;
  memset(buf_,0,sizeof(buf_));

	buf_[0]=0xff;
	buf_[1]=0xff;
	buf_[2]=ID;
	buf_[3]=0x04;
	buf_[4]=RW;
	buf_[5]=Address;
	buf_[6]=Data;
	for(i=2;i<=6;i++){
		CheckSum+=buf_[i];
	}
	buf_[7]=0x00ff&(~CheckSum);
  buf_size_ = 8;
}

void DynamixelMotor::Send_Word_Mx106(uint8_t ID,uint8_t RW,uint8_t Address,uint8_t Low_Byte,uint8_t High_Byte)//02-Read 03-Write
{
	uint8_t CheckSum,i;

	CheckSum=0;
  memset(buf_,0,sizeof(buf_));

	buf_[0]=0xff;
	buf_[1]=0xff;
	buf_[2]=ID;
	buf_[3]=0x05;
	buf_[4]=RW;
	buf_[5]=Address;
	buf_[6]=Low_Byte;
	buf_[7]=High_Byte;
	for(i=2;i<=7;i++){
		CheckSum+=buf_[i];
	}
	buf_[8]=0x00ff&(~CheckSum);
  buf_size_ = 9;
}

int DynamixelMotor::update_buf(int read, int reg, int val, uint8_t *p){
  auto iter =dynamixel_regs.find(reg);
  if (iter != dynamixel_regs.end()){
    if ((iter->second.write==false) && (read==0)){
      printf("not writable\n");
      return 0;
    }
    if ((read==0) && (val < iter->second.min || val > iter->second.max)){
      printf("value over range\n");
      return 0;
    }
    if (iter->second.byte_length == 1){
      Send_Byte_Mx106(0x01, read?0x02:0x03, reg, val);
    }
    else if (iter->second.byte_length == 2){
      Send_Word_Mx106(0x01, read?0x02:0x03, reg, val&0xFF, (val>>8)&0xFF);
    } 
    memcpy(p, buf_, buf_size_);
    return buf_size_;
  }
  return 0;
}

void DynamixelMotor::print_reg_list(){
  std::cout << "----------" << std::endl;
  std::cout << "PAN motor registers:" << std::endl;
  for (auto iter =dynamixel_regs.begin(); iter != dynamixel_regs.end(); iter++){
    std::cout << iter->first << "," << iter->second.name << "," << (int)iter->second.byte_length << "," 
      << (int)iter->second.write << "," << (int)iter->second.min << "," << (int)iter->second.max << std::endl;
  }
  std::cout << "----------" << std::endl;
}