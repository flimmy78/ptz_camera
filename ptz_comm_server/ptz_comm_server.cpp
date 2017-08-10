#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <pthread.h>
#include "ptz_comm_server.h"

static pthread_t thread_cmd_ui;
static pthread_t thread_ctrl;

PTZCommServer& PTZCommServer::getInstance(void)
{
	static PTZCommServer EthCommUtil_instance;
	return EthCommUtil_instance;
}

PTZCommServer::PTZCommServer(){
}

PTZCommServer::~PTZCommServer(){
  if (ctrl_socket_>=0){
    close(ctrl_socket_);
  }
  if (cmd_ui_socket_>=0){
    close(cmd_ui_socket_);
  }
} 

int PTZCommServer::ctrl_init(char *server_ip){
  if((ctrl_socket_ = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
      printf("\n Error : Could not create socket \n");
      return -1;
  }

  int opt = true;
  if( setsockopt(ctrl_socket_, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt)) < 0 )
  {
    perror("setsockopt");
    if (ctrl_socket_ >= 0){
      close(ctrl_socket_);
      ctrl_socket_ = -1;
    }
    return -1;
  }

  memset(&serv_addr_, 0, sizeof(serv_addr_)); 
  serv_addr_.sin_family = AF_INET;
  serv_addr_.sin_port = htons(PORT_CTRL);
  if(inet_pton(AF_INET, server_ip, &serv_addr_.sin_addr)<=0)
  {
      printf("\n inet_pton error occured\n");
      return -1;
  } 

  if( connect(ctrl_socket_, (struct sockaddr *)&serv_addr_, sizeof(serv_addr_)) < 0)
  {
     printf("\n Error : Connect Failed \n");
     return -1;
  }

  return 0;
}

int PTZCommServer::cmd_ui_init(){
  cmd_ui_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (cmd_ui_socket_ < 0) {
    printf("create cmd_ui_socket_ failed, %s\n", strerror(errno));
    if (cmd_ui_socket_ >= 0){
      close(cmd_ui_socket_);
      cmd_ui_socket_ = -1;
    }
    return -1;
  }

  int opt = true;
  if( setsockopt(cmd_ui_socket_, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt)) < 0 )
  {
    perror("setsockopt");
    if (cmd_ui_socket_ >= 0){
      close(cmd_ui_socket_);
      cmd_ui_socket_ = -1;
    }
    return -1;
  }

  memset(&cmd_server_, 0, sizeof(cmd_server_));
  cmd_server_.sin_family = AF_INET;
  cmd_server_.sin_addr.s_addr = INADDR_ANY;
  cmd_server_.sin_port = htons(PORT_CMD_UTIL);  

  if (bind(cmd_ui_socket_, (struct sockaddr *)&cmd_server_, sizeof(cmd_server_))<0) 
  {
    perror("bind failed");
    if (cmd_ui_socket_ >= 0){
      close(cmd_ui_socket_);
      cmd_ui_socket_ = -1;
    }
    return -1;
  }
}

int PTZCommServer::ctrl_process(){
  fd_set fdSet;
  int ret, msglen;
  uint8_t buffer[64];
  struct timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  int old_recv_pos;
  while (1) {
    memset(buffer,0,sizeof(buffer));
    FD_ZERO(&fdSet);
    FD_SET(ctrl_socket_, &fdSet);
    ret = select(FD_SETSIZE, &fdSet, NULL, NULL, &tv);
    if (ret < 0) {
      if (errno == EINTR)
          continue;
      printf("select exit, %s\n", strerror(errno));
      if (ctrl_socket_ >= 0){
        close(ctrl_socket_);
        ctrl_socket_ = -1;
      }        
      return -1;
    }

    if (FD_ISSET(ctrl_socket_, &fdSet)) {
      msglen = recv(ctrl_socket_, buffer, sizeof(buffer), MSG_DONTWAIT);
      if (msglen > 0){
        printf("recv: ");
        for (uint16_t i=0; i<msglen;i++){
          printf("0x%x,",buffer[i]);
        }
        printf("\n");
        memcpy(recv_buf_ + recv_pos_, buffer, msglen);
        recv_pos_ = (recv_pos_ + msglen) % MAX_RECV_BUF_SIZE;
      } 
      else{
        //printf("read failed, %s\n", strerror(errno));
      }
    }
    
    if (old_recv_pos != recv_pos_){
      old_recv_pos = recv_pos_;
    }
    else if (recv_pos_){
      sendto(cmd_ui_socket_, recv_buf_, recv_pos_, MSG_DONTWAIT, (struct sockaddr*)&cmd_client_, sizeof(cmd_client_));
      recv_pos_ = 0;
      memset(recv_buf_,0,sizeof(recv_buf_));
    }
  }

  return 0;
}

int PTZCommServer::cmd_ui_process(){
  fd_set fdSet;
  int ret, msglen;
  uint8_t buffer[64];
  int slen = sizeof(cmd_client_);
  while (1) {
    memset(buffer,0,sizeof(buffer));
    FD_ZERO(&fdSet);
    FD_SET(cmd_ui_socket_, &fdSet);
    ret = select(FD_SETSIZE, &fdSet, NULL, NULL, NULL);
    if (ret < 0) {
      if (errno == EINTR)
          continue;
      printf("select exit, %s\n", strerror(errno));
      if (cmd_ui_socket_ >= 0){
        close(cmd_ui_socket_);
        cmd_ui_socket_ = -1;
      }        
      return -1;
    }

    if (FD_ISSET(cmd_ui_socket_, &fdSet)) {
      msglen = recvfrom(cmd_ui_socket_, buffer, sizeof(buffer), MSG_DONTWAIT, (struct sockaddr*) &cmd_client_, &slen);
      if (msglen > 0){
        int read = buffer[0];
        int device = buffer[1];
        int reg = buffer[2];
        unsigned long val = (buffer[3]<<8)|buffer[4];
        printf("cmd_ui: 0x%x 0x%x 0x%x 0x%x,%d\n", read, device, reg, val, (int16_t)val);
        
        memset(buffer,0,sizeof(buffer));
        switch(device){
          case DEVICE_ID_PTZ:
            buffer[0] = 0xA0;
            buffer[1] = 0xA1;
            buffer[2] = 0xA2;
            buffer[3] = 0xA3;
            if (reg<0 || reg>15){
              msglen = 0;
            }
            else{
              buffer[4] = read;
              buffer[5] = reg;
              buffer[6] = (val>>8) & 0xFF;
              buffer[7] = val & 0xFF;
              msglen = 4;
            }
            break;
          case DEVICE_ID_PANMOTOR:
            buffer[0] = 0xB0;
            buffer[1] = 0xB1;
            buffer[2] = 0xB2;
            buffer[3] = 0xB3;
            msglen = PTZCommUtil_INST.pan_motor_.update_buf(read, reg, (int16_t)val, buffer+4);           
            break;
          case DEVICE_ID_CAMERA:
            buffer[0] = 0xC0;
            buffer[1] = 0xC1;
            buffer[2] = 0xC2;
            buffer[3] = 0xC3;
            msglen = PTZCommUtil_INST.camera_.update_buf(read, reg, (int16_t)val, buffer+4);           
            break;
          default:
            return -1;
        }

        if (msglen == 0){
          continue;
        }
        msglen += 4;
        buffer[msglen] = 0xAA;
        msglen++;
        for (int i=0; i<msglen; i++){
          printf("0x%x,", buffer[i]);
        }
        printf("\n");

        if (recv_pos_){
          sendto(cmd_ui_socket_, recv_buf_, recv_pos_, MSG_DONTWAIT, (struct sockaddr*)&cmd_client_, sizeof(cmd_client_));
          recv_pos_ = 0;
          memset(recv_buf_,0,sizeof(recv_buf_));
        }

        write(ctrl_socket_, buffer, msglen);
      } 
      else{
        //printf("read failed, %s\n", strerror(errno));
      }
    }    
  }

  return 0;	
}

void *thread_cmd_ui_func(void *ptr)
{
  char *message;
  message = (char *) ptr;
  printf("%s \n", message);
  std::string str;

  PTZCommUtil_INST.cmd_ui_process();
}

void *thread_ctrl_func(void *ptr)
{
  char *message;
  message = (char *) ptr;
  printf("%s \n", message);
  std::string str;

  PTZCommUtil_INST.ctrl_process();
}

int PTZCommServer::get_ctrl_socket(){
  return ctrl_socket_;
}

int main(int argc, char *argv[])
{
  int ret = 0;
  if (PTZCommUtil_INST.ctrl_init(argv[1]) == 0){
    const char *message = "ctrl thread";
    ret = pthread_create(&thread_ctrl, NULL, thread_ctrl_func, (void*)message);
    if(ret)
    {
      fprintf(stderr,"ctrl process Error: %d\n",ret);
      return -1;
    }
  }
  else {
    printf("ctrl process initialization failed\n");
    return -1;
  }
  if (PTZCommUtil_INST.cmd_ui_init() == 0){
    const char *message = "cmd ui thread";
    ret = pthread_create(&thread_cmd_ui, NULL, thread_cmd_ui_func, (void*)message);
    if(ret)
    {
      fprintf(stderr,"cmd ui process Error: %d\n",ret);
      return -1;
    }
  }
  else{
    printf("cmd ui process initialization failed\n");
    return -1;
  }
  
  sleep(2);

  PTZCommUtil_INST.ptz_.print_reg_list();
  PTZCommUtil_INST.pan_motor_.print_reg_list();
  PTZCommUtil_INST.camera_.print_reg_list();
  
  uint8_t buf_cw_limit[] = {0xb0,0xb1,0xb2,0xb3,0xff,0xff,0x1,0x5,0x3,0x6,0xff,0xf,0xe2,0xaa};
  write(PTZCommUtil_INST.get_ctrl_socket(), buf_cw_limit, sizeof(buf_cw_limit));

  while (1){
    sleep(1);
  }

  return 0;
}