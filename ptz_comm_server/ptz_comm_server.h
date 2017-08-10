#ifndef PTZ_COMM_SERVER_H_
#define PTZ_COMM_SERVER_H_

#include "ptz_control.h"
#include "dynamixel_motor.h"
#include "fcb_camera.h"
#include "crc.h"

#define PORT_CTRL       10001
#define PORT_CMD_UTIL   10002

#define MAX_RECV_BUF_SIZE   1024


class PTZCommServer{
public:
  enum DEVICE_ID {
    DEVICE_ID_PTZ = 0,
    DEVICE_ID_PANMOTOR,
    DEVICE_ID_CAMERA,
  };

  static PTZCommServer& getInstance(void);
  int ctrl_init(char *server_ip);
  int cmd_ui_init();
  
  int cmd_ui_process();
  int ctrl_process();
  void get_reg();
  void set_reg();

  int get_ctrl_socket();

  PTZControl ptz_;
  DynamixelMotor pan_motor_;
  FCBCamera camera_;

protected:
  CRC crc_;

private:
  PTZCommServer();
  virtual ~PTZCommServer();
  int cmd_ui_socket_ = -1;
  int ctrl_socket_ = -1;
  struct sockaddr_in serv_addr_;
  uint16_t recv_pos_ = 0;
  uint8_t recv_buf_[MAX_RECV_BUF_SIZE];
  struct sockaddr_in cmd_server_, cmd_client_;
};

#define PTZCommUtil_INST    \
	PTZCommServer::getInstance()

#endif    // PTZ_COMM_SERVER_H_


