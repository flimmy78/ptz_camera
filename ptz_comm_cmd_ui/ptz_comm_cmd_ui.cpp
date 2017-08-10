#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>


int main(int argc, char *argv[])
{
  int sockfd = 0;
  struct sockaddr_in serv_addr;
  uint8_t sendBuff[8];
  uint8_t recvBuff[64];
  int opt_read;

  if (argc > 2){
    opt_read = strtoul(argv[2], NULL, 0);
    if (opt_read){
      if(argc < 5)
      {
        printf("\n Usage: %s <ip of server> <w-0/r-1> <device: 0-PTZ, 1-Pan Motor, 2-Camera> <reg>\n",argv[0]);
        return -1;
      }      
    }
    else{
      if(argc < 6)
      {
        printf("\n Usage: %s <ip of server> <w-0/r-1> <device: 0-PTZ, 1-Pan Motor, 2-Camera> <reg> <val>\n",argv[0]);
        return -1;
      }
    }
  }
  else{
    printf("\n Usage: %s <ip of server> <w-0/r-1> <device: 0-PTZ, 1-Pan Motor, 2-Camera> <reg> <val>\n",argv[0]);
    printf("\n Example: %s 192.168.1.5 0 1 0 0\n",argv[0]);
    return -1;
  }

  if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    printf("\n Error : Could not create socket \n");
    return -1;
  }

  int opt = true;
  if( setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt)) < 0 )
  {
    perror("setsockopt");
    if (sockfd >= 0){
      close(sockfd);
      sockfd = -1;
    }
    return -1;
  }

  memset(&serv_addr, '0', sizeof(serv_addr)); 

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(10002); 

  if(inet_pton(AF_INET, argv[1], &serv_addr.sin_addr)<=0)
  {
    printf("\n inet_pton error occured\n");
    return -1;
  } 

  if(connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
  {
    printf("\n Error : Connect Failed \n");
    return -1;
  }

  opt_read = strtoul(argv[2], NULL, 0);
  sendBuff[0] = opt_read;

  int device = strtoul(argv[3], NULL, 0);
  sendBuff[1] = device;

  int reg = strtoul(argv[4], NULL, 0);
  sendBuff[2] = reg;

  unsigned long val = strtoul(argv[5], NULL, 0);
  sendBuff[3] = (val>>8)&0xFF;
  sendBuff[4] = val&0xFF;
  
  write(sockfd, sendBuff, 5);
  
  if (opt_read){
    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(sockfd, &rfds);
    
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    int retval = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);
    if (retval){
      int n = recv(sockfd, recvBuff, sizeof(recvBuff), 0);
      if (n > 0)
      {
        for (uint16_t i=0; i<n; i++){
          printf("0x%x,",recvBuff[i]);
        }
        printf("\n");
      }
    }
  }
  
  close(sockfd);

  return 0;
}