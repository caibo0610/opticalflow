#ifndef _GDUUDPCLIENT_H__
#define _GDUUDPCLIENT_H__

#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/ip.h>
#include <stdbool.h>

#include "common.h"

extern "C"
{
    #include "udp.h"
}


//#define SOCKET_RCV_GCS_wifi_PORT        7000 //7080 release

//#define MAX_MSG_BUFSIZE                 1024

#define MAX_LEN     32
#define SER_PORT	7088  //cbc : 7082
#define SER_ADDR	"192.168.1.1"

/************变量**************/
//int socketfd_local = -1;

//struct sockaddr_in client_addr;
//struct sockaddr_in server_addr;

//static void print_hex(char *buf, int len);
struct cbc_position_data
{
        int16_t  cbc_x;
        int16_t  cbc_y;
        int16_t  cbc_z;
};

class gduUdpClient{

    public:
        gduUdpClient();
        ~gduUdpClient();
        
        int  initalParameter();
        int  createThread();
        int  udpSendData(struct impr_exchange_data_cbc  send_data);
        int  udpReciveData(struct impr_exchange_data_cbc  &receive_data);
        int  udpfilterByKey(struct impr_exchange_data_cbc input_data, struct impr_exchange_data_cbc &output_data);
        int  udpGetAndSendAppData(struct impr_ce_analysis_data analysised_data);
        int  udpTakeOff();
        int  udpLand();
        int  udpClose();
        int  udpSetPosition(int16_t x,int16_t y,int16_t z);
        int  udpGetPosition(struct cbc_position_data &xyz_data);
        int  udpSetAppType(uint8_t type);
        uint8_t  udpGetAPPType();

        int gduSetUAV();

        
    private:
        struct sockaddr_in m_server_addr;
	    int m_sockfd ;
        
        uint8_t  m_app_al_type ;//0x01===开始 0x02 == 停止 0x03 === 一键降落
        uint8_t  m_pre_SerialNum ;
        struct cbc_position_data m_cbc_pose_data;
        uint8_t  m_initalFlage ;

};



#endif
