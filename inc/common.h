#ifndef _COMMON_H_
#define _COMMON_H_

#include <string.h>
#include <time.h> 
#include <vector>
#include <iostream>
#include <unistd.h>
//#include "log.h"
//#include "gdu_udp_client.h"

#define SAVEVIDEO

#define RUN     1  
#define STOP    0 

#define SUCCESS				0
#define FAILED				-1

#define GDU_PROTOCOL_HB_TO_CE_ADDR  			    0x7d  //APP -> CBC
#define GDU_PROTOCOL_CE_TO_AHB_ACK_ADDR  			0xd7  //CBC -> APP
#define GDU_PROTOCOL_CBC_TO_FC_ADDR					0x72  //cbc->fc
#define GDU_PROTOCOL_FC_TO_CBC_ADDR					0x27  //fc->cbc



#define GDU_PROTOCOL_CE_CMD_KEY_CBC  0xa5			//hb data
#define GDU_PROTOCOL_APP_CMD_KEY_CBC  0xa4         //app control data
#define GDU_PROTOCOL_TAKEOFF_CMD_KEY_CBC  0x07    //一键起飞
#define GDU_PROTOCOL_LAND_CMD_KEY_CBC     0x09    //垂直降落

#define GDU_CBC_DATA_TO_HEARTBEAT_LEN      17
#define GDU_CBC_DATA_TO_HEARTBEAT_ACK_LEN  17
#define GDU_CBC_APP_TO_CBC_LEN   			8
#define GDU_CBC_APP_TO_CBC_ACK_LEN   		8
#define GDU_CBC_TAKEOFF_TO_FC_LEN			7
#define GDU_CBC_TAKEOFF_TO_CBC_ACK_LEN		8
#define GDU_CBC_LAND_TO_FC_LEN				7
#define GDU_CBC_LAND_TO_CBC_ACK_LEN			8



#define GDU_PROTOCOL_CMD_ADDR_POS                             2     //��ַ�ǵڶ�λ
#define GDU_PROTOCOL_CMD_KEY_POS                              3     //�������ǵ���λ

//ack value
#define ack_aulve_OK 0x00 //
#define ack_aulve_STORAGE_FULL 0X02
#define ack_aulve_INPRO 0x03
#define ack_aulve_NOT_OPEN_CAMERA 0X04
#define ack_aulve_NOT_REC 0X05

struct impr_exchange_data_cbc{
              //data and size
        uint8_t      size;
	    uint8_t     *data;
};

struct impr_fc_data{
	uint8_t     serial_num;
};

struct impr_fc_ack_data{
	uint8_t     serial_num;
	uint8_t     ack_value;
};

struct impr_app_data{
    uint8_t     serial_num;
	uint8_t		al_type;

};

struct impr_app_data_ack{
    uint8_t     ack_value;
	uint8_t     serial_num;
};

struct impr_cb_cbc_hb_data{
	uint8_t ack_value;
	uint8_t serial_num;
	int16_t	 x;//dang
	int16_t	 y;
	int16_t	 z;
};

struct impr_cb_comp_data_hb_ack{
    uint8_t ack_value;
    uint8_t serial_num;
};

typedef enum {
	IMPR_DT_COMP_NONE = 1,
	IMPR_DT_COMP_CBC_APP,  
    IMPR_DT_COMP_CBC_HB,  
	IMPR_DT_COMP_CBC_TAKEOFF,
	IMPR_DT_COMP_CBC_LAND,
} cs_data_in_type_t;


struct impr_ce_analysis_data {
	cs_data_in_type_t type;
	union {
		struct impr_app_data app_data;
		struct impr_cb_comp_data_hb_ack hb_ack;
		struct impr_fc_ack_data			fc_ack; 
	} u;
};

struct impr_cbc_pack_data {
	cs_data_in_type_t type;
	union {
		struct impr_cb_cbc_hb_data 	hb_data;
		struct impr_app_data_ack 	app_ack; 
		struct impr_fc_data			fc_data;        
	} u;
};



std::string gettwo(int m);
std::string yyyyMMdd();

void thread_resume(int &statu,pthread_mutex_t &mut,pthread_cond_t &cond);
void thread_pause(int &status,pthread_mutex_t &mut,pthread_cond_t &cond);

int WhereIsPoint(int point_x,int point_y);

extern int cbc_ctrl_position[4][4];

uint8_t xorCheck(uint8_t *pData, int offset, int len);
int8_t outputData_pack(uint8_t *size, uint8_t *data_out, struct impr_cbc_pack_data &cb_data);
int8_t inputData_analysis(uint8_t size, uint8_t *data_in, struct impr_ce_analysis_data &analysis_data);



#endif