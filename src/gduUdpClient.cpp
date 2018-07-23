#include "gduUdpClient.h"


static void print_hex( char *buf, int len)
{
	int i;
	printf("HEX(len =): ");
	for(i = 0; i < len; i ++)
		printf("%02x ", buf[i]);
	printf("\n");
}

gduUdpClient::gduUdpClient()
{
    m_app_al_type  = 0x00;//0x01===开始 0x02 == 停止 0x03 === 一键降落
    m_pre_SerialNum = 0x00;
    m_cbc_pose_data = {0};
    this->m_initalFlage = 0;
}

int gduUdpClient::initalParameter()
{
    if(0==this->m_initalFlage)
    {
        this->m_initalFlage = 1;
        this->m_sockfd = UDP_ClientInit();
        printf("build UDP %d!\n", m_sockfd);
        int rtn = UDP_SetSockAddr(SER_ADDR, SER_PORT, &this->m_server_addr);
        printf("UDP app_task_spawn]set socket option return value :%d\n", rtn);
        this->createThread();
    }
    return 1;
}
gduUdpClient::~gduUdpClient()
{
    this->udpClose();
}
static void *udp_receive_thread(void *phandle)
{

    gduUdpClient *pUdp = (gduUdpClient *)phandle;
    struct impr_ce_analysis_data analysised_data;
    struct impr_exchange_data_cbc recieve_data = {0};
    recieve_data.data = new uint8_t[MAX_LEN];
    recieve_data.size = MAX_LEN;

    while(1)
    {
         pUdp->udpReciveData(recieve_data);

         struct impr_exchange_data_cbc  real_data;
         uint8_t tmp_key_num = recieve_data.data[GDU_PROTOCOL_CMD_KEY_POS];
         switch(tmp_key_num)
         {
             case GDU_PROTOCOL_APP_CMD_KEY_CBC:
             {
                real_data.size = GDU_CBC_APP_TO_CBC_LEN;
                real_data.data = new uint8_t[GDU_CBC_APP_TO_CBC_LEN];
                
                 break;
             }
             case GDU_PROTOCOL_CE_CMD_KEY_CBC:
             {
                real_data.size = GDU_CBC_DATA_TO_HEARTBEAT_ACK_LEN;
                real_data.data = new uint8_t[GDU_CBC_DATA_TO_HEARTBEAT_ACK_LEN];
                break;
             }
             case GDU_PROTOCOL_TAKEOFF_CMD_KEY_CBC:
             {
                 real_data.size = GDU_CBC_TAKEOFF_TO_CBC_ACK_LEN;
                 real_data.data = new uint8_t[GDU_CBC_TAKEOFF_TO_CBC_ACK_LEN];
                 break;
             }
             case GDU_PROTOCOL_LAND_CMD_KEY_CBC:
             {
                 real_data.size =GDU_CBC_LAND_TO_CBC_ACK_LEN;
                 real_data.data =new uint8_t[GDU_CBC_LAND_TO_CBC_ACK_LEN];
                 break;
             }
             default:
             {
                continue;
                break;
             }

         }
         pUdp->udpfilterByKey(recieve_data,real_data);
        int8_t rtn_flage = FAILED;
        rtn_flage = inputData_analysis(real_data.size, real_data.data, analysised_data);
        if (FAILED == rtn_flage)
        {
            printf("WARNNING!!!inputData_analysis failed!rtn_flage is %d,analysised_data.type is %d\n", rtn_flage, analysised_data.type);
        }
        else
        {
            switch (analysised_data.type)
            {
            case IMPR_DT_COMP_CBC_HB:
                break;
            case IMPR_DT_COMP_CBC_APP:
            {
                pUdp->udpGetAndSendAppData(analysised_data);
                break;
            }
            case IMPR_DT_COMP_CBC_TAKEOFF:
            {

                break;
            }
	        case IMPR_DT_COMP_CBC_LAND:
            {
                break;
            }
            default:
                break;
            }
        }
        delete[] real_data.data;
        real_data.data = NULL;
        usleep(1000);//1ms
    }
    delete[] recieve_data.data;
    recieve_data.data = NULL;
    return ((void *)0);
}

static void *HeartBeathread(void *phandle)
{
    //////////////////////////////////
    gduUdpClient *pUdp = (gduUdpClient *)phandle;

    struct impr_cbc_pack_data cbc_data_out;
    cbc_data_out.type = IMPR_DT_COMP_CBC_HB;
    cbc_data_out.u.hb_data.ack_value = 0x00;
    cbc_data_out.u.hb_data.serial_num = 0x01;

    struct impr_exchange_data_cbc callback_data = {0};
    callback_data.data = new uint8_t[GDU_CBC_DATA_TO_HEARTBEAT_LEN];
    
    //struct impr_ce_analysis_data analysised_data;
    struct impr_exchange_data_cbc recieve_data = {0};
    recieve_data.data = new uint8_t[GDU_CBC_DATA_TO_HEARTBEAT_ACK_LEN];
    recieve_data.size = GDU_CBC_DATA_TO_HEARTBEAT_ACK_LEN;

    struct cbc_position_data tmp_cbc_data={0};
    pUdp->udpSetPosition(1,1,1);
    while(1)
    {
        pUdp->udpGetPosition(tmp_cbc_data);
        cbc_data_out.u.hb_data.x = tmp_cbc_data.cbc_x;
        cbc_data_out.u.hb_data.y = tmp_cbc_data.cbc_y;
        cbc_data_out.u.hb_data.z = tmp_cbc_data.cbc_z;
        int rtn = outputData_pack(&callback_data.size, callback_data.data, cbc_data_out);
        if (SUCCESS == rtn && callback_data.size == GDU_CBC_DATA_TO_HEARTBEAT_LEN)
		{
            pUdp->udpSendData(callback_data);
        }
        else
		{
			printf("WARNING!!!output_pack callback to CE failed,size is %d,return is %d\n", callback_data.size, rtn);
		}
        /*int tmp_ack_counter = 0;
        for(int i = 0;i<5;i++)
        {
            //my_udp.udpReciveData(recieve_data);
            int8_t rtn_flage = FAILED;
            rtn_flage = inputData_analysis(recieve_data.size, recieve_data.data, analysised_data);
            if (FAILED == rtn_flage)
            {
                printf("WARNNING!!!inputData_analysis failed!rtn_flage is %d,analysised_data.type is %d\n", rtn_flage, analysised_data.type);

            }
            else
            {
               uint8_t ack_value = analysised_data.u.hb_ack.ack_value;
               uint8_t serial_num = analysised_data.u.hb_ack.serial_num;

                if(ack_value == 0x00){tmp_ack_counter++;}
            }
            usleep(2000);//20ms
        }
        if (tmp_ack_counter == 0)
        {
             printf("WARNNING!!!no right ack return.\n");
        }*/
        sleep(1);//2s
    }

    delete[] callback_data.data;
    callback_data.data =  NULL;
    delete[] recieve_data.data;
    recieve_data.data = NULL;
    
    return ((void *)0);
//////////////////////////////////////
}

int gduUdpClient::createThread()
{
    ////////////////////////////////
    pthread_t receive_data_thread;
    pthread_create(&receive_data_thread, NULL, &udp_receive_thread, (void *)this);
    pthread_detach(receive_data_thread);
    /////////////////////////////////
    pthread_t send_hb_thread;
    pthread_create(&send_hb_thread, NULL, &HeartBeathread, (void *)this);
    pthread_detach(send_hb_thread);
    printf(".............create receive_data_thread and send_hb_thread success!\n");
    return 1;
}

int gduUdpClient::udpSendData(struct impr_exchange_data_cbc  send_data)
{
    char *data_tmp = (char*)send_data.data;
    printf("send data to udp,size is %d,value is: ",send_data.size);
    print_hex(data_tmp, send_data.size);
    int rtn = UDP_Send(this->m_sockfd, data_tmp, send_data.size, &this->m_server_addr);
    
    if(rtn <0)
    {
        printf("[WARRING]------>UDP_Send rtn value is %d\n",rtn ); 
        return 0;
    }
    return 1;
}
int gduUdpClient::udpReciveData(struct impr_exchange_data_cbc &receive_data)
{
    
    char *data_tmp  = (char*)receive_data.data;
    int rtn = UDP_Recv(this->m_sockfd, data_tmp, receive_data.size, &this->m_server_addr);
    printf("[INF]------>UDP_Recv rtn value is %d\n", rtn);
    printf("receive data to udp,size is %d,value is: ",receive_data.size);
    print_hex(data_tmp, receive_data.size);
    if(rtn <0)
    {
        
        return 0;
    }
    return 1;
}

int  gduUdpClient::udpClose()
{

    UDP_CloseSocket(this->m_sockfd);
    return 1;
}

int gduUdpClient::udpfilterByKey(struct impr_exchange_data_cbc input_data, struct impr_exchange_data_cbc &output_data)
{
    for (int i = 0; i < output_data.size; i++)
    {
        output_data.data[i] = input_data.data[i];
    }

    return 1;
}

int gduUdpClient::udpGetAndSendAppData(struct impr_ce_analysis_data analysised_data)
{
    struct impr_app_data_ack tmp_cb_app_ack;
    tmp_cb_app_ack.ack_value = 0x00;

    struct impr_cbc_pack_data cbc_data_out;
    cbc_data_out.type = IMPR_DT_COMP_CBC_APP;

    struct impr_exchange_data_cbc callback_data = {0};
    callback_data.data = new uint8_t[GDU_CBC_APP_TO_CBC_ACK_LEN];
    cbc_data_out.u.app_ack = tmp_cb_app_ack;

    uint8_t tmp_serial_num = analysised_data.u.app_data.serial_num;
    uint8_t tmp_al_type = analysised_data.u.app_data.al_type;
    if(tmp_serial_num > this->m_pre_SerialNum)
    {
        this->m_app_al_type = tmp_al_type;
    }
    printf("----------------------->app data serial_num,al_type value is %d,%d\n", tmp_serial_num, tmp_al_type);

    cbc_data_out.u.app_ack.serial_num = tmp_serial_num;
    int rtn = outputData_pack(&callback_data.size, callback_data.data, cbc_data_out);
    if (SUCCESS == rtn && callback_data.size == GDU_CBC_APP_TO_CBC_ACK_LEN)
    {
        this->udpSendData(callback_data);
    }
    else
    {
        printf("WARNING!!!output_pack callback to CE failed,size is %d,return is %d\n", callback_data.size, rtn);
    }

    delete[] callback_data.data;
    callback_data.data = NULL;
    
    return 1;
}

int  gduUdpClient::udpTakeOff()
{
    struct impr_cbc_pack_data cbc_data_out;
    cbc_data_out.type = IMPR_DT_COMP_CBC_TAKEOFF;
    cbc_data_out.u.fc_data.serial_num = 0x01;

    struct impr_exchange_data_cbc callback_data = {0};
    callback_data.data = new uint8_t[GDU_CBC_TAKEOFF_TO_FC_LEN];
    
    int rtn = outputData_pack(&callback_data.size, callback_data.data, cbc_data_out);
    if (SUCCESS == rtn && callback_data.size == GDU_CBC_TAKEOFF_TO_FC_LEN)
    {
        this->udpSendData(callback_data);
    }
    else
    {
        printf("WARNING!!!output_pack callback to CE failed,size is %d,return is %d\n", callback_data.size, rtn);
    }
    delete[] callback_data.data;
    callback_data.data = NULL;
    return 1;
}

int  gduUdpClient::udpLand()
{
    struct impr_cbc_pack_data cbc_data_out;
    cbc_data_out.type = IMPR_DT_COMP_CBC_LAND;
    cbc_data_out.u.fc_data.serial_num = 0x01;

    struct impr_exchange_data_cbc callback_data = {0};
    callback_data.data = new uint8_t[GDU_CBC_LAND_TO_FC_LEN];
    
    int rtn = outputData_pack(&callback_data.size, callback_data.data, cbc_data_out);
    if (SUCCESS == rtn && callback_data.size == GDU_CBC_LAND_TO_FC_LEN )
    {
        this->udpSendData(callback_data);
    }
    else
    {
        printf("WARNING!!!output_pack callback to CE failed,size is %d,return is %d\n", callback_data.size, rtn);
    }
    delete[] callback_data.data;
    callback_data.data = NULL;
    
    return 1;
}

int gduUdpClient::udpSetPosition(int16_t x, int16_t y, int16_t z)
{
    this->m_cbc_pose_data.cbc_x = x;
    this->m_cbc_pose_data.cbc_y = y;
    this->m_cbc_pose_data.cbc_z = z;
    return 1;
}
int gduUdpClient::udpGetPosition(struct cbc_position_data &xyz_data)
{
    xyz_data.cbc_x =  this->m_cbc_pose_data.cbc_x;
    xyz_data.cbc_y =  this->m_cbc_pose_data.cbc_y;
    xyz_data.cbc_z =  this->m_cbc_pose_data.cbc_z;

    return 1;
}

 uint8_t  gduUdpClient::udpGetAPPType()
 {
    return this->m_app_al_type;
 }

 int  gduUdpClient::udpSetAppType(uint8_t type)
 {
     this->m_app_al_type = type;
     return 1;
 }

int gduUdpClient::gduSetUAV()
{
    struct impr_exchange_data_cbc callback_data = {0};
    callback_data.size = 8;
    callback_data.data = new uint8_t[8];
    callback_data.data[0] = 0x55;
    callback_data.data[1] = 4;
    callback_data.data[2] = 0x73;
    callback_data.data[3] = 0x86;
    callback_data.data[4] = 0xff;
    callback_data.data[5] =  0x00;
    callback_data.data[6] = xorCheck(callback_data.data, 1, 5);
    callback_data.data[7] = 0xf0; 

    this->udpSendData(callback_data);

    delete[] callback_data.data;
    callback_data.data = NULL;
    return 1;
}
