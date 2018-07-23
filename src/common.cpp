

#include "common.h"
//35*24
//int cbc_ctrl_position[4][4] = {{0,20,1,1},{90,330,-1,-1},{180,220,1,-1},{-90,20,-1,1}};//角度、pid控制线、1=y轴和-1=x轴、pid控制变号 （1=一、四区域   -1=二三区域）
//int cbc_four_area[4][8] = {{0,0,40,0,40,40,0,40},{310,0,350,0,350,40,310,40},{310,200,350,200,350,240,310,240},{0,200,40,200,40,240,0,240}};

//44*34
int cbc_ctrl_position[4][4] = {{0,80,1,1},{90,350,-1,-1},{180,260,1,-1},{-90,100,-1,1}};
int cbc_four_area[4][8] = {{80,60,120,60,120,100,80,100},{330,60,370,60,370,100,330,100},{330,240,370,240,370,280,330,280},{80,240,120,240,120,280,80,280}};//0412

std::string gettwo(int m)
{
	//int m1 = 0;
	//int m2 = 0;
	return std::to_string(m / 10) + std::to_string(m % 10);
}
std::string yyyyMMdd()
{
	time_t tt = time(NULL); //这句返回的只是一个时间cuo
	tm *t = localtime(&tt);
	return "" + std::to_string(t->tm_year + 1900) +
		   gettwo(t->tm_mon + 1) +
		   gettwo(t->tm_mday) + gettwo(t->tm_hour) + gettwo(t->tm_min) + gettwo(t->tm_sec);
}

void thread_resume(int &status, pthread_mutex_t &mut, pthread_cond_t &cond)
{
	if (status == STOP)
	{
		pthread_mutex_lock(&mut);
		status = RUN;
		pthread_cond_signal(&cond);
		printf("pthread run!\n");
		pthread_mutex_unlock(&mut);
	}
	else
	{
		printf("pthread run already\n");
	}
}
void thread_pause(int &status, pthread_mutex_t &mut, pthread_cond_t &cond)
{
	if (status == RUN)
	{
		pthread_mutex_lock(&mut);
		status = STOP;
		printf("thread stop!\n");
		pthread_mutex_unlock(&mut);
	}
	else
	{
		printf("pthread pause already\n");
	}
}

uint8_t xorCheck(uint8_t *pData, int offset, int len)
{
	int index = 0;
	uint8_t xorCheckout = 0x00;

	for (index = 0; index < len; index++)
	{
		xorCheckout = xorCheckout ^ *(pData + index + offset);
	}
	return xorCheckout;
}

int8_t inputData_analysis(uint8_t size, uint8_t *data_in, struct impr_ce_analysis_data &analysis_data)
{
	int8_t rt_ = SUCCESS;
	uint8_t xorCheckValue = 0;
	//printf("inputData_analysis check 1");
	if (NULL == data_in || size < 7)
	{
		rt_ = FAILED;
		printf("FAILED!! data_in is NULL! data_in's size is %d. Returned %d.\n", size, rt_);
		return rt_;
	}
	//printf("inputData_analysis check 2");

	uint8_t normal_size_ = 0;
	uint8_t normal_addr_ = 0;
	cs_data_in_type_t data_type = IMPR_DT_COMP_NONE;
	uint8_t cmd_addr_value = data_in[GDU_PROTOCOL_CMD_ADDR_POS];
	uint8_t cmd_key_value = data_in[GDU_PROTOCOL_CMD_KEY_POS];

	switch (cmd_key_value)
	{
	/*case GDU_PROTOCOL_CS_CMD_KEY_OT:
			{
				data_type = IMPR_DT_COMP_OT;
				normal_size_ = GDU_OT_DATA_TO_CS_LEN;
				normal_addr_ = GDU_PROTOCOL_CE_TO_CS_ADDR;
				break;
			}*/
	case GDU_PROTOCOL_CE_CMD_KEY_CBC:
	{
		data_type = IMPR_DT_COMP_CBC_HB;
		normal_size_ = GDU_CBC_DATA_TO_HEARTBEAT_ACK_LEN;
		normal_addr_ = GDU_PROTOCOL_HB_TO_CE_ADDR;
		break;
	}
	case GDU_PROTOCOL_APP_CMD_KEY_CBC:
	{
		data_type = IMPR_DT_COMP_CBC_APP;
		normal_size_ = GDU_CBC_APP_TO_CBC_LEN;
		normal_addr_ = GDU_PROTOCOL_HB_TO_CE_ADDR;
		break;
	}
	case GDU_PROTOCOL_TAKEOFF_CMD_KEY_CBC:
	{
		data_type = IMPR_DT_COMP_CBC_TAKEOFF;
		normal_size_ = GDU_CBC_TAKEOFF_TO_CBC_ACK_LEN;
		normal_addr_ = GDU_PROTOCOL_FC_TO_CBC_ADDR;
		break;
	}
	case GDU_PROTOCOL_LAND_CMD_KEY_CBC:
	{
		data_type = IMPR_DT_COMP_CBC_LAND;
		normal_size_ = GDU_CBC_LAND_TO_CBC_ACK_LEN;
		normal_addr_ = GDU_PROTOCOL_FC_TO_CBC_ADDR;
		break;
	}
	default:
		break;
	}
	//printf("inputData_analysis check 3");

	//check with size and addr
	if (normal_size_ != size || normal_addr_ != cmd_addr_value)
	{
		rt_ = FAILED;
		printf("ERROR!! normal size is %d, but data_in size is %d! normal_addr_ is %x, but cmd_addr_value is %x. Returned %d.\n", normal_size_, size, normal_addr_, cmd_addr_value, rt_);
		return rt_;
	}
		//printf("inputData_analysis check 4");
//#ifdef GDU_GR_PRINT_LOG_ENABLE
	//test
	printf("*********************************************gr Get data: data_in size is %d.\n", size);
	for (int j = 0; j < size; j++)
	{
		printf("0x%02x ", data_in[j]);
	}
	printf("\n");
//#endif

	xorCheckValue = xorCheck(data_in, 1, normal_size_ - 3);
	if (xorCheckValue == data_in[normal_size_ - 2])
	{
		analysis_data.type = data_type;
		switch (analysis_data.type)
		{
		/*case IMPR_DT_COMP_OT:
        {
            analysis_data.u.ot.algorithm_type = data_in[5];
            analysis_data.u.ot.obj_rect.x = (unsigned short)(data_in[6] | (data_in[7] << 8));
            analysis_data.u.ot.obj_rect.y = (unsigned short)(data_in[8] | (data_in[9] << 8));
            analysis_data.u.ot.obj_rect.w = (unsigned short)(data_in[10] | (data_in[11] << 8));
            analysis_data.u.ot.obj_rect.h = (unsigned short)(data_in[12] | (data_in[13] << 8));
            break;
        }*/
		case IMPR_DT_COMP_CBC_HB:
		{
			
			analysis_data.u.hb_ack.serial_num = data_in[4];
			analysis_data.u.hb_ack.ack_value = data_in[5];
			break;
		}
		case IMPR_DT_COMP_CBC_APP:
		{
			printf("analysis_data.u.hb_ack.serial_num is %02x",analysis_data.u.hb_ack.serial_num);
			analysis_data.u.app_data.serial_num = data_in[4];
			analysis_data.u.app_data.al_type = data_in[5];
			break;
		}
		case IMPR_DT_COMP_CBC_TAKEOFF:
		{
			analysis_data.u.fc_ack.serial_num = data_in[4];
			analysis_data.u.fc_ack.ack_value = data_in[5];
			break;
		}
		case IMPR_DT_COMP_CBC_LAND:
		{
			analysis_data.u.fc_ack.serial_num = data_in[4];
			analysis_data.u.fc_ack.ack_value = data_in[5];
			break;
		}
		default:
			break;
		}
#ifdef GDU_PRINT_LOG_ENABLE
			printf("Get tack info: algorithm_type is %x.\n",analysis_data.u.ot.algorithm_type,
#endif
	}
	else
	{
		rt_ = FAILED;
		printf("FAILED!! xorCheck failed! Returned %d.\n", rt_);
	}

	return rt_;
}

int8_t outputData_pack(uint8_t *size, uint8_t *data_out, struct impr_cbc_pack_data &cb_data)
{
	int8_t rt_ = SUCCESS;
	uint8_t xorCheckValue = 0;
	uint8_t cmd_addr_value = 0;
	uint8_t serial_num = 0x01;

	if (NULL == data_out)
	{
		rt_ = FAILED;
		printf("FAILED!! data_in is NULL! Returned %d.\n", rt_);
		return rt_;
	}

	switch (cb_data.type)
	{
	/*case IMPR_DT_COMP_OT:
			{
			*size = GDU_OT_DATA_TO_CE_LEN;
			cmd_addr_value = GDU_PROTOCOL_CS_TO_CE_ADDR;
			data_out[1]    = GDU_OT_DATA_TO_CE_LEN - 4;
			data_out[3]    = GDU_PROTOCOL_CE_CMD_KEY_OT;

			data_out[5]    = cb_data.u.ot.status;
			data_out[6]    = (uint8_t)(cb_data.u.ot.objs.x);
			data_out[7]    = (uint8_t)(cb_data.u.ot.objs.x >> 8);
			data_out[8]    = (uint8_t)(cb_data.u.ot.objs.y);
			data_out[9]    = (uint8_t)(cb_data.u.ot.objs.y >> 8);
			data_out[10]   = (uint8_t)(cb_data.u.ot.objs.w);
			data_out[11]   = (uint8_t)(cb_data.u.ot.objs.w >> 8);
			data_out[12]   = (uint8_t)(cb_data.u.ot.objs.h);
			data_out[13]   = (uint8_t)(cb_data.u.ot.objs.h >> 8);
			data_out[14]   = (uint8_t)(cb_data.u.ot.track_frameID);
			data_out[15]   = (uint8_t)(cb_data.u.ot.track_frameID >> 8);
			data_out[16]   = (uint8_t)(cb_data.u.ot.track_frameID >> 16);
			data_out[17]   = (uint8_t)(cb_data.u.ot.track_frameID >> 24);
			data_out[18]   = (uint8_t)(cb_data.u.ot.procedure_status);
			data_out[19]   = (uint8_t)(cb_data.u.ot.algFunc_type);
			break;
			}*/
	case IMPR_DT_COMP_CBC_HB:
	{
		*size = GDU_CBC_DATA_TO_HEARTBEAT_LEN;
		cmd_addr_value = GDU_PROTOCOL_CE_TO_AHB_ACK_ADDR;
		serial_num = cb_data.u.hb_data.serial_num;
		data_out[1] = GDU_CBC_DATA_TO_HEARTBEAT_LEN - 4;
		data_out[3] = GDU_PROTOCOL_CE_CMD_KEY_CBC;
		data_out[5] = cb_data.u.hb_data.ack_value;
		
		data_out[6] = (uint8_t)cb_data.u.hb_data.x >> 8;
		data_out[7] = (uint8_t)cb_data.u.hb_data.x;
		
		data_out[8] = (uint8_t)cb_data.u.hb_data.y >> 8;
		data_out[9] = (uint8_t)cb_data.u.hb_data.y;
		
		data_out[10] = (uint8_t)cb_data.u.hb_data.z >> 8;
		data_out[11] = (uint8_t)cb_data.u.hb_data.z;
		break;
	}
	case IMPR_DT_COMP_CBC_APP:
	{
		*size = GDU_CBC_APP_TO_CBC_ACK_LEN;
		cmd_addr_value =  GDU_PROTOCOL_CE_TO_AHB_ACK_ADDR;
		serial_num	= cb_data.u.app_ack.serial_num;
		printf("serial_num value is %d\n",serial_num);
		data_out[1] = GDU_CBC_APP_TO_CBC_ACK_LEN - 4;
		data_out[3] = GDU_PROTOCOL_APP_CMD_KEY_CBC;
		data_out[5] = cb_data.u.app_ack.ack_value;
		break;
	}
	case IMPR_DT_COMP_CBC_TAKEOFF:
	{
		*size = GDU_CBC_TAKEOFF_TO_FC_LEN;
		cmd_addr_value = GDU_PROTOCOL_CBC_TO_FC_ADDR;
		serial_num	= cb_data.u.fc_data.serial_num;
		data_out[1] = GDU_CBC_TAKEOFF_TO_FC_LEN -4;
		data_out[3] = GDU_PROTOCOL_TAKEOFF_CMD_KEY_CBC;
		break;
	}
	case IMPR_DT_COMP_CBC_LAND:
	{
		*size = GDU_CBC_LAND_TO_FC_LEN;
		cmd_addr_value = GDU_PROTOCOL_CBC_TO_FC_ADDR;
		serial_num	= cb_data.u.fc_data.serial_num;
		data_out[1] = GDU_CBC_LAND_TO_FC_LEN-4;
		data_out[3] = GDU_PROTOCOL_LAND_CMD_KEY_CBC;
		break;
	}
	default:
		break;
	}
	data_out[0] = 0x55;
	data_out[2] = cmd_addr_value;
	data_out[4] = serial_num; //�����

	xorCheckValue = xorCheck(data_out, 1, *size - 3);
	data_out[*size - 2] = xorCheckValue;
	data_out[*size - 1] = 0xf0;

#ifdef GDU_PRINT_LOG_ENABLE
	//test
	printf("Pack output data: size is %d.\n", *size);
	for (int j = 0; j < *size; j++)
	{
		printf("0x%02x ", data_out[j]);
	}
	printf("\n");
#endif

	return rt_;
}
int WhereIsPoint(int point_x,int point_y)//
{
    for(int i=0;i<4;i++){
        if(point_x-cbc_four_area[i][0]<40&&point_x-cbc_four_area[i][0]>0
         &&point_y-cbc_four_area[i][1]<40&&point_y-cbc_four_area[i][1]>0)
         {
             return i=i+1;
         }
    }
    return 0;
}
