
#include<stdio.h>
#include <stdint.h>
#include "radar.h"
#include<string.h>


RAW_DET_LIST_T raw_det_list;
OBJECT_LIST_T object_list;
Radar_State_T radar_state;
Radar_Version_T radar_version;
Radar_Set_T  can_transmit_data;


int track_tar_num = 0;        //跟踪目标计数
int org_tar_num = 0;          //原始目标计数
unsigned short int tempData;  //临时数据

//原始目标头帧信息临时存放列表
typedef struct{
	unsigned char raw_det_num;	     //检测目标数
	unsigned char raw_det_meas_mode;     //测量模式
	int raw_det_meas_counter;            //测量次数
	int raw_det_time_stamp;              //时间戳


}Tmp_Org_Msg_T;

Tmp_Org_Msg_T tmp_raw_list;


//跟踪目标头帧信息临时存放列表
typedef struct{
	unsigned char object_num;             //目标数
	int object_meas_counter;              //测量次数
	int object_time_stamp;                //时间戳

}Tmp_Track_Msg_T;

Tmp_Track_Msg_T tmp_object_list;


void receive_radar_can_msg(int can_id, unsigned char *can_data)
{

//****************原始目标解析******************************************************************
	if (can_id == 0x501 || can_id == 0x511 || can_id == 0x521 || can_id == 0x531 || can_id == 0x541 || can_id == 0x551 
		|| can_id == 0x561 || can_id == 0x571)    //解析头帧
	{
		//先把头帧解析放在临时列表暂时存放
		tmp_raw_list.raw_det_num = can_data[0];
		tmp_raw_list.raw_det_meas_counter = (uint16_t)(can_data[2]) | (uint16_t)((can_data[1]) << 8);
		tmp_raw_list.raw_det_meas_mode = (uint16_t)((can_data[3] & 0xF0) >> 8);
		tmp_raw_list.raw_det_time_stamp = (uint16_t)(can_data[5]) | (uint16_t)((can_data[4] << 8));

		if (org_tar_num != 0) //目标数不等于0，说明这是中间某一帧，则判断上一帧解析是否正确
		{	
			if (org_tar_num == raw_det_list.raw_det_num)	 //目标数等于头帧上报数目，上一帧正常
			{
				raw_det_list.update_flag = 1;    //上一帧目标解析完成，目标数目等于头帧上报数目，更新标志置为1，表示正常
			}
			else  //目标数目不等于头帧上报数目，数据有丢失
			{
				raw_det_list.update_flag = -1;    //上一帧目标解析完成，目标数目不等于头帧上报数目，更新标志置为2，表示有错误
			}
			org_tar_num = 0;  //目标列表更新完成，目标计数清零
		 }
	}
	if (can_id == 0x503 || can_id == 0x513 || can_id == 0x523 || can_id == 0x533 || can_id == 0x543 || can_id == 0x553 ||
		can_id == 0x563 || can_id == 0x573)    //解析目标
	{
		//解析目标时将临时列表中的头帧信息更新给目标列表
		raw_det_list.raw_det_num = tmp_raw_list.raw_det_num;
		raw_det_list.raw_det_meas_counter = tmp_raw_list.raw_det_meas_counter;
		raw_det_list.raw_det_meas_mode = tmp_raw_list.raw_det_meas_mode;
		raw_det_list.raw_det_time_stamp = tmp_raw_list.raw_det_time_stamp;

		raw_det_list.raw_tar_msg.org_id[org_tar_num] = can_data[0];

		tempData = ((uint16_t)(can_data[2] & 0xF0)) >> 4 | ((uint16_t)(can_data[1] & 0xFF)) << 4;
		raw_det_list.raw_tar_msg.range[org_tar_num] = ((float)tempData)* 0.1f;

		tempData = ((uint16_t)(can_data[3] & 0xFF)) >> 0 | ((uint16_t)(can_data[2] & 0x07)) << 8;
		raw_det_list.raw_tar_msg.speed[org_tar_num] = ((float)tempData)* 0.1f - 102.4f;

		tempData = ((uint16_t)(can_data[5] & 0xF0)) >> 4 | ((uint16_t)(can_data[4] & 0x7F)) << 4;
		raw_det_list.raw_tar_msg.angle[org_tar_num] = ((float)tempData)* 0.1f - 102.4;

		tempData = can_data[6] & 0xFF;
		raw_det_list.raw_tar_msg.level[org_tar_num] = ((float)tempData)* 0.5f - 64.0f;

		tempData = can_data[7] & 0xFF;
		raw_det_list.raw_tar_msg.snr[org_tar_num] = ((float)tempData)*1.0f;

		org_tar_num++;

		raw_det_list.update_flag = 0;//解析目标时，将列表跟新标志清零，表示正在解析目标
	}

//*************************跟踪目标解析*******************************************************************************
	if (can_id == 0x601 || can_id == 0x611 || can_id == 0x621 || can_id == 0x631 || can_id == 0x641 || can_id == 0x651
		|| can_id == 0x661 || can_id == 0x671)    //解析头帧
	{
		//先把头帧解析暂时存放在临时头帧信息列表
		tmp_object_list.object_num = can_data[0];
		tmp_object_list.object_meas_counter = ((uint16_t)(can_data[2])) >> 0 | ((uint16_t)(can_data[1])) << 8;
		tmp_object_list.object_time_stamp = (uint16_t)(can_data[4]) | (uint16_t)((can_data[3] << 8));

		if (track_tar_num != 0)   //目标数不等于0，说明这是中间某一帧，则判断上一帧解析是否正确
		{
			if (track_tar_num == tmp_object_list.object_num)	 //目标数等于头帧上报数目，上一帧正常
			{
				object_list.update_flag = 1;     //上一帧目标解析完成，目标数目等于头帧上报数目，更新标志置为1，表示正常
			}
			else                                    //目标数目不等于头帧上报数目，数据有丢失
			{
				object_list.update_flag = -1;    //上一帧目标解析完成，目标数目不等于头帧上报数目，更新标志置为2，表示有错误
			}
			track_tar_num = 0;                     //目标列表更新完成，目标计数清零
		}
	}

	if (can_id == 0x603 || can_id == 0x613 || can_id == 0x623 || can_id == 0x633 || can_id == 0x643 || can_id == 0x653 ||
		can_id == 0x663 || can_id == 0x673)    //解析目标
	{
		//解析目标时将临时列表中的头帧信息更新给目标列表
		object_list.object_num = tmp_object_list.object_num;
		object_list.object_meas_counter = tmp_object_list.object_meas_counter;
		object_list.object_time_stamp = tmp_object_list.object_time_stamp;

		//解析目标信息
		object_list.track_tar_msg.tar_id[track_tar_num] = can_data[0];

		tempData = ((uint16_t)(can_data[2] & 0xF0)) >> 4 | ((uint16_t)(can_data[1] & 0xFF)) << 4;
		object_list.track_tar_msg.x[track_tar_num] = ((float)tempData)* 0.1f - 204.8f;

		tempData = ((uint16_t)(can_data[3] & 0xFF)) >> 0 | ((uint16_t)(can_data[2] & 0x0F)) << 8;
			
		object_list.track_tar_msg.y[track_tar_num] = ((float)tempData) * 0.1f;

		tempData = ((uint16_t)(can_data[5] & 0xE0)) >> 5 | ((uint16_t)(can_data[4] & 0xFF)) << 3;
		object_list.track_tar_msg.Vx[track_tar_num] = ((float)tempData)*0.1f - 102.4f;

		tempData = ((uint16_t)(can_data[6] & 0xFF)) >> 0 | ((uint16_t)(can_data[5] & 0x07)) << 8;
		object_list.track_tar_msg.Vy[track_tar_num] = ((float)tempData)*0.1f - 102.4f;

		tempData = (uint16_t)(can_data[7] & 0xFF);
		object_list.track_tar_msg.rcs[track_tar_num] = ((float)tempData)* 0.5f - 64.0f;

		object_list.track_tar_msg.obj_dynpropr[track_tar_num] = ((uint16_t)(can_data[5] & 0x18)) >> 3;

		track_tar_num++;

		object_list.update_flag = 0; //解析目标时，将列表跟新标志清零，表示正在解析目标
	}

	//跟踪目标报警信息
	if (can_id == 0x609 || can_id == 0x619 || can_id == 0x629 || can_id == 0x639 || can_id == 0x649 || can_id == 0x659
		|| can_id == 0x669 || can_id == 0x679)
	{
		object_list.obj_warrning[0] = can_data[0];
		object_list.obj_warrning[1] = can_data[1];
	}

	// 雷达状态读取
	if (can_id == 0x401 || can_id == 0x411 || can_id == 0x421 || can_id == 0x431 || can_id == 0x441 || can_id == 0x451
		|| can_id == 0x461 || can_id == 0x471)
	{
		radar_state.radar_state_work_mode = (uint16_t)(can_data[0] & 0x07);
		radar_state.radar_state_id = ((uint16_t)(can_data[0] & 0x38)) >> 3;
		radar_state.radar_state_fail = ((uint16_t)(can_data[0] & 0x40)) >> 6;
		radar_state.radar_state_temporary_error = ((uint16_t)(can_data[0] & 0x80)) >> 7;

		radar_state.radar_state_eeprom_error = (uint16_t)(can_data[1] & 0x03);
		radar_state.radar_state_transceiver_error = ((uint16_t)(can_data[1] & 0x0c)) >> 2;
		radar_state.radar_state_voltage_error = ((uint16_t)(can_data[1] & 0x30)) >> 4;
		radar_state.radar_state_temperature_error = ((uint16_t)(can_data[1] & 0xc0)) >> 6;

		radar_state.radar_state_outtype = can_data[3] & 0x03;
		radar_state.radar_state_send_quality = ((uint16_t)(can_data[3] & 0x04)) >> 2;
		radar_state.radar_state_send_ext = ((uint16_t)(can_data[3] & 0x08)) >> 3;

		radar_state.radar_state_sort_index = ((uint16_t)(can_data[4] & 0x70)) >> 4;

		radar_state.radar_state_motion_rx_state = ((uint16_t)(can_data[5] & 0xc0)) >> 6;

	}

	// 雷达软件版本读取
	if (can_id == 0x701 || can_id == 0x711 || can_id == 0x721 || can_id == 0x731 || can_id == 0x741 || can_id == 0x751
		|| can_id == 0x761 || can_id == 0x771)
	{
		radar_version.software_major = can_data[0] & 0xFF;
		radar_version.software_minor = can_data[1] & 0xFF;
		radar_version.hadware_version = can_data[2] & 0xFF;

		radar_version.release_year = (((uint16_t)(can_data[3] & 0xFE)) >> 1) + 2000;
		radar_version.release_month = (can_data[4] & 0x0F);
		radar_version.release_day = ((uint16_t)(can_data[4] & 0xF0)) >> 4 | ((uint16_t)(can_data[3] & 0x01)) << 4;
	}
}
	

//设置雷达是否输出跟踪模式目标
void set_radar_track(char radar_id,char track_en)
{
	if ((track_en != 0 && track_en != 1) ||( radar_id<0 || radar_id > 7))
	{
		can_transmit_data.send_flag = -1;
	}
	else
	{
		can_transmit_data.can_id = 0x400 + radar_id * 16;
		can_transmit_data.can_data[0] = 0x44;
		can_transmit_data.can_data[1] = 0x0;
		can_transmit_data.can_data[2] = 0x0;
		can_transmit_data.can_data[3] = track_en + 1;
		can_transmit_data.can_data[4] = 0x0;
		can_transmit_data.can_data[5] = 0x0;
		can_transmit_data.can_data[6] = 0x0;
		can_transmit_data.can_data[7] = 0x1;
		can_transmit_data.can_dlc = 8;

		can_transmit_data.send_flag = 1;
	}
}


void set_radar_id(char radar_id,char new_id)
{
	if (radar_id<0 || radar_id>7 || new_id < 0 || new_id>7)

	{
		can_transmit_data.send_flag = -1;
	}
	else
	{
		can_transmit_data.can_id = 0x400 + radar_id * 16;
		can_transmit_data.can_data[0] = 0x42;
		can_transmit_data.can_data[1] = 0x0;
		can_transmit_data.can_data[2] = new_id*8;
		can_transmit_data.can_data[3] = 0x0;
		can_transmit_data.can_data[4] = 0x0;
		can_transmit_data.can_data[5] = 0x0;
		can_transmit_data.can_data[6] = 0x0;
		can_transmit_data.can_data[7] = 0x1;
		can_transmit_data.can_dlc = 8;

		can_transmit_data.send_flag = 1;		
	}
}


//过滤器设置
void set_radar_filter(char radar_id,char filter_cfg_type, char filter_mode, char filter_cfg_index, float filter_value_min, float filter_value_max)
{
	switch (filter_cfg_index)
	{
		//case 0:按发送目标数过滤，最小值默认为0，
	case 0:   
		filter_value_min = filter_value_min;
		filter_value_max = filter_value_max;
		break;

	case 1://case 1:按径向距离过滤
		filter_value_min = filter_value_min*10;
		filter_value_max = filter_value_max*10;
		break;

		//case 2:按水平角过滤
	case 2:
		filter_value_min = (filter_value_min + 102.4)*20;
		filter_value_max = (filter_value_max + 102.4)*20;

		//case 5:按rcs过滤
	case 5:
		filter_value_min = (filter_value_min+50)*40;
		filter_value_max = (filter_value_max+50)*40;
		break;

		//case 6:按信噪比过滤
	case 6:
		filter_value_min = filter_value_min * 80;
		filter_value_max = filter_value_max * 80;
		break;

		//case 7:按X坐标过滤过滤
	case 7:
		filter_value_min = (filter_value_min +204.8)* 10;
		filter_value_max = (filter_value_max +204.8)* 10;
		break;

		//case 8:按Y坐标过滤过滤
	case 8:
		filter_value_min = filter_value_min * 10;
		filter_value_max = filter_value_max * 10;
		break;

		/*
		case 3:按靠近速度绝对值过滤，case 4:按远离速度绝对值过滤， case 9:对靠近目标按Vy绝对值过滤保留所有远离目标
		case 0xA:对远离目标按Vy绝对值过滤保留所有靠近目标，case 0xB:对从右到左运动的目标，按vx绝对值过滤，保留所有
		从左到右的目标，  case 0xC:对从左到右运动的目标，按vy绝对值过滤，保留所有从右到左的目标
		*/
	case 3:	
	case 4:	
	case 9:
	case 10:
	case 11:
	case 12:
		filter_value_min = filter_value_min * 40;
		filter_value_max = filter_value_max * 40;
		break;	
	}

	if ((radar_id>7 || radar_id<0) || (filter_cfg_type != 0 && filter_cfg_type != 1) || (filter_mode != 0 && filter_mode != 1)
		|| (filter_cfg_index<0 || filter_cfg_index>12) || (filter_value_min<0 || filter_value_min>4096) ||( filter_value_max<0 
		|| filter_value_max>4096))
	{
		can_transmit_data.send_flag = -1;
	}
	else
	{
		can_transmit_data.can_id = 0x402 + radar_id * 16;  //发送ID
		can_transmit_data.can_data[0] = (filter_mode & 0x01) | ((1 & 0x01) << 1) | ((1 & 0x1) << 2) 
						| ((filter_cfg_index & 0x0f) << 3) | ((filter_cfg_type & 0x01) << 7);
		can_transmit_data.can_data[1] = ((int)(filter_value_min)& 0xF00) >> 8;
		can_transmit_data.can_data[2] = ((int)(filter_value_min)& 0xFF);
		can_transmit_data.can_data[3] = ((int)(filter_value_max)& 0xF00) >> 8;
		can_transmit_data.can_data[4] = ((int)(filter_value_max)& 0xFF);
		can_transmit_data.can_data[5] = 0x0;
		can_transmit_data.can_data[6] = 0x0;
		can_transmit_data.can_data[7] = 0x0;
		can_transmit_data.can_dlc = 8;

		can_transmit_data.send_flag = 1;
	}
}


//清除过滤器设置
void clear_radar_filter(char radar_id, char filter_cfg_type, char filter_cfg_index, char filter_cfg_clear)
{
	if ((radar_id<0 || radar_id>7) || (filter_cfg_type<0 || filter_cfg_type>2) || (filter_cfg_index<0 || filter_cfg_index>13)
		|| (filter_cfg_clear<0 || filter_cfg_clear>2))
	{
		can_transmit_data.send_flag = -1;

	}
	else
	{
		can_transmit_data.can_id = 0x402 + radar_id * 16;  //发送ID
		can_transmit_data.can_data[0] = 0x07 | ((filter_cfg_index & 0x0f) << 3) | ((filter_cfg_type & 0x01) << 7);
		can_transmit_data.can_data[1] = (filter_cfg_clear) << 4;
		can_transmit_data.can_data[2] = 0x0;
		can_transmit_data.can_data[3] = 0x0;
		can_transmit_data.can_data[4] = 0x0;
		can_transmit_data.can_data[5] = 0x0;
		can_transmit_data.can_data[6] = 0x0;
		can_transmit_data.can_data[7] = 0x0;
		can_transmit_data.can_dlc = 8;

		can_transmit_data.send_flag = 1;
	}
}

// 碰撞检测总体设置
void  set_coll_det(char radar_id,char coll_det_activition, char coll_det_reset, char coll_det_clear_regions, char coll_det_clear_region_index)
{	
	if ((radar_id<0 || radar_id>7) 
		|| (coll_det_activition != 0 && coll_det_activition != 1) 
		|| (coll_det_reset != 0 && coll_det_reset!=1)
		|| (coll_det_clear_regions<0 || coll_det_clear_regions>2) 
		|| (coll_det_clear_region_index<0 || coll_det_clear_region_index>7))
	{
		can_transmit_data.send_flag = -1;
	}
	else
	{
		can_transmit_data.can_id = 0x406 + radar_id * 16;  //发送ID
		can_transmit_data.can_data[0] = (coll_det_activition & 0x01) | ((coll_det_reset & 0x01) << 1) |
			((coll_det_clear_regions & 0x02) << 2) | ((coll_det_clear_region_index & 0x07) << 4);
		can_transmit_data.can_data[1] = 0x0;
		can_transmit_data.can_data[2] = 0x0;
		can_transmit_data.can_data[3] = 0x0;
		can_transmit_data.can_data[4] = 0x0;
		can_transmit_data.can_data[5] = 0x0;
		can_transmit_data.can_data[6] = 0x0;
		can_transmit_data.can_data[7] = 0x0;
		can_transmit_data.can_dlc = 8;

		can_transmit_data.send_flag = 1;
	}
}

//碰撞检测区域设置
void  set_coll_det_region(char radar_id, char coll_det_region_hsd_sel, char coll_det_region_index, char coll_det_region_mode, 
	char coll_det_send_to_radar_sign, float region_1, float region_2, float region_3, float region_4, float coll_det_region_time)
{
	if ((radar_id<0 || radar_id>7) 
		|| (coll_det_region_hsd_sel<0 || coll_det_region_hsd_sel>3) 
		|| (coll_det_region_index<0 || coll_det_region_index>7)
		|| (coll_det_region_mode<0 || coll_det_region_mode>1) 
		|| (coll_det_send_to_radar_sign<0 || coll_det_send_to_radar_sign>1)
		|| (region_1<-204.8 || region_1>204.7) 
		|| (region_2<0 || region_2>409.5) 
		|| (region_3<-204.8 || region_3>203.7) 
		|| (region_4<0 || region_4>409.5))
	{
		can_transmit_data.send_flag = -1;
	}
	else
	{
		region_1 = (region_1 + 204.8) * 10;
		region_2 = region_2 * 10;
		region_3 = (region_3 + 204.8) * 10;
		region_4 = region_4 * 10;
		coll_det_region_time = coll_det_region_time * 10;
	
		can_transmit_data.can_data[0] = 0x01 | ((coll_det_region_hsd_sel & 0x03) << 1) | ((coll_det_region_index & 0x07) << 3) |
			((coll_det_region_mode & 0x01) << 6) | ((coll_det_send_to_radar_sign & 0x01) << 7);

		can_transmit_data.can_data[1]= (int)(region_1)& 0xFF0;
		can_transmit_data.can_data[2] = (((int)(region_1)& 0x00F) << 4) | (int)(region_2)& 0xF00;
		can_transmit_data.can_data[3] = (int)(region_2)& 0x0FF;
		can_transmit_data.can_data[4] = (int)(region_3)& 0xFF0;
		can_transmit_data.can_data[5] = (((int)(region_3)& 0x00F) << 4) | (int)(region_4)& 0xF00;
		can_transmit_data.can_data[6] = (int)(region_4)& 0x0FF;
		can_transmit_data.can_data[7] = (int)(coll_det_region_time);
		can_transmit_data.can_id = 0x408 + radar_id * 16;  //发送ID
		can_transmit_data.can_dlc = 8;

		can_transmit_data.send_flag = 1;			
	}
}
 
//高边驱动器输出设置
void set_radar_hsd(char radar_id, char hsd, char hsd_vout, float hsd_on_time, float hsd_off_time, char hsd_cycles, float hsd_overload)
{
	if ((radar_id<0 || radar_id>7) || (hsd<0 || hsd>3) || (hsd_vout<0 || hsd_vout>15) || (hsd_on_time<0 || hsd_on_time>3.75) ||
		(hsd_cycles<0 || hsd_cycles>15) || (hsd_overload<0 || hsd_overload>0.75))
	{
		can_transmit_data.send_flag = -1;
	}
	else
	{	
		can_transmit_data.can_id = 0x40A + radar_id * 16;

		hsd_on_time = hsd_on_time * 4;
		hsd_off_time = hsd_off_time * 4;
		hsd_overload = hsd_overload * 20;

		can_transmit_data.can_data[0] = (hsd & 0x3) << 7;

		if (hsd == 1)
		{
			can_transmit_data.can_data[1] = hsd_vout & 0x0F;
			can_transmit_data.can_data[2] = (int)hsd_on_time & 0x0F;
			can_transmit_data.can_data[3] = (int)hsd_off_time & 0x0F;
			can_transmit_data.can_data[4] = hsd_cycles & 0x0F;
			can_transmit_data.can_data[5] = (int)hsd_overload & 0x0F;
		}

		if (hsd == 2)
		{
			can_transmit_data.can_data[1] = (hsd_vout & 0x0F) << 4;
			can_transmit_data.can_data[2] = ((int)hsd_on_time & 0x0F) << 4;
			can_transmit_data.can_data[3] = ((int)hsd_off_time & 0x0F) << 4;
			can_transmit_data.can_data[4] = (hsd_cycles & 0x0F) << 4;
			can_transmit_data.can_data[5] = ((int)hsd_overload & 0x0F) << 4;
		}

		if (hsd == 3)
		{
			can_transmit_data.can_data[1] = ((int)hsd_vout & 0x0F) | ((int)hsd_vout & 0x0F) << 4;
			can_transmit_data.can_data[2] = ((int)hsd_on_time & 0x0F) | ((int)hsd_on_time & 0x0F) << 4;
			can_transmit_data.can_data[3] = ((int)hsd_off_time & 0x0F) | ((int)hsd_off_time & 0x0F) << 4;
			can_transmit_data.can_data[4] = (hsd_cycles & 0x0F) | (hsd_cycles & 0x0F) << 4;
			can_transmit_data.can_data[5] = ((int)hsd_overload & 0x0F) | ((int)hsd_overload & 0x0F) << 4;
		}
		can_transmit_data.can_data[6] = 0x0;
		can_transmit_data.can_data[7] = 0x0;
		can_transmit_data.can_dlc = 8;

		can_transmit_data.send_flag = 1;
	}
}


//设置参数存储
void set_save(int radar_id,char en)  
{
	if (radar_id<0 || radar_id>7 || (en != 0 && en != 1))  
	{
		can_transmit_data.send_flag = -1;
	}
	else if (en == 1)
	{	
		can_transmit_data.can_id = 0x400 + radar_id * 16;
		can_transmit_data.can_data[0] = 0x40;
		can_transmit_data.can_data[1] = 0x0;
		can_transmit_data.can_data[2] = 0x0;
		can_transmit_data.can_data[3] = 0x0;
		can_transmit_data.can_data[4] = 0x0;
		can_transmit_data.can_data[5] = 0x0;
		can_transmit_data.can_data[6] = 0x0;
		can_transmit_data.can_data[7] = 0x01;
		can_transmit_data.can_dlc = 8;

		can_transmit_data.send_flag = 1;
	}
}

