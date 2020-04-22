

/*
当用户第一次拿到易来达雷达时，雷达出厂默认统一设置为：0号位，跟踪目标输出，其他设置均无。

以下example将为用户展示从从第一次拿到雷达到设置雷达以及使用雷达的过程：
首先我们将一个出厂状态的雷达连接好，CAN接收和发送正常
（1）将雷达由0号位设置为5号位，
（2）将检测距离范围设置为10--20米
（3）设置一个四边形的碰撞检测区域四个点坐标为((-10，10），（10，10），（10，20），（-10，20)),目标在区域内存在时间超过1.0S时触发报警
（4）解析一个周期的雷达数据并输出每个目标的距离信息
（5）清除所有雷达设置
（6）关闭跟踪将雷达设置为输出原始目标
*/

#include"radar.h"
#include"stdio.h"

//CAN发送数据，example以控制台输出为示范，实际设置时只需将此函数更换为CAN发送函数，将消息通过CAN发送给雷达CAN1即可完成相应的设置
void can_transmit(int can_id, unsigned char can_data[8])  //
{

	printf("Send to radar: can_id =  %x", can_id);
	printf(" , can_data =  %x", can_data[0]);

	for (int i = 1; i < 8;i++)
	{
		printf(" ,%x",can_data[i]);
	}
	printf("\n");
}


//CAN接收到雷达的数据
typedef struct
{
	int can_id;
	unsigned char can_data[8];

}CAN_RECEIVE_T;

CAN_RECEIVE_T  can_receive_data;



void main()
{
	/************************   设置雷达ID   ************************/
	char radar_id = 0;   //雷达的初始ID为0
	char new_id = 5;     //设置雷达新的ID为5

	set_radar_id(radar_id, new_id);

	if (can_transmit_data.send_flag == 1)
	{
		can_transmit(can_transmit_data.can_id, can_transmit_data.can_data);
	}
	else
	{
		printf("error...\n");	//当参数设置不符合规范时报错（如：某个参数超出最大设置范围）
	}

	/************************   设置雷达过滤   ************************/

	char filter_cfg_type = 0;    //过滤原始目标
	char filter_mode = 0;		 //通过式过滤
	char filter_cfg_index = 1;	 //按照目标径向距离过滤
	float filter_value_min = 10.0; //设最小距离为10
	float filter_value_max = 20.0; //设置最大距离为20

	set_radar_filter(new_id, filter_cfg_type, filter_mode, filter_cfg_index, filter_value_min, filter_value_max);

	if (can_transmit_data.send_flag == 1)
	{
		can_transmit(can_transmit_data.can_id, can_transmit_data.can_data);
	}
	else
	{
		printf("error...\n");	//当参数设置不符合规范时报错（如：超出最大设置范围）
	}

	/********************************   碰撞检测总体设置（碰撞检测仅适用于跟踪目标）   ******************************/

	char coll_det_activition = 1;    //碰撞检测功能打开
	char coll_det_reset = 0;         //雷达出厂没有任何报警标志，不需要清除
	char coll_det_clear_regions = 0; //雷达出厂没有任何报警标志，不需要清除
	char coll_det_clear_region_index = 1;//设置碰撞区域1

	set_coll_det(new_id, coll_det_activition, coll_det_reset, coll_det_clear_regions, coll_det_clear_region_index);

	if (can_transmit_data.send_flag == 1)
	{
		can_transmit(can_transmit_data.can_id, can_transmit_data.can_data);
	}
	else
	{
		printf("error...\n");	//当参数设置不符合规范时报错（如：超出最大设置范围）
	}

	/********************************   碰撞检测区域设置 (需要连续发2帧CAN消息，具体可参见雷达使用说明)  ******************************/

	char coll_det_region_hsd_sel = 1;		//选择高边驱动器1
	char coll_det_region_index = 0;         //设置碰撞区域编号为0
	char coll_det_region_mode = 0;	        //设置四边形碰撞区域
	char coll_det_send_to_radar_sign = 0;   //发送给雷达的CAN消息标记：第一帧
	float region_1 = -10.0;                 //凸四边形碰撞区域第一点x坐标
	float region_2 = 10.0;                  //凸四边形碰撞区域第一点y坐标
	float region_3 = 10.0;			//凸四边形碰撞区域第二点x坐标
	float region_4 = 10.0;			//凸四边形碰撞区域第二点y坐标
	float coll_det_region_time = 3.5;       //目标需要在碰撞区域存在的时间大于3.5S，触发报警
	set_coll_det_region(new_id, coll_det_region_hsd_sel, coll_det_region_index, coll_det_region_mode, coll_det_send_to_radar_sign,
		region_1, region_2, region_3, region_4, coll_det_region_time);   //设置碰撞检测区域的第一帧消息

	if (can_transmit_data.send_flag == 1)
	{
		can_transmit(can_transmit_data.can_id, can_transmit_data.can_data);
	}
	else
	{
		printf("error...\n");	//当参数设置不符合规范时报错（如：超出最大设置范围）
	}

	coll_det_send_to_radar_sign = 1;    //发送给雷达的CAN消息标记：第二帧
	region_1 = 10.0;                    //凸四边形碰撞区域第三点x坐标
	region_2 = 20.0;                    //凸四边形碰撞区域第三点y坐标
	region_3 = -10.0;		    //凸四边形碰撞区域第四点x坐标
	region_4 = 20.0;		    //凸四边形碰撞区域第四点y坐标
	coll_det_region_time = 1.0;         //目标需要在碰撞区域存在的时间大于1S，触发报警

	set_coll_det_region(new_id, coll_det_region_hsd_sel, coll_det_region_index, coll_det_region_mode, coll_det_send_to_radar_sign,
		region_1, region_2, region_3, region_4, coll_det_region_time);   //设置碰撞检测区域的第二帧消息

	if (can_transmit_data.send_flag == 1)
	{
		can_transmit(can_transmit_data.can_id, can_transmit_data.can_data);
	}
	else
	{
		printf("error...\n");	//当参数设置不符合规范时报错（如：超出最大设置范围）
	}

	/********************************   高边驱动器输出设置   ******************************/
	char hsd = 1;                   //设置高边驱动器1
	char hsd_vout = 0xf;		//设置输出电压100%
	float hsd_on_time = 1;          //0.25S ON
	float hsd_off_time = 1;		//0.25S OFF
	char hsd_cycles = 5;            //连续输出5个周期
	float hsd_overload = 0.75;      //平均电流大于0.75A时，高边驱动器关闭输出
	set_radar_hsd(new_id, hsd, hsd_vout, hsd_on_time,  hsd_off_time, hsd_cycles, hsd_overload);

	if (can_transmit_data.send_flag == 1)
	{
		can_transmit(can_transmit_data.can_id, can_transmit_data.can_data);
	}
	else
	{
		printf("error...\n");	//当参数设置不符合规范时报错（如：超出最大设置范围）
	}

	/********************************   雷达设置保存   ******************************/

	//配置完雷达之后需要将设置保存，否则只能本次有效，下次上电恢复原状态
	char en = 1;
	set_save( new_id, en);

	if (can_transmit_data.send_flag == 1)
	{
		can_transmit(can_transmit_data.can_id, can_transmit_data.can_data);
	}
	else
	{
		printf("error...\n");	//当参数设置不符合规范时报错（如：超出最大设置范围）
	}

	/********************************   雷达目标解析   ******************************/
	int i = 0;
	while (i < 2)
	{
		can_receive_data.can_id = 0x551;
		unsigned char data[8] = { 0x0a, 00, 00, 00, 00, 00, 00, 00 };
		memcpy(can_receive_data.can_data, data, 8);

		receive_radar_can_msg(can_receive_data.can_id, can_receive_data.can_data);  //解析目标头帧

		if (raw_det_list.update_flag == 1)               // 每遇到一个头析头帧时查看目标列表update_flag是否=1时，表示上一个周期目标全部解析完成,
		{						 //最后一个周期无下一个头帧，update_flag不更新
			printf("target_num = %d\n", raw_det_list.raw_det_num);
			printf("range = ");
			for (int i = 0; i < raw_det_list.raw_det_num; i++)
			{
				printf(" %.2f,", raw_det_list.raw_tar_msg.range[i]);  //取目标的距离信息
			}
			printf("\n");
		}
		for (int j = 0; j < 10; j++)     //解析目标数据，一个周期连续来10个目标
		{
			can_receive_data.can_id = 0x553;
			unsigned char data[8] = { 1, 6, 68, 200, 82, 192, 108, 10 };
			memcpy(can_receive_data.can_data, data, 8);
			receive_radar_can_msg(can_receive_data.can_id, can_receive_data.can_data);
		}

		i++;		
	}


	/********************************   清除雷达设置   ******************************/

	/********************************   （1）清除过滤设置   ******************************/
	filter_cfg_type = 0;
	filter_cfg_index = 0;
	char filter_cfg_clear = 1;
	clear_radar_filter(new_id, filter_cfg_type, filter_cfg_index, filter_cfg_clear);
	if (can_transmit_data.send_flag == 1)
	{
		can_transmit(can_transmit_data.can_id, can_transmit_data.can_data);
	}
	else
	{
		printf("error...\n");	//当参数设置不符合规范时报错（如：超出最大设置范围）
	}

	/************************   （2）恢复雷达ID=0   ************************/

	radar_id = 5;   //雷达被设置ID为5
	new_id = 0;     //将雷达ID恢复为0

	set_radar_id(radar_id, new_id);

	if (can_transmit_data.send_flag == 1)
	{
		can_transmit(can_transmit_data.can_id, can_transmit_data.can_data);
	}
	else
	{
		printf("error...\n");	//当参数设置不符合规范时报错（如：超出最大设置范围）
	}

	/************************   （3）关闭碰撞检测，并清除所有已经设置的碰撞检测区域   ************************/
	coll_det_activition = 0;    //碰撞检测功能关闭
	coll_det_reset = 1;         //清除所有报警标志
	coll_det_clear_regions = 1; //清除所有区域
	coll_det_clear_region_index = 0;
	set_coll_det(new_id, coll_det_activition, coll_det_reset, coll_det_clear_regions, coll_det_clear_region_index);

	if (can_transmit_data.send_flag == 1)
	{
		can_transmit(can_transmit_data.can_id, can_transmit_data.can_data);
	}
	else
	{
		printf("error...\n");   //当参数设置不符合规范时报错（如：超出最大设置范围）
	}

	/************************   将雷达目标输出跟踪关闭，输出原始目标   ************************/

	char track_en = 0;  //设置雷达模式为原始目标输出
	set_radar_track(new_id, track_en);
	if (can_transmit_data.send_flag == 1)
	{
		can_transmit(can_transmit_data.can_id, can_transmit_data.can_data);
	}
	else
	{
		printf("error...\n");	//当参数设置不符合规范时报错（如：超出最大设置范围）
	}

	printf("end\n");
	system("pause");

}


