/*
雷达接口函数主要分为两类：
（1）解析雷达数据，雷达是通过CAN向外发送数据的，CAN消息有四类（原始目标信息、跟踪目标信息，状态信息，版本信息），具体详细解释用户可参阅ESRR/EMRR使用说明
文档。如：当用户调用RADAR_CAN_MSG函数时，该函数会识别雷达消息类型并将相关信息解析分类存储在对应的信息列表中，当列表的更新标志update_flag=1时表示更新完
成，用户即可根据需求取用相关信息。

（2）设置雷达，即向雷达发送相应的CAN消息，雷达会识别CAN消息内容，从而改变工作模式等，当配置雷达发送的CAN消息列表can_transmit_data中的send_flag=1时，
表示设置正确，用户此时取出列表中的send_id和send_data,将其发送给雷达，雷达即可识别该消息从而改变相应的设置
具体详细解释用户可参阅ESRR/EMRR使用说明文档。
*/

#ifndef RADARH
#define RADARH


#define USE_RAW_DET_LIST          1  //使用原始目标
#define USE_OBJECT_LIST           1  //使用跟踪目标
#define USE_RADAR_STATE           1  //读取雷达状态
#define USE_RADAR_VERSION         1  //读取雷达版本
#define USE_RADAR_SET		  1  //使用雷达设置

#define MAX_TARGET_NUM 128

typedef struct{
	unsigned char org_id;           //目标ID
	float range;                   //距离
	float angle;                   //角度
	float speed;                   //速度
	float level;	               //幅度
	float snr;                     //信噪比
}raw_tar_msg_t;

//原始目标信息列表
typedef struct{
	unsigned char update_flag;           // 目标列表更新标志 =1表示一帧目标接收正常，=0表示一帧目标没有完全接收，=-1表示该帧接收有错误
	unsigned char raw_det_num;	     //一个周期检测到的目标数
	unsigned char raw_det_meas_mode;     //测量模式
	int raw_det_meas_counter;            //测量次数
	int raw_det_time_stamp;              //原始目标时间戳
	raw_tar_msg_t target[MAX_TARGET_NUM];//目标信息
}RAW_DET_LIST_T;


typedef struct{
	unsigned char tar_id;          //目标ID
	float x;                       //水平距离
	float y;                       //垂直距离
	float Vx;                      //X方向速度
	float Vy;	               //Y方向速度 
	float rcs;                     //目标RCS
	float obj_dynpropr;            //目标运动属性
}track_tar_msg_t;
//跟踪目标信息列表
typedef struct {
	unsigned char update_flag;              // 目标列表更新标志 =1表示一帧目标接收正常，=0表示一帧目标没有完全接收，=-1表示该帧接收有错误
	unsigned char object_num;               //目标数
	int object_meas_counter;                //测量次数
	int object_time_stamp;                  //跟踪目标时间戳
	track_tar_msg_t target[MAX_TARGET_NUM]; //目标信息
	unsigned char obj_warrning[2];		//跟踪目标报警信息（obj_warrning[0] = 报警目标的ID, obj_warrning[1]报警目标位于哪个区域），设置碰撞检测时才有效
}OBJECT_LIST_T;

//雷达状态信息
typedef struct {
	unsigned char radar_state_work_mode;            //雷达当前工作模式(0X0：正常模式,0X1：高分辨模式)
	unsigned char radar_state_id;			//雷达ID(0--7)
	unsigned char radar_state_fail;			//(0X0: 雷达工作正常,0X1: 雷达处于失效状态)
	unsigned char radar_state_temporary_error;	//0X0: 雷达临时故障
	unsigned char radar_state_eeprom_error;		//0X0：无故障,0X1：读错误,0X2：写错误,0X3：读写错误
	unsigned char radar_state_transceiver_error;	//0X0：无故障,0X1：发射机故障,0X2：接收机故障,0X3：收发机故障
	unsigned char radar_state_voltage_error;	//0x0：无故障,0x1：欠压故障, VBAT<9V,0x2：过压故障, VBAT>36
	unsigned char radar_state_temperature_error;	//0x0: 无故障,0x1：低温故障	,0x2:  高温故障
	unsigned char radar_state_outtype;		//雷达输出目标类型(0x0：不输出,0x1：原始目标,0x2：跟踪目标)
	unsigned char radar_state_send_quality;		//雷达是否输出目标质量信息(0x0：不输出,0x1：输出)
	unsigned char radar_state_send_ext;		//雷达是否输出跟踪目标的扩展信息(0x0：不输出,0x1：输出)
	unsigned char radar_state_sort_index;		//雷达输出跟踪目标时的排序方式(0x0: 不排序,0x1 : 按距离排序(升序),0x2 : 按RCS排序(降序))
	unsigned char radar_state_motion_rx_state;	//雷达接收车速横摆消息故障(0x0: 正常,0x1 : 无车速,0x2 : 无横摆,0x3 : 无车速无横摆)
}Radar_State_T;


//雷达软硬件版本信息
typedef struct {
	int release_year;			//发布时间: 年
	unsigned char release_month;		//发布时间：月
	unsigned char release_day;		//发布时间：日

	unsigned char software_major;		//软件主版本号
	unsigned char software_minor;		//软件次版本号
	unsigned char hadware_version;		//硬件版本
}Radar_Version_T;



//配置雷达时发送给雷达的CAN消息
typedef struct{
	int can_id;
	int can_dlc;
	unsigned char can_data[8];
	unsigned char send_flag;  // =1表示设置正常，可以发送，=0表示未设置，=-1表示设置有错误
}Radar_Set_T;


//雷达数据解析,用于解析雷达CAN消息，包括原始目标解析、跟踪目标解析、雷达状态、雷达软硬件版本
void receive_radar_can_msg(int can_id, unsigned char can_data[8]);
//函数参数说明：can_id: 雷达发出CAN消息ID,can_data:雷达发出的CAN消息数据,

//雷达位号设置（0--7）
void set_radar_id(char radar_id, char new_id);
/* 函数参数说明：current_id: 雷达当前位号（0--7）, goal_id :想要设置的位号（0--7），
mode：雷达当前工作模式（0x0:原始目标，0x1:跟踪目标）*/

//雷达模式切换（跟踪/原始目标输出）
void set_radar_track(char radar_id, char track_en);
// 函数参数说明：track: （0x0跟踪输出关闭雷达输出原始目标，0x1跟踪打开雷达输出跟踪后的目标）, radar_id: 雷达当前位号（0--7）

//过滤器设置
void set_radar_filter(char radar_id, char filter_cfg_type, char filter_mode, char filter_cfg_index, float filter_value_min, float filter_value_max);
/* 函数参数说明： 
radar_id：雷达当前位号（0--7）
filter_cfg_type:（过滤对象 0x0:过滤原始目标，0x1:过滤跟踪目标） 
filter_mode：过滤模式（0x0:通过式过滤，0x1:拦截式过滤 ）
filter_cfg_index：过滤参数选择（
0x0		按发送目标数过滤，最小值默认为0
0x1		按径向距离过滤
0x2		按水平角度过滤
0x3		按靠近速度的绝对值过滤
0x4		按远离速度的绝对值过滤
0x5		按RCS的绝对值过滤
0x6		按信噪比过滤
0x7		按X坐标过滤
0x8		按Y坐标过滤
0x9		对靠近的目标按vy绝对值过滤，保留所有远离目标
0xa		对远离的目标按vy绝对值过滤，保留所有靠近目标
0xb		对从右到左运动的目标，按vx绝对值过滤，保留所有从左到右的目标
0xc		对从左到右运动的目标，按vy绝对值过滤，保留所有从右到左的目标
）
filter_value_min：过滤参数下限
filter_value_max：过滤参数上限
*/

//清除过滤器设置
void clear_radar_filter(char radar_id, char filter_cfg_type, char filter_cfg_index, char filter_cfg_clear);
/* 函数参数说明：
radar_id:雷达当前位号(0--7)
filter_cfg_index：过滤参数
filter_cfg_clear：清除方式（
0x0:  不清除
0x1:  清除所有过滤条件
0x2:  清除所有filter_cfg_index对应的过滤条件
）
*/

//高边驱动器输出设置
void set_radar_hsd(char radar_id, char hsd, char hsd_vout, float hsd_on_time, float hsd_off_time, char hsd_cycles, float hsd_overload);
/*
radar_id:雷达当前位号(0--7)
hsd:选择hsd(0x1:hsd1,0x2:hsd2, 0x3:hsd1和hsd2)

hsd_vout	HSD输出电压(0x0: VBAT×6.25% ,0x1: VBAT×12.5%,…,0xf: VBAT×100%)

hsd_on_time	HSD导通持续时间(0x0: 0s,0x1: 0.25s,…,0xf: 3.75s)

hsd_off_time	HSD关断持续时间(0x0: 0s, 0x1: 0.25s, … ,0xf: 3.75s	)

hsd_cycles	HSD输出周期数(一个周期 = off_time + on_time,0x0: 一直输出,0x1: 一个周期,…,0xf: 15个周期)

hsd_overload  HSD电流过载保护门限，当平均电流大于保护门限时，高边驱动器关闭输出(0--0.75A)

*/

// 碰撞检测总体设置
void  set_coll_det(char radar_id, char coll_det_activition, char coll_det_reset, char coll_det_clear_regions, char coll_det_clear_region_index);
/*函数说明：
radar_id：雷达当前位号（0--7）
coll_det_activition	    （0X0: 碰撞检测功能关闭，0x1: 碰撞检测功能打开）
coll_det_reset          （0x1：清除所有报警标志）
coll_det_clear_regions	（0x1：清除所有已经设置的区域，0x2：清除选中的区域）
coll_det_clear_region_index	（0x1: 碰撞区域0 … 0x7: 碰撞区域7）
*/

//碰撞检测区域设置
void  set_coll_det_region(char radar_id, char coll_det_region_hsd_sel, char coll_det_region_index, char coll_det_region_mode,
	char coll_det_send_to_radar_sign, float region_1, float region_2, float region_3, float region_4, float coll_det_region_time);
/*函数说明：
radar_id：雷达当前位号（0--7）
coll_det_region_hsd_sel	报警信号从哪一路高边驱动输出（0x0: 不输出, 0x1: 从HSD1输出, 0x2: 从HSD2输出, 0x3: 从HSD1和HSD2输出）
coll_det_region_index	碰撞区域索引（0--7）
coll_det_region_mode	碰撞区域形状（0x0: 凸四边形，0x1: 扇环）
coll_det_send_to_radar_sign 发送给雷达CAN消息帧标记(0x0: 第一帧,0x1: 第二帧)
axis_1 （扇环碰撞区域第一点θ坐标，凸四边形碰撞区域第一点x坐标，凸四边形碰撞区域第三点x坐标）
axis_2	(扇环碰撞区域第一点r坐标,凸四边形碰撞区域第一点y坐标,凸四边形碰撞区域第三点y坐标)
axis_3	(扇环碰撞区域第二点θ坐标,凸四边形碰撞区域第二点x坐标,凸四边形碰撞区域第四点x坐标)
axis_4	(扇环碰撞区域第二点r坐标,凸四边形碰撞区域第二点y坐标,凸四边形碰撞区域第四点y坐标)
coll_det_region_time	目标需要在碰撞区域存在的时间大于此时间，目标才会触发报警(0--25.5S)
*/

//设置参数保存
void set_save(int radar_id, char en);
/*函数说明：
radar_id：雷达当前位号（0--7）
memory:   是否存储(en=1：存储 ， en=0：不存储)
*/


#if USE_RAW_DET_LIST
extern RAW_DET_LIST_T raw_det_list;
#endif 

#if USE_OBJECT_LIST
extern OBJECT_LIST_T object_list;
#endif 

#if USE_RADAR_STATE 
extern Radar_State_T radar_state;
#endif

#if USE_RADAR_VERSION
extern Radar_Version_T radar_version;
#endif

#if USE_RADAR_SET
extern Radar_Set_T can_transmit_data;
#endif
#endif

