/**
 * @file referee_protocol.h
 * @author kidneygood (you@domain.com)
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */

 #ifndef referee_protocol_H
 #define referee_protocol_H
 
 #include "stdint.h"
#include <stdint.h>
 
 /****************************宏定义部分****************************/
 
 #define REFEREE_SOF 0xA5 // 起始字节,协议固定为0xA5
 #define Robot_Red 0
 #define Robot_Blue 1
 #define Communicate_Data_LEN 4 // 自定义交互数据长度，该长度决定了我方发送和他方接收，自定义交互数据协议更改时只需要更改此宏定义即可
 
 #pragma pack(1)
 
 /****************************通信协议格式****************************/
 
 /* 通信协议格式偏移，枚举类型,代替#define声明 */
 typedef enum
 {
     FRAME_HEADER_Offset = 0,
     CMD_ID_Offset = 5,
     DATA_Offset = 7,
 } JudgeFrameOffset_e;
 
 /* 通信协议长度 */
 typedef enum
 {
     LEN_HEADER = 5, // 帧头长
     LEN_CMDID = 2,	// 命令码长度
     LEN_TAIL = 2,	// 帧尾CRC16
 
     LEN_CRC8 = 4, // 帧头CRC8校验长度=帧头+数据长+包序号
 } JudgeFrameLength_e;
 
 /****************************帧头****************************/
 /****************************帧头****************************/
 
 /* 帧头偏移 */
 typedef enum
 {
     SOF = 0,		 // 起始位
     DATA_LENGTH = 1, // 帧内数据长度,根据这个来获取数据长度
     SEQ = 3,		 // 包序号
     CRC8 = 4		 // CRC8
 } FrameHeaderOffset_e;
 
 /* 帧头定义 */
 typedef struct
 {
     uint8_t SOF;
     uint16_t DataLength;
     uint8_t Seq;
     uint8_t CRC8;
 } xFrameHeader;
 
 /****************************cmd_id命令码说明****************************/
 /****************************cmd_id命令码说明****************************/
 
 /* 命令码ID,用来判断接收的是什么数据 */
 typedef enum
 {
     ID_game_state = 0x0001,				   // 比赛状态数据
     ID_game_result = 0x0002,			   // 比赛结果数据
     ID_game_robot_survivors = 0x0003,	   // 比赛机器人血量数据
     ID_event_data = 0x0101,				   // 场地事件数据
     ID_supply_projectile_action = 0x0102,  // 场地补给站动作标识数据
     ID_supply_projectile_booking = 0x0103, // 场地补给站预约子弹数据
     ID_game_robot_state = 0x0201,		   // 机器人状态数据
     ID_power_heat_data = 0x0202,		   // 实时功率热量数据
     ID_game_robot_pos = 0x0203,			   // 机器人位置数据
     ID_buff_musk = 0x0204,				   // 机器人增益数据
     ID_aerial_robot_energy = 0x0205,	   // 空中机器人能量状态数据
     ID_robot_hurt = 0x0206,				   // 伤害状态数据
     ID_shoot_data = 0x0207,				   // 实时射击数据
     ID_shoot_remaining = 0x0208,           //剩余发单量
     ID_rfid_status = 0X0209,                //rfid状态
     ID_ground_robot_position = 0X020B,     // 地面机器人位置，仅限哨兵机器人
     ID_sentry_info = 0X020D,              // 烧饼机器人自主决策指令
     ID_student_interactive = 0x0301,	   // 机器人间交互数据
     ID_map_data = 0x0307,                 //哨兵发送小地图
 } CmdID_e;
 
 /* 命令码数据段长,根据官方协议来定义长度，还有自定义数据长度 */
 typedef enum
 {
     LEN_game_state = 11,						 // 0x0001
     LEN_game_result = 1,						 // 0x0002
     LEN_game_robot_HP = 32,						 // 0x0003
     LEN_event_data = 4,							 // 0x0101
     LEN_supply_projectile_action = 4,			 // 0x0102
     LEN_game_robot_state = 13,					 // 0x0201
     LEN_power_heat_data = 16,					 // 0x0202
     LEN_game_robot_pos = 16,					 // 0x0203
     LEN_buff_musk = 6,							 // 0x0204
     LEN_aerial_robot_energy = 2,				 // 0x0205
     LEN_robot_hurt = 1,							 // 0x0206
     LEN_shoot_data = 7,							 // 0x0207
     LEN_shoot_remaing = 6,							 // 0x0208
     LEN_rfid_status =4,                             //0x0209
     LEN_ground_robot_position = 40,                 //0X020B
     LEN_sentry_info  = 6,                         //0x020D
     LEN_receive_data = 6 + Communicate_Data_LEN, // 0x0301
     LEN_map_data = 103,                          // 0x0307
 } JudgeDataLength_e;
 
 /****************************接收数据的详细说明****************************/
 /****************************接收数据的详细说明****************************/

typedef struct {
    uint8_t W_key : 1;
    uint8_t S_key : 1;
    uint8_t A_key : 1;
    uint8_t D_key : 1;
    uint8_t Shift_key : 1;
    uint8_t Ctrl_key : 1;
    uint8_t Q_key : 1;
    uint8_t E_key : 1;
    uint8_t R_key : 1;
    uint8_t F_key : 1;
    uint8_t G_key : 1;
    uint8_t Z_key : 1;
    uint8_t X_key : 1;
    uint8_t C_key : 1;
    uint8_t V_key : 1;
    uint8_t B_key : 1;
} keyboard_keys_t;

 typedef  struct
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;
 
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    keyboard_keys_t keyboard;
    uint16_t crc16;
}referee_remote_data_t;
 
 /* ID: 0x0001  Byte:  11    比赛状态数据 */
 typedef  struct 
 { 
  uint8_t game_type : 4; 
  uint8_t game_progress : 4; 
  uint16_t stage_remain_time; 
  uint64_t SyncTimeStamp; 
 } ext_game_state_t;
 
 /* ID: 0x0002  Byte:  1    比赛结果数据 */
 typedef struct
 {
    uint8_t winner;
 } ext_game_result_t;
 
 /* ID: 0x0003  Byte:  32    比赛机器人血量数据 */
 typedef struct
 {
     uint16_t red_1_robot_HP;
     uint16_t red_2_robot_HP;
     uint16_t red_3_robot_HP;
     uint16_t red_4_robot_HP;
     uint16_t red_5_robot_HP;
     uint16_t red_7_robot_HP;
     uint16_t red_outpost_HP;
     uint16_t red_base_HP;
     uint16_t blue_1_robot_HP;
     uint16_t blue_2_robot_HP;
     uint16_t blue_3_robot_HP;
     uint16_t blue_4_robot_HP;
     uint16_t blue_5_robot_HP;
     uint16_t blue_7_robot_HP;
     uint16_t blue_outpost_HP;
     uint16_t blue_base_HP;
 } ext_game_robot_HP_t;
 
 /* ID: 0x0101  Byte:  4    场地事件数据 */
 typedef struct
 {
     uint32_t event_type;
 } ext_event_data_t;
 
 /* ID: 0x0102  Byte:  3    场地补给站动作标识数据 */
 typedef struct
 {
     uint8_t supply_projectile_id;
     uint8_t supply_robot_id;
     uint8_t supply_projectile_step;
     uint8_t supply_projectile_num;
 } ext_supply_projectile_action_t;
 
 /* ID: 0X0201  Byte: 13    机器人状态数据 */
 typedef struct
 {
     uint8_t robot_id; 
     uint8_t robot_level; 
     uint16_t current_HP; 
     uint16_t maximum_HP; 
     uint16_t shooter_barrel_cooling_value; 
     uint16_t shooter_barrel_heat_limit; 
     uint16_t chassis_power_limit; 
     uint8_t power_management_gimbal_output : 1; 
     uint8_t power_management_chassis_output : 1; 
     uint8_t power_management_shooter_output : 1; 
 } ext_game_robot_state_t;
 
 /* ID: 0X0202  Byte: 16    实时功率热量数据 */
 typedef struct
 {
     uint16_t reserved_1; 
     uint16_t reserved_2; 
     float reserved_3; 
     uint16_t buffer_energy; 
     uint16_t shooter_17mm_1_barrel_heat; 
     uint16_t shooter_17mm_2_barrel_heat; 
     uint16_t shooter_42mm_barrel_heat; 
 } ext_power_heat_data_t;
 
 /* ID: 0x0203  Byte: 16    机器人位置数据 */
 typedef struct
 {
     float x;
     float y;
     float z;
     float yaw;
 } ext_game_robot_pos_t;
 
 /* ID: 0x0204  Byte:  6    机器人增益数据 */
 typedef struct
 {
     uint8_t recovery_buff; 
     uint8_t cooling_buff; 
     uint8_t defence_buff; 
     uint8_t vulnerability_buff; 
     uint16_t attack_buff; 
     uint8_t remaining_energy;
 } ext_buff_musk_t;
 
 /* ID: 0x0205  Byte:  2    空中机器人能量状态数据 */
 typedef struct
 {
    uint8_t airforce_status; 
    uint8_t time_remain; 
 } aerial_robot_energy_t;
 
 /* ID: 0x0206  Byte:  1    伤害状态数据 */
 typedef struct
 {
     uint8_t armor_id : 4;
     uint8_t hurt_type : 4;
 } ext_robot_hurt_t;
 
 /* ID: 0x0207  Byte:  7    实时射击数据 */
 typedef struct
 {
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
 } ext_shoot_data_t;

 /* ID: 0x0208  Byte:  6    剩余发单量 */
 typedef struct
 {
    uint16_t projectile_allowance_17mm; 
    uint16_t projectile_allowance_42mm;  
    uint16_t remaining_gold_coin; 
 } ext_shoot_remaing_t;


 /* ID: 0x0209  Byte:  4    RFID状态 */

 typedef  struct 
 { 
   uint32_t rfid_status; 
 }rfid_status_t; 


/* ID: 0x020B  Byte:  40  己方地面机器人位置数据，仅限哨兵机器人 */
typedef struct
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reversed_x;
    float reversed_y;
}ground_robot_position_t;

/* ID: 0x020D  Byte:  6  哨兵机器人自主决策指令 */
typedef struct
{
    // 第一个 32 位字段
    uint32_t sentry_success_local_exchange : 11; // bit 0-10
    uint32_t sentry_success_remote_exchange_count : 4; // bit 11-14
    uint32_t sentry_success_remote_hp_exchange_count : 4; // bit 15-18
    uint32_t sentry_can_free_revive : 1; // bit 19
    uint32_t sentry_can_immediate_revive : 1; // bit 20
    uint32_t sentry_immediate_revive_cost : 11; // bit 21-30
    uint32_t reserved_1 : 1; // bit 31

    // 第二个 16 位字段
    uint16_t sentry_is_out_of_combat : 1; // bit 0
    uint16_t team_17mm_ammo_remaining : 11; // bit 1-11
    uint16_t reserved_2 : 4; // bit 12-15
} sentry_info_t;


 /* ID: 0x0307  Byte:  103    哨兵发送小地图数据 */
 typedef  struct 
 { 
 uint8_t intention; 
 uint16_t start_position_x; 
 uint16_t start_position_y; 
 int8_t delta_x[49]; 
 int8_t delta_y[49]; 
 uint16_t sender_id; 
 }map_data_t;
 
 /****************************机器人交互数据****************************/
 /****************************机器人交互数据****************************/
 /* 发送的内容数据段最大为 113 检测是否超出大小限制?实际上图形段不会超，数据段最多30个，也不会超*/
 /* 交互数据头结构 */
 typedef struct
 {
     uint16_t data_cmd_id; // 由于存在多个内容 ID，但整个cmd_id 上行频率最大为 10Hz，请合理安排带宽。注意交互部分的上行频率
     uint16_t sender_ID;
     uint16_t receiver_ID;
 } ext_student_interactive_header_data_t;
 
 /* 机器人id */
 typedef enum
 {
     // 红方机器人ID
     RobotID_RHero = 1,
     RobotID_REngineer = 2,
     RobotID_RStandard1 = 3,
     RobotID_RStandard2 = 4,
     RobotID_RStandard3 = 5,
     RobotID_RAerial = 6,
     RobotID_RSentry = 7,
     RobotID_RRadar = 9,
     // 蓝方机器人ID
     RobotID_BHero = 101,
     RobotID_BEngineer = 102,
     RobotID_BStandard1 = 103,
     RobotID_BStandard2 = 104,
     RobotID_BStandard3 = 105,
     RobotID_BAerial = 106,
     RobotID_BSentry = 107,
     RobotID_BRadar = 109,
 } Robot_ID_e;
 
 /* 交互数据ID */
 typedef enum
 {
     UI_Data_ID_Del = 0x100,
     UI_Data_ID_Draw1 = 0x101,
     UI_Data_ID_Draw2 = 0x102,
     UI_Data_ID_Draw5 = 0x103,
     UI_Data_ID_Draw7 = 0x104,
     UI_Data_ID_DrawChar = 0x110,
     Sentinel_Autonomous_Instructions =0x120,
 
     /* 自定义交互数据部分 */
     Communicate_Data_ID = 0x0200,
 
 } Interactive_Data_ID_e;
 /* 交互数据长度 */
 typedef enum
 {
     Interactive_Data_LEN_Head = 6,
     UI_Operate_LEN_Del = 2,
     UI_Operate_LEN_PerDraw = 15,
     UI_Operate_LEN_DrawChar = 15 + 30,
     Sentinel_Autonomous_Instructions_LEN = 4,
 
     /* 自定义交互数据部分 */
     // Communicate_Data_LEN = 5,
 
 } Interactive_Data_Length_e;
 
 /****************************自定义交互数据****************************/
 /*
     学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
     自定义交互数据 机器人间通信：0x0301。
     发送频率：上限 10Hz
 */
 // 自定义交互数据协议，可更改，更改后需要修改最上方宏定义数据长度的值
 typedef struct
 {
     uint8_t data[Communicate_Data_LEN]; // 数据段,n需要小于113
 } robot_interactive_data_t;
 
 // 机器人交互信息_发送
 typedef struct
 {
     xFrameHeader FrameHeader;
     uint16_t CmdID;
     ext_student_interactive_header_data_t datahead;
     robot_interactive_data_t Data; // 数据段
     uint16_t frametail;
 } Communicate_SendData_t;
 // 机器人交互信息_接收
 typedef struct
 {
     ext_student_interactive_header_data_t datahead;
     robot_interactive_data_t Data; // 数据段
 } Communicate_ReceiveData_t;
 
 /****************************UI交互数据****************************/
 
 /* 图形数据 */
 typedef struct
 {
     uint8_t graphic_name[3];
     uint32_t operate_tpye : 3;
     uint32_t graphic_tpye : 3;
     uint32_t layer : 4;
     uint32_t color : 4;
     uint32_t start_angle : 9;
     uint32_t end_angle : 9;
     uint32_t width : 10;
     uint32_t start_x : 11;
     uint32_t start_y : 11;
     uint32_t radius : 10;
     uint32_t end_x : 11;
     uint32_t end_y : 11;
 } Graph_Data_t;
 
 typedef struct
 {
     Graph_Data_t Graph_Control;
     uint8_t show_Data[30];
 } String_Data_t; // 打印字符串数据
 
 /* 删除操作 */
 typedef enum
 {
     UI_Data_Del_NoOperate = 0,
     UI_Data_Del_Layer = 1,
     UI_Data_Del_ALL = 2, // 删除全部图层，后面的参数已经不重要了。
 } UI_Delete_Operate_e;
 
 /* 图形配置参数__图形操作 */
 typedef enum
 {
     UI_Graph_ADD = 1,
     UI_Graph_Change = 2,
     UI_Graph_Del = 3,
 } UI_Graph_Operate_e;
 
 /* 图形配置参数__图形类型 */
 typedef enum
 {
     UI_Graph_Line = 0,		// 直线
     UI_Graph_Rectangle = 1, // 矩形
     UI_Graph_Circle = 2,	// 整圆
     UI_Graph_Ellipse = 3,	// 椭圆
     UI_Graph_Arc = 4,		// 圆弧
     UI_Graph_Float = 5,		// 浮点型
     UI_Graph_Int = 6,		// 整形
     UI_Graph_Char = 7,		// 字符型
 
 } UI_Graph_Type_e;
 
 /* 图形配置参数__图形颜色 */
 typedef enum
 {
     UI_Color_Main = 0, // 红蓝主色
     UI_Color_Yellow = 1,
     UI_Color_Green = 2,
     UI_Color_Orange = 3,
     UI_Color_Purplish_red = 4, // 紫红色
     UI_Color_Pink = 5,
     UI_Color_Cyan = 6, // 青色
     UI_Color_Black = 7,
     UI_Color_White = 8,
 
 } UI_Graph_Color_e;
 
 #pragma pack()
 
 #endif
 
