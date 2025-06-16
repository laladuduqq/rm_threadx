#include "referee.h"
#include "bsp_uart.h"
#include "crc_rm.h"
#include "dwt.h"
#include "offline.h"
#include "referee_protocol.h"
#include "systemwatch.h"
#include "tx_port.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>
#include "tx_api.h"
#include "robotdef.h"
 
#define LOG_TAG "referee"
#define LOG_LVL LOG_LVL_DBG
#include <elog.h>

static referee_info_t referee_info;			  // 裁判系统数据
uint8_t UI_Seq;
static TX_THREAD refereeTask_recv_thread;
static TX_THREAD refereeTask_send_thread;
static uint8_t referee_buf[2][1024];

void RefereeTask(ULONG thread_input);
void referee_send_task(ULONG thread_input);
void JudgeReadData(uint8_t *buff);
void DeterminRobotID(void);
void RefereeInit(TX_BYTE_POOL *pool)
{   
    OfflineDeviceInit_t offline_init = {
        .name = "referee",
        .timeout_ms = 500,
        .level = OFFLINE_LEVEL_HIGH,
        .beep_times = 6,
        .enable = OFFLINE_ENABLE
    };
    
    referee_info.offline_index = offline_device_register(&offline_init);

    UART_Device_init_config uart6_cfg = {
        .huart = &huart6,
        .expected_len = 0,       // 不定长
        .rx_buf_size = 1024,
        .rx_buf = (uint8_t (*)[2])referee_buf,
        .rx_mode = UART_MODE_DMA,
        .tx_mode = UART_MODE_DMA,
        .timeout = 1000,
        .rx_complete_cb = NULL,
        .cb_type = UART_CALLBACK_EVENT,
        .event_flag = UART_RX_DONE_EVENT
    };
    UART_Device* uart6 = UART_Init(&uart6_cfg);
    referee_info.uart_device = uart6;

    // 用内存池分配监控线程栈
    CHAR *referee_recv_thread_stack;
    CHAR *referee_send_thread_stack;
    if (tx_byte_allocate(pool, (VOID **)&referee_recv_thread_stack, 1024, TX_NO_WAIT) != TX_SUCCESS) {
        log_e("Failed to allocate stack for refereeTask!");
        return;
    }
    DWT_Delay(0.5);
    if (tx_byte_allocate(pool, (VOID **)&referee_send_thread_stack, 1024, TX_NO_WAIT) != TX_SUCCESS) {
        log_e("Failed to allocate stack for refereesendTask!");
        return;
    }

    UINT status = tx_thread_create(&refereeTask_recv_thread, "refereeTask", RefereeTask, 0,referee_recv_thread_stack, 1024, 7, 7, TX_NO_TIME_SLICE, TX_AUTO_START);
         status = tx_thread_create(&refereeTask_send_thread, "refereesendTask", referee_send_task, 0,referee_send_thread_stack, 1024, 12, 12, TX_NO_TIME_SLICE, TX_AUTO_START);

    if(status != TX_SUCCESS) {
        log_e("Failed to create referee task!");
        return;
    }
}

void RefereeTask(ULONG thread_input)
{ 
        (void)thread_input;
        //SystemWatch_RegisterTask(&refereeTask_recv_thread, "RefereeTask");
        while (1) {
            //SystemWatch_ReportTaskAlive(&refereeTask_recv_thread);
            ULONG actual_flags = 0;
            // 等待事件标志，自动清除
            UINT status = tx_event_flags_get(
                &referee_info.uart_device->rx_event, // 事件组
                UART_RX_DONE_EVENT,
                TX_OR_CLEAR,      // 自动清除
                &actual_flags,
                TX_WAIT_FOREVER   // 无限等待
            );
            if ((status == TX_SUCCESS) && (actual_flags & UART_RX_DONE_EVENT)) {
                JudgeReadData(*referee_info.uart_device->rx_buf);
            }
            tx_thread_sleep(10);
        }
}

void referee_send_task(ULONG thread_input){
    (void)thread_input;
    SystemWatch_RegisterTask(&refereeTask_send_thread, "referee_send_task");
    while (1) 
    {
        SystemWatch_ReportTaskAlive(&refereeTask_send_thread);
        // 烧饼自动复活
        if (referee_info.sentry_info.sentry_can_free_revive ==1) {Sentry_Free_Revive();};
        tx_thread_sleep(100); 
    }
}

 /**
  * @brief  读取裁判数据,中断中读取保证速度
  * @param  buff: 读取到的裁判系统原始数据
  * @retval 是否对正误判断做处理
  * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
  */
void JudgeReadData(uint8_t *buff)
{
    uint16_t judge_length; // 统计一帧数据长度
    if (buff == NULL)	   // 空数据包，则不作任何处理
        return;
 
    // 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
    memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);
 
    // 判断帧头数据(0)是否为0xA5
    if (buff[SOF] == REFEREE_SOF)
    {
        // 帧头CRC8校验
         if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == RM_TRUE)
        {
            offline_device_update(referee_info.offline_index);
             // 统计一帧数据长度(byte),用于CR16校验
             judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
             // 帧尾CRC16校验
             if (Verify_CRC16_Check_Sum(buff, judge_length) == RM_TRUE)
             {
                 // 2个8位拼成16位int
                referee_info.CmdID = (buff[6] << 8 | buff[5]);
                 // 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
                 // 第8个字节开始才是数据 data=7
                switch (referee_info.CmdID)
                {
                 case ID_game_state: // 0x0001
                    memcpy(&referee_info.GameState, (buff + DATA_Offset), LEN_game_state);
                    break;
                 case ID_game_result: // 0x0002
                    memcpy(&referee_info.GameResult, (buff + DATA_Offset), LEN_game_result);
                    break;
                 case ID_game_robot_survivors: // 0x0003
                    memcpy(&referee_info.GameRobotHP, (buff + DATA_Offset), LEN_game_robot_HP);
                    break;
                 case ID_event_data: // 0x0101
                    memcpy(&referee_info.EventData, (buff + DATA_Offset), LEN_event_data);
                    break;
                 case ID_supply_projectile_action: // 0x0102
                    memcpy(&referee_info.SupplyProjectileAction, (buff + DATA_Offset), LEN_supply_projectile_action);
                    break;
                 case ID_game_robot_state: // 0x0201
                    memcpy(&referee_info.GameRobotState, (buff + DATA_Offset), LEN_game_robot_state);
                    if (referee_info.init_flag == 0)
                    {
                        DeterminRobotID();
                        referee_info.init_flag=1;
                    }
                    break;
                 case ID_power_heat_data: // 0x0202
                    memcpy(&referee_info.PowerHeatData, (buff + DATA_Offset), LEN_power_heat_data);
                    break;
                 case ID_game_robot_pos: // 0x0203
                    memcpy(&referee_info.GameRobotPos, (buff + DATA_Offset), LEN_game_robot_pos);
                    break;
                 case ID_buff_musk: // 0x0204
                    memcpy(&referee_info.BuffMusk, (buff + DATA_Offset), LEN_buff_musk);
                    break;
                 case ID_aerial_robot_energy: // 0x0205
                    memcpy(&referee_info.AerialRobotEnergy, (buff + DATA_Offset), LEN_aerial_robot_energy);
                    break;
                 case ID_robot_hurt: // 0x0206
                    memcpy(&referee_info.RobotHurt, (buff + DATA_Offset), LEN_robot_hurt);
                    break;
                 case ID_shoot_data: // 0x0207
                    memcpy(&referee_info.ShootData, (buff + DATA_Offset), LEN_shoot_data);
                    break;
                 case ID_shoot_remaining:
                    memcpy(&referee_info.ext_shoot_remaing, (buff + DATA_Offset), LEN_shoot_remaing);
                    break;
                 case ID_rfid_status:
                    memcpy(&referee_info.rfid_status, (buff + DATA_Offset), LEN_rfid_status);
                    break;
                 case ID_ground_robot_position :
                    memcpy(&referee_info.ground_robot_position, (buff + DATA_Offset), LEN_ground_robot_position);
                    break;
                 case ID_sentry_info:
                    memcpy(&referee_info.sentry_info, (buff + DATA_Offset), LEN_sentry_info);
                    break;
                }
            }
        }
        // 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
        if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
        { // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
             JudgeReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL);
        }
    }
}

void RefereeSend(uint8_t *send, uint16_t tx_len){
    UART_Send(referee_info.uart_device, send, tx_len);
}
 
void DeterminRobotID(void)
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_info.referee_id.Robot_Color = referee_info.GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_info.referee_id.Robot_ID = referee_info.GameRobotState.robot_id;
    referee_info.referee_id.Cilent_ID = 0x0100 + referee_info.referee_id.Robot_ID; // 计算客户端ID
    referee_info.referee_id.Receiver_Robot_ID = 0;
}
void Sentry_Free_Revive(void)
{
    Communicate_SendData_t Communicate_SendData;

	uint8_t temp_datalength = Interactive_Data_LEN_Head + Sentinel_Autonomous_Instructions_LEN; // 计算交互数据长度

	Communicate_SendData.FrameHeader.SOF = REFEREE_SOF;
	Communicate_SendData.FrameHeader.DataLength = temp_datalength;
	Communicate_SendData.FrameHeader.Seq = UI_Seq;
	Communicate_SendData.FrameHeader.CRC8 = Get_CRC8_Check_Sum((uint8_t *)&Communicate_SendData, LEN_CRC8, 0xFF);

	Communicate_SendData.CmdID = ID_student_interactive;

	Communicate_SendData.datahead.data_cmd_id = Sentinel_Autonomous_Instructions;
	Communicate_SendData.datahead.receiver_ID = 0x8080; // 发送给裁判系统
	Communicate_SendData.datahead.sender_ID = referee_info.referee_id.Robot_ID; // 发送者ID

    Communicate_SendData.Data.data[0] = 0x01; // 1:免费复活

	Communicate_SendData.frametail = Get_CRC16_Check_Sum((uint8_t *)&Communicate_SendData, LEN_HEADER + LEN_CMDID + temp_datalength, 0xFFFF);

	RefereeSend((uint8_t *)&Communicate_SendData, LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL); // 发送

	UI_Seq++; // 包序号+1
}

/**
 * @brief 根据命令码获取对应的裁判系统数据
 * @param referee 裁判系统数据结构体指针
 * @param cmd_id 命令码
 * @param data_length 返回数据长度的指针
 * @return const void* 返回对应数据的指针，如果命令码无效则返回NULL
 */
const void* GetRefereeDataByCmd(CmdID_e cmd_id, uint16_t* data_length)
{
    if(data_length == NULL) {
        return NULL;
    }

    switch(cmd_id)
    {
        case ID_game_state:
            *data_length = sizeof(ext_game_state_t);
            return &referee_info.GameState;

        case ID_game_result:
            *data_length = sizeof(ext_game_result_t);
            return &referee_info.GameResult;

        case ID_game_robot_survivors:
            *data_length = sizeof(ext_game_robot_HP_t);
            return &referee_info.GameRobotHP;

        case ID_event_data:
            *data_length = sizeof(ext_event_data_t);
            return &referee_info.EventData;

        case ID_supply_projectile_action:
            *data_length = sizeof(ext_supply_projectile_action_t);
            return &referee_info.SupplyProjectileAction;

        case ID_game_robot_state:
            *data_length = sizeof(ext_game_robot_state_t);
            return &referee_info.GameRobotState;

        case ID_power_heat_data:
            *data_length = sizeof(ext_power_heat_data_t);
            return &referee_info.PowerHeatData;

        case ID_game_robot_pos:
            *data_length = sizeof(ext_game_robot_pos_t);
            return &referee_info.GameRobotPos;

        case ID_buff_musk:
            *data_length = sizeof(ext_buff_musk_t);
            return &referee_info.BuffMusk;

        case ID_aerial_robot_energy:
            *data_length = sizeof(aerial_robot_energy_t);
            return &referee_info.AerialRobotEnergy;

        case ID_robot_hurt:
            *data_length = sizeof(ext_robot_hurt_t);
            return &referee_info.RobotHurt;

        case ID_shoot_data:
            *data_length = sizeof(ext_shoot_data_t);
            return &referee_info.ShootData;

        case ID_shoot_remaining:
            *data_length = sizeof(ext_shoot_remaing_t);
            return &referee_info.ext_shoot_remaing;

        case ID_rfid_status:
            *data_length = sizeof(rfid_status_t);
            return &referee_info.rfid_status;

        case ID_ground_robot_position:
            *data_length = sizeof(ground_robot_position_t);
            return &referee_info.ground_robot_position;

        case ID_sentry_info:
            *data_length = sizeof(sentry_info_t);
            return &referee_info.sentry_info;

        case ID_map_data:
            *data_length = sizeof(map_data_t);
            return &referee_info.map_data;

        default:
            *data_length = 0;
            return NULL;
    }
}


void referee_to_gimbal(Chassis_referee_Upload_Data_s *Chassis_referee_Upload_Data){
    Chassis_referee_Upload_Data->Robot_Color = 
    (referee_info.referee_id.Robot_ID <= 7) ? 0 : 
    (referee_info.referee_id.Robot_ID > 100) ? 1 : 0;  // 默认为红方
    
    Chassis_referee_Upload_Data->current_hp_percent = referee_info.GameRobotState.current_HP;
    
    Chassis_referee_Upload_Data->outpost_HP = 
    (referee_info.referee_id.Robot_ID <= 7) ? referee_info.GameRobotHP.red_outpost_HP: 
    (referee_info.referee_id.Robot_ID > 100) ? referee_info.GameRobotHP.blue_outpost_HP : referee_info.GameRobotHP.red_outpost_HP;  // 默认为红方

    Chassis_referee_Upload_Data->base_HP = 
    (referee_info.referee_id.Robot_ID <= 7) ? referee_info.GameRobotHP.red_base_HP: 
    (referee_info.referee_id.Robot_ID > 100) ? referee_info.GameRobotHP.blue_base_HP : referee_info.GameRobotHP.red_base_HP;  // 默认为红方

    Chassis_referee_Upload_Data->game_progess = referee_info.GameState.game_progress;
    
    Chassis_referee_Upload_Data->game_time = referee_info.GameState.stage_remain_time;

    Chassis_referee_Upload_Data->projectile_allowance_17mm = referee_info.ext_shoot_remaing.projectile_allowance_17mm;

    Chassis_referee_Upload_Data->power_management_shooter_output = referee_info.GameRobotState.power_management_shooter_output;
}
