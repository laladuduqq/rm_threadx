/*
 * @Author: laladuduqq 17503181697@163.com
 * @Date: 2025-06-06 21:31:47
 * @LastEditors: laladuduqq 17503181697@163.com
 * @LastEditTime: 2025-06-13 11:15:06
 * @FilePath: \rm_threadx\applications\robot_init.c
 * @Description: 
 * 
 */
#include "BMI088.h"
#include "RGB.h"
#include "SEGGER_RTT.h"
#include "chassis/chassiscmd.h"
#include "dm_imu.h"
#include "dwt.h"
#include "elog.h"
#include "gimbal/gimbalcmd.h"
#include "imu.h"
#include "iwdg.h"
#include "motor_task.h"
#include "offline.h"
#include "referee.h"
#include "robot_control/robot_task.h"
#include "sbus.h"
#include "shoot/shootcmd.h"
#include "systemwatch.h"
#include "robotdef.h"
#include "usb_user.h"


void base_init(void)
{
    RGB_init();
    DWT_Init(168);
    SEGGER_RTT_Init();
    if (elog_user_init() == ELOG_NO_ERR) 
    { elog_start();}
    BMI088_init();
    //reset cdc_acm
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    DWT_Delay(0.1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    DWT_Delay(0.1);
    #if defined (GIMBAL_BOARD)
    extern void cdc_acm_init(uint8_t busid, uintptr_t reg_base);
    cdc_acm_init(0, USB_OTG_FS_PERIPH_BASE);
    usb_user_init();
    #endif
}
void robot_init(TX_BYTE_POOL *pool)
{
    SystemWatch_Init(pool);
    offline_init(pool);
    INS_TASK_init(pool);
    #if defined (GIMBAL_BOARD)
        DM_IMU_Init(pool);
        #if CONTROL_SOURCE == 1
            Remote_init();
        #endif
    #else
    RefereeInit(pool);
    #endif
    motor_task_init(pool);
    robot_control_task_init(pool);
    #if defined (GIMBAL_BOARD)
        gimbal_task_init(pool);
        shoot_task_init(pool);
    #else
    chassis_task_init(pool);
    #endif
}
