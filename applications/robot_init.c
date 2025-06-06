#include "BMI088.h"
#include "RGB.h"
#include "SEGGER_RTT.h"
#include "dwt.h"
#include "elog.h"
#include "imu.h"
#include "iwdg.h"
#include "offline.h"
#include "referee.h"
#include "sbus.h"
#include "systemwatch.h"
#include "robotdef.h"


void base_init(void)
{
    __HAL_DBGMCU_FREEZE_IWDG();
    RGB_init();
    DWT_Init(168);
    SEGGER_RTT_Init();
    if (elog_user_init() == ELOG_NO_ERR) 
    { elog_start();}
    BMI088_init();
}
void robot_init(TX_BYTE_POOL *pool)
{
    SystemWatch_Init(pool);
    offline_init(pool);
    INS_TASK_init(pool);
    #if defined (GIMBAL_BOARD)
        #if CONTROL_SOURCE == 1
            //Remote_init();
            RefereeInit(pool);
        #endif
    #else
    RefereeInit();
    #endif
    MX_IWDG_Init();
}
