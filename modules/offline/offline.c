#include "offline.h"
#include "RGB.h"
#include "dwt.h"
#include "systemwatch.h"
#include <stdint.h>
#include <string.h>
#include "tim.h"
#include "tx_api.h"
#include "robot_config.h"


#define LOG_TAG  "offline"
#include "elog.h"

// 静态变量
static OfflineManager_t offline_manager;
static TX_THREAD offline_thread;


//对于beep 部分定义
#define BEEP_PERIOD   2000  //注意这里的周期，由于在offline task(10ms)中,尽量保证整除
#define BEEP_ON_TIME  100   //这里BEEP_ON_TIME BEEP_OFF_TIME 共同影响在周期内的最大beep times（BEEP_PERIOD / （这里BEEP_ON_TIME + BEEP_OFF_TIME））
#define BEEP_OFF_TIME 100

#define BEEP_TUNE_VALUE 500  //这两个部分决定beep的音调，音色
#define BEEP_CTRL_VALUE 100

#define BEEP_TUNE        TIM4->ARR  //对应的tim的自动重装载值
#define BEEP_CTRL        TIM4->CCR3 //对应通道的比较值

static uint8_t beep_times;
// 内部函数声明
static void offline_task(ULONG thread_input);
static inline int32_t beep_set_times(uint8_t times);
static void beep_ctrl_times(void);
void beep_set_tune(uint16_t tune, uint16_t ctrl);

void offline_init(TX_BYTE_POOL *pool)
{
    // 初始化管理器
    memset(&offline_manager, 0, sizeof(offline_manager)); 

    #if OFFLINE_Enable == 1
        // 用内存池分配线程栈
        CHAR *offline_thread_stack;
        if (tx_byte_allocate(pool, (VOID **)&offline_thread_stack, 1024, TX_NO_WAIT) != TX_SUCCESS) {
            log_e("Failed to allocate stack for offlineTask!");
            return;
        }

        UINT status = tx_thread_create(&offline_thread, "offlineTask", offline_task, 0,
                                    offline_thread_stack, 1024,
                                    4, 4, TX_NO_TIME_SLICE, TX_AUTO_START);

        if(status != TX_SUCCESS) {
            log_e("Failed to create offline task!");
            return;
        }

        HAL_TIM_PWM_Start(&htim4,  TIM_CHANNEL_3);
        
        log_i("Offline task initialized successfully.");
    #else
        (void)pool;
        log_i("Offline is disabled.");
    #endif
}

void offline_task(ULONG thread_input)
{    
    (void)(thread_input);
    SystemWatch_RegisterTask(&offline_thread, "offline_task");
    for (;;)
    {
        SystemWatch_ReportTaskAlive(&offline_thread);

        static uint8_t beep_time = 0xFF;
        static uint8_t error_level = 0;
        static uint8_t display_index = OFFLINE_INVALID_INDEX;
        uint32_t current_time = tx_time_get();
        
        // 重置错误状态
        error_level = 0;
        display_index = OFFLINE_INVALID_INDEX;
        bool any_device_offline = false;
    
        // 检查所有设备状态
        for (uint8_t i = 0; i < offline_manager.device_count; i++) {
            OfflineDevice_t* device = &offline_manager.devices[i];
            
            if (!device->enable) {
                continue;
            }
    
            if (current_time - device->last_time > device->timeout_ms) {
                device->is_offline = STATE_OFFLINE;
                any_device_offline = true;
                
                // 更新最高优先级设备
                if (device->level > error_level) {
                    error_level = device->level;
                    display_index = i;
                    beep_time = device->beep_times;
                }
                // 相同优先级时的处理
                else if (device->level == error_level) {
                    // 如果当前设备不需要蜂鸣（beep_times=0），保持原来的设备
                    if (device->beep_times == 0) {
                        continue;
                    }
                    // 如果之前选中的设备不需要蜂鸣，或者当前设备蜂鸣次数更少
                    if (beep_time == 0 || 
                        (device->beep_times > 0 && device->beep_times < beep_time)) {
                        display_index = i;
                        beep_time = device->beep_times;
                    }
                }
            } else {
                device->is_offline = STATE_ONLINE;
            }
        }
    
        // 触发报警或清除报警
        if (display_index != OFFLINE_INVALID_INDEX && any_device_offline) {
            beep_set_times(offline_manager.devices[display_index].beep_times);
        } else {
            // 所有设备都在线，清除报警
            beep_set_times(0);
            beep_set_tune(0, 0);         // 立即关闭蜂鸣器
            RGB_show(LED_Green);              // 表示所有设备都在线
        }

        beep_ctrl_times();
        
        tx_thread_sleep(10);
    }
}

uint8_t offline_device_register(const OfflineDeviceInit_t* init)
{
    #if OFFLINE_Enable ==1
        if (init == NULL || offline_manager.device_count >= MAX_OFFLINE_DEVICES) {
            return OFFLINE_INVALID_INDEX;
        }
        
        uint8_t index = offline_manager.device_count;
        OfflineDevice_t* device = &offline_manager.devices[index];
        
        strncpy(device->name, init->name, sizeof(device->name) - 1);
        device->timeout_ms = init->timeout_ms;
        device->level = init->level;
        device->beep_times = init->beep_times;
        device->is_offline = STATE_OFFLINE;
        device->last_time = tx_time_get();
        device->index = index;
        device->enable = init->enable;
        
        offline_manager.device_count++;
        return index;
    #else
        (void) init;
        return OFFLINE_INVALID_INDEX;
    #endif

}

void offline_device_update(uint8_t device_index)
{
    #if OFFLINE_Enable ==1
        if (device_index < offline_manager.device_count) {
            offline_manager.devices[device_index].last_time = tx_time_get();
            offline_manager.devices[device_index].dt = DWT_GetDeltaT(&offline_manager.devices[device_index].dt_cnt);
        }
    #else
        (void)device_index;
    #endif
}

void offline_device_enable(uint8_t device_index)
{
    #if OFFLINE_Enable ==1
        if (device_index < offline_manager.device_count) {
            offline_manager.devices[device_index].enable = OFFLINE_ENABLE;
        }
    #else
        (void)device_index;
    #endif
}

void offline_device_disable(uint8_t device_index)
{
    #if OFFLINE_Enable ==1
        if (device_index < offline_manager.device_count) {
            offline_manager.devices[device_index].enable = OFFLINE_DISABLE;
        }
    #else
        (void)device_index;
    #endif
}
uint8_t get_device_status(uint8_t device_index){
    #if OFFLINE_Enable ==1
        if(device_index < offline_manager.device_count){
            return offline_manager.devices[device_index].is_offline;
        }
        else {return STATE_ONLINE;}
    #else
        (void)device_index;
        return STATE_ONLINE;
    #endif
}

uint8_t get_system_status(void){
    uint8_t status = 0;

    #if OFFLINE_Enable ==1
        for (uint8_t i = 0; i < offline_manager.device_count; i++) {
            if (offline_manager.devices[i].is_offline) {
                status |= (1 << i);
            }
        }
    #endif

    return status;
}


// 设置蜂鸣器状态

int32_t beep_set_times(uint8_t times)
{
    beep_times = times;
    return 0;
}

void beep_set_tune(uint16_t tune, uint16_t ctrl)
{
    BEEP_TUNE = tune;
    BEEP_CTRL = ctrl;
}

/**
  * @brief  called by cycle, control beep times.(one BEEP_PERIOD)
  * @param  NULL
  * @retval
  */
void beep_ctrl_times(void)
{
    static uint32_t beep_tick;
    static uint32_t times_tick;
    static uint8_t times;

    if (DWT_GetTimeline_ms() - beep_tick > BEEP_PERIOD)
    {
        times = beep_times;
        beep_tick = DWT_GetTimeline_ms();
        times_tick = DWT_GetTimeline_ms();
    }
    else if (times != 0)
    {
        if (DWT_GetTimeline_ms() - times_tick < BEEP_ON_TIME)
        {
            #if OFFLINE_Beep_Enable == 1
            beep_set_tune(BEEP_TUNE_VALUE, BEEP_CTRL_VALUE);
            #else
            beep_set_tune(0, 0);
            #endif
            RGB_show(LED_Red);
        }
        else if (DWT_GetTimeline_ms() - times_tick < BEEP_ON_TIME + BEEP_OFF_TIME)
        {
            beep_set_tune(0, 0);
            RGB_show(LED_Black);
        }
        else
        {
            times--;
            times_tick = DWT_GetTimeline_ms();
        }
    }
}
