#include "systemwatch.h"
#include "dwt.h"
#include "iwdg.h"
#include "stm32f4xx_hal_iwdg.h"
#include "tim.h"
#include <stdint.h>
#include <string.h>

#define LOG_TAG  "systemwatch"
#include "elog.h"

// 静态变量
static TaskMonitor_t taskList[MAX_MONITORED_TASKS];
static uint8_t taskCount = 0;
static TX_THREAD *watchTaskHandle;
static volatile uint32_t watch_task_last_active = 0;

// 辅助函数声明
static void PrintTaskInfo(TaskMonitor_t *pxTaskMonitor);
static void PrintSystemStatus(void);

static void SystemWatch_Task(ULONG thread_input)
{
    (void)thread_input;
    while(1) {
        HAL_IWDG_Refresh(&hiwdg);
        taskList[0].dt = DWT_GetDeltaT(&taskList[0].dt_cnt); //自身更新
        watch_task_last_active = tx_time_get();
        for(uint8_t i = 0; i < taskCount; i++) {
            if(taskList[i].isActive) {
                // 检查任务执行间隔是否过长
                if(taskList[i].dt > TASK_BLOCK_TIMEOUT) {
                    // ThreadX临界区
                    UINT old_posture = tx_interrupt_control(TX_INT_DISABLE);
                    
                    log_e("\r\n**** Task Blocked Detected! System State Dump ****");
                    DWT_Delay(0.005);
                    log_e("Time: %.3f s", DWT_GetTimeline_s());
                    DWT_Delay(0.005);
                    log_e("----------------------------------------");
                    DWT_Delay(0.005);

                    log_e("Blocked Task Information:");
                    DWT_Delay(0.005);
                    PrintTaskInfo(&taskList[i]);
                    DWT_Delay(0.5);

                    PrintSystemStatus();
                    DWT_Delay(0.5);
                    tx_interrupt_control(old_posture);
                    HAL_NVIC_SystemReset(); 
                }
            }
        }
        tx_thread_sleep(10);
    }
}



void SystemWatch_Init(TX_BYTE_POOL *pool)
{
    // 初始化任务列表
    memset(taskList, 0, sizeof(taskList));
    taskCount = 0;
    watch_task_last_active = tx_time_get();

    // 用内存池分配监控线程栈
    CHAR *watch_thread_stack;
    if (tx_byte_allocate(pool, (VOID **)&watch_thread_stack, 1024, TX_NO_WAIT) != TX_SUCCESS) {
        log_e("Failed to allocate stack for WatchTask!");
        return;
    }

    static TX_THREAD watch_thread;
    UINT status = tx_thread_create(&watch_thread, "WatchTask", SystemWatch_Task, 0,
                                   watch_thread_stack, 1024,
                                   3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);

    if(status != TX_SUCCESS) {
        log_e("Failed to create SystemWatch task!");
        return;
    }
    watchTaskHandle = &watch_thread;

    SystemWatch_RegisterTask(watchTaskHandle, "WatchTask");

    log_i("SystemWatch initialized, watch task created.");
}

static void PrintTaskInfo(TaskMonitor_t *pxTaskMonitor)
{
    log_e("Name: %s", pxTaskMonitor->name);
    log_e("Handle: 0x%p", (void*)&pxTaskMonitor->handle);
    log_e("Last dt: %.3f ms", pxTaskMonitor->dt * 1000.0f);
}

static void PrintSystemStatus(void)
{
    log_e("\r\nSystem Status:");
    log_e("----------------------------------------");
}


int8_t SystemWatch_RegisterTask(TX_THREAD *taskHandle, const char* taskName)
{
    if (taskHandle == NULL || taskName == NULL) {
        return -1;
    }
    // 检查任务是否已注册
    UINT old_posture = tx_interrupt_control(TX_INT_DISABLE);

    for(uint8_t i = 0; i < taskCount; i++) {
        if(taskList[i].handle == taskHandle) {
            tx_interrupt_control(old_posture);
            return -1;
        }
    }

    // 检查是否超过最大监控任务数
    if(taskCount >= MAX_MONITORED_TASKS) {
        tx_interrupt_control(old_posture);
        return -1;
    }

    TaskMonitor_t* newTask = &taskList[taskCount];
    newTask->handle = taskHandle;
    newTask->name = taskName;
    newTask->dt = DWT_GetDeltaT(&taskList[taskCount].dt_cnt);
    newTask->isActive = 1;

    taskCount++;

    tx_interrupt_control(old_posture);

    return 0;
}

void SystemWatch_ReportTaskAlive(TX_THREAD *taskHandle)
{
    for(uint8_t i = 0; i < taskCount; i++) {
        if(taskList[i].handle == taskHandle) {
            taskList[i].dt = DWT_GetDeltaT(&taskList[i].dt_cnt);
            HAL_IWDG_Refresh(&hiwdg);
            break;
        }
    }
}
