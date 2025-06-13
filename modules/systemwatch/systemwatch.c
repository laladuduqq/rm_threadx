#include "systemwatch.h"
#include "dwt.h"
#include "iwdg.h"
#include "stm32f4xx_hal_iwdg.h"
#include "robot_config.h"
#include <stdint.h>
#include <string.h>

#define LOG_TAG  "systemwatch"
#include "elog.h"

// 静态变量
static TaskMonitor_t taskList[MAX_MONITORED_TASKS];
static uint8_t taskCount = 0;
static TX_THREAD watchTaskHandle;

// 辅助函数声明
static void PrintTaskInfo(TaskMonitor_t *pxTaskMonitor);

static void SystemWatch_Task(ULONG thread_input)
{
    (void)thread_input;

    #if SystemWatch_Iwdg_Enable == 1
        __HAL_DBGMCU_FREEZE_IWDG();
        MX_IWDG_Init();
    #endif
    
    while(1) {
        HAL_IWDG_Refresh(&hiwdg);
        taskList[0].dt = DWT_GetDeltaT(&taskList[0].dt_cnt); //自身更新
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
                    tx_interrupt_control(old_posture);

                    #if SystemWatch_Reset_Enable == 1
                    HAL_NVIC_SystemReset(); 
                    #endif
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


    #if SystemWatch_Enable == 1
        // 用内存池分配线程栈
        CHAR *watch_thread_stack;
        if (tx_byte_allocate(pool, (VOID **)&watch_thread_stack, 1024, TX_NO_WAIT) != TX_SUCCESS) {
            log_e("Failed to allocate stack for WatchTask!");
            return;
        }

        UINT status = tx_thread_create(&watchTaskHandle, "WatchTask", SystemWatch_Task, 0,
                                    watch_thread_stack, 1024,
                                    3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);

        if(status != TX_SUCCESS) {
            log_e("Failed to create SystemWatch task!");
            return;
        }

        SystemWatch_RegisterTask(&watchTaskHandle, "WatchTask");

        log_i("SystemWatch initialized, watch task created.");
    #else 
        (void)pool;
        log_i("SystemWatch is disabled");
    #endif

}

static void PrintTaskInfo(TaskMonitor_t *pxTaskMonitor)
{
    log_e("Name: %s", pxTaskMonitor->name);
    log_e("Handle: 0x%p", (void*)pxTaskMonitor->handle);
    log_e("Last dt: %.3f ms", pxTaskMonitor->dt * 1000.0f);

    TX_THREAD *thread = pxTaskMonitor->handle;
    if (thread) {
        CHAR *stack_start = (CHAR *)thread->tx_thread_stack_start;
        ULONG stack_size = thread->tx_thread_stack_size;
        CHAR *stack_highest = (CHAR *)thread->tx_thread_stack_highest_ptr;

        // 注意：Cortex-M 架构栈向下增长
        ULONG stack_used = (ULONG)(stack_start + stack_size - stack_highest);
        ULONG stack_free = stack_size - stack_used;

        log_e("Stack start: 0x%p", stack_start);
        log_e("Stack size: %lu bytes", stack_size);
        log_e("Stack used: %lu bytes", stack_used);
        log_e("Stack free: %lu bytes", stack_free);
    } else {
        log_e("Thread pointer is NULL!");
    }
}


int8_t SystemWatch_RegisterTask(TX_THREAD *taskHandle, const char* taskName)
{
    #if SystemWatch_Enable == 1
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
    #else 
        (void)taskHandle;
        (void)taskName;
        return 0;
    #endif
}

void SystemWatch_ReportTaskAlive(TX_THREAD *taskHandle)
{
    #if SystemWatch_Enable == 1
        for(uint8_t i = 0; i < taskCount; i++) {
            if(taskList[i].handle == taskHandle) {
                taskList[i].dt = DWT_GetDeltaT(&taskList[i].dt_cnt);
                HAL_IWDG_Refresh(&hiwdg);
                break;
            }
        }
    #else
        (void)taskHandle;
    #endif
}
