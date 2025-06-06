#include "tx_api.h"
#include "robot_control.h"
#include "systemwatch.h"
#include "robot_task.h"

#define LOG_TAG              "robottask"
#define LOG_LVL              LOG_LVL_DBG
#include <elog.h>

static TX_THREAD robotTask_thread;

void robotcmdtask(ULONG thread_input)
{
    (void)thread_input; 
    robot_control_init();
    SystemWatch_RegisterTask(&robotTask_thread, "robotTask");
    for (;;)
    {
        SystemWatch_ReportTaskAlive(&robotTask_thread);
        robot_control();  
        tx_thread_sleep(3);
    }
}

void robot_control_task_init(TX_BYTE_POOL *pool){
    // 用内存池分配监控线程栈
    CHAR *robot_thread_stack;
    if (tx_byte_allocate(pool, (VOID **)&robot_thread_stack, 1024, TX_NO_WAIT) != TX_SUCCESS) {
        log_e("Failed to allocate stack for robotTask!");
        return;
    }

    UINT status = tx_thread_create(&robotTask_thread, "robotTask", robotcmdtask, 0,robot_thread_stack, 1024, 8, 8, TX_NO_TIME_SLICE, TX_AUTO_START);

    if(status != TX_SUCCESS) {
        log_e("Failed to create robot task!");
        return;
    }
    log_i("robot_control task created");
}

