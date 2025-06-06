#include "motor_task.h"
#include "damiao.h"
#include "dji.h"
#include "systemwatch.h"
#include "tx_api.h"

#define LOG_TAG  "motortask"
#include "elog.h"

static TX_THREAD motorTask_thread;

void motortask(ULONG thread_input)
{
    (void)thread_input; 
    SystemWatch_RegisterTask(&motorTask_thread, "motorTask");
    for (;;)
    {
        SystemWatch_ReportTaskAlive(&motorTask_thread);
        DJIMotorControl();
        DMMotorcontrol();
        tx_thread_sleep(2);
    }
}

void motor_task_init(TX_BYTE_POOL *pool){
    // 用内存池分配监控线程栈
    CHAR *motor_thread_stack;
    if (tx_byte_allocate(pool, (VOID **)&motor_thread_stack, 1024, TX_NO_WAIT) != TX_SUCCESS) {
        log_e("Failed to allocate stack for motorTask!");
        return;
    }

    UINT status = tx_thread_create(&motorTask_thread, "motorTask", motortask, 0,motor_thread_stack, 1024, 7, 7, TX_NO_TIME_SLICE, TX_AUTO_START);

    if(status != TX_SUCCESS) {
        log_e("Failed to create ins task!");
        return;
    }
    log_i("motorTask create success");
}
