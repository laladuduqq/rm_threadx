/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2015, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2015-04-28
 */
 

#include "elog.h"
#include <stdint.h>
#include <stdio.h>
#include "SEGGER_RTT.h"
#include "tx_api.h"


/* 定义互斥量对象 */
static TX_MUTEX elog_mutex;

/* 互斥锁用于日志输出保护 */
/**
 * EasyLogger port initialize
 *
 * @return status
 */
ElogErrCode elog_port_init(void) {
    /* 创建互斥量 */
    if (tx_mutex_create(&elog_mutex, "elog_mutex", TX_INHERIT) != TX_SUCCESS) {
        return ELOG_ERR_INIT_FAILED;
    }
    return ELOG_NO_ERR;
}

/**
 * EasyLogger port deinitialize
 *
 */
void elog_port_deinit(void) {

    /* add your code here */
    /* 删除互斥量 */
    tx_mutex_delete(&elog_mutex);
}

/**
 * output log port interface
 *
 * @param log output of log
 * @param size log size
 */
void elog_port_output(const char *log, size_t size) {
        
    SEGGER_RTT_Write(0, log, size);
}

/**
 * output lock
 */
void elog_port_output_lock(void) {
    tx_mutex_get(&elog_mutex, TX_WAIT_FOREVER);
    /* add your code here */
}

/**
 * output unlock
 */
void elog_port_output_unlock(void) {
    tx_mutex_put(&elog_mutex);
    /* add your code here */
}

/**
 * get current time interface
 *
 * @return current time
 */
 const char *elog_port_get_time(void) {
    static char time[16] = {0};
    ULONG current_time;
    ULONG seconds;
    ULONG milliseconds;
    
    /* 获取系统滴答计数 */
    current_time = tx_time_get();
    
    /* 转换为秒和毫秒 
     * 注意：TX_TIMER_TICKS_PER_SECOND 是系统每秒的滴答数
     */
    seconds = current_time / TX_TIMER_TICKS_PER_SECOND;
    milliseconds = (current_time % TX_TIMER_TICKS_PER_SECOND) * 1000 / TX_TIMER_TICKS_PER_SECOND;
    
    /* 格式化时间字符串: [s.ms] */
    snprintf(time, sizeof(time), "[%04lu.%03lu]", seconds, milliseconds);
    
    return time;
}

/**
 * get current process name interface
 *
 * @return current process name
 */
const char *elog_port_get_p_info(void) {
    
    /* add your code here */
    return "";
}

/**
 * get current thread name interface
 *
 * @return current thread name
 */
 const char *elog_port_get_t_info(void) {
    static char name[64];
    TX_THREAD *current_thread;
    
    /* 获取当前线程控制块 */
    current_thread = tx_thread_identify();
    if (current_thread != NULL) {
        snprintf(name, sizeof(name), "%s", current_thread->tx_thread_name);
    } else {
        snprintf(name, sizeof(name), "unknown");
    }
    return name;
}
/* 在elog_user_init()之前声明 */
ElogErrCode elog_user_init(void) {
    ElogErrCode result;

    /* 初始化 EasyLogger */
    result = elog_init();
    if(result != ELOG_NO_ERR) {
        return result;
    }

    /* 设置 EasyLogger 输出格式 */
    elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
    elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_LVL | ELOG_FMT_TAG | ELOG_FMT_TIME);
    elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL & ~ELOG_FMT_FUNC);
    elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL & ~ELOG_FMT_FUNC);

    return ELOG_NO_ERR;
}
