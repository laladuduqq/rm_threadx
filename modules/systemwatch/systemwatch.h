/*
 * @Author: laladuduqq 17503181697@163.com
 * @Date: 2025-06-06 18:47:55
 * @LastEditors: laladuduqq 17503181697@163.com
 * @LastEditTime: 2025-06-11 22:55:41
 * @FilePath: \rm_threadx\modules\systemwatch\systemwatch.h
 * @Description: 
 * 
 */
#ifndef __SYSTEMWATCH_H__
#define __SYSTEMWATCH_H__

#include <stdint.h>
#include "tx_api.h"

// 最大监控任务数
#define MAX_MONITORED_TASKS 10
// 任务阻塞判定时间 (s)
#define TASK_BLOCK_TIMEOUT 1

typedef struct {
    TX_THREAD* handle;        //任务句柄
    const char* name;        //任务名称
    uint8_t isActive;       //是否在监控
    float dt;              // 更新间隔
    uint32_t dt_cnt;
} TaskMonitor_t;

// 初始化系统监控
/**
 * @description: systemwatch 初始化
 * @param {TX_BYTE_POOL} *pool
 * @return {*}
 */
void SystemWatch_Init(TX_BYTE_POOL *pool);
/**
 * @description: 注册需要监控的任务
 * @param {TX_THREAD} *taskHandle
 * @param {char*} taskName
 * @return {int8_t} 返回值: 0-成功, -1-失败
 */
int8_t SystemWatch_RegisterTask(TX_THREAD *taskHandle, const char* taskName);
/**
 * @description: 在被监控的任务中调用此函数更新计数器
 * @param {TX_THREAD} *taskHandle
 * @return {*}
 */
void SystemWatch_ReportTaskAlive(TX_THREAD *taskHandle);

#endif /* __SYSTEMWATCH_H__ */
