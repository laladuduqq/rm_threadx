/*
 * @Author: laladuduqq 17503181697@163.com
 * @Date: 2025-06-08 00:35:28
 * @LastEditors: laladuduqq 17503181697@163.com
 * @LastEditTime: 2025-06-12 23:56:23
 * @FilePath: \rm_threadx\applications\robot_config.h
 * @Description: 
 * 
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "tx_api.h"
#include <stddef.h>
//这里是调试和一些关键功能比如内存分配释放等统一上层开关
//注意：如果需要使用某个功能，请在robot_config.h中打开对应的宏定义

//easylogger
#define ROBOT_CONFIG_DEBUG 1 // 开启调试模式  //注意这里只会改变log的输出级别，log_e及以上不受该定义影响

//systemwatch
#define SystemWatch_Enable 1 // 开启系统监控功能  //注意下述两个宏定义只有在开启系统监控功能时才会生效
#define SystemWatch_Reset_Enable 1 // 开启系统监控重置功能
#define SystemWatch_Iwdg_Enable 1 // 开启看门狗功能

//offline
#define OFFLINE_Enable 1 // 开启离线检测功能
#define OFFLINE_Beep_Enable 0 // 开启离线蜂鸣器功能

//rtos内存分配函数
void* threadx_malloc(size_t size);
void threadx_free(void *ptr);

//调试参数
void Print_BytePool_Info(TX_BYTE_POOL *pool, const char *name);


#endif // ROBOT_CONFIG_H
