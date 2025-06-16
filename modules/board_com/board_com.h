/*
 * @Author: laladuduqq 17503181697@163.com
 * @Date: 2025-06-06 23:35:26
 * @LastEditors: laladuduqq 17503181697@163.com
 * @LastEditTime: 2025-06-15 14:07:59
 * @FilePath: \rm_threadx\modules\board_com\board_com.h
 * @Description: 
 * 
 */
#ifndef __BOARD_COM_H
#define __BOARD_COM_H

#include "bsp_can.h"
#include "offline.h"
#include <stdint.h>


#define MAX_PACKET_SIZE 64    // 最大数据包大小
#define FRAME_HEADER 0xAA     // 帧头
#define CAN_COMM_OFFSET_BYTES 4  // 帧头长度等占用字节数

// 接收状态枚举
typedef enum {
    PACKET_IDLE,      // 空闲状态
    PACKET_RECEIVING, // 接收中
    PACKET_COMPLETE,  // 接收完成
    PACKET_ERROR     // 接收错误
} PacketStatus_e;

// 接收缓存结构
typedef struct {
    uint8_t raw_buf[2][MAX_PACKET_SIZE + CAN_COMM_OFFSET_BYTES];  // 原始接收双缓冲区
    volatile uint8_t recv_buf_index;     // 当前使用的接收缓冲区索引
    uint8_t unpacked_data[MAX_PACKET_SIZE];                    // 解包后的数据
    uint8_t recv_len;       // 当前已接收长度
    uint8_t total_len;      // 期望接收的总长度
    uint8_t current_seq;    // 当前包序号
    uint8_t expected_seq;   //期望收到的包序号
    uint8_t total_seq;      // 总包数
    PacketStatus_e status;  // 接收状态
} PacketBuffer_t;

// 板间通信结构体
typedef struct {
    Can_Device *candevice;
    uint8_t offlinemanage_index;
    PacketBuffer_t rx_buffer;      // 接收缓冲区
    uint8_t total_send_seq;        // 总发送序号
    uint8_t current_send_seq;      // 发送序号
} board_com_t;

typedef struct
{
    Can_Device_Init_Config_s Can_Device_Init_Config;
    OfflineDeviceInit_t offline_manage_init;
} board_com_init_t;

/**
 * @description: 板件通讯初始化
 * @param {board_com_init_t*} board_com_init
 * @return *board_com_t
 */
board_com_t *board_com_init(board_com_init_t* board_com_init);
/**
 * @description: 板件通讯发送函数
 * @param {uint8_t} *data
 * @param {uint8_t} size
 * @return {*}
 */
void board_com_send(uint8_t *data, uint8_t size);
/**
 * @description: 获取板件通讯数据，注意返回的是数组指针
 * @return {*}
 */
void *board_com_get_data(void);

#endif // __BOARD_COM_H
