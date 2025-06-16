#include "board_com.h"
#include "offline.h"
#include "stm32f4xx_hal_def.h"
#include <string.h>


#define LOG_TAG              "board_com"
#define LOG_LVL              LOG_LVL_DBG
#include <elog.h>


static board_com_t board_com;
static void board_recv(const CAN_HandleTypeDef* hcan, const uint32_t rx_id);

// 重置接收缓冲区
static void reset_rx_buffer(void)
{
    board_com.rx_buffer.recv_len = 0;
    board_com.rx_buffer.total_len = 0;
    board_com.rx_buffer.current_seq = 0;
    board_com.rx_buffer.expected_seq = 1;
    board_com.rx_buffer.status = PACKET_IDLE;
}

board_com_t *board_com_init(board_com_init_t* board_com_init)
{
    // 初始化板间通讯的掉线检测
    board_com.offlinemanage_index = offline_device_register(&board_com_init->offline_manage_init);

    // CAN 设备初始化配置
    Can_Device_Init_Config_s can_config = {
        .can_handle = board_com_init->Can_Device_Init_Config.can_handle,
        .tx_id = board_com_init->Can_Device_Init_Config.tx_id,
        .rx_id = board_com_init->Can_Device_Init_Config.rx_id,
        .tx_mode = CAN_MODE_BLOCKING,
        .rx_mode = CAN_MODE_IT,
        .can_callback = board_recv
    };
    
    Can_Device *can_dev = BSP_CAN_Device_Init(&can_config);
    if (can_dev == NULL) {
        log_e("Failed to initialize CAN device for board_com");
        return NULL;
    }
    board_com.candevice = can_dev;
    
    // 初始化接收缓冲区
    reset_rx_buffer();
    board_com.total_send_seq = 0;
    board_com.current_send_seq = 0;

    return &board_com;
}

void board_com_send(uint8_t *data, uint8_t size)
{
    if (size == 0 || data == NULL || size > MAX_PACKET_SIZE) {
        log_e("Invalid data size: %d", size);
        return;
    }
    
    // 计算总包数
    if (size <= 5) {
        board_com.total_send_seq = 1;
    } else {
        // 计算中间包数量：向上取整
        uint8_t remaining = size - 5;  // 减去第一包的5字节
        board_com.total_send_seq = 1 + (remaining + 6) / 7;  // 加6是为了向上取整
    }
    
    // 发送数据包
    for (uint8_t i = 0; i < board_com.total_send_seq; i++) {
        board_com.current_send_seq = i+1;
        
        if (i == 0) {
            // 第一包：帧头 + 总长度 + 序号 + 数据
            board_com.candevice->tx_buff[0] = FRAME_HEADER;
            board_com.candevice->tx_buff[1] = size;
            board_com.candevice->tx_buff[2] = board_com.current_send_seq;
            uint8_t data_len = (size > 5) ? 5 : size;  // 如果总长度小于5，只发送实际长度
            memcpy(&board_com.candevice->tx_buff[3], data, data_len);
        } else if (i == board_com.total_send_seq - 1) {
            // 最后一包：序号 + 数据
            board_com.candevice->tx_buff[0] = board_com.current_send_seq;
            uint8_t data_len = size - (i-1)*7 - 5;  // 计算剩余数据长度
            if (data_len > 0) {  // 只有当还有数据时才复制
                memcpy(&board_com.candevice->tx_buff[1], data + (i-1)*7 + 5, data_len);
            }
        } else {
            // 中间包：序号 + 数据
            board_com.candevice->tx_buff[0] = board_com.current_send_seq;
            memcpy(&board_com.candevice->tx_buff[1], data + (i-1)*7 + 5, 7);
        }
        
        CAN_SendMessage(board_com.candevice, 8);
    }
}

void board_recv(const CAN_HandleTypeDef* hcan, const uint32_t rx_id)
{
    UNUSED(hcan);
    UNUSED(rx_id);

    // 检查帧头
    if (board_com.candevice->rx_buff[0] == FRAME_HEADER) {
        // 新数据包开始
        reset_rx_buffer();
        board_com.rx_buffer.status = PACKET_RECEIVING;
        board_com.rx_buffer.total_len = board_com.candevice->rx_buff[1];
        board_com.rx_buffer.current_seq = 1;
        board_com.rx_buffer.expected_seq = 2;  // 期望收到的下一个包序号为2
        
        // 计算总包数
        if (board_com.rx_buffer.total_len <= 5) {
            board_com.rx_buffer.total_seq = 1;
        } else {
            // 计算中间包数量：向上取整
            uint8_t remaining = board_com.rx_buffer.total_len - 5;  // 减去第一包的5字节
            board_com.rx_buffer.total_seq = 1 + (remaining + 6) / 7;  // 向上取整
        }
        
        // 复制第一包数据
        uint8_t *current_buf = board_com.rx_buffer.raw_buf[board_com.rx_buffer.recv_buf_index];
        uint8_t data_len = (board_com.rx_buffer.total_len > 5) ? 5 : board_com.rx_buffer.total_len;  // 根据总长度决定第一包长度
        memcpy(current_buf + CAN_COMM_OFFSET_BYTES, &board_com.candevice->rx_buff[3], data_len);
        board_com.rx_buffer.recv_len = data_len;
        
        return;
    }
    
    // 检查接收状态
    if (board_com.rx_buffer.status != PACKET_RECEIVING) {
        return;
    }
    
    // 检查包序号
    if (board_com.candevice->rx_buff[0] != board_com.rx_buffer.expected_seq) {
        reset_rx_buffer();
        return;
    }
    
    board_com.rx_buffer.current_seq++;

    // 计算当前包的数据长度
    uint8_t *current_buf = board_com.rx_buffer.raw_buf[board_com.rx_buffer.recv_buf_index];
    uint8_t data_len;
    if (board_com.rx_buffer.current_seq == board_com.rx_buffer.total_seq) {
        // 最后一包：根据总长度计算剩余数据长度
        data_len = board_com.rx_buffer.total_len - (board_com.rx_buffer.current_seq-2)*7 - 5;
    } else {
        data_len = 7;
    }
    
    // 复制数据
    memcpy(current_buf + CAN_COMM_OFFSET_BYTES + board_com.rx_buffer.recv_len, 
           &board_com.candevice->rx_buff[1], data_len);
    board_com.rx_buffer.recv_len += data_len;
    
    // 检查是否接收完成
    if (board_com.rx_buffer.current_seq == board_com.rx_buffer.total_seq 
        && board_com.rx_buffer.status == PACKET_RECEIVING) {
        if (board_com.rx_buffer.recv_len == board_com.rx_buffer.total_len) {
            // 数据接收完成，复制到解包缓冲区
            memcpy(board_com.rx_buffer.unpacked_data, 
                   current_buf + CAN_COMM_OFFSET_BYTES, 
                   board_com.rx_buffer.total_len);
            board_com.rx_buffer.status = PACKET_COMPLETE;
            
            // 更新掉线检测
            offline_device_update(board_com.offlinemanage_index);
            
            // 切换缓冲区
            board_com.rx_buffer.recv_buf_index = !board_com.rx_buffer.recv_buf_index;
        } else {
            board_com.rx_buffer.status = PACKET_ERROR;
        }
        reset_rx_buffer();
    }
    board_com.rx_buffer.expected_seq++;
}


void *board_com_get_data(void)
{
    return board_com.rx_buffer.unpacked_data;
}

