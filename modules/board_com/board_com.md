# 板间通讯

板间通讯模块基于CAN总线实现，支持多包数据传输，采用帧头标识和序号管理机制确保数据可靠传输。

## 协议格式

### 2.1 数据包格式

- 第一包：[帧头(1字节)][总长度(1字节)][序号(1字节)][数据(5字节)]

- 中间包：[序号(1字节)][数据(7字节)]

- 最后一包：[序号(1字节)][数据(1-7字节)]

### 2.2 字段说明

- 帧头：固定值 0xAA

- 总长度：整个数据包的总长度（1-64字节）

- 序号：从1开始的包序号

- 数据：实际传输的数据

## 工作原理

### 3.1 发送流程

1. 计算总包数：
- 数据长度 ≤ 5字节：1包

- 数据长度 > 5字节：1 + (剩余长度 + 6) / 7 包
1. 分包发送：
- 第一包：发送帧头、总长度、序号和最多5字节数据

- 中间包：发送序号和7字节数据

- 最后一包：发送序号和剩余数据

### 3.2 接收流程

1. 帧头检测：
- 检测到帧头(0xAA)时开始新数据包接收

- 获取总长度和初始化接收状态
1. 数据接收：
- 第一包：接收最多5字节数据

- 中间包：接收7字节数据

- 最后一包：接收剩余数据
1. 完整性检查：
- 检查包序号连续性

- 验证接收数据总长度

## 使用示例

```c
// 初始化配置
board_com_init_t board_com_config = {
    .offline_manage_init = {
                      .name = "board_com",
                      .timeout_ms = 100,
                      .level = OFFLINE_LEVEL_HIGH,
                      .beep_times = 8,
                      .enable = OFFLINE_ENABLE,
                    },
    .Can_Device_Init_Config = {
                        .can_handle = &hcan2,
                        .tx_id = GIMBAL_ID,
                        .rx_id = CHASSIS_ID,
                    }
};
// 初始化板间通讯
board_com_t *board_com = board_com_init(&init_config);



//发送数据
uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
board_com_send(data, sizeof(data));


// 接收数据示例
void process_received_data(void)
{
    uint8_t *data = board_com_get_data();
}
```


