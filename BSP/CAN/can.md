# can

## bsp_can介绍

整体通过总线管理，设备注册的方式来实现不同can设备的管理和调度。

### can_bus介绍

```c
/* CAN总线管理结构 */
typedef struct {
    CAN_HandleTypeDef *hcan;
    Can_Device devices[MAX_DEVICES_PER_BUS];
    TX_MUTEX tx_mutex;
    uint8_t device_count;
} CANBusManager;
这是对应的canbusmanager,实现对每一路can总线的管理

#define CAN_BUS_NUM 2            // 总线数量
#define MAX_DEVICES_PER_BUS  8  // 每总线最大设备数
上述宏定义用来确定canbusmanager的总线个数，以及每个总线上可挂载的最大设备数，按照需要修改
如果需要添加新的总线（以喵板三路can为例），
首先宏定义修改为3,然后
/* CAN总线硬件配置数组 */
static const struct {
    CAN_HandleTypeDef *hcan;
    const char *name;
} can_configs[CAN_BUS_NUM] = {
    {&hcan1, "CAN1"},
    {&hcan2, "CAN2"},
    {&hcan3, "CAN3"} //这里添加新的can对应的句柄和名字
};

/* 总线管理器实例 */
static CANBusManager can_bus[CAN_BUS_NUM] = {
    {.hcan = NULL,.devices = {0},.tx_mutex = {0},.device_count = 0},
    {.hcan = NULL,.devices = {0},.tx_mutex = {0},.device_count = 0}
    {.hcan = NULL,.devices = {0},.tx_mutex = {0},.device_count = 0} //这里copy一个新的即可
};
这样添加新的总线就成功了
```

### can设备

```c
/* CAN设备实例结构体 */
typedef struct
{
    CAN_HandleTypeDef *can_handle;  // CAN句柄
    CAN_TxHeaderTypeDef txconf;     // 发送配置
    uint32_t tx_id;                 // 发送ID
    uint32_t tx_mailbox;            // 发送邮箱号
    uint8_t tx_buff[8];             // 发送缓冲区

    uint32_t rx_id;                 // 接收ID
    uint8_t rx_buff[8];             // 接收缓冲区
    uint8_t rx_len;                 // 接收长度

    CAN_Mode tx_mode;
    CAN_Mode rx_mode;

    void (*can_callback)(const CAN_HandleTypeDef* hcan, const uint32_t rx_id); // 接收回调
} Can_Device;
这是can设备对应的结构体

/* 初始化配置结构体 */
typedef struct
{
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    CAN_Mode tx_mode;
    CAN_Mode rx_mode;
    void (*can_callback)(const CAN_HandleTypeDef* hcan, const uint32_t rx_id);
} Can_Device_Init_Config_s;
这是对应初始化can设备的结构体

一个can设备的使用示例
// CAN 设备初始化配置
Can_Device_Init_Config_s can_config = {
        .can_handle = &hcan2,
        .tx_id = 0x01,
        .rx_id = 0x11,
        .tx_mode = CAN_MODE_BLOCKING,
        .rx_mode = CAN_MODE_IT,
        .can_callback = 自己写的回调函数，严禁有任何延时处理
 };
// 注册 CAN 设备并获取引用
Can_Device *can_dev = BSP_CAN_Device_Init(&can_config);
```
