# uart

这里是bsp层串口部分，由于串口基本独占，所以根据串口数量设定实例个数。

> 如何新添加串口实例
>
> #define UART_MAX_INSTANCE_NUM 3 //修改这个即可

## uart设备说明

```c
如何注册一个uart设备
直接回调函数版本
static uint8_t sbus_buf[2][30];
UART_Device_init_config uart3_cfg = {
        .huart = &huart3,
        .expected_len = 25,  
        .rx_buf = (uint8_t (*)[2])sbus_buf,
        .rx_buf_size = 30,
        .rx_mode = UART_MODE_DMA,
        .tx_mode = UART_MODE_BLOCKING,
        .rx_complete_cb = uart3_rx_callback, //回调函数，自己声明，严禁延时
        .cb_type = UART_CALLBACK_DIRECT,
        .event_flag = 0x01,
    };
    UART_Device *remote = UART_Init(&uart3_cfg); //注意缓冲区要自己定义，设备会优先调用外部接收缓冲区，

事件通知线程处理版本
static uint8_t referee_buf[2][1024];
UART_Device_init_config uart6_cfg = {
        .huart = &huart6,
        .expected_len = 0,       // 不定长
        .rx_buf_size = 1024,
        .rx_buf = (uint8_t (*)[2])referee_buf,
        .rx_mode = UART_MODE_DMA,
        .tx_mode = UART_MODE_DMA,
        .timeout = 1000,
        .rx_complete_cb = NULL,
        .cb_type = UART_CALLBACK_EVENT,//事件通知
        .event_flag = UART_RX_DONE_EVENT
    };
    UART_Device* uart6 = UART_Init(&uart6_cfg);

```
