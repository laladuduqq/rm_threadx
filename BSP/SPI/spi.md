# spi

## bsp_spi 介绍

整体通过总线管理，设备注册方式实现不同spi设备管理。


## spi_bus

```c
/* SPI总线管理结构 */
typedef struct {
    SPI_HandleTypeDef* hspi;
    SPI_Device devices[MAX_DEVICES_PER_BUS];
    TX_SEMAPHORE tx_sem;           // 使用信号量而不是互斥量
    uint8_t device_count;
    SPI_Device* active_dev;        // 当前活动设备
} SPI_Bus_Manager;
这里保存注册的设备，同时保证唯一设备占据总线。

新spi总线添加方式与can同样
```

## spi设备

```c
注册使用spi设备示例
static SPI_Device *bmi_gyro_device;
static SPI_Device_Init_Config gyro_cfg = {
        .hspi       = &hspi1
        .cs_port    = GPIOB,
        .cs_pin     = GPIO_PIN_0,
        .tx_mode    = BSP_SPI_MODE_BLOCKING,
        .rx_mode    = BSP_SPI_MODE_BLOCKING,
        .callback   = NULL,
        .timeout    = 1000
    };
    bmi_gyro_device = SPI_DeviceRegister(&gyro_cfg);
```
