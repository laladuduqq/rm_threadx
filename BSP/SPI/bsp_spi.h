#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#include "spi.h"
#include "tx_api.h"

#define SPI_BUS_NUM 2                  // 总线数量
#define MAX_DEVICES_PER_BUS 3         // 每条总线最大设备数

/* 传输模式枚举 */
typedef enum {
    SPI_MODE_BLOCKING,
    SPI_MODE_IT,
    SPI_MODE_DMA
} SPI_Mode;

/* SPI设备实例结构体 */
typedef struct {
    SPI_HandleTypeDef* hspi;         // SPI句柄
    GPIO_TypeDef* cs_port;          // 片选端口
    uint16_t cs_pin;                // 片选引脚
    SPI_Mode tx_mode;              // 发送模式
    SPI_Mode rx_mode;              // 接收模式
    uint32_t timeout;              // 超时时间
    void (*spi_callback)(void);    // 回调函数
} SPI_Device;

/* 初始化配置结构体 */
typedef struct {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
    SPI_Mode tx_mode;
    SPI_Mode rx_mode;
    uint32_t timeout;
    void (*spi_callback)(void);
} SPI_Device_Init_Config;

/* SPI总线管理结构 */
typedef struct {
    SPI_HandleTypeDef* hspi;
    SPI_Device devices[MAX_DEVICES_PER_BUS];
    TX_SEMAPHORE tx_sem;           // 使用信号量而不是互斥量
    uint8_t device_count;
    SPI_Device* active_dev;        // 当前活动设备
} SPI_Bus_Manager;

/* 公有函数声明 */
SPI_Device* BSP_SPI_Device_Init(SPI_Device_Init_Config* config);
void BSP_SPI_Device_DeInit(SPI_Device* dev);
HAL_StatusTypeDef BSP_SPI_TransReceive(SPI_Device* dev, const uint8_t* tx_data, uint8_t* rx_data, uint16_t size);
HAL_StatusTypeDef BSP_SPI_Transmit(SPI_Device* dev, const uint8_t* tx_data, uint16_t size);
HAL_StatusTypeDef BSP_SPI_Receive(SPI_Device* dev, uint8_t* rx_data, uint16_t size);
HAL_StatusTypeDef BSP_SPI_TransAndTrans(SPI_Device* dev, const uint8_t* tx_data1, uint16_t size1, 
                                       const uint8_t* tx_data2, uint16_t size2);

#endif
