#include "bsp_spi.h"
#include <string.h>
#include "tx_api.h"

#define LOG_TAG "bsp_spi"
#include "elog.h"

/* SPI总线硬件配置数组 */
static const struct {
    SPI_HandleTypeDef *hspi;
    const char *name;
} spi_configs[SPI_BUS_NUM] = {
    {&hspi1, "SPI1"},
    {&hspi2, "SPI2"}
};

/* 总线管理器实例 */
static SPI_Bus_Manager spi_bus[SPI_BUS_NUM] = {
    {
        .hspi = NULL,
        .devices = {{0}},
        .tx_sem = {0},
        .device_count = 0,
        .active_dev = NULL
    },
    {
        .hspi = NULL,
        .devices = {{0}},
        .tx_sem = {0},
        .device_count = 0,
        .active_dev = NULL
    }
};

/* 私有函数声明 */
static void SPI_BusInit(void);

static void SPI_BusInit(void) {
    for(uint8_t i = 0; i < SPI_BUS_NUM; i++) {
        spi_bus[i].hspi = spi_configs[i].hspi;
        
        // 创建信号量
        if(tx_semaphore_create(&spi_bus[i].tx_sem, 
                              (CHAR *)spi_configs[i].name, 
                              1) != TX_SUCCESS) {
            log_e("Failed to create %s semaphore", spi_configs[i].name);
            continue;
        }
        
        log_i("%s bus initialized", spi_configs[i].name);
    }
}

SPI_Device* BSP_SPI_Device_Init(SPI_Device_Init_Config* config) {
    /* 首次注册时初始化总线 */
    if (spi_bus[0].hspi == NULL) {
        log_i("First time initializing SPI bus");
        SPI_BusInit();
    }

    /* 查找对应总线 */
    SPI_Bus_Manager *bus = NULL;
    for(uint8_t i = 0; i < SPI_BUS_NUM; i++) {
        if(spi_bus[i].hspi == config->hspi) {
            bus = &spi_bus[i];
            break;
        }
    }

    if(!bus) {
        log_e("SPI bus not found for handle: 0x%08X", (unsigned int)config->hspi);
        return NULL;
    }

    /* 检查设备数量 */
    if(bus->device_count >= MAX_DEVICES_PER_BUS) {
        log_e("Maximum devices reached on bus");
        return NULL;
    }

    /* 查找空闲设备槽位 */
    SPI_Device *dev = NULL;
    for(uint8_t i = 0; i < MAX_DEVICES_PER_BUS; i++) {
        if(bus->devices[i].hspi == NULL) {
            dev = &bus->devices[i];
            break;
        }
    }

    if(!dev) {
        log_e("No free device slot");
        return NULL;
    }

    /* 初始化设备 */
    dev->hspi = config->hspi;
    dev->cs_port = config->cs_port;
    dev->cs_pin = config->cs_pin;
    dev->tx_mode = config->tx_mode;
    dev->rx_mode = config->rx_mode;
    dev->timeout = config->timeout;
    dev->spi_callback = config->spi_callback;

    /* 配置CS引脚 */
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = dev->cs_pin,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };
    HAL_GPIO_Init(dev->cs_port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    bus->device_count++;
    log_i("SPI device registered on bus %s", 
          spi_configs[config->hspi == &hspi1 ? 0 : 1].name);

    return dev;
}

HAL_StatusTypeDef BSP_SPI_TransReceive(SPI_Device* dev, const uint8_t* tx_data, 
                                      uint8_t* rx_data, uint16_t size)
{
    SPI_Bus_Manager *bus = NULL;
    for(uint8_t i = 0; i < SPI_BUS_NUM; i++) {
        if(spi_bus[i].hspi == dev->hspi) {
            bus = &spi_bus[i];
            break;
        }
    }
    if(!bus) return HAL_ERROR;

    // 获取信号量
    if(tx_semaphore_get(&bus->tx_sem, TX_NO_WAIT) != TX_SUCCESS) {
        return HAL_BUSY;
    }

    HAL_StatusTypeDef status;
    bus->active_dev = dev;
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);

    if (dev->tx_mode == SPI_MODE_DMA && dev->rx_mode == SPI_MODE_DMA) {
        status = HAL_SPI_TransmitReceive_DMA(dev->hspi, tx_data, rx_data, size);
    }
    else if (dev->tx_mode == SPI_MODE_IT && dev->rx_mode == SPI_MODE_IT) {
        status = HAL_SPI_TransmitReceive_IT(dev->hspi, tx_data, rx_data, size);
    }
    else {
        status = HAL_SPI_TransmitReceive(dev->hspi, tx_data, rx_data, size, dev->timeout);
        tx_semaphore_put(&bus->tx_sem);
        bus->active_dev = NULL;
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    }

    return status;
}


void BSP_SPI_Device_DeInit(SPI_Device* dev) {
    for(uint8_t i = 0; i < SPI_BUS_NUM; i++) {
        for(uint8_t j = 0; j < MAX_DEVICES_PER_BUS; j++) {
            if(&spi_bus[i].devices[j] == dev) {
                memset(dev, 0, sizeof(SPI_Device));
                spi_bus[i].device_count--;
                return;
            }
        }
    }
}

HAL_StatusTypeDef BSP_SPI_Transmit(SPI_Device* dev, const uint8_t* tx_data, uint16_t size)
{
    SPI_Bus_Manager *bus = NULL;
    for(uint8_t i = 0; i < SPI_BUS_NUM; i++) {
        if(spi_bus[i].hspi == dev->hspi) {
            bus = &spi_bus[i];
            break;
        }
    }
    if(!bus) return HAL_ERROR;

    // 获取信号量
    if(tx_semaphore_get(&bus->tx_sem, TX_NO_WAIT) != TX_SUCCESS) {
        return HAL_BUSY;
    }

    HAL_StatusTypeDef status;
    bus->active_dev = dev;
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);

    if (dev->tx_mode == SPI_MODE_DMA) {
        status = HAL_SPI_Transmit_DMA(dev->hspi, tx_data, size);
    }
    else if (dev->tx_mode == SPI_MODE_IT) {
        status = HAL_SPI_Transmit_IT(dev->hspi, tx_data, size);
    }
    else {
        status = HAL_SPI_Transmit(dev->hspi, tx_data, size, dev->timeout);
        tx_semaphore_put(&bus->tx_sem);
        bus->active_dev = NULL;
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    }

    return status;
}

HAL_StatusTypeDef BSP_SPI_Receive(SPI_Device* dev, uint8_t* rx_data, uint16_t size)
{
    SPI_Bus_Manager *bus = NULL;
    for(uint8_t i = 0; i < SPI_BUS_NUM; i++) {
        if(spi_bus[i].hspi == dev->hspi) {
            bus = &spi_bus[i];
            break;
        }
    }
    if(!bus) return HAL_ERROR;

    // 获取信号量
    if(tx_semaphore_get(&bus->tx_sem, TX_NO_WAIT) != TX_SUCCESS) {
        return HAL_BUSY;
    }

    HAL_StatusTypeDef status;
    bus->active_dev = dev;
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);

    if (dev->rx_mode == SPI_MODE_DMA) {
        status = HAL_SPI_Receive_DMA(dev->hspi, rx_data, size);
    }
    else if (dev->rx_mode == SPI_MODE_IT) {
        status = HAL_SPI_Receive_IT(dev->hspi, rx_data, size);
    }
    else {
        status = HAL_SPI_Receive(dev->hspi, rx_data, size, dev->timeout);
        tx_semaphore_put(&bus->tx_sem);
        bus->active_dev = NULL;
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    }

    return status;
}

HAL_StatusTypeDef BSP_SPI_TransAndTrans(SPI_Device* dev, const uint8_t* tx_data1, 
                                       uint16_t size1, const uint8_t* tx_data2, 
                                       uint16_t size2)
{
    HAL_StatusTypeDef status;
    
    // 第一次传输
    status = BSP_SPI_Transmit(dev, tx_data1, size1);
    if(status != HAL_OK) return status;
    
    // 等待第一次传输完成
    while(HAL_SPI_GetState(dev->hspi) != HAL_SPI_STATE_READY);
    
    // 第二次传输
    return BSP_SPI_Transmit(dev, tx_data2, size2);
}

// SPI 接收完成回调
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    UINT old_posture;
    
    /* 进入临界区 */
    old_posture = tx_interrupt_control(TX_INT_DISABLE);
    
    for(uint8_t i = 0; i < SPI_BUS_NUM; i++) {
        if(spi_bus[i].hspi == hspi && spi_bus[i].active_dev != NULL) {
            HAL_GPIO_WritePin(spi_bus[i].active_dev->cs_port, 
                            spi_bus[i].active_dev->cs_pin, 
                            GPIO_PIN_SET);
            
            if(spi_bus[i].active_dev->spi_callback) {
                spi_bus[i].active_dev->spi_callback();
            }
            
            tx_semaphore_put(&spi_bus[i].tx_sem);
            spi_bus[i].active_dev = NULL;
            break;
        }
    }
    
    /* 退出临界区 */
    tx_interrupt_control(old_posture);
}

// SPI 发送接收完成回调
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    UINT old_posture;
    
    /* 进入临界区 */
    old_posture = tx_interrupt_control(TX_INT_DISABLE);
    
    for(uint8_t i = 0; i < SPI_BUS_NUM; i++) {
        if(spi_bus[i].hspi == hspi && spi_bus[i].active_dev != NULL) {
            HAL_GPIO_WritePin(spi_bus[i].active_dev->cs_port, 
                            spi_bus[i].active_dev->cs_pin, 
                            GPIO_PIN_SET);
            
            if(spi_bus[i].active_dev->spi_callback) {
                spi_bus[i].active_dev->spi_callback();
            }
            
            tx_semaphore_put(&spi_bus[i].tx_sem);
            spi_bus[i].active_dev = NULL;
            break;
        }
    }
    
    /* 退出临界区 */
    tx_interrupt_control(old_posture);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    UINT old_posture;
    
    /* 进入临界区 */
    old_posture = tx_interrupt_control(TX_INT_DISABLE);
    
    for(uint8_t i = 0; i < SPI_BUS_NUM; i++) {
        if(spi_bus[i].hspi == hspi && spi_bus[i].active_dev != NULL) {
            HAL_GPIO_WritePin(spi_bus[i].active_dev->cs_port, 
                            spi_bus[i].active_dev->cs_pin, 
                            GPIO_PIN_SET);
            
            if(spi_bus[i].active_dev->spi_callback) {
                spi_bus[i].active_dev->spi_callback();
            }
            
            tx_semaphore_put(&spi_bus[i].tx_sem);
            spi_bus[i].active_dev = NULL;
            break;
        }
    }
    
    /* 退出临界区 */
    tx_interrupt_control(old_posture);
}
