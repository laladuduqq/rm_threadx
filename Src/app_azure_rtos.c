
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_azure_rtos.c
  * @author  MCD Application Team
  * @brief   azure_rtos application implementation file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "app_azure_rtos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "robot_init.h"
#include "elog.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN TX_Pool_Buffer */
/* USER CODE END TX_Pool_Buffer */
static UCHAR tx_byte_pool_buffer[TX_APP_MEM_POOL_SIZE];
static TX_BYTE_POOL tx_app_byte_pool;

/* USER CODE BEGIN UX_Device_Pool_Buffer */
/* USER CODE END UX_Device_Pool_Buffer */
static UCHAR  ux_device_byte_pool_buffer[UX_DEVICE_APP_MEM_POOL_SIZE];
static TX_BYTE_POOL ux_device_app_byte_pool;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void Print_BytePool_Info(TX_BYTE_POOL *pool, const char *name);
/* USER CODE END PFP */

/**
  * @brief  Define the initial system.
  * @param  first_unused_memory : Pointer to the first unused memory
  * @retval None
  */
VOID tx_application_define(VOID *first_unused_memory)
{
  /* USER CODE BEGIN  tx_application_define */
  (void)first_unused_memory;
  /* USER CODE END  tx_application_define */

  VOID *memory_ptr;

  if (tx_byte_pool_create(&tx_app_byte_pool, "Tx App memory pool", tx_byte_pool_buffer, TX_APP_MEM_POOL_SIZE) != TX_SUCCESS)
  {
    /* USER CODE BEGIN TX_Byte_Pool_Error */

    /* USER CODE END TX_Byte_Pool_Error */
  }
  else
  {
    /* USER CODE BEGIN TX_Byte_Pool_Success */

    /* USER CODE END TX_Byte_Pool_Success */

    memory_ptr = (VOID *)&tx_app_byte_pool;

    if (App_ThreadX_Init(memory_ptr) != TX_SUCCESS)
    {
      /* USER CODE BEGIN  App_ThreadX_Init_Error */

      /* USER CODE END  App_ThreadX_Init_Error */
    }

    /* USER CODE BEGIN  App_ThreadX_Init_Success */

    /* USER CODE END  App_ThreadX_Init_Success */

  }

  if (tx_byte_pool_create(&ux_device_app_byte_pool, "Ux App memory pool", ux_device_byte_pool_buffer, UX_DEVICE_APP_MEM_POOL_SIZE) != TX_SUCCESS)
  {
    /* USER CODE BEGIN UX_Device_Byte_Pool_Error */

    /* USER CODE END UX_Device_Byte_Pool_Error */
  }
  else
  {
    /* USER CODE BEGIN UX_Device_Byte_Pool_Success */

    /* USER CODE END UX_Device_Byte_Pool_Success */

    memory_ptr = (VOID *)&ux_device_app_byte_pool;

    if (MX_USBX_Device_Init(memory_ptr) != UX_SUCCESS)
    {
      /* USER CODE BEGIN MX_USBX_Device_Init_Error */

      /* USER CODE END MX_USBX_Device_Init_Error */
    }

    /* USER CODE BEGIN MX_USBX_Device_Init_Success */
    base_init();
    robot_init(&tx_app_byte_pool);
    Print_BytePool_Info(&tx_app_byte_pool, "TX_APP_MEM_POOL");
    log_i("Azure RTOS application initialized successfully.");
    /* USER CODE END MX_USBX_Device_Init_Success */
  }
}

/* USER CODE BEGIN  0 */
void Print_BytePool_Info(TX_BYTE_POOL *pool, const char *name)
{
    ULONG total = pool->tx_byte_pool_size;
    ULONG available = pool->tx_byte_pool_available;
    ULONG used = total - available;
    UINT fragments = pool->tx_byte_pool_fragments;

    log_i("%s: total=%lu bytes, available=%lu bytes, used=%lu bytes, fragments=%u\n",
        name, total, available, used, fragments);
}
/* USER CODE END  0 */
