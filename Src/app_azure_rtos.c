
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
#include "SEGGER_RTT.h"
#include "elog.h"
#include "iwdg.h"
#include "stm32f4xx_hal_iwdg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
unsigned long my_thread_counter = 0;
TX_THREAD my_thread;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void my_thread_entry(ULONG thread_input);
/* USER CODE END PFP */

/**
  * @brief  Define the initial system.
  * @param  first_unused_memory : Pointer to the first unused memory
  * @retval None
  */
VOID tx_application_define(VOID *first_unused_memory)
{
  /* USER CODE BEGIN  tx_application_define */

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
    CHAR *pointer;
    
    // 分配线程栈空间
    if (tx_byte_allocate(&tx_app_byte_pool, (VOID **) &pointer,
                        1024, TX_NO_WAIT) != TX_SUCCESS)
    {
        return;
    }
    
    // 创建线程
    if (tx_thread_create(&my_thread,
                        "My Thread",
                        my_thread_entry,
                        0,
                        pointer,
                        1024,
                        3,                 // 优先级 (0-31, 0 最高)  
                        3,                 // 抢占阈值
                        TX_NO_TIME_SLICE,   // 时间片
                        TX_AUTO_START)      // 自动启动
        != TX_SUCCESS)
    {
        return;
    }
    MX_IWDG_Init();
    SEGGER_RTT_Init();
    if (elog_user_init() == ELOG_NO_ERR) 
    { elog_start();}
    /* USER CODE END MX_USBX_Device_Init_Success */
  }
}

/* USER CODE BEGIN  0 */
void my_thread_entry(ULONG thread_input)
{
  (void)thread_input;
    /* Enter into a forever loop. */
    while(1)
    {
        /* Increment thread counter. */
        my_thread_counter++;
        log_e("helloworld!!!");
        log_i("hello world!!");
        log_d("hello world!!");
        HAL_IWDG_Refresh(&hiwdg);
        /* Sleep for 1 tick. */
        tx_thread_sleep(1);
    }
}
/* USER CODE END  0 */
