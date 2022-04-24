/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

#include "stm32l4xx_hal.h"
//#include "mxconstants.h"
#include "usbd_core.h"
#include "usb_device.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId appTaskHandle;
osThreadId usbTaskHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void appTaskBody(void const * argument);
void usbTaskBody(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

void Error_Handler(void);

/* USER CODE END FunctionPrototypes */

void appTaskBody(void const * argument);
void usbTaskBody(void const * argument);
extern USBD_HandleTypeDef hUsbDeviceFS;
extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of appTask */
  osThreadDef(appTask, appTaskBody, osPriorityNormal, 0, 4096);
  appTaskHandle = osThreadCreate(osThread(appTask), NULL);

  /* definition and creation of usbTask */
  osThreadDef(usbTask, usbTaskBody, osPriorityHigh, 0, 256);
  usbTaskHandle = osThreadCreate(osThread(usbTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* appTaskBody function */
void appTaskBody(void const * argument)
{
  /* init code for USB_DEVICE */
  //MX_USB_DEVICE_Init();

  /* USER CODE BEGIN appTaskBody */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);

    /* GREEN LED TOGGLE */
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
  }
  /* USER CODE END appTaskBody */
}

/* usbTaskBody function */
void usbTaskBody(void const * argument)
{
  /* USER CODE BEGIN usbTaskBody */
  uint32_t USB_VBUS_counter = 0;

  /* Infinite loop */
  for(;;)
  {
    USB_VBUS_counter = 0;
    /* USB_VBUS availability check */
    while (USB_VBUS_counter < 5)
    {
      osDelay(10);
      if (HAL_GPIO_ReadPin(USB_VBUS_GPIO_Port, USB_VBUS_Pin) != GPIO_PIN_RESET)
      {
        USB_VBUS_counter++;
      }
      else {
        break;
      }
    }
    if(USB_VBUS_counter >= 5)
    {
      /* Initialize USB peripheral */
      MX_USB_DEVICE_Init();

      /* RED LED ON */
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);

      /* USB_VBUS availability check */
      while (USB_VBUS_counter)
      {
        /* Wait 100ms, then check the USB_VBUS availability */
        osDelay(100);

        if (HAL_GPIO_ReadPin(USB_VBUS_GPIO_Port, USB_VBUS_Pin) == GPIO_PIN_RESET)
        {
          USB_VBUS_counter--;
        }
        else {
          USB_VBUS_counter = 5;
        }
      }

      /* Deinitialize USB peripheral */
      USBD_DeInit(&hUsbDeviceFS);

      /* RED LED OFF */
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    }
    /* Every 1s we will check if USB_VBUS is available */
    osDelay(1000);
  }
  /* USER CODE END usbTaskBody */
}

/* USER CODE BEGIN Application */

/**
  * @brief Application Error Handler
  * @param  None
  * @retval None
  */
//void Error_Handler(void)
//{
//  while(1)
//  {
    /* Error management behavior to be added here */
//  }
//}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
