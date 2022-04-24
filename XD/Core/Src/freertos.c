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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32l4xx_hal.h"
//#include "mxconstants.h"
#include "usbd_core.h"
#include "usb_device.h"
#include "fatfs.h"

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
/* USER CODE BEGIN Variables */

FIL                       File;
FATFS                     FatFs;

/* USER CODE END Variables */
osThreadId apptaskHandle;
osThreadId usbTaskHandle;
osMutexId qspiMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void Error_Handler(void);
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END FunctionPrototypes */

void appTaskBody(void const * argument);
void usbTaskBody(void const * argument);

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
  /* Create the mutex(es) */
  /* definition and creation of qspiMutex */
  osMutexDef(qspiMutex);
  qspiMutexHandle = osMutexCreate(osMutex(qspiMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of apptask */
  osThreadDef(apptask, appTaskBody, osPriorityNormal, 0, 4096);
  apptaskHandle = osThreadCreate(osThread(apptask), NULL);

  /* definition and creation of usbTask */
  osThreadDef(usbTask, usbTaskBody, osPriorityHigh, 0, 256);
  usbTaskHandle = osThreadCreate(osThread(usbTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_appTaskBody */
/**
  * @brief  Function implementing the apptask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_appTaskBody */
void appTaskBody(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN appTaskBody */

  /* Wait for the qspiMutex */
  osMutexWait(qspiMutexHandle, osWaitForever);

  /* Register the file system object to the FatFs module */
  if(f_mount(&FatFs, (TCHAR const*)USERPath, 1) != FR_OK)
  {
    /* FatFs Initialization Error */
//    if (f_mkfs((TCHAR const*)USER_Path, 0, 128) != FR_OK)
//    {
//      Error_Handler();
//    }
//    else {
//      /* Second trial to register the file system object */
//      if(f_mount(&FatFs, (TCHAR const*)USER_Path, 1) != FR_OK)
//      {
//        Error_Handler();
//      }
//    }
  }

  /* FatFS file write test */
  if(f_open(&File, "FATFSOK", FA_CREATE_NEW | FA_WRITE) == FR_OK)
  {
    f_printf(&File, "FatFS is working properly.\n");
    f_close(&File);
  }

  /* Release the qspiMutex */
  osMutexRelease(qspiMutexHandle);

  /* Infinite loop */
  for(;;)
  {
    //osDelay(100);

    /* GREEN LED ON */
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

    /* Wait for the qspiMutex */
    osMutexWait(qspiMutexHandle, osWaitForever);

    /* You can access qspi again here */

    osDelay(1);

    /* Release the qspiMutex */
    osMutexRelease(qspiMutexHandle);

    /* GREEN LED OFF */
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
  }
  /* USER CODE END appTaskBody */
}

/* USER CODE BEGIN Header_usbTaskBody */
/**
* @brief Function implementing the usbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usbTaskBody */
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
      /* Wait for the qspiMutex */
      osMutexWait(qspiMutexHandle, osWaitForever);

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

      /* Release the qspiMutex */
      osMutexRelease(qspiMutexHandle);

      /* RED LED OFF */
      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    }
    /* Every 1s we will check if USB_VBUS is available */
    osDelay(1000);
  }
  /* USER CODE END usbTaskBody */
}

/* Private application code --------------------------------------------------*/
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
