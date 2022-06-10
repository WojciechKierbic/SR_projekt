/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dfsdm.h"
#include "dma.h"
#include "i2c.h"

#include "quadspi.h"
#include "sai.h"
#include "spi.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "audio.h"
#include "cs43l22.h"
#include "stm32l476g_discovery_qspi.h"
#include "stm32l476g_discovery.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))
#define BUFFER_SIZE         ((uint32_t)0x800) // 2048 bajtów
#define WRITE_READ_ADDR     ((uint32_t)0x0000)
#define QSPI_BASE_ADDR      ((uint32_t)0x90000000)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
AUDIO_DrvTypeDef            *audio_drv;
uint32_t                     DmaRecHalfBuffCplt  = 0;
uint32_t                     DmaRecBuffCplt      = 0;
extern DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
extern DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
extern SAI_HandleTypeDef hsai_BlockA1;
extern DMA_HandleTypeDef hdma_sai1_a;
int32_t                      RecBuff[2048];
int16_t                      PlayBuff[4096];
int32_t 					filter_arr[4096];
uint32_t                     PlaybackStarted         = 0;
volatile int 				tryb = -1;//
int flag = 0;
static QSPI_Info pQSPI_Info;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t i;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  MX_DMA_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();

  MX_QUADSPI_Init();
  MX_SAI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USB_HOST_Init();
  MX_DFSDM1_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_OK != HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, RecBuff, 2048))
    {
         Error_Handler();
     }


  __HAL_SAI_ENABLE(&hsai_BlockA1);

      if(CS43L22_ID != cs43l22_drv.ReadID(AUDIO_I2C_ADDRESS))
        {
          Error_Handler();
        }
      	audio_drv = &cs43l22_drv;
        audio_drv->Reset(AUDIO_I2C_ADDRESS);
        if(0 != audio_drv->Init(AUDIO_I2C_ADDRESS, OUTPUT_DEVICE_HEADPHONE, 90, AUDIO_FREQUENCY_44K))
        {
          Error_Handler();
        }



        if (QSPI_OK == BSP_QSPI_Init())
        {
      	  pQSPI_Info.FlashSize          = (uint32_t)0x00;
      	  pQSPI_Info.EraseSectorSize    = (uint32_t)0x00;
      	  pQSPI_Info.EraseSectorsNumber = (uint32_t)0x00;
      	  pQSPI_Info.ProgPageSize       = (uint32_t)0x00;
      	  pQSPI_Info.ProgPagesNumber    = (uint32_t)0x00;
      	  /* Read the QSPI memory info */
      	  BSP_QSPI_GetInfo(&pQSPI_Info);

      	  /* Test the correctness */
      	  if((pQSPI_Info.FlashSize != 0x1000000) || (pQSPI_Info.EraseSectorSize != 0x1000)  ||
      	    (pQSPI_Info.ProgPageSize != 0x100)  || (pQSPI_Info.EraseSectorsNumber != 4096) ||
      	    (pQSPI_Info.ProgPagesNumber != 65536))
      	  {

      	  }
      	  else
      	  {
      		  /*##-3- Erase QSPI memory ################################################*/
      		  if(BSP_QSPI_Erase_Block(WRITE_READ_ADDR) != QSPI_OK)
      		  {

      	      }
      	  }
        }






        int index = 0; //ilosc wiadomosci

      for (uint32_t l = 0; l < 256; l++)
      {
        if(BSP_QSPI_Erase_Sector(l) != QSPI_OK)
        {
      	  Error_Handler();
        }
      }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {





      		  if(BSP_JOY_GetState() == JOY_SEL)
      			 {
      			  for(int j = 0; j<2000; j++)
      			  {
      				BSP_QSPI_Read(filter_arr, WRITE_READ_ADDR+(BUFFER_SIZE*j), 4*BUFFER_SIZE);

      				if(DmaRecHalfBuffCplt == 1)
      				    {
      				      /* Store values on Play buff */
      				      for(i = 0; i < BUFFER_SIZE/2; i++)
      				      {
      				        PlayBuff[2*i]     = SaturaLH((filter_arr[i] >> 8), -32768, 32767);
      				        PlayBuff[(2*i)+1] = PlayBuff[2*i];
      				      }
      				      if(PlaybackStarted == 0)
      				      {
      				    	 if(HAL_OK != HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *) &PlayBuff[0], 2*BUFFER_SIZE))
      				    	      				        {
      				    	      				          Error_Handler();
      				    	      				        }
      				    	if(0 != audio_drv->Play(AUDIO_I2C_ADDRESS, (uint16_t *) &PlayBuff[0], 2*BUFFER_SIZE))
      				    	{
      				    		Error_Handler();

      				    	}


      				    PlaybackStarted = 1;
      				      }
      				      DmaRecHalfBuffCplt  = 0;
      				    }
      				    if(DmaRecBuffCplt == 1)
      				    {
      				      /* Store values on Play buff */
      				      for(i = BUFFER_SIZE/2; i < BUFFER_SIZE; i++)
      				      {
      				        PlayBuff[2*i]     = SaturaLH((filter_arr[i] >> 8), -32768, 32767);
      				        PlayBuff[(2*i)+1] = PlayBuff[2*i];
      				      }
      				      DmaRecBuffCplt  = 0;
      				    }

      			 }
      			if(0 != audio_drv->Stop(AUDIO_I2C_ADDRESS, CODEC_PDWN_HW))
      					{
      				Error_Handler();
      					}
      			if(HAL_SAI_DMAStop(&hsai_BlockA1) != HAL_OK)
      			{
      				Error_Handler();
      			}
      			PlaybackStarted = 0;
      			  tryb = 1;
      			 }
      	  }
      	  while(tryb == 3)
      	  {
      		  if(BSP_JOY_GetState() == JOY_LEFT)
      		  {

      			  tryb = 1;
      		  }



      		for(int k = 0; k < 1000; k++)
      		{
      		if(DmaRecHalfBuffCplt == 1)
      		{
      		      				      /* Store values on Play buff */
      			for(i = 0; i < BUFFER_SIZE/2; i++)
      			{
      				filter_arr[2*i]     = SaturaLH((RecBuff[i] >> 8), -32768, 32767);
      				filter_arr[(2*i)+1] = filter_arr[2*i];
      			}
      			if (BSP_QSPI_Write(filter_arr, WRITE_READ_ADDR+(BUFFER_SIZE*k), BUFFER_SIZE/2) != QSPI_OK) Error_Handler();
      			DmaRecHalfBuffCplt  = 0;
      		}
      		if(DmaRecBuffCplt == 1)
      		{
      			/* Store values on Play buff */
      			for(i = BUFFER_SIZE/2; i < BUFFER_SIZE; i++)
      			{
      				filter_arr[2*i]     = SaturaLH((RecBuff[i] >> 8), -32768, 32767);
      				filter_arr[(2*i)+1] = filter_arr[2*i];
      			}
      			if (BSP_QSPI_Write(filter_arr, WRITE_READ_ADDR+(BUFFER_SIZE*k)+z, BUFFER_SIZE/2) != QSPI_OK) Error_Handler();
      			DmaRecBuffCplt  = 0;
      		}
      		}




      		tryb = 1;
      		index++;
      	  }

        /* USER CODE END 3 */
      }
  /* USER CODE END 3 */
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_DFSDM1
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 48;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV17;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm1_filter0)
{

	DmaRecHalfBuffCplt = 1;

}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm1_filter0)
{

  DmaRecBuffCplt = 1;

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	 while (1)
	  {
		  BSP_LED_Toggle(LED4);
		  HAL_Delay(1000);
	  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
