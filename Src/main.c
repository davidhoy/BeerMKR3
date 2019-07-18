/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void spi_flash_test(void)
{
    uint16_t pageAddr = 0x123;
    const char wmsg[] = "This is a test message";
    char rmsg[sizeof(wmsg)] = {0};
    HAL_StatusTypeDef res1, res2;

    // read the device id
    {
        uint8_t devid_cmd[1] = { 0x9F };
        uint8_t devid_res[5];

        HAL_GPIO_WritePin(Flash_SPI1_NSS_GPIO_Port, Flash_SPI1_NSS_Pin, GPIO_PIN_RESET);
        res1 = HAL_SPI_Transmit_DMA(&hspi1, devid_cmd, sizeof(devid_cmd));
        res2 = HAL_SPI_Receive_DMA(&hspi1, devid_res, sizeof(devid_res));
        HAL_GPIO_WritePin(Flash_SPI1_NSS_GPIO_Port, Flash_SPI1_NSS_Pin, GPIO_PIN_SET);

        if((res1 != HAL_OK) || (res2 != HAL_OK)) {
            char msg[256];
            snprintf(msg, sizeof(msg), "Error during getting the device id, res1 = %d, res2 = %d\r\n", res1, res2);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return;
        }

        {
            char msg[256];
            snprintf(msg, sizeof(msg),
                "Manufacturer ID: 0x%02X\r\n"
                "Device ID (byte 1): 0x%02X\r\n"
                "Device ID (byte 2): 0x%02X\r\n"
                "Extended device information (EDI) string length: 0x%02X\r\n"
                "EDI byte 1: 0x%02X\r\n"
                "--------\r\n",
                devid_res[0], devid_res[1], devid_res[2], devid_res[3], devid_res[4]);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }

    // write test
    /* if(0) */ {
        uint8_t wcmd[4];
        // opcode
        wcmd[0] = 0x82; // 0x82 for buffer 1, 0x85 for buffer 2
        // for 512 bytes/page chip address is transfered in form:
        // 000AAAAA AAAAAAAa aaaaaaaa
        // wcmd[1] = (pageAddr >> 7) & 0x1F;
        // wcmd[2] = (pageAddr << 1) & 0xFE;
        // wcmd[3] = 0x00;

        // 00PPPPPP PPPPPPBB BBBBBBBB
        wcmd[1] = (pageAddr >> 6) & 0x3F;
        wcmd[2] = (pageAddr << 2) & 0xFC;
        wcmd[3] = 0x00;

        HAL_GPIO_WritePin(Flash_SPI1_NSS_GPIO_Port, Flash_SPI1_NSS_Pin, GPIO_PIN_RESET);
        res1 = HAL_SPI_Transmit(&hspi1, wcmd, sizeof(wcmd), HAL_MAX_DELAY);
        res2 = HAL_SPI_Transmit(&hspi1, (uint8_t*)wmsg, sizeof(wmsg), HAL_MAX_DELAY);
        HAL_GPIO_WritePin(Flash_SPI1_NSS_GPIO_Port, Flash_SPI1_NSS_Pin, GPIO_PIN_SET);

        if((res1 != HAL_OK) || (res2 != HAL_OK)) {
            char msg[256];
            snprintf(msg, sizeof(msg), "Error during writing the data, res1 = %d, res2 = %d\r\n", res1, res2);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return;
        }
    }

    // wait until device is ready (using HAL_Delay is error-prone!)
    {
        uint32_t delta = HAL_GetTick();
        uint32_t cnt = 0;

        uint8_t status_cmd[1] = { 0xD7 };
        uint8_t status_res[2];
        HAL_GPIO_WritePin(Flash_SPI1_NSS_GPIO_Port, Flash_SPI1_NSS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, status_cmd, sizeof(status_cmd), HAL_MAX_DELAY);
        do {
            cnt++;
            res1 = HAL_SPI_Receive(&hspi1, status_res, sizeof(status_res), HAL_MAX_DELAY);
            if(res1 != HAL_OK)
                break;
        } while (! (status_res[0] & 0x80)); // check RDY flag
        HAL_GPIO_WritePin(Flash_SPI1_NSS_GPIO_Port, Flash_SPI1_NSS_Pin, GPIO_PIN_SET);

        delta = HAL_GetTick() - delta;
        uint8_t protect = (status_res[0] >> 1) & 0x01;
        uint8_t page_size = (status_res[0]) & 0x01;
        uint8_t epe = (status_res[1] >> 5) & 0x01;
        uint8_t sle = (status_res[1] >> 3) & 0x01;
        char msg[256];
        snprintf(msg, sizeof(msg),
            "Await loop took %ld ms, %ld iterations\r\n"
            "Sector protection status: %s\r\n"
            "Page size: %d bytes\r\n"
            "Erase/program error: %s\r\n"
            "Sector lockdown command: %s\r\n"
            "--------\r\n",
            delta, cnt,
            protect ? "enabled" : "disabled",
            page_size ? 512 : 528,
            epe ? "ERROR!" : "no error",
            sle ? "enabled" : "disabled");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    // read test
    {
        uint8_t rcmd[5];
        // opcode
        rcmd[0] = 0x0B; // Note: 0x1B is supported by Adesto chips, but not Atmel chips, so use 0x0B

        // for 512 bytes/page chip address is transfered in form:
        // rcmd[1] = (pageAddr >> 7) & 0x1F;
        // rcmd[2] = (pageAddr << 1) & 0xFE;
        // rcmd[3] = 0x00;

        // 00PPPPPP PPPPPPBB BBBBBBBB
        rcmd[1] = (pageAddr >> 6) & 0x3F;
        rcmd[2] = (pageAddr << 2) & 0xFC;
        rcmd[3] = 0x00;

        // one dummy byte
        rcmd[4] = 0x00;

        HAL_GPIO_WritePin(Flash_SPI1_NSS_GPIO_Port, Flash_SPI1_NSS_Pin, GPIO_PIN_RESET);
        res1 = HAL_SPI_Transmit(&hspi1, rcmd, sizeof(rcmd), HAL_MAX_DELAY);
        res2 = HAL_SPI_Receive(&hspi1, (uint8_t*)rmsg, sizeof(rmsg), HAL_MAX_DELAY);
        HAL_GPIO_WritePin(Flash_SPI1_NSS_GPIO_Port, Flash_SPI1_NSS_Pin, GPIO_PIN_SET);

        if((res1 != HAL_OK) || (res2 != HAL_OK)) {
            char msg[256];
            snprintf(msg, sizeof(msg), "Error during reading the data, res1 = %d, res2 = %d\r\n", res1, res2);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return;
        }
    }

    if(memcmp(rmsg, wmsg, sizeof(rmsg)) == 0) {
        const char result[] = "Test passed!\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)result, sizeof(result)-1,
                          HAL_MAX_DELAY);
    } else {
        char msg[256];
        snprintf(msg, sizeof(msg), "Test failed: wmsg = '%s', rmsg = '%s'\r\n", wmsg, rmsg);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  for ( ;; )
  {
	  spi_flash_test();
	  HAL_Delay(1000);
  }
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
