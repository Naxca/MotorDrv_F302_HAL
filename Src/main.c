/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void simpleDelay(uint16_t delayCnt);
uint8_t drv10983_Read(uint8_t regAddr);

void HAL_SYSTICK_Callback(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t spiTxData[4] = {0xFF,0xFF,0xFF,0xFF};
  uint8_t spiTxSize = 2;
  uint8_t spiRxData[4];
	uint8_t spiOutData[10];
  uint16_t spiTimeout = 10;
	uint16_t angle;
  
  uint16_t devWriteCmd = 0xA4;
  uint16_t devReadCmd = 0xA5;
  uint8_t enSiData[2] = {0x03,0x40};
  uint8_t i2cData[10] = {0x03,0x40,0x02,0x03,0x04};
  uint8_t i2cTempData[20];
 
  uint8_t eeprom_program_key[2] = {0x02,0xB6};
	uint8_t eeprom_eeWrite[2] = {0x03,0x50};
	
  uint16_t size = 3;
  uint32_t timeOut = 10;
  uint16_t delay = 200;
  uint16_t speed;
	/*
  uint8_t regWDSet[28] = {
    0x20, 0x4A,  //1101001: 5.57ohm
		0x21, 0x0A,  //0101000: 28mv/hz
    0x22, 0x2A,  //Tdelay
    0x23, 0x00,  //
    0x24, 0x78,
    0x25, 0x3F,
    0x26, 0x08,
    0x27, 0xFC,
    0x28, 0x69,
    0x29, 0x37,
    0x2A, 0x14,
    0x2B, 0x04,
	0x00, 0x10,
	0x01, 0x00				//0x80
  };*/
	

//	  uint8_t regWDSet[28] = {
//    0x20, 0x5A,  //1101001: 5.57ohm
//		0x21, 0x5C,  //0101000: 28mv/hz
//    0x22, 0x2A,  //Tdelay
//    0x23, 0x00,  //
//    0x24, 0xd2,
//    0x25, 0xfd,
//    0x26, 0xbf,
//    0x27, 0xFC,
//    0x28, 0x69,
//    0x29, 0xbf,
//    0x2A, 0x4,
//    0x2B, 0xf,
//	0x00, 0xff,
//	0x01, 0x81				
//  };


	  uint8_t regWDSet[28] = {
    0x20, 0x5A,  //1101001: 5.57ohm
		0x21, 0x5c,  //0101000: 28mv/hz
    0x22, 0x4,  //Tdelay
    0x23, 0x00,  //
    0x24, 0xc2,
    0x25, 0xff,
    0x26, 0x3c,
    0x27, 0x2,
    0x28, 0xf,
    0x29, 0x17,
    0x2A, 0x4,
    0x2B, 0xc,
	0x00, 0x66,
	0x01, 0x80				
  };

  uint8_t regDftSet[24] = {
    0x20, 0x4A,
    0x21, 0x4E,
    0x22, 0x2A,
    0x23, 0x00,
    0x24, 0x98,
    0x25, 0xE4,
    0x26, 0x7A,
    0x27, 0xFC,
    0x28, 0x69,
    0x29, 0xB7,
    0x2A, 0xAD,
    0x2B, 0x0C
  };  
  
  
  uint8_t regSPDctrl[2] = {
	0x01, 0x00
  };

  int i=0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USB_PCD_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
//  HAL_SuspendTick();
  /* test pulse */
  GPIOB->ODR = TP6_Pin;
  __nop();
  __nop();
  GPIOB->ODR = 0x0000U;
  GPIOB->ODR = TP6_Pin;
  GPIOB->ODR = 0x0000U;

  //enable Sidata bit 10983
  HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, enSiData, 2, timeOut);
  
	for (i = 0; i < 14; i++)
	{
	  HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, &regWDSet[2*i], 2, timeOut);
	  delay = 20;
	  while(delay--){};
	}
	
  /* set OverRide to 0, use PWM, forbid i2c SPD ctrl */
//  HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, regSPDctrl, 2, timeOut);
	
 // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    uint8_t tempAddr = 0x00;
    GPIOB->ODR = TP6_Pin;
    GPIOB->ODR = 0x0000U;

	/* read regs of drv10983 addr 0x20 - 0x2B */
	//USART1->TDR = 0xFF;
		
		/*
		    tempAddr = 0x20;
    for (i = 0; i < 12; i++)
    {
      HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, &tempAddr, 1, timeOut);
      delay = 20;
      while(delay--){};
      HAL_I2C_Master_Receive(&hi2c1, devReadCmd, i2cTempData + i, 1, timeOut);
      // HAL_UART_Transmit(&huart1, i2cTempData + i, 1, timeOut);
      tempAddr += 0x01;
	  USART1->TDR = i2cTempData[i];
	  simpleDelay(200);
    }*/
		
		tempAddr = 0x11;
    for (i = 0; i < 2; i++)
    {
      HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, &tempAddr, 1, timeOut);
      delay = 20;
      while(delay--){};
      HAL_I2C_Master_Receive(&hi2c1, devReadCmd, i2cTempData + i, 1, timeOut);
      // HAL_UART_Transmit(&huart1, i2cTempData + i, 1, timeOut);
      tempAddr += 0x01;
	 // USART1->TDR = i2cTempData[i];
	  simpleDelay(200);
    }
		
		speed = i2cTempData[0]*256 + i2cTempData[1];
		i2cTempData[0] = (speed/1000)%10+0x30;	
		i2cTempData[1] = (speed/100)%10+0x30;
		i2cTempData[2] = (speed/10)%10+0x30;
		i2cTempData[3] = speed%10 +0x30;		
		i2cTempData[4] = 0x0d;
		i2cTempData[5] = 0x0a;
	//	 HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, eeprom_program_key, 2, timeOut);
	//	 HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, eeprom_eeWrite, 2, timeOut);
		
//    HAL_UART_Transmit(&huart1, i2cTempData, 6, timeOut);

//    tempAddr = 0x20;
//    for (i = 0; i < 12; i++)
//    {
//      i2cTempData[i] = drv10983_Read(tempAddr);
//      tempAddr += 0x01;
//    }
//    HAL_UART_Transmit(&huart1, i2cTempData, 12, timeOut);
	  
	/* read regs of drv10983 addr 0x00 - 0x01 */
//    tempAddr = 0x00;
//    for (i = 0; i < 2; i++)
//    {
//      HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, &tempAddr, 1, timeOut);
//      delay = 20;
//      while(delay--){};
//      HAL_I2C_Master_Receive(&hi2c1, devReadCmd, i2cTempData + i, 1, timeOut);
//      // HAL_UART_Transmit(&huart1, i2cTempData + i, 1, timeOut);
//      tempAddr += 0x01;
//    }
//     HAL_UART_Transmit(&huart1, i2cTempData, 2, timeOut);

	/* read regs of drv10983 addr 0x10 - 0x1E */
//    tempAddr = 0x00;
//    for (i = 0; i < 14; i++)
//    {
//      HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, &tempAddr, 1, timeOut);
//      delay = 20;
//      while(delay--){};
//      HAL_I2C_Master_Receive(&hi2c1, devReadCmd, i2cTempData + i, 1, timeOut);
//      // HAL_UART_Transmit(&huart1, i2cTempData + i, 1, timeOut);
//      tempAddr += 0x01;
//    }
//     HAL_UART_Transmit(&huart1, i2cTempData, 12, timeOut);	
	
//    HAL_UART_Transmit_DMA(&huart1, i2cTempData, 12);
    // HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, i2cTempData, 1, timeOut);
    // delay = 20;
    // while(delay--);
    // delay = 2000;
    // HAL_I2C_Master_Receive(&hi2c1, devReadCmd, i2cTempData, 1, timeOut);
    // HAL_UART_Transmit(&huart1, i2cTempData, 1, timeOut);
//  HAL_I2C_Master_Receive(&hi2c1, (uint16_t)devAddr, i2cData, (uint16_t)size, (uint32_t)timeOut);
    delay = 2000;
    while(delay--){};

    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    delay = 20;
    while(delay--){};
    HAL_SPI_TransmitReceive(&hspi1, spiTxData, spiRxData, spiTxSize, spiTimeout);
    delay = 20;
    while(delay--){};
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
//    HAL_UART_Transmit(&huart1, spiRxData, spiTxSize, timeOut);
		spiRxData[0] = spiRxData[0] & ~~0xC0;
		angle = spiRxData[0]*256 + spiRxData[1];
		spiOutData[0] = (angle/10000)%10+0x30;	
		spiOutData[1] = (angle/1000)%10+0x30;
		spiOutData[2] = (angle/100)%10+0x30;
		spiOutData[3] = (angle/10)%10+0x30;
		spiOutData[4] = angle%10 +0x30;
		spiOutData[5] = 0x0d;
		spiOutData[6] = 0x0a;
		
    HAL_UART_Transmit(&huart1, spiOutData, 7, timeOut);

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void simpleDelay(uint16_t delayCnt)
{
  uint16_t delay = delayCnt;
  while(delay--){};
}

uint8_t drv10983_Read(uint8_t regAddr)
{
  uint8_t tempAddr;
  uint8_t tempData;
  tempAddr = regAddr;
  HAL_I2C_Master_Transmit(&hi2c1, DEVWRITECMD, &tempAddr, 1, 10);
  simpleDelay(20);
  HAL_I2C_Master_Receive(&hi2c1, DEVREADCMD, &tempData, 1, 10);
  return tempData;
}

void HAL_SYSTICK_Callback(void)
{
  
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
