/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#include <ctype.h>
#include <string.h>

#define ARG_NUM 10
#define CMD_FUNC_CNT 8
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t as5048spiTxData[2] = {0xFF,0xFF};
uint8_t as5048spiTxSize = 2;
uint8_t as5048spiRxData[2] = {0x00,0x00};
uint8_t as5048spiOutData[20];
uint16_t as5048angle;
uint16_t as5048preAngle;
uint16_t as5048avgAngle[5];
uint8_t as5048avgCnt = 5;

uint8_t ctrlReg[8] = {
  0x00, 0x00,
  0x01, 0x00,
  0x02, 0x00,
  0x03, 0x00
};

uint8_t speedCtrl1[2] = {0x00,0xFF};
uint8_t speedCtrl2[2] = {0x01,0x80};
uint8_t devCtrl[2] = {0x02,0xB6};
uint8_t eeCtrl[2] = {0x03,0x50};

uint16_t devWriteCmd = 0xA4;
uint16_t devReadCmd = 0xA5;

uint8_t rxBuf[4];
uint8_t rxCmdBuf[21];
uint8_t testCmd[] = {'1','2','3'};
volatile uint8_t rxCmdCnt=0;
volatile uint8_t rxCmdLen=0;
volatile uint8_t sflag=0;

char cmd_cmd[21];
int cmd_arg[ARG_NUM];

typedef struct {
  char const *cmd_name;
  int32_t max_args;
  void (*handle)(int argc,int *cmd_arg);
//  void (*handle)(int argc,void *cmd_arg);
//  void (*handle)(void);
  char  *help;
}cmd_list_struct;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void simpleDelay(uint16_t delayCnt);
uint8_t drv10983_Read(uint8_t regAddr);

void HAL_SYSTICK_Callback(void);

static int32_t cmd_arg_analyze(uint8_t *rec_buf,unsigned int len);
static int32_t string_to_dec(uint8_t *buf,uint32_t len);

void printf_hello(int argc, int *cmd_arg);
void handle_arg(int argc, int *cmd_arg);
void toggle_led(int argc, int *cmd_arg);
void read_reg_10983(int argc, int *cmd_arg);
void set_speed(int argc, int *cmd_arg);
void set_reg(int argc, int *cmd_arg);
void start_motor(int argc, int *cmd_arg);
void stop_motor(int argc, int *cmd_arg);

void set_pwm(int pulseWidth);
void Motor_SPEED_GPIO_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{      
	uint8_t temp = ch;
  HAL_UART_Transmit(&huart1, &temp, 1, 0xFFFF);
	return ch;
}

const cmd_list_struct cmd_list[]={  
/* CMD    ARGnum    processfunc       help                         */
{"hello",   0,      printf_hello,   "hello                     -HelloWorld!"},
{"arg",     8,      handle_arg,     "arg<arg1> <arg2> ...      -Test, Print args"},
{"toggleled", 0, toggle_led, "no help" },
{"readreg", 0, read_reg_10983, "Read Drv10983 Regs"},
{"setspeed", 1, set_speed, "Set Motor speed"},
{"setreg", 2, set_reg, "Set DRV10983 Register"},
{"startmotor", 0, start_motor, "wakeup speed pin"},
{"stopmotor", 0, stop_motor, "sleep mode"}
};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t spiTxData[4] = {0xFF,0xFF,0xFF,0xFF};
  uint8_t spiTxSize = 2;
  uint8_t spiRxData[4];
	uint8_t spiOutData[10];
  uint16_t spiTimeout = 10;
	uint16_t angle;
  uint16_t preAngle=0;
  uint16_t deltaAngle=0;
  

  uint8_t enSiData[2] = {0x03,0x40};
  uint8_t i2cData[10] = {0x03,0x40,0x02,0x03,0x04};
  uint8_t i2cTempData[20];
  uint8_t eeprom_program_key[2] = {0x02,0xB6};
	uint8_t eeprom_eeWrite[2] = {0x03,0x50};
	
  uint16_t size = 3;
  uint32_t timeOut = 0xFFFF;
  uint32_t delay = 200;
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


  uint8_t regWDSet[28] = {
//    0x20, 0x5A,  //1101001: 5.57ohm
    0x20, 0x58,  //9.5 ohm for MY-3514C
//    0x21, 0x5c,  //0101000: 28mv/hz
    0x21, 0x29,  //145RPM/V ?mV/Hz for MY-3514C
//    0x22, 0x4,  //Tdelay
    0x22, 0x04,  //test for MY-3514
    0x23, 0x00,  //
    0x24, 0xc2,
    0x25, 0xff,
    0x26, 0x3c,
    0x27, 0x2,
    0x28, 0xf,
    0x29, 0x17,
    0x2A, 0x4,
    0x2B, 0xc,
    0x00, 0x8F,
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
    0x2B, 0x0E
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
//  HAL_TIM_Base_Start_IT(&htim2);

//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  Motor_SPEED_GPIO_Init();

//  HAL_SuspendTick();
  /* test pulse */
  GPIOB->BRR = TP6_Pin;
  __nop();
  __nop();
  GPIOB->BSRR = TP6_Pin;

  //enable Sidata bit 10983
  HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, enSiData, 2, timeOut);
  
//	for (i = 0; i < 14; i++)
//	{
//	  HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, &regWDSet[2*i], 2, timeOut);
//	}
	
  
  
  /* set OverRide to 0, use PWM, forbid i2c SPD ctrl */
//  HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, regSPDctrl, 2, timeOut);
	
 // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//  delay = 0x0FFFFFFF;
//  while(delay--){};
  
//  HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, eeprom_program_key, 2, timeOut);
//  HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, eeprom_eeWrite, 2, timeOut);
  
  HAL_UART_Receive_IT(&huart1, rxBuf, 1);
  printf("Start:\r\n");
  printf("Commands count: %d\r\n", CMD_FUNC_CNT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    int j=0;
    int argNum;
    
    HAL_Delay(4);
//    HAL_UART_Transmit(&huart2, txBuf, 4, 0xFFFF);
    switch(sflag)
    {
      case 0:
        break;
      case 1:
        sflag = 0;
        HAL_UART_Transmit(&huart1, rxCmdBuf+rxCmdCnt-1, 1, 0xFFFF);
        break;
      case 2: //Command received!
        sflag = 0;
        printf("\r\nYour Command is:\r\n");
        HAL_UART_Transmit(&huart1, rxCmdBuf, rxCmdLen, 0xFFFF); // send back command without last '\r'
        printf("\r\n");
        /* Call parser function */
//        if(parser(rxCmdBuf,rxCmdLen))
//        {
//          printf("\r\nOK!");
//        }
//        printf("\r\n");
//        if( 123 == string_to_dec(rxCmdBuf,rxCmdLen) )
//        {
//          printf("Match!\r\n");
//        }
        argNum = cmd_arg_analyze(rxCmdBuf, rxCmdLen);
        for( j=0; j<21; j++)
        {
          cmd_cmd[j]='\0';
        }
        if(!isdigit(rxCmdBuf[0]))
        {
          for(j=0; j<rxCmdLen; j++)
          {
            if( (j>0) && (rxCmdBuf[j]==' ' || rxCmdBuf[j]==0x0D) )
            {
              cmd_cmd[j]='\0';
              break;
            }
            else
            {
              cmd_cmd[j]=rxCmdBuf[j];
            }
          }
          printf("Command:");
          printf("%s\r\n",cmd_cmd);
        }

        printf("ArgNum: %d\r\n", argNum);
        for(j=0; j<argNum; j++)
        {
          printf("Arg%d: %d\r\n", j+1, cmd_arg[j]);
        }
        for( j=0; j<CMD_FUNC_CNT; j++)
        {
          printf("Trying to match Command %d\r\n", j+1);
          if(!strcmp((char *)cmd_cmd, cmd_list[j].cmd_name))
          {
            if( argNum<0 || argNum>cmd_list[j].max_args)
            {
              printf("Too much args!\r\n");
            }
            else
            {
              cmd_list[j].handle(argNum, cmd_arg);
            }
            break;
          }
        }
        break;
      case 3:
        sflag=0;
        printf("\r\nStack OverFlow!!!\r\n");
        break;
      case 4:
        sflag=0;
        printf("\r\nCtrl + c!\r\n");
        break;
    }
  }
  
//    printf("Test1234567789\n");

//    uint8_t tempAddr = 0x00;
//    GPIOB->BRR = TP6_Pin;
//    __nop();
//    __nop();
//    GPIOB->BSRR = TP6_Pin;
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
		
//		tempAddr = 0x11;
//    for (i = 0; i < 2; i++)
//    {
//      HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, &tempAddr, 1, timeOut);
//      delay = 20;
//      while(delay--){};
//      HAL_I2C_Master_Receive(&hi2c1, devReadCmd, i2cTempData + i, 1, timeOut);
//      // HAL_UART_Transmit(&huart1, i2cTempData + i, 1, timeOut);
//      tempAddr += 0x01;
//	 // USART1->TDR = i2cTempData[i];
//	  simpleDelay(200);
//    }
//		
//		speed = i2cTempData[0]*256 + i2cTempData[1];
//		i2cTempData[0] = (speed/1000)%10+0x30;	
//		i2cTempData[1] = (speed/100)%10+0x30;
//		i2cTempData[2] = (speed/10)%10+0x30;
//		i2cTempData[3] = speed%10 +0x30;		
//		i2cTempData[4] = 0x0d;
//		i2cTempData[5] = 0x0a;
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
//    delay = 0x04FFFFFF;
//    while(delay--){};

//    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
//    delay = 20;
//    while(delay--){};
//    HAL_SPI_TransmitReceive(&hspi1, spiTxData, spiRxData, 2, spiTimeout);
//    delay = 20;
//    while(delay--){};
//    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
////    HAL_UART_Transmit(&huart1, spiRxData, spiTxSize, timeOut);
//		spiRxData[0] = spiRxData[0] & ~0xC0;
//		angle = spiRxData[0]*256 + spiRxData[1];
//		deltaAngle = ( angle >= preAngle ) ? ( angle - preAngle ) : ( angle + 16384 -preAngle );
//		preAngle = angle;
//		
//		spiOutData[0] = (angle/10000)%10+0x30;	
//		spiOutData[1] = (angle/1000)%10+0x30;
//		spiOutData[2] = (angle/100)%10+0x30;
//		spiOutData[3] = (angle/10)%10+0x30;
//		spiOutData[4] = angle%10 +0x30;
//		spiOutData[5] = 0x0d;
//		spiOutData[6] = 0x0a;
		
//		spiOutData[0] = (deltaAngle/10000)%10+0x30;	
//		spiOutData[1] = (deltaAngle/1000)%10+0x30;
//		spiOutData[2] = (deltaAngle/100)%10+0x30;
//		spiOutData[3] = (deltaAngle/10)%10+0x30;
//		spiOutData[4] = deltaAngle%10 +0x30;
//		spiOutData[5] = 0x0d;
//		spiOutData[6] = 0x0a;
		
//    HAL_UART_Transmit(&huart1, spiOutData, 7, timeOut);

//  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
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
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  
{
//  uint16_t as5048angle;
//  uint16_t as5048preAngle;
  uint16_t deltaAngle;
  if( htim == &htim2 )
  {
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    simpleDelay(20);
    HAL_SPI_TransmitReceive(&hspi1, as5048spiTxData, as5048spiRxData, 2, 1);
    simpleDelay(20);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	as5048spiRxData[0] = as5048spiRxData[0] & 0x3F;
	as5048angle = as5048spiRxData[0]*256 + as5048spiRxData[1];
	deltaAngle = ( as5048angle >= as5048preAngle ) ? ( as5048angle - as5048preAngle ) : ( as5048angle + 16384 - as5048preAngle );
//	as5048angle = (uint16_t) (as5048angle/16384.0*36000.0);
	as5048preAngle = as5048angle;
//    deltaAngle = (uint16_t) (deltaAngle/16384.0*10000.0);
	
//	if(as5048avgCnt==4)
//	{
//	  as5048avgCnt = 0;
//	  as5048angle = ( as5048avgAngle[0] + as5048avgAngle[1] + as5048avgAngle[2] + as5048avgAngle[3] + as5048angle ) / 5;
//    /* Absolute angle output */
//	as5048spiOutData[0] = (as5048angle/10000)%10+0x30;	
//	as5048spiOutData[1] = (as5048angle/1000)%10+0x30;
//	as5048spiOutData[2] = (as5048angle/100)%10+0x30;
//	as5048spiOutData[3] = (as5048angle/10)%10+0x30;
//	as5048spiOutData[4] = as5048angle%10 +0x30;
//	as5048spiOutData[5] = 0x0d;
//	as5048spiOutData[6] = 0x0a;
////    HAL_UART_Transmit(&huart1, as5048spiOutData, 7, 2);
//	HAL_UART_Transmit_DMA(&huart1, as5048spiOutData, 7);
//	}
//	else
//	{
//	  as5048avgAngle[as5048avgCnt] = as5048angle;
//	  as5048avgCnt++;
//	}


    /* Absolute angle output */
//	as5048spiOutData[0] = (as5048angle/10000)%10+0x30;	
//	as5048spiOutData[1] = (as5048angle/1000)%10+0x30;
//	as5048spiOutData[2] = (as5048angle/100)%10+0x30;
//	as5048spiOutData[3] = (as5048angle/10)%10+0x30;
//	as5048spiOutData[4] = as5048angle%10 +0x30;
//	as5048spiOutData[5] = 0x0d;
//	as5048spiOutData[6] = 0x0a;
////    HAL_UART_Transmit(&huart1, as5048spiOutData, 7, 2);
//	HAL_UART_Transmit_DMA(&huart1, as5048spiOutData, 7);
	  
    /* Rotational speed out put */
	as5048spiOutData[0] = (deltaAngle/10000)%10+0x30;	
	as5048spiOutData[1] = (deltaAngle/1000)%10+0x30;
	as5048spiOutData[2] = (deltaAngle/100)%10+0x30;
	as5048spiOutData[3] = (deltaAngle/10)%10+0x30;
	as5048spiOutData[4] = deltaAngle%10+0x30;
	as5048spiOutData[5] = 0x0d;
	as5048spiOutData[6] = 0x0a;
//    HAL_UART_Transmit(&huart1, as5048spiOutData, 7, 2);
    HAL_UART_Transmit_DMA(&huart1, as5048spiOutData, 7);
  }
//  HAL_GPIO_WritePin(TP1_GPIO_Port, TP1_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(TP1_GPIO_Port, TP1_Pin, GPIO_PIN_RESET);
}  

void set_pwm(int pulseWidth)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulseWidth;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart1, rxBuf, 1);
  rxCmdBuf[rxCmdCnt] = rxBuf[0];
  rxCmdCnt++;
  if( 20 == rxCmdCnt )
  {
    rxCmdCnt = 0;
    sflag = 3;
  }

  else
  {
    switch(rxCmdBuf[rxCmdCnt-1])
    {
      case 0x0D:  //enter
        rxCmdLen = rxCmdCnt - 1;
        rxCmdCnt = 0;
        sflag = 2;
        break;
      case 0x03:  //ctrl+c
        rxCmdCnt = 0;
        sflag = 4;
        break;
      case 0x08:  //backspace
        if(rxCmdCnt>=1)
        {
          rxCmdBuf[rxCmdCnt] = 0;
          rxCmdCnt--;
        }
        sflag = 0;
        break;
      default:
        sflag = 1;
        break;
    }
  }
}

static int32_t cmd_arg_analyze(uint8_t *rec_buf,unsigned int len)
{
  uint32_t i;
  uint32_t blank_space_flag=0;
  uint32_t arg_num=0;
  uint32_t index[ARG_NUM];

  for(i=0;i<len;i++)
  {
    if(rec_buf[i]==0x20)
    {
      blank_space_flag=1;
      continue;
    }
    else if(rec_buf[i]==0x0D)
    {
      break;
    }
    else
    {
      if(blank_space_flag==1)
      {
        blank_space_flag=0;
        if(arg_num < ARG_NUM)
        {
          index[arg_num]=i;
          arg_num++;
        }
        else
        {
          return -1;
        }
      }
    }
  }

    for(i=0;i<arg_num;i++)
    {
        cmd_arg[i]=string_to_dec((unsigned char *)(rec_buf+index[i]),len-index[i]);
    }
    return arg_num;
}

static int32_t string_to_dec(uint8_t *buf,uint32_t len)
{
  uint32_t i=0;
  uint32_t base=10;
  int32_t neg=1;
  int32_t result=0;

  if((buf[0]=='0')&&(buf[1]=='x'))
  {
    base=16;
    neg=1;
    i=2;
  }
  else if(buf[0]=='-')
  {
    base=10;
    neg=-1;
    i=1;
  }
  for(;i<len;i++)
  {
    if(buf[i]==0x20 || buf[i]==0x0D)
    {
      break;
    }

    result *= base;
    if(isdigit(buf[i]))
    {
      result += buf[i]-'0';
    }
    else if(isxdigit(buf[i]))
    {
      result+=tolower(buf[i])-87;
    }
    else
    {
      result += buf[i]-'0';
    }
  }
  result *= neg;

  return result;
}

void printf_hello(int argc,int *cmd_arg)
{
  printf("Hello\r\n");
}

void handle_arg(int argc, int *cmd_arg)
{
  int i=0;
  for( i=0; i<argc; i++ )
  {
    printf("%d\r\n",cmd_arg[i]);
  }
}

void toggle_led(int argc, int *cmd_arg)
{
  HAL_GPIO_TogglePin(TP6_GPIO_Port, TP6_Pin);
}

void read_reg_10983(int argc, int *cmd_arg)
{
  printf("Drv10983 Reg:\r\n");
  int i;
  int delay;
  uint8_t i2cTempData[24];
  uint8_t tempAddr = 0x00;
  /* read regs of drv10983 addr 0x20 - 0x2B */
		
  tempAddr = 0x20;
  for (i = 0; i < 12; i++)
  {
    HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, &tempAddr, 1, 0xFFFF);
    delay = 20;
    while(delay--){};
    HAL_I2C_Master_Receive(&hi2c1, devReadCmd, &i2cTempData[i], 1, 0xFFFF);
    // HAL_UART_Transmit(&huart1, i2cTempData + i, 1, timeOut);
    tempAddr += 0x01;
      
    simpleDelay(200);
  }
  for(i = 0; i < 12; i++)
  {
    printf("Reg 0x%02X:  0x%02X\r\n", i+0x20, i2cTempData[i]);
  }
  
  tempAddr = 0x00;
  for (i = 0; i < 4; i++)
  {
    HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, &tempAddr, 1, 0xFFFF);
    delay = 20;
    while(delay--){};
    HAL_I2C_Master_Receive(&hi2c1, devReadCmd, &i2cTempData[i], 1, 0xFFFF);
    // HAL_UART_Transmit(&huart1, i2cTempData + i, 1, timeOut);
    tempAddr += 0x01;
      
    simpleDelay(200);
  }
  for(i = 0; i < 4; i++)
  {
    printf("Reg 0x%02X:  0x%02X\r\n", i, i2cTempData[i]);
  }
  
  tempAddr = 0x10;
  for (i = 0; i < 15; i++)
  {
    HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, &tempAddr, 1, 0xFFFF);
    delay = 20;
    while(delay--){};
    HAL_I2C_Master_Receive(&hi2c1, devReadCmd, &i2cTempData[i], 1, 0xFFFF);
    // HAL_UART_Transmit(&huart1, i2cTempData + i, 1, timeOut);
    tempAddr += 0x01;
      
    simpleDelay(200);
  }
  for(i = 0; i < 15; i++)
  {
    printf("Reg 0x%02X:  0x%02X\r\n", i+0x10, i2cTempData[i]);
  }
}

void set_speed(int argc, int *cmd_arg)
{
  printf("Speed: %d\r\n", cmd_arg[0]);  // DEC
//  printf("Speed: %04X\r\n", cmd_arg[0]);  // HEX
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  set_pwm(cmd_arg[0]);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void set_reg(int argc, int *cmd_arg)
{
  printf("Write Address 0x%02X with Data 0x%02X\r\n", cmd_arg[0], cmd_arg[1]);
  uint8_t addr_reg[2];
  addr_reg[0] = cmd_arg[0];
  addr_reg[1] = cmd_arg[1];
  HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, addr_reg, 2, 0xFFFF);
  printf("OK!\r\n");
}

void Motor_SPEED_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 -> Motor Speed */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Set PA0 to LOW */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

void start_motor(int argc, int *cmd_arg)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  uint8_t addr_reg[2];
  addr_reg[0] = 0x00;
  addr_reg[1] = 0xA0;
  HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, addr_reg, 2, 0xFFFF);
  addr_reg[0] = 0x01;
  addr_reg[1] = 0x80;
  HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, addr_reg, 2, 0xFFFF);
  printf("Motor started\r\n");
}

void stop_motor(int argc, int *cmd_arg)
{
  uint8_t addr_reg[2];
  addr_reg[0] = 0x00;
  addr_reg[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, devWriteCmd, addr_reg, 2, 0xFFFF);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  printf("Motor stoped\r\n");
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
