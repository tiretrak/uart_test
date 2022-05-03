/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "lcd.h"
#include <string.h>                // для работы со строками
//#include <stdin.h>
#include <stdlib.h>
#include <ctype.h>
//#include "stdio.h";
//#include "curemidi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define LCD_ADDR (0x27 << 1)       // адрес дисплея, сдвинутый на 1 бит влево (HAL работает с I2C-адресами, сдвинутыми на 1 бит влево)
//
//#define PIN_RS    (1 << 0)         // если на ножке 0, данные воспринимаются как команда, если 1 - как символы для вывода
//#define PIN_EN    (1 << 2)         // бит, по изменению сост. которого считывается информация
//#define BACKLIGHT (1 << 3)         // управление подсветкой
//
//#define LCD_DELAY_MS 5             // пауза перед высвечиванием символа
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
//uint8_t  inMsg[1];

//for MIDI rx
uint8_t midi_recieved_buf;
//char *buf[];

//uint8_t  outMsg[30];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//int _write(int file, char *ptr, int len)
//{
//	int i=0;
//	for(i=0 ; i<len ; i++)
//		ITM_SendChar((*ptr++));
//	return len;
//}
// функция, показывающаяя адреса подключенных модулей:
//void I2C_Scan ()
//{
//	HAL_StatusTypeDef res;                                                    // переменная, содержащая статус
//	char info[] = "Scanning I2C bus...\r\n";                                  // сообщение о начале процедуры
////	HAL_UART_Transmit(&huart5, (uint8_t*)info, strlen(info), HAL_MAX_DELAY);  // отправка сообщения по UART
//	/* &huart5 - адрес используемого UART
//	 * (uint8_t*)info - указатель на значение для отправки
//	 * strlen(info) - длина отправляемого сообщения (ф-ция strlen указывает количество символов)
//	 * HAL_MAX_DELAY - задержка
//	 */
//	for(uint16_t i = 0; i < 128; i++)                                         // перебор всех возможных адресов
//	{
//		res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, HAL_MAX_DELAY);                   // проверяем, готово ли устройство по адресу i для связи
//	    if(res == HAL_OK)                                                     // если да, то
//	    {
//	    	char msg[64];
//	    	// преобразование адреса i, на который откликнулись, в строку в виде 16тиричного значения:
//	    	snprintf(msg, sizeof(msg), "0x%02X", i);
//	    	// отправка номера откликнувшегося адреса
////	    	HAL_UART_Transmit(&huart5, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//	    	// переход на новую строчку
////	    	HAL_UART_Transmit(&huart5, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
//	    }
////	    else HAL_UART_Transmit(&huart5, (uint8_t*)".", 1, HAL_MAX_DELAY);
//	}
////	HAL_UART_Transmit(&huart5, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
//}

// функция для отправки данных, data - сами данные, flags - 1 (отправка данных) или 0 (отправка команд)
//void I2C_send(uint8_t data, uint8_t flags)
//{
//	HAL_StatusTypeDef res;
//	    for(;;) {                                                                     // бесконечный цикл
//	        res = HAL_I2C_IsDeviceReady(&hi2c1, LCD_ADDR, 1, HAL_MAX_DELAY);          // проверяем, готово ли устройство по адресу lcd_addr для связи
//	        if(res == HAL_OK) break;                                                  // если да, то выходим из бесконечного цикла
//	    }
//
//	uint8_t up = data & 0xF0;                 // операция �? с 1111 0000, приводит к обнулению последних бит с 0 по 3, остаются биты с 4 по 7
//	uint8_t lo = (data << 4) & 0xF0;          // тоже самое, но data сдвигается на 4 бита влево, т.е. в этой
//	                                           // переменной остаются  биты с 0 по 3
//	uint8_t data_arr[4];
//	data_arr[0] = up|flags|BACKLIGHT|PIN_EN;  // 4-7 биты содержат информацию, биты 0-3 конфигурируют работу
//	data_arr[1] = up|flags|BACKLIGHT;         // ублирование сигнала, на выводе Е в этот раз 0
//	data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
//	data_arr[3] = lo|flags|BACKLIGHT;
//
//	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
//	HAL_Delay(LCD_DELAY_MS);
//}

//void LCD_SendString(char *str)
//{
//    // *char по сути является строкой
//	while(*str) {                                   // пока строчка не закончится
//		I2C_send((uint8_t)(*str), 1);               // передача первого символа строки
//        str++;                                      // сдвиг строки налево на 1 символ
//    }
//}
int __io_putchar(int ch)
{
 // Write character to ITM ch.0
 ITM_SendChar(ch);
 return(ch);
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
  /* Configure LED3, LED4, LED5 and LED6 */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  //  MX_USART1_UART_Init();
  //  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  //start MIDI message reception with interrupt.
  HAL_UART_Receive_DMA(&huart2, &midi_recieved_buf, 1);

//  HAL_UART_Receive(&huart2,inMsg,4,10);

//  I2C_send(0b00110000,0);   // 8ми битный интерфейс
//  I2C_send(0b00000010,0);   // установка курсора в начале строки
//  I2C_send(0b00001100,0);   // нормальный режим работы
//  I2C_send(0b00000001,0);   // очистка дисплея
//
//  I2C_send(0b10000000,0);   // переход на 1 строку, тут не обязателен
//  LCD_SendString("  Hello Habr");
//  I2C_send(0b11000000,0);   // переход на 2 строку
//  LCD_SendString(" STM32 + LCD 1602");
////  I2C_send(0b00000001,0);   // очистка дисплея
//  I2C_send(0b10010100,0);   // переход на 3 строку
//  LCD_SendString(" +LCD 2004A");
//  I2C_send(0b11010100,0);   // переход на 4 строку
//  LCD_SendString(" library HAL");

//   LCD_ini();
//// sprintf(str,"start");
//   LCD_Clear();
//   LCD_SetPos(0, 1);
//   LCD_String("123");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  uint8_t  inMsg[3];
//  string text[10];
//  char text[3] ={0} ;
//  HAL_UART_Receive_IT(&huart2,(uint8_t*) str,1);
  while (1)
  {
//	  I2C_send(0b00000001,0);
//	  LCD_SendString(" library HAL");
//	  inMsg[0] = 0;
//	  inMsg[1] = 0;
//	  inMsg[2] = 0;
//	  HAL_UART_Receive (&huart2,inMsg,1,1);
//	  HAL_UART_Transmit(&huart2,inMsg,1,1);

//	  //		I2C_send(0b10000000,0);   // переход на 1 строку, тут не обязателен
//	  //		LCD_SendString("  Hello Habr");
//	  I2C_send(0b00000001,0);   // очистка дисплея
//	  I2C_send(0b11000000,0);   // переход на 2 строку
//	  LCD_SendString((char)&midi_recieved_buf);

//		I2C_send(0b00000001,0);   // очистка дисплея
//		I2C_send(0b10000000,0);   // переход на 1 строку, тут не обязателен
//		LCD_SendString((char)midi_recieved_buf);

//	  b = (char*) counter;

	  printf("«Hello STM32 world!\r\n");
//	  printf(b);
//	  printf("%d ", counter);
//	  counter++;
//	  I2C_send(0b00000001,0);   // очистка дисплея
//	  I2C_send(0b10000000,0);   // переход на 1 строку, тут не обязателен
//	  LCD_SendString((char*)midi_recieved_buf);

//	  HAL_UART_Transmit(&huart1,"cheers\n",7,1000);
//	  HAL_Delay(1000);

//	  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
//	  HAL_Delay(100);
//	  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
//	  HAL_Delay(100);
//	  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
//	  HAL_Delay(100);
//	  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
//	  HAL_Delay(100);
//	  LCD_Clear;
//	  HAL_Delay(100);
//      if(huart2.RxXferCount==0)
//      {
//		  str[8]=0;
//	  	HAL_UART_Receive_IT(&huart2,(uint8_t*) inMsg,1);

//	  	sprintf(text,"String 1");
//	  	sprintf(text,(char)inMsg[0]);
//		text[0] = (char)inMsg[0];
//		text[1] = (char)inMsg[1];
//		text[2] = (char)inMsg[2];
//	  	int count = sizeof(inMsg);
//	  	for (int i =0;i<count ;i++)
//	  	{
//	  	  text = inMsg[0];
//		  I2C_send(0b00000001,0);   // очистка дисплея
//		  I2C_send(0b10000000,0);   // переход на 1 строку, тут не обязателен
//		  LCD_SendString(text);
//		  HAL_Delay(100);
//		  I2C_send((uint8_t)(inMsg[0]), 1);
//		  I2C_send(0b11000000,0);   // переход на 2 строку
//		  LCD_SendString((int)inMsg[0]);
//	  	}

//		  LCD_SetPos(0, 0);
// 		  LCD_String((char*)inMsg[0]);
//      }
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance 			= USART1;
  huart1.Init.BaudRate 		= 115200;
  huart1.Init.WordLength 	= UART_WORDLENGTH_8B;
  huart1.Init.StopBits 		= UART_STOPBITS_1;
  huart1.Init.Parity 		= UART_PARITY_NONE;
  huart1.Init.Mode 			= UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl 	= UART_HWCONTROL_NONE;
  huart1.Init.OverSampling 	= UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance 			= USART2;
  huart2.Init.BaudRate 		= 31250;
  huart2.Init.WordLength 	= UART_WORDLENGTH_8B;
  huart2.Init.StopBits 		= UART_STOPBITS_1;
  huart2.Init.Parity 		= UART_PARITY_NONE;
  huart2.Init.Mode 			= UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl 	= UART_HWCONTROL_NONE;
  huart2.Init.OverSampling 	= UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//if(huart->Instance == USART2)
//{

//GPIOC->ODR |= GPIO_PIN_9;
//cureMidiBufferEnqueue(&midi_recieved_buf);
//HAL_UART_Receive_IT(&huart2, &midi_recieved_buf,1);
//buf = midi_recieved_buf;
//HAL_UART_Transmit	(&huart2,&midi_recieved_buf,1,10);
//HAL_UART_Transmit_IT(&huart2,&midi_recieved_buf,1);

HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);

for (int i=0;i<200;i++)
{
//	buf[i] = midi_recieved_buf;
}

//I2C_send(0b00000001,0);   // очистка дисплея
//I2C_send(0b10000000,0);   // переход на 1 строку, тут не обязателен
//LCD_SendString(buf);


//	}
  /* Prevent unused argument(s) compilation warning */
//  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
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
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

