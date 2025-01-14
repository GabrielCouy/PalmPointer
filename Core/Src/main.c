/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"
#include "bno055.h"
#include "liquidcrystal_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_READINGS 3

#define LEFT_CLICK_UPPER_THRESHOLD 600
#define LEFT_CLICK_LOWER_THRESHOLD 200

#define RIGHT_CLICK_UPPER_THRESHOLD 600
#define RIGHT_CLICK_LOWER_THRESHOLD 200

#define DOUBLE_CLICK_FIRST_TIME_LIMIT 30
#define DOUBLE_CLICK_SECOND_TIME_LIMIT 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ADC_ChannelConfTypeDef sConfig = {0};
bno055_t bno = (bno055_t){
      .i2c = &hi2c1, .addr = BNO_ADDR, .mode = BNO_MODE_IMU,
  };
bno055_vec4_t qua;

int16_t offsets[3] = {0};
uint8_t mode = 0;
//0 = normal
//1 = vertical
//2 = rotated

uint8_t DPI = 100;
bool on = true;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ADC_select_CH0(void) {
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	Error_Handler();
  }
}

void ADC_select_CH1(void) {
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	Error_Handler();
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize */
  HD44780_Init(2);
  HD44780_SetBacklight(0x8);

  /* Clear buffer */
  HD44780_Clear();

  /* Hide characters */
  HD44780_SetCursor(0,0);
  HD44780_PrintStr("MODE    ON   DPI");


  char base[16] = "Normal       ";
  snprintf(&base[13], 4, "%03d", DPI);

  HD44780_SetCursor(0, 1);
  HD44780_PrintStr(base);



     /* Blink cursor */
//     HD44780_Blink();

  HAL_ADC_Start_IT(&hadc1);//start conversion

  int readingsL[NUM_READINGS];  // Array to store FSR readings
  int readIndexL = 0;           // Index of the current reading
  int totalL = 0;               // Total of the readings
  int averageL = 0;             // The moving average

  int fsrValueL = 0; // Variable to store FSR value
  bool already_countedL = false;

  int readingsR[NUM_READINGS];  // Array to store FSR readings
  int readIndexR = 0;           // Index of the current reading
  int totalR = 0;               // Total of the readings
  int averageR = 0;             // The moving average

  int fsrValueR = 0; // Variable to store FSR value
  bool already_countedR = false;

  for (int i = 0; i < NUM_READINGS; i++) {
      readingsL[i] = 0;
      readingsR[i] = 0;
  }
  int clicksL = 0;
  int clicksR = 0;



  //DOUBLE CLICK LOGIC
  int onCount = 0;
  int onCountSecond = 0;
  bool incOnCount = false;
  bool incOnCountSecond = false;
  bool firstDoubleClick = false;
  bool secondDoubleClick = false;
  bool alreadyCountedDoubleClick = false;

  bool delayClickLeft = false;
  uint8_t delayCountLeft = 0;

  bool delayClickRight = false;
  uint8_t delayCountRight = 0;


  bool leftClick = false;
  bool leftPressed = false;
  bool rightClick = false;
  bool rightPressed = false;

  bool possibleDoubleHold = false;
  uint8_t doubleHoldCounter = 0;
  bool doubleHold = false;




  bno055_init(&bno);

  int8_t data[4] = {0,0,0,0}; // Example integers to send
  uint8_t txBuffer[4];              // Buffer to store the bytes to send

  bno.quaternion(&bno, &qua);
  printf("CALIBRATED");


  offsets[0] = qua.w * 100;
  offsets[1] = qua.x * 100;
  offsets[2] = qua.y * 100;

  // Encode integers into the transmission buffer
  uint32_t first_click_time = 0;
  uint32_t second_click_time = 0;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t start_time = HAL_GetTick();
	  bno.quaternion(&bno, &qua);

	  //Delay logic for double click

	  leftClick = ((delayCountLeft > 2 && delayClickLeft)) ? true : false;

	  rightClick = (delayCountRight > 2 && delayClickRight) ? true : false;

	  if (doubleHold) {
		  rightClick = true;
		  leftClick = true;
	  }



	  data[0] = -1 * (qua.x * 100 - offsets[1]); //(x_val * -1) / 3
	  data[1] = (qua.y * 100 - offsets[2]); //(y_val) / 3
	  data[2] = (leftClick);
	  data[3] = (rightClick);
//	  data[4] = DPI;

//	  if (rightClick) {
//		rightClick = false;
//	  }




	  memcpy(&txBuffer[0], &data[0], sizeof(int8_t));
	  memcpy(&txBuffer[1], &data[1], sizeof(int8_t));
	  memcpy(&txBuffer[2], &data[2], sizeof(int8_t));
	  memcpy(&txBuffer[3], &data[3], sizeof(int8_t));
//	  memcpy(txBuffer, data, sizeof(data));
//	  memcpy(&txBuffer[4], &data[4], sizeof(int8_t));

	  printf("x: %d, y: %d, L: %d, R: %d\n\r", data[0],data[1],data[2],data[3]);


	  if (on) {
		if (HAL_UART_Transmit(&huart1, txBuffer, sizeof(txBuffer), HAL_MAX_DELAY) != HAL_OK) {

			printf("UART Transmission Error: %d\n\r", HAL_UART_GetError(&huart1));
		}
//		HAL_Delay(30);s
	  }



	  	totalL = totalL - readingsL[readIndexL];
	  	totalR = totalR - readingsR[readIndexR];



	  	//ADC LOGIC-------------------------------------------------------------

	  	ADC_select_CH0();
	  	HAL_ADC_Start(&hadc1);//start conversion
	  	HAL_ADC_PollForConversion(&hadc1, 10000);
	  	fsrValueL = HAL_ADC_GetValue(&hadc1);
	  	HAL_ADC_Stop(&hadc1);

	  	ADC_select_CH1();
	  	HAL_ADC_Start(&hadc1);//start conversion
	  	HAL_ADC_PollForConversion(&hadc1, 10000);
	  	fsrValueR = HAL_ADC_GetValue(&hadc1);
	  	HAL_ADC_Stop(&hadc1);

	  	//-----------------------------------------------------------------------



	  	//FILTER LOGIC-------------------------------------------------------------

	  	// Add the new reading to the total
	  	totalL = totalL + fsrValueL;
	  	totalR = totalR + fsrValueR;

	  	// Store the new reading in the array
	  	readingsL[readIndexL] = fsrValueL;
	  	readingsR[readIndexR] = fsrValueR;


	  	// Advance to the next position in the array
	  	readIndexL = (readIndexL + 1) % NUM_READINGS;
	  	readIndexR = (readIndexR + 1) % NUM_READINGS;

	  	// Calculate the moving average
	  	averageL = totalL / NUM_READINGS;
	  	averageR = totalR / NUM_READINGS;

	  	//-----------------------------------------------------------------------



	      //CLICK LOGIC-------------------------------------------------------------

	      //LEFT SIDE
	  	if (averageL > LEFT_CLICK_UPPER_THRESHOLD && !already_countedL) {
	  	    ++clicksL;

	  	    incOnCount = true; //Begin double click logic

	  	    printf("Click Left: %d\n\r", clicksL);
	  	    leftClick = true;
	  	    delayClickLeft = true;
	  	    leftPressed = true;

	  	    already_countedL = true;
	  	}
	  	else if (averageL < LEFT_CLICK_LOWER_THRESHOLD) {
	  	    already_countedL = false;
		    if (leftClick) {
			    leftClick = false;
		    }
		    leftPressed = false;
	  	}

	  	//RIGHT SIDE
	  	if (averageR > RIGHT_CLICK_UPPER_THRESHOLD && !already_countedR) {
	  	    ++clicksR;


	  	    printf("Click RIGHT: %d\n\r", clicksR);

	  	    rightClick = true;
	  	    delayClickRight = true;
	  	    rightPressed = true;

	  	    already_countedR = true;
	  	}
	  	else if (averageR < RIGHT_CLICK_LOWER_THRESHOLD) {
	  	    already_countedR = false;
	  	    rightClick = false;
	  	    rightPressed = false;

	  	}

	  	//-----------------------------------------------------------------------
	  	if (possibleDoubleHold && rightPressed && leftPressed) {
	  		doubleHoldCounter += HAL_GetTick() - start_time;
	  	}
	  	if (!rightPressed || !leftPressed) {
	  		possibleDoubleHold = false;
	  		doubleHold = false;
	  		doubleHoldCounter = 0;
	  	}

	  	if (doubleHoldCounter > 500) {
	  		doubleHold = true;
	  	}

	  	if (delayClickLeft) {
			++delayCountLeft;
		}

		if (delayCountLeft > 3) {
			if (!leftPressed) {
				delayClickLeft = false;
				delayCountLeft = 0;
			}
			else {
				delayCountLeft = 3;
			}

		}


		if (delayCountRight > 3) {
			if (!rightPressed) {
				delayClickRight = false;
				delayCountRight = 0;
			}
			else {
				delayCountRight = 3;
			}

		}

		if (delayClickRight) {
			++delayCountRight;
		}

		if (delayCountRight > 3) {
			delayClickRight = false;
			delayCountRight = 0;
		}



	  	if (incOnCount) {
	  		++onCount;
	  		first_click_time += HAL_GetTick() - start_time;


	  	}
	  	if (incOnCountSecond) {
	  		++onCountSecond;
	  		second_click_time += HAL_GetTick() - start_time;
	  	}



	  	if (first_click_time > 0 && second_click_time > 0 && firstDoubleClick) {
	  		if (delayCountRight > 0) {//register right click
	  			secondDoubleClick = true;
	  		}

	  	}



	  	if (first_click_time > 0) {
	  		if (delayCountRight > 0) {//register right click
	  			firstDoubleClick = true;
	  			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);
	  		}
	  	}

	  	if (firstDoubleClick || secondDoubleClick) {
	  		leftClick = false;
	  		rightClick = false;
	  		delayClickLeft = false;
	  		delayCountLeft = 0;
	  		delayClickRight = false;
	  		delayCountRight = 0;

	  		possibleDoubleHold = true;
	  	}




	  	if (secondDoubleClick && !alreadyCountedDoubleClick) {
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
	  		alreadyCountedDoubleClick = true;

	  		on = !on;
	  		updateScreen(on, DPI, mode);

	  	}


	  	if (first_click_time > DOUBLE_CLICK_FIRST_TIME_LIMIT) {
	  		incOnCount = false;
	  		incOnCountSecond = true;
	  		first_click_time = 0;
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
	  	}
	  	if (second_click_time >= DOUBLE_CLICK_SECOND_TIME_LIMIT) {
	  		incOnCountSecond = false;
	  		second_click_time = 0;
	  		firstDoubleClick = false;
	  		secondDoubleClick = false;
	  		alreadyCountedDoubleClick = false;
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);
	  	}

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC2 PC3 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
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
