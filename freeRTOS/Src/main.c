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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "stdio.h"
#include <stdarg.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

osThreadId_t DHT11_TaskHandle;
osThreadId_t HCSR04_TaskHandle;
osThreadId_t LCD_TaskHandle;
osSemaphoreId_t myBinarySem01Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
void StartDHT11_Task(void *argument);
void StartHCSR04_Task(void *argument);
void StartLCD_Task(void *argument);

/* USER CODE BEGIN PFP */
uint8_t data_uart; // bien de luu data nhan duoc
uint8_t flag_uart=0; //khi co data den thi set len 1
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// callback UART receive 1 byte function
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == huart2.Instance) // check xem co dung uart2 hay khong
//	{
//		flag_uart = 1;
//		HAL_UART_Receive_IT(&huart2,&data_uart,1); // cho phep nhan data nhieu lan
//	}
//}

// ham printf
//_ARMABI int fputc(int c, FILE * stream)
//	{
//		HAL_UART_Transmit(&huart2, (uint8_t *)&c, 1, 100);
//		return 0;
//	}
	
//int __io_putchar(int ch)
//{
//	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0x10);
//	return ch;
//}

#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
	{
		HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
		return ch;
	}

//void vprint(const char *fmt, va_list argp)
//{
//    char string[200];
//    if(0 < vsprintf(string,fmt,argp)) // build string
//    {
//        HAL_UART_Transmit(&huart2, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
//    }
//}

//void my_printf(const char *fmt, ...) // custom printf() function
//{
//    va_list argp;
//    va_start(argp, fmt);
//    vprint(fmt, argp);
//    va_end(argp);
//}

// delay function in microseconds
void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < time);
}

//function to set the pins as outputs
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

//function to set pin as input
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL; //can be changed to PULLUP if no data is received from the pin
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


/*****HCSR04 FUNCTIONS*****/
//HCSR04 variables declarations
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;
float distance=0;
//define the pin and the port for sensor
#define TRIG_PIN GPIO_PIN_7
#define TRIG_PORT GPIOE

// Let's write the callback function
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}

// function to read data from hcsr04 signal pin
void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}

// function to display data from hcsr04
void Display_Dist (void)
{
	//void HCSR04_Read (void);

	lcd_put_cur(1,0);
	lcd_send_string ("Dist= ");
//HCSR04_Read();
	  lcd_send_data((Distance/100) + 48);   // 100th pos
	  lcd_send_data(((Distance/10)%10) +48);  // 10th pos
	  lcd_send_data((Distance%10)+48);  // 1st pos
	  lcd_send_string(" cm");
	  HAL_Delay(200);
}


/*****DHT11 FUNCTIONS*****/
//variables declarations
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;
float Temperature = 0;
float Humidity = 0;
uint8_t Presence = 0;

//Define the pin and the port for DHT11 Sensor
#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_1

//Function to display temperature
void Display_Temp (float Temp)
{
	char str[20] = {0};
	lcd_put_cur(0,0);
	
	sprintf (str, "TEMP:- %.2f", Temp);
	lcd_send_string(str);
	lcd_send_data('c');
}

//function to display RH
void Display_Rh (float Rh)
{
	char str[20] = {0};
	lcd_put_cur(1,0);
	
	sprintf (str, "Rh:- %.2f", Rh);
	lcd_send_string(str);
	lcd_send_data('%');
}

//Function to send the start signal
void DHT11_Start (void) 
{
	Set_Pin_Output (DHT11_PORT, DHT11_PIN); //set the dht pin as output
	/***********************************************/
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET); //initialize with data pin high
	HAL_Delay(1000); //wait for 1000 milliseconds
	/***********************************************/

	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET); //pull the pin low
	delay(18000); //wait 18 milliseconds
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, GPIO_PIN_SET); //pull the pin high
	delay(20); //wait for 20 microseconds
	Set_Pin_Input(DHT11_PORT, DHT11_PIN); //set the pin as input
}

//dh11 function to check response
uint8_t DHT11_Check_Response (void) 
{
	uint8_t Response = 0;
	delay(40); //first wait for 40 microseconds
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) //check for pin to be low
	{
		delay(80); //wait for 80 microseconds
		if((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1; //check if pin is high and return 1 to show sensor is present
		else Response = 0; //255
	}
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))); //wait for the pin to go low again
	
	return Response;
}

//function to read data from dht11 signal pin
uint8_t DHT11_Read (void)
{
	uint8_t i, j;
	for (j=0;j<8;j++)
	{
		while(!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))); //wait for the pin to change to high
		delay(40); //wait for 40 microseconds
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) //if the pin is low
		{
			i&= ~(1<<(7-j)); //write 0
		}
		else i|= (1<<(7-j)); //if the pin is high write 1
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))); //wait for the pin to go low
	}
	
	return i;
}

// function to update data each 1200ms
void DHT_Update(void)
{
	void DHT11_Start (void);
	uint8_t DHT11_Check_Response (void);
	uint8_t DHT11_Read (void);

		DHT11_Start();
		Presence = DHT11_Check_Response(); //record the response from the sensor
		
		//Five bytes of data
		Rh_byte1 = DHT11_Read ();
		Rh_byte2 = DHT11_Read ();
		Temp_byte1 = DHT11_Read ();
		Temp_byte2 = DHT11_Read ();
		SUM = DHT11_Read ();
		
		TEMP = Temp_byte1;
		RH = Rh_byte1;
		
		Temperature = (float) TEMP;
	
		Humidity = (float) RH;
	
}


/*****FreeRTOS*****/
void do_task1 (void)
	{
		void DHT_Update(void);
		uint8_t data_1[] = "task1 reading the DHT11 sensor...\n";
		HAL_UART_Transmit(&huart2, data_1, sizeof(data_1), HAL_MAX_DELAY);
		DHT_Update();
		printf("TEMP: %f \n", Temperature);
	}
	void do_task2 (void)
	{
		void HCSR04_Read (void);
		uint8_t data_2[] = "task2 reading the HCSR04 sensor...\n";
		HAL_UART_Transmit(&huart2, data_2, sizeof(data_2), HAL_MAX_DELAY);
		HCSR04_Read();
		printf("DIST= %f \n",Distance);
	}
	void do_task3 (void)
	{
		void Display_Dist (void);
		void Display_Temp (float Temp);
//		uint8_t data_3[] = "task3 displaying the temperature and distance measured on LCD...\n";
//		HAL_UART_Transmit(&huart2, data_3, sizeof(data_3), HAL_MAX_DELAY);
		Display_Temp(Temperature);
		Display_Dist();
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_UART_Receive_IT(&huart2,&data_uart,1);
	HAL_TIM_Base_Start(&htim1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  lcd_init();
	lcd_send_string("INITIALIZING.."); //display string on lcd
	HAL_Delay(2000); //wait for 2 seconds
	lcd_clear(); //clear lcd display

	
  /* USER CODE END 2 */

  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  const osSemaphoreAttr_t myBinarySem01_attributes = {
    .name = "myBinarySem01"
  };
  myBinarySem01Handle = osSemaphoreNew(1, 1, &myBinarySem01_attributes);

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
  /* definition and creation of DHT11_Task */
  const osThreadAttr_t DHT11_Task_attributes = {
    .name = "DHT11_Task",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 1024
  };
  DHT11_TaskHandle = osThreadNew(StartDHT11_Task, NULL, &DHT11_Task_attributes);

  /* definition and creation of HCSR04_Task */
  const osThreadAttr_t HCSR04_Task_attributes = {
    .name = "HCSR04_Task",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 1024
  };
  HCSR04_TaskHandle = osThreadNew(StartHCSR04_Task, NULL, &HCSR04_Task_attributes);

  /* definition and creation of LCD_Task */
  const osThreadAttr_t LCD_Task_attributes = {
    .name = "LCD_Task",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 1024
  };
  LCD_TaskHandle = osThreadNew(StartLCD_Task, NULL, &LCD_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDHT11_Task */
/**
  * @brief  Function implementing the DHT11_Task thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDHT11_Task */
void StartDHT11_Task(void *argument)
{
  /* USER CODE BEGIN 5 */
//	__IO uint32_t tick = osKernelGetTickCount();

  /* Infinite loop */
  while(1)
  {
		osSemaphoreAcquire(myBinarySem01Handle, osWaitForever);
    printf("DHT11_Task start: @%lu \n",osKernelGetTickCount());
//		tick += 50;
    do_task1();
		printf("DHT11_Task stop: @%lu \n",osKernelGetTickCount());
		osSemaphoreRelease(myBinarySem01Handle);
		osDelay(50);
		//osDelay(50);
//		osDelayUntil(tick);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartHCSR04_Task */
/**
* @brief Function implementing the HCSR04_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHCSR04_Task */
void StartHCSR04_Task(void *argument)
{
  /* USER CODE BEGIN StartHCSR04_Task */
//	__IO uint32_t tick = osKernelGetTickCount();
  /* Infinite loop */
  while(1)
  {
		osSemaphoreAcquire(myBinarySem01Handle, osWaitForever);
    printf("HCSR04_Task start: @%lu \n",osKernelGetTickCount());
//		tick += 50;
    do_task2();
		printf("HCSR04_Task stop: @%lu \n",osKernelGetTickCount());
		osSemaphoreRelease(myBinarySem01Handle);
		osDelay(50);
//		osDelayUntil(tick);
  }
  /* USER CODE END StartHCSR04_Task */
}

/* USER CODE BEGIN Header_StartLCD_Task */
/**
* @brief Function implementing the LCD_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCD_Task */
void StartLCD_Task(void *argument)
{
  /* USER CODE BEGIN StartLCD_Task */
//	__IO uint32_t tick = osKernelGetTickCount();
  /* Infinite loop */
  while(1)
  {
		osSemaphoreAcquire(myBinarySem01Handle, osWaitForever);
    printf("LCD_Task start: @%lu \n",osKernelGetTickCount());
//		tick += 50;
    do_task3();
		printf("LCD_Task stop: @%lu \n",osKernelGetTickCount());
		osSemaphoreRelease(myBinarySem01Handle);
		osDelay(50);
//		osDelay(50);
//		osDelayUntil(tick);
  }
  /* USER CODE END StartLCD_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
