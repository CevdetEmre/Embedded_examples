/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "st7735.h"
#include "fonts.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define DHT11_PORT GPIOC																															//DHT-11 data portu	
#define DHT11_PIN GPIO_PIN_8																													//DHT-11 data pini
#define YMAX	100
#define YMIN	0
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//DHT11 degiskenleri
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, Presence = 0;
uint16_t SUM, RH, TEMP_DHT11;

volatile uint16_t Temperature_DHT11 = 0;
volatile uint16_t  Humidity_DHT11 = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
GPIO_InitTypeDef GPIO_InitStruct;
TIM_HandleTypeDef htim21;																															//DHT11 icin	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM21_Init(void);
void drawaxes(void);
void plotData(void);
void 				Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void 				Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void 				delay(uint16_t time);
void 				DHT11_Start(void);
uint8_t 		DHT11_CheckResponse (void);
uint8_t 		DHT11_Read(void);
void 				DHT11_sensor_olcum(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void plotData(void)
{
	ST7735_PlotPoint(Temperature_DHT11,ST7735_GREEN);
	ST7735_PlotPoint(Humidity_DHT11,ST7735_CYAN);
	ST7735_PlotIncrement();	
}



void drawaxes(void){
    ST7735_Drawaxes(AXISCOLOR, BGCOLOR, "time", "temp", LIGHTCOLOR, "hum", ST7735_CYAN, YMAX, YMIN);
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
	MX_TIM21_Init();
	HAL_TIM_Base_Start(&htim21);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	ST7735_Init();
	ST7735_FillScreen(ST7735_BLACK);
	drawaxes();
	DHT11_Start();	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		DHT11_sensor_olcum();
		plotData();
		HAL_Delay(200);
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

  /* Enable HSE Oscillator and Activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;    // HSI-> LSE YAPILDI
  RCC_OscInitStruct.HSEState            = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLLMUL_6;
  RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLLDIV_3;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Set Voltage scale1 as MCU will run at 32MHz */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);	
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	
	GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
		/*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	

}


void delay(uint16_t time){
	__HAL_TIM_SET_COUNTER(&htim21,0);
	while ((__HAL_TIM_GET_COUNTER(&htim21)) < time);
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){

	GPIO_InitTypeDef Gpio_InitStruct;
	Gpio_InitStruct.Pin = GPIO_Pin;
	Gpio_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	Gpio_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx,&Gpio_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){

	GPIO_InitTypeDef Gpio_InitStruct;
	Gpio_InitStruct.Pin = GPIO_Pin;
	Gpio_InitStruct.Mode = GPIO_MODE_INPUT;
	Gpio_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx,&Gpio_InitStruct);
}

//-----------------------------------------------------------------
void DHT11_Start(void){
	// Dht11 Sensorunun olcume baslamasi icin gerekli fonk.
	Set_Pin_Output (DHT11_PORT,DHT11_PIN);  			
	HAL_GPIO_WritePin (DHT11_PORT,DHT11_PIN,GPIO_PIN_RESET);   
	delay(18000);																	
	HAL_GPIO_WritePin (DHT11_PORT,DHT11_PIN,GPIO_PIN_SET);   
	delay(30);																																				
	Set_Pin_Input(DHT11_PORT,DHT11_PIN);					
}


uint8_t DHT11_CheckResponse (void){
	uint8_t Response = 0;
	delay(40);
	if(!(HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN))){
		
		delay(80);
		if((HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN))){
			
			Response = 1;
		}
		else{
			
			Response = -1; // 255
		}
	}
	while((HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN)));
	
	return Response;
}


uint8_t DHT11_Read(void){

	uint8_t i,j;
	for(j=0;j<8;j++){
		
		while(!(HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN)));
		delay(40);
		if(!(HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN))){
			
			i &= ~(1 << (7-j));
		}
		else{
		
			i |= (1 << (7-j));
		}
		while((HAL_GPIO_ReadPin(DHT11_PORT,DHT11_PIN)));
	}
	return i;                       
}

void DHT11_sensor_olcum(void){

		DHT11_Start();
		Presence = DHT11_CheckResponse();
		Rh_byte1 = DHT11_Read();
		Rh_byte2 = DHT11_Read();
		Temp_byte1 = DHT11_Read();
		Temp_byte2 = DHT11_Read();
		SUM = DHT11_Read();
		
		TEMP_DHT11 = Temp_byte1;
		RH 	= Rh_byte1;
		Temperature_DHT11 = (float) TEMP_DHT11;
		Humidity_DHT11 = (float) RH;
		HAL_Delay(1000);	
}


/* USER CODE BEGIN 4 */
static void MX_TIM21_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 32-1;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 0xffff-1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim){
		

		if (htim -> Instance == TIM21){
		
				__HAL_RCC_TIM21_CLK_ENABLE();
		}
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim){

		if (htim -> Instance == TIM21){
		
			__HAL_RCC_TIM21_CLK_DISABLE();
		}		
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
