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
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Value of VREF+ during production calibration in volts
#define CAL_VREF (3.0)

// Memory address of Internal Voltage Reference (VREFINT) - 0x1FFF 75AA (STM32L4S5 datasheet, Section 3.19.2)
#define VREFINT_CAL_ADDR (uint16_t*)(0x1FFF75AA) // VREFINT calibration value at VREF+ = 3.0 V (STM32L4+ ref manual)

// Temperature sensor memory addresses - found in stm32l476je document, Section 3.15.1 PAGE 42
#define TS_CAL1_ADDR (uint16_t*)(0x1FFF75A8) // Temperature sensor first calibration value at VREF+ = 3.0 V @ 30 degrees
#define TS_CAL2_ADDR (uint16_t*)(0x1FFF75CA) // Temperature sensor second calibration value  at VREF+ = 3.0 V @ 110 degrees
#define TS_CAL1_TEMP (30.0) // TS_CAL1 measurement temperature
#define TS_CAL2_TEMP (110.0) // TS_CAL1 measurement temperature

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function to start, read, and return ADC conversion
uint16_t readADC(){
	uint16_t ADCdata; // Initialize variable to store ADC conversion data

	// Got functions from 'STM32L4/L4+ HAL and low-layer drivers user manual'
	HAL_ADC_Start(&hadc1); // Activate the ADC peripheral and start conversions
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // This causes the processor to hang until conversion is over
	ADCdata = HAL_ADC_GetValue(&hadc1); // Retrieve conversion results
	HAL_ADC_Stop(&hadc1); // Stop conversion and disable the ADC peripheral

	return ADCdata; // Returns converted data: 12-bit ADC so range is: 0 -> 2^12 = 4095 (FS Digital Value)
}

// Function that uses macro helper to get VREF from VREFINT ADC measurement
float calibVREFMacro(uint16_t VREFINTdata){
	// From page 102 in UM1884, converts VREFINT value to voltage based on calibration value stored in memory during production
	float VREF = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(VREFINTdata, ADC_RESOLUTION_12B);
	return VREF;
}

// Function that uses macro helper to get TEMP from VREFINT ADC measurement
float calibTEMPMacro(uint16_t VREFINTdata, uint16_t ADCreading){
//_HAL_ADC_CALC_TEMPERATURE()
	float TEMP = __HAL_ADC_CALC_TEMPERATURE(VREFINTdata, ADCreading, ADC_RESOLUTION_12B);
	return TEMP;
}

// Function to reconfigure ADC to measure VREFINT channel
void configVREFINT(){
	ADC_ChannelConfTypeDef sConfig = {0}; // Need to declare sConfig to initialize the rest
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	  Error_Handler();
	}
}

// Function to reconfigure ADC to measure TEMPSENSOR channel
void configTEMPSENSOR(){
	ADC_ChannelConfTypeDef sConfig = {0}; // Need to declare sConfig to initialize the rest
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR; // Changed to read TEMPSENSOR channel
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  char buttonStatus = 0;

  uint16_t VREFINTdata = 0;
  uint16_t TEMPdata = 0;
  float VREF = 0; // VREF, defined as VREF+ in the texts, is the conversion voltage range for the ADC
  float VREFmacro = 0;
  float VREFINT_V = 0;
  float VREFINT_CAL_V = 0;

  float TEMPdata_adjusted = 0;
  float TEMP_C;
  float TEMP_Cmacro = 0;

  // VREFINT CALIBRATION
  uint16_t VREFINT_CAL = *VREFINT_CAL_ADDR; // VREFINT_CAL holds VREFINT calibration value

  uint16_t TS_CAL1 = *TS_CAL1_ADDR;
  uint16_t TS_CAL2 = *TS_CAL2_ADDR;

  // Using steps from PAGE 71 from UM1884 HAL User Manual as a guide
  // Optionally, perform an automatic ADC calibration to improve the conversion accuracy using function
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // PAGE 113, UM1884, Calibration prerequisite: ADC must be disabled (execute this function before HAL_ADC_Start() or after HAL_ADC_Stop() ).

  // Getting VREF so that we can use temperature sensor
  configVREFINT(); // Configure ADC for VREFINT measurement
  VREFINTdata = readADC(); // VREFINT measurement in bits
  VREFmacro = calibVREFMacro(VREFINTdata); // Using macro helper on Page 102 from UM1884, in mV

  // For UART printing
  char buffer[100] = {0};
  int data;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*---------------------------------STEP 5-------------------------------------*/

	  //uint8_t data[] = "HELLO WORLD \r\n";
	  // HAL_UART_Transmit (&huart1, data, sizeof (data), 10);

	  //Code to toggle LED2 output based on blue button state - TESTED, WORKS
	  //Get button state
	  buttonStatus = HAL_GPIO_ReadPin(blueButton_GPIO_Port, blueButton_Pin);

	  //toggle LED2 output based on state of the blue button - button is active low
	  if(buttonStatus == 1){
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

		  // ADC - Internal Reference Voltage Measurement
		  configVREFINT(); // Configure ADC for VREFINT measurement
		  VREFINT_CAL_V = (((float)VREFINT_CAL)*3.0)/4095; // VREFINT CALIBRATION VALUE in volts, Full scale digital value = 4095, Page 692 of RM0432
		  VREFINTdata = readADC(); // VREFINT measurement in bits

		  // FOUND VREF IN TWO WAYS -> equation and macro helper
		  VREF = (float)CAL_VREF * ((float)VREFINT_CAL/ (float)VREFINTdata); // VREF using equation on PAGE 691 in RM0432
		  VREFmacro = calibVREFMacro(VREFINTdata); // Using macro helper on Page 102 from UM1884
		  VREFmacro = VREFmacro/1000;

		  VREFINT_V = VREF * ((float)VREFINTdata/4095); // VREFINT in volts - VREFINT is within range: MIN-1.182, TYP-1.212, MAX-1.232

		  data = sprintf(buffer, "VREFINT: %f V\r\n", VREFINT_V);
		  HAL_UART_Transmit (&huart1, buffer, data, 100);

		  //HAL_Delay(1000);

	  }
	  else{
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

		  // Getting VREF so that we can use temperature sensor
		  configVREFINT(); // Configure ADC for VREFINT measurement
		  VREFINTdata = readADC(); // VREFINT measurement in bits
		  VREFmacro = calibVREFMacro(VREFINTdata); // Using macro helper on Page 102 from UM1884, in mV

		  configTEMPSENSOR(); // reconfiguring works
		  TEMPdata = readADC(); // TEMPSENSOR measurement in bits

		  // Got temperature in two ways: macro helper and equation
		  TEMP_Cmacro = calibTEMPMacro(VREFmacro, TEMPdata); // Calculating temperature using macro - PAGE 103, UM1884 - THIS WORKS

		  TEMPdata_adjusted = (float)TEMPdata * ((float)VREFINT_CAL/ (float)VREFINTdata);

		  // RM0432 says program with appropriate sampling time - need to check this

		  // Conversion to celcius is done with equation on PAGE 689 on RM0432
		  // Calibration values are at 3.15.1 on PAGE 41 on stm32l476je functional overview document

		  float gradient = ((float)TS_CAL2_TEMP - (float)TS_CAL1_TEMP)/((float)TS_CAL2 - (float)TS_CAL1);
		  float gain = (TEMPdata_adjusted - (float)TS_CAL1);

		  // Equation (PAGE 680 on RM0432) works, value matches up with the value from macro helper
		  TEMP_C = (uint16_t)(gradient*gain + 30.0);

		  data = sprintf(buffer, "TEMP: %f degrees celcius\r\n", TEMP_C);
		  HAL_UART_Transmit (&huart1, buffer, data, 100);

		  //HAL_Delay(1000);

	  }


	  /*---------------------------------STEP 2-------------------------------------*/
	  /*
	  // ADC - Internal Reference Voltage Measurement
	  configVREFINT(); // Configure ADC for VREFINT measurement
	  VREFINT_CAL_V = (((float)VREFINT_CAL)*3.0)/4095; // VREFINT CALIBRATION VALUE in volts, Full scale digital value = 4095, Page 692 of RM0432
	  VREFINTdata = readADC(); // VREFINT measurement in bits

	  // FOUND VREF IN TWO WAYS -> equation and macro helper
	  VREF = (float)CAL_VREF * ((float)VREFINT_CAL/ (float)VREFINTdata); // VREF using equation on PAGE 691 in RM0432
	  VREF2 = calibVREFMacro(VREFINTdata); // Using macro helper on Page 102 from UM1884

	  VREFINT_V = VREF * ((float)VREFINTdata/4095); // VREFINT in volts - VREFINT is within range: MIN-1.182, TYP-1.212, MAX-1.232
	  */
	  /*---------------------------------STEP 3-------------------------------------*/

	  // Section 21.4.32, PAGE 688 RM0432
	  // Temperature sensor output voltage changes linearly with temperature
	  // Offset of the line varies from chip to chip so needs to be calibrated based on saved calibration values in memory
	  // This needs to be done in cases where absolute temperature is needed

	  /*
	  configTEMPSENSOR(); // reconfiguring works
	  TEMPdata = readADC(); // TEMPSENSOR measurement in bits

	  // Got temperature in two ways: macro helper and equation

	  VREF_mV = VREF * 1000;

	  TEMP_C2 = calibTEMPMacro(VREF_mV, TEMPdata); // Calculating temperature using macro - PAGE 103, UM1884 - THIS WORKS

	  TEMPdata_C = (float)TEMPdata * ((float)VREFINT_CAL/ (float)VREFINTdata);

	  // RM0432 says program with appropriate sampling time - need to check this

	  // Conversion to celcius is done with equation on PAGE 689 on RM0432
	  // Calibration values are at 3.15.1 on PAGE 41 on stm32l476je functional overview document

	  // Equation doesn't give correct temperature - increased offset to give the right value (not sure if we can do this though lol)

	  TEMP_C = (uint16_t)(((((float)TS_CAL2_TEMP - (float)TS_CAL1_TEMP)/((float)TS_CAL2 - (float)TS_CAL1)) * (TEMPdata - (float)TS_CAL1)) + 30.0 + 17);
	  */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : blueButton_Pin */
  GPIO_InitStruct.Pin = blueButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(blueButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
