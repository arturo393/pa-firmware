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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <uart1.h>
#include "eeprom.h"
#include "utils.h"
#include "i2c1.h"
#include "stdbool.h"
#include "bda4601.h"
#include "led.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define SYS_FREQ 64000000
#define APB_FREQ SYS_FREQ
#define HS16_CLK 16000000
#define BAUD_RATE 115200
#define LED_PIN PB1
#define DE_PIN PA15


#define RX_UART1_BUFFLEN  25
#define TX_UART1_BUFFLEN  100
#define LTEL_FRAME_SIZE 14
#define SIGMA_FRAME_SIZE 14
#define MEDIA_NUM 20

#define STARTING_SECS 5000U

static const uint8_t MODULE_ADDR = 0x08;
static const uint8_t MODULE_FUNCTION = 0x09;

static const uint8_t QUERY_PARAMETER_LTEL = 0x11;
static const uint8_t QUERY_PARAMETER_SIGMA = 0x12;
static const uint8_t QUERY_PARAMETER_STR = 0x15;
static const uint8_t QUERY_ADC = 0x16;

static const uint8_t SET_ATT_LTEL = 0x20;
static const uint8_t SET_POUT_MAX = 0x24;
static const uint8_t SET_POUT_MIN = 0x23;

static const float ADC_CURRENT_FACTOR = 298.1818182f;
static const float ADC_VOLTAGE_FACTOR = 0.007404330f;

static const uint8_t LTEL_START_MARK = 0x7e;
static const uint8_t LTEL_END_MARK = 0x7f;

static const int8_t POUT_DBM_MAX = 0;
static const int8_t POUT_DBM_MIN = -30;
static const uint16_t POUT_ADC_MAX = 1833;
static const uint16_t POUT_ADC_MIN = 488;

static uint8_t ATT_VALUE_ADDR = 0x00;
static uint8_t POUT_ADC_MAX_ADDR = 0x03;
static uint8_t POUT_ADC_MIN_ADDR = 0x05;
static uint8_t POUT_ISCALIBRATED_ADDR = 0x07;

static uint8_t POUT_ISCALIBRATED = 0xAA;

struct Lna {
	uint8_t attenuation;
	uint8_t gain;
	int8_t pout;
	uint8_t current;
	int8_t pin;
	float voltage;
};

volatile uint16_t adcResultsDMA[4];
uint16_t adc0_values[MEDIA_NUM];
uint16_t adc1_values[MEDIA_NUM];
uint16_t adc2_values[MEDIA_NUM];
uint16_t adc3_values[MEDIA_NUM];
uint16_t adc0_media;
uint16_t adc1_media;
uint16_t adc2_media;
uint16_t adc3_media;
uint8_t adc_counter = 0;

bool adcDataReady = false;

static const uint16_t LED_STATE_TIMEOUT = 1000;
static const uint8_t LED_ON_TIMEOUT = 50;
uint32_t led_counter = 0;

volatile uint8_t UART1_rxBuffer[RX_UART1_BUFFLEN] = { 0 };
uint8_t UART1_txBuffer[TX_UART1_BUFFLEN] = { 0 };
bool isDataReady = false;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

void sigma_set_parameter_frame(uint8_t *frame, struct Lna lna);
void ltel_set_parameter_frame(uint8_t *frame, struct Lna lna);
void lna_cmd_action();
void lna_uart_read();
bool lna_check_valid_str();
uint8_t get_db_gain(uint16_t adc_gain);
uint8_t get_dbm_pout(uint16_t pout_adc);
struct Lna get_lna();
void bda4601_set_att(uint8_t attenuation, uint8_t times);
void adc_media_calc();
void uart_clean_buffer();

//void uart_reset_reading(UART_HandleTypeDef *huart);
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
	//__disable_irq();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

	/* enable clock access ro GPIOA and GPIOB */
	SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);
	SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);

	/* PBA15 as output */
	SET_BIT(GPIOA->MODER, GPIO_MODER_MODE15_0);
	CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE15_1);



  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	MX_DMA_Init();
	MX_ADC1_Init();
//	MX_IWDG_Init();

	led_init();
	i2c1_init();
	uart1_init(HS16_CLK, BAUD_RATE);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  //MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
 // MX_I2C1_Init();
 // MX_USART1_UART_Init();
 // MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

// Calibrate The ADC On Power-Up For Better Accuracy
	HAL_ADCEx_Calibration_Start(&hadc1);
	uart1_send_str("LNA init\n\r");

	uint8_t attenuation = eeprom_1byte_read(ATT_VALUE_ADDR);
	while (HAL_GetTick() - STARTING_SECS > 0) {
	bda4601_set_att(attenuation, 3);
	}

if (eeprom_1byte_read(POUT_ISCALIBRATED_ADDR) != POUT_ISCALIBRATED) {
	eeprom_2byte_write(POUT_ADC_MIN_ADDR, POUT_ADC_MIN);
	eeprom_2byte_write(POUT_ADC_MAX_ADDR, POUT_ADC_MAX);
}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcResultsDMA, 4);
led_counter = HAL_GetTick();
while (1) {

//Fin function 1 second
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (isDataReady) {
		lna_cmd_action();
		isDataReady = false;
		uart_clean_buffer();
	} else {
		lna_uart_read();
		isDataReady = lna_check_valid_str();
	}
	if (HAL_GetTick() - led_counter > LED_STATE_TIMEOUT)
		led_counter = HAL_GetTick();
	else {
		if (HAL_GetTick() - led_counter > LED_ON_TIMEOUT)
			sys_rp_led_off();
		else
			sys_rp_led_on();
	}

	//HAL_IWDG_Refresh(&hiwdg);

}   //Fin while
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
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
  hi2c1.Init.Timing = 0x10707DBC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PA_HAB_Pin|LE_ATT_Pin|TEMP_HIGH_Pin|TEMP_OK_Pin
                          |CURR_H_Pin|CURR_N_Pin|CURR_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CLK_ATT_Pin|DATA_ATT_Pin|DE_485_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SYS_RP_GPIO_Port, SYS_RP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TEMP_INT_Pin */
  GPIO_InitStruct.Pin = TEMP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TEMP_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA_HAB_Pin LE_ATT_Pin TEMP_HIGH_Pin TEMP_OK_Pin
                           CURR_H_Pin CURR_N_Pin CURR_L_Pin */
  GPIO_InitStruct.Pin = PA_HAB_Pin|LE_ATT_Pin|TEMP_HIGH_Pin|TEMP_OK_Pin
                          |CURR_H_Pin|CURR_N_Pin|CURR_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_ATT_Pin DATA_ATT_Pin DE_485_Pin */
  GPIO_InitStruct.Pin = CLK_ATT_Pin|DATA_ATT_Pin|DE_485_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SYS_RP_Pin */
  GPIO_InitStruct.Pin = SYS_RP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SYS_RP_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void ltel_set_parameter_frame(uint8_t *frame, struct Lna lna) {
uint8_t crc_frame[2];
uint16_t crc;
frame[0] = LTEL_START_MARK;
frame[1] = MODULE_FUNCTION;
frame[2] = MODULE_ADDR;
frame[3] = QUERY_PARAMETER_LTEL;
frame[4] = 0x00;
frame[5] = 0x05;
frame[6] = 0x00;
frame[7] = lna.attenuation;
frame[8] = lna.gain;
frame[9] = lna.pout;
frame[10] = lna.voltage;
crc = crc_get(&(frame[1]), 10);
memcpy(crc_frame, &crc, 2);
frame[11] = crc_frame[0];
frame[12] = crc_frame[1];
frame[13] = LTEL_END_MARK;
}

void sigma_set_parameter_frame(uint8_t *frame, struct Lna lna) {
uint8_t crc_frame[2];
uint16_t crc;
frame[0] = LTEL_START_MARK;
frame[1] = MODULE_FUNCTION;
frame[2] = MODULE_ADDR;
frame[3] = QUERY_PARAMETER_SIGMA;
frame[4] = 0x06;
frame[5] = lna.pout;
frame[6] = lna.attenuation;
frame[7] = lna.gain;
frame[8] = lna.current;
frame[9] = lna.voltage;
frame[10] = lna.pin;
crc = crc_get(&(frame[1]), 10);
memcpy(crc_frame, &crc, 2);
frame[11] = crc_frame[0];
frame[12] = crc_frame[1];
frame[13] = LTEL_END_MARK;
}

uint8_t get_db_gain(uint16_t adc_gain) {

if (adc_gain >= 3781)
	return 45;
else if (adc_gain < 3781 && adc_gain >= 1515)
	return 0.0022f * adc_gain + 36.6571f;
else if (adc_gain < 1515 && adc_gain >= 1188)
	return (0.0153f * adc_gain + 16.8349f);
else if (adc_gain < 1188 && adc_gain >= 1005)
	return (0.0273f * adc_gain + 2.540f);
else if (adc_gain < 1005 && adc_gain >= 897)
	return (0.0463f * adc_gain - 16.5278f);
else if (adc_gain < 897 && adc_gain >= 825)
	return (0.0694f * adc_gain - 37.2917f);
else if (adc_gain < 825 && adc_gain >= 776)
	return (0.1020f * adc_gain - 64.1837f);
else if (adc_gain < 776 && adc_gain >= 746)
	return (0.1667f * adc_gain - 114.333f);
else if (adc_gain < 746 && adc_gain >= 733)
	return (0.3846f * adc_gain - 276.9231f);
else if (adc_gain < 733 && adc_gain >= 725)
	return (0.625f * adc_gain - 453.125f);
else if (adc_gain < 725)
	return 0;
return 0;
}

uint8_t get_dbm_pout(uint16_t pout_adc) {
uint16_t adc_max = 0;
uint16_t adc_min = 0;

//	do {
adc_max = eeprom_2byte_read(POUT_ADC_MAX_ADDR);
HAL_Delay(2);
adc_min = eeprom_2byte_read(POUT_ADC_MIN_ADDR);
//	} while (adc_max == adc_min);

float m = (float) (POUT_DBM_MAX - POUT_DBM_MIN) / (float) (adc_max - adc_min);
float b = POUT_DBM_MAX - adc_max * m;

if (pout_adc > adc_max) {
	return POUT_DBM_MAX;
} else if (pout_adc < adc_min) {
	return POUT_DBM_MIN;
}

return (int8_t) (m * (float) pout_adc + b);
}

struct Lna get_lna() {

struct Lna lna;
adc_media_calc();
lna.pout = get_dbm_pout(adc0_media);
lna.current = ADC_CURRENT_FACTOR * adc1_media / 4096.0f;
lna.gain = get_db_gain(adc2_media);
lna.voltage = ADC_VOLTAGE_FACTOR * (float) adc3_media;
lna.attenuation = eeprom_1byte_read(ATT_VALUE_ADDR);
lna.pin = lna.pout - lna.gain + lna.attenuation;
return lna;
}

void lna_cmd_action() {
uint8_t lna_cmd = UART1_rxBuffer[3];
if (lna_cmd == QUERY_PARAMETER_LTEL) {
	struct Lna lna;
	uint8_t frame[14];
	lna = get_lna();
	ltel_set_parameter_frame(frame, lna);
	uart1_send_frame((char*) frame, 14);
} else if (lna_cmd == SET_ATT_LTEL) {
	uint8_t attenuation_value = UART1_rxBuffer[6];
	bda4601_set_att(attenuation_value, 3);
	eeprom_1byte_write(ATT_VALUE_ADDR, attenuation_value);
	sprintf(UART1_txBuffer, "Attenuation %u\r\n", attenuation_value);
	uart1_send_frame((char*) UART1_txBuffer, TX_UART1_BUFFLEN);
} else if (lna_cmd == SET_POUT_MAX) {
	eeprom_2byte_write(POUT_ADC_MAX_ADDR, adc0_media);
	HAL_Delay(5);
	eeprom_1byte_write(POUT_ISCALIBRATED_ADDR, POUT_ISCALIBRATED);
	uart1_send_str("Saved Pout max value\n\r");

} else if (lna_cmd == SET_POUT_MIN) {
	eeprom_2byte_write(POUT_ADC_MIN_ADDR, adc0_media);
	HAL_Delay(5);
	eeprom_1byte_write(POUT_ISCALIBRATED_ADDR, POUT_ISCALIBRATED);
	uart1_send_str("Saved Pout min value\n\r");

} else if (lna_cmd == QUERY_PARAMETER_STR) {
	struct Lna lna;
	lna = get_lna();
	sprintf(UART1_txBuffer,
			"Pout %d[dBm] Att %u[dB] Gain %u[dB] Pin %d[dBm] Curent %d[mA] Voltage %u[V]\r\n",
			lna.pout, lna.attenuation, lna.gain, lna.pin, lna.current,
			(uint8_t) lna.voltage);
	uart1_send_frame((char*) UART1_txBuffer, TX_UART1_BUFFLEN);

} else if (lna_cmd == QUERY_ADC) {
	adc_media_calc();
	sprintf(UART1_txBuffer,
			"Pout %d  \t Gain %u \t Curent %u \t Voltage %u\r\n", adc0_media,
			adc1_media, adc2_media, adc3_media);
	uart1_send_frame((char*) UART1_txBuffer, TX_UART1_BUFFLEN);

} else if (lna_cmd == QUERY_PARAMETER_SIGMA) {
	struct Lna lna;
	uint8_t frame[LTEL_FRAME_SIZE];
	lna = get_lna();
	sigma_set_parameter_frame(frame, lna);
	uart1_send_frame((char*) frame, LTEL_FRAME_SIZE);
}
}

void lna_uart_read() {
uint8_t rcvcount = 0;
uint32_t tickstart = HAL_GetTick();
bool timeout = false;
uint8_t timeout_value = 10;

while (rcvcount < RX_UART1_BUFFLEN && timeout == false) {
	if (((HAL_GetTick() - tickstart) > timeout_value)
			|| (timeout_value == 0U)) {
		timeout = true;
		if (READ_BIT(USART1->ISR, USART_ISR_ORE))
			SET_BIT(USART1->ICR, USART_ICR_ORECF);
		if (READ_BIT(USART1->ISR, USART_ISR_IDLE))
			SET_BIT(USART1->ICR, USART_ICR_IDLECF);
		if (READ_BIT(USART1->ISR, USART_ISR_FE))
			SET_BIT(USART1->ICR, USART_ICR_FECF);
	}

	if (READ_BIT(USART1->ISR, USART_ISR_RXNE_RXFNE))
		UART1_rxBuffer[rcvcount++] = USART1->RDR;
}
}

bool lna_check_valid_str() {
bool is_valid_data = false;
if (UART1_rxBuffer[0] == LTEL_START_MARK)
	if (UART1_rxBuffer[1] == MODULE_FUNCTION)
		if (UART1_rxBuffer[2] == MODULE_ADDR)
			for (int i = 3; i < RX_UART1_BUFFLEN && is_valid_data == false; i++)
				is_valid_data = UART1_rxBuffer[i] == LTEL_END_MARK ?
				true :
																		false;
return is_valid_data;
}



void uart_clean_buffer() {
for (int i = 0; i < TX_UART1_BUFFLEN; i++) {
	if (i < RX_UART1_BUFFLEN)
		UART1_rxBuffer[i] = 0x00;
	UART1_txBuffer[i] = 0x00;
}
}
void adc_media_calc() {
uint32_t sum0 = 0;
uint32_t sum1 = 0;
uint32_t sum2 = 0;
uint32_t sum3 = 0;

for (int i = 0; i < MEDIA_NUM; i++) {
	sum0 += adcResultsDMA[0];
	sum1 += adcResultsDMA[1];
	sum2 += adcResultsDMA[2];
	sum3 += adcResultsDMA[3];
}
adc0_media = sum0 / MEDIA_NUM;
adc1_media = sum1 / MEDIA_NUM;
adc2_media = sum2 / MEDIA_NUM;
adc3_media = sum3 / MEDIA_NUM;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
// TODO : se puede reemplazar leyendo el flag del registro
if (adc_counter < MEDIA_NUM) {
	adc0_values[adc_counter] = adcResultsDMA[0];
	adc1_values[adc_counter] = adcResultsDMA[1];
	adc2_values[adc_counter] = adcResultsDMA[2];
	adc3_values[adc_counter] = adcResultsDMA[3];
	adc_counter++;
} else {
	adc_counter = 0;
}
HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcResultsDMA, 4);

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
while (1) {
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
