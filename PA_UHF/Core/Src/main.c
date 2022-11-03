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
#include <math.h>
#include "utils.h"
#include "i2c1.h"
#include "stdbool.h"
#include "bda4601.h"
#include "led.h"
#include "module.h"
#include "rs485.h"
#include "ad8363.h"
#include "led.h"
#include "max4003.h"
#include "adc.h"
#include "lm75.h"
#include <m24c64.h>
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

IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */
#define SYS_FREQ 64000000
#define APB_FREQ SYS_FREQ
#define HS16_CLK 16000000
#define BAUD_RATE 115200
#define LED_PIN PB1
#define DE_PIN PA15

#define LTEL_FRAME_SIZE 14
#define SIGMA_FRAME_SIZE 14
#define MEDIA_NUM 20
#define ADC_CHANNEL_NUM 6

#define IS_READY 0xaa

typedef enum ADC_INDEX {
	GAIN_i, POUT_i, VOLTAGE_i, CURRENT_i, VSWR_i, PIN_i
} ADC_INDEX_t;

#define STARTING_MILLIS 5000U

static const float ADC_CURRENT_FACTOR = 298.1818182f;
static const float ADC_VOLTAGE_FACTOR = 0.007404330f;

volatile uint16_t adcResultsDMA[ADC_CHANNEL_NUM];
uint16_t adc_values[ADC_CHANNEL_NUM][MEDIA_NUM];
uint16_t adc_media[ADC_CHANNEL_NUM];
uint16_t sum[ADC_CHANNEL_NUM];
uint8_t adc_counter = 0;

bool adcDataReady = false;

UART1_t uart1;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

uint8_t get_db_gain(uint16_t adc_gain);
uint8_t get_dbm_pout(uint16_t pout_adc);

void print_parameters(UART1_t *u, Module_t m) {
	sprintf((char*) u->tx_buffer,
			"Pout %d[dBm] Att %u[dB] Gain %u[dB] Pin %d[dBm] Curent %d[mA] Voltage %u[V]\r\n",
			m.pout, m.att, m.gain, m.pin, m.current, (uint8_t) m.voltage);
	uart1_send_frame((char*) u->tx_buffer, TX_BUFFLEN);
	uart1_clean_buffer(u);
}
void print_adc(UART1_t *u, uint16_t *adc) {
	sprintf((char*) u->tx_buffer,
			"Pout %d  \t Gain %u \t Curent %u \t Voltage %u\r\n", adc[POUT_i],
			adc[GAIN_i], adc[CURRENT_i], adc[VOLTAGE_i]);
	uart1_send_frame((char*) u->tx_buffer, TX_BUFFLEN);
	uart1_clean_buffer(u);
}

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
	Module_t pa;
	RS485_t rs485;
	AD8363_t pout;
	MAX4003_t pin;
	MAX4003_t vswr;
	LED_t led;
	uint8_t rcv_buff[2];
	uint8_t send_buff[2];

	module_init(&pa, POWER_AMPLIFIER, ID8);
	led_init();
	led_reset(&led);
	i2c1_init();
	uart1_init(HS16_CLK, BAUD_RATE, &uart1);
	lm75_init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
//  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */


// Calibrate The ADC On Power-Up For Better Accuracy
	HAL_ADCEx_Calibration_Start(&hadc1);
	uart1_send_str("PA init\n\r");
//	uint8_t addrs[5] = { 0 };  // 0x50 0x60 0x4f
//	i2c1_scanner(addrs);
	send_buff[0] = 30;

	m24c64_read_N(BASE_ADDR, &(pa.att), ATT_VALUE_ADDR, 1);

	if (pa.att > 0 && pa.att < 30)
		bda4601_set_initial_att(pa.att, STARTING_MILLIS);
	else
		bda4601_set_att(0, 3);

	m24c64_init_16uvalue(POUT_MAX_READY_ADDR, AD8363_ADC_MAX);
	m24c64_init_16uvalue(POUT_MIN_READY_ADDR, AD8363_ADC_MIN);
	m24c64_init_16uvalue(PIN_MAX_READY_ADDR, MAX4003_ADC_MAX);
	m24c64_init_16uvalue(PIN_MIN_READY_ADDR, MAX4003_ADC_MIN);
	m24c64_init_16uvalue(VSWR_MAX_READY_ADDR, MAX4003_ADC_MAX);
	m24c64_init_16uvalue(VSWR_MIN_READY_ADDR, MAX4003_ADC_MIN);

	m24c64_read_N(BASE_ADDR, rcv_buff, POUT_ADC_MIN_ADDR_0, 2);
	pout.min = (rcv_buff[0] << 8) | (rcv_buff[1] & 0xff);
	m24c64_read_N(BASE_ADDR, rcv_buff, POUT_ADC_MAX_ADDR_0, 2);
	pout.max = (rcv_buff[0] << 8) | (rcv_buff[1] & 0xff);
	m24c64_read_N(BASE_ADDR, rcv_buff, PIN_ADC_MIN_ADDR_0, 2);
	pin.min = (rcv_buff[0] << 8) | (rcv_buff[1] & 0xff);
	m24c64_read_N(BASE_ADDR, rcv_buff, PIN_ADC_MAX_ADDR_0, 2);
	pin.max = (rcv_buff[0] << 8) | (rcv_buff[1] & 0xff);
	m24c64_read_N(BASE_ADDR, rcv_buff, VSWR_ADC_MIN_ADDR_0, 2);
	vswr.min = (rcv_buff[0] << 8) | (rcv_buff[1] & 0xff);
	m24c64_read_N(BASE_ADDR, rcv_buff, VSWR_ADC_MAX_ADDR_0, 2);
	vswr.max = (rcv_buff[0] << 8) | (rcv_buff[1] & 0xff);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcResultsDMA, 4);
	led.ka_counter = HAL_GetTick();
	uart1.timeout = HAL_GetTick();

	while (1) {

		/* USER CODE BEGIN 3 */
		rs485.status = rs485_check_frame(&rs485, &uart1);
		switch (rs485.status) {
		case DATA_OK:
			rs485.cmd = uart1.rx_buffer[3];
			break;
		case NOT_VALID_FRAME:
			// TODO
			uart1_clean_buffer(&uart1);
			break;
		case WRONG_MODULE_ID:
			// TODO
			uart1_clean_buffer(&uart1);
			break;
		case CRC_ERROR:
			// TODO add crc
			uart1_clean_buffer(&uart1);
			break;
		case WAITING:

			break;
		case NO_DATA:

			break;
		default:
			rs485.cmd = NO_DATA;
			if (uart1.rx_buffer[0] != '\0')
				if (HAL_GetTick() - uart1.timeout > SECONDS(1))
					uart1_clean_buffer(&uart1);
			break;
		}

		switch (rs485.cmd) {
		case QUERY_PARAMETER_LTEL:
			pa.pr = max4003_get_dbm(&vswr, adc_media[VSWR_i]);
			pa.pout = ad8363_get_dbm(&pout, adc_media[POUT_i]);
			pa.current = ADC_CURRENT_FACTOR * adc_media[CURRENT_i] / 4096.0f;
			pa.gain = get_db_gain(adc_media[GAIN_i]);
			pa.vswr = module_vswr_calc(pa.pout, pa.pr);
			pa.pin = max4003_get_dbm(&pin, adc_media[PIN_i]);
			pa.voltage = ADC_VOLTAGE_FACTOR * adc_media[VOLTAGE_i]/ 4096.0f;
			rs485.len = 14;
			rs485.frame = (uint8_t*) malloc(14);
			uart1_send_frame((char*) rs485.frame, 14);
			free(rs485.frame);
			break;
		case SET_ATT_LTEL:
			pa.att = uart1.rx_buffer[6];
			bda4601_set_att(pa.att, 3);
			send_buff[0] = pa.att;
			m24c64_write_N(BASE_ADDR, send_buff, ATT_VALUE_ADDR, 1);
			sprintf((char*) uart1.tx_buffer, "Attenuation %u\r\n", pa.att);
			uart1_send_frame((char*) uart1.tx_buffer, TX_BUFFLEN);
			break;
		case SET_POUT_MAX:
			pout.max = adc_media[POUT_i];
			m24c64_store_16uvalue(POUT_MAX_READY_ADDR, pout.max);
			uart1_send_str("Saved Pout max value\n\r");
			break;
		case SET_POUT_MIN:
			pout.min = adc_media[POUT_i];
			m24c64_store_16uvalue(POUT_MIN_READY_ADDR, pout.min);
			uart1_send_str("Saved Pout min value\n\r");
			break;
		case SET_PIN_MAX:
			pin.max = adc_media[PIN_i];
			m24c64_store_16uvalue(PIN_MAX_READY_ADDR, pin.max);
			uart1_send_str("Saved Pin max value\n\r");
			break;
		case SET_PIN_MIN:
			pin.min = adc_media[PIN_i];
			m24c64_store_16uvalue(PIN_MAX_READY_ADDR, pin.min);
			uart1_send_str("Saved Pin min value\n\r");
			break;
		case SET_VSWR_MAX:
			vswr.max = adc_media[VSWR_i];
			m24c64_store_16uvalue(VSWR_MIN_READY_ADDR, vswr.max);
			uart1_send_str("Saved VSWR max value\n\r");
			break;
		case SET_VSWR_MIN:
			vswr.min = adc_media[VSWR_i];
			m24c64_store_16uvalue(VSWR_MAX_READY_ADDR, vswr.min);
			uart1_send_str("Saved Pout min value\n\r");
			break;
		case QUERY_PARAMETER_STR:
			print_parameters(&uart1, pa);
			break;
		case QUERY_ADC:
			print_adc(&uart1, adc_media);
			break;
		case QUERY_PARAMETER_SIGMA:
			pa.temperature = lm75_read();
			pa.pr = max4003_get_dbm(&vswr, adc_media[VSWR_i]);
			pa.pout = ad8363_get_dbm(&pout, adc_media[POUT_i]);
			pa.current = ADC_CURRENT_FACTOR * adc_media[CURRENT_i] / 4096.0f;
			pa.gain = get_db_gain(adc_media[GAIN_i]);
			pa.vswr = module_vswr_calc(pa.pout, pa.pr);
			pa.pin = max4003_get_dbm(&pin, adc_media[PIN_i]);
			pa.voltage = ADC_VOLTAGE_FACTOR * adc_media[VOLTAGE_i]/ 4096.0f;

			rs485_set_query_frame(&rs485, &pa);
			uart1_send_frame((char*) rs485.frame, 14);
			break;
		default:
		//	vswr.media = adc_media[VSWR_i];
		//	pout.media = adc_media[POUT_i];
		//	current.media = adc_media[CURRENT_i];
		//	gain.media = adc_media[GAIN_i];
		//	pin.media = adc_media[PIN_i];
		//	voltage.media = adc_media[VOLTAGE_i];
			rs485.cmd = NONE;

			break;
		}
		led_enable_kalive(led.sysrp_counter);

		//HAL_IWDG_Refresh(&hiwdg);
	}   //Fin while
    /* USER CODE END WHILE */
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 7;
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

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB9 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	for (int i = 0; i < ADC_CHANNEL_NUM; i++) {
		sum[i] -= adc_values[i][adc_counter];
		adc_values[i][adc_counter] = adcResultsDMA[i];
		sum[i] += adc_values[i][adc_counter];
		adc_media[i] = sum[i] / MEDIA_NUM;
	}
	adc_counter++;

	if (adc_counter >= MEDIA_NUM)
		adc_counter = 0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcResultsDMA, ADC_CHANNEL_NUM);
}

void USART1_IRQHandler(void) {
	uart1_read_to_frame(&uart1);
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
