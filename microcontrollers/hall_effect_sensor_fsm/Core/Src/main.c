/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include "fsm.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define M_PI 3.14159265358979323846

#define ADC_BUF_LEN 4096
#define RX_BUF_LEN 64
#define FILTER_LEN 150

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// adc buffer
uint16_t adc_buf[ADC_BUF_LEN];
bool adc_dma_started = false;

// usart
uint8_t rx_byte;
uint8_t rx_buf[RX_BUF_LEN];
uint16_t rx_idx = 0;

// moving average filter
uint16_t filter_out = 0;
uint16_t filter_buf[FILTER_LEN];
uint16_t counter = 0;
uint32_t sum = 0;

// level detection in listening state
uint32_t hall_high_time = 0;

// available commands
typedef enum
{
  CMD_RAW = 0b00,
  CMD_MOVING_AVG = 0b01,
  CMD_RAND_NOISE = 0b10
} cmd_t;

// default command
cmd_t selected_cmd = CMD_RAW;

// List of state functions
state_func_t *const state_table[NUM_STATES] = {
  do_INIT,         // in state INIT
  do_WAIT_REQUEST, // in state WAIT_REQUEST
  do_ERROR,        // in state ERROR
  do_LISTENING,    // in state LISTENING
  do_WARNING,      // in state WARNING
  do_PAUSE,        // in state PAUSE
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */

void print(const char* msg)
{
  HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg, strlen(msg));
}

uint16_t map(uint16_t val, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  // Preso da https://docs.arduino.cc/language-reference/en/functions/math/map/#appendix

  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t toMilliVolt(uint16_t val)
{
  // Del tutto equivalente a map(val, 0, 4095, 0, 5000) ma usato per semplicità

  return (val * 5000) / 4095;
}

uint16_t correct(long val)
{
  if (val < 0) { return (uint16_t)0; }
  if (val > 5000) { return (uint16_t)5000; }
  return val;
}

float gaussian_noise_clt(float mean, float stddev)
{
  float sum = .0f;

  for (int i = 0; i < 6; i++)
  {
    sum += ((float)rand() / RAND_MAX);
  }
  sum -= 3.0f;

  return mean + stddev * sum;
}

void start_timer(uint32_t ms)
{
  uint32_t period = ms - 1;
  __HAL_TIM_SET_AUTORELOAD(&htim14, period);
  HAL_TIM_Base_Start_IT(&htim14);
}

void stop_timer()
{
  HAL_TIM_Base_Stop_IT(&htim14);
}

void cli_process_cmd(const char* cmd)
{
  if (strcmp(cmd, "raw") == 0) 
  {
    print("removed all filters\r\n");
    selected_cmd = CMD_RAW;
  }
  else if (strcmp(cmd, "moving average") == 0) 
  { 
    selected_cmd ^= CMD_MOVING_AVG;

    if (selected_cmd & CMD_MOVING_AVG)
    {
      print("moving average on\r\n");
    }
    else
    {
      print("moving average off\r\n");
    }
  }
  else if (strcmp(cmd, "random noise") == 0) 
  {
    selected_cmd ^= CMD_RAND_NOISE;

    if (selected_cmd & CMD_RAND_NOISE)
    {
      print("random noise on\r\n");
    }
    else
    {
      print("random noise off\r\n");
    }
  }
  else
  {
    char msg[64];
    snprintf(msg, sizeof(msg), "unknown command: %s\r\n", cmd);
    print(msg);
  }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

  state_t current_state = STATE_INIT;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  while (1)
  {
    current_state = run_state(current_state);
  }
  /* USER CODE END 3 */
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : HALL_DIGITAL_Pin */
  GPIO_InitStruct.Pin = HALL_DIGITAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HALL_DIGITAL_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

state_t do_INIT(void) 
{
  state_t next_state = STATE_WAIT_REQUEST;

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  if (HAL_Init() != HAL_OK) { next_state = STATE_ERROR; };

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    next_state = STATE_ERROR;
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    next_state = STATE_ERROR;
  }


  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  

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
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    next_state = STATE_ERROR;
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    next_state = STATE_ERROR;
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */


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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    next_state = STATE_ERROR;
  }

  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */



  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47999;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    next_state = STATE_ERROR;
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */


  /* USER CODE BEGIN 2 */

  srand(time(NULL));
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) { next_state = STATE_ERROR; }

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  return next_state;
}

// Function to be executed in state WAIT_REQUEST
// valid return states: STATE_LISTENING, STATE_ERROR
state_t do_WAIT_REQUEST(void) 
{
  state_t next_state = NO_CHANGE;
  
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  if (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_RESET) 
  {
    next_state = STATE_LISTENING;
    HAL_Delay(200);
  }

  return next_state;
}

// Function to be executed in state LISTENING
// valid return states: STATE_WARNING, STATE_ERROR, STATE_PAUSE
state_t do_LISTENING(void) 
{
  state_t next_state = NO_CHANGE;

  BSP_LED_On(LED_GREEN);
  char msg[64];
  uint16_t analog_val;
  uint16_t analog_mv;
  GPIO_PinState digital_val;
  filter_out = 0;

  if (!adc_dma_started)
  {
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN) != HAL_OK) { next_state = STATE_ERROR; }
    adc_dma_started = true;
  }

  analog_val = adc_buf[0];
  analog_mv = toMilliVolt(analog_val);
  digital_val = HAL_GPIO_ReadPin(HALL_DIGITAL_GPIO_Port, HALL_DIGITAL_Pin);

  if (selected_cmd & CMD_MOVING_AVG)
  {
    sum += analog_mv - filter_buf[counter];
    filter_out = sum / FILTER_LEN;
    filter_buf[counter] = analog_mv;
    counter++;
    if (counter == FILTER_LEN) { counter = 0; }
  }

  if (selected_cmd & CMD_RAND_NOISE)
  {
    analog_mv = (analog_mv + gaussian_noise_clt(0.0, 100));
  }

  if (selected_cmd == CMD_RAW)
  {
    filter_out = 0;
  }

  snprintf(msg, sizeof(msg), "%u,%u,%u\r\n", analog_mv, filter_out, digital_val);
  print(msg);

  if (digital_val == GPIO_PIN_SET)
  {
    if (hall_high_time == 0)
    {
      hall_high_time = HAL_GetTick();
    }
    else if (HAL_GetTick() - hall_high_time >= 5000)
    {
      HAL_ADC_Stop_DMA(&hadc1);
      adc_dma_started = false;
      next_state = STATE_WARNING;
      hall_high_time = 0;
    }
  }
  else { hall_high_time = 0; }

  if (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_RESET) 
  {
    HAL_ADC_Stop_DMA(&hadc1);
    adc_dma_started = false;
    BSP_LED_Off(LED_GREEN);
    next_state = STATE_PAUSE;
    HAL_Delay(200);
  }
 
  return next_state;
}

// Function to be executed in state PAUSE
// valid return states: STATE_LISTENING, STATE_ERROR
state_t do_PAUSE(void) 
{
  state_t next_state = NO_CHANGE;
  hall_high_time = 0;
  
  start_timer(1000);

  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  if (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_RESET)
  {
    stop_timer();
    next_state = STATE_LISTENING;
    HAL_Delay(200);
  }

  return next_state;
}

// Function to be executed in state WARNING
// valid return states: STATE_WAIT_REQUEST, STATE_ERROR
state_t do_WARNING(void) 
{
  state_t next_state = NO_CHANGE;
  
  stop_timer();
  BSP_LED_Off(LED_GREEN);
  print("WARNING\r\n");

  if (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_RESET) 
  {
    next_state = STATE_WAIT_REQUEST;
    HAL_Delay(200);
  }

  return next_state;
}

// Function to be executed in state ERROR
// valid return states: NO_CHANGE
state_t do_ERROR(void) 
{
  state_t next_state = NO_CHANGE;

  start_timer(200);
  print("ERROR\r\n");
  
  if (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_RESET) 
  {
    HAL_Delay(200);
    NVIC_SystemReset();
  }

  return next_state;
}

state_t run_state(state_t cur_state) 
{
  state_t new_state = state_table[cur_state]();
  if (new_state == NO_CHANGE) { new_state = cur_state; }

  return new_state;
};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim->Instance == TIM14)
  {
    BSP_LED_Toggle(LED_GREEN);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
  if (huart->Instance == USART2) {}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART2)
    {
      if (rx_byte == '\r' || rx_byte == '\n')
      {
        rx_buf[rx_idx] = '\0';
        cli_process_cmd((char*)rx_buf);
        rx_idx = 0;
      }
      else
      {
        if (rx_idx < RX_BUF_LEN - 1)
        {
          rx_buf[rx_idx++] = rx_byte;
        }
      }

      HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) {}
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
  {}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
