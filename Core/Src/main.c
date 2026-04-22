/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "ir_transceiver.h"
#include "rc5_decode.h"
#include "rc5_encode.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_MS                 300U
#define PLAYER_ADDRESS_COUNT        32U
#define BLE_RX_DMA_BUFFER_SIZE      128U
#define BLE_COMMAND_BUFFER_SIZE     96U
#define BLE_TX_QUEUE_DEPTH          64U
#define BLE_TX_MAX_MESSAGE_SIZE     160U
#define BLE_STARTUP_BANNER_DELAY_MS 750U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
static uint8_t toggle_bit = 0U;
static uint8_t tx_address = 0U;
static uint8_t tx_command = 0U;
static volatile uint8_t button_pressed = 0U;
static uint32_t last_button_tick = 0U;
static uint32_t led_off_tick = 0U;
static uint8_t led_is_on = 0U;
static uint32_t total_hits = 0U;
static uint32_t hit_count_by_address[PLAYER_ADDRESS_COUNT];
static uint8_t ble_rx_dma_buffer[BLE_RX_DMA_BUFFER_SIZE];
static uint16_t ble_rx_last_pos = 0U;
static char ble_command_buffer[BLE_COMMAND_BUFFER_SIZE];
static uint16_t ble_command_length = 0U;
static volatile uint8_t ble_tx_busy = 0U;
static volatile uint8_t ble_tx_queue_head = 0U;
static volatile uint8_t ble_tx_queue_tail = 0U;
static volatile uint8_t ble_tx_queue_count = 0U;
static char ble_tx_queue[BLE_TX_QUEUE_DEPTH][BLE_TX_MAX_MESSAGE_SIZE];
static uint16_t ble_tx_lengths[BLE_TX_QUEUE_DEPTH];
static uint8_t ble_startup_banner_sent = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void App_SendDebug(const char *format, ...);
static void App_BlinkStatusLed(uint32_t duration_ms);
static void App_ProcessStatusLed(void);
static void App_RegisterHit(uint8_t address);
static uint8_t App_ParseUnsignedValue(const char *text, uint32_t min_value,
                                      uint32_t max_value, uint32_t *value_out);
static void Ble_StartReception(void);
static void Ble_ProcessRxDma(void);
static void Ble_ProcessRxChunk(const uint8_t *data, uint16_t length);
static void Ble_ProcessLine(const char *line);
static void Ble_ProcessAppCommand(const char *line);
static uint8_t Ble_TryParseCommandValue(const char *line, const char *prefix,
                                        uint32_t max_value, uint32_t *value_out);
static void Ble_ReportCurrentSettings(void);
static void Ble_ReportCurrentHits(void);
static void Ble_ResetHits(void);
static void Ble_AppQueueBytes(const uint8_t *data, uint16_t length);
static void Ble_AppQueueString(const char *text);
static void Ble_AppQueueFormatted(const char *format, ...);
static void Ble_QueueBytes(const uint8_t *data, uint16_t length);
static void Ble_KickTx(void);
static void Ble_SendStartupBanner(void);
static void Ble_ServiceStartupBanner(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_SendDebug(const char *format, ...)
{
  char buffer[BLE_TX_MAX_MESSAGE_SIZE];
  va_list args;
  int length;

  va_start(args, format);
  length = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (length <= 0)
  {
    return;
  }

  if (length >= (int)sizeof(buffer))
  {
    length = (int)sizeof(buffer) - 1;
  }

  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)length, 100);
}

static void App_BlinkStatusLed(uint32_t duration_ms)
{
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  led_is_on = 1U;
  led_off_tick = HAL_GetTick() + duration_ms;
}

static void App_ProcessStatusLed(void)
{
  if ((led_is_on != 0U) && ((int32_t)(HAL_GetTick() - led_off_tick) >= 0))
  {
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
    led_is_on = 0U;
  }
}

static void App_RegisterHit(uint8_t address)
{
  if (address < PLAYER_ADDRESS_COUNT)
  {
    hit_count_by_address[address]++;
    total_hits++;
  }
}

static uint8_t App_ParseUnsignedValue(const char *text, uint32_t min_value,
                                      uint32_t max_value, uint32_t *value_out)
{
  char *end_ptr;
  unsigned long parsed_value;

  if ((text == NULL) || (*text == '\0') || (value_out == NULL))
  {
    return 0U;
  }

  parsed_value = strtoul(text, &end_ptr, 0);
  if ((*end_ptr != '\0') || (parsed_value < min_value) || (parsed_value > max_value))
  {
    return 0U;
  }

  *value_out = (uint32_t)parsed_value;
  return 1U;
}

static void Ble_StartReception(void)
{
  ble_rx_last_pos = 0U;
  ble_command_length = 0U;
  ble_startup_banner_sent = 0U;

  if (HAL_UART_Receive_DMA(&huart1, ble_rx_dma_buffer, BLE_RX_DMA_BUFFER_SIZE) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_TC);
}

static void Ble_ProcessRxDma(void)
{
  uint16_t current_pos;

  current_pos = BLE_RX_DMA_BUFFER_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

  if (current_pos == ble_rx_last_pos)
  {
    return;
  }

  if (current_pos > ble_rx_last_pos)
  {
    Ble_ProcessRxChunk(&ble_rx_dma_buffer[ble_rx_last_pos], current_pos - ble_rx_last_pos);
  }
  else
  {
    Ble_ProcessRxChunk(&ble_rx_dma_buffer[ble_rx_last_pos],
                       BLE_RX_DMA_BUFFER_SIZE - ble_rx_last_pos);

    if (current_pos > 0U)
    {
      Ble_ProcessRxChunk(&ble_rx_dma_buffer[0], current_pos);
    }
  }

  ble_rx_last_pos = current_pos;
}

static void Ble_ProcessRxChunk(const uint8_t *data, uint16_t length)
{
  uint16_t index;

  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  for (index = 0U; index < length; index++)
  {
    const char current_char = (char)data[index];

    if ((current_char == '\r') || (current_char == '\n'))
    {
      if (ble_command_length > 0U)
      {
        ble_command_buffer[ble_command_length] = '\0';
        Ble_ProcessLine(ble_command_buffer);
        ble_command_length = 0U;
      }
    }
    else if ((current_char >= 32) && (current_char <= 126))
    {
      if (ble_command_length < (BLE_COMMAND_BUFFER_SIZE - 1U))
      {
        ble_command_buffer[ble_command_length++] = current_char;
      }
      else
      {
        ble_command_length = 0U;
        Ble_AppQueueString("ERROR: command too long\r\n");
      }
    }
  }
}

static uint8_t Ble_TryParseCommandValue(const char *line, const char *prefix,
                                        uint32_t max_value, uint32_t *value_out)
{
  char value_buffer[24];
  const char *value_text;
  size_t prefix_length;
  size_t value_length;

  if ((line == NULL) || (prefix == NULL))
  {
    return 0U;
  }

  prefix_length = strlen(prefix);
  if (strncmp(line, prefix, prefix_length) != 0)
  {
    return 0U;
  }

  value_text = line + prefix_length;
  while (*value_text == ' ')
  {
    value_text++;
  }

  if (*value_text == '"')
  {
    value_text++;
    value_length = strlen(value_text);

    if ((value_length < 1U) || (value_text[value_length - 1U] != '"'))
    {
      return 0U;
    }

    value_length--;
    if (value_length >= sizeof(value_buffer))
    {
      return 0U;
    }

    memcpy(value_buffer, value_text, value_length);
    value_buffer[value_length] = '\0';
    value_text = value_buffer;
  }
  else
  {
    value_length = strlen(value_text);
    while ((value_length > 0U) && (value_text[value_length - 1U] == ' '))
    {
      value_length--;
    }

    if (value_length >= sizeof(value_buffer))
    {
      return 0U;
    }

    memcpy(value_buffer, value_text, value_length);
    value_buffer[value_length] = '\0';
    value_text = value_buffer;
  }

  return App_ParseUnsignedValue(value_text, 0U, max_value, value_out);
}

static void Ble_ReportCurrentSettings(void)
{
  Ble_AppQueueFormatted("current_settings: address=%u (0x%02X), command=%u (0x%02X)\r\n",
                        tx_address, tx_address, tx_command, tx_command);
}

static void Ble_ReportCurrentHits(void)
{
  uint32_t address;

  Ble_AppQueueFormatted("current_hits: total=%lu\r\n", (unsigned long)total_hits);
  for (address = 0U; address < PLAYER_ADDRESS_COUNT; address++)
  {
    Ble_AppQueueFormatted("address[%02lu]=%lu\r\n",
                          (unsigned long)address,
                          (unsigned long)hit_count_by_address[address]);
  }
}

static void Ble_ResetHits(void)
{
  memset(hit_count_by_address, 0, sizeof(hit_count_by_address));
  total_hits = 0U;
  Ble_AppQueueString("OK: hit counters reset\r\n");
}

static void Ble_ProcessLine(const char *line)
{
  if ((line == NULL) || (*line == '\0'))
  {
    return;
  }

  Ble_ProcessAppCommand(line);
}

static void Ble_ProcessAppCommand(const char *line)
{
  uint32_t new_value;

  if ((line == NULL) || (*line == '\0'))
  {
    return;
  }

  App_SendDebug("[BLE-CMD] %s\r\n", line);

  if (strcmp(line, "current_settings") == 0)
  {
    Ble_ReportCurrentSettings();
  }
  else if (Ble_TryParseCommandValue(line, "set_address:", PLAYER_ADDRESS_COUNT - 1U,
                                    &new_value) != 0U)
  {
    tx_address = (uint8_t)new_value;
    Ble_AppQueueFormatted("OK: address set to %u (0x%02X)\r\n", tx_address, tx_address);
  }
  else if (strncmp(line, "set_address:", 12) == 0)
  {
    Ble_AppQueueString("ERROR: address must be 0..31\r\n");
  }
  else if (Ble_TryParseCommandValue(line, "set_command:", 63U, &new_value) != 0U)
  {
    tx_command = (uint8_t)new_value;
    Ble_AppQueueFormatted("OK: command set to %u (0x%02X)\r\n", tx_command, tx_command);
  }
  else if (strncmp(line, "set_command:", 12) == 0)
  {
    Ble_AppQueueString("ERROR: command must be 0..63\r\n");
  }
  else if (strcmp(line, "current_hits") == 0)
  {
    Ble_ReportCurrentHits();
  }
  else if (strcmp(line, "reset_hits") == 0)
  {
    Ble_ResetHits();
  }
  else
  {
    Ble_AppQueueString("ERROR: unknown command\r\n");
  }
}

static void Ble_AppQueueBytes(const uint8_t *data, uint16_t length)
{
  Ble_QueueBytes(data, length);
}

static void Ble_AppQueueString(const char *text)
{
  if (text != NULL)
  {
    Ble_AppQueueBytes((const uint8_t *)text, (uint16_t)strlen(text));
  }
}

static void Ble_AppQueueFormatted(const char *format, ...)
{
  char buffer[BLE_TX_MAX_MESSAGE_SIZE];
  va_list args;
  int length;

  va_start(args, format);
  length = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (length <= 0)
  {
    return;
  }

  if (length >= (int)sizeof(buffer))
  {
    length = (int)sizeof(buffer) - 1;
  }

  Ble_QueueBytes((const uint8_t *)buffer, (uint16_t)length);
}

static void Ble_QueueBytes(const uint8_t *data, uint16_t length)
{
  uint8_t slot;

  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  if (length >= BLE_TX_MAX_MESSAGE_SIZE)
  {
    length = BLE_TX_MAX_MESSAGE_SIZE - 1U;
  }

  __disable_irq();
  if (ble_tx_queue_count >= BLE_TX_QUEUE_DEPTH)
  {
    __enable_irq();
    return;
  }

  slot = ble_tx_queue_tail;
  memcpy(ble_tx_queue[slot], data, length);
  ble_tx_queue[slot][length] = '\0';
  ble_tx_lengths[slot] = length;
  ble_tx_queue_tail = (uint8_t)((ble_tx_queue_tail + 1U) % BLE_TX_QUEUE_DEPTH);
  ble_tx_queue_count++;
  __enable_irq();

  Ble_KickTx();
}

static void Ble_KickTx(void)
{
  uint8_t slot;
  uint8_t should_start = 0U;

  __disable_irq();
  if ((ble_tx_busy == 0U) && (ble_tx_queue_count > 0U))
  {
    slot = ble_tx_queue_head;
    ble_tx_busy = 1U;
    should_start = 1U;
  }
  else
  {
    slot = 0U;
  }
  __enable_irq();

  if (should_start != 0U)
  {
    if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)ble_tx_queue[slot],
                              ble_tx_lengths[slot]) != HAL_OK)
    {
      __disable_irq();
      ble_tx_busy = 0U;
      __enable_irq();
    }
  }
}

static void Ble_SendStartupBanner(void)
{
  Ble_AppQueueString("Hello BLE\r\n");
  Ble_AppQueueString("[BOOT] BLE command interface ready\r\n");
  Ble_AppQueueString("Commands: current_settings, set_address:\"x\", set_command:\"x\", current_hits, reset_hits\r\n");
}

static void Ble_ServiceStartupBanner(void)
{
  if (ble_startup_banner_sent != 0U)
  {
    return;
  }

  if (HAL_GetTick() < BLE_STARTUP_BANNER_DELAY_MS)
  {
    return;
  }

  Ble_SendStartupBanner();
  ble_startup_banner_sent = 1U;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_3)  /* PA3 — BTN_TX */
  {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) != GPIO_PIN_SET) return;

    uint32_t now = HAL_GetTick();
    if ((now - last_button_tick) < DEBOUNCE_MS) return;
    last_button_tick = now;

    if (button_pressed) return;
    button_pressed = 1U;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __disable_irq();
    if (ble_tx_queue_count > 0U)
    {
      ble_tx_queue_head = (uint8_t)((ble_tx_queue_head + 1U) % BLE_TX_QUEUE_DEPTH);
      ble_tx_queue_count--;
    }
    ble_tx_busy = 0U;
    __enable_irq();

    Ble_KickTx();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    App_SendDebug("[BLE] USART1 error, restarting DMA reception\r\n");
    HAL_UART_DMAStop(&huart1);
    Ble_StartReception();
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  IR_Transceiver_Init();
  Ble_StartReception();

  App_SendDebug("\r\n[BOOT] IR TX/RX ready on USART2 (115200-8N1)\r\n");
  App_SendDebug("[BOOT] USART1 linked to LilyGO BLE bridge with DMA RX circular + DMA TX\r\n");
  App_SendDebug("[BOOT] Transparent BLE-UART mode active on USART1\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    App_ProcessStatusLed();
    Ble_ProcessRxDma();
    Ble_ServiceStartupBanner();
    IR_Transceiver_Process();

    /* --- TX trigger from the local button --- */
    if (button_pressed != 0U)
    {
      uint8_t next_toggle = (uint8_t)(toggle_bit ^ 1U);

      button_pressed = 0U;

      if ((IR_GetState() == IR_STATE_IDLE) &&
          (IR_StartTransmit(next_toggle, tx_address, tx_command) == 0U))
      {
        toggle_bit = next_toggle;
        App_RegisterHit(tx_address);

        App_SendDebug("[TX] Addr:0x%02X Cmd:0x%02X Tog:%u\r\n",
                      tx_address, tx_command, toggle_bit);
        Ble_AppQueueFormatted("[TX] Addr:0x%02X Cmd:0x%02X Tog:%u\r\n",
                              tx_address, tx_command, toggle_bit);
        App_SendDebug("[SELF-TEST] address 0x%02X hits:%lu\r\n",
                      tx_address, (unsigned long)hit_count_by_address[tx_address]);
        Ble_AppQueueFormatted("[SELF-TEST] address 0x%02X hits:%lu\r\n",
                              tx_address, (unsigned long)hit_count_by_address[tx_address]);
      }
      else
      {
        App_SendDebug("[TX] Ignored because IR is busy or repeat guard is active\r\n");
      }
    }

    /* --- RX frame received --- */
    if (RC5FrameReceived && IR_GetState() == IR_STATE_IDLE)
    {
      RC5_Decode(&RC5_FRAME);
      App_RegisterHit(RC5_FRAME.Address);

      App_SendDebug("[RX] Addr:0x%02X Cmd:0x%02X Tog:%u Hits:%lu\r\n",
                    RC5_FRAME.Address, RC5_FRAME.Command, RC5_FRAME.ToggleBit,
                    (unsigned long)hit_count_by_address[RC5_FRAME.Address]);
      Ble_AppQueueFormatted("[RX] Addr:0x%02X Cmd:0x%02X Tog:%u Hits:%lu\r\n",
                            RC5_FRAME.Address, RC5_FRAME.Command, RC5_FRAME.ToggleBit,
                            (unsigned long)hit_count_by_address[RC5_FRAME.Address]);

      App_BlinkStatusLed(100U);
    }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 31;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3700;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* Re-apply slave reset mode AFTER IC channel config, because
     HAL_TIM_IC_ConfigChannel on CH1 can clobber SMCR. */
  {
    TIM_SlaveConfigTypeDef sSlaveRe = {0};
    sSlaveRe.SlaveMode        = TIM_SLAVEMODE_RESET;
    sSlaveRe.InputTrigger     = TIM_TS_TI1FP1;
    sSlaveRe.TriggerPolarity  = TIM_TRIGGERPOLARITY_FALLING;
    sSlaveRe.TriggerPrescaler = TIM_ICPSC_DIV1;
    sSlaveRe.TriggerFilter    = 0;
    HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveRe);
  }

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 31;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 888;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 842;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 210;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
