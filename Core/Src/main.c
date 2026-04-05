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
typedef enum
{
  BLE_UART_MODE_AT = 0U,
  BLE_UART_MODE_DATA
} BleUartMode;

typedef enum
{
  BLE_STATE_BOOT_GRACE = 0U,
  BLE_STATE_SYNC_AT,
  BLE_STATE_ECHO_OFF,
  BLE_STATE_WIFI_OFF,
  BLE_STATE_BLE_DEINIT,
  BLE_STATE_BLE_INIT_SERVER,
  BLE_STATE_SET_NAME,
  BLE_STATE_QUERY_ADDRESS,
  BLE_STATE_CREATE_SERVICE,
  BLE_STATE_START_SERVICE,
  BLE_STATE_SET_ADV_DATA,
  BLE_STATE_START_ADV,
  BLE_STATE_WAIT_CONNECTION,
  BLE_STATE_WAIT_SPP_SUBSCRIPTION,
  BLE_STATE_CONFIGURE_SPP,
  BLE_STATE_ENABLE_SPP,
  BLE_STATE_DATA_MODE
} BleLinkState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_MS              300U
#define PLAYER_ADDRESS_COUNT     32U
#define BLE_RX_DMA_BUFFER_SIZE   128U
#define BLE_COMMAND_BUFFER_SIZE  96U
#define BLE_TX_QUEUE_DEPTH       32U
#define BLE_TX_MAX_MESSAGE_SIZE  160U
#define BLE_BOOT_GRACE_MS        1500U
#define BLE_AT_COMMAND_TIMEOUT_MS 3000U
#define BLE_AT_SYNC_RETRY_MS     1000U
#define BLE_SPP_RETRY_LIMIT      3U
#define BLE_MODULE_NAME          "IR-Game"
#define BLE_ADV_DATA_HEX         "020106080949522D47616D65030302A0"
#define BLE_SPP_SERVICE_INDEX    1U
#define BLE_SPP_RX_CHAR_INDEX    5U
#define BLE_SPP_TX_CHAR_INDEX    7U
#define BLE_SPP_TX_DESC_INDEX    1U

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
static BleUartMode ble_uart_mode = BLE_UART_MODE_AT;
static BleLinkState ble_link_state = BLE_STATE_BOOT_GRACE;
static uint32_t ble_state_deadline_tick = 0U;
static uint32_t ble_at_deadline_tick = 0U;
static uint8_t ble_at_command_pending = 0U;
static uint8_t ble_at_accept_error = 0U;
static uint8_t ble_at_expect_prompt = 0U;
static uint8_t ble_client_connected = 0U;
static uint8_t ble_spp_retry_count = 0U;
static char ble_last_at_command[48];
static char ble_public_address[24];
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
static uint8_t Ble_IsModuleStatusLine(const char *line);
static void Ble_HandleModuleLine(const char *line);
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
static void Ble_QueueFormatted(const char *format, ...);
static void Ble_KickTx(void);
static void Ble_SendAtCommand(const char *command, uint8_t accept_error,
                              uint8_t expect_prompt);
static void Ble_ClearPendingAtCommand(void);
static void Ble_EnterState(BleLinkState next_state);
static void Ble_HandleAtCommandSuccess(void);
static void Ble_HandleAtCommandError(void);
static void Ble_ResetLink(const char *reason);
static void Ble_ServiceLink(void);

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

static uint8_t Ble_IsModuleStatusLine(const char *line)
{
  if ((line == NULL) || (*line == '\0'))
  {
    return 1U;
  }

  if ((strcmp(line, "OK") == 0) ||
      (strcmp(line, "AT") == 0) ||
      (strcmp(line, ">") == 0) ||
      (strncmp(line, "ERROR", 5) == 0) ||
      (strncmp(line, "ERR CODE", 8) == 0) ||
      (strncmp(line, "ready", 5) == 0) ||
      (strncmp(line, "AT+", 3) == 0) ||
      (strncmp(line, "+BLE", 4) == 0) ||
      (strncmp(line, "+WRITE", 6) == 0) ||
      (strncmp(line, "+READ", 5) == 0) ||
      (strncmp(line, "Brownout detector was triggered", 31) == 0) ||
      (strncmp(line, "ESP-ROM:", 8) == 0) ||
      (strncmp(line, "Build:", 6) == 0) ||
      (strncmp(line, "rst:", 4) == 0) ||
      (strncmp(line, "Saved PC:", 9) == 0) ||
      (strncmp(line, "SPIWP:", 6) == 0) ||
      (strncmp(line, "mode:", 5) == 0) ||
      (strncmp(line, "load:", 5) == 0) ||
      (strncmp(line, "entry", 5) == 0) ||
      (strncmp(line, "boot:", 5) == 0) ||
      (strncmp(line, "no external 32k oscillator", 26) == 0) ||
      (strncmp(line, "at param mode:", 14) == 0) ||
      (strncmp(line, "AT cmd port:", 12) == 0) ||
      (strncmp(line, "module_name:", 12) == 0) ||
      ((line[0] == '[') &&
       ((strstr(line, "boot:") != NULL) ||
        (strstr(line, "esp32c3") != NULL) ||
        (strstr(line, "esp_image") != NULL) ||
        (strstr(line, "partition") != NULL))) ||
      (line[0] == '+'))
  {
    return 1U;
  }

  return 0U;
}

static void Ble_ClearPendingAtCommand(void)
{
  ble_at_command_pending = 0U;
  ble_at_accept_error = 0U;
  ble_at_expect_prompt = 0U;
  ble_at_deadline_tick = 0U;
}

static void Ble_SendAtCommand(const char *command, uint8_t accept_error,
                              uint8_t expect_prompt)
{
  if ((command == NULL) || (*command == '\0'))
  {
    return;
  }

  (void)snprintf(ble_last_at_command, sizeof(ble_last_at_command), "%s", command);
  ble_at_command_pending = 1U;
  ble_at_accept_error = accept_error;
  ble_at_expect_prompt = expect_prompt;
  ble_at_deadline_tick = HAL_GetTick() + BLE_AT_COMMAND_TIMEOUT_MS;

  App_SendDebug("[BLE-AT] >> %s\r\n", command);
  Ble_QueueFormatted("%s\r\n", command);
}

static void Ble_EnterState(BleLinkState next_state)
{
  ble_link_state = next_state;

  switch (next_state)
  {
    case BLE_STATE_BOOT_GRACE:
      ble_uart_mode = BLE_UART_MODE_AT;
      ble_client_connected = 0U;
      ble_spp_retry_count = 0U;
      ble_public_address[0] = '\0';
      Ble_ClearPendingAtCommand();
      ble_state_deadline_tick = HAL_GetTick() + BLE_BOOT_GRACE_MS;
      App_SendDebug("[BLE-AT] waiting for module boot\r\n");
      break;

    case BLE_STATE_SYNC_AT:
      ble_uart_mode = BLE_UART_MODE_AT;
      ble_client_connected = 0U;
      ble_state_deadline_tick = HAL_GetTick();
      App_SendDebug("[BLE-AT] syncing with ESP-AT\r\n");
      break;

    case BLE_STATE_ECHO_OFF:
      Ble_SendAtCommand("ATE0", 0U, 0U);
      break;

    case BLE_STATE_WIFI_OFF:
      Ble_SendAtCommand("AT+CWMODE=0", 0U, 0U);
      break;

    case BLE_STATE_BLE_DEINIT:
      Ble_SendAtCommand("AT+BLEINIT=0", 1U, 0U);
      break;

    case BLE_STATE_BLE_INIT_SERVER:
      Ble_SendAtCommand("AT+BLEINIT=2", 0U, 0U);
      break;

    case BLE_STATE_SET_NAME:
      Ble_SendAtCommand("AT+BLENAME=\"" BLE_MODULE_NAME "\"", 0U, 0U);
      break;

    case BLE_STATE_QUERY_ADDRESS:
      Ble_SendAtCommand("AT+BLEADDR?", 0U, 0U);
      break;

    case BLE_STATE_CREATE_SERVICE:
      Ble_SendAtCommand("AT+BLEGATTSSRVCRE", 0U, 0U);
      break;

    case BLE_STATE_START_SERVICE:
      Ble_SendAtCommand("AT+BLEGATTSSRVSTART", 0U, 0U);
      break;

    case BLE_STATE_SET_ADV_DATA:
      Ble_SendAtCommand("AT+BLEADVDATA=\"" BLE_ADV_DATA_HEX "\"", 0U, 0U);
      break;

    case BLE_STATE_START_ADV:
      Ble_SendAtCommand("AT+BLEADVSTART", 1U, 0U);
      break;

    case BLE_STATE_WAIT_CONNECTION:
      ble_uart_mode = BLE_UART_MODE_AT;
      ble_state_deadline_tick = 0U;
      App_SendDebug("[BLE-AT] advertising active, waiting for BLE client\r\n");
      break;

    case BLE_STATE_WAIT_SPP_SUBSCRIPTION:
      ble_uart_mode = BLE_UART_MODE_AT;
      ble_state_deadline_tick = 0U;
      App_SendDebug("[BLE-AT] client connected; enable indications on service %u char %u (UUID 0xC306) in nRF Connect\r\n",
                    BLE_SPP_SERVICE_INDEX, BLE_SPP_TX_CHAR_INDEX);
      break;

    case BLE_STATE_CONFIGURE_SPP:
      Ble_SendAtCommand("AT+BLESPPCFG=1,1,7,1,5", 0U, 0U);
      break;

    case BLE_STATE_ENABLE_SPP:
      Ble_SendAtCommand("AT+BLESPP", 0U, 1U);
      break;

    case BLE_STATE_DATA_MODE:
      ble_uart_mode = BLE_UART_MODE_DATA;
      ble_spp_retry_count = 0U;
      App_SendDebug("[BLE-AT] BLE passthrough active\r\n");
      Ble_AppQueueString("\r\n[BOOT] BLE command interface ready\r\n");
      Ble_AppQueueString("Commands: current_settings, set_address:\"x\", set_command:\"x\", current_hits, reset_hits\r\n");
      break;

    default:
      break;
  }
}

static void Ble_HandleAtCommandSuccess(void)
{
  switch (ble_link_state)
  {
    case BLE_STATE_SYNC_AT:
      Ble_EnterState(BLE_STATE_ECHO_OFF);
      break;

    case BLE_STATE_ECHO_OFF:
      Ble_EnterState(BLE_STATE_WIFI_OFF);
      break;

    case BLE_STATE_WIFI_OFF:
      Ble_EnterState(BLE_STATE_BLE_DEINIT);
      break;

    case BLE_STATE_BLE_DEINIT:
      Ble_EnterState(BLE_STATE_BLE_INIT_SERVER);
      break;

    case BLE_STATE_BLE_INIT_SERVER:
      Ble_EnterState(BLE_STATE_SET_NAME);
      break;

    case BLE_STATE_SET_NAME:
      Ble_EnterState(BLE_STATE_QUERY_ADDRESS);
      break;

    case BLE_STATE_QUERY_ADDRESS:
      Ble_EnterState(BLE_STATE_CREATE_SERVICE);
      break;

    case BLE_STATE_CREATE_SERVICE:
      Ble_EnterState(BLE_STATE_START_SERVICE);
      break;

    case BLE_STATE_START_SERVICE:
      Ble_EnterState(BLE_STATE_SET_ADV_DATA);
      break;

    case BLE_STATE_SET_ADV_DATA:
      Ble_EnterState(BLE_STATE_START_ADV);
      break;

    case BLE_STATE_START_ADV:
      Ble_EnterState(BLE_STATE_WAIT_CONNECTION);
      break;

    case BLE_STATE_WAIT_SPP_SUBSCRIPTION:
      Ble_EnterState(BLE_STATE_CONFIGURE_SPP);
      break;

    case BLE_STATE_CONFIGURE_SPP:
      Ble_EnterState(BLE_STATE_ENABLE_SPP);
      break;

    case BLE_STATE_ENABLE_SPP:
      Ble_EnterState(BLE_STATE_DATA_MODE);
      break;

    default:
      break;
  }
}

static void Ble_ResetLink(const char *reason)
{
  if ((reason != NULL) && (*reason != '\0'))
  {
    App_SendDebug("[BLE-AT] %s, restarting init\r\n", reason);
  }

  Ble_EnterState(BLE_STATE_BOOT_GRACE);
}

static void Ble_HandleAtCommandError(void)
{
  uint8_t accept_error = ble_at_accept_error;

  Ble_ClearPendingAtCommand();

  if (accept_error != 0U)
  {
    App_SendDebug("[BLE-AT] tolerated ERROR after %s\r\n", ble_last_at_command);
    Ble_HandleAtCommandSuccess();
    return;
  }

  switch (ble_link_state)
  {
    case BLE_STATE_START_ADV:
      App_SendDebug("[BLE-AT] advertising was already active\r\n");
      Ble_EnterState(BLE_STATE_WAIT_CONNECTION);
      break;

    case BLE_STATE_CONFIGURE_SPP:
    case BLE_STATE_ENABLE_SPP:
      ble_spp_retry_count++;

      if (ble_spp_retry_count < BLE_SPP_RETRY_LIMIT)
      {
        App_SendDebug("[BLE-AT] SPP setup failed, retry %u/%u\r\n",
                      ble_spp_retry_count, BLE_SPP_RETRY_LIMIT);
        Ble_EnterState(BLE_STATE_CONFIGURE_SPP);
      }
      else
      {
        App_SendDebug("[BLE-AT] SPP setup failed; waiting for a reconnect\r\n");
        ble_client_connected = 0U;
        ble_spp_retry_count = 0U;
        Ble_EnterState(BLE_STATE_START_ADV);
      }
      break;

    default:
      Ble_ResetLink("AT command returned ERROR");
      break;
  }
}

static void Ble_HandleModuleLine(const char *line)
{
  if ((line == NULL) || (*line == '\0'))
  {
    return;
  }

  App_SendDebug("[BLE-MODULE] %s\r\n", line);

  if ((strncmp(line, "Brownout detector was triggered", 31) == 0) ||
      (strncmp(line, "ESP-ROM:", 8) == 0) ||
      (strncmp(line, "rst:", 4) == 0))
  {
    if (ble_link_state != BLE_STATE_BOOT_GRACE)
    {
      Ble_ResetLink("module reboot detected");
    }
    return;
  }

  if (strncmp(line, "+BLECONN:", 9) == 0)
  {
    ble_client_connected = 1U;
    ble_spp_retry_count = 0U;

    if (ble_uart_mode == BLE_UART_MODE_AT)
    {
      Ble_EnterState(BLE_STATE_WAIT_SPP_SUBSCRIPTION);
    }
    return;
  }

  if (strncmp(line, "+BLEADDR:", 9) == 0)
  {
    const char *address_text = line + 9;

    while (*address_text == ' ')
    {
      address_text++;
    }

    (void)snprintf(ble_public_address, sizeof(ble_public_address), "%s", address_text);
    App_SendDebug("[BLE-AT] scan for BLE address %s\r\n", ble_public_address);
    return;
  }

  if (strncmp(line, "+BLESECREQ:", 10) == 0)
  {
    App_SendDebug("[BLE-AT] peer requested pairing; reconnect after reflashing this no-security config\r\n");
    return;
  }

  if (strncmp(line, "+BLEAUTHCMPL:", 12) == 0)
  {
    if (strcmp(line, "+BLEAUTHCMPL:0,1") == 0)
    {
      App_SendDebug("[BLE-AT] BLE pairing/authentication failed\r\n");
    }
    else if (strcmp(line, "+BLEAUTHCMPL:0,0") == 0)
    {
      App_SendDebug("[BLE-AT] BLE pairing/authentication succeeded\r\n");
    }
    return;
  }

  if (strncmp(line, "+BLEDISCONN:", 11) == 0)
  {
    ble_uart_mode = BLE_UART_MODE_AT;
    ble_client_connected = 0U;
    ble_spp_retry_count = 0U;
    Ble_EnterState(BLE_STATE_START_ADV);
    return;
  }

  if (strncmp(line, "+WRITE:", 7) == 0)
  {
    unsigned long conn_index = 0UL;
    unsigned long service_index = 0UL;
    unsigned long char_index = 0UL;
    unsigned long desc_index = 0UL;
    unsigned long value_len = 0UL;
    char value_text[12] = {0};
    int parsed_fields;

    parsed_fields = sscanf(line,
                           "+WRITE:%lu,%lu,%lu,%lu,%lu,%11s",
                           &conn_index, &service_index, &char_index,
                           &desc_index, &value_len, value_text);

    if ((parsed_fields == 6) &&
        (service_index == BLE_SPP_SERVICE_INDEX) &&
        (char_index == BLE_SPP_TX_CHAR_INDEX) &&
        (desc_index == BLE_SPP_TX_DESC_INDEX) &&
        (strcmp(value_text, "0002") == 0))
    {
      App_SendDebug("[BLE-AT] client enabled indications on 0xC306\r\n");

      if (ble_link_state == BLE_STATE_WAIT_SPP_SUBSCRIPTION)
      {
        Ble_EnterState(BLE_STATE_CONFIGURE_SPP);
      }
    }

    return;
  }

  if (strcmp(line, "OK") == 0)
  {
    if (ble_at_command_pending != 0U)
    {
      if (ble_at_expect_prompt == 0U)
      {
        Ble_ClearPendingAtCommand();
        Ble_HandleAtCommandSuccess();
      }
      else
      {
        ble_at_deadline_tick = HAL_GetTick() + BLE_AT_COMMAND_TIMEOUT_MS;
      }
    }
    return;
  }

  if (strcmp(line, ">") == 0)
  {
    if ((ble_at_command_pending != 0U) && (ble_at_expect_prompt != 0U))
    {
      Ble_ClearPendingAtCommand();
      Ble_HandleAtCommandSuccess();
    }
    return;
  }

  if ((strncmp(line, "ERROR", 5) == 0) || (strncmp(line, "ERR CODE", 8) == 0))
  {
    if (ble_at_command_pending != 0U)
    {
      Ble_HandleAtCommandError();
    }
    return;
  }

  if ((strncmp(line, "ready", 5) == 0) &&
      ((ble_link_state == BLE_STATE_BOOT_GRACE) || (ble_link_state == BLE_STATE_SYNC_AT)))
  {
    Ble_EnterState(BLE_STATE_SYNC_AT);
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

  return App_ParseUnsignedValue(value_text, 0U, max_value, value_out);
}

static void Ble_ReportCurrentSettings(void)
{
  Ble_AppQueueFormatted("current_settings: address=%u (0x%02X), command=%u (0x%02X)\r\n",
                        tx_address, tx_address, tx_command, tx_command);
}

static void Ble_ReportCurrentHits(void)
{
  char message[BLE_TX_MAX_MESSAGE_SIZE];
  int offset;
  uint8_t has_hits = 0U;
  uint32_t address;

  offset = snprintf(message, sizeof(message), "current_hits: total=%lu\r\n",
                    (unsigned long)total_hits);
  if (offset < 0)
  {
    return;
  }

  if (offset >= (int)sizeof(message))
  {
    offset = (int)sizeof(message) - 1;
  }

  for (address = 0U; address < PLAYER_ADDRESS_COUNT; address++)
  {
    int written;

    if (hit_count_by_address[address] == 0U)
    {
      continue;
    }

    has_hits = 1U;
    written = snprintf(&message[offset], sizeof(message) - (size_t)offset,
                       "address[%02lu]=%lu\r\n",
                       (unsigned long)address,
                       (unsigned long)hit_count_by_address[address]);

    if ((written < 0) || (written >= (int)(sizeof(message) - (size_t)offset)))
    {
      Ble_AppQueueBytes((uint8_t *)message, (uint16_t)offset);
      offset = snprintf(message, sizeof(message), "address[%02lu]=%lu\r\n",
                        (unsigned long)address,
                        (unsigned long)hit_count_by_address[address]);

      if (offset < 0)
      {
        return;
      }

      if (offset >= (int)sizeof(message))
      {
        offset = (int)sizeof(message) - 1;
      }
    }
    else
    {
      offset += written;
    }
  }

  if (has_hits == 0U)
  {
    const char *no_hits_line = "no hits recorded\r\n";
    size_t no_hits_length = strlen(no_hits_line);

    if ((size_t)offset + no_hits_length >= sizeof(message))
    {
      Ble_AppQueueBytes((uint8_t *)message, (uint16_t)offset);
      offset = 0;
    }

    memcpy(&message[offset], no_hits_line, no_hits_length);
    offset += (int)no_hits_length;
  }

  Ble_AppQueueBytes((uint8_t *)message, (uint16_t)offset);
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

  if (Ble_IsModuleStatusLine(line) != 0U)
  {
    Ble_HandleModuleLine(line);
    return;
  }

  if (ble_uart_mode != BLE_UART_MODE_DATA)
  {
    App_SendDebug("[BLE-MODULE] %s\r\n", line);
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
  else if (Ble_TryParseCommandValue(line, "set_command:", 127U, &new_value) != 0U)
  {
    tx_command = (uint8_t)new_value;
    Ble_AppQueueFormatted("OK: command set to %u (0x%02X)\r\n", tx_command, tx_command);
  }
  else if (strncmp(line, "set_command:", 12) == 0)
  {
    Ble_AppQueueString("ERROR: command must be 0..127\r\n");
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
  if ((ble_uart_mode != BLE_UART_MODE_DATA) || (ble_client_connected == 0U))
  {
    return;
  }

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

  if ((ble_uart_mode != BLE_UART_MODE_DATA) || (ble_client_connected == 0U))
  {
    return;
  }

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

static void Ble_QueueFormatted(const char *format, ...)
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

static void Ble_ServiceLink(void)
{
  uint32_t now = HAL_GetTick();

  if (ble_at_command_pending != 0U)
  {
    if ((int32_t)(now - ble_at_deadline_tick) >= 0)
    {
      Ble_ClearPendingAtCommand();

      if (ble_link_state == BLE_STATE_SYNC_AT)
      {
        App_SendDebug("[BLE-AT] no AT reply yet, retrying sync\r\n");
        ble_state_deadline_tick = now + BLE_AT_SYNC_RETRY_MS;
      }
      else
      {
        Ble_ResetLink("AT response timeout");
      }
    }
    return;
  }

  switch (ble_link_state)
  {
    case BLE_STATE_BOOT_GRACE:
      if ((int32_t)(now - ble_state_deadline_tick) >= 0)
      {
        Ble_EnterState(BLE_STATE_SYNC_AT);
      }
      break;

    case BLE_STATE_SYNC_AT:
      if ((int32_t)(now - ble_state_deadline_tick) >= 0)
      {
        Ble_SendAtCommand("AT", 0U, 0U);
      }
      break;

    case BLE_STATE_WAIT_CONNECTION:
    case BLE_STATE_WAIT_SPP_SUBSCRIPTION:
    default:
      break;
  }
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
    HAL_UART_DMAStop(&huart1);
    Ble_StartReception();
    Ble_ResetLink("USART1 error");
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
  Ble_EnterState(BLE_STATE_BOOT_GRACE);

  App_SendDebug("\r\n[BOOT] IR TX/RX ready on USART2 (115200-8N1)\r\n");
  App_SendDebug("[BOOT] BLE UART ready on USART1 with DMA (115200-8N1)\r\n");
  App_SendDebug("[BOOT] ESP-AT BLE init sequence started on USART1\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    App_ProcessStatusLed();
    Ble_ProcessRxDma();
    Ble_ServiceLink();
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
        App_SendDebug("[TX] Addr:0x%02X Cmd:0x%02X Tog:%u\r\n",
                      tx_address, tx_command, toggle_bit);
        Ble_AppQueueFormatted("[TX] Addr:0x%02X Cmd:0x%02X Tog:%u\r\n",
                              tx_address, tx_command, toggle_bit);
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
