/**
  ******************************************************************************
  * @file    ir_transceiver.c
  * @brief   Half-duplex IR transceiver state machine for STM32L432KC.
  *          Manages TIM2 (RX), TIM15 (TX envelope), TIM16 (TX carrier).
  ******************************************************************************
  */
#include "ir_transceiver.h"
#include "rc5_decode.h"
#include "rc5_encode.h"
#include "main.h"

/* --- Extern timer handles (from main.c) --------------------------------- */

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;

/* --- Private variables --------------------------------------------------- */

static volatile IR_State_t ir_state = IR_STATE_IDLE;
static uint32_t guard_start_tick = 0;
static uint32_t last_tx_start_tick = 0;

#define GUARD_TIME_MS        5U

/*
 * RC5 standard: minimum repeat period = 114 ms.
 * Frame itself takes ~25 ms, guard 5 ms, so enforce at least
 * 114 ms from start-to-start of consecutive transmissions.
 */
#define IR_TX_MIN_REPEAT_MS  114U

/* --- Capture values for decode callback ---------------------------------- */

static uint32_t ICValue1 = 0;
static uint32_t ICValue2 = 0;

/* --- Public functions ---------------------------------------------------- */

void IR_Transceiver_Init(void)
{
  /* Start 38 kHz carrier timer (CCR = 0 → carrier off) */
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);

  /* CRITICAL: Only counter overflow (not slave-reset) may trigger the Update
     interrupt.  Without this, every falling edge resets the counter AND fires
     PeriodElapsedCallback → RC5_ResetPacket(), killing every frame. */
  __HAL_TIM_URS_ENABLE(&htim2);
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

  /* Start RX input capture + timeout */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  /* Enable update (overflow) interrupt for frame timeout */
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

  RC5_ResetPacket();
  ir_state = IR_STATE_IDLE;
}

IR_State_t IR_GetState(void)
{
  return ir_state;
}

uint8_t IR_StartTransmit(uint8_t toggle, uint8_t address, uint8_t command)
{
  if (ir_state != IR_STATE_IDLE) return 1;

  /* RC5 standard: enforce min 114 ms between frame starts */
  if ((HAL_GetTick() - last_tx_start_tick) < IR_TX_MIN_REPEAT_MS) return 1;

  ir_state = IR_STATE_TX_SENDING;
  last_tx_start_tick = HAL_GetTick();

  /* Disable RX captures during TX */
  HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);

  RC5_Encode_SendFrame(toggle, address, command);
  return 0;
}

void IR_TX_Complete(void)
{
  ir_state = IR_STATE_TX_GUARD;
  guard_start_tick = HAL_GetTick();
}

void IR_Transceiver_Process(void)
{
  if (ir_state == IR_STATE_TX_GUARD)
  {
    if ((HAL_GetTick() - guard_start_tick) >= GUARD_TIME_MS)
    {
      /* Guard time elapsed — re-enable RX */
      RC5_ResetPacket();
      HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
      HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
      ir_state = IR_STATE_IDLE;
    }
  }
}

/* --- HAL Callbacks (called from stm32l4xx_it.c via HAL) ------------------ */

/**
  * @brief  Period-elapsed callback — routes TIM15 (TX) and TIM2 (RX timeout).
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM15)
  {
    RC5_Encode_SignalGenerate();

    /* Check if encoder just finished */
    if (!RC5_Encode_IsBusy())
    {
      IR_TX_Complete();
    }
  }
  else if (htim->Instance == TIM2)
  {
    /* Timeout — no edge within ARR (3.7 ms) → reset decoder */
    if (ir_state == IR_STATE_IDLE)
    {
      RC5_ResetPacket();
    }
  }
}

/**
  * @brief  Input-capture callback — TIM2 channels (RX edge detection).
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Ignore captures while transmitting */
  if (ir_state != IR_STATE_IDLE) return;

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    /* Falling edge (direct channel) — HIGH phase measurement.
       CCR1 = LOW + HIGH since last falling edge.
       Subtract CCR2 (LOW phase) to isolate the HIGH duration. */
    ICValue1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
    RC5_DataSampling(ICValue1 - ICValue2, 0);   /* 0 = falling edge, HIGH phase */
  }
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    /* Rising edge (indirect channel) — pulse width since last falling edge. */
    ICValue2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
    RC5_DataSampling(ICValue2, 1);   /* 1 = rising edge */
  }
}
