/**
  ******************************************************************************
  * @file    rc5_encode.c
  * @brief   RC5 infrared encoder for STM32L432KC.
  *
  *  Approach (from architecture document):
  *    TIM15 — envelope timer, Update ISR every 889 µs (half-bit).
  *    TIM16 — 38 kHz carrier on PA6.  Carrier ON = CCR1=210, OFF = CCR1=0.
  *
  *  Transmission flow:
  *    1. RC5_Encode_SendFrame() builds a 28-element half-bit array and
  *       starts TIM15 with interrupt.
  *    2. Each TIM15 Update ISR calls RC5_Encode_SignalGenerate() which
  *       advances to the next half-bit and sets TIM16 CCR accordingly.
  *    3. After the last half-bit, carrier is turned off and TIM15 stopped.
  ******************************************************************************
  */
#include "rc5_encode.h"
#include "main.h"

/* --- Extern timer handles (from main.c) --------------------------------- */

extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;

/* --- Private constants --------------------------------------------------- */

#define RC5_FRAME_BITS       14     /* S1 + S2 + T + 5 addr + 6 cmd */
#define RC5_HALFBIT_COUNT    (RC5_FRAME_BITS * 2)   /* 28 half-bits */
#define CARRIER_DUTY         210U   /* 25 % of ARR+1 = 843 */

/*
 * RC5 standard timing (Philips specification):
 *   T (half-bit)      = 889 µs   (32/36 kHz carrier cycles)
 *   Bit period         = 2T = 1778 µs
 *   Frame duration     = 14 × 2T = 24.889 ms
 *   Min repeat period  = 113.778 ms (= 128T ≈ 4096 carrier cycles)
 *   Min silence gap    = 113.778 - 24.889 = 88.889 ms
 *
 * We enforce the gap in ir_transceiver.c via IR_TX_MIN_REPEAT_MS.
 */

/* --- Private variables --------------------------------------------------- */

static uint8_t           rc5_halfbits[RC5_HALFBIT_COUNT];
static volatile uint8_t  rc5_pos;
static volatile uint8_t  rc5_busy;

/* --- Private helpers ----------------------------------------------------- */

/**
  * @brief  Build the 14-bit RC5 word and Manchester-encode into half-bits.
  *
  * Frame format (MSB first):
  *   [S1=1][S2/Field][Toggle][A4..A0][C5..C0]
  *
  * Manchester encoding per bit (carrier perspective):
  *   bit '1' → first half = carrier ON,  second half = carrier OFF
  *   bit '0' → first half = carrier OFF, second half = carrier ON
  */
static void RC5_BuildHalfBits(uint8_t toggle, uint8_t address, uint8_t command)
{
  uint16_t frame = 0;

  /* S1 — always 1 */
  frame |= (1U << 13);

  /* S2 / Field bit: 1 for commands 0-63, 0 for commands 64-127 */
  if (command < 64)
  {
    frame |= (1U << 12);
  }

  /* Toggle */
  frame |= (uint16_t)((toggle & 1U) << 11);

  /* Address (5 bits) */
  frame |= (uint16_t)((address & 0x1FU) << 6);

  /* Command (lowest 6 bits) */
  frame |= (uint16_t)(command & 0x3FU);

  /* Manchester encode — MSB first */
  for (int i = 0; i < RC5_FRAME_BITS; i++)
  {
    uint8_t bit = (frame >> (13 - i)) & 1U;
    if (bit)
    {
      rc5_halfbits[i * 2]     = 0;   /* carrier OFF (space first) */
      rc5_halfbits[i * 2 + 1] = 1;   /* carrier ON  (mark second) */
    }
    else
    {
      rc5_halfbits[i * 2]     = 1;   /* carrier ON  (mark first)  */
      rc5_halfbits[i * 2 + 1] = 0;   /* carrier OFF (space second)*/
    }
  }
}

/* --- Public functions ---------------------------------------------------- */

void RC5_Encode_SendFrame(uint8_t toggle, uint8_t address, uint8_t command)
{
  if (rc5_busy) return;

  RC5_BuildHalfBits(toggle, address, command);

  rc5_pos  = 0;
  rc5_busy = 1;

  /* Apply the first half-bit immediately */
  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1,
                         rc5_halfbits[0] ? CARRIER_DUTY : 0U);

  /* Reset TIM15 counter and start its Update interrupt */
  __HAL_TIM_SET_COUNTER(&htim15, 0);
  HAL_TIM_Base_Start_IT(&htim15);
}

void RC5_Encode_SignalGenerate(void)
{
  if (!rc5_busy) return;

  rc5_pos++;

  if (rc5_pos < RC5_HALFBIT_COUNT)
  {
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1,
                           rc5_halfbits[rc5_pos] ? CARRIER_DUTY : 0U);
  }
  else
  {
    /* Frame complete — carrier off, stop envelope timer */
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0U);
    HAL_TIM_Base_Stop_IT(&htim15);
    rc5_busy = 0;
  }
}

uint8_t RC5_Encode_IsBusy(void)
{
  return rc5_busy;
}
