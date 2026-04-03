/**
  ******************************************************************************
  * @file    rc5_decode.c
  * @brief   RC5 infrared decoder for STM32L432KC.
  *          Core algorithm from X-CUBE-IRREMOTE (ST AN4834), adapted:
  *          - BSP / LCD / Menu dependencies removed
  *          - Timing hardcoded for 32 MHz, PSC=31 → 1 µs/tick
  *          - Timer init removed (CubeMX handles TIM2 config)
  ******************************************************************************
  */
#include "rc5_decode.h"
#include <stdint.h>

/* --- Private constants --------------------------------------------------- */

#define RC5_1T_TIME                 0x00
#define RC5_2T_TIME                 0x01
#define RC5_WRONG_TIME              0xFF
#define RC5_PACKET_BIT_COUNT        13      /* Total bits in an RC5 frame */
#define RC5_PACKET_STATUS_EMPTY     (1U << 0)

/*
 * Timing thresholds for 1 µs/tick (32 MHz, PSC = 31).
 * Half-bit (T)  = 889 µs   → ~889 ticks
 * Full-bit (2T) = 1778 µs  → ~1778 ticks
 * Tolerance = ±300 µs
 */
#define RC5_MIN_T      600U        /* 889 - 300 ≈ 589 → rounded 600 */
#define RC5_MAX_T      1200U       /* 889 + 300 ≈ 1189 → rounded 1200 */
#define RC5_MIN_2T     1500U       /* 1778 - 300 ≈ 1478 → rounded 1500 */
#define RC5_MAX_2T     2100U       /* 1778 + 300 ≈ 2078 → rounded 2100 */

/* --- Logic tables -------------------------------------------------------- */

/* Rising edge: rows = previous bit, columns = pulse length (1T / 2T) */
static const RC5_lastBit_t RC5_logicTableRisingEdge[2][2] =
{
  { RC5_ZER, RC5_INV },   /* lastbit = ZERO */
  { RC5_NAN, RC5_ZER},   /* lastbit = ONE  (2T LOW after 1 is impossible) */
};

/* Falling edge: rows = previous bit, columns = pulse length (1T / 2T) */
static const RC5_lastBit_t RC5_logicTableFallingEdge[2][2] =
{
  { RC5_NAN, RC5_ONE },   /* lastbit = ZERO (2T HIGH after 0 is impossible) */
  { RC5_ONE, RC5_INV },   /* lastbit = ONE  */
};

/* --- Private variables --------------------------------------------------- */

static __IO RC5_Packet_t RC5TmpPacket;

/* --- Public variables ---------------------------------------------------- */

__IO uint8_t  RC5FrameReceived = 0;
RC5_Frame_t   RC5_FRAME;

/* --- Private function prototypes ----------------------------------------- */

static uint8_t  RC5_GetPulseLength(uint32_t pulseLength);
static void     RC5_modifyLastBit(RC5_lastBit_t bit);
static void     RC5_WriteBit(uint8_t bitVal);

/* --- Public functions ---------------------------------------------------- */

/**
  * @brief  Reset the incoming packet to default (empty) state.
  */
void RC5_ResetPacket(void)
{
  RC5TmpPacket.data     = 0;
  RC5TmpPacket.bitCount = RC5_PACKET_BIT_COUNT - 1;
  RC5TmpPacket.lastBit  = RC5_ONE;
  RC5TmpPacket.status   = RC5_PACKET_STATUS_EMPTY;
}

/**
  * @brief  Feed a pulse measurement into the decoder.
  * @param  rawPulseLength  Pulse duration in timer ticks (1 µs/tick).
  * @param  edge            1 = rising edge, 0 = falling edge.
  *
  * Called from HAL_TIM_IC_CaptureCallback() for TIM2 channels.
  */
void RC5_DataSampling(uint32_t rawPulseLength, uint32_t edge)
{
  uint8_t pulse;
  RC5_lastBit_t tmp_bit;

  pulse = RC5_GetPulseLength(rawPulseLength);

  if (edge == 1)  /* Rising edge */
  {
    if (pulse <= RC5_2T_TIME)
    {
      tmp_bit = RC5_logicTableRisingEdge[RC5TmpPacket.lastBit][pulse];
      RC5_modifyLastBit(tmp_bit);
    }
    else
    {
      RC5_ResetPacket();
    }
  }
  else  /* Falling edge */
  {
    if (RC5TmpPacket.status & RC5_PACKET_STATUS_EMPTY)
    {
      /* First falling edge — just clear the empty flag, don't decode yet */
      RC5TmpPacket.status &= (uint8_t)~RC5_PACKET_STATUS_EMPTY;
    }
    else
    {
      if (pulse <= RC5_2T_TIME)
      {
        tmp_bit = RC5_logicTableFallingEdge[RC5TmpPacket.lastBit][pulse];
        RC5_modifyLastBit(tmp_bit);
      }
      else
      {
        RC5_ResetPacket();
      }
    }
  }
}

/**
  * @brief  Extract address / command / toggle from the received frame.
  * @param  pFrame  Pointer to RC5_Frame_t to fill.
  *
  * Call this from the main loop when RC5FrameReceived == 1.
  */
void RC5_Decode(RC5_Frame_t *pFrame)
{
  if (RC5FrameReceived)
  {
    pFrame->FieldBit  = (RC5TmpPacket.data >> 12) & 0x1;
    pFrame->ToggleBit = (RC5TmpPacket.data >> 11) & 0x1;
    pFrame->Address   = (RC5TmpPacket.data >>  6) & 0x1F;
    pFrame->Command   = (uint8_t)(RC5TmpPacket.data & 0x3F);

    /* Extended RC5: if field bit == 0, command range is 64..127 */
    if (pFrame->FieldBit == 0)
    {
      pFrame->Command |= (1U << 6);
    }

    RC5FrameReceived = 0;
    RC5_ResetPacket();
  }
}

/* --- Private functions --------------------------------------------------- */

/**
  * @brief  Classify a raw pulse duration as 1T, 2T or invalid.
  */
static uint8_t RC5_GetPulseLength(uint32_t pulseLength)
{
  if ((pulseLength > RC5_MIN_T) && (pulseLength < RC5_MAX_T))
  {
    return RC5_1T_TIME;
  }
  else if ((pulseLength > RC5_MIN_2T) && (pulseLength < RC5_MAX_2T))
  {
    return RC5_2T_TIME;
  }
  return RC5_WRONG_TIME;
}

/**
  * @brief  Validate and store the decoded bit.
  */
static void RC5_modifyLastBit(RC5_lastBit_t bit)
{
  if (bit != RC5_NAN)
  {
    if (RC5TmpPacket.lastBit != RC5_INV)
    {
      RC5TmpPacket.lastBit = bit;
      RC5_WriteBit(RC5TmpPacket.lastBit);
    }
    else
    {
      RC5_ResetPacket();
    }
  }
}

/**
  * @brief  Shift one decoded bit into the data word (MSB first).
  */
static void RC5_WriteBit(uint8_t bitVal)
{
  if (bitVal == RC5_ONE)
  {
    bitVal = 1;
  }
  else if (bitVal == RC5_ZER)
  {
    bitVal = 0;
  }
  else
  {
    RC5_ResetPacket();
    return;
  }

  RC5TmpPacket.data |= bitVal;

  if (RC5TmpPacket.bitCount != 0)
  {
    RC5TmpPacket.data = RC5TmpPacket.data << 1;
    RC5TmpPacket.bitCount--;
  }
  else
  {
    RC5FrameReceived = 1;
  }
}
