/**
  ******************************************************************************
  * @file    rc5_decode.h
  * @brief   RC5 infrared decoder for STM32L432KC.
  *          Adapted from X-CUBE-IRREMOTE (ST AN4834).
  ******************************************************************************
  */
#ifndef __RC5_DECODE_H
#define __RC5_DECODE_H

#include <stdint.h>
#include "stm32l4xx_hal.h"  /* for __IO */

/* --- Types --------------------------------------------------------------- */

typedef struct
{
  __IO uint8_t FieldBit;   /*!< Field bit (S2, inverted for cmd >= 64) */
  __IO uint8_t ToggleBit;  /*!< Toggle bit */
  __IO uint8_t Address;    /*!< Address (5-bit) */
  __IO uint8_t Command;    /*!< Command (7-bit, including field bit) */
} RC5_Frame_t;

typedef struct
{
  __IO uint16_t data;
  __IO uint8_t  status;
  __IO uint8_t  lastBit;
  __IO uint8_t  bitCount;
} RC5_Packet_t;

typedef enum
{
  RC5_ZER = 0,
  RC5_ONE,
  RC5_NAN,
  RC5_INV
} RC5_lastBit_t;

/* --- Public variables ---------------------------------------------------- */

extern __IO uint8_t  RC5FrameReceived;  /* 1 when a complete frame is ready */
extern RC5_Frame_t   RC5_FRAME;

/* --- Public functions ---------------------------------------------------- */

void     RC5_ResetPacket(void);
void     RC5_DataSampling(uint32_t rawPulseLength, uint32_t edge);
void     RC5_Decode(RC5_Frame_t *pFrame);

#endif /* __RC5_DECODE_H */
