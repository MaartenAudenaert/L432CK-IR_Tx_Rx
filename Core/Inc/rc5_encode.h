/**
  ******************************************************************************
  * @file    rc5_encode.h
  * @brief   RC5 infrared encoder for STM32L432KC.
  *          Uses TIM15 (889 µs envelope ISR) + TIM16 (38 kHz carrier PWM).
  ******************************************************************************
  */
#ifndef __RC5_ENCODE_H
#define __RC5_ENCODE_H

#include <stdint.h>

/* --- Public functions ---------------------------------------------------- */

/**
  * @brief  Build the Manchester half-bit array and start transmission.
  * @param  toggle   Toggle bit (0 or 1, flip each new press).
  * @param  address  RC5 device address (0..31).
  * @param  command  RC5 command (0..127, field bit auto-set for >=64).
  */
void RC5_Encode_SendFrame(uint8_t toggle, uint8_t address, uint8_t command);

/**
  * @brief  Called from TIM15 Update ISR — steps to next half-bit.
  *         Do NOT call from user code.
  */
void RC5_Encode_SignalGenerate(void);

/**
  * @brief  Returns 1 while a frame is being transmitted.
  */
uint8_t RC5_Encode_IsBusy(void);

#endif /* __RC5_ENCODE_H */
