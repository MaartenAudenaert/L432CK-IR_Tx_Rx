/**
  ******************************************************************************
  * @file    ir_transceiver.h
  * @brief   Half-duplex IR transceiver state machine for STM32L432KC.
  ******************************************************************************
  */
#ifndef __IR_TRANSCEIVER_H
#define __IR_TRANSCEIVER_H

#include <stdint.h>

/* --- State machine ------------------------------------------------------- */

typedef enum
{
  IR_STATE_IDLE,            /* RX active, waiting */
  IR_STATE_TX_SENDING,      /* Frame being transmitted, RX disabled */
  IR_STATE_TX_GUARD,        /* Guard time after TX (~5 ms), RX still off */
  IR_STATE_RX_PROCESSING    /* Frame received, processing */
} IR_State_t;

/* --- Public functions ---------------------------------------------------- */

/** Start RX capture and carrier PWM (CCR=0). Call once after MX_TIMx_Init(). */
void     IR_Transceiver_Init(void);

/** Current transceiver state. */
IR_State_t IR_GetState(void);

/**
  * @brief  Request an RC5 transmission (non-blocking).
  *         Only accepted from IR_STATE_IDLE.
  * @retval 0 = accepted, 1 = busy / refused.
  */
uint8_t  IR_StartTransmit(uint8_t toggle, uint8_t address, uint8_t command);

/**
  * @brief  Call from main loop — handles guard-time completion and RX re-enable.
  */
void     IR_Transceiver_Process(void);

/**
  * @brief  Signal that the encoder has finished (called from TIM15 ISR context).
  *         Transitions state from TX_SENDING → TX_GUARD.
  */
void     IR_TX_Complete(void);

#endif /* __IR_TRANSCEIVER_H */
