/*!
 * @file stm32.h
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
 * This file contains STM32-SPECIFIC CODE.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Phil "Paint Your Dragon" Burgess and Jeff Epler for
 * Adafruit Industries, with contributions from the open source community.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#pragma once

#if defined(STM32F4_SERIES) || defined(STM32F405xx) // Arduino, CircuitPy

#if defined(ARDUINO) // COMPILING FOR ARDUINO ------------------------------

// Arduino port register lookups go here, else ones in arch.h are used.

#elif defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------

#include "timers.h"

#undef _PM_portBitMask
#define _PM_portBitMask(pin) (1u << ((pin) & 15))
#define _PM_byteOffset(pin) ((pin & 15) / 8)
#define _PM_wordOffset(pin) ((pin & 15) / 16)

#define _PM_pinOutput(pin_)                                                    \
  do {                                                                         \
    int8_t pin = (pin_);                                                       \
    GPIO_InitTypeDef GPIO_InitStruct = {0};                                    \
    GPIO_InitStruct.Pin = 1 << (pin & 15);                                     \
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;                                \
    GPIO_InitStruct.Pull = GPIO_NOPULL;                                        \
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;                         \
    HAL_GPIO_Init(pin_port(pin / 16), &GPIO_InitStruct);                       \
  } while (0)
#define _PM_pinInput(pin_)                                                     \
  do {                                                                         \
    int8_t pin = (pin_);                                                       \
    GPIO_InitTypeDef GPIO_InitStruct = {0};                                    \
    GPIO_InitStruct.Pin = 1 << (pin & 15);                                     \
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;                                    \
    GPIO_InitStruct.Pull = GPIO_NOPULL;                                        \
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;                         \
    HAL_GPIO_Init(pin_port(pin / 16), &GPIO_InitStruct);                       \
  } while (0)
#define _PM_pinHigh(pin)                                                       \
  HAL_GPIO_WritePin(pin_port(pin / 16), 1 << (pin & 15), GPIO_PIN_SET)
#define _PM_pinLow(pin)                                                        \
  HAL_GPIO_WritePin(pin_port(pin / 16), 1 << (pin & 15), GPIO_PIN_RESET)

#define _PM_PORT_TYPE uint16_t

volatile uint16_t *_PM_portOutRegister(uint32_t pin) {
  return (uint16_t *)&pin_port(pin / 16)->ODR;
}

volatile uint16_t *_PM_portSetRegister(uint32_t pin) {
  return (uint16_t *)&pin_port(pin / 16)->BSRR;
}

// To make things interesting, STM32F4xx places the set and clear
// GPIO bits within a single register.  The "clear" bits are upper, so
// offset by 1 in uint16_ts
volatile uint16_t *_PM_portClearRegister(uint32_t pin) {
  return 1 + (uint16_t *)&pin_port(pin / 16)->BSRR;
}

// TODO: was this somehow specific to TIM6?
#define _PM_timerFreq 42000000

// Because it's tied to a specific timer right now, there can be only
// one instance of the Protomatter_core struct. The Arduino library
// sets up this pointer when calling begin().
// TODO: this is no longer true, should it change?
void *_PM_protoPtr = NULL;

static TIM_HandleTypeDef tim_handle;

// Timer interrupt service routine
void _PM_IRQ_HANDLER(void) {
  // Clear overflow flag:
  //_PM_TIMER_DEFAULT->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;
  _PM_row_handler(_PM_protoPtr); // In core.c
}

// Initialize, but do not start, timer
void _PM_timerInit(Protomatter_core *core) {
  TIM_TypeDef *tim_instance = (TIM_TypeDef *)core->timer;
  stm_peripherals_timer_reserve(tim_instance);
  // Set IRQs at max priority and start clock
  stm_peripherals_timer_preinit(tim_instance, 0, _PM_IRQ_HANDLER);

  tim_handle.Instance = tim_instance;
  tim_handle.Init.Period = 1000; // immediately replaced.
  tim_handle.Init.Prescaler = 0;
  tim_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_TIM_Base_Init(&tim_handle);

  size_t tim_irq = stm_peripherals_timer_get_irqnum(tim_instance);
  HAL_NVIC_DisableIRQ(tim_irq);
  NVIC_ClearPendingIRQ(tim_irq);
  NVIC_SetPriority(tim_irq, 0); // Top priority
}

inline void _PM_timerStart(Protomatter_core *core, uint32_t period) {
  TIM_TypeDef *tim = core->timer;
  tim->SR = 0;
  tim->ARR = period;
  tim->CR1 |= TIM_CR1_CEN;
  tim->DIER |= TIM_DIER_UIE;
  HAL_NVIC_EnableIRQ(stm_peripherals_timer_get_irqnum(tim));
}

inline uint32_t _PM_timerGetCount(Protomatter_core *core) {
  TIM_TypeDef *tim = core->timer;
  return tim->CNT;
}

uint32_t _PM_timerStop(Protomatter_core *core) {
  TIM_TypeDef *tim = core->timer;
  HAL_NVIC_DisableIRQ(stm_peripherals_timer_get_irqnum(tim));
  tim->CR1 &= ~TIM_CR1_CEN;
  tim->DIER &= ~TIM_DIER_UIE;
  return tim->CNT;
}
// settings from M4 for >= 150MHz, we use this part at 168MHz
#define _PM_clockHoldHigh asm("nop; nop; nop");
#define _PM_clockHoldLow asm("nop");

#define _PM_minMinPeriod 140

#endif // END CIRCUITPYTHON ------------------------------------------------

#endif // END STM32F4_SERIES || STM32F405xx
