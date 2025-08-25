/*!
 * @file nrf52.h
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
 * This file contains NRF52-SPECIFIC CODE.
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

#if defined(NRF52_SERIES)

#if defined(ARDUINO) // COMPILING FOR ARDUINO ------------------------------

// digitalPinToPort, g_ADigitalPinMap[] are Arduino specific:

void *_PM_portOutRegister(uint32_t pin) {
  NRF_GPIO_Type *port = digitalPinToPort(pin);
  return &port->OUT;
}

void *_PM_portSetRegister(uint32_t pin) {
  NRF_GPIO_Type *port = digitalPinToPort(pin);
  return &port->OUTSET;
}

void *_PM_portClearRegister(uint32_t pin) {
  NRF_GPIO_Type *port = digitalPinToPort(pin);
  return &port->OUTCLR;
}

// Leave _PM_portToggleRegister(pin) undefined on nRF!

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define _PM_byteOffset(pin) ((g_ADigitalPinMap[pin] & 0x1F) / 8)
#define _PM_wordOffset(pin) ((g_ADigitalPinMap[pin] & 0x1F) / 16)
#else
#define _PM_byteOffset(pin) (3 - ((g_ADigitalPinMap[pin] & 0x1F) / 8))
#define _PM_wordOffset(pin) (1 - ((g_ADigitalPinMap[pin] & 0x1F) / 16))
#endif

// Because it's tied to a specific timer right now, there can be only
// one instance of the Protomatter_core struct. The Arduino library
// sets up this pointer when calling begin().
void *_PM_protoPtr = NULL;

// Arduino implementation is tied to a specific timer/counter,
// Partly because IRQs must be declared at compile-time.
#define _PM_IRQ_HANDLER TIMER4_IRQHandler
#define _PM_timerFreq 16000000
#define _PM_TIMER_DEFAULT NRF_TIMER4

#ifdef __cplusplus
extern "C" {
#endif

// Timer interrupt service routine
void _PM_IRQ_HANDLER(void) {
  if (_PM_TIMER_DEFAULT->EVENTS_COMPARE[0]) {
    _PM_TIMER_DEFAULT->EVENTS_COMPARE[0] = 0;
  }
  _PM_row_handler(_PM_protoPtr); // In core.c
}

#ifdef __cplusplus
}
#endif

#elif defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------

#include "nrf_gpio.h"

volatile uint32_t *_PM_portOutRegister(uint32_t pin) {
  NRF_GPIO_Type *port = nrf_gpio_pin_port_decode(&pin);
  return &port->OUT;
}

volatile uint32_t *_PM_portSetRegister(uint32_t pin) {
  NRF_GPIO_Type *port = nrf_gpio_pin_port_decode(&pin);
  return &port->OUTSET;
}

volatile uint32_t *_PM_portClearRegister(uint32_t pin) {
  NRF_GPIO_Type *port = nrf_gpio_pin_port_decode(&pin);
  return &port->OUTCLR;
}
#define _PM_pinOutput(pin)                                                     \
  nrf_gpio_cfg(pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,    \
               NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE)
#define _PM_pinInput(pin) nrf_gpio_cfg_input(pin)
#define _PM_pinHigh(pin) nrf_gpio_pin_set(pin)
#define _PM_pinLow(pin) nrf_gpio_pin_clear(pin)
#define _PM_portBitMask(pin) (1u << ((pin) & 31))

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define _PM_byteOffset(pin) ((pin & 31) / 8)
#define _PM_wordOffset(pin) ((pin & 31) / 16)
#else
#define _PM_byteOffset(pin) (3 - ((pin & 31) / 8))
#define _PM_wordOffset(pin) (1 - ((pin & 31) / 16))
#endif

// CircuitPython implementation is tied to a specific freq (but the counter
// is dynamically allocated):
#define _PM_timerFreq 16000000

// Because it's tied to a specific timer right now, there can be only
// one instance of the Protomatter_core struct. The Arduino library
// sets up this pointer when calling begin().
void *_PM_protoPtr = NULL;

// Timer interrupt service routine
void _PM_IRQ_HANDLER(void) {
  NRF_TIMER_Type *timer = (((Protomatter_core *)_PM_protoPtr)->timer);
  if (timer->EVENTS_COMPARE[0]) {
    timer->EVENTS_COMPARE[0] = 0;
  }

  _PM_row_handler(_PM_protoPtr); // In core.c
}

#else // END CIRCUITPYTHON -------------------------------------------------

// Byte offset macros, timer and ISR work for other environments go here.

#endif

// CODE COMMON TO ALL ENVIRONMENTS -----------------------------------------

void _PM_timerInit(Protomatter_core *core) {
  static const struct {
    NRF_TIMER_Type *tc; // -> Timer peripheral base address
    IRQn_Type IRQn;     // Interrupt number
  } timer[] = {
#if defined(NRF_TIMER0)
      {NRF_TIMER0, TIMER0_IRQn},
#endif
#if defined(NRF_TIMER1)
      {NRF_TIMER1, TIMER1_IRQn},
#endif
#if defined(NRF_TIMER2)
      {NRF_TIMER2, TIMER2_IRQn},
#endif
#if defined(NRF_TIMER3)
      {NRF_TIMER3, TIMER3_IRQn},
#endif
#if defined(NRF_TIMER4)
      {NRF_TIMER4, TIMER4_IRQn},
#endif
  };
#define NUM_TIMERS (sizeof timer / sizeof timer[0])

  // Determine IRQn from timer address
  uint8_t timerNum = 0;
  while ((timerNum < NUM_TIMERS) &&
         (timer[timerNum].tc != (NRF_TIMER_Type *)core->timer)) {
    timerNum++;
  }
  if (timerNum >= NUM_TIMERS)
    return;

  NRF_TIMER_Type *tc = timer[timerNum].tc;

  tc->TASKS_STOP = 1;               // Stop timer
  tc->MODE = TIMER_MODE_MODE_Timer; // Timer (not counter) mode
  tc->TASKS_CLEAR = 1;
  tc->BITMODE = TIMER_BITMODE_BITMODE_16Bit
                << TIMER_BITMODE_BITMODE_Pos; // 16-bit timer res
  tc->PRESCALER = 0;                          // 1:1 prescale (16 MHz)
  tc->INTENSET = TIMER_INTENSET_COMPARE0_Enabled
                 << TIMER_INTENSET_COMPARE0_Pos; // Event 0 interrupt
  // NVIC_DisableIRQ(timer[timerNum].IRQn);
  // NVIC_ClearPendingIRQ(timer[timerNum].IRQn);
  // NVIC_SetPriority(timer[timerNum].IRQn, 0); // Top priority
  NVIC_EnableIRQ(timer[timerNum].IRQn);
}

inline void _PM_timerStart(Protomatter_core *core, uint32_t period) {
  volatile NRF_TIMER_Type *tc = (volatile NRF_TIMER_Type *)core->timer;
  tc->TASKS_STOP = 1;  // Stop timer
  tc->TASKS_CLEAR = 1; // Reset to 0
  tc->CC[0] = period;
  tc->TASKS_START = 1; // Start timer
}

inline uint32_t _PM_timerGetCount(Protomatter_core *core) {
  volatile NRF_TIMER_Type *tc = (volatile NRF_TIMER_Type *)core->timer;
  tc->TASKS_CAPTURE[1] = 1; // Capture timer to CC[1] register
  return tc->CC[1];         // (don't clobber value in CC[0])
}

uint32_t _PM_timerStop(Protomatter_core *core) {
  volatile NRF_TIMER_Type *tc = (volatile NRF_TIMER_Type *)core->timer;
  tc->TASKS_STOP = 1; // Stop timer
  __attribute__((unused)) uint32_t count = _PM_timerGetCount(core);
  return count;
}

#define _PM_clockHoldHigh asm("nop; nop");

#define _PM_minMinPeriod 100

#endif // END NRF52_SERIES
