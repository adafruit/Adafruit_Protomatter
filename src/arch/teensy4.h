/*!
 * @file teensy4.h
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
 * This file contains i.MX 1062 (Teensy 4.x) SPECIFIC CODE.
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

#if defined(__IMXRT1062__)

// i.MX only allows full 32-bit aligned writes to GPIO.
#define _PM_STRICT_32BIT_IO ///< Change core.c behavior for long accesses only

#if defined(ARDUINO) // COMPILING FOR ARDUINO ------------------------------

static const struct {
  volatile uint32_t *base; ///< GPIO base address for pin
  uint8_t bit;             ///< GPIO bit number for pin (0-31)
} _PM_teensyPins[] = {
    {&CORE_PIN0_PORTREG, CORE_PIN0_BIT},
    {&CORE_PIN1_PORTREG, CORE_PIN1_BIT},
    {&CORE_PIN2_PORTREG, CORE_PIN2_BIT},
    {&CORE_PIN3_PORTREG, CORE_PIN3_BIT},
    {&CORE_PIN4_PORTREG, CORE_PIN4_BIT},
    {&CORE_PIN5_PORTREG, CORE_PIN5_BIT},
    {&CORE_PIN6_PORTREG, CORE_PIN6_BIT},
    {&CORE_PIN7_PORTREG, CORE_PIN7_BIT},
    {&CORE_PIN8_PORTREG, CORE_PIN8_BIT},
    {&CORE_PIN9_PORTREG, CORE_PIN9_BIT},
    {&CORE_PIN10_PORTREG, CORE_PIN10_BIT},
    {&CORE_PIN11_PORTREG, CORE_PIN11_BIT},
    {&CORE_PIN12_PORTREG, CORE_PIN12_BIT},
    {&CORE_PIN13_PORTREG, CORE_PIN13_BIT},
    {&CORE_PIN14_PORTREG, CORE_PIN14_BIT},
    {&CORE_PIN15_PORTREG, CORE_PIN15_BIT},
    {&CORE_PIN16_PORTREG, CORE_PIN16_BIT},
    {&CORE_PIN17_PORTREG, CORE_PIN17_BIT},
    {&CORE_PIN18_PORTREG, CORE_PIN18_BIT},
    {&CORE_PIN19_PORTREG, CORE_PIN19_BIT},
    {&CORE_PIN20_PORTREG, CORE_PIN20_BIT},
    {&CORE_PIN21_PORTREG, CORE_PIN21_BIT},
    {&CORE_PIN22_PORTREG, CORE_PIN22_BIT},
    {&CORE_PIN23_PORTREG, CORE_PIN23_BIT},
    {&CORE_PIN24_PORTREG, CORE_PIN24_BIT},
    {&CORE_PIN25_PORTREG, CORE_PIN25_BIT},
    {&CORE_PIN26_PORTREG, CORE_PIN26_BIT},
    {&CORE_PIN27_PORTREG, CORE_PIN27_BIT},
    {&CORE_PIN28_PORTREG, CORE_PIN28_BIT},
    {&CORE_PIN29_PORTREG, CORE_PIN29_BIT},
    {&CORE_PIN30_PORTREG, CORE_PIN30_BIT},
    {&CORE_PIN31_PORTREG, CORE_PIN31_BIT},
    {&CORE_PIN32_PORTREG, CORE_PIN32_BIT},
    {&CORE_PIN33_PORTREG, CORE_PIN33_BIT},
    {&CORE_PIN34_PORTREG, CORE_PIN34_BIT},
    {&CORE_PIN35_PORTREG, CORE_PIN35_BIT},
    {&CORE_PIN36_PORTREG, CORE_PIN36_BIT},
    {&CORE_PIN37_PORTREG, CORE_PIN37_BIT},
    {&CORE_PIN38_PORTREG, CORE_PIN38_BIT},
    {&CORE_PIN39_PORTREG, CORE_PIN39_BIT},
};

#define _PM_SET_OFFSET 33    ///< 0x84 byte offset = 33 longs
#define _PM_CLEAR_OFFSET 34  ///< 0x88 byte offset = 34 longs
#define _PM_TOGGLE_OFFSET 35 ///< 0x8C byte offset = 35 longs

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define _PM_byteOffset(pin) (_PM_teensyPins[pin].bit / 8)
#define _PM_wordOffset(pin) (_PM_teensyPins[pin].bit / 16)
#else
#define _PM_byteOffset(pin) (3 - (_PM_teensyPins[pin].bit / 8))
#define _PM_wordOffset(pin) (1 - (_PM_teensyPins[pin].bit / 16))
#endif

#define _PM_portOutRegister(pin) (void *)_PM_teensyPins[pin].base

#define _PM_portSetRegister(pin)                                               \
  ((volatile uint32_t *)_PM_teensyPins[pin].base + _PM_SET_OFFSET)

#define _PM_portClearRegister(pin)                                             \
  ((volatile uint32_t *)_PM_teensyPins[pin].base + _PM_CLEAR_OFFSET)

#define _PM_portToggleRegister(pin)                                            \
  ((volatile uint32_t *)_PM_teensyPins[pin].base + _PM_TOGGLE_OFFSET)

// As written, because it's tied to a specific timer right now, the
// Arduino lib only permits one instance of the Protomatter_core struct,
// which it sets up when calling begin().
void *_PM_protoPtr = NULL;

// Code as written works with the Periodic Interrupt Timer directly,
// rather than using the Teensy IntervalTimer library, reason being we
// need to be able to poll the current timer value in _PM_timerGetCount(),
// but that's not available from IntervalTimer, and the timer base address
// it keeps is a private member (possible alternative is to do dirty pool
// and access the pointer directly, knowing it's the first element in the
// IntervalTimer object, but this is fraught with peril).

#define _PM_timerFreq 24000000 // 24 MHz
#define _PM_timerNum 0         // PIT timer #0 (can be 0-3)
#define _PM_TIMER_DEFAULT (IMXRT_PIT_CHANNELS + _PM_timerNum) // PIT channel *

// Interrupt service routine for Periodic Interrupt Timer
static void _PM_timerISR(void) {
  IMXRT_PIT_CHANNEL_t *timer = _PM_TIMER_DEFAULT;
  _PM_row_handler(_PM_protoPtr); // In core.c
  timer->TFLG = 1;               // Clear timer interrupt
}

// Initialize, but do not start, timer.
void _PM_timerInit(Protomatter_core *core) {
  IMXRT_PIT_CHANNEL_t *timer = (IMXRT_PIT_CHANNEL_t *)core->timer;
  CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON); // Enable clock signal to PIT
  PIT_MCR = 1;                             // Enable PIT
  timer->TCTRL = 0;                        // Disable timer and interrupt
  timer->LDVAL = 100000;                   // Timer initial load value
  // Interrupt is attached but not enabled yet
  attachInterruptVector(IRQ_PIT, &_PM_timerISR);
  NVIC_ENABLE_IRQ(IRQ_PIT);
}

// Set timer period, initialize count value to zero, enable timer.
inline void _PM_timerStart(Protomatter_core *core, uint32_t period) {
  IMXRT_PIT_CHANNEL_t *timer = (IMXRT_PIT_CHANNEL_t *)core->timer;
  timer->TCTRL = 0;      // Disable timer and interrupt
  timer->LDVAL = period; // Set load value
  // timer->CVAL = period; // And current value (just in case?)
  timer->TFLG = 1;  // Clear timer interrupt
  timer->TCTRL = 3; // Enable timer and interrupt
}

// Return current count value (timer enabled or not).
// Timer must be previously initialized.
inline uint32_t _PM_timerGetCount(Protomatter_core *core) {
  IMXRT_PIT_CHANNEL_t *timer = (IMXRT_PIT_CHANNEL_t *)core->timer;
  return (timer->LDVAL - timer->CVAL);
}

// Disable timer and return current count value.
// Timer must be previously initialized.
uint32_t _PM_timerStop(Protomatter_core *core) {
  IMXRT_PIT_CHANNEL_t *timer = (IMXRT_PIT_CHANNEL_t *)core->timer;
  timer->TCTRL = 0; // Disable timer and interrupt
  return _PM_timerGetCount(core);
}

#define _PM_clockHoldHigh                                                      \
  asm("nop; nop; nop; nop; nop; nop; nop;");                                   \
  asm("nop; nop; nop; nop; nop; nop; nop;");
#define _PM_clockHoldLow                                                       \
  asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");                    \
  asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");

#define _PM_chunkSize 1 ///< DON'T unroll loop, Teensy 4 is SO FAST

#elif defined(CIRCUITPY) // COMPILING FOR CIRCUITPYTHON --------------------

// Teensy 4 CircuitPython magic goes here.

#endif // END CIRCUITPYTHON ------------------------------------------------

#endif // END __IMXRT1062__ (Teensy 4)
