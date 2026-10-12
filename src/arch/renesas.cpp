/*!
 * @file renesas.cpp
 *
 * Part of Adafruit's Protomatter library for HUB75-style RGB LED matrices.
 * Timer allocation for Renesas RA devices. This needs the Arduino core's
 * C++ FspTimer class, so it can't live in renesas.h (which is only ever
 * compiled as part of core.c). Register-level timer access is in renesas.h.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#if defined(ARDUINO_ARCH_RENESAS)

#include <Arduino.h>
#include <FspTimer.h>

#include "../core.h"

extern "C" void* _PM_protoPtr; // In core.c (via renesas.h)

static FspTimer _PM_fspTimer;

static void _PM_timerCallback(timer_callback_args_t* args) {
  if (args->event == TIMER_EVENT_CYCLE_END) {
    _PM_row_handler((Protomatter_core*)_PM_protoPtr); // In core.c
  }
}

extern "C" void* _PM_renesasTimerInit(void) {
  static R_GPT0_Type* gpt = NULL;
  if (gpt)
    return gpt; // Already allocated on a prior begin()

  uint8_t type = GPT_TIMER;
  int8_t channel = FspTimer::get_available_timer(type);
  if ((channel < 0) || (type != GPT_TIMER)) {
    // Only AGT left (different registers). Take a GPT that's reserved
    // for PWM instead; that pin's analogWrite() won't work afterward.
    type = GPT_TIMER;
    channel = FspTimer::get_available_timer(type, true);
    if ((channel < 0) || (type != GPT_TIMER))
      return NULL;
  }

  // Periodic mode, but don't start yet; _PM_timerStart() does that.
  if (!_PM_fspTimer.begin(TIMER_MODE_PERIODIC, GPT_TIMER, channel, 1000, 0,
                          TIMER_SOURCE_DIV_4, _PM_timerCallback) ||
      !_PM_fspTimer.setup_overflow_irq(0) || // Top priority
      !_PM_fspTimer.open()) {
    return NULL;
  }

  gpt = (R_GPT0_Type*)((uint32_t)R_GPT0 +
                       channel * ((uint32_t)R_GPT1 - (uint32_t)R_GPT0));
  gpt->GTCR_b.CST = 0;
  return gpt;
}

#endif // END ARDUINO_ARCH_RENESAS
