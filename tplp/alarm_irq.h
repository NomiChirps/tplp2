#ifndef TPLP_ALARM_IRQ_H_
#define TPLP_ALARM_IRQ_H_

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "picolog/picolog.h"

namespace tplp {

using PeriodicAlarm_isr_body_fn_t = void (*)();
using PeriodicAlarm_get_delay_fn_t = int (*)();

// Encapsulates the use of a hardware alarm to trigger an interrupt handler
// at predictable intervals. Good all the way down to a 2us period.
template <int AlarmNum, PeriodicAlarm_isr_body_fn_t IsrBody,
          PeriodicAlarm_get_delay_fn_t GetDelay>
struct PeriodicAlarm {
  static void Check() {
    CHECK(!(reinterpret_cast<intptr_t>(IsrBody) & 0x10000000))
        << "IsrBodyFn was placed in XIP/Flash memory, but must be in SRAM";
    CHECK(!(reinterpret_cast<intptr_t>(GetDelay) & 0x10000000))
        << "GetDelayFn was placed in XIP/Flash memory, but must be in SRAM";
    CHECK(!(reinterpret_cast<intptr_t>(ISR) & 0x10000000))
        << "ISR was placed in XIP/Flash memory, but must be in SRAM";
  }

  // Due to GCC bug https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70435,
  // this must be explicitly instantiated with the section name, e.g.:
  //
  // template __not_in_flash("PeriodicAlarm") void PeriodicAlarm<
  //    kMyAlarmNum, MyIsrBodyFn, MyGetDelayFn>::ISR();
  __not_in_flash("PeriodicAlarm") static void ISR() {
    static uint64_t timer_target_us = time_us_64();
    gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));

    timer_hw->intr = 1u << AlarmNum;

    IsrBody();

    // Reset the alarm to fire at the next future delay interval.
    const int delay = GetDelay();
    for (;;) {
      timer_target_us += delay;
      timer_hw->armed = 1u << AlarmNum;
      timer_hw->alarm[AlarmNum] = timer_target_us;

      // read current time
      uint32_t hi = timer_hw->timerawh;
      uint32_t lo = timer_hw->timerawl;
      // reread if low bits rolled over
      if (timer_hw->timerawh != hi) {
        hi = timer_hw->timerawh;
        lo = timer_hw->timerawl;
      }

      if ((((uint64_t)hi << 32) | lo) <= timer_target_us) {
        break;
      }
    }
    gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));
  }
};

}  // namespace tplp

#endif  // TPLP_ALARM_IRQ_H_