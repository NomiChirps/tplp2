#ifndef TPLP_MOTOR_STEPPER_H_
#define TPLP_MOTOR_STEPPER_H_

#include <optional>

#include "hardware/pio.h"
#include "hardware/timer.h"
#include "picolog/picolog.h"
#include "tplp/bus/types.h"
#include "tplp/clkdiv.h"
#include "tplp/motor/stepper.pio.h"
#include "tplp/util.h"

namespace tplp {

// Not thread-safe.
class StepperMotor {
 private:
  // We need the size of the table to be a power of 2 for efficient modular
  // arithmetic. kCommandBufSize > 256 isn't very useful because the PIO program
  // only has 8-bit resolution for the PWM period.
  static constexpr size_t kCommandBufLen = 128;
  static_assert(__builtin_popcount(kCommandBufLen) == 1);
  static constexpr size_t kCommandBufNumBytes =
      kCommandBufLen * sizeof(uint32_t);

 public:
  explicit StepperMotor();
  struct Hardware {
    // a1, a2, b1, b2 must be consecutive GPIO numbers.
    gpio_pin_t a1;
    gpio_pin_t a2;
    gpio_pin_t b1;
    gpio_pin_t b2;
  };

  static constexpr size_t kMicrostepsPerStep = kCommandBufLen;

  // The initial state of the motor is not energized.
  template <int AlarmNum, StepperMotor* motor>
  static void Init(PIO pio, const Hardware& hw, int pwm_freq_hz,
                   int timer_irq_priority);

  // Sets the speed and direction of the motor. This takes effect immediately.
  // The motor will step forward by `stride` microsteps every `interval_us`
  // microseconds. The value of `interval_us` must be positive and no less than
  // min_microstep_interval(). The motor can be paused by setting `stride` to
  // zero, which is exactly what `Stop()` does.
  //
  // Note that there's potential for a race condition here (though with only
  // minor consequences). The motor control timer interrupt might occur after
  // updating stride_ but before updating interval_us_, potentially causing a
  // brief glitch. If that's unacceptable, wrap this call in a critical
  // section.
  //
  // ISR-safe.
  //
  // TODO: Shortening the interval currently does not cause an immediate step;
  //       you still have to wait for the previous interval to end. This can be
  //       a problem if quickly changing from a very long to a very short
  //       interval is required.
  void SetSpeed(int32_t stride, int32_t interval_us) {
    stride_ = stride;
    timer_interval_us_ = interval_us;
  }
  // See SetSpeed(); this updates only one of the parameters. ISR-safe.
  void SetStride(int32_t stride) { stride_ = stride; }
  // See SetSpeed(); this updates only one of the parameters. ISR-safe.
  void SetInterval(int32_t interval_us) { timer_interval_us_ = interval_us; }

  // Returns the interval_us value corresponding to the fastest allowed
  // speed. This is the smallest valid nonzero argument to SetInterval().
  int32_t min_interval_us() const {
    // The timer can go all the way down to 2 but let's not push our luck.
    return 5;
  }

  bool moving() const { return stride_; }

  // Sets stride to zero and re-energizes the motor if it was released,
  // effectively pausing it in the current position (or a nearby one, if it was
  // moved by some other force while released).
  void Stop();

  // Sets stride to zero and de-energizes the motor, allowed it to spin more
  // freely. This is a "short-brake", not a true high impedance freewheel.
  void Release();

 private:
  static void StaticInit();
  static void StaticInit_Clocks(int pwm_freq_hz);
  static void StaticInit_Commands();
  static uint16_t pwm_period_;  // num iterations
  static int pio_clkdiv_int_;
  static int pio_clkdiv_frac_;
  static int pio_hz_;  // approximate clock speed
  static uint32_t shortbrake_command_;
  // Aligning the buffer allows for very efficient modular arithmetic.
  [[gnu::aligned(
      kCommandBufNumBytes)]] static uint32_t commands_[kCommandBufLen];

  template <int AlarmNum, StepperMotor* motor>
  static void __not_in_flash("StepperMotor") timer_isr();
  template <int AlarmNum, StepperMotor* motor>
  static void InitTimer(int irq_priority);

  static std::optional<int> pio0_program_offset_;
  static std::optional<int> pio1_program_offset_;
  static bool static_init_done_;

 private:
  PIO pio_;
  int sm_;
  // TX FIFO register.
  volatile uint32_t* txf_;

  int program_offset_;
  Hardware hw_;

  int timer_alarm_num_;
  int timer_irq_;

  volatile int32_t stride_;
  volatile uint32_t timer_interval_us_;
  volatile uint32_t command_index_;

 private:
  // Not copyable or moveable.
  StepperMotor(const StepperMotor&) = delete;
  StepperMotor& operator=(const StepperMotor&) = delete;
};

template <int AlarmNum, StepperMotor* motor>
void StepperMotor::InitTimer(int irq_priority) {
  static_assert(AlarmNum < NUM_TIMERS);
  CHECK(!hardware_alarm_is_claimed(AlarmNum));
  hardware_alarm_claim(AlarmNum);
  motor->timer_alarm_num_ = AlarmNum;
  motor->timer_irq_ = TIMER_IRQ_0 + AlarmNum;
  CHECK(!IsInFlash(timer_isr<AlarmNum, motor>));
  irq_set_exclusive_handler(motor->timer_irq_, timer_isr<AlarmNum, motor>);
  irq_set_priority(motor->timer_irq_, irq_priority);
  irq_set_enabled(motor->timer_irq_, true);
  hw_set_bits(&timer_hw->inte, 1u << motor->timer_alarm_num_);
  // keep trying to start the timer (in case we're preempted)
  uint64_t target_time;
  do {
    target_time = time_us_64() + motor->timer_interval_us_;
  } while (hardware_alarm_set_target(motor->timer_alarm_num_, {target_time}));
}

template <int AlarmNum, StepperMotor* motor>
void StepperMotor::Init(PIO pio, const Hardware& hw, int pwm_freq_hz,
                        int timer_irq_priority) {
  if (!static_init_done_) {
    StaticInit_Clocks(pwm_freq_hz);
    StaticInit_Commands();
    static_init_done_ = true;
  }

  CHECK_EQ(hw.a1 + 1, hw.a2) << "StepperMotor pins must be consecutive";
  CHECK_EQ(hw.a2 + 1, hw.b1) << "StepperMotor pins must be consecutive";
  CHECK_EQ(hw.b1 + 1, hw.b2) << "StepperMotor pins must be consecutive";

  int program_offset;
  if (pio_get_index(pio) == 0) {
    if (pio0_program_offset_.has_value()) {
      program_offset = *pio0_program_offset_;
    } else {
      CHECK(pio_can_add_program(pio, &stepper_program))
          << "Not enough free instruction space in PIO" << pio_get_index(pio);
      pio0_program_offset_ = program_offset =
          pio_add_program(pio, &stepper_program);
    }
  } else if (pio_get_index(pio) == 1) {
    if (pio1_program_offset_.has_value()) {
      program_offset = *pio1_program_offset_;
    } else {
      CHECK(pio_can_add_program(pio, &stepper_program))
          << "Not enough free instruction space in PIO" << pio_get_index(pio);
      pio1_program_offset_ = program_offset =
          pio_add_program(pio, &stepper_program);
    }
  } else {
    LOG(FATAL) << "bad pio index";
  }

  int sm = pio_claim_unused_sm(pio, false);
  LOG_IF(FATAL, sm < 0) << "No free state machines in PIO"
                        << pio_get_index(pio);
  pio_sm_config c = stepper_program_get_default_config(program_offset);

  pio_gpio_init(pio, hw.a1);
  pio_gpio_init(pio, hw.a2);
  pio_gpio_init(pio, hw.b1);
  pio_gpio_init(pio, hw.b2);
  sm_config_set_out_pins(&c, hw.a1, 4);
  sm_config_set_out_shift(&c, /*shift_right=*/true, /*autopull=*/false,
                          /*pull_threshold=*/32);
  sm_config_set_clkdiv_int_frac(&c, pio_clkdiv_int_, pio_clkdiv_frac_);
  // Although RX FIFO isn't used, we don't want to join the FIFOs together
  // because (a) we're using DMA to fill them and a depth of 4 is plenty,
  // and (b) the shorter the FIFO is, the quicker it'll respond to changes
  // in what we're sending.

  pio_sm_init(pio, sm, program_offset, &c);
  pio_sm_set_consecutive_pindirs(pio, sm, hw.a1, 4, /*is_out=*/true);

  motor->pio_ = pio;
  motor->hw_ = hw;
  motor->txf_ = &pio->txf[sm];
  motor->sm_ = sm;
  motor->program_offset_ = program_offset;
  motor->command_index_ = 0;
  motor->stride_ = 0;
  motor->timer_interval_us_ = 1'000;

  motor->InitTimer<AlarmNum, motor>(timer_irq_priority);

  // Put a safe initial command into the FIFO before starting the state machine.
  *motor->txf_ = motor->shortbrake_command_;
  pio_sm_set_enabled(pio, sm, true);
}

template <int AlarmNum, StepperMotor* motor>
[[gnu::hot]] __not_in_flash("StepperMotor") void StepperMotor::timer_isr() {
  static uint64_t target_time = time_us_64();
  timer_hw->intr = 1u << AlarmNum;
  for (;;) {
    // Take a copy of stride_ in case it changes.
    int32_t stride = motor->stride_;
    if (stride) {
      *motor->txf_ = motor->commands_[motor->command_index_];
      // Advance command index by the stride.
      motor->command_index_ = (motor->command_index_ + stride) % kCommandBufLen;
    }

    // Advance target time by one interval.
    target_time += motor->timer_interval_us_;
    // Reset the alarm to fire at the new target time.
    timer_hw->armed = 1u << AlarmNum;
    timer_hw->alarm[AlarmNum] = target_time;

    // Make sure new target time is in the future:
    // Read current time.
    uint32_t hi = timer_hw->timerawh;
    uint32_t lo = timer_hw->timerawl;
    // Reread if low bits rolled over.
    if (timer_hw->timerawh != hi) {
      hi = timer_hw->timerawh;
      lo = timer_hw->timerawl;
    }
    if ((((uint64_t)hi << 32) | lo) <= target_time) {
      // Alarm successfuly set to the future; done.
      break;
    }
    // The new target time is in the past, meaning we missed an interval.
    // Keep trying to reset it, taking more strides if requested.
    continue;
  }
}

}  // namespace tplp

#endif  // TPLP_MOTOR_STEPPER_H_