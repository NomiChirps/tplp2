#ifndef TPLP_MOTOR_STEPPER_H_
#define TPLP_MOTOR_STEPPER_H_

#include "hardware/pio.h"
#include "tplp/bus/types.h"
#include "tplp/clkdiv.h"

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
  struct Hardware {
    // a1, a2, b1, b2 must be consecutive GPIO numbers.
    gpio_pin_t a1;
    gpio_pin_t a2;
    gpio_pin_t b1;
    gpio_pin_t b2;
  };

  static constexpr size_t kMicrostepsPerStep = kCommandBufLen;

  static void StaticInit(int pwm_freq_hz);
  // The initial state of the motor is stopped and not energized.
  static StepperMotor* Init(PIO pio, const Hardware& hw, int alarm_num,
                            int timer_irq_priority);

  // Sets the speed and direction of the motor. This takes effect immediately.
  // The motor will step forward by `stride` microsteps every `interval_us`
  // microseconds. The value of `interval_us` must be positive and no less than
  // min_microstep_interval(). The motor can be paused by setting `stride` to
  // zero, which is exactly what `Stop()` does.
  //
  // TODO: Shortening the interval currently does not cause an immediate step;
  //       you still have to wait for the previous interval to end. This can be
  //       a problem if quickly changing from a very long to a very short
  //       interval is required.
  void SetSpeed(int32_t stride, int32_t interval_us);
  // See SetSpeed(); this updates only one of the parameters.
  void SetStride(int32_t stride);
  // See SetSpeed(); this updates only one of the parameters.
  void SetInterval(int32_t interval_us);

  // As SetStride(), but with no checks or logging.
  inline void SetStrideFromISR(int32_t stride) { stride_ = stride; }
  // As SetInterval(), but with no checks or logging.
  void SetIntervalFromISR(int32_t interval_us) {
    timer_interval_us_ = interval_us;
  }

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
  static bool static_init_done_;
  static uint16_t pwm_period_;  // num iterations
  static int pio_clkdiv_int_;
  static int pio_clkdiv_frac_;
  static int pio_hz_;  // approximate clock speed
  static uint32_t shortbrake_command_;
  // Aligning the buffer allows for very efficient modular arithmetic.
  [[gnu::aligned(
      kCommandBufNumBytes)]] static uint32_t commands_[kCommandBufLen];

  // templated on the motor id, which is also the hardware alarm number
  template <int M>
  static void __not_in_flash("StepperMotor") timer_isr();
  template <int M>
  void InitTimer(int irq_priority);

 private:
  explicit StepperMotor();

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

}  // namespace tplp

#endif  // TPLP_MOTOR_STEPPER_H_