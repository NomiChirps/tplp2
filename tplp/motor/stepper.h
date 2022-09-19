#ifndef TPLP_MOTOR_STEPPER_H_
#define TPLP_MOTOR_STEPPER_H_

#include "hardware/pio.h"
#include "tplp/bus/dma.h"
#include "tplp/bus/types.h"
#include "tplp/clkdiv.h"

namespace tplp {

// Not thread-safe.
class StepperMotor {
 private:
  // We need the size of the table to be a power of 2 for ring DMA. This also
  // makes modular arithmetic in this base more efficient. kCommandBufSize > 256
  // isn't very useful because the PIO program only has 8-bit resolution for the
  // PWM period anyway.
  static constexpr size_t kCommandBufLen = 128;
  static constexpr size_t kCommandBufNumBytes =
      kCommandBufLen * sizeof(uint32_t);
  static constexpr uint8_t kCommandBufRingSizeBits = 9;
  static_assert(1 << kCommandBufRingSizeBits == kCommandBufNumBytes);

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
  static StepperMotor* Init(DmaController* dma, PIO pio, const Hardware& hw,
                            int timer_irq_priority);

  // Step forward or backward by a certain number of microsteps. This triggers a
  // DMA transfer to send commands to the motor and returns immediately, without
  // waiting for the motor to finish moving. Does nothing if a move is currently
  // in progress. Does nothing if count==0.
  // TODO: might it be useful to queue up moves?
  // TODO: might it be useful to block until done, or have a callback?
  void Move(int32_t microsteps);

  bool moving() const {
    return !current_dma_handle_.finished() || timer_steps_remaining_;
  }

  // Sets the speed of the motor, automatically deciding whether to use DMA or
  // timer mode. This may be slow because it needs to compute a clock divider
  // for DMA mode.
  void SetSpeedSlow(uint32_t microstep_hz);

  // Set the speed of the next move, or change the speed of a move currently
  // in progress. Use `ComputeClockDivider` to get an appropriate value for
  // clkdiv. This also switches to DMA mode if not in that mode already.
  void SetDmaSpeed(ClockDivider clkdiv);
  // Set the speed of the next move, or change the speed of a move currently
  // in progress. The value is given in microseconds per microstep and must be
  // greater than zero. This also switches to timer interrupt mode if not in
  // that mode already.
  void SetTimerSpeed(uint32_t us_per_microstep);

  // After Init(), returns the minimum achievable microstep rate in DMA mode.
  static uint32_t microstep_hz_dma_min();
  // After Init(), returns the maximum achievable microstep rate in DMA mode.
  // TODO: test at this frequency
  static uint32_t microstep_hz_dma_max();

  enum class StopType {
    // Motor coils remain energized to whatever state they were in
    // at the time of the stop. High torque.
    HOLD,
    // Motor coils are shorted together. Medium-low torque.
    SHORT_BRAKE,
  };

  // Abort any move in progress and apply the given stop type.
  void Stop(StopType type);

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
  // Command buffer MUST be aligned to its power-of-2 size in order for ring DMA
  // to function.
  [[gnu::aligned(
      kCommandBufNumBytes)]] static uint32_t fwd_commands_[kCommandBufLen];
  [[gnu::aligned(
      kCommandBufNumBytes)]] static uint32_t rev_commands_[kCommandBufLen];

  // templated on the motor id, which is also the hardware alarm number
  template <int M>
  static void __not_in_flash("StepperMotor") timer_isr();
  template <int M>
  void InitTimer(int irq_priority);

  // Don't call this while the motor is moving,
  // otherwise it fights against the DMA.
  void SendImmediateCommand(uint32_t);

  size_t command_index_after_move() const {
    return offset_after_move_ % kCommandBufLen;
  }

  void StartDma(uint32_t microsteps);
  void StartTimer(uint32_t microsteps);

  // Aborts the current DMA-based move in progress and updates
  // offset_after_move_ to account for the unfinished steps.
  // Returns the number of microsteps skipped.
  uint32_t AbortDma();
  // Aborts the current timer-based move in progress and updates
  // offset_after_move_ to account for the unfinished steps.
  // Returns the number of microsteps skipped.
  uint32_t AbortTimer();

 private:
  explicit StepperMotor(DmaController* dma);

  const uint32_t* commands_;
  DmaController* const dma_;
  PIO pio_;
  int sm_;
  // TX FIFO register.
  volatile uint32_t* txf_;

  int program_offset_;
  Hardware hw_;

  // True if using DMA to push commands, false if using a timer.
  bool dma_mode_;
  uint8_t dma_timer_num_;  // 0 to 3
  uint8_t dma_timer_dreq_;
  DmaController::TransferHandle current_dma_handle_;

  int timer_alarm_num_;
  int timer_irq_;
  uint32_t timer_steps_remaining_;
  uint32_t timer_delay_us_;
  uint64_t timer_target_us_;

  // unbounded; see command_index_after_move()
  uint32_t offset_after_move_;

 private:
  // Not copyable or moveable.
  StepperMotor(const StepperMotor&) = delete;
  StepperMotor& operator=(const StepperMotor&) = delete;
};

}  // namespace tplp

#endif  // TPLP_MOTOR_STEPPER_H_