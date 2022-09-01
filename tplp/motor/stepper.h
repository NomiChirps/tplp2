#ifndef TPLP_MOTOR_STEPPER_H_
#define TPLP_MOTOR_STEPPER_H_

#include "hardware/pio.h"
#include "tplp/bus/dma.h"
#include "tplp/bus/types.h"

namespace tplp {

// Not thread-safe.
class StepperMotor {
 private:
  // We need the size of the table to be a power of 2 for ring DMA.
  // kCommandBufSize > 256 isn't very useful because the PIO program only has
  // 8-bit resolution for the PWM period anyway.
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

  // TODO: see about increasing it. TB6612 wasn't happy @ 100kHz.
  static constexpr int kPwmFreqHz = 25'000;
  static constexpr size_t kMicrostepsPerStep = kCommandBufLen;

  static StepperMotor* Init(DmaController* dma, PIO pio, const Hardware& hw);

  // Step forward or backward by a certain number of microsteps. This triggers a
  // DMA transfer to send commands to the motor and returns immediately, without
  // waiting for the motor to finish moving. Does nothing if a move is currently
  // in progress. Does nothing if count==0.
  // TODO: might it be useful to queue up moves?
  // TODO: might it be useful to block until done, or have a callback?
  void Move(int32_t microsteps);

  // Moves both motors simultaneously. This is guaranteed to keep them
  // synchronized, unlike calling a->Move() followed by b->Move().
  // Note that after starting two motors with SimultaneousMove, calling Stop()
  // on EITHER motor will stop both of them, simultaneously.
  static void SimultaneousMove(StepperMotor* a, int32_t microsteps_a,
                               StepperMotor* b, int32_t microsteps_b);

  bool moving() const { return !current_move_handle_.finished(); }

  struct ClockDivider {
    uint16_t num;
    uint16_t den;
  };

  // Set the speed of the next move, or change the speed of a move currently
  // in progress. Use `CalculateClockDivider` to get an appropriate value for
  // clkdiv.
  void SetSpeed(ClockDivider clkdiv);

  // After Init(), returns the minimum achievable microstep rate.
  static uint32_t microstep_hz_min();
  // After Init(), returns the maximum achievable microstep rate.
  // TODO: test at this frequency
  static uint32_t microstep_hz_max();

  // Calculates the clock divider value which most closely approximates the
  // given frequency, in units of microsteps per second. The frequency must be
  // positive. This is a complicated calculation and could take up to a few
  // hundred cycles.
  //
  // Returns true if successful, false if the given value is outside of the
  // range given by `microstep_hz_min` and `microstep_hz_max`.
  //
  // Not valid to call this before Init().
  static bool CalculateClockDivider(int microstep_hz, ClockDivider* out_clkdiv);

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
  static void StaticInit_Clocks();
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
      kCommandBufNumBytes)]] static uint32_t commands_[kCommandBufLen];

  // Don't call this while the motor is moving,
  // otherwise it fights against the DMA.
  void SendImmediateCommand(uint32_t);

  size_t command_index_after_move() const {
    return offset_after_move_ % kCommandBufLen;
  }

 private:
  explicit StepperMotor(DmaController* dma);

  DmaController* const dma_;
  PIO pio_;
  int sm_;
  // TX FIFO register.
  volatile uint32_t* txf_;

  int program_offset_;
  Hardware hw_;

  uint8_t dma_timer_num_;  // 0 to 3
  uint8_t dma_timer_dreq_;
  DmaController::TransferHandle current_move_handle_;
  // true is forward, false is reverse.
  bool move_dir_;
  // unbounded; see command_index_after_move()
  uint32_t offset_after_move_;

 private:
  // Not copyable or moveable.
  StepperMotor(const StepperMotor&) = delete;
  StepperMotor& operator=(const StepperMotor&) = delete;
};

}  // namespace tplp

#endif  // TPLP_MOTOR_STEPPER_H_