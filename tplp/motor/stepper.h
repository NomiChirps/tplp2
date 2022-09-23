#ifndef TPLP_MOTOR_STEPPER_H_
#define TPLP_MOTOR_STEPPER_H_

#include <optional>

#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "picolog/picolog.h"
#include "tplp/bus/dma.h"
#include "tplp/bus/types.h"
#include "tplp/motor/stepper.pio.h"
#include "tplp/util.h"

namespace tplp {

// Not thread-safe.
class StepperMotor {
 private:
  // We need the size of the table to be a power of 2 for efficient modular
  // arithmetic and for ring DMA. kCommandBufSize > 256 isn't very useful
  // because the PIO program only has 8-bit resolution for the PWM period.
  static constexpr size_t kCommandBufLen = 128;
  static_assert(__builtin_popcount(kCommandBufLen) == 1);
  static constexpr size_t kCommandBufNumBytes =
      kCommandBufLen * sizeof(uint32_t);
  static constexpr uint8_t kCommandBufRingSize =
      __builtin_popcount(kCommandBufNumBytes - 1);
  static_assert(1 << kCommandBufRingSize == kCommandBufNumBytes);

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

  // The initial state of the motor is de-energized.
  template <StepperMotor* motor>
  static void Init(PIO pio, const Hardware& hw, int pio_pwm_freq_hz,
                   int pwm_slice_num, DmaController* dma);

  // Starts moving in a positive or negative direction at the given rate of
  // microsteps per second. If the motor is already moving, this can change both
  // the speed and the direction. Changes take effect immediately. Excessively
  // high or low values of microstep_hz are capped to the appropriate values.
  void Move(int32_t microstep_hz);
  // Stops the motor and holds it in the current position.
  void Stop() { Move(0); }
  // Stops the motor and de-energizes it, releasing the position.
  void Release();

  static int32_t min_microstep_hz();
  static int32_t max_microstep_hz();

  // Returns the current open-loop position of the motor, not accounting for
  // missed steps due to external factors. This may wrap around int32_max if the
  // motor runs fast enough for long enough.
  int32_t GetPosition() const;
  // Resets the stored position to the given value.
  void SetPosition(int32_t position);

 private:
  static bool static_init_done_;
  static void StaticInit();
  static void StaticInit_Clocks(int pio_pwm_freq_hz);
  static void StaticInit_Commands();
  static std::optional<int> pio0_program_offset_;
  static std::optional<int> pio1_program_offset_;
  static uint16_t pio_pwm_period_;  // num iterations
  static int pio_clkdiv_int_;
  static int pio_clkdiv_frac_;
  static int pio_hz_;  // approximate clock speed
  static uint32_t shortbrake_command_;
  // Aligning the buffer allows for very efficient modular arithmetic.
  // It's also required for ring DMA.
  [[gnu::aligned(
      kCommandBufNumBytes)]] static uint32_t commands_fwd_[kCommandBufLen];
  [[gnu::aligned(
      kCommandBufNumBytes)]] static uint32_t commands_rev_[kCommandBufLen];

  // DMA config
  static constexpr uint32_t kInitialTransCount =
      std::numeric_limits<uint32_t>::max();

  DmaController::Request MakeDmaRequest(const uint32_t* read_addr) {
    return {
        .c0_enable = true,
        .c0_treq_sel = static_cast<uint8_t>(pwm_get_dreq(pwm_slice_num_)),
        .c0_read_addr = read_addr,
        .c0_read_incr = true,
        .c0_write_addr = txf_,
        .c0_write_incr = false,
        // PIO commands are 32 bits wide.
        .c0_data_size = DmaController::DataSize::k32,
        // TODO: maybe make provisions for refreshing this occasionally?
        //       just in case the printer is left running for days/weeks?
        .c0_trans_count = kInitialTransCount,
        .c0_ring_size = kCommandBufRingSize,
        .c0_ring_sel = 0,  // ring size applies to reads
    };
  }

 private:
  PIO pio_;
  int sm_;
  // TX FIFO register.
  volatile uint32_t* txf_;
  int program_offset_;
  int pwm_slice_num_;
  DmaController* dma_;
  // command index at which the previous dma transfer stopped reading
  size_t last_dma_read_index_;
  int current_dma_signum_;

  Hardware hw_;

  // Current speed and direction (+/-)
  int32_t microstep_hz_;
  // Currently running DMA transfer
  DmaController::TransferHandle current_dma_;
  // Motor "position" as of the start of current DMA transfer.
  int32_t microstep_counter_;

 private:
  // Not copyable or moveable.
  StepperMotor(const StepperMotor&) = delete;
  StepperMotor& operator=(const StepperMotor&) = delete;
};

template <StepperMotor* motor>
void StepperMotor::Init(PIO pio, const Hardware& hw, int pio_pwm_freq_hz,
                        int pwm_slice_num, DmaController* dma) {
  if (!static_init_done_) {
    StaticInit_Clocks(pio_pwm_freq_hz);
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
  {
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
    // in what we're sending. And anyway, we're exceedingly unlikely to take
    // `pio_pwm_freq_hz` microsteps in a second.
    pio_sm_init(pio, sm, program_offset, &c);
    pio_sm_set_consecutive_pindirs(pio, sm, hw.a1, 4, /*is_out=*/true);

    // Put a safe initial command into the FIFO before starting the state
    // machine.
    *motor->txf_ = motor->shortbrake_command_;
    pio_sm_set_enabled(pio, sm, true);
  }

  // Set up the PWM/DMA pair that will push commands to the PIO.
  {
    pwm_config c = pwm_get_default_config();
    // use phase correct mode to extend the maximum period by a factor of 2,
    // allowing microstep rates as low as 4 Hz (vs. 7-ish Hz).
    pwm_config_set_phase_correct(&c, true);
    // pico-sdk's function doesn't allow setting div to 0, but we need to do
    // that because the hardware interprets this as 256, i.e., maximum division.
    // So just set it ourselves.
    c.div = 0;
    pwm_config_set_clkdiv_mode(&c, PWM_DIV_FREE_RUNNING);
    pwm_config_set_wrap(&c, 65535);
    // Start it in the disabled state.
    pwm_init(pwm_slice_num, &c, false);
    // XXX: does irq actually need to be enabled for dreqs to happen?
    pwm_set_irq_enabled(pwm_slice_num, true);
  }

  motor->pio_ = pio;
  motor->hw_ = hw;
  motor->txf_ = &pio->txf[sm];
  motor->sm_ = sm;
  motor->program_offset_ = program_offset;
  motor->pwm_slice_num_ = pwm_slice_num;
  motor->dma_ = dma;
  motor->microstep_hz_ = 0;

  // Start initial DMA in the forward direction.
  // This won't send any commands until the PWM is enabled.
  motor->last_dma_read_index_ = 0;
  motor->current_dma_signum_ = 1;
  motor->current_dma_ =
      motor->dma_->Transfer(motor->MakeDmaRequest(commands_fwd_));
}

}  // namespace tplp

#endif  // TPLP_MOTOR_STEPPER_H_