#include "tplp/motor/stepper.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/time.h"
#include "picolog/picolog.h"
#include "tplp/motor/stepper.pio.h"

namespace tplp {
namespace {
static constexpr int stepper_offset_phase[] = {
    stepper_offset_new_command, stepper_offset_phase1,
    stepper_offset_new_command, stepper_offset_phase3,
    stepper_offset_new_command, stepper_offset_phase5,
    stepper_offset_new_command, stepper_offset_phase7,
};
}

static constexpr int kCyclesPerPwmCount = 4;

StepperMotor* StepperMotor::Init(PIO pio, const Hardware& hw) {
  CHECK_EQ(hw.a1 + 1, hw.a2) << "StepperMotor pins must be consecutive";
  CHECK_EQ(hw.a2 + 1, hw.b1) << "StepperMotor pins must be consecutive";
  CHECK_EQ(hw.b1 + 1, hw.b2) << "StepperMotor pins must be consecutive";

  CHECK(pio_can_add_program(pio, &stepper_program));
  int offset = pio_add_program(pio, &stepper_program);
  int sm = pio_claim_unused_sm(pio, false);
  LOG_IF(FATAL, sm < 0) << "No free state machines in PIO"
                        << pio_get_index(pio);
  pio_sm_config c = stepper_program_get_default_config(offset);

  pio_gpio_init(pio, hw.a1);
  pio_gpio_init(pio, hw.a2);
  pio_gpio_init(pio, hw.b1);
  pio_gpio_init(pio, hw.b2);
  pio_sm_set_consecutive_pindirs(pio, sm, hw.a1, 4, /*is_out=*/true);
  sm_config_set_sideset_pins(&c, hw.a1);
  sm_config_set_out_shift(&c, /*shift_right=*/true, /*autopull=*/true,
                          /*pull_threshold=*/32);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  // Use the fastest possible PIO clock speed.
  sm_config_set_clkdiv_int_frac(&c, 1, 0);

  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_enabled(pio, sm, true);

  int sys_hz = clock_get_hz(clk_sys);
  int pwm_period = sys_hz / (kCyclesPerPwmCount * hw.pwm_freq_hz);
  LOG(INFO) << "Stepper motor PWM period set to " << pwm_period
            << " counts at PIO clock " << sys_hz << "Hz";

  pio->txf[sm] = pwm_period;
  busy_wait_at_least_cycles(8);
  CHECK_EQ(pio_sm_get_pc(pio, sm), offset + stepper_offset_new_command);

  StepperMotor* self = CHECK_NOTNULL(new StepperMotor());
  self->pio_ = pio;
  self->hw_ = hw;
  self->sm_ = sm;
  self->offset_ = offset;
  self->pwm_period_ = pwm_period;
  self->InitCommands();
  return self;
}

StepperMotor::StepperMotor() {}

void StepperMotor::Move(int32_t count) {
  //
}

void StepperMotor::Stop(bool brake) {
  //
}

uint32_t StepperMotor::make_command(int phase, uint16_t pwm_value) {
  return (pio_encode_jmp(offset_ + stepper_offset_phase[phase]) << 16) |
         pwm_value;
}

void StepperMotor::InitCommands() {
  for (int phase = 0; phase < 8; ++phase) {
    if (phase % 2) {
      commands_.push_back(make_command(phase, 0));
      commands_.push_back(make_command(phase, pwm_period_ / 2));
      commands_.push_back(make_command(phase, pwm_period_));
    } else {
      // even phases are not PWM; just side-set and jump to new_command
      uint bits = 0b1111;
      switch (phase) {
        case 0:
          bits = 0b0111;
          break;
        case 2:
          bits = 0b1101;
          break;
        case 4:
          bits = 0b1011;
          break;
        case 6:
          bits = 0b1110;
          break;
      }
      commands_.push_back(
          (pio_encode_jmp(offset_ + stepper_offset_new_command) |
           pio_encode_sideset_opt(4, bits))
          << 16);
    }
  }
}

void StepperMotor::RunPioTest() {
  static size_t n = 0;
  LOG(INFO) << "send command " << n;
  pio_->txf[sm_] = commands_[n++];
  if (n == commands_.size()) {
    n = 0;
  }
}

}  // namespace tplp