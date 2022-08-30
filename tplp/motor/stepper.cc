#include "tplp/motor/stepper.h"

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/time.h"
#include "picolog/picolog.h"
#include "tplp/motor/cosine_table.h"
#include "tplp/motor/stepper.pio.h"

namespace tplp {
namespace {
uint32_t make_command(uint8_t pwm_period, uint8_t t1, uint8_t t2_t3,
                      uint8_t pins_t2_t3, uint8_t pins_t4) {
  CHECK_GE(pwm_period, t1);
  CHECK_GE(t1, t2_t3);
  return (pwm_period << 0) | (t1 << 8) | (t2_t3 << 16) | (pins_t2_t3 << 24) |
         (pins_t4 << 28);
}
}  // namespace

StepperMotor* StepperMotor::Init(PIO pio, const Hardware& hw) {
  CHECK_EQ(hw.a1 + 1, hw.a2) << "StepperMotor pins must be consecutive";
  CHECK_EQ(hw.a2 + 1, hw.b1) << "StepperMotor pins must be consecutive";
  CHECK_EQ(hw.b1 + 1, hw.b2) << "StepperMotor pins must be consecutive";

  CHECK(pio_can_add_program(pio, &stepper_program))
      << "Not enough free instruction space in PIO" << pio_get_index(pio);
  int offset = pio_add_program(pio, &stepper_program);
  int sm = pio_claim_unused_sm(pio, false);
  LOG_IF(FATAL, sm < 0) << "No free state machines in PIO"
                        << pio_get_index(pio);
  pio_sm_config c = stepper_program_get_default_config(offset);

  pio_gpio_init(pio, hw.a1);
  pio_gpio_init(pio, hw.a2);
  pio_gpio_init(pio, hw.b1);
  pio_gpio_init(pio, hw.b2);
  sm_config_set_out_pins(&c, hw.a1, 4);
  sm_config_set_out_shift(&c, /*shift_right=*/true, /*autopull=*/true,
                          /*pull_threshold=*/32);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  // Use the fastest possible PIO clock speed.
  sm_config_set_clkdiv_int_frac(&c, 1, 0);

  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_enabled(pio, sm, true);
  pio_sm_set_consecutive_pindirs(pio, sm, hw.a1, 4, /*is_out=*/true);

  const int sys_hz = clock_get_hz(clk_sys);
  const int pwm_period =
      sys_hz / (stepper_instructions_per_count * hw.pwm_freq_hz);
  CHECK_NE(pwm_period, 0);

  LOG(INFO) << "Stepper motor PWM period set to " << pwm_period
            << " counts at PIO clock " << sys_hz
            << "Hz. Actual PWM frequency = "
            << sys_hz / (stepper_instructions_per_count * pwm_period)
            << "Hz";

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

void StepperMotor::InitCommands() {
  // We need the size of the table to be a power of 2 so ring DMA works.
  // TODO: consider increasing :)
  constexpr size_t kBufSize = 128;
  commands_.clear();
  commands_.reserve(kBufSize);
  for (int phase = 0; phase < 8; ++phase) {
    LOG(FATAL) << "Not implemented";
  }
  CHECK_EQ(kBufSize, commands_.size());
  LOG(INFO) << "StepperMotor command buffer initialized. Size = "
            << commands_.size();
}

void StepperMotor::RunPioTest() {
  for (int j = 0; j < 10; ++j) {
    for (size_t i = 0; i < commands_.size(); ++i) {
      pio_->txf[sm_] = commands_[i];
      vTaskDelay(2);
    }
    pio_->txf[sm_] = commands_[0];
  }
}

}  // namespace tplp