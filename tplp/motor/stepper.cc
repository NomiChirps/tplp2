#include "tplp/motor/stepper.h"

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/time.h"
#include "picolog/picolog.h"
#include "tplp/motor/sin_table.h"
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
  sm_config_set_set_pins(&c, hw.a1, 4);
  sm_config_set_out_pins(&c, hw.a1, 4);
  sm_config_set_out_shift(&c, /*shift_right=*/true, /*autopull=*/false,
                          /*pull_threshold=*/32);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

  // Variables:
  // Fpio : pio clock
  // N    : pwm period
  //
  // Constants:
  // Fpwm : desired pwm frequency
  //
  // We want the highest possible clock while keeping N < 256.
  // i.e. the smallest possible clock divider. fractional is okay.
  const int sys_hz = clock_get_hz(clk_sys);
  int clkdiv_int = 1;
  int clkdiv_frac = 0;
  int pwm_period = sys_hz / (stepper_instructions_per_count * hw.pwm_freq_hz);
  // System clock too fast?
  if (pwm_period > 255) {
    // Set pwm_period to max and calculate an appropriate divider.
    pwm_period = 255;
    // 255 = (sys_hz / divider) / (isp * pwm_freq)
    // 255 * isp * pwm_freq * divider = sys_hz
    // divider = sys_hz / (255 * isp * pwm_freq)
    int clkdiv_256 = (256 * (sys_hz / 255)) /
                     (stepper_instructions_per_count * hw.pwm_freq_hz);
    VLOG(1) << "sys_hz = " << sys_hz;
    VLOG(1) << "pwm_freq_hz = " << hw.pwm_freq_hz;
    VLOG(1) << "clkdiv_256 = " << clkdiv_256;
    clkdiv_int = clkdiv_256 / 256;
    clkdiv_frac = clkdiv_256 % 256;
  }

  CHECK_GT(pwm_period, 0);
  CHECK_LE(pwm_period, 255);  // 8 bits max
  CHECK_GT(clkdiv_int, 0);
  CHECK_GE(clkdiv_frac, 0);
  CHECK_LT(clkdiv_frac, 256);
  sm_config_set_clkdiv_int_frac(&c, clkdiv_int, clkdiv_frac);
  const int pio_hz = 256 * (sys_hz / (256 * clkdiv_int + clkdiv_frac));

  LOG(INFO) << "Stepper motor PWM period set to " << pwm_period
            << " counts at PIO clock " << pio_hz << "Hz (divider " << clkdiv_int
            << " + " << clkdiv_frac << "/256). Actual PWM frequency = "
            << pio_hz / (stepper_instructions_per_count * pwm_period) << "Hz";

  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_consecutive_pindirs(pio, sm, hw.a1, 4, /*is_out=*/true);
  // Put initial command into the FIFO before starting the state machine.
  // This brings all pins high and keeps them there.
  pio->txf[sm] = make_command(pwm_period, pwm_period, 0, 0b1111, 0);
  pio_sm_set_enabled(pio, sm, true);

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
  // FIXME: check inter-phase timings!
  constexpr size_t kBufSize = 128;
  constexpr int kCommandsPerPhase = kBufSize / 4;
  commands_.clear();
  commands_.reserve(kBufSize);
  VLOG(1) << "kBufSize = " << kBufSize;
  VLOG(1) << "kCommandsPerOddPhase = " << kCommandsPerPhase;
  VLOG(1) << "pwm_period = " << pwm_period_;

  const auto push = [this](uint8_t t1_duration, uint8_t t2_t3_duration,
                           uint8_t pins_t2_t3, uint8_t pins_t4) {
    uint8_t t1 = pwm_period_ - t1_duration;
    uint8_t t2_t3 = t1 - t2_t3_duration;
    uint32_t command =
        make_command(pwm_period_, t1, t2_t3, pins_t2_t3, pins_t4);
    VLOG(2) << "push t1=" << (int)t1 << " t2_t3=" << (int)t2_t3
            << " pins_t2_t3=0x" << std::hex << (int)pins_t2_t3 << " pins_t4=0x"
            << (int)pins_t4 << " command=0x" << command;
    commands_.push_back(command);
  };
  const uint16_t kSinMax = 65535;
  const auto sine = [](uint8_t command_index) -> uint16_t {
    int i = (command_index * kSinTableSize) / kCommandsPerPhase;
    if (i == kSinTableSize) return kSinMax;
    return kSinTable[i];
  };
  for (int phase = 0; phase < 4; ++phase) {
    // pins_t1 == 0b1111
    uint8_t pins_t2 = 0;  // first half of period
    uint8_t pins_t3 = 0;  // second half of period
    bool polarity = 0;
    switch (phase) {
      case 0:
        pins_t2 = 0b0111;
        pins_t3 = 0b1101;
        polarity = 0;
        break;
      case 1:
        pins_t2 = 0b1101;
        pins_t3 = 0b1011;
        polarity = 1;
        break;
      case 2:
        pins_t2 = 0b1011;
        pins_t3 = 0b1110;
        polarity = 0;
        break;
      case 3:
        pins_t2 = 0b1110;
        pins_t3 = 0b0111;
        polarity = 1;
        break;
    }
    // remainder of period, both low
    uint8_t pins_t4 = pins_t2 & pins_t3;
    // weow...
    for (uint8_t i = 0; i < kCommandsPerPhase; ++i) {
      // duty cycle for A and B pins
      int da = (pwm_period_ * (kSinMax - sine(i))) / kSinMax;
      int db =
          (pwm_period_ * (kSinMax - sine(kCommandsPerPhase - i))) / kSinMax;
      if (polarity) std::swap(da, db);
      int t1_duration = std::min(da, db);
      int t2_t3_duration = std::abs(da - db);
      CHECK_GE(t1_duration, 0);
      CHECK_LT(t1_duration, 256);
      CHECK_GE(t2_t3_duration, 0);
      CHECK_LT(t2_t3_duration, 256);
      VLOG(1) << "phase=" << phase << " i=" << (int)i << " da=" << da
              << " db=" << db << " t1_duration=" << t1_duration
              << " t2_t3_duration=" << t2_t3_duration;
      push(t1_duration, t2_t3_duration,
           (i < (kCommandsPerPhase / 2)) ? pins_t2 : pins_t3, pins_t4);
    }
  }
  CHECK_EQ(kBufSize, commands_.size());
  LOG(INFO) << "StepperMotor command buffer initialized. Size = "
            << commands_.size();
}

void StepperMotor::RunPioTest() {
  for (size_t i = 0; i < commands_.size(); ++i) {
    if (pio_sm_is_tx_fifo_full(pio_, sm_)) {
      LOG(ERROR) << "tx fifo full";
      return;
    }
    VLOG(1) << "i=" << i << " command=0x" << std::hex << commands_[i];
    pio_->txf[sm_] = commands_[i];
    vTaskDelay(5);
  }
  Stop(1);
}

}  // namespace tplp