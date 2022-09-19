#include "tplp/motor/stepper.h"

#include <numeric>
#include <optional>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "pico/time.h"
#include "picolog/picolog.h"
#include "tplp/motor/sin_table.h"
#include "tplp/motor/stepper.pio.h"
#include "tplp/rtos_util.h"

namespace tplp {
namespace {
uint32_t make_command(uint8_t pwm_period, uint8_t t1, uint8_t t2_t3,
                      uint8_t pins_t2_t3, uint8_t pins_t4) {
  CHECK_GE(pwm_period, t1);
  CHECK_GE(t1, t2_t3);
  return (pwm_period << 0) | (t1 << 8) | (t2_t3 << 16) | (pins_t2_t3 << 24) |
         (pins_t4 << 28);
}

static std::optional<int> pio0_program_offset = std::nullopt;
static std::optional<int> pio1_program_offset = std::nullopt;

// motors[] is indexed by rp2040 hardware alarm number
static constexpr int kMaxNumMotors = NUM_TIMERS;
static StepperMotor* motors[kMaxNumMotors] = {};

}  // namespace

// M is the motor id and also the hardware alarm number
template <int AlarmNum>
[[gnu::hot]] __not_in_flash("StepperMotor") void StepperMotor::timer_isr() {
  static uint64_t target_time = time_us_64();
  timer_hw->intr = 1u << AlarmNum;
  StepperMotor* const motor = motors[AlarmNum];
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

template __not_in_flash("StepperMotor") void StepperMotor::timer_isr<0>();
template __not_in_flash("StepperMotor") void StepperMotor::timer_isr<1>();

template <int AlarmNum>
void StepperMotor::InitTimer(int irq_priority) {
  static_assert(AlarmNum < NUM_TIMERS);
  CHECK(!hardware_alarm_is_claimed(AlarmNum));
  hardware_alarm_claim(AlarmNum);
  motors[AlarmNum] = this;
  timer_alarm_num_ = AlarmNum;
  timer_irq_ = TIMER_IRQ_0 + AlarmNum;
  irq_set_exclusive_handler(timer_irq_, timer_isr<AlarmNum>);
  irq_set_priority(timer_irq_, irq_priority);
  irq_set_enabled(timer_irq_, true);
  hw_set_bits(&timer_hw->inte, 1u << timer_alarm_num_);
  // keep trying to start the timer (in case we're preempted)
  uint64_t target_time;
  do {
    target_time = time_us_64() + timer_interval_us_;
  } while (hardware_alarm_set_target(timer_alarm_num_, {target_time}));
}

template void StepperMotor::InitTimer<0>(int);
template void StepperMotor::InitTimer<1>(int);

bool StepperMotor::static_init_done_ = false;
uint16_t StepperMotor::pwm_period_ = 0;
int StepperMotor::pio_clkdiv_int_ = 0;
int StepperMotor::pio_clkdiv_frac_ = 0;
int StepperMotor::pio_hz_ = 0;
uint32_t StepperMotor::shortbrake_command_ = 0;
uint32_t StepperMotor::commands_[kCommandBufLen] = {};

StepperMotor* StepperMotor::Init(PIO pio, const Hardware& hw, int alarm_num,
                                 int timer_irq_priority) {
  LOG_IF(FATAL, !static_init_done_) << "must call StaticInit() before Init()";
  CHECK_EQ(hw.a1 + 1, hw.a2) << "StepperMotor pins must be consecutive";
  CHECK_EQ(hw.a2 + 1, hw.b1) << "StepperMotor pins must be consecutive";
  CHECK_EQ(hw.b1 + 1, hw.b2) << "StepperMotor pins must be consecutive";

  int program_offset;
  if (pio_get_index(pio) == 0) {
    if (pio0_program_offset.has_value()) {
      program_offset = *pio0_program_offset;
    } else {
      CHECK(pio_can_add_program(pio, &stepper_program))
          << "Not enough free instruction space in PIO" << pio_get_index(pio);
      pio0_program_offset = program_offset =
          pio_add_program(pio, &stepper_program);
    }
  } else if (pio_get_index(pio) == 1) {
    if (pio1_program_offset.has_value()) {
      program_offset = *pio1_program_offset;
    } else {
      CHECK(pio_can_add_program(pio, &stepper_program))
          << "Not enough free instruction space in PIO" << pio_get_index(pio);
      pio1_program_offset = program_offset =
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

  StepperMotor* self = CHECK_NOTNULL(new StepperMotor());
  self->pio_ = pio;
  self->hw_ = hw;
  self->txf_ = &pio->txf[sm];
  self->sm_ = sm;
  self->program_offset_ = program_offset;
  self->command_index_ = 0;
  self->stride_ = 0;
  self->timer_interval_us_ = 1'000;

  CHECK_LT(alarm_num, kMaxNumMotors);
  if (alarm_num == 0) {
    self->InitTimer<0>(timer_irq_priority);
  } else if (alarm_num == 1) {
    self->InitTimer<1>(timer_irq_priority);
  } else {
    LOG(FATAL) << "unexpected num motors";
  }

  // Put a safe initial command into the FIFO before starting the state machine.
  *self->txf_ = self->shortbrake_command_;
  pio_sm_set_enabled(pio, sm, true);

  return self;
}

void StepperMotor::SetSpeed(int32_t stride, int32_t interval_us) {
  VLOG(1) << "SetSpeed(" << stride << ", " << interval_us << ")";
  if (interval_us < min_interval_us()) {
    LOG(ERROR) << "interval_us too small " << interval_us;
    interval_us = min_interval_us();
  }
  taskENTER_CRITICAL();
  stride_ = stride;
  timer_interval_us_ = interval_us;
  taskEXIT_CRITICAL();
}

void StepperMotor::SetStride(int32_t stride) {
  VLOG(1) << "SetStride(" << stride << ")";
  SetStrideFromISR(stride);
}

void StepperMotor::SetInterval(int32_t interval_us) {
  VLOG(1) << "SetInterval(" << interval_us << ")";
  if (interval_us < min_interval_us()) {
    LOG(ERROR) << "interval_us too small";
    interval_us = min_interval_us();
  }
  SetIntervalFromISR(interval_us);
}

StepperMotor::StepperMotor() {}

void StepperMotor::Stop() {
  stride_ = 0;
  *txf_ = commands_[command_index_];
}

void StepperMotor::Release() {
  stride_ = 0;
  *txf_ = shortbrake_command_;
}

void StepperMotor::StaticInit(int pwm_freq_hz) {
  if (static_init_done_) return;
  StaticInit_Clocks(pwm_freq_hz);
  StaticInit_Commands();
  static_init_done_ = true;
}

void StepperMotor::StaticInit_Clocks(int pwm_freq_hz) {
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
  pio_clkdiv_int_ = 1;
  pio_clkdiv_frac_ = 0;
  pwm_period_ = sys_hz / (stepper_instructions_per_count * pwm_freq_hz);
  // System clock too fast?
  if (pwm_period_ > 255) {
    // Set pwm_period to max and calculate an appropriate divider.
    pwm_period_ = 255;
    // 255 = (sys_hz / divider) / (isp * pwm_freq)
    // 255 * isp * pwm_freq * divider = sys_hz
    // divider = sys_hz / (255 * isp * pwm_freq)
    int clkdiv_256 =
        (256 * (sys_hz / 255)) / (stepper_instructions_per_count * pwm_freq_hz);
    VLOG(1) << "sys_hz = " << sys_hz;
    VLOG(1) << "pwm_freq_hz = " << pwm_freq_hz;
    VLOG(1) << "clkdiv_256 = " << clkdiv_256;
    pio_clkdiv_int_ = clkdiv_256 / 256;
    pio_clkdiv_frac_ = clkdiv_256 % 256;
  }

  CHECK_GT(pwm_period_, 0);
  CHECK_LE(pwm_period_, 255);  // 8 bits max
  CHECK_GT(pio_clkdiv_int_, 0);
  CHECK_GE(pio_clkdiv_frac_, 0);
  CHECK_LT(pio_clkdiv_frac_, 256);
  pio_hz_ = 256 * (sys_hz / (256 * pio_clkdiv_int_ + pio_clkdiv_frac_));

  LOG(INFO) << "Stepper motor PWM period set to " << pwm_period_
            << " counts at PIO clock " << pio_hz_ << "Hz (divider "
            << pio_clkdiv_int_ << " + " << pio_clkdiv_frac_
            << "/256). Actual PWM frequency = "
            << pio_hz_ / (stepper_instructions_per_count * pwm_period_) << "Hz";
}

void StepperMotor::StaticInit_Commands() {
  // Make sure command buffer is aligned.
  CHECK(!(reinterpret_cast<uint32_t>(&commands_[0]) & (kCommandBufLen - 1)))
      << "Command buffer array not aligned; address = " << &commands_[0]
      << ". Check compiler settings.";

  // Construct the command buffer.
  // FIXME: check inter-phase timings!
  constexpr int kCommandsPerPhase = kCommandBufLen / 4;
  VLOG(1) << "kBufSize = " << kCommandBufLen;
  VLOG(1) << "kCommandsPerPhase = " << kCommandsPerPhase;
  VLOG(1) << "pwm_period = " << pwm_period_;

  int command_index = 0;
  const auto push = [&command_index](uint8_t t1_duration,
                                     uint8_t t2_t3_duration, uint8_t pins_t2_t3,
                                     uint8_t pins_t4) {
    uint8_t t1 = pwm_period_ - t1_duration;
    uint8_t t2_t3 = t1 - t2_t3_duration;
    uint32_t command =
        make_command(pwm_period_, t1, t2_t3, pins_t2_t3, pins_t4);
    VLOG(2) << "push t1=" << (int)t1 << " t2_t3=" << (int)t2_t3
            << " pins_t2_t3=0x" << std::hex << (int)pins_t2_t3 << " pins_t4=0x"
            << (int)pins_t4 << " command=0x" << command;
    CHECK_LT(command_index, (int)kCommandBufLen);
    commands_[command_index++] = command;
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
    // This corresponds to the full-step cycle:
    //    0b0111
    //    0b1101
    //    0b1011
    //    0b1110
    // TODO: think about an alternate table, defaulting to freewheel instead of
    // short-brake? does this make any sense at all? basically inverted...
    //    0b1000
    //    0b0010
    //    0b0100
    //    0b0001
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
  LOG(INFO) << "StepperMotor command buffer initialized. Size = "
            << kCommandBufLen;

  // Auxiliary commands.
  shortbrake_command_ = make_command(pwm_period_, pwm_period_, 0, 0b1111, 0);
}

}  // namespace tplp