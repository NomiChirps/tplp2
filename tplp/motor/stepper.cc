#include "tplp/motor/stepper.h"

#include <numeric>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
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

static constexpr int kMaxNumMotors = 2;
static int num_motors = 0;
static StepperMotor* motors[kMaxNumMotors] = {};

}  // namespace

// M is the motor id and also the hardware alarm number
template <int M>
[[gnu::hot]] void StepperMotor::timer_isr() {
  timer_hw->intr = 1u << M;
  StepperMotor* const motor = motors[M];
  while (motor->timer_steps_remaining_) {
    *motor->txf_ = motor->commands_[(motor->offset_after_move_ -
                                     motor->timer_steps_remaining_) %
                                    kCommandBufLen];
    if (!--motor->timer_steps_remaining_) [[unlikely]]
      break;
    motor->timer_target_us_ =
        delayed_by_us(motor->timer_target_us_, motor->timer_delay_us_);
    if (hardware_alarm_set_target(M, motor->timer_target_us_)) {
      // missed the next target; take another step immediately and try again
      continue;
    } else {
      // next target is in the future & alarm is armed. let's go
      break;
    }
  }
}

template void StepperMotor::timer_isr<0>();
template void StepperMotor::timer_isr<1>();

template <int M>
void StepperMotor::InitTimer(int irq_priority) {
  static_assert(M < 4);
  CHECK(!hardware_alarm_is_claimed(M));
  hardware_alarm_claim(M);
  timer_alarm_num_ = M;
  timer_irq_ = TIMER_IRQ_0 + M;
  irq_set_exclusive_handler(timer_irq_, timer_isr<M>);
  irq_set_priority(timer_irq_, irq_priority);
  irq_set_enabled(timer_irq_, true);
  hw_set_bits(&timer_hw->inte, 1u << timer_alarm_num_);
}

template void StepperMotor::InitTimer<0>(int);
template void StepperMotor::InitTimer<1>(int);

bool StepperMotor::static_init_done_ = false;
uint16_t StepperMotor::pwm_period_ = 0;
int StepperMotor::pio_clkdiv_int_ = 0;
int StepperMotor::pio_clkdiv_frac_ = 0;
int StepperMotor::pio_hz_ = 0;
uint32_t StepperMotor::shortbrake_command_ = 0;
uint32_t StepperMotor::fwd_commands_[kCommandBufLen] = {};
uint32_t StepperMotor::rev_commands_[kCommandBufLen] = {};

StepperMotor* StepperMotor::Init(DmaController* dma, PIO pio,
                                 const Hardware& hw, int timer_irq_priority) {
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

  int dma_timer_num = dma_claim_unused_timer(false);
  CHECK_GE(dma_timer_num, 0) << "No free DMA timer available";
  LOG(INFO) << "Claimed DMA timer " << dma_timer_num;

  StepperMotor* self = CHECK_NOTNULL(new StepperMotor(dma));
  self->pio_ = pio;
  self->hw_ = hw;
  self->txf_ = &pio->txf[sm];
  self->sm_ = sm;
  self->program_offset_ = program_offset;
  self->dma_mode_ = true;
  self->dma_timer_num_ = dma_timer_num;
  self->dma_timer_dreq_ = dma_get_timer_dreq(dma_timer_num);
  // timer values don't matter in DMA mode
  self->timer_delay_us_ = 0;
  self->timer_steps_remaining_ = 0;
  self->timer_target_us_ = get_absolute_time();

  CHECK_LT(num_motors, kMaxNumMotors);
  if (num_motors == 0) {
    self->InitTimer<0>(timer_irq_priority);
  } else if (num_motors == 1) {
    self->InitTimer<1>(timer_irq_priority);
  } else {
    LOG(FATAL) << "unexpected num motors";
  }
  motors[num_motors++] = self;

  ClockDivider clkdiv;
  CHECK(ComputeClockDivider(clock_get_hz(clk_sys), self->microstep_hz_dma_min(),
                            &clkdiv));
  self->SetDmaSpeed(clkdiv);

  LOG(INFO) << "StepperMotor microstep_hz_min = "
            << self->microstep_hz_dma_min()
            << ", microstep_hz_max = " << self->microstep_hz_dma_max();

  // Put a safe initial command into the FIFO before starting the state machine.
  self->SendImmediateCommand(self->shortbrake_command_);
  pio_sm_set_enabled(pio, sm, true);

  return self;
}

void StepperMotor::SetSpeedSlow(uint32_t microstep_hz) {
  VLOG(1) << "SetSpeedSlow( " << microstep_hz << " )";
  static const uint32_t sys_hz = clock_get_hz(clk_sys);
  if (microstep_hz < microstep_hz_dma_min()) {
    SetTimerSpeed(1'000'000 / microstep_hz);
  } else {
    ClockDivider clkdiv;
    CHECK(ComputeClockDivider(sys_hz, microstep_hz, &clkdiv));
    SetDmaSpeed(clkdiv);
  }
}

void StepperMotor::SetDmaSpeed(ClockDivider clkdiv) {
  VLOG(1) << "SetDmaSpeed( " << clkdiv.num << " / " << clkdiv.den << " ) ~ "
          << clkdiv.num * (clock_get_hz(clk_sys) / clkdiv.den);
  dma_timer_set_fraction(dma_timer_num_, clkdiv.num, clkdiv.den);
  if (!dma_mode_) {
    uint32_t n = AbortTimer();
    dma_mode_ = true;
    if (n) {
      // Start DMA to handle the remaining steps.
      StartDma(n);
    }
  }
}

void StepperMotor::SetTimerSpeed(uint32_t us_per_microstep) {
  VLOG(1) << "SetTimerSpeed( " << us_per_microstep << " )";
  CHECK_GT(us_per_microstep, 0u);
  timer_delay_us_ = us_per_microstep;
  if (dma_mode_) {
    uint32_t n = AbortDma();
    dma_mode_ = false;
    if (n) {
      // Start timer to handle the remaining steps.
      StartTimer(n);
    }
  }
}

uint32_t StepperMotor::microstep_hz_dma_min() {
  static const uint32_t sys_hz = clock_get_hz(clk_sys);
  // Clock divider on the DMA timer is 16 bits.
  return sys_hz / 65535 + (sys_hz / 65535 ? 1 : 0);
}

uint32_t StepperMotor::microstep_hz_dma_max() {
  // DMA can send commands as fast as 1 per system clock cycle,
  // but the PIO program is much slower and only checks for new
  // commands between PWM periods.
  return pio_hz_ / (stepper_instructions_per_count * pwm_period_);
}

inline void StepperMotor::SendImmediateCommand(uint32_t cmd) { *txf_ = cmd; }

StepperMotor::StepperMotor(DmaController* dma)
    : commands_(fwd_commands_),
      dma_(dma),
      // Initialize with an arbitrary dummy transfer.
      current_dma_handle_(dma->Transfer({
          .c0_enable = true,
          .c0_read_addr = &offset_after_move_,
          .c0_write_addr = &offset_after_move_,
          .c0_trans_count = 1,
      })) {
  while (!current_dma_handle_.finished()) tight_loop_contents();
  offset_after_move_ = 0;
}

void StepperMotor::StartDma(uint32_t microsteps) {
  CHECK(current_dma_handle_.finished());
  current_dma_handle_ = dma_->Transfer({
      .c0_enable = true,
      .c0_treq_sel = dma_timer_dreq_,
      .c0_read_addr = &commands_[command_index_after_move()],
      .c0_read_incr = true,
      .c0_write_addr = txf_,
      .c0_write_incr = false,
      .c0_data_size = DmaController::DataSize::k32,
      .c0_trans_count = microsteps,
      .c0_ring_size = kCommandBufRingSizeBits,
      .c0_ring_sel = false,
  });
}

void StepperMotor::StartTimer(uint32_t microsteps) {
  CHECK_EQ(timer_steps_remaining_, 0u);
  timer_steps_remaining_ = microsteps;
  // keep trying to start the timer (in case we're preempted)
  do {
    timer_target_us_ = make_timeout_time_us(timer_delay_us_);
  } while (hardware_alarm_set_target(timer_alarm_num_, timer_target_us_));
}

void StepperMotor::Move(int32_t microsteps) {
  VLOG(1) << "StepperMotor::Move(" << microsteps << ")";
  if (microsteps == 0) return;
  if (moving()) {
    LOG(ERROR) << "Move(" << microsteps
               << ") failed: a move is already in progress";
    return;
  }
  if (VLOG_IS_ON(1)) {
    // clear TXOVER debug register
    pio_->fdebug = 1 << (16 + sm_);
  }

  if (microsteps >= 0) {
    if (commands_ == rev_commands_) {
      offset_after_move_ =
          kCommandBufLen -
          2 * (offset_after_move_ % (intdiv_ceil(kCommandBufLen, 2u)));
    }
    commands_ = fwd_commands_;
  } else {
    if (commands_ == fwd_commands_) {
      offset_after_move_ =
          kCommandBufLen -
          2 * (offset_after_move_ % (intdiv_ceil(kCommandBufLen, 2u)));
    }
    commands_ = rev_commands_;
  }

  if (dma_mode_) {
    StartDma(std::abs(microsteps));
  } else {
    StartTimer(std::abs(microsteps));
  }

  offset_after_move_ += std::abs(microsteps);

  if (VLOG_IS_ON(1)) {
    LOG_IF(ERROR, pio_->fdebug & (1 << (16 + sm_)))
        << "TX FIFO overflow during move";
  }
  VLOG(1) << "Move done";
}

uint32_t StepperMotor::AbortDma() {
  uint32_t usteps_remaining = current_dma_handle_.Abort()[0];
  offset_after_move_ -= usteps_remaining;
  return usteps_remaining;
}

uint32_t StepperMotor::AbortTimer() {
  irq_set_enabled(timer_irq_, false);
  hardware_alarm_cancel(timer_alarm_num_);
  uint32_t n = timer_steps_remaining_;
  offset_after_move_ -= n;
  timer_steps_remaining_ = 0;
  irq_set_enabled(timer_irq_, true);
  return n;
}

void StepperMotor::Stop(StopType type) {
  if (dma_mode_) {
    AbortDma();
  } else {
    AbortTimer();
  }
  switch (type) {
    case StopType::HOLD:
      SendImmediateCommand(commands_[command_index_after_move()]);
      break;
    case StopType::SHORT_BRAKE:
      SendImmediateCommand(shortbrake_command_);
      break;
  }
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
  CHECK(!(reinterpret_cast<uint32_t>(&fwd_commands_[0]) &
          ((1 << kCommandBufRingSizeBits) - 1)))
      << "Command buffer array not aligned; address = " << &fwd_commands_[0]
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
    fwd_commands_[command_index++] = command;
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

  // Reverse commands.
  std::reverse_copy(fwd_commands_, fwd_commands_ + kCommandBufLen,
                    rev_commands_);
}

}  // namespace tplp