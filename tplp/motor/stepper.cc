#include "tplp/motor/stepper.h"

#include <numeric>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
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

bool StepperMotor::static_init_done_ = false;
uint16_t StepperMotor::pwm_period_ = 0;
int StepperMotor::pio_clkdiv_int_ = 0;
int StepperMotor::pio_clkdiv_frac_ = 0;
int StepperMotor::pio_hz_ = 0;
uint32_t StepperMotor::shortbrake_command_ = 0;
uint32_t StepperMotor::commands_[kCommandBufLen] = {};

StepperMotor* StepperMotor::Init(DmaController* dma, PIO pio,
                                 const Hardware& hw) {
  StaticInit();
  CHECK_EQ(hw.a1 + 1, hw.a2) << "StepperMotor pins must be consecutive";
  CHECK_EQ(hw.a2 + 1, hw.b1) << "StepperMotor pins must be consecutive";
  CHECK_EQ(hw.b1 + 1, hw.b2) << "StepperMotor pins must be consecutive";

  CHECK(pio_can_add_program(pio, &stepper_program))
      << "Not enough free instruction space in PIO" << pio_get_index(pio);
  int program_offset = pio_add_program(pio, &stepper_program);
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
  self->dma_timer_num_ = dma_timer_num;
  self->dma_timer_dreq_ = dma_get_timer_dreq(dma_timer_num);

  ClockDivider clkdiv;
  CHECK(self->CalculateClockDivider(self->microstep_hz_min(), &clkdiv));
  self->SetSpeed(clkdiv);

  LOG(INFO) << "StepperMotor microstep_hz_min = " << self->microstep_hz_min()
            << ", microstep_hz_max = " << self->microstep_hz_max();

  // Put a safe initial command into the FIFO before starting the state machine.
  self->SendImmediateCommand(self->shortbrake_command_);
  pio_sm_set_enabled(pio, sm, true);

  return self;
}

void StepperMotor::SetSpeed(ClockDivider clkdiv) {
  VLOG(1) << "SetSpeed( " << clkdiv.num << " / " << clkdiv.den << " ) ~ "
          << clkdiv.num * (clock_get_hz(clk_sys) / clkdiv.den);
  dma_timer_set_fraction(dma_timer_num_, clkdiv.num, clkdiv.den);
}

bool StepperMotor::CalculateClockDivider(int microstep_hz,
                                         ClockDivider* out_clkdiv) {
  constexpr int kMaxCf = 8;
  const int sys_hz = clock_get_hz(clk_sys);
  if (microstep_hz >= sys_hz) {
    LOG(ERROR) << "Cannot step faster than the system clock. Requested "
                  "microstep_hz = "
               << microstep_hz;
    return false;
  }
  // y * sys_hz / x = microstep_hz
  // y / x = microstep_hz / sys_hz
  // NB: we'll be calculating stuff with x and y swapped
  //     so that the fraction is greater than 1.
  const int y = microstep_hz;
  const int x = sys_hz;
  // we need the closest rational approximation to x/y
  // such that both the numerator and the denominator
  // fit into a uint16_t. strap in we're gonna do math.
  // continued fraction expansion of x/y:
  int cf[kMaxCf];
  int cf_len = 0;
  int num = x;
  int den = y;
  for (; cf_len < kMaxCf;) {
    int whole = num / den;
    int remainder = num % den;
    cf[cf_len++] = whole;
    VLOG(1) << "cf[" << cf_len - 1 << "] = " << whole;
    if (!remainder) break;
    num = den;
    den = remainder;
  }
  CHECK_GT(cf_len, 0);
  // calculate rational approximations from right to left
  int nds[cf_len][2];
  nds[cf_len - 1][0] = cf[0];
  nds[cf_len - 1][1] = 1;
  if (nds[cf_len - 1][0] > std::numeric_limits<uint16_t>::max() ||
      nds[cf_len - 1][1] > std::numeric_limits<uint16_t>::max()) {
    // the necessary clock divider is too large and can't be represented.
    LOG(WARNING)
        << "Cannot step slower than sys_clk/65535. Requested microstep_hz = "
        << microstep_hz;
    return false;
  }
  VLOG(1) << x << " / " << y << " ~ " << nds[cf_len - 1][0] << " / "
          << nds[cf_len - 1][1];
  bool found = false;
  int approx_num;
  int approx_den;
  for (int i = cf_len - 2; i >= 0; --i) {
    nds[i][0] = cf[i] * nds[i + 1][0] + nds[i + 1][1];
    nds[i][1] = nds[i + 1][0];
    VLOG(1) << x << " / " << y << " ~ " << nds[i][0] << " / " << nds[i][1];
    // is this one too big?
    if (nds[i][0] > std::numeric_limits<uint16_t>::max() ||
        nds[i][1] > std::numeric_limits<uint16_t>::max()) {
      // use the previous one
      approx_num = nds[i + 1][0];
      approx_den = nds[i + 1][1];
      found = true;
      break;
    }
  }
  if (!found) {
    // cool, all of the approximations will fit;
    // use the closest one
    approx_num = nds[0][0];
    approx_den = nds[0][1];
  }
  CHECK_GT(approx_num, 0);
  CHECK_GT(approx_den, 0);

  // See comment at declaration of x and y.
  std::swap(approx_num, approx_den);
  VLOG(1) << "CalculateClockDivider(" << microstep_hz << ") == " << approx_num
          << " / " << approx_den << "; error ~ "
          << (approx_num * (sys_hz / approx_den)) - microstep_hz << "Hz";
  out_clkdiv->num = approx_num;
  out_clkdiv->den = approx_den;
  return true;
}

uint32_t StepperMotor::microstep_hz_min() {
  const uint32_t sys_hz = clock_get_hz(clk_sys);
  // Clock divider on the DMA timer is 16 bits.
  return sys_hz / 65535 + (sys_hz / 65535 ? 1 : 0);
}

uint32_t StepperMotor::microstep_hz_max() {
  // DMA can send commands as fast as 1 per system clock cycle,
  // but the PIO program is much slower and only checks for new
  // commands between PWM periods.
  return pio_hz_ / (stepper_instructions_per_count * pwm_period_);
}

inline void StepperMotor::SendImmediateCommand(uint32_t cmd) { *txf_ = cmd; }

StepperMotor::StepperMotor(DmaController* dma)
    : dma_(dma),
      // Initialize with an arbitrary dummy transfer.
      current_move_handle_(dma->Transfer({
          .c0_enable = true,
          .c0_read_addr = &offset_after_move_,
          .c0_write_addr = &offset_after_move_,
          .c0_trans_count = 1,
      })) {
  while (!current_move_handle_.finished()) tight_loop_contents();
  offset_after_move_ = 0;
}

void StepperMotor::Move(int32_t microsteps) {
  VLOG(1) << "StepperMotor::Move(" << microsteps << ")";
  if (microsteps == 0) return;
  if (microsteps < 0) {
    LOG(ERROR) << "Reverse moves not implemented yet";
    return;
  }
  if (!current_move_handle_.finished()) {
    LOG(ERROR) << "Move(" << microsteps
               << ") failed: a move is already in progress";
    return;
  }
  if (VLOG_IS_ON(1)) {
    // clear TXOVER debug register
    pio_->fdebug = 1 << (16 + sm_);
  }

  current_move_handle_ = dma_->Transfer({
      .c0_enable = true,
      .c0_treq_sel = dma_timer_dreq_,
      .c0_read_addr = &commands_[command_index_after_move()],
      .c0_read_incr = true,
      .c0_write_addr = txf_,
      .c0_write_incr = false,
      .c0_data_size = DmaController::DataSize::k32,
      // XXX: positive count?
      .c0_trans_count = static_cast<uint32_t>(microsteps),
      .c0_ring_size = kCommandBufRingSizeBits,
      .c0_ring_sel = false,
  });
  offset_after_move_ += microsteps;

  if (VLOG_IS_ON(1)) {
    LOG_IF(ERROR, pio_->fdebug & (1 << (16 + sm_)))
        << "TX FIFO overflow during move";
  }
  VLOG(1) << "Move done";
}

// FIXME: not tested!
void StepperMotor::SimultaneousMove(StepperMotor* a, int32_t microsteps_a,
                                    StepperMotor* b, int32_t microsteps_b) {
  VLOG(1) << "StepperMotor::SimultaneoussMove(" << microsteps_a << ", "
          << microsteps_b << ")";
}

void StepperMotor::Stop(StopType type) {
  if (current_move_handle_.started()) {
    // Abort DMA and get the remaining step count.
    uint32_t usteps_remaining = current_move_handle_.Abort()[0];
    offset_after_move_ -= usteps_remaining;
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

void StepperMotor::StaticInit() {
  if (static_init_done_) return;
  StaticInit_Clocks();
  StaticInit_Commands();
  static_init_done_ = true;
}

void StepperMotor::StaticInit_Clocks() {
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
  pwm_period_ = sys_hz / (stepper_instructions_per_count * kPwmFreqHz);
  // System clock too fast?
  if (pwm_period_ > 255) {
    // Set pwm_period to max and calculate an appropriate divider.
    pwm_period_ = 255;
    // 255 = (sys_hz / divider) / (isp * pwm_freq)
    // 255 * isp * pwm_freq * divider = sys_hz
    // divider = sys_hz / (255 * isp * pwm_freq)
    int clkdiv_256 =
        (256 * (sys_hz / 255)) / (stepper_instructions_per_count * kPwmFreqHz);
    VLOG(1) << "sys_hz = " << sys_hz;
    VLOG(1) << "pwm_freq_hz = " << kPwmFreqHz;
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
  CHECK(!(reinterpret_cast<uint32_t>(&commands_[0]) &
          ((1 << kCommandBufRingSizeBits) - 1)))
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