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

}  // namespace

std::optional<int> StepperMotor::pio0_program_offset_ = std::nullopt;
std::optional<int> StepperMotor::pio1_program_offset_ = std::nullopt;
bool StepperMotor::static_init_done_ = false;

uint16_t StepperMotor::pwm_period_ = 0;
int StepperMotor::pio_clkdiv_int_ = 0;
int StepperMotor::pio_clkdiv_frac_ = 0;
int StepperMotor::pio_hz_ = 0;
uint32_t StepperMotor::shortbrake_command_ = 0;
uint32_t StepperMotor::commands_[kCommandBufLen] = {};

StepperMotor::StepperMotor() {}

void StepperMotor::Stop() {
  stride_ = 0;
  *txf_ = commands_[command_index_];
}

void StepperMotor::Release() {
  stride_ = 0;
  *txf_ = shortbrake_command_;
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