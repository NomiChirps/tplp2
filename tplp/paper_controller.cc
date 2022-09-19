#include "tplp/paper_controller.h"

#include <iomanip>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "pico/time.h"
#include "picolog/picolog.h"
#include "tplp/alarm_irq.h"
#include "tplp/config/params.h"

TPLP_PARAM(int32_t, loadcell_offset, 100'000, "Load cell zeroing offset");
TPLP_PARAM(int32_t, loadcell_scale, 200, "Load cell scaling divider");

TPLP_PARAM(int32_t, tension_speed, 1'000,
           "Motor speed to use during paper tensioning");
TPLP_PARAM(int32_t, tension_timeout_ms, 2'000,
           "Timeout if paper tension can't be established");
// TODO: make these polarity params bools instead of ints?
TPLP_PARAM(
    int32_t, motor_polarity_src, 1,
    "Which feed direction (forward or reverse) is positive for the motor");
TPLP_PARAM(
    int32_t, motor_polarity_dst, -1,
    "Which feed direction (forward or reverse) is positive for the motor");

TPLP_PARAM(int32_t, max_load_noise, 10, "????");  // XXX what?

TPLP_PARAM(int32_t, target_tension, 100,
           "Target loadcell value to maintain by tensioning the paper");
TPLP_PARAM(int32_t, panic_stop_tension, 200,
           "Abort print and stop feeding if tension reaches this level, to "
           "avoid tearing the paper");
TPLP_PARAM(int32_t, paper_timer_delay_us, 1000,
           "How often to run the PaperController interrupt, in microseconds");

namespace tplp {
namespace {

enum TaskNotificationBits : uint32_t {
  CMD_TENSION = 0x1,
  CMD_START_FEED = 0x2,
  CMD_STOP_FEED = 0x4,
  CMD_RELEASE = 0x8,
};

static PaperController* instance = nullptr;
static constexpr int kAlarmNum = 2;

int __not_in_flash_func(GetDelayFn)() {
  return PARAM_paper_timer_delay_us.Get();
}

void __not_in_flash_func(IsrBodyFn)() {
  // TODO: do stuff based on instance->state_.
}

using MyTimer = PeriodicAlarm<kAlarmNum, IsrBodyFn, GetDelayFn>;

}  // namespace

template __not_in_flash("PeriodicAlarm") void PeriodicAlarm<
    kAlarmNum, IsrBodyFn, GetDelayFn>::ISR();

PaperController::PaperController(HX711* loadcell, StepperMotor* motor_a,
                                 StepperMotor* motor_b)
    : state_(State::NOT_TENSIONED),
      sys_hz_(clock_get_hz(clk_sys)),
      loadcell_(loadcell),
      motor_src_(motor_a),
      motor_dst_(motor_b) {
  CHECK(!instance);
  instance = this;
}

void PaperController::Init(int task_priority, int stack_size, int alarm_num,
                           uint8_t irq_priority) {
  CHECK(xTaskCreate(&PaperController::TaskFn, "PaperController", stack_size,
                    this, task_priority, &task_));
  LOG(INFO) << (void*)IsrBodyFn;
  LOG(INFO) << (void*)&IsrBodyFn;
  LOG(INFO) << reinterpret_cast<intptr_t>(IsrBodyFn);
  MyTimer::Check();
  CHECK_EQ(alarm_num, kAlarmNum);
  CHECK(!hardware_alarm_is_claimed(alarm_num));
  hardware_alarm_claim(alarm_num);
  int timer_irq = TIMER_IRQ_0 + alarm_num;
  irq_set_exclusive_handler(timer_irq, MyTimer::ISR);
  irq_set_priority(timer_irq, irq_priority);
  irq_set_enabled(timer_irq, true);
  hw_set_bits(&timer_hw->inte, 1u << alarm_num);
  // Start the timer; it will keep running forever.
  hardware_alarm_set_target(
      alarm_num, make_timeout_time_us(PARAM_paper_timer_delay_us.Get()));
  /// XXX Delete me
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

void PaperController::TaskFn(void* task_param) {
  static_cast<PaperController*>(task_param)->TaskFnBody();
}

int32_t PaperController::GetLoadCellValue() const {
  return (loadcell_->raw_value() + PARAM_loadcell_offset.Get()) /
         PARAM_loadcell_scale.Get();
}

int32_t PaperController::GetRawLoadCellValue() const {
  return loadcell_->raw_value();
}

util::Status PaperController::SetFeedRate(linear_velocity_t velocity) {
  return util::UnimplementedError("SetFeedRate");
}

util::Status PaperController::Cmd_Tension() {
  if (state() != State::NOT_TENSIONED) {
    return util::FailedPreconditionError(
        "Cannot tension unless state is NOT_TENSIONED");
  }
  xTaskNotify(task_, TaskNotificationBits::CMD_TENSION, eSetBits);
  return util::OkStatus();
}

util::Status PaperController::Cmd_StartFeed() {
  return util::UnimplementedError("StartFeed");
}

util::Status PaperController::Cmd_StopFeed() {
  return util::UnimplementedError("StopFeed");
}

util::Status PaperController::Cmd_Release() {
  if (state() != State::TENSIONED_IDLE) {
    return util::FailedPreconditionError(
        "Cannot release unless state is TENSIONED_IDLE");
  }
  xTaskNotify(task_, TaskNotificationBits::CMD_RELEASE, eSetBits);
  return util::OkStatus();
}

void PaperController::TaskFnBody() {
  util::Status status;
  for (;;) {
    TaskNotificationBits notify;
    CHECK(xTaskNotifyWait(0, 0xffffffff, reinterpret_cast<uint32_t*>(&notify),
                          portMAX_DELAY));
    if (notify & CMD_TENSION) {
      status = Tension();
      if (!status.ok()) PostError(std::move(status));
      continue;
    }
    if (notify & CMD_RELEASE) {
      status = Release();
      if (!status.ok()) PostError(std::move(status));
      continue;
    }
    LOG(ERROR) << "Unhandled notification bits: 0x" << std::hex
               << std::setfill('0') << std::setw(8) << (int)notify;
  }
}

util::Status PaperController::Tension() {
  LOG(INFO) << "Executing Tension command";
  if (state_ != State::NOT_TENSIONED) {
    return util::FailedPreconditionError("Invalid state for CMD_TENSION");
  }
  motor_src_->Stop(StepperMotor::StopType::SHORT_BRAKE);
  motor_dst_->Stop(StepperMotor::StopType::SHORT_BRAKE);
  motor_dst_->SetSpeedSlow(PARAM_tension_speed.Get());

  if (std::abs(GetLoadCellValue()) > PARAM_max_load_noise.Get()) {
    return util::FailedPreconditionError(
        "Unexpected load. Clear obstruction or recalibrate load cell");
  }

  // Feed the DST motor forward while holding SRC fixed, stretching out the
  // paper between them.
  motor_src_->Stop(StepperMotor::StopType::HOLD);
  motor_dst_->Move(
      PARAM_motor_polarity_dst.Get() *
      (PARAM_tension_speed.Get() * PARAM_tension_timeout_ms.Get()) / 1000);

  // TODO: use the timer interrupt instead
  bool timed_out = true;
  while (motor_dst_->moving()) {
    if (GetLoadCellValue() >= PARAM_target_tension.Get()) {
      motor_dst_->Stop(StepperMotor::StopType::HOLD);
      timed_out = false;
      break;
    }
  }
  if (timed_out) {
    motor_dst_->Stop(StepperMotor::StopType::SHORT_BRAKE);
    return util::FailedPreconditionError(
        "Cannot establish tension; is the paper properly loaded?");
  }

  state_ = State::TENSIONED_IDLE;

  return util::OkStatus();
}

util::Status PaperController::Release() {
  if (state_ != State::TENSIONED_IDLE) {
    return util::FailedPreconditionError("Invalid state for CMD_RELEASE");
  }
  state_ = State::NOT_TENSIONED;
  motor_src_->Stop(StepperMotor::StopType::SHORT_BRAKE);
  motor_dst_->Stop(StepperMotor::StopType::SHORT_BRAKE);
  return util::OkStatus();
}

void PaperController::PostError(util::Status status) {
  LOG(ERROR) << "Paper controller error: " << status;
}

}  // namespace tplp
