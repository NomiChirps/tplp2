#include "tplp/paper_controller.h"

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "hardware/clocks.h"
#include "picolog/picolog.h"
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

namespace tplp {
namespace {

enum TaskNotificationBits : uint32_t {
  CMD_TENSION = 0x1,
  CMD_START_FEED = 0x2,
  CMD_STOP_FEED = 0x4,
  CMD_RELEASE = 0x8,
};

}  // namespace

PaperController::PaperController(HX711* loadcell, StepperMotor* motor_a,
                                 StepperMotor* motor_b)
    : state_(State::NOT_TENSIONED),
      sys_hz_(clock_get_hz(clk_sys)),
      loadcell_(loadcell),
      motor_src_(motor_a),
      motor_dst_(motor_b) {}

void PaperController::Init(int task_priority, int stack_size) {
  CHECK(xTaskCreate(&PaperController::TaskFn, "PaperController", stack_size,
                    this, task_priority, &task_));
}

void PaperController::TaskFn(void* task_param) {
  static_cast<PaperController*>(task_param)->TaskFn();
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

void PaperController::TaskFn() {
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
  }
}

util::Status PaperController::Tension() {
  LOG(INFO) << "Executing Tension command";
  if (state_ != State::NOT_TENSIONED) {
    return util::FailedPreconditionError("Invalid state for CMD_TENSION");
  }
  motor_src_->Stop(StepperMotor::StopType::SHORT_BRAKE);
  motor_dst_->Stop(StepperMotor::StopType::SHORT_BRAKE);

  ClockDivider clkdiv;
  if (!ComputeClockDivider(sys_hz_, PARAM_tension_speed.Get(), &clkdiv)) {
    return util::InvalidArgumentError("tension_speed out of range");
  }
  motor_dst_->SetSpeed(clkdiv);

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

  // TODO: use a timer interrupt or something instead
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
  // TODO: launch the PID loop
}

void PaperController::PostError(util::Status status) {
  LOG(ERROR) << "Paper controller error: " << status;
}

}  // namespace tplp
