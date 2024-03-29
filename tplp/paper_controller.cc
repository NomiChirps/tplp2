#include "tplp/paper_controller.h"

#include <iomanip>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "pico/time.h"
#include "picolog/picolog.h"
#include "tplp/config/constants.h"
#include "tplp/config/params.h"
#include "tplp/interrupts.h"
#include "tplp/numbers.h"
#include "tplp/pid.h"

TPLP_PARAM(int32_t, loadcell_offset, 100'000, "Load cell zeroing offset");
TPLP_PARAM(int32_t, loadcell_scale, 200, "Load cell scaling divider");

TPLP_PARAM(int32_t, tension_timeout_ms, 2'000,
           "Timeout if initial paper tension can't be established");

TPLP_PARAM(int32_t, target_tension, 200,
           "Target loadcell value to maintain by tensioning the paper");
// TODO: this parameter doesn't actuall do what it says
TPLP_PARAM(int32_t, tension_tolerance, 50,
           "Maximum allowable variation in tension before it's assumed that "
           "something went wrong");
TPLP_PARAM(int32_t, paper_timer_delay_ms, 100,
           "How often to run the PaperController interrupt, in milliseconds");

TPLP_PARAM(int32_t, max_stepper_speed, 500,
           "maximum stepper speed the paper controller will use");

static constexpr int kPidFracBits = 16;
using fixed = fix32_t<kPidFracBits>;
TPLP_PARAM(fixed, tension_loop_p, fixed(0, 0), "");
TPLP_PARAM(fixed, tension_loop_i, fixed(0, 0), "");
TPLP_PARAM(fixed, tension_loop_d, fixed(0, 0), "");
TPLP_PARAM(fixed, tension_loop_i_deadband, fixed(5, 0),
           "+/- range from the tension setpoint inside of which the integral "
           "is not accumulated");
TPLP_PARAM(fixed, tension_loop_i_liveband, fixed(20, 0),
           "+/- range from the tension setpoint outside of which the integral "
           "is set to zero");
TPLP_PARAM(fixed, tension_loop_ff, fixed(0, 0), "feedforward gain");

namespace tplp {
namespace {

enum TaskNotificationBits : uint32_t {
  CMD_TENSION = 0x1,
  CMD_START_FEED = 0x2,
  CMD_STOP_FEED = 0x4,
  CMD_RELEASE = 0x8,
};

static PaperController* instance = nullptr;
}  // namespace

int __not_in_flash("PaperController") PaperController::GetTimerDelay() {
  return 1000 * PARAM_paper_timer_delay_ms.Get();
}

void __not_in_flash("PaperController") PaperController::IsrBody() {
  BaseType_t higher_priority_task_woken;
  xTaskGenericNotifyFromISR(instance->loop_task_, 0, 0, eNoAction, nullptr,
                            &higher_priority_task_woken);
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

template __not_in_flash("PeriodicAlarm") void PaperController::MyTimer::ISR();

PaperController::PaperController(HX711* loadcell, StepperMotor* motor_a,
                                 StepperMotor* motor_b)
    : state_(State::IDLE),
      sys_hz_(clock_get_hz(clk_sys)),
      loadcell_(loadcell),
      motor_src_(motor_a),
      motor_dst_(motor_b) {
  CHECK(!instance);
  instance = this;
}

void PaperController::Init(int cmd_task_priority, int cmd_task_stack_size,
                           int loop_task_priority, int loop_task_stack_size,
                           int alarm_num, uint8_t irq_priority) {
  CHECK(xTaskCreate(&PaperController::CmdTaskFn, "PC Cmd", cmd_task_stack_size,
                    this, cmd_task_priority, &cmd_task_));
  CHECK(xTaskCreate(&PaperController::LoopTaskFn, "PC Loop",
                    loop_task_stack_size, this, loop_task_priority,
                    &loop_task_));
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
      alarm_num, make_timeout_time_ms(PARAM_paper_timer_delay_ms.Get()));
}

void PaperController::CmdTaskFn(void* task_param) {
  static_cast<PaperController*>(task_param)->CmdTaskFnBody();
}
void PaperController::LoopTaskFn(void* task_param) {
  static_cast<PaperController*>(task_param)->LoopTaskFnBody();
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
  if (state() != State::IDLE) {
    return util::FailedPreconditionError("Cannot tension unless state is IDLE");
  }
  xTaskNotify(cmd_task_, TaskNotificationBits::CMD_TENSION, eSetBits);
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
  xTaskNotify(cmd_task_, TaskNotificationBits::CMD_RELEASE, eSetBits);
  return util::OkStatus();
}

void PaperController::UpdateTensionLoop() {
  int32_t new_src_speed;
  // int32_t new_dst_speed;
  // We assume that the Cmd task's priority is lower, so it can't preempt and
  // change the state out from under us.
  State state = state_;

  // TODO: subsume load cell scale into one of the divisions here
  if (state == State::TENSIONING || state == State::TENSIONED_IDLE) {
    static constexpr PIDParams<kPidFracBits> pid_params{
        .P = &PARAM_tension_loop_p,
        .I = &PARAM_tension_loop_i,
        .D = &PARAM_tension_loop_d,
        .I_deadband = &PARAM_tension_loop_i_deadband,
        .I_liveband = &PARAM_tension_loop_i_liveband,
    };
    static PID<kPidFracBits, pid_params> tension("tension",
                                                 /*initial_output=*/0);

    // TODO: get actual time delta instead of assuming
    int32_t dt = PARAM_paper_timer_delay_ms.Get();
    int32_t tension_op =
        tension.Update(PARAM_target_tension.Get(), GetLoadCellValue(), dt);

    // To increase tension (+), set source roller to reverse feed (-).
    // To decrease tension (-), set source roller to forward feed (+).
    new_src_speed = -tension_op;
    // new_dst_speed = position_dst_op;

    // apply feedforward correction
    new_src_speed +=
        (PARAM_tension_loop_ff.Get() * motor_dst_->speed()).trunc();
  } else {
    // Nothing to do; don't touch the motors.
    return;
  }
  // Update motor speeds
  int32_t max_speed = PARAM_max_stepper_speed.Get();
  motor_src_->Move(std::clamp(new_src_speed, -max_speed, max_speed));
  // motor_dst_->Move(std::clamp(new_dst_speed, -max_speed, max_speed));

  // TODO: time this function and see how it actually performs
}

void PaperController::LoopTaskFnBody() {
  for (;;) {
    CHECK(xTaskGenericNotifyWait(0, 0, 0, nullptr, portMAX_DELAY));
    UpdateTensionLoop();
  }
}

void PaperController::CmdTaskFnBody() {
  util::Status status;
  for (;;) {
    TaskNotificationBits notify;
    CHECK(xTaskGenericNotifyWait(
        0, 0, 0xffffffff, reinterpret_cast<uint32_t*>(&notify), portMAX_DELAY));
    if (notify & CMD_TENSION) {
      status = Tension();
      if (!status.ok()) PostError(std::move(status));
    }
    if (notify & CMD_RELEASE) {
      status = Release();
      if (!status.ok()) PostError(std::move(status));
    }
  }
}

util::Status PaperController::Tension() {
  LOG(INFO) << "Executing Tension command";
  if (state_ != State::IDLE) {
    return util::FailedPreconditionError("Invalid state for CMD_TENSION");
  }

  // Release the motors and wait a moment for the paper to go slack.
  motor_src_->Release();
  motor_dst_->Release();
  vTaskDelay(pdMS_TO_TICKS(500));

  if (std::abs(GetLoadCellValue()) > PARAM_tension_tolerance.Get()) {
    return util::FailedPreconditionError(
        "Unexpected load. Clear obstruction or recalibrate load cell");
  }

  // Reactivate the motors, holding current position.
  motor_src_->Stop();
  motor_dst_->Stop();

  // Enable the tensioning PID loop in the timer handler.
  state_ = State::TENSIONING;

  uint64_t end_time = time_us_64() + 1000 * PARAM_tension_timeout_ms.Get();
  bool timed_out = true;
  while (time_us_64() < end_time) {
    if (std::abs(GetLoadCellValue() - PARAM_target_tension.Get()) <=
        PARAM_tension_tolerance.Get()) {
      timed_out = false;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  if (timed_out) {
    // Abort tensioning.
    state_ = State::IDLE;
    motor_src_->Release();
    motor_dst_->Release();
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
  state_ = State::IDLE;
  motor_src_->Release();
  motor_dst_->Release();
  return util::OkStatus();
}

void PaperController::PostError(util::Status status) {
  // TODO: maybe do something more here? idk
  LOG(ERROR) << "Paper controller error: " << status;
}

}  // namespace tplp
