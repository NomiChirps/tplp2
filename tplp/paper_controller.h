#ifndef TPLP_PAPER_CONTROLLER_H_
#define TPLP_PAPER_CONTROLLER_H_

#include <variant>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "picolog/status.h"
#include "tplp/config/constants.h"
#include "tplp/hx711/hx711.h"
#include "tplp/interrupts.h"
#include "tplp/motor/stepper.h"
#include "tplp/numbers.h"

namespace tplp {

class PaperController {
 public:
  enum class State {
    IDLE,
    TENSIONING,
    TENSIONED_IDLE,
    FEEDING,
  };

  // TODO: decide what physical units this should be
  using linear_velocity_t = int32_t;

 public:
  // motor_src should be connected to the input roller, and motor_dst to the
  // output roller.
  PaperController(HX711* loadcell, StepperMotor* motor_src,
                  StepperMotor* motor_dst);
  // alarm_num: rp2040 hardware alarm number, 0-3.
  void Init(int cmd_task_priority, int cmd_task_stack_size,
            int loop_task_priority, int loop_task_stack_size, int alarm_num,
            uint8_t irq_priority);

  inline State state() const { return state_; }

  // Sets the target paper feed rate. This can be adjusted at any time,
  // including during feeding. The value may be positive (forward) or negative
  // (backward).
  util::Status SetFeedRate(linear_velocity_t velocity);

  // Begins attempting to tension the paper, which is assumed to have already
  // been loaded onto both rollers. Returns immediately.
  //
  // Current state must be IDLE.
  // State will transition to TENSIONED_IDLE if successful.
  util::Status Cmd_Tension();

  // Starts feeding the paper forward at the current target feed rate.
  //
  // Current state must be TENSIONED_IDLE.
  // State will transition to FEEDING if successful.
  util::Status Cmd_StartFeed();

  // Stops feeding the paper, maintaining tension.
  //
  // Current state must be FEEDING.
  // State will transition to TENSIONED_IDLE if successful.
  util::Status Cmd_StopFeed();

  // Releases the paper tension.
  //
  // Current state must be TENSIONED_IDLE.
  // State will transition to IDLE if successful.
  util::Status Cmd_Release();

  // Returns the current load cell value after zeroing and scaling.
  int32_t GetLoadCellValue() const;
  // Returns the raw load cell sensor reading.
  int32_t GetRawLoadCellValue() const;

 private:
  static void CmdTaskFn(void* task_param);
  void CmdTaskFnBody();
  static void LoopTaskFn(void* task_param);
  void LoopTaskFnBody();
  void PostError(util::Status status);

  void UpdateTensionLoop();

  static int __not_in_flash("PaperController") GetTimerDelay();
  static void __not_in_flash("PaperController") IsrBody();
  // must match HardwareAlarms::kPaperController
  // TODO: sort out this odd dependency by statically allocating everything
  static constexpr int kAlarmNum = 0;
  using MyTimer = PeriodicAlarm<kAlarmNum, IsrBody, GetTimerDelay>;
  friend MyTimer;

  // PRE: IDLE
  // POST if OK: TENSIONED_IDLE
  // POST if error: IDLE
  util::Status Tension();

  // PRE: TENSIONED_IDLE
  // POST: IDLE
  util::Status Release();

 private:
  TaskHandle_t cmd_task_;
  TaskHandle_t loop_task_;
  volatile State state_;

  const int32_t sys_hz_;
  HX711* const loadcell_;
  StepperMotor* const motor_src_;
  StepperMotor* const motor_dst_;
};

}  // namespace tplp

#endif  // TPLP_PAPER_CONTROLLER_H_