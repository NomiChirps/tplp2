#ifndef TPLP_TPLPINTERFACEIMPL_H_
#define TPLP_TPLPINTERFACEIMPL_H_

#include "tplp/bus/i2c.h"
#include "tplp/hx711/hx711.h"
#include "tplp/hx8357/hx8357.h"
#include "tplp/motor/stepper.h"
#include "tplp/ui/TplpInterface.h"

namespace tplp {
namespace ui {

class TplpInterfaceImpl : public ui::TplpInterface {
 public:
  explicit TplpInterfaceImpl(HX8357* display, I2cController* i2c0_controller,
                             HX711* load_cell, StepperMotor* motor_a,
                             StepperMotor* motor_b);
  ~TplpInterfaceImpl() override;

  // Starts the UI background work task.
  void StartTask(int priority, int stack_depth);

  void FlashScreen() override;

  void ScanI2cBus(
      const std::function<void(const I2cScanResult&)>& callback) override;

  int32_t GetLoadCellValue() override;
  int32_t GetRawLoadCellValue() override;
  void SetLoadCellParams(const LoadCellParams& params) override;
  LoadCellParams GetLoadCellParams() override;

  util::Status StepperMotorSetSpeed(int microstep_hz_a,
                                    int microstep_hz_b) override;
  util::Status StepperMotorMove(int microsteps_a, int microsteps_b) override;
  util::Status StepperMotorStopAll(StopType type) override;

  void RunDevTest() override;

 private:
  static void TaskFn(void*);
  void PushWork(const std::function<void()>&);

 private:
  TaskHandle_t task_;
  HX8357* const display_;
  I2cController* const i2c0_controller_;
  HX711* const load_cell_;

  StepperMotor* const motor_a_;
  StepperMotor* const motor_b_;

  QueueHandle_t work_queue_;
};

}  // namespace ui
}  // namespace tplp

#endif  // TPLP_TPLPINTERFACEIMPL_H_