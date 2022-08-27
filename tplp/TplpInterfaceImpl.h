#ifndef TPLP_TPLPINTERFACEIMPL_H_
#define TPLP_TPLPINTERFACEIMPL_H_

#include "tplp/HX8357/HX8357.h"
#include "tplp/hx711/hx711.h"
#include "tplp/bus/I2cController.h"
#include "tplp/ui/TplpInterface.h"

namespace tplp {
namespace ui {

class TplpInterfaceImpl : public ui::TplpInterface {
 public:
  explicit TplpInterfaceImpl(HX8357* display, I2cController* i2c0_controller, HX711* load_cell);
  virtual ~TplpInterfaceImpl();

  // Starts the UI background work task.
  void StartTask(int priority, int stack_depth);

  virtual void FlashScreen();

  virtual void ScanI2cBus(
      const std::function<void(const I2cScanResult&)>& callback);

  virtual int32_t GetLoadCellValue();
  virtual void SetLoadCellParams(const LoadCellParams& params);
  virtual LoadCellParams GetLoadCellParams();

 private:
  static void TaskFn(void*);
  void PushWork(const std::function<void()>&);

 private:
  TaskHandle_t task_;
  HX8357* const display_;
  I2cController* const i2c0_controller_;
  HX711* const load_cell_;

  QueueHandle_t work_queue_;
};

}  // namespace ui
}  // namespace tplp

#endif  // TPLP_TPLPINTERFACEIMPL_H_