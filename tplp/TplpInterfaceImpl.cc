#include "tplp/TplpInterfaceImpl.h"

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/task.h"
#include "hardware/clocks.h"
#include "picolog/picolog.h"
#include "tplp/clkdiv.h"
#include "tplp/graphics/lvgl_mutex.h"

namespace tplp {
namespace ui {

static constexpr int kWorkQueueDepth = 2;
namespace {
struct WorkQueueItem {
  std::function<void()>* work = nullptr;
};
}  // namespace

TplpInterfaceImpl::TplpInterfaceImpl(HX8357* display,
                                     I2cController* i2c0_controller,
                                     HX711* load_cell, StepperMotor* motor_a,
                                     StepperMotor* motor_b)
    : task_(nullptr),
      display_(display),
      i2c0_controller_(i2c0_controller),
      load_cell_(load_cell),
      motor_a_(motor_a),
      motor_b_(motor_b),
      work_queue_(nullptr) {}

TplpInterfaceImpl::~TplpInterfaceImpl() {}

void TplpInterfaceImpl::StartTask(int priority, int stack_depth) {
  CHECK(!task_) << "already started";
  work_queue_ =
      CHECK_NOTNULL(xQueueCreate(kWorkQueueDepth, sizeof(WorkQueueItem)));
  CHECK(xTaskCreate(&TaskFn, "UI Worker", stack_depth, this, priority, &task_));
}

void TplpInterfaceImpl::TaskFn(void* task_param) {
  TplpInterfaceImpl* self = static_cast<TplpInterfaceImpl*>(task_param);
  LOG(INFO) << "UI worker task started.";
  for (;;) {
    WorkQueueItem item;
    CHECK(xQueueReceive(self->work_queue_, &item, portMAX_DELAY));
    if (item.work) {
      LvglMutex lock;
      (*item.work)();
      delete item.work;
    }
  }
}

void TplpInterfaceImpl::PushWork(const std::function<void()>& work) {
  WorkQueueItem item{
      .work = new std::function<void()>(work),
  };
  CHECK(xQueueSend(CHECK_NOTNULL(work_queue_), &item, portMAX_DELAY));
}

void TplpInterfaceImpl::FlashScreen() {
  LOG(INFO) << "FlashScreen called";
  display_->SetInvertedColors(true);
  vTaskDelay(pdMS_TO_TICKS(100));
  display_->SetInvertedColors(false);
}

void TplpInterfaceImpl::ScanI2cBus(
    const std::function<void(const I2cScanResult&)>& callback) {
  if (!i2c0_controller_) {
    callback(I2cScanResult{
        .status = util::FailedPreconditionError("I2C bus not available"),
    });
    return;
  }
  PushWork([this, callback]() {
    I2cScanResult result;
    std::vector<i2c_address_t> addrs;
    result.status = i2c0_controller_->ScanBus(&addrs);
    for (auto a : addrs) {
      result.addresses.push_back(a.get());
    }
    callback(result);
  });
}

int32_t TplpInterfaceImpl::GetLoadCellValue() { return load_cell_->value(); }
int32_t TplpInterfaceImpl::GetRawLoadCellValue() {
  return load_cell_->raw_value();
}
void TplpInterfaceImpl::SetLoadCellParams(const LoadCellParams& params) {
  LOG(INFO) << "Load cell parameters updated: offset = " << params.offset
            << ", scale = " << params.scale;
  load_cell_->SetOffset(params.offset);
  load_cell_->SetScale(params.scale);
}
LoadCellParams TplpInterfaceImpl::GetLoadCellParams() {
  return {.offset = load_cell_->offset(), .scale = load_cell_->scale()};
}

util::Status TplpInterfaceImpl::StepperMotorSetSpeed(int microstep_hz_a,
                                                     int microstep_hz_b) {
  const int sys_hz = clock_get_hz(clk_sys);
  ClockDivider clkdiv_a;
  ClockDivider clkdiv_b;
  if (motor_a_ && !CalculateClockDivider(sys_hz, microstep_hz_a, &clkdiv_a)) {
    return util::InvalidArgumentError("microstep_hz_a out of range");
  }
  if (motor_b_ && !CalculateClockDivider(sys_hz, microstep_hz_b, &clkdiv_b)) {
    return util::InvalidArgumentError("microstep_hz_b out of range");
  }
  if (motor_a_) motor_a_->SetSpeed(clkdiv_a);
  if (motor_b_) motor_b_->SetSpeed(clkdiv_b);
  return util::OkStatus();
}

util::Status TplpInterfaceImpl::StepperMotorMove(int microsteps_a,
                                                 int microsteps_b) {
  if ((motor_a_ && motor_a_->moving()) || (motor_b_ && motor_b_->moving())) {
    return util::UnavailableError("motor is already moving");
  }
  if (!microsteps_b || !motor_b_) {
    motor_a_->Move(microsteps_a);
  } else if (!microsteps_a || !motor_a_) {
    motor_b_->Move(microsteps_b);
  } else {
    StepperMotor::SimultaneousMove(motor_a_, microsteps_a, motor_b_,
                                   microsteps_b);
  }
  return util::OkStatus();
}

util::Status TplpInterfaceImpl::StepperMotorStopAll(StopType type) {
  StepperMotor::StopType impl_type;
  switch (type) {
    case TplpInterface::StopType::HOLD:
      impl_type = StepperMotor::StopType::HOLD;
      break;
    case TplpInterface::StopType::SHORT_BRAKE:
      impl_type = StepperMotor::StopType::SHORT_BRAKE;
      break;
    case TplpInterface::StopType::FREEWHEEL:
      return util::UnimplementedError("FREEWHEEL stop not implemented");
      break;
    default:
      return util::InvalidArgumentError("bad StopType");
  }
  if (motor_a_) {
    motor_a_->Stop(impl_type);
  }
  if (motor_b_) {
    motor_b_->Stop(impl_type);
  }
  return util::OkStatus();
}

void TplpInterfaceImpl::RunDevTest() {
  LOG(WARNING) << "RunDevTest() (nothing to do)";
}

}  // namespace ui
}  // namespace tplp