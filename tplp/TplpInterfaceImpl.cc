#include "tplp/TplpInterfaceImpl.h"

#include <iomanip>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/task.h"
#include "hardware/clocks.h"
#include "picolog/picolog.h"
#include "tplp/clkdiv.h"
#include "tplp/config/params.h"
#include "tplp/config/params_storage.h"
#include "tplp/fs/fs.h"
#include "tplp/graphics/lvgl_mutex.h"
#include "tplp/numbers.h"

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
                                     PaperController* paper,
                                     StepperMotor* motor_a,
                                     StepperMotor* motor_b)
    : task_(nullptr),
      display_(display),
      i2c0_controller_(i2c0_controller),
      paper_(paper),
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

int32_t TplpInterfaceImpl::GetLoadCellValue() {
  return paper_->GetLoadCellValue();
}
int32_t TplpInterfaceImpl::GetRawLoadCellValue() {
  return paper_->GetRawLoadCellValue();
}

util::Status TplpInterfaceImpl::StepperMotorSetSpeed(int microstep_hz_a,
                                                     int microstep_hz_b) {
  return util::UnimplementedError("the ui needs work ok?");
}

util::Status TplpInterfaceImpl::StepperMotorMove(int microsteps_a,
                                                 int microsteps_b) {
  return util::UnimplementedError("the ui needs work ok?");
}

util::Status TplpInterfaceImpl::StepperMotorStopAll(StopType type) {
  switch (type) {
    case TplpInterface::StopType::HOLD:
      motor_a_->Stop();
      motor_b_->Stop();
      break;
    case TplpInterface::StopType::SHORT_BRAKE:
      motor_a_->Release();
      motor_b_->Release();
      break;
    case TplpInterface::StopType::FREEWHEEL:
      return util::UnimplementedError("FREEWHEEL stop not implemented");
      break;
    default:
      return util::InvalidArgumentError("bad StopType");
  }
  return util::OkStatus();
}

util::Status TplpInterfaceImpl::SaveAllParameters() {
  return ::tplp::config::SaveAllParameters();
}

void TplpInterfaceImpl::RunDevTest() { LOG(INFO) << "RunDevTest()"; }

std::string TplpInterfaceImpl::GetPaperState() {
  switch (paper_->state()) {
    case PaperController::State::IDLE:
      return "IDLE";
    case PaperController::State::TENSIONING:
      return "TENSIONING";
    case PaperController::State::TENSIONED_IDLE:
      return "TENSIONED_IDLE";
    case PaperController::State::FEEDING:
      return "FEEDING";
  }
  return "UNKNOWN";
}
util::Status TplpInterfaceImpl::TensionPaper() { return paper_->Cmd_Tension(); }
util::Status TplpInterfaceImpl::StartFeed() { return paper_->Cmd_StartFeed(); }
util::Status TplpInterfaceImpl::StopFeed() { return paper_->Cmd_StopFeed(); }
util::Status TplpInterfaceImpl::ReleasePaper() { return paper_->Cmd_Release(); }

}  // namespace ui
}  // namespace tplp