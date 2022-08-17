#include "tplp/TplpInterfaceImpl.h"

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/task.h"
#include "picolog/picolog.h"
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
                                     I2cController* i2c0_controller)
    : task_(nullptr),
      display_(display),
      i2c0_controller_(i2c0_controller),
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
  display_->SetInvertedColors(true);
  display_->SetInvertedColors(false);
}

void TplpInterfaceImpl::ScanI2cBus(
    const std::function<void(const I2cScanResult&)>& callback) {
  if (!i2c0_controller_) {
    callback({
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

}  // namespace ui
}  // namespace tplp