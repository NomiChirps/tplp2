#include "tplp/I2cController.h"

#include "FreeRTOS/semphr.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "picolog/picolog.h"
#include "tplp/config/tplp_config.h"
#include "tplp/rtos_utils.h"

namespace tplp {
namespace {
// I2C reserves any addresses of the form 000 0xxx or 111 1xxx
// for special purposes.
bool is_reserved(i2c_address_t addr) {
  return (addr.get() & 0x78) == 0 || (addr.get() & 0x78) == 0x78;
}
}  // namespace

constexpr int kEventQueueDepth = 8;
struct I2cController::Event {
  enum class Tag { INVALID, READ, WRITE } tag = Tag::INVALID;
  i2c_address_t addr = i2c_address_t(0);
  uint8_t* buf = nullptr;
  size_t len = 0;
  bool stop = true;

  // If not null, will be given after the operation completes.
  SemaphoreHandle_t blocking_sem = nullptr;

  // If not null, will be set to the result status of the operation.
  // FIXME: make sure these are correct
  // util::IsNotFound(): read/write was unacknowledged
  util::Status* out_result = nullptr;

  friend std::ostream& operator<<(std::ostream& stream, const Event::Tag& tag) {
    switch (tag) {
      case Tag::INVALID:
        return stream << "INVALID";
      case Tag::READ:
        return stream << "READ";
      case Tag::WRITE:
        return stream << "WRITE";
      default:
        return stream << (int)tag;
    }
  }

  friend std::ostream& operator<<(std::ostream& stream, const Event& event) {
    return stream << "{ " << event.tag << " addr=" << event.addr
                  << " buf=" << std::hex << (void*)event.buf << std::dec
                  << " len=" << event.len << " }";
  }
};

I2cController* I2cController::Init(int task_priority, i2c_inst_t* i2c_instance,
                                   gpio_pin_t scl, gpio_pin_t sda,
                                   int baudrate) {
  const int i2c_instance_index = i2c_hw_index(i2c_instance);
  CHECK_GT(baudrate, 0);
  // i2c_init configures it for fast mode, which the hardware considers
  // equivalent to fast mode plus, which has a maximum rate of 1000kbps.
  CHECK_LE(baudrate, 1'000'000);
  int actual_baudrate = i2c_init(i2c_instance, baudrate);
  LOG(INFO) << "I2C" << i2c_instance_index << " baudrate set to "
            << actual_baudrate << "Hz";

  gpio_set_function(scl, GPIO_FUNC_I2C);
  gpio_set_function(sda, GPIO_FUNC_I2C);
  gpio_pull_up(scl);
  gpio_pull_up(sda);

  // TODO: acquire dma channels etc

  I2cController* self = new I2cController();
  self->i2c_ = i2c_instance;
  self->event_queue_ =
      CHECK_NOTNULL(xQueueCreate(kEventQueueDepth, sizeof(Event)));
  self->txn_mutex_ = CHECK_NOTNULL(xSemaphoreCreateMutex());
  self->shared_blocking_sem_ = CHECK_NOTNULL(xSemaphoreCreateBinary());

  char name[5] = "I2C ";
  name[3] = i2c_instance_index ? '1' : '0';
  xTaskCreate(&TaskFn, name, TaskStacks::kI2cController, self, task_priority,
              &self->task_);

  return self;
}

I2cController::I2cController(){};

void I2cController::TaskFn(void* task_param) {
  I2cController* self = static_cast<I2cController*>(task_param);
  LOG(INFO) << "I2cController task started.";
  for (;;) {
    Event event;
    VLOG(1) << "Waiting for next event.";
    CHECK(xQueueReceive(self->event_queue_, &event, portMAX_DELAY));
    VLOG(1) << "Event: " << event;

    switch (event.tag) {
      case Event::Tag::READ:
        self->DoRead(event);
        break;
      case Event::Tag::WRITE:
        self->DoWrite(event);
        break;
      default:
        LOG(FATAL) << "Invalid event received: " << event;
    }

    if (event.blocking_sem) CHECK(xSemaphoreGive(event.blocking_sem));
  }
}

void I2cController::DoRead(const Event& event) {
  int ret = i2c_read_blocking(i2c_, event.addr.get(), event.buf, event.len,
                              !event.stop);
  // FIXME: for now, assuming all errors are NotFoundError.
  if (event.out_result) {
    *event.out_result = ret <= 0 ? util::NotFoundError("") : util::OkStatus();
  }
}

void I2cController::DoWrite(const Event& event) {
  int ret = i2c_write_blocking(i2c_, event.addr.get(), event.buf, event.len,
                               !event.stop);
  // FIXME: for now, assuming all errors are NotFoundError.
  if (event.out_result) {
    *event.out_result = ret <= 0 ? util::NotFoundError("") : util::OkStatus();
  }
}

util::Status I2cController::ScanBus(
    std::vector<i2c_address_t>* detected_addresses) {
  CHECK_NOTNULL(detected_addresses);
  uint8_t dummy;

  // addresses above 0x77 are reserved
  LOG(INFO) << "Starting I2C bus scan...";
  for (i2c_address_t addr(0); addr < i2c_address_t(0x78); ++addr) {
    if (is_reserved(addr)) continue;
    I2cTransaction txn = StartTransaction(addr);
    util::Status status = txn.ReadAndStop(&dummy, 1);

    if (util::IsNotFound(status)) {
      VLOG(1) << "SCAN " << addr << " no response";
      continue;
    } else if (!status.ok()) {
      return status;
    }
    VLOG(1) << "SCAN " << addr << " found!";
    detected_addresses->push_back(addr);
  }
  return util::OkStatus();
}

util::Status I2cController::ReadDeviceId(i2c_address_t target,
                                         I2cDeviceId* out) {
  return util::UnimplementedError("ReadDeviceId");
}

I2cTransaction I2cController::StartTransaction(i2c_address_t addr) {
  CHECK(xSemaphoreTake(txn_mutex_, portMAX_DELAY));
  return I2cTransaction(this, addr);
}

I2cTransaction::I2cTransaction(I2cController* controller, i2c_address_t addr)
    : controller_(controller), addr_(addr), stopped_(false) {}

I2cTransaction::~I2cTransaction() {
  if (!stopped_) {
    util::Status s = Abort();
    LOG(WARNING) << "Transaction destroyed before STOP. Abort status: " << s;
    CHECK(xSemaphoreGive(controller_->txn_mutex_));
  }
}
I2cTransaction::I2cTransaction(I2cTransaction&& other)
    : controller_(other.controller_),
      addr_(other.addr_),
      stopped_(other.stopped_) {
  other.stopped_ = true;
}

util::Status I2cTransaction::Write(const uint8_t* buf, size_t len, bool stop) {
  CHECK(!stopped_);
  util::Status status;
  I2cController::Event event{
      .tag = I2cController::Event::Tag::WRITE,
      .addr = addr_,
      .buf = const_cast<uint8_t*>(buf),
      .len = len,
      .stop = stop,
      .blocking_sem = controller_->shared_blocking_sem_,
      .out_result = &status,
  };
  CHECK(xQueueSendToBack(controller_->event_queue_, &event, portMAX_DELAY));
  CHECK(xSemaphoreTake(controller_->shared_blocking_sem_, portMAX_DELAY));
  if (stop) {
    DoStop();
  }
  return status;
}

util::Status I2cTransaction::Read(uint8_t* buf, size_t len, bool stop) {
  CHECK(!stopped_);
  util::Status status;
  I2cController::Event event{
      .tag = I2cController::Event::Tag::READ,
      .addr = addr_,
      .buf = buf,
      .len = len,
      .stop = stop,
      .blocking_sem = controller_->shared_blocking_sem_,
      .out_result = &status,
  };
  CHECK(xQueueSendToBack(controller_->event_queue_, &event, portMAX_DELAY));
  CHECK(xSemaphoreTake(controller_->shared_blocking_sem_, portMAX_DELAY));
  if (stop) {
    DoStop();
  }
  return status;
}

void I2cTransaction::DoStop() {
  stopped_ = true;
  xSemaphoreGive(controller_->txn_mutex_);
}

util::Status I2cTransaction::Abort() {
  // FIXME: implement
  return util::UnimplementedError("I2cTransaction::Abort()");
}

}  // namespace tplp