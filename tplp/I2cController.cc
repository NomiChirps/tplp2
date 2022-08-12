#include "tplp/I2cController.h"

#include "FreeRTOS/semphr.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "picolog/picolog.h"
#include "tplp/config/tplp_config.h"

namespace tplp {
namespace {
// I2C reserves any addresses of the form 000 0xxx or 111 1xxx
// for special purposes.
bool is_reserved(i2c_address_t addr) {
  return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}
}  // namespace

constexpr int kEventQueueDepth = 8;
struct I2cController::Event {
  enum class Tag { INVALID, READ, WRITE } tag = Tag::INVALID;
  i2c_address_t addr = 0;
  uint8_t* buf = nullptr;
  size_t len = 0;
  bool no_stop = false;

  // If not null, will be given after the operation completes.
  SemaphoreHandle_t blocking_sem = nullptr;

  // If not null, will be set to.. .something
  int* out_result = nullptr;

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
    return stream << "{ " << event.tag
                  << " addr=" << static_cast<int>(event.addr)
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
  self->event_queue_ = xQueueCreate(kEventQueueDepth, sizeof(Event));

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
  int ret = i2c_read_blocking(i2c_, event.addr, event.buf, event.len, event.no_stop);
  if (event.out_result) *event.out_result = ret;
}

void I2cController::DoWrite(const Event& event) {
  int ret = i2c_write_blocking(i2c_, event.addr, event.buf, event.len, event.no_stop);
  if (event.out_result) *event.out_result = ret;
}

std::vector<i2c_address_t> I2cController::ScanBus() {
  // TODO: acquire bus mutex
  SemaphoreHandle_t blocking_sem = xSemaphoreCreateBinary();
  uint8_t dummy;
  std::vector<i2c_address_t> result;
  // addresses above 0x77 are reserved
  LOG(INFO) << "Starting I2C bus scan...";
  for (i2c_address_t addr = 0; addr < 0x78; ++addr) {
    if (is_reserved(addr)) continue;
    int ret = 0;
    Event event{
        .tag = Event::Tag::READ,
        .addr = addr,
        .buf = &dummy,
        .len = 1,
        .blocking_sem = blocking_sem,
        .out_result = &ret,
    };
    CHECK(xQueueSendToBack(event_queue_, &event, portMAX_DELAY));
    xSemaphoreTake(blocking_sem, portMAX_DELAY);
    VLOG(1) << "SCAN " << static_cast<int>(addr) << " = " << ret;
    if (ret > 0) {
      LOG(INFO) << "Detected I2C device at address " << (int)addr;
      result.push_back(addr);
    }
  }
  vSemaphoreDelete(blocking_sem);
  return result;
}

bool I2cController::ReadDeviceId(i2c_address_t target, I2cDeviceId* out) {
    LOG(FATAL) << "not implemented";
}

}  // namespace tplp