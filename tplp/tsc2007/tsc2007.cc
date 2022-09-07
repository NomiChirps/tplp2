#include "tplp/tsc2007/tsc2007.h"

#include <iomanip>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "hardware/gpio.h"
#include "picolog/picolog.h"
#include "picolog/status_macros.h"

namespace tplp {
namespace {

enum tsc2007_function : uint8_t {
  MEASURE_TEMP0 = 0 << 4,
  MEASURE_AUX = 2 << 4,
  MEASURE_TEMP1 = 4 << 4,
  ACTIVATE_X = 8 << 4,
  ACTIVATE_Y = 9 << 4,
  ACTIVATE_YPLUS_X = 10 << 4,
  SETUP_COMMAND = 11 << 4,
  MEASURE_X = 12 << 4,
  MEASURE_Y = 13 << 4,
  MEASURE_Z1 = 14 << 4,
  MEASURE_Z2 = 15 << 4,
};

enum tsc2007_power : uint8_t {
  POWERDOWN_IRQON = 0 << 2,
  ADON_IRQOFF = 1 << 2,
  ADOFF_IRQON = 2 << 2,
};

enum tsc2007_resolution : uint8_t {
  ADC_12BIT = 0 << 1,
  ADC_8BIT = 1 << 1,
};

util::Status Command(I2cTransaction& txn, tsc2007_function func,
                     tsc2007_power pwr, tsc2007_resolution res,
                     bool stop = false) {
  uint8_t cmd = func | pwr | res;
  return txn.Write(&cmd, 1, stop);
}

int16_t Convert12BitValue(const uint8_t buf[2]) {
  return (static_cast<uint16_t>(buf[0]) << 4) |
         (static_cast<uint16_t>(buf[1]) >> 4);
}

static TaskHandle_t global_task = nullptr;
static gpio_pin_t global_penirq;

void PenIrqHandlerISR() {
  if (gpio_get_irq_event_mask(global_penirq) & GPIO_IRQ_EDGE_FALL) {
    gpio_acknowledge_irq(global_penirq, GPIO_IRQ_EDGE_FALL);
    BaseType_t higher_priority_task_woken = 0;
    vTaskNotifyGiveFromISR(global_task, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
}

// "In both cases previously listed, it is recommended that whenever the host
// writes to the TSC2007, the controller processor masks the interrupt
// associated to PENIRQ. This masking prevents false triggering of interrupts
// when the PENIRQ line is disabled in the cases previously listed."
void EnablePenIrq() {
  VLOG(1) << "<- PENIRQ unmask";
  gpio_set_irq_enabled(global_penirq, GPIO_IRQ_EDGE_FALL, true);
}
void DisablePenIrq() {
  VLOG(1) << "-> PENIRQ mask";
  gpio_set_irq_enabled(global_penirq, GPIO_IRQ_EDGE_FALL, false);
}

class ScopedDisablePenIrq {
 public:
  explicit ScopedDisablePenIrq() { DisablePenIrq(); }
  ~ScopedDisablePenIrq() { EnablePenIrq(); }
};

}  // namespace

TSC2007::TSC2007(I2cDeviceHandle device)
    : i2c_(device), callback_(nullptr) {}

util::Status TSC2007::Setup() {
  I2cTransaction txn = i2c_.StartTransaction();
  return Command(txn, MEASURE_TEMP0, POWERDOWN_IRQON, ADC_12BIT, true);
}

util::Status TSC2007::ReadPosition(int16_t* x, int16_t* y, int16_t* z1,
                                   int16_t* z2) {
  // "For best performance, the I2C bus should remain in an idle state while
  // an A/D conversion is taking place. This idling prevents digital clock noise
  // from affecting the bit decisions being made by the TSC2007. The controller
  // should wait for at least 10us before attempting to read data from the
  // TSC2007 to realize this best performance. However, the controller does not
  // need to wait for a completed conversion before beginning a read from the
  // target, if full 12-bit performance is not necessary."
  uint8_t buf[2];
  // TODO: it would be so nice if I2cTransaction supported nonblocking ops
  //       and we could queue up this whole thing in one go!
  ScopedDisablePenIrq pause;
  I2cTransaction txn = i2c_.StartTransaction();
  if (x) {
    RETURN_IF_ERROR(Command(txn, MEASURE_X, ADON_IRQOFF, ADC_12BIT));
    busy_wait_us(10);
    RETURN_IF_ERROR(txn.Read(buf, 2));
    VLOG(1) << std::hex << std::setw(2) << std::setfill('0') << "X = [0x"
            << (int)buf[0] << ", 0x" << (int)buf[1] << "]";
    *x = Convert12BitValue(buf);
  }
  if (y) {
    RETURN_IF_ERROR(Command(txn, MEASURE_Y, ADON_IRQOFF, ADC_12BIT));
    busy_wait_us(10);
    RETURN_IF_ERROR(txn.Read(buf, 2));
    VLOG(1) << std::hex << std::setw(2) << std::setfill('0') << "Y = [0x"
            << (int)buf[0] << ", 0x" << (int)buf[1] << "]";
    *y = Convert12BitValue(buf);
  }
  if (z1) {
    RETURN_IF_ERROR(Command(txn, MEASURE_Z1, ADON_IRQOFF, ADC_8BIT));
    RETURN_IF_ERROR(txn.Read(buf, 1));
    VLOG(1) << std::hex << std::setw(2) << std::setfill('0') << "Z1 = 0x"
            << (int)buf[0];
    *z1 = buf[0];
  }
  if (z2) {
    RETURN_IF_ERROR(Command(txn, MEASURE_Z1, ADON_IRQOFF, ADC_8BIT));
    RETURN_IF_ERROR(txn.Read(buf, 1));
    VLOG(1) << std::hex << std::setw(2) << std::setfill('0') << "Z2 = 0x"
            << (int)buf[0];
    *z2 = buf[0];
  }
  return Command(txn, MEASURE_TEMP0, POWERDOWN_IRQON, ADC_12BIT, true);
}

void TSC2007::ReceiveTouchEvents(gpio_pin_t penirq, int priority, int stack_depth,
                                 const TouchCallback& callback) {
  CHECK(!callback_) << "Callback already set";
  CHECK(!global_task)
      << "Sorry, I was lazy and only 1 instance of TSC2007 is allowed";
  CHECK(xTaskCreate(&TSC2007::TaskFn, "TSC2007", stack_depth, this, priority,
                    &global_task));
  global_penirq = penirq;
  callback_ = callback;

  gpio_init(penirq);
  gpio_set_function(penirq, GPIO_FUNC_SIO);
  gpio_set_dir(penirq, GPIO_IN);
  gpio_set_input_enabled(penirq, true);
  gpio_add_raw_irq_handler(penirq, &PenIrqHandlerISR);
  irq_set_enabled(IO_IRQ_BANK0, true);
  EnablePenIrq();
}

void TSC2007::TaskFn(void* task_param) {
  TSC2007* self = static_cast<TSC2007*>(task_param);
  LOG(INFO) << "TSC2007 task started.";
  util::Status status;
  TouchInfo touch;
  for (;;) {
    VLOG(1) << "Waiting for PENIRQ";
    CHECK(ulTaskNotifyTake(true, portMAX_DELAY));
    status = self->ReadPosition(&touch.x, &touch.y, &touch.z1, &touch.z2);
    if (!status.ok()) {
      LOG(ERROR) << "Failed to read touch position: " << status;
      // Give it a second in case the interrupt is stuck or something.
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    if (self->callback_) self->callback_(touch);
  }
}

}  // namespace tplp
