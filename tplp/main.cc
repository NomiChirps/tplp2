#include "tplp/ui/main.h"

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <memory>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"
#include "picolog/picolog.h"
#include "tplp/HX8357/HX8357.h"
#include "tplp/I2cController.h"
#include "tplp/RuntimeStats.h"
#include "tplp/SpiManager.h"
#include "tplp/config/tplp_config.h"
#include "tplp/graphics/lvgl_init.h"
#include "tplp/graphics/lvgl_mutex.h"
#include "tplp/time.h"
#include "tplp/types.h"

using std::chrono_literals::operator""ms;

namespace tplp {

static irq_handler_t test_pushbutton_handler_body = nullptr;
static uint64_t last_buttonpress_time;

void TestPushbuttonISR() {
  if (gpio_get_irq_event_mask(Pins::TEST_PUSHBUTTON) & GPIO_IRQ_EDGE_FALL) {
    gpio_acknowledge_irq(Pins::TEST_PUSHBUTTON, GPIO_IRQ_EDGE_FALL);
    uint64_t now = to_us_since_boot(get_absolute_time());
    if (now - last_buttonpress_time < 100) return;
    last_buttonpress_time = now;
    if (test_pushbutton_handler_body) (*test_pushbutton_handler_body)();
  }
}

void SetTestButtonHandler(irq_handler_t fn) {
  gpio_init(Pins::TEST_PUSHBUTTON);
  gpio_set_function(Pins::TEST_PUSHBUTTON, GPIO_FUNC_SIO);
  gpio_set_dir(Pins::TEST_PUSHBUTTON, GPIO_IN);
  gpio_pull_up(Pins::TEST_PUSHBUTTON);
  gpio_set_input_enabled(Pins::TEST_PUSHBUTTON, true);
  gpio_add_raw_irq_handler(Pins::TEST_PUSHBUTTON, &TestPushbuttonISR);
  test_pushbutton_handler_body = fn;

  last_buttonpress_time = to_us_since_boot(get_absolute_time());

  gpio_set_irq_enabled(Pins::TEST_PUSHBUTTON, GPIO_IRQ_EDGE_FALL, true);
  irq_set_enabled(IO_IRQ_BANK0, true);
}

class I2cTest {
 private:
  static I2cController* i2c_;
  static TaskHandle_t task_;

 public:
  static void Start(I2cController* i2c) {
    i2c_ = i2c;
    CHECK(xTaskCreate(&I2cTestTask, "I2C Test", TaskStacks::kTESTONLY, nullptr,
                      1, &task_));
  }

  static void I2cTestTask(void* task_param) {
    SetTestButtonHandler(&ButtonPressedISR);
    for (;;) {
      bool go = ulTaskNotifyTake(true, as_ticks(10'000ms));
      LOG(INFO) << "I2cTestTask awake.";
      if (!go) continue;

      std::vector<i2c_address_t> addrs;
      CHECK_OK(i2c_->ScanBus(&addrs));
      for (i2c_address_t addr : addrs) {
        LOG(INFO) << "Found device at " << addr;
        // TODO: implement DeviceId protocol
        // I2cDeviceId did;
        // i2c_->ReadDeviceId(addr, &did);
      }
    }
  }

 private:
  static void ButtonPressedISR() {
    BaseType_t higher_priority_task_woken;
    vTaskNotifyGiveFromISR(task_, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }
};

I2cController* I2cTest::i2c_;
TaskHandle_t I2cTest::task_;

void StartupTask(void*) {
  picolog::InitLogging();
  if (!xTaskCreate(&picolog::BackgroundTask, "picolog", TaskStacks::kLogging,
                   nullptr, TaskPriorities::kLogging, nullptr)) {
    panic("Failed to start log task");
  }
  LOG(INFO) << "Begin startup...";

  StartRuntimeStatsReportingTask(TaskPriorities::kRuntimeStats);

  I2cController* i2c0_controller =
      I2cController::Init(TaskPriorities::kI2cController0, i2c0, Pins::I2C0_SCL,
                          Pins::I2C0_SDA, 100'000);

  I2cTest::Start(i2c0_controller);

  // skip gui & display for now, i'm working on i2c
  if (0) {
    // TODO: increase frequency when done debugging HX8357
    int kSlow = 500000;
    int kFast = HX8357::kNominalMaxSpiFrequency;
    SpiManager* spi1_manager =
        SpiManager::Init(TaskPriorities::kSpiManager1, spi1, kSlow,
                         Pins::SPI1_SCLK, Pins::SPI1_MOSI, Pins::SPI1_MISO);
    LOG(INFO) << "SpiManager::Init() OK";

    HX8357* display =
        new HX8357D(spi1_manager, Pins::HX8357_CS, Pins::HX8357_DC);
    display->Begin();
    if (!display->SelfTest()) {
      // TODO: flash out an error code on something? board LED?
      LOG(ERROR) << "HX8357 self test failed! Continuing anyway...";
    }
    // Rotate to widescreen and so it's the right way around on my workbench.
    display->SetRotation(0, 1, 1);
    LOG(INFO) << "HX8357 setup OK";

    InitLvgl(display);
    LOG(INFO) << "InitLvgl() OK";

    // Create GUI screens.
    {
      LvglMutex mutex;
      ui_main();
    }
  }

  LOG(INFO) << "Startup complete.";
  vTaskDelete(nullptr);
}

int main() {
#if LIB_PICO_STDIO_USB
  stdio_usb_init();
#endif
#if LIB_PICO_STDIO_UART
  stdio_uart_init_full(uart_default, PICO_DEFAULT_UART_BAUD_RATE, Pins::UART_TX,
                       -1);
#endif

  // TODO: add a build timestamp and version line at bootup
  // printf("tplp2 built %s at %s", TPLP_BUILD_TIMESTAMP, TPLP_VERSION_ID);
  printf("tplp2 boot. TODO: add a build timestamp\n");

  xTaskCreate(&StartupTask, "STARTUP", TaskStacks::kStartup, nullptr,
              TaskPriorities::kStartup, nullptr);
  vTaskStartScheduler();
  panic("scheduler died :(");
}

}  // namespace tplp

int main() { return tplp::main(); }
