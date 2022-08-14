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
#include "tplp/SpiController.h"
#include "tplp/TSC2007/TSC2007.h"
#include "tplp/TplpInterfaceImpl.h"
#include "tplp/config/tplp_config.h"
#include "tplp/graphics/lvgl_init.h"
#include "tplp/graphics/lvgl_mutex.h"
#include "tplp/time.h"
#include "tplp/types.h"

using std::chrono_literals::operator""ms;

namespace tplp {

static irq_handler_t test_pushbutton_handler_body = nullptr;
static uint64_t last_buttonpress_time;

static void TestPushbuttonISR() {
  if (gpio_get_irq_event_mask(Pins::TEST_PUSHBUTTON) & GPIO_IRQ_EDGE_FALL) {
    gpio_acknowledge_irq(Pins::TEST_PUSHBUTTON, GPIO_IRQ_EDGE_FALL);
    uint64_t now = to_us_since_boot(get_absolute_time());
    // this button is really very bad. give it a WHOLE ENTIRE SECOND
    // to stop bouncing.
    if (now - last_buttonpress_time < 1'000'000) return;
    last_buttonpress_time = now;
    if (test_pushbutton_handler_body) (*test_pushbutton_handler_body)();
  }
}

static void SetTestButtonHandler(irq_handler_t fn) {
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

void StartupTask(void*) {
  util::Status status;

  picolog::InitLogging();
  if (!xTaskCreate(&picolog::BackgroundTask, "picolog", TaskStacks::kLogging,
                   nullptr, TaskPriorities::kLogging, nullptr)) {
    panic("Failed to start log task");
  }
  LOG(INFO) << "Begin startup...";

  I2cController* i2c0_controller =
      I2cController::Init(TaskPriorities::kI2cController0, i2c0, Pins::I2C0_SCL,
                          Pins::I2C0_SDA, 100'000);

  SpiController* spi1_manager = SpiController::Init(
      TaskPriorities::kSpiController1, spi1, HX8357::kNominalMaxSpiFrequency,
      Pins::SPI1_SCLK, Pins::SPI1_MOSI, Pins::SPI1_MISO);
  LOG(INFO) << "SpiController::Init() OK";

  HX8357* display = new HX8357D(spi1_manager, Pins::HX8357_CS, Pins::HX8357_DC);
  display->Begin();
  if (!display->SelfTest()) {
    // TODO: flash out an error code on something? board LED?
    LOG(ERROR) << "HX8357 self test failed";
  }
  // Rotate to widescreen and so it's the right way around on my workbench.
  display->SetRotation(0, 1, 1);
  LOG(INFO) << "HX8357 setup OK";

  // Touchscreen reader.
  TSC2007* touchscreen = CHECK_NOTNULL(new TSC2007(
      I2cDeviceHandle(i2c0_controller, I2cPeripheralAddress::kTSC2007),
      Pins::TOUCHSCREEN_PENIRQ));
  status = touchscreen->Setup();
  LOG_IF(INFO, status.ok()) << "TSC2007 setup OK";
  // TODO: if we bring the lvgl display up first, we can fail more gracefully by
  //       displaying a message
  LOG_IF(FATAL, !status.ok()) << "TSC2007 setup failed: " << status;

  LvglInit lvgl;
  lvgl.BaseInit();
  lvgl.SetDisplay(display);
  lvgl.AddTouchscreen(touchscreen);
  lvgl.Start();

  // Create GUI screens.
  {
    ui::TplpInterface* tplp =
        CHECK_NOTNULL(new ui::TplpInterfaceImpl(display, i2c0_controller));
    LvglMutex mutex;
    ui::ui_main(tplp);
  }
  LOG(INFO) << "User interface setup OK";

  StartRuntimeStatsReportingTask(TaskPriorities::kRuntimeStats);

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
