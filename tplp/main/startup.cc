#include "tplp/main/startup.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "picolog/picolog.h"
#include "tplp/HX8357/HX8357.h"
#include "tplp/RuntimeStats.h"
#include "tplp/TSC2007/TSC2007.h"
#include "tplp/TplpInterfaceImpl.h"
#include "tplp/bus/I2cController.h"
#include "tplp/bus/SpiController.h"
#include "tplp/config/pins.h"
#include "tplp/config/tasks.h"
#include "tplp/graphics/lvgl_init.h"

// hmm, maybe lvgl_init should have these instead
#include "tplp/graphics/lvgl_mutex.h"
#include "tplp/ui/main.h"

namespace tplp {

void StartupTask(void*) {
  util::Status status;

  picolog::InitLogging();
  if (!xTaskCreate(&picolog::BackgroundTask, "picolog", TaskStacks::kLogging,
                   nullptr, TaskPriorities::kLogging, nullptr)) {
    panic("Failed to start log task");
  }
  LOG(INFO) << "Begin startup...";

  // Overclocking! Whoo! Doubling the nominal frequency seems to work on my
  // breadboard, surprisingly.
  // const int kSpi1Frequency = HX8357::kNominalMaxSpiFrequency;
  const int kSpi1Frequency = 32'000'000;
  DmaController* dma1_controller0 = DmaController::Init(kDma1);
  SpiController* spi1_manager =
      SpiController::Init(spi1, kSpi1Frequency, Pins::SPI1_SCLK,
                          Pins::SPI1_MOSI, Pins::SPI1_MISO, dma1_controller0);
  LOG(INFO) << "SpiController::Init() OK";

  HX8357* display = new HX8357D(spi1_manager, Pins::HX8357_CS, Pins::HX8357_DC);
  display->Begin();
  if (!display->SelfTest()) {
    // TODO: flash out an error code on something? board LED?
    // reduce the SPI frequency?
    LOG(ERROR) << "HX8357 self test failed";
  }
  // Rotate to widescreen and so it's the right way around on my workbench.
  display->SetRotation(0, 1, 1);
  LOG(INFO) << "HX8357 setup OK";

  I2cController* i2c0_controller = I2cController::Init(
      TaskPriorities::kI2cController0, TaskStacks::kI2cController, i2c0,
      Pins::I2C0_SCL, Pins::I2C0_SDA, 100'000);

  // Touchscreen reader.
  TSC2007* touchscreen = CHECK_NOTNULL(new TSC2007(
      I2cDeviceHandle(i2c0_controller, I2cPeripheralAddress::kTSC2007),
      Pins::TOUCHSCREEN_PENIRQ));
  status = touchscreen->Setup();
  LOG_IF(INFO, status.ok()) << "TSC2007 setup OK";
  // TODO: if we bring the lvgl display up first, we can fail more gracefully
  // by displaying a message
  LOG_IF(FATAL, !status.ok()) << "TSC2007 setup failed: " << status;

  LvglInit lvgl;
  lvgl.BaseInit(TaskPriorities::kLvglTimerHandler,
                TaskStacks::kLvglTimerHandler);
  lvgl.SetDisplay(display, TaskPriorities::kHX8357, TaskStacks::kHX8357);
  lvgl.AddTouchscreen(touchscreen);
  lvgl.Start();

  // Create GUI screens.
  ui::TplpInterfaceImpl* ui_adapter =
      CHECK_NOTNULL(new ui::TplpInterfaceImpl(display, i2c0_controller));
  ui_adapter->StartTask(TaskPriorities::kUiWorker, TaskStacks::kUiWorker);
  {
    LvglMutex mutex;
    ui::ui_main(ui_adapter);
  }
  LOG(INFO) << "User interface setup OK";

  StartRuntimeStatsReportingTask(TaskPriorities::kRuntimeStats,
                                 TaskStacks::kRuntimeStats);

  LOG(INFO) << "Startup complete.";
  vTaskDelete(nullptr);
}

}  // namespace tplp