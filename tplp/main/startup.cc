#include "tplp/main/startup.h"

#include <iomanip>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "picolog/picolog.h"
#include "picolog/status_macros.h"
#include "tplp/RuntimeStats.h"
#include "tplp/TplpInterfaceImpl.h"
#include "tplp/adafruit_seesaw/adafruit_seesaw.h"
#include "tplp/bus/i2c.h"
#include "tplp/bus/spi.h"
#include "tplp/config/hw.h"
#include "tplp/config/params.h"
#include "tplp/config/params_storage.h"
#include "tplp/config/tasks.h"
#include "tplp/fs/fs.h"
#include "tplp/fs/sdspi.h"
#include "tplp/graphics/lvgl_init.h"
#include "tplp/hx711/hx711.h"
#include "tplp/hx8357/hx8357.h"
#include "tplp/tsc2007/tsc2007.h"

// hmm, maybe lvgl_init should have these instead
#include "tplp/graphics/lvgl_mutex.h"
#include "tplp/ui/main.h"

namespace tplp {
void StartupTask(void*) {
  util::Status status;

  picolog::InitLogging();
  if (!xTaskCreate(&picolog::BackgroundTask, "picolog", TaskStacks::kPicolog,
                   nullptr, TaskPriorities::kPicolog, nullptr)) {
    panic("Failed to start log task");
  }
  LOG(INFO) << "Begin startup...";

  if (!config::DeferredInitError().ok()) {
    LOG(FATAL) << "Error during params static initialization: "
               << config::DeferredInitError();
  }

  // DMA allocations.
  // FIXME: I2cController is incredibly fragile and requires its own DMA IRQ.
  DmaController* dma_i2c0 = DmaController::Init(kDma1);
  DmaController* dma_spi1 = DmaController::Init(kDma0);
  DmaController* dma_stepper_a = DmaController::Init(kDma0);
  DmaController* dma_stepper_b = DmaController::Init(kDma0);

  SpiController* spi1_manager =
      SpiController::Init(spi1, Frequencies::kSpi1, Pins::SPI1_SCLK,
                          Pins::SPI1_MOSI, Pins::SPI1_MISO, dma_spi1);
  LOG(INFO) << "SpiController::Init() OK";
  SpiDevice* spi_sdcard = spi1_manager->AddDevice(Pins::HX8357SD_CS, "SD Card");
  SpiDevice* spi_hx8357 = spi1_manager->AddDevice(Pins::HX8357_CS, "HX8357");

  SdSpi* sdcard = CHECK_NOTNULL(new SdSpi(spi_sdcard));
  status = sdcard->Init();
  LOG_IF(FATAL, !status.ok()) << "SD card init failed: " << status;
  status = fs::Init(sdcard);
  LOG_IF(FATAL, !status.ok()) << "Filesystem init failed: " << status;

  // Load flag values ASAP after getting the SD card up.
  status = config::InitParameterStorage();
  if (!status.ok()) {
    LOG(FATAL) << "Failed to init parameter storage: " << status;
  }
  status = config::LoadAllParameters();
  LOG_IF(ERROR, !status.ok()) << "Error loading flags: " << status;

  HX8357* display = new HX8357D(spi_hx8357, Pins::HX8357_DC);
  display->Init();
  while (!display->SelfTest()) {
    // TODO: flash out an error code on something? board LED?
    LOG(ERROR) << "HX8357 self test failed";
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  // Rotate to widescreen and so it's the right way around on my workbench.
  display->SetRotation(0, 1, 1);
  LOG(INFO) << "HX8357 setup OK";

  I2cController* i2c0_controller = I2cController::Init(
      dma_i2c0, TaskPriorities::kI2cController0, TaskStacks::kI2cController,
      i2c0, Pins::I2C0_SCL, Pins::I2C0_SDA, Frequencies::kI2c0);

  // Run a quick bus scan.
  {
    std::vector<i2c_address_t> addrs;
    CHECK_OK(i2c0_controller->ScanBus(&addrs));
    for (i2c_address_t addr : addrs) {
      LOG(INFO) << "Detected I2C device " << addr;
    }
  }

  // Rotary encoder input device.
  // FIXME: doesn't fuckin work
  // Adafruit4991* encoder = CHECK_NOTNULL(new Adafruit4991(
  //     I2cDeviceHandle(i2c0_controller,
  //     I2cPeripheralAddress::kRotaryEncoder)));
  // status = encoder->Init();
  // LOG_IF(FATAL, !status.ok()) << "Adafruit4991 init failed: " << status;

  // Touchscreen reader.
  TSC2007* touchscreen = CHECK_NOTNULL(new TSC2007(
      I2cDeviceHandle(i2c0_controller, I2cPeripheralAddress::kTSC2007)));
  status = touchscreen->Init();
  // TODO: if we bring the lvgl display up first, we can fail more gracefully
  // by displaying a message
  LOG_IF(FATAL, !status.ok()) << "TSC2007 setup failed: " << status;
  LOG(INFO) << "TSC2007 setup OK";

  LvglInit lvgl;
  lvgl.BaseInit(TaskPriorities::kLvglTimerHandler,
                TaskStacks::kLvglTimerHandler);
  lvgl.SetDisplay(display, TaskPriorities::kHX8357, TaskStacks::kHX8357);
  lvgl.AddTouchscreen(touchscreen);
  // lvgl.AddEncoder(encoder);
  lvgl.Start();

  // Load cell reader.
  HX711* loadcell = HX711::Init(pio0, Pins::HX711_SCK, Pins::HX711_DOUT);

  // Steppies!
  StepperMotor::StaticInit(Frequencies::kStepperPwm);
  StepperMotor* motor_a =
      CHECK_NOTNULL(StepperMotor::Init(dma_stepper_a, pio1,
                                       {.a1 = Pins::MOTOR_A_A1,
                                        .a2 = Pins::MOTOR_A_A2,
                                        .b1 = Pins::MOTOR_A_B1,
                                        .b2 = Pins::MOTOR_A_B2},
                                       IrqPriorities::kStepperTimer));
  StepperMotor* motor_b =
      CHECK_NOTNULL(StepperMotor::Init(dma_stepper_b, pio1,
                                       {.a1 = Pins::MOTOR_B_A1,
                                        .a2 = Pins::MOTOR_B_A2,
                                        .b1 = Pins::MOTOR_B_B1,
                                        .b2 = Pins::MOTOR_B_B2},
                                       IrqPriorities::kStepperTimer));

  PaperController* paper_controller = CHECK_NOTNULL(new PaperController(
      loadcell, /*motor_src=*/motor_b, /*motor_dst=*/motor_a));
  paper_controller->Init(TaskPriorities::kPaperController,
                         TaskStacks::kPaperController,
                         // alarms 0 and 1 are used by the steppers
                         /*alarm_num=*/2, IrqPriorities::kPaperController);

  // Create GUI screens.
  ui::TplpInterfaceImpl* ui_adapter = CHECK_NOTNULL(new ui::TplpInterfaceImpl(
      display, i2c0_controller, paper_controller, motor_a, motor_b));
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