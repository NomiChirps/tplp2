#include "tplp/ui/TplpInterface.h"

namespace tplp {
namespace ui {
TplpInterface::TplpInterface() {}
TplpInterface::~TplpInterface() {}

void TplpInterface::FlashScreen() {}

void TplpInterface::ScanI2cBus(
    const std::function<void(const I2cScanResult&)>& callback) {
  callback({
      .status = util::UnimplementedError("TplpInterface stub"),
  });
}

int32_t TplpInterface::GetLoadCellValue() { return 0; }
int32_t TplpInterface::GetRawLoadCellValue() { return 0; }
void TplpInterface::RunDevTest() {}
util::Status TplpInterface::StepperMotorSetSpeed(int microstep_hz_a,
                                                 int microstep_hz_b) {
  return util::OkStatus();
}
util::Status TplpInterface::StepperMotorMove(int microsteps_a,
                                             int microsteps_b) {
  return util::OkStatus();
}
util::Status TplpInterface::StepperMotorStopAll(StopType type) {
  return util::OkStatus();
}
std::string TplpInterface::GetPaperState() { return "<paper status>"; }
util::Status TplpInterface::SaveAllParameters() { return util::OkStatus(); }
util::Status TplpInterface::TensionPaper() { return util::OkStatus(); }
util::Status TplpInterface::StartFeed() { return util::OkStatus(); }
util::Status TplpInterface::StopFeed() { return util::OkStatus(); }
util::Status TplpInterface::ReleasePaper() { return util::OkStatus(); }

}  // namespace ui
}  // namespace tplp