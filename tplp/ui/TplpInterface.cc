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
void TplpInterface::SetLoadCellParams(const LoadCellParams& params) {}
LoadCellParams TplpInterface::GetLoadCellParams() { return LoadCellParams(); }
void TplpInterface::RunDevTest() {}

}  // namespace ui
}  // namespace tplp