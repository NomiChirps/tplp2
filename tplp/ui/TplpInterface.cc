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

}  // namespace ui
}  // namespace tplp