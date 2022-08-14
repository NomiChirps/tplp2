#include "tplp/TplpInterfaceImpl.h"

namespace tplp {
namespace ui {

TplpInterfaceImpl::TplpInterfaceImpl(HX8357* display,
                                     I2cController* i2c0_controller)
    : display_(display), i2c0_controller_(i2c0_controller) {}

TplpInterfaceImpl::~TplpInterfaceImpl() {}

void TplpInterfaceImpl::FlashScreen() {
  display_->SetInvertedColors(true);
  display_->SetInvertedColors(false);
}

void TplpInterfaceImpl::ScanI2cBus(
    const std::function<void(const I2cScanResult&)>& callback) {
  // TODO: do this in a ui worker task instead
  I2cScanResult result;
  std::vector<i2c_address_t> addrs;
  result.status = i2c0_controller_->ScanBus(&addrs);
  for (auto a : addrs) {
    result.addresses.push_back(a.get());
  }
  callback(result);
}

}  // namespace ui
}  // namespace tplp