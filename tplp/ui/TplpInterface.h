#ifndef TPLP_UI_TPLPADAPTER_H_
#define TPLP_UI_TPLPADAPTER_H_

#include <functional>
#include <vector>

#include "picolog/status.h"

namespace tplp {
namespace ui {

struct I2cScanResult {
  util::Status status;
  std::vector<uint8_t> addresses;
};

struct LoadCellParams {
  int32_t offset;
  int32_t scale;
};

// Provides a generic interface into whatever bits of the program the UI needs
// to work with. This serves as a dependency injection mechanism, allowing the
// same UI code to run both under the simulator and on the actual hardware.
//
// Callbacks won't necessarily be run in the same thread as the LVGL timer
// handler, but are guaranteed to be holding the global LVGL mutex while they
// run. UI code can be written without worrying about synchronization.
//
// Do-nothing default implementations are provided for the virtual functions.
class TplpInterface {
 public:
  explicit TplpInterface();
  virtual ~TplpInterface();

  // Quickly inverts and un-inverts the display colors, producing an
  // obvious visual effect.
  virtual void FlashScreen();

  virtual void ScanI2cBus(
      const std::function<void(const I2cScanResult&)>& callback);

  // See also params::kLoadCellExpectedRangeAfterScaling.
  virtual int32_t GetLoadCellValue();
  virtual int32_t GetRawLoadCellValue();
  virtual void SetLoadCellParams(const LoadCellParams& params);
  virtual LoadCellParams GetLoadCellParams();

  // Does a test of whatever it is I'm currently working on.
  virtual void RunDevTest();

 private:
  TplpInterface(const TplpInterface&) = delete;
  const TplpInterface& operator=(const TplpInterface&) = delete;
};

}  // namespace ui
}  // namespace tplp

#endif  // TPLP_UI_TPLPADAPTER_H_