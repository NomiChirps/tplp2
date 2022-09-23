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

  virtual int32_t GetLoadCellValue();
  virtual int32_t GetRawLoadCellValue();

  // Direct motor control for manual moves from the UI.
  virtual util::Status SteppersRelease();
  virtual util::Status SteppersSetSpeed(int microstep_hz_a,
                                            int microstep_hz_b);

  // Paper feed control.
  virtual std::string GetPaperState();
  virtual util::Status TensionPaper();
  virtual util::Status StartFeed();
  virtual util::Status StopFeed();
  virtual util::Status ReleasePaper();

  // Saves all runtime-tweakable parameters to disk.
  virtual util::Status SaveAllParameters();

  // Does a test of whatever it is I'm currently working on.
  virtual void RunDevTest();

 private:
  TplpInterface(const TplpInterface&) = delete;
  const TplpInterface& operator=(const TplpInterface&) = delete;
};

}  // namespace ui
}  // namespace tplp

#endif  // TPLP_UI_TPLPADAPTER_H_