#ifndef TPLP_UI_MAIN_H_
#define TPLP_UI_MAIN_H_

#include "tplp/ui/TplpInterface.h"

namespace tplp {
namespace ui {

// Caller must be holding the LVGL mutex.
// Caller retains ownership of `tplp`.
void ui_main(TplpInterface* tplp);

}  // namespace ui
}  // namespace tplp

#endif  // TPLP_UI_MAIN_H_