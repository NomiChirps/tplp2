#ifndef TPLP_PAPER_CONTROLLER_H_
#define TPLP_PAPER_CONTROLLER_H_

#include "tplp/config/params.h"
#include "tplp/hx711/hx711.h"

TPLP_DECLARE_PARAM(int32_t, loadcell_offset);
TPLP_DECLARE_PARAM(int32_t, loadcell_scale);

namespace tplp {

class PaperController {
 public:
  PaperController(HX711* loadcell);

  // Returns the current load cell value after zeroing and scaling.
  int32_t GetLoadCellValue() const;
  int32_t GetRawLoadCellValue() const;

 private:
  HX711* const loadcell_;
};

}  // namespace tplp

#endif  // TPLP_PAPER_CONTROLLER_H_