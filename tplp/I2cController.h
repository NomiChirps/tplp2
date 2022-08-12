#ifndef TPLP_I2CCONTROLLER_H_
#define TPLP_I2CCONTROLLER_H_

#include <vector>

#include "hardware/i2c.h"
#include "tplp/types.h"
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"

namespace tplp {

// clang-format off
// Features to implement
// - program my address
// - general call
// - send software reset cmd
// - send START BYTE (this is a special address to support slow-polling targets)
// - normal / fast / fast+ mode (high / ultra-fast not supported)
// - only 1 device can be addressed per txn; this is a rp2040 limitation
// - check for reserved addresses
// - 

// notes
// - disable waits for the next command with a STOP bit (not for fifo to be fully empty)
// - changing target address requires disabling
// - disabling requires polling for 25us+ ???
//      above 2^: well, maybe. unclear. see pp.468/469.
//      also pp.488 suggests the procedure is only needed when IC_CLK_TYPE=asynchronous
// - IC_xS_SPKLEN must be configured according to the baud rate (SS/FS)
// - likewise IC_SDA_SETUP, IC_xS_SCL_LCNT, IC_xS_SCL_HCNT
// - programming timing is complicated! see Table 449 for minimums
// - lol the pico-sdk code is super unsubtle about it. reimplement!
// -     uint freq_in = clock_get_hz(clk_sys);
// - pico-sdk says simply:
//           i2c->hw->enable = 0;
//           i2c->hw->tar = addr;
//           i2c->hw->enable = 1;
//  without waiting!
// clang-format on

// Numeric address, 0<=value<256.
using i2c_address_t = uint8_t;

struct I2cDeviceId {
    uint16_t manufacturer : 12;
    uint16_t part_id : 9;
    uint8_t revision : 3;
};

// Only supports 7-bit addressing (for now?)
class I2cController {
 public:
  // Initializes the I2C and DMA hardware.
  static I2cController* Init(int task_priority, 
                             i2c_inst_t* i2c_instance, gpio_pin_t scl,
                             gpio_pin_t sda, int baudrate);

  std::vector<i2c_address_t> ScanBus();

  // TODO: we need an error result object
  // Returns true on success.
  bool ReadDeviceId(i2c_address_t target, I2cDeviceId* out);

 private:
  I2cController();
  I2cController(const I2cController&) = delete;
  I2cController& operator=(const I2cController&) = delete;
  ~I2cController() = delete;

  struct Event;
  static void TaskFn(void*);
  void DoRead(const Event&);
  void DoWrite(const Event&);

private:
  // TODO: const correctness
  i2c_inst_t* i2c_;
  TaskHandle_t task_;

  QueueHandle_t event_queue_;
};

}  // namespace tplp

#endif  // TPLP_I2CCONTROLLER_H_