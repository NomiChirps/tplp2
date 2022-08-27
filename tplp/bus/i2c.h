#ifndef TPLP_BUS_I2C_H_
#define TPLP_BUS_I2C_H_

#include <vector>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/semphr.h"
#include "hardware/i2c.h"
#include "picolog/status.h"
#include "tplp/bus/dma.h"
#include "tplp/bus/types.h"

namespace tplp {

// clang-format off
// Features to implement
// - general call
// - send software reset cmd
// - send START BYTE (this is a special address to support slow-polling targets)
// - normal / fast / fast+ mode (high / ultra-fast not supported)

// notes
// - IC_xS_SPKLEN must be configured according to the baud rate (SS/FS)
// - likewise IC_SDA_SETUP, IC_xS_SCL_LCNT, IC_xS_SCL_HCNT
// - programming timing is complicated! see Table 449 for minimums
// - lol the pico-sdk code is super unsubtle about it. reimplement!
// -     uint freq_in = clock_get_hz(clk_sys);
// clang-format on

class I2cController;
class I2cTransaction {
 public:
  ~I2cTransaction();

  i2c_address_t target_address() const { return addr_; }
  bool stopped() const { return stopped_; }

  // TODO: add nonblocking operations

  util::Status WriteAndStop(const uint8_t* buf, size_t len) {
    return Write(buf, len, true);
  }
  util::Status ReadAndStop(uint8_t* buf, size_t len) {
    return Read(buf, len, true);
  }

  util::Status Write(const uint8_t* buf, size_t len, bool stop = false);
  util::Status Read(uint8_t* buf, size_t len, bool stop = false);

  // Flags that the next Read or Write of this transaction should be preceded by
  // a RESTART condition on the I2C bus. This flag is cleared after any call to
  // Read or Write.
  void Restart();

  // TODO: document nicely.
  // it's like dispose. this also happens if you don't read/write with a stop
  // before destruction
  util::Status Abort();

 private:
  friend class I2cController;

  I2cTransaction(const I2cTransaction&) = delete;
  I2cTransaction& operator=(const I2cTransaction&) = delete;
  I2cTransaction(I2cTransaction&&);  // moveable
  I2cTransaction& operator=(I2cTransaction&&) = delete;

  explicit I2cTransaction(I2cController* controller_, i2c_address_t addr);

  void AfterStop();

 private:
  I2cController* controller_;
  i2c_address_t addr_;
  // also covers the moved-from condition
  bool stopped_;
  bool restart_;
};

// Only supports 7-bit addressing (for now?)
class I2cController {
 public:
  // Initializes the I2C and DMA hardware and starts a task to service it.
  static I2cController* Init(DmaController* dma, int priority, int stack_depth,
                             i2c_inst_t* i2c_instance, gpio_pin_t scl,
                             gpio_pin_t sda, int baudrate);

  // Note: Device ID is not supported in RP2040.
  util::Status ScanBus(std::vector<i2c_address_t>* detected_addresses);

  // Returns a transaction handle that locks the bus for the duration of its
  // lifetime and can be used to communicate with the given target address.
  // Blocks until the bus is free, which could be a long time.
  // TODO: use StatusOr so we can specify a timeout
  I2cTransaction StartTransaction(i2c_address_t target);

  struct Event;

 private:
  friend class I2cTransaction;

  explicit I2cController();
  I2cController(const I2cController&) = delete;
  I2cController& operator=(const I2cController&) = delete;
  ~I2cController() = delete;

  static void TaskFn(void*);
  void DoTransfer(const Event&);

 private:
  // TODO: const correctness
  DmaController* dma_;
  i2c_inst_t* i2c_;
  TaskHandle_t task_;

  QueueHandle_t event_queue_;
  SemaphoreHandle_t txn_mutex_;

  // Used by blocking operations in I2cTransaction. Only "shared" serially,
  // because no more than one transaction can be active at a time.
  SemaphoreHandle_t shared_blocking_sem_;

  bool new_transaction_;
};

// A restriction of the I2cController interface, allowing communication with
// only one address. Lightweight and freely copyable.
class I2cDeviceHandle {
 public:
  explicit I2cDeviceHandle(I2cController* i2c, i2c_address_t addr)
      : i2c_(i2c), addr_(addr) {}

  I2cTransaction StartTransaction() { return i2c_->StartTransaction(addr_); }

 private:
  I2cController* const i2c_;
  const i2c_address_t addr_;
};

}  // namespace tplp

#endif  // TPLP_BUS_I2C_H_