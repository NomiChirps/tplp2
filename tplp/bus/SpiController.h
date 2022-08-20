#ifndef TPLP_BUS_SPICONTROLLER_H_
#define TPLP_BUS_SPICONTROLLER_H_

#include <memory>
#include <optional>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"
#include "FreeRTOS/task.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "tplp/bus/dma.h"
#include "tplp/bus/types.h"

namespace tplp {

class SpiDevice;

// Manages a number of devices on one SPI bus.
// TODO: document
class SpiController {
  friend class SpiDevice;
  friend class SpiTransactionBuilder;

 public:
  // Initializes the SPI hardware and any necessary FreeRTOS structures.
  static SpiController* Init(spi_inst_t* spi, int freq_hz, gpio_pin_t sclk,
                             std::optional<gpio_pin_t> mosi,
                             std::optional<gpio_pin_t> miso,
                             DmaController* dma);

  // We use software chip-select (GPIO), so cs can be any GPIO pin.
  SpiDevice* AddDevice(gpio_pin_t cs, std::string_view name);

  // Hz
  int actual_frequency() const { return actual_frequency_; }

 private:
  explicit SpiController(spi_inst_t* spi, DmaController* dma,
                         int actual_frequency, SemaphoreHandle_t bus_mutex,
                         SemaphoreHandle_t unblock_semaphore_);
  SpiController(const SpiController&) = delete;
  SpiController& operator=(const SpiController&) = delete;
  ~SpiController() = delete;

 private:
  spi_inst_t* const spi_;
  DmaController* const dma_;
  const int actual_frequency_;

  const SemaphoreHandle_t bus_mutex_;
  const SemaphoreHandle_t unblock_semaphore_;
};

class SpiTransactionBuilder {
 public:
  SpiTransactionBuilder(SpiDevice* device);

  struct TransferArg {
    std::optional<const void*> read_addr = nullptr;
    std::optional<void*> write_addr = nullptr;
    // Must not be zero.
    std::optional<size_t> size = std::nullopt;
  };
  // For arg.read_addr, arg.write_addr, and arg.size:
  //   `std::nullopt`: address will be supplied at runtime.
  //   `nullptr`: send nulls / discard data. (convenience)
  // Transfers are more efficient without an action in between.
  void AddTransfer(const TransferArg& arg,
                   std::optional<Action> before = std::nullopt,
                   std::optional<Action> after = std::nullopt);

  // Length of `args` must be equal to the number of calls to AddTransfer().
  // Each arg must provide exactly the fields that were given as std::nullopt to
  // AddTransfer(). Not thread-safe!
  //
  // `ticks_to_wait` only indicates how long to wait for a spot in the queue.
  // The actual transfer may take longer and must always block once started.
  util::Status RunBlocking(std::initializer_list<TransferArg> args,
                           TickType_t ticks_to_wait = portMAX_DELAY);

 private:
  SpiDevice* device_;
  DmaProgram program_;
  uint32_t dummy_read_;
  uint32_t dummy_write_;
  std::vector<ChannelConfig> args_;
};

class SpiDevice {
  friend class SpiController;
  friend class SpiTransaction;
  friend class SpiTransactionBuilder;

 public:
  SpiTransactionBuilder NewTransactionBuilder();

 private:
  explicit SpiDevice(SpiController* spi, gpio_pin_t cs, std::string_view name);

 private:
  SpiDevice(const SpiDevice&) = delete;
  SpiDevice& operator=(const SpiDevice&) = delete;

  SpiController* const spi_;
  const gpio_pin_t cs_;
  const std::string name_;
};

}  // namespace tplp

#endif  // TPLP_BUS_SPICONTROLLER_H_