#ifndef TPLP_BUS_SPI_H_
#define TPLP_BUS_SPI_H_

#include <memory>
#include <optional>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"
#include "FreeRTOS/task.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "picolog/status.h"
#include "tplp/bus/dma.h"
#include "tplp/bus/types.h"

namespace tplp {

class SpiDevice;

// Manages a number of devices on one SPI bus.
// TODO: document
class SpiController {
  friend class SpiDevice;

 public:
  // Initializes the SPI hardware and any necessary FreeRTOS structures.
  static SpiController* Init(spi_inst_t* spi, int freq_hz, gpio_pin_t sclk,
                             std::optional<gpio_pin_t> mosi,
                             std::optional<gpio_pin_t> miso,
                             DmaController* dma);

  // We use software chip-select (GPIO), so cs can be any pin.
  SpiDevice* AddDevice(gpio_pin_t cs, std::string_view name);

  // Hz
  int GetActualFrequency() const { return actual_frequency_; }

 private:
  explicit SpiController(spi_inst_t* spi, DmaController* dma,
                         int actual_frequency,
                         SemaphoreHandle_t transaction_mutex,
                         SemaphoreHandle_t pending_transfers_sem);
  SpiController(const SpiController&) = delete;
  SpiController& operator=(const SpiController&) = delete;
  ~SpiController() = delete;

 private:
  spi_inst_t* const spi_;
  DmaController* const dma_;
  const int actual_frequency_;

  // Held for the duration of an `SpiTransaction`. Note that as it's a mutex
  // with priority inheritance (which is important in this case!), it must be
  // returned by the same task that took it.
  const SemaphoreHandle_t transaction_mutex_;
  const SemaphoreHandle_t pending_transfers_sem_;
};

// Represents an exclusive lock on the SPI bus while it communicates with one
// device, allowing full-duplex transfers to be queued up to be DMA'd in/out.
//
// This class is not thread-safe, and only the task that originally obtained an
// SpiTransaction may Dispose() or destroy it.
class SpiTransaction {
  friend class SpiDevice;

 public:
  // Represents a full-duplex transfer. The buffers, if provided, are read from
  // and written to simultaneously. The SPI bus will toggle the clock signal
  // exactly `len*8` times regardless of which buffers are provided.
  struct TransferConfig {
    // Optional data to transmit.
    const void* read_addr = nullptr;
    // Optional buffer to hold received bytes.
    void* write_addr = nullptr;
    // Number of bytes to transfer.
    uint32_t trans_count = 0;

    // Optional gpio pin to toggle immediately after this transfer is complete.
    // -1 to disable.
    gpio_pin_t toggle_gpio = gpio_pin_t(-1);
  };

 public:
  // Wait for all pending transfers to complete and releases the transaction's
  // lock on the SPI bus.
  ~SpiTransaction();
  // Moveable, but not copyable.
  SpiTransaction(SpiTransaction&&);
  // Not stompable.
  SpiTransaction& operator=(SpiTransaction&&) = delete;

  // Waits for space in the DMA transfer queue, enqueues the given transfer, and
  // returns immediately, without waiting for the transfer to finish.
  void Transfer(const TransferConfig& req);

  // Waits until queued transfers are complete.
  void Flush();

  // Flushes pending transfers, closes the transaction, and releases the lock on
  // the SPI bus.
  void Dispose();

 private:
  SpiTransaction(const SpiTransaction&) = delete;
  SpiTransaction& operator=(const SpiTransaction&) = delete;

  explicit SpiTransaction(spi_inst_t* spi, DmaController* dma,
                          SemaphoreHandle_t pending_transfers_sem,
                          gpio_pin_t cs, SemaphoreHandle_t transaction_mutex);

 private:
  spi_inst_t* const spi_;
  DmaController* const dma_;
  const SemaphoreHandle_t pending_transfers_sem_;
  const gpio_pin_t cs_;
  const SemaphoreHandle_t transaction_mutex_;

  int pending_transfer_count_;
  bool moved_from_;
};

class SpiDevice {
  friend class SpiController;
  friend class SpiTransaction;

 public:
  // Locks the SPI bus to this device and returns a transaction handle that can
  // be used to send and receive messages. Returns `std::nullopt` if a lock on
  // the bus could not be obtained before `timeout` (due to other transactions
  // in progress). The returned SpiTransaction object MUST be deleted by the
  // same task that created it.
  std::optional<SpiTransaction> StartTransaction(TickType_t timeout);

  // As `StartTransaction(timeout)`, but waits forever.
  SpiTransaction StartTransaction();

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

#endif  // TPLP_BUS_SPI_H_