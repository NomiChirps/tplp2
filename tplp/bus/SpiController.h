#ifndef TPLP_BUS_SPICONTROLLER_H_
#define TPLP_BUS_SPICONTROLLER_H_

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
class SpiTransaction;

// Manages a number of devices on one SPI bus.
// TODO: document
class SpiController {
  friend class SpiDevice;
  friend class SpiTransaction;

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
                         std::optional<gpio_pin_t> mosi,
                         std::optional<gpio_pin_t> miso);
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
  SemaphoreHandle_t transaction_mutex_;
};

// Represents an exclusive lock on the SPI bus while it communicates with one
// device, allowing full-duplex transfers to be queued up to be DMA'd in/out.
//
// Note that although Transfer() is nonblocking, the destructor of
// SpiTransaction will wait until all pending transfers are complete before
// returning. This is necessary because the task using an SpiTransaction holds a
// mutex locking out the SPI bus in order to participate in priority inheritance
// if other, higher priority tasks need to use it, and FreeRTOS requires that
// the task releasing such a mutex be the same task that acquired it. Even if it
// didn't, and enqueueing multiple transactions was allowed, SpiController would
// then have to implement its own priority inheritance or queue-jumping
// mechanism in order to provide priority guarantees for access to the bus,
// which, uh, no thank you. Signed, your humble editor.
//
// This class is not thread-safe, and only the task that originally obtained an
// SpiTransaction may Dispose() of or destroy it.
class SpiTransaction {
  friend class SpiDevice;

 public:
  // Represents a full-duplex transfer. The buffers, if provided, are read from
  // and written to simultaneously. The SPI bus will toggle the clock signal
  // exactly `len*8` times regardless of which buffers are provided.
  struct TransferConfig {
    // Optional bytes to transmit.
    const uint8_t* tx_buf;
    // Optional buffer to hold received bytes.
    uint8_t* rx_buf;
    // Number of bytes to transfer.
    uint32_t len;
  };

 public:
  // Wait for all pending transfers to complete and releases the transaction's
  // lock on the SPI bus.
  ~SpiTransaction();
  // Moveable, but not copyable.
  SpiTransaction(SpiTransaction&&);
  // Not stompable.
  SpiTransaction& operator=(SpiTransaction&&) = delete;

  // TODO: document
  // XXX: maybe we can also have Transfer() not blocking!!!
  void TransferBlocking(const TransferConfig& req);

  void Dispose();

 private:
  SpiTransaction(const SpiTransaction&) = delete;
  SpiTransaction& operator=(const SpiTransaction&) = delete;

  explicit SpiTransaction(SpiDevice* device);

 private:
  bool moved_from_;
  SpiDevice* const device_;

  // Used for consistency checks.
  TaskHandle_t originating_task_;
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

  // TODO: It would be more efficient to use a direct-to-task notification!
  // XXX: move to SpiController
  SemaphoreHandle_t blocking_sem_;
};

}  // namespace tplp

#endif  // TPLP_BUS_SPICONTROLLER_H_