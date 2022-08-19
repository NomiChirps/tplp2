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
class SpiTransaction;

// Manages a number of devices on one SPI bus.
// TODO: document
class SpiController {
  friend class SpiDevice;
  friend class SpiTransaction;

 public:
  // Initializes the SPI hardware and any necessary FreeRTOS structures.
  static SpiController* Init(int priority, int stack_depth, spi_inst_t* spi,
                             int freq_hz, gpio_pin_t sclk,
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
                         QueueHandle_t event_queue, SemaphoreHandle_t flush_sem,
                         std::optional<gpio_pin_t> mosi,
                         std::optional<gpio_pin_t> miso);
  SpiController(const SpiController&) = delete;
  SpiController& operator=(const SpiController&) = delete;
  ~SpiController() = delete;

  static void TaskFn(void*);

  // Events that are handled in the event queue.
  struct Event;
  void DoTransfer(const Event&);
  void DoFlush(const Event&);
  void DoEndTransaction(const Event&);

  // Transaction starts are handled outside the event queue.
  // Caller must hold transaction_mutex_
  void DoStartTransaction(SpiDevice* new_device);

 private:
  spi_inst_t* const spi_;
  DmaController* const dma_;
  const int actual_frequency_;
  TaskHandle_t task_;

  // Primary event queue.
  QueueHandle_t event_queue_;
  // Held for the duration of an `SpiTransaction`. Note that as it's a mutex
  // with priority inheritance (which is important in this case!), it must be
  // returned by the same task that took it.
  SemaphoreHandle_t transaction_mutex_;
  // Synchronizes with SpiTransaction::Flush().
  SemaphoreHandle_t flush_sem_;

  // Used for consistency checks.
  SpiDevice* active_device_;
  std::optional<gpio_pin_t> mosi_;
  std::optional<gpio_pin_t> miso_;
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
    // Optional buffer to transmit.
    const void* tx_buf;
    // Optional buffer to hold received data.
    void* rx_buf;
    // Number of bytes to transfer.
    uint32_t len;
  };

  enum class Result {
    OK = 0,
    // Timed out waiting for space in the event queue. The operation will not be
    // executed.
    ENQUEUE_TIMEOUT,
    // Timed out waiting for the operation to finish. It will still be executed
    // at some point in the future.
    EXEC_TIMEOUT,
  };

 public:
  // Wait for all pending transfers to complete and releases the transaction's
  // lock on the SPI bus.
  ~SpiTransaction();
  // Moveable, but not copyable.
  SpiTransaction(SpiTransaction&&);
  // Not stompable.
  SpiTransaction& operator=(SpiTransaction&&) = delete;

  // Attempts to queue the given transfer, then returns without
  // waiting for it to complete. See `SpiDevice::Transfer` for options.
  // If `ticks_to_wait` is set to 0, does not block waiting for space in the
  // queue and returns immediately.
  //
  // If set, `msg.tx_buf` and `msg.rx_buf` MUST remain valid until the transfer
  // is complete.
  //
  // Returns `OK` if the message was enqueued, `ENQUEUE_TIMEOUT` otherwise.
  Result Transfer(const TransferConfig& req,
                  TickType_t ticks_to_wait = portMAX_DELAY);

  // Waits until there is space in the queue, enqueues the given
  // transfer, and waits until it completes. Yields to the scheduler
  // while waiting. Note that if a nonblocking transfer was previously queued,
  // this one will be queued behind it and must wait for it to complete.
  // Not thread-safe.
  //
  // Transfer() is more efficient than TransferBlocking(). If you don't need to
  // do something after the transfer finishes but before ending the transaction,
  // use Transfer() instead.
  //
  // Returns `OK` if the transfer was fully completed.
  Result TransferBlocking(const TransferConfig& req,
                          TickType_t ticks_to_wait_enqueue = portMAX_DELAY,
                          TickType_t ticks_to_wait_transmit = portMAX_DELAY);

  // Returns OK if transfers so far have been completed.
  // Returns ENQUEUE_TIMEOUT if the event queue was too full to receive the
  // flush event. In this case, the flush will not be executed. Otherwise
  // returns EXEC_TIMEOUT. In this case, the flush remains pending and you may
  // attempt to wait for it again. Any additional transfers queued up in the
  // interim will also be flushed by this second (3rd, 4th, etc) Flush call.
  // Note that any pending flushes will be completed in the destructor
  // regardless. A transfer cannot outlive its containing transaction.
  Result Flush(TickType_t ticks_to_wait_enqueue = portMAX_DELAY,
               TickType_t ticks_to_wait_flush = portMAX_DELAY);

  // Waits until all pending transfers are complete and ends the transaction,
  // releasing its lock on the bus. Calling any other method on this object
  // after Dispose() returns is an error.
  void Dispose();

 private:
  SpiTransaction(const SpiTransaction&) = delete;
  SpiTransaction& operator=(const SpiTransaction&) = delete;

  explicit SpiTransaction(SpiDevice* device);

 private:
  bool moved_from_;
  SpiDevice* const device_;
  bool flush_pending_;

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

  DmaProgram transfer_program_;
  std::optional<SpiTransaction> txn_;
  // A semaphore used in the implementation of TransmitBlocking. SpiTransaction
  // uses these, but since only one transaction can be active on a device at a
  // time, that is fine.
  // TODO: It would be more efficient to use a direct-to-task notification!
  SemaphoreHandle_t blocking_sem_;
  // TODO: It would be more efficient to use a direct-to-task notification!
  SemaphoreHandle_t end_transaction_sem_;
};

std::ostream& operator<<(std::ostream&, const SpiTransaction::Result&);

}  // namespace tplp

#endif  // TPLP_BUS_SPICONTROLLER_H_