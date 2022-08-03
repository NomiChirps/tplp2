#ifndef TPLP_SPIMANAGER_H_
#define TPLP_SPIMANAGER_H_

#include <memory>
#include <optional>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"
#include "FreeRTOS/task.h"
#include "hardware/spi.h"
#include "tplp/thread_local.h"
#include "tplp/types.h"

namespace tplp {

class SpiDevice;
class SpiTransaction;

// Manages a number of devices on one SPI bus.
// TODO: finish implementing receive mode
class SpiManager {
  friend class SpiDevice;
  friend class SpiTransaction;

 public:
  SpiManager(const SpiManager&) = delete;
  SpiManager& operator=(const SpiManager&) = delete;

  // Initializes the SPI hardware and any necessary FreeRTOS structures.
  // `task_priority` is a FreeRTOS priority value.
  static SpiManager* Init(int task_priority, spi_inst_t* spi, int freq_hz,
                          gpio_pin_t sclk, std::optional<gpio_pin_t> mosi,
                          std::optional<gpio_pin_t> miso);

  // We use software chip-select (GPIO), so cs can be any pin.
  SpiDevice* AddDevice(gpio_pin_t cs, std::string_view name);

  // Hz
  int GetActualFrequency() const { return actual_frequency_; }

 private:
  explicit SpiManager(spi_inst_t* spi, dma_irq_index_t dma_irq_index,
                      dma_irq_number_t dma_irq_number,
                      std::optional<dma_channel_t> dma_tx, int actual_frequency,
                      SemaphoreHandle_t transaction_mutex,
                      QueueHandle_t event_queue);
  static void TaskFn(void*);

  struct Event;
  struct StartTransactionEvent;
  void HandleEvent(const StartTransactionEvent&);
  struct TransmitEvent;
  void HandleEvent(const TransmitEvent&);
  struct ReceiveEvent;
  void HandleEvent(const ReceiveEvent&);
  struct EndTransactionEvent;
  void HandleEvent(const EndTransactionEvent&);

 private:
  spi_inst_t* const spi_;
  const dma_irq_index_t dma_irq_index_;
  const dma_irq_number_t dma_irq_number_;
  const std::optional<dma_channel_t> dma_tx_;
  const int actual_frequency_;
  TaskHandle_t task_;

  // Held for the duration of an `SpiTransaction`.
  SemaphoreHandle_t transaction_mutex_;
  QueueHandle_t event_queue_;
  // Only used for consistency checks.
  SpiDevice* active_device_;
};

class SpiTransaction {
  friend class SpiDevice;

 public:
  struct TxMessage {
    // Data to transmit.
    const uint8_t* buf;
    // Number of bytes in the buffer to transmit.
    uint32_t len;
    // Optional callback to be run just before setting CS active.
    std::function<void()> run_before;
    // Optional callback to be run just after setting CS inactive.
    std::function<void()> run_after;
  };

  enum class Result {
    OK = 0,
    // Timed out waiting for space in the event queue. The operation will not be
    // executed.
    ENQUEUE_TIMEOUT,
    // Timed out waiting for the operation to finish. It will still be executed
    // at some point in the future.
    TRANSMIT_TIMEOUT,
  };

 public:
  // Ends the transaction and allows the lock on the bus to be released when all
  // pending operations have completed. If you want to wait for them to finish,
  // use `Flush()`.
  ~SpiTransaction();
  // Moveable, but not copyable.
  SpiTransaction(SpiTransaction&&) = default;

  // Attempts to queue the given message for transmission, then returns without
  // waiting for it to complete. See `SpiDevice::TxMessage` for options.
  // If `ticks_to_wait` is set to 0, does not block waiting for space in the
  // queue and returns immediately.
  //
  // Returns `OK` if the event was enqueued, `ENQUEUE_TIMEOUT` otherwise.
  Result Transmit(const TxMessage& msg,
                  TickType_t ticks_to_wait = portMAX_DELAY);

  // Waits until there is space in the transmission queue, enqueues the given
  // message, and waits until the transfer is complete. Yields to the scheduler
  // while waiting. Note that if a nonblocking `Transmit` was previously queued,
  // this one will be queued behind it and must wait for it to complete.
  // Not thread-safe.
  //
  // Returns `OK` if the transmission was fully completed.
  Result TransmitBlocking(const TxMessage& msg,
                          TickType_t ticks_to_wait_enqueue = portMAX_DELAY,
                          TickType_t ticks_to_wait_transmit = portMAX_DELAY);

  // TODO: Flush()

 private:
  SpiTransaction(const SpiTransaction&) = delete;
  SpiTransaction& operator=(const SpiTransaction&) = delete;

  explicit SpiTransaction(SpiDevice* device);

 private:
  SpiDevice* const device_;

  struct SemaphoreDeleter {
    void operator()(SemaphoreHandle_t*);
  };
  // A semaphore used in the implementation of TransmitBlocking.
  std::unique_ptr<SemaphoreHandle_t, SemaphoreDeleter> blocking_mutex_;
};

class SpiDevice {
  friend class SpiManager;
  friend class SpiTransaction;

 public:
  // Locks the SPI bus to this device and returns a transaction handle that can
  // be used to send and receive messages. Returns `std::nullopt` if a lock on
  // the bus could not be obtained before `timeout` (due to other transactions
  // in progress).
  std::optional<SpiTransaction> StartTransaction(TickType_t timeout);

  // As `StartTransaction(timeout)`, but waits forever.
  SpiTransaction StartTransaction();

 private:
  explicit SpiDevice(SpiManager* spi, gpio_pin_t cs, std::string_view name);

 private:
  SpiDevice(const SpiDevice&) = delete;
  SpiDevice& operator=(const SpiDevice&) = delete;

  SpiManager* const spi_;
  const gpio_pin_t cs_;
  const std::string name_;
};

}  // namespace tplp

#endif  // TPLP_SPIMANAGER_H_