#ifndef TPLP_SPIMANAGER_H_
#define TPLP_SPIMANAGER_H_

#include <memory>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"
#include "FreeRTOS/task.h"
#include "hardware/spi.h"
#include "tplp/thread_local.h"
#include "tplp/types.h"

namespace tplp {

class SpiDevice;

// Manages a number of devices on one SPI bus.
class SpiManager {
  friend class SpiDevice;

 public:
  SpiManager(const SpiManager&) = delete;
  SpiManager& operator=(const SpiManager&) = delete;

  // Initializes the SPI hardware and any necessary FreeRTOS structures.
  // `mosi` or `miso` may be 0, indicating that that channel is unused.
  // `task_priority` is a FreeRTOS priority value.
  static SpiManager* Init(int task_priority, spi_inst_t* spi, int freq_hz,
                          gpio_pin_t sclk, gpio_pin_t mosi, gpio_pin_t miso);

  // We use software chip-select (GPIO), so cs can be any pin.
  SpiDevice* AddDevice(gpio_pin_t cs);

  // Hz
  int GetActualFrequency() const { return actual_frequency_; }

 private:
  explicit SpiManager(spi_inst_t* spi, int dma_irq_index, int dma_irq_number,
                      dma_channel_t dma_tx, int actual_frequency,
                      QueueHandle_t transmit_queue);

  static void TaskFn(void*);

 private:
  spi_inst_t* const spi_;
  // Either 0 or 1.
  const int dma_irq_index_;
  // Some actual IRQ number.
  const int dma_irq_number_;
  const dma_channel_t dma_tx_;
  const int actual_frequency_;
  TaskHandle_t task_;
  QueueHandle_t transmit_queue_;
};

class SpiDevice {
  friend class SpiManager;

 public:
  using transmit_callback_t = std::function<void()>;

  SpiDevice(const SpiDevice&) = delete;
  SpiDevice& operator=(const SpiDevice&) = delete;

  // Attempts to queue the given buffer for transmission.
  // `buf` MUST remain valid until the transfer is complete.
  // `ticks_to_wait`: how long to wait if the transmit queue is full.
  // `callback`: an optional callback for when the transfer is complete. Runs
  // within a task context, not an interrupt.
  //
  // Returns: true if successful, false if we timed out waiting for space in the
  // transmit queue.
  bool Transmit(const uint8_t* buf, uint32_t len, TickType_t ticks_to_wait,
                const transmit_callback_t& callback = nullptr);

  // Waits until there is space in the transmission queue, enqueues the given
  // buffer, and waits until the transfer is complete. Yields to the scheduler
  // while waiting.
  // Returns 0 if successful, 1 if timed out enqueueing, 2 if timed out
  // transmitting (message may still be sent later in this case!).
  int TransmitBlocking(const uint8_t* buf, uint32_t len,
                       TickType_t ticks_to_wait_enqueue = portMAX_DELAY,
                       TickType_t ticks_to_wait_transmit = portMAX_DELAY);

 private:
  explicit SpiDevice(SpiManager*, gpio_pin_t);

 private:
  SpiManager* const spi_;
  const gpio_pin_t cs_;
  ThreadLocal<SemaphoreHandle_t> transmit_blocking_mutex_;
};

}  // namespace tplp

#endif  // TPLP_SPIMANAGER_H_