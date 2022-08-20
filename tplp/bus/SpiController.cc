#include "tplp/bus/SpiController.h"

#include <cstdio>
#include <experimental/source_location>
#include <iomanip>
#include <string>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/semphr.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "picolog/picolog.h"
#include "tplp/bus/DmaController.h"
#include "tplp/rtos_util.h"

namespace tplp {
struct SpiController::Event {
  enum class Tag { NOT_INITIALIZED = 0, TRANSFER, FLUSH, END_TRANSACTION } tag;

  // For TRANSFER events.
  SpiDevice* device;
  const uint8_t* tx_buf = nullptr;
  uint8_t* rx_buf = nullptr;
  size_t len = 0;
  // Waited on by SpiTransaction::TransferBlocking
  SemaphoreHandle_t txn_unblock_sem = nullptr;

  // Semaphore used in the END_TRANSACTION flow to signal the user's
  // transaction task that it can now give up SpiController::transaction_mutex_.
  // We could use this field for FLUSH too, but it's okay for flush to
  // use the single shared binary semaphore SpiController::flush_sem_ (because
  // flush does not require priority inheritance on top of that provided by
  // transaction_mutex_).
  SemaphoreHandle_t end_transaction_sem;
};

namespace {
// This limit is mostly arbitrary.
static constexpr int kEventQueueDepth = 8;
}  // namespace

SpiController* SpiController::Init(int priority, int stack_depth,
                                   spi_inst_t* spi, int freq_hz,
                                   gpio_pin_t sclk,
                                   std::optional<gpio_pin_t> mosi,
                                   std::optional<gpio_pin_t> miso,
                                   DmaController* dma) {
  const int spi_index = spi_get_index(spi);
  const int actual_freq_hz = spi_init(spi, freq_hz);
  LOG(INFO) << "SPI" << spi_index << " clock set to " << actual_freq_hz << "Hz";
  spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
  gpio_set_function(sclk, GPIO_FUNC_SPI);

  QueueHandle_t event_queue =
      CHECK_NOTNULL(xQueueCreate(kEventQueueDepth, sizeof(Event)));

  if (mosi) {
    gpio_set_function(*mosi, GPIO_FUNC_SPI);
  }
  if (miso) {
    gpio_set_function(*miso, GPIO_FUNC_SPI);
  }

  SemaphoreHandle_t transaction_mutex = CHECK_NOTNULL(xSemaphoreCreateMutex());
  SemaphoreHandle_t flush_sem = CHECK_NOTNULL(xSemaphoreCreateBinary());
  SpiController* that =
      new SpiController(spi, dma, actual_freq_hz, transaction_mutex,
                        event_queue, flush_sem, mosi, miso);

  std::ostringstream task_name;
  task_name << "SPI" << spi_get_index(spi);
  CHECK(xTaskCreate(&SpiController::TaskFn, task_name.str().c_str(),
                    stack_depth, that, priority, &that->task_));
  LOG(INFO) << "SPI" << spi_get_index(spi) << " initialization complete.";
  return that;
}

SpiController::SpiController(spi_inst_t* spi, DmaController* dma,
                             int actual_frequency,
                             SemaphoreHandle_t transaction_mutex,
                             QueueHandle_t event_queue,
                             SemaphoreHandle_t flush_sem,
                             std::optional<gpio_pin_t> mosi,
                             std::optional<gpio_pin_t> miso)
    : spi_(spi),
      dma_(dma),
      actual_frequency_(actual_frequency),
      event_queue_(event_queue),
      transaction_mutex_(transaction_mutex),
      flush_sem_(flush_sem),
      active_device_(nullptr),
      mosi_(mosi),
      miso_(miso) {}

SpiDevice* SpiController::AddDevice(gpio_pin_t cs, std::string_view name) {
  gpio_init(cs);
  gpio_set_dir(cs, GPIO_OUT);
  gpio_put(cs, 1);
  return new SpiDevice(this, cs, name);
}

SpiDevice::SpiDevice(SpiController* spi, gpio_pin_t cs, std::string_view name)
    : spi_(spi), cs_(cs), name_(name) {
  blocking_sem_ = xSemaphoreCreateBinary();
  end_transaction_sem_ = xSemaphoreCreateBinary();
}

SpiTransaction SpiDevice::StartTransaction() {
  auto txn = StartTransaction(portMAX_DELAY);
  LOG_IF(FATAL, !txn) << "Unable to get SpiTransaction after waiting forever?";
  return std::move(*txn);
}

void SpiController::TaskFn(void* task_param) {
  SpiController* self = static_cast<SpiController*>(task_param);
  LOG(INFO) << "SpiController task started.";
  CHECK_EQ(self->task_, xTaskGetCurrentTaskHandle());

  Event event;
  for (;;) {
    VLOG(1) << "SpiController waiting for event.";
    CHECK(xQueueReceive(self->event_queue_, &event, portMAX_DELAY));
    switch (event.tag) {
      case Event::Tag::TRANSFER:
        self->DoTransfer(event);
        break;
      case Event::Tag::FLUSH:
        self->DoFlush(event);
        break;
      case Event::Tag::END_TRANSACTION:
        self->DoEndTransaction(event);
        break;
      default:
        LOG(FATAL) << "Uninitialized/corrupt event. tag="
                   << static_cast<int>(event.tag);
    }
  }
}

void SpiController::DoStartTransaction(SpiDevice* new_device) {
  CHECK_EQ(xSemaphoreGetMutexHolder(transaction_mutex_),
           xTaskGetCurrentTaskHandle());
  CHECK_EQ(uxQueueMessagesWaiting(event_queue_), 0u);
  CHECK_EQ(active_device_, nullptr)
      << "DoStartTransaction() for device=" << new_device->name_
      << " while a transaction is already in progress on device="
      << active_device_->name_;
  CHECK_EQ(new_device->spi_, this);

  // Select the device.
  active_device_ = new_device;
  VLOG(1) << "Active device=" << new_device->name_ << "; setting CS low";
  gpio_put(new_device->cs_, 0);
}

void SpiController::DoTransfer(const Event& event) {
  static uint32_t kDummyTxBuffer = 0x0a;
  static uint32_t kDummyRxBuffer = 0xa0;

  VLOG(1) << "TRANSFER device=" << event.device->name_;
  CHECK_EQ(active_device_, event.device)
      << "Got transfer for device=" << event.device->name_
      << " while a transaction is already in progress on device="
      << active_device_->name_;
  CHECK_EQ(event.device->spi_, this);

  CHECK(!spi_is_busy(spi_));

  if (event.len == 0) {
    VLOG(1) << "Skipping zero-length transfer";
    return;
  }

  volatile io_rw_32* spi_dr = &spi_get_hw(spi_)->dr;
  DmaController::Request req{
      // Both are always enabled, since our transfer are always full-duplex.
      .tx_enable = true,
      .tx_dreq = spi_get_dreq(spi_, true),
      .tx_write = spi_dr,
      .tx_write_incr = false,

      .rx_enable = true,
      .rx_dreq = spi_get_dreq(spi_, false),
      .rx_read = spi_dr,
      .rx_read_incr = false,

      // TODO: increase transfer width (8b,16b,32b) if buf is big and the right
      // size? alternatively, change tx_buf/rx_buf to void* and let caller
      // choose.
      .transfer_width = DmaController::TransferWidth::k8,
      .transfer_count = event.len,

      .action =
          {
              .notify_task = task_,
              .notify_action = eIncrement,
          },
  };

  if (event.tx_buf) {
    CHECK(mosi_)
        << "Cannot transmit on an SpiController with no configured MOSI pin";
    req.tx_read = event.tx_buf;
    req.tx_read_incr = true;
  } else {
    req.tx_read = &kDummyTxBuffer;
    req.tx_read_incr = false;
  }

  if (event.rx_buf) {
    CHECK(miso_)
        << "Cannot receive on an SpiController with no configured MISO pin";
    req.rx_write = event.rx_buf;
    req.rx_write_incr = true;
  } else {
    req.rx_write = &kDummyRxBuffer;
    req.rx_write_incr = false;
  }
  // Start TX and RX simultaneously so the SPI FIFOs don't overflow.
  CHECK_NOTNULL(dma_)->Transfer(req);

  // Wait for the DMA to complete before processing any more events. Set a
  // generous timeout- no reasonable DMA or SPI transfer should take longer.
  CHECK_EQ(1u, ulTaskNotifyTake(true, MillisToTicks(5'000)))
      << "missed a DMA notification?";
  // Since our notification comes only after the RX DMA has finished, the SPI's
  // FIFOs should already be empty by the time.
  uint32_t fifo_status = spi_get_hw(spi_)->sr;
  CHECK(!(fifo_status & SPI_SSPSR_RNE_BITS)) << "Receive FIFO not empty";
  CHECK(fifo_status & SPI_SSPSR_TFE_BITS) << "Transmit FIFO not empty";
  CHECK(!spi_is_busy(spi_));
  VLOG(1) << "DMA transfer finished";

  if (event.txn_unblock_sem) xSemaphoreGive(event.txn_unblock_sem);
}

void SpiController::DoFlush(const Event& event) {
  VLOG(1) << "FLUSH device=" << event.device->name_;
  CHECK_EQ(active_device_, event.device)
      << "active_device_->name_=" << active_device_->name_
      << "; event.device->name_=" << event.device->name_;

  // Allow Flush() to proceed in the task that enqueued this event.
  VLOG(1) << "Returning flush_sem_";
  CHECK(xSemaphoreGive(flush_sem_));
}

void SpiController::DoEndTransaction(const Event& event) {
  VLOG(1) << "END_TRANSACTION device=" << event.device->name_;
  CHECK_EQ(active_device_, event.device)
      << "active_device_->name_=" << active_device_->name_
      << "; event.device->name_=" << event.device->name_;
  CHECK_EQ(uxQueueMessagesWaiting(event_queue_), 0u)
      << "Transaction finished but there are still events in the queue.";

  // Deselect the device.
  gpio_put(active_device_->cs_, 1);
  active_device_ = nullptr;

  // Signal the originating task to continue, so it can release
  // transaction_mutex_.
  VLOG(1) << "CS is set high; returning end_transaction_sem";
  CHECK(xSemaphoreGive(event.end_transaction_sem));
}

std::optional<SpiTransaction> SpiDevice::StartTransaction(
    TickType_t ticks_to_wait) {
  VLOG(1) << "StartTransaction() device=" << name_
          << " waiting for transaction_mutex_";
  CHECK_NE(xSemaphoreGetMutexHolder(spi_->transaction_mutex_),
           xTaskGetCurrentTaskHandle())
      << "Transaction on SPI" << spi_get_index(spi_->spi_)
      << " already active in this task - would deadlock";
  if (!xSemaphoreTake(spi_->transaction_mutex_, ticks_to_wait)) {
    // timed out
    return std::nullopt;
  }

  spi_->DoStartTransaction(this);

  return SpiTransaction(this);
}

SpiTransaction::SpiTransaction(SpiDevice* device)
    : moved_from_(false),
      device_(device),
      flush_pending_(false),
      originating_task_(CHECK_NOTNULL(xTaskGetCurrentTaskHandle())) {}

SpiTransaction::SpiTransaction(SpiTransaction&& other)
    : moved_from_(false),
      device_(other.device_),
      flush_pending_(other.flush_pending_),
      originating_task_(other.originating_task_) {
  other.moved_from_ = true;
}

SpiTransaction::Result SpiTransaction::Transfer(const TransferConfig& req,
                                                TickType_t ticks_to_wait) {
  VLOG(1) << "Transfer() device=" << device_->name_ << " len=" << req.len;
  CHECK(!moved_from_);
  CHECK_EQ(device_->spi_->active_device_, device_);
  SpiController::Event event(SpiController::Event{
      .tag = SpiController::Event::Tag::TRANSFER,
      .device = device_,
      .tx_buf = req.tx_buf,
      .rx_buf = req.rx_buf,
      .len = req.len,
  });
  if (xQueueSendToBack(device_->spi_->event_queue_, &event, ticks_to_wait)) {
    return Result::OK;
  }
  return Result::ENQUEUE_TIMEOUT;
}

SpiTransaction::Result SpiTransaction::TransferBlocking(
    const TransferConfig& req, TickType_t ticks_to_wait_enqueue,
    TickType_t ticks_to_wait_transmit) {
  VLOG(1) << "TransferBlocking() device=" << device_->name_
          << " len=" << req.len;
  CHECK(!moved_from_);
  CHECK_EQ(device_->spi_->active_device_, device_);
  SpiController::Event event(SpiController::Event{
      .tag = SpiController::Event::Tag::TRANSFER,
      .device = device_,
      .tx_buf = req.tx_buf,
      .rx_buf = req.rx_buf,
      .len = req.len,
      .txn_unblock_sem = device_->blocking_sem_,
  });
  if (!xQueueSendToBack(device_->spi_->event_queue_, &event,
                        ticks_to_wait_enqueue)) {
    return Result::ENQUEUE_TIMEOUT;
  }
  VLOG(1) << "TransferBlocking() enqueued device=" << device_->name_
          << " len=" << req.len;
  if (!xSemaphoreTake(device_->blocking_sem_, ticks_to_wait_transmit)) {
    return Result::EXEC_TIMEOUT;
  }
  VLOG(1) << "TransferBlocking() complete device=" << device_->name_
          << " len=" << req.len;
  return Result::OK;
}

SpiTransaction::~SpiTransaction() { Dispose(); }

void SpiTransaction::Dispose() {
  if (moved_from_) return;
  VLOG(1) << "Dispose() device=" << device_->name_;
  CHECK_EQ(originating_task_, xTaskGetCurrentTaskHandle())
      << "An SpiTransaction must be deleted by the same task that created it!";

  SpiController::Event event(SpiController::Event{
      .tag = SpiController::Event::Tag::END_TRANSACTION,
      .device = device_,
      .end_transaction_sem = device_->end_transaction_sem_,
  });
  VLOG(1) << "Dispose() enqueueing transaction end event";
  CHECK(xQueueSendToBack(device_->spi_->event_queue_, &event, portMAX_DELAY));

  if (flush_pending_) {
    // We need to consume any pending flush signal, otherwise the semaphore will
    // go out of sync and SpiController will eventually get stuck.
    VLOG(1) << "Dispose() waiting to take flush_sem_ for pending flush";
    CHECK(xSemaphoreTake(device_->spi_->flush_sem_, portMAX_DELAY));
  }

  // Wait for SpiController to reach the end of the queue and deselect this
  // device's CS line, before allowing another transaction to begin.
  VLOG(1) << "Dispose() waiting to take end_transaction_sem_";
  CHECK(xSemaphoreTake(device_->end_transaction_sem_, portMAX_DELAY));
  VLOG(1) << "Dispose() returning transaction_mutex_";
  CHECK(xSemaphoreGive(device_->spi_->transaction_mutex_));
  VLOG(1) << "Dispose() done";
  moved_from_ = true;
}

SpiTransaction::Result SpiTransaction::Flush(TickType_t ticks_to_wait_enqueue,
                                             TickType_t ticks_to_wait_flush) {
  VLOG(1) << "Flush() ticks_to_wait_enqueue=" << ticks_to_wait_enqueue
          << " ticks_to_wait_flush=" << ticks_to_wait_flush;
  CHECK(!moved_from_);
  if (!flush_pending_) {
    SpiController::Event event(SpiController::Event{
        .tag = SpiController::Event::Tag::FLUSH,
        .device = device_,
    });
    if (xQueueSendToBack(device_->spi_->event_queue_, &event,
                         ticks_to_wait_enqueue)) {
      VLOG(1) << "Sync event queued; waiting to take flush_sem_";
      if (xSemaphoreTake(device_->spi_->flush_sem_, ticks_to_wait_flush)) {
        VLOG(1) << "Flush OK";
        return Result::OK;
      } else {
        flush_pending_ = true;
        VLOG(1) << "flush_sem_ timed out; flush is now pending";
        return Result::EXEC_TIMEOUT;
      }
    } else {
      VLOG(1) << "flush sync event enqueue timed out";
      return Result::ENQUEUE_TIMEOUT;
    }
  } else {
    // A previously timed-out flush is still pending; wait for it to complete
    // first.
    TickType_t start = xTaskGetTickCount();
    VLOG(1) << "Waiting to take flush_sem_ for pending flush";
    if (!xSemaphoreTake(device_->spi_->flush_sem_, ticks_to_wait_flush)) {
      return Result::EXEC_TIMEOUT;
    }
    flush_pending_ = false;
    // We'll now need to catch up in case any transmits or receives were queued
    // up since the last Flush call. This isn't strictly necessary, but
    // preserves a kind of linearity guarantee for the user: after *any*
    // successful call to Flush(), all queued operations are finished.
    TickType_t end = xTaskGetTickCount();
    VLOG(1) << "Pending flush cleared. Starting another";
    return Flush(ticks_to_wait_enqueue, ticks_to_wait_flush - (end - start));
  }
}

std::ostream& operator<<(std::ostream& out, const SpiTransaction::Result& res) {
  switch (res) {
    case tplp::SpiTransaction::Result::OK:
      return out << "OK";
    case tplp::SpiTransaction::Result::ENQUEUE_TIMEOUT:
      return out << "ENQUEUE_TIMEOUT";
    case tplp::SpiTransaction::Result::EXEC_TIMEOUT:
      return out << "EXEC_TIMEOUT";
    default:
      return out << (int)res;
  }
}

}  // namespace tplp