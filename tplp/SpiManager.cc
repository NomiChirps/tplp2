#include "tplp/SpiManager.h"

#include <chrono>
#include <cstdio>
#include <experimental/source_location>
#include <iomanip>
#include <string>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/semphr.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "picolog/picolog.h"
#include "tplp/config/tplp_config.h"
#include "tplp/time.h"

using std::chrono_literals::operator""ms;

namespace tplp {
struct SpiManager::Event {
  enum class Tag { NOT_INITIALIZED = 0, TRANSFER, FLUSH, END_TRANSACTION } tag;

  // For TRANSFER events.
  SpiDevice* device;
  const uint8_t* tx_buf = nullptr;
  uint8_t* rx_buf = nullptr;
  size_t len = 0;
  std::function<void()> run_before;
  std::function<void()> run_after;

  // Semaphore used in the END_TRANSACTION flow to signal the user's
  // transaction task that it can now give up SpiManager::transaction_mutex_.
  // We could use this field for FLUSH too, but it's okay for flush to
  // use the single shared binary semaphore SpiManager::flush_sem_ (because
  // flush does not require priority inheritance on top of that provided by
  // transaction_mutex_).
  SemaphoreHandle_t end_transaction_sem;
};

namespace {
// Notification index 0 is reserved by the FreeRTOS message buffer
// implementation, so we use the next one. This index of the SpiManager task is
// notified from `TransferDone::ISR` whenever a send or receive DMA finishes.
static constexpr int kDmaFinishedNotificationIndex = 1;

// The RP2040 DMA unit provides 2 IRQs to which each of the 12 DMA channels can
// be assigned.
static constexpr int kNumDmaIrqs = 2;
// This limit is arbitrary and only exists to save some static memory.
static constexpr int kMaxDevicesPerDmaIrq = 4;
// This limit is mostly arbitrary.
static constexpr int kEventQueueDepth = 8;

class DmaFinishedNotifier {
 private:
  struct DeviceInfo {
    std::optional<dma_channel_t> dma_rx;
    TaskHandle_t task;
  };
  // index: (irq, device)
  static DeviceInfo devices[kNumDmaIrqs][kMaxDevicesPerDmaIrq];
  static int num_devices[kNumDmaIrqs];

 public:
  template <int irq_index>
  static void ISR() {
    static_assert(irq_index >= 0 && irq_index < kNumDmaIrqs, "bad irq_index");
    BaseType_t higher_priority_task_woken = 0;
    for (int i = 0; i < num_devices[irq_index]; ++i) {
      if (dma_irqn_get_channel_status(irq_index,
                                      *devices[irq_index][i].dma_rx)) {
        dma_irqn_acknowledge_channel(irq_index, *devices[irq_index][i].dma_rx);
        vTaskNotifyGiveIndexedFromISR(devices[irq_index][i].task,
                                      kDmaFinishedNotificationIndex,
                                      &higher_priority_task_woken);
        break;
      }
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }

  static void RegisterDevice(dma_irq_index_t irq_index,
                             std::optional<dma_channel_t> dma_rx,
                             TaskHandle_t task) {
    CHECK_GE(irq_index, 0);
    CHECK_LT(irq_index, kNumDmaIrqs);
    CHECK_LT(num_devices[irq_index], kMaxDevicesPerDmaIrq)
        << "Too many devices assigned to the SpiManager associated with DMA "
        << irq_index;
    devices[irq_index][num_devices[irq_index]++] = {.dma_rx = dma_rx,
                                                    .task = task};
  }
};

// First index is the DMA interrupt index, 0 or 1. A value of the second index
// is assigned to an `SpiDevice` when it's created.
DmaFinishedNotifier::DeviceInfo
    DmaFinishedNotifier::devices[kNumDmaIrqs][kMaxDevicesPerDmaIrq] = {};
int DmaFinishedNotifier::num_devices[kNumDmaIrqs] = {0, 0};

template void DmaFinishedNotifier::ISR<0>();
template void DmaFinishedNotifier::ISR<1>();

static uint8_t* GetDmaReadAddress(dma_channel_t channel) {
  return reinterpret_cast<uint8_t*>(dma_channel_hw_addr(channel)->read_addr);
}
static uint8_t* GetDmaWriteAddress(dma_channel_t channel) {
  return reinterpret_cast<uint8_t*>(dma_channel_hw_addr(channel)->write_addr);
}
static uint32_t GetDmaTransferCount(dma_channel_t channel) {
  return dma_channel_hw_addr(channel)->transfer_count;
}

}  // namespace

SpiManager* SpiManager::Init(int task_priority, spi_inst_t* spi, int freq_hz,
                             gpio_pin_t sclk, std::optional<gpio_pin_t> mosi,
                             std::optional<gpio_pin_t> miso) {
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
  dma_channel_t dma_tx = dma_channel_t(dma_claim_unused_channel(false));
  LOG_IF(FATAL, dma_tx < 0) << "No free DMA channels available!";
  if (miso) {
    gpio_set_function(*miso, GPIO_FUNC_SPI);
  }
  dma_channel_t dma_rx = dma_channel_t(dma_claim_unused_channel(false));
  LOG_IF(FATAL, dma_rx < 0) << "No free DMA channels available!";

  // We're gonna use DMA_IRQ_0 for SPI0 and DMA_IRQ_1 for SPI1,
  // but this correspondence is not actually required. Each DMA channel can be
  // assigned to either IRQ.
  dma_irq_index_t irq_index;
  dma_irq_number_t irq_number;
  if (spi_index == 0) {
    irq_index = dma_irq_index_t(0);
    irq_number = dma_irq_number_t(DMA_IRQ_0);
    irq_set_exclusive_handler(irq_number, &DmaFinishedNotifier::ISR<0>);
  } else if (spi_index == 1) {
    irq_index = dma_irq_index_t(1);
    irq_number = dma_irq_number_t(DMA_IRQ_1);
    irq_set_exclusive_handler(irq_number, &DmaFinishedNotifier::ISR<1>);
  } else {
    LOG(FATAL) << "spi index out of expected range: " << spi_index;
  }
  // The IRQ will only be enabled when the task starts up.
  // There is no particularly strong reason to do it there instead of here.

  SemaphoreHandle_t transaction_mutex = CHECK_NOTNULL(xSemaphoreCreateMutex());
  SemaphoreHandle_t flush_sem = CHECK_NOTNULL(xSemaphoreCreateBinary());
  SpiManager* that =
      new SpiManager(spi, irq_index, irq_number, dma_tx, dma_rx, actual_freq_hz,
                     transaction_mutex, event_queue, flush_sem, mosi, miso);

  // task_name remains allocated forever
  char* task_name = new char[16];
  snprintf(task_name, 16, "SpiManager%d", spi_get_index(spi));
  CHECK(xTaskCreate(&SpiManager::TaskFn, task_name, TaskStacks::kDefault, that,
                    task_priority, &that->task_));
  LOG(INFO) << "SPI" << spi_get_index(spi) << " initialization complete.";
  return that;
}

static dma_channel_config MakeChannelConfig(spi_inst_t* spi, dma_channel_t dma,
                                            bool is_tx, bool read_increment,
                                            bool write_increment,
                                            bool irq_quiet) {
  dma_channel_config c = dma_channel_get_default_config(dma);
  channel_config_set_dreq(&c, spi_get_dreq(spi, is_tx));
  VLOG(1) << "DMA channel " << dma << " will be connected to SPI"
          << spi_get_index(spi) << " DREQ " << spi_get_dreq(spi, is_tx);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_read_increment(&c, read_increment);
  channel_config_set_write_increment(&c, write_increment);
  channel_config_set_irq_quiet(&c, irq_quiet);
  return c;
}

SpiManager::SpiManager(spi_inst_t* spi, dma_irq_index_t dma_irq_index,
                       dma_irq_number_t dma_irq_number, dma_channel_t dma_tx,
                       dma_channel_t dma_rx, int actual_frequency,
                       SemaphoreHandle_t transaction_mutex,
                       QueueHandle_t event_queue, SemaphoreHandle_t flush_sem,
                       std::optional<gpio_pin_t> mosi,
                       std::optional<gpio_pin_t> miso)
    : spi_(spi),
      dma_irq_index_(dma_irq_index),
      dma_irq_number_(dma_irq_number),
      dma_tx_(dma_tx),
      dma_rx_(dma_rx),
      actual_frequency_(actual_frequency),
      dma_tx_config_(MakeChannelConfig(spi, dma_tx, true, true, false, true)),
      dma_tx_null_config_(
          MakeChannelConfig(spi, dma_tx, true, false, false, true)),
      dma_rx_config_(MakeChannelConfig(spi, dma_rx, false, false, true, false)),
      dma_rx_null_config_(
          MakeChannelConfig(spi, dma_rx, false, false, false, false)),
      event_queue_(event_queue),
      transaction_mutex_(transaction_mutex),
      flush_sem_(flush_sem),
      active_device_(nullptr),
      mosi_(mosi),
      miso_(miso) {}

SpiDevice* SpiManager::AddDevice(gpio_pin_t cs, std::string_view name) {
  gpio_init(cs);
  gpio_set_dir(cs, GPIO_OUT);
  gpio_put(cs, 1);
  DmaFinishedNotifier::RegisterDevice(dma_irq_index_, dma_rx_, task_);
  return new SpiDevice(this, cs, name);
}

SpiDevice::SpiDevice(SpiManager* spi, gpio_pin_t cs, std::string_view name)
    : spi_(spi), cs_(cs), name_(name) {}

SpiTransaction SpiDevice::StartTransaction() {
  auto txn = StartTransaction(portMAX_DELAY);
  LOG_IF(FATAL, !txn) << "Unable to get SpiTransaction after waiting forever?";
  return std::move(*txn);
}

void SpiManager::TaskFn(void* task_param) {
  SpiManager* self = static_cast<SpiManager*>(task_param);
  LOG(INFO) << "SpiManager task started.";

  // Enable only the RX DMA IRQ for DmaFinishedNotifier. Since we always run in
  // full-duplex mode, with both DMAs active, we only need one to notify us that
  // the transfer is done. The receive one is better because when it's done, the
  // SPI RX FIFO will already be empty. Contrariwise, when the TX DMA is done,
  // the SPI TX FIFO will still be full.
  dma_irqn_set_channel_enabled(self->dma_irq_index_, self->dma_tx_, false);
  dma_irqn_set_channel_enabled(self->dma_irq_index_, self->dma_rx_, true);

  // TODO: set irq priority? the default middle-priority is probably fine
  irq_set_enabled(self->dma_irq_number_, true);
  LOG(INFO) << "IRQ " << self->dma_irq_number_ << " enabled.";

  Event event;
  for (;;) {
    VLOG(1) << "SpiManager waiting for event.";
    while (!xQueueReceive(self->event_queue_, &event, as_ticks(10'000ms))) {
      // Just keep waiting for events.
      VLOG(1) << "SpiManager" << spi_get_index(self->spi_) << " idle.";
    }
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

void SpiManager::DoStartTransaction(SpiDevice* new_device) {
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

void SpiManager::DoTransfer(const Event& event) {
  static uint32_t kDummyTxBuffer = 0x0a;
  static uint32_t kDummyRxBuffer = 0xa0;

  VLOG(1) << "TRANSFER device=" << event.device->name_;
  CHECK_EQ(active_device_, event.device)
      << "Got TransmitEvent for device=" << event.device->name_
      << " while a transaction is already in progress on device="
      << active_device_->name_;
  CHECK_EQ(event.device->spi_, this);

  CHECK(!dma_channel_is_busy(dma_tx_));
  CHECK(!dma_channel_is_busy(dma_rx_));
  CHECK(!spi_is_busy(spi_));

  if (event.run_before) event.run_before();

  if (event.len == 0) {
    VLOG(1)<< "Skipping zero-length transfer";
    if (event.run_after) event.run_after();
    return;
  }

  VLOG(1) << "Start DMA SPI transfer dma_tx=" << dma_tx_
          << " tx_buf=" << static_cast<const void*>(event.tx_buf)
          << " dma_rx=" << dma_rx_
          << " rx_buf=" << static_cast<const void*>(event.rx_buf)
          << " len=" << event.len;

  volatile io_rw_32* spi_dr = &spi_get_hw(spi_)->dr;
  // TODO: increase transfer width (8b,16b,32b) if buf is big and the right size?
  // alternatively, change tx_buf/rx_buf to void* and let caller choose.
  // what does the datasheet say about how long it takes to switch?
  if (event.tx_buf) {
    CHECK(mosi_)
        << "Cannot transmit on an SpiManager with no configured MOSI pin";
    dma_channel_configure(dma_tx_, &dma_tx_config_, spi_dr, event.tx_buf,
                          event.len, false);
  } else {
    dma_channel_configure(dma_tx_, &dma_tx_null_config_, spi_dr,
                          &kDummyTxBuffer, event.len, false);
  }
  if (event.rx_buf) {
    CHECK(miso_)
        << "Cannot receive on an SpiManager with no configured MISO pin";
    dma_channel_configure(dma_rx_, &dma_rx_config_, event.rx_buf, spi_dr,
                          event.len, false);
  } else {
    dma_channel_configure(dma_rx_, &dma_rx_null_config_, &kDummyRxBuffer,
                          spi_dr, event.len, false);
  }
  // Start TX and RX simultaneously so the SPI FIFOs don't overflow.
  dma_start_channel_mask((1u << dma_tx_) | (1u << dma_rx_));

  // Wait for the DMA to complete before processing any more events. Set a
  // generous timeout- no reasonable DMA or SPI transfer should take longer.
  CHECK_EQ(1u, ulTaskNotifyTakeIndexed(kDmaFinishedNotificationIndex, true,
                                       as_ticks(1'000ms)))
      << "missed a DMA notification, or timed out waiting. tx_busy="
      << dma_channel_is_busy(dma_tx_)
      << " rx_busy=" << dma_channel_is_busy(dma_rx_);
  // Since our notification comes from the RX DMA, the SPI's FIFOs should
  // already be empty by the time that DMA is finished.
  CHECK(!spi_is_busy(spi_));
  CHECK_EQ(GetDmaTransferCount(dma_tx_), 0u) << "DMA TX incomplete";
  CHECK_EQ(GetDmaTransferCount(dma_rx_), 0u) << "DMA RX incomplete";
  if (event.tx_buf) {
    CHECK_EQ(GetDmaReadAddress(dma_tx_), event.tx_buf + event.len)
        << "DMA TX misaligned?";
  }
  if (event.rx_buf) {
    CHECK_EQ(GetDmaWriteAddress(dma_rx_), event.rx_buf + event.len)
        << "DMA RX misaligned?";
  }
  VLOG(1) << "DMA transfer finished";

  if (event.run_after) event.run_after();
}

void SpiManager::DoFlush(const Event& event) {
  VLOG(1) << "FLUSH device=" << event.device->name_;
  CHECK_EQ(active_device_, event.device)
      << "active_device_->name_=" << active_device_->name_
      << "; event.device->name_=" << event.device->name_;

  // Allow Flush() to proceed in the task that enqueued this event.
  VLOG(1) << "Returning flush_sem_";
  CHECK(xSemaphoreGive(flush_sem_));
}

void SpiManager::DoEndTransaction(const Event& event) {
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
      blocking_sem_(CHECK_NOTNULL(
          new SemaphoreHandle_t(CHECK_NOTNULL(xSemaphoreCreateBinary())))),
      end_transaction_sem_(CHECK_NOTNULL(
          new SemaphoreHandle_t(CHECK_NOTNULL(xSemaphoreCreateBinary())))),
      originating_task_(CHECK_NOTNULL(xTaskGetCurrentTaskHandle())) {}

SpiTransaction::SpiTransaction(SpiTransaction&& other)
    : moved_from_(false),
      device_(other.device_),
      flush_pending_(other.flush_pending_),
      blocking_sem_(std::move(other.blocking_sem_)),
      end_transaction_sem_(std::move(other.end_transaction_sem_)),
      originating_task_(other.originating_task_) {
  other.moved_from_ = true;
}

void SpiTransaction::SemaphoreDeleter::operator()(SemaphoreHandle_t* sem) {
  if (sem && *sem) vSemaphoreDelete(*sem);
}

SpiTransaction::Result SpiTransaction::Transfer(const TransferConfig& req,
                                                TickType_t ticks_to_wait) {
  VLOG(1) << "Transfer() device=" << device_->name_ << " len=" << req.len;
  CHECK(!moved_from_);
  CHECK_EQ(device_->spi_->active_device_, device_);
  SpiManager::Event event(SpiManager::Event{
      .tag = SpiManager::Event::Tag::TRANSFER,
      .device = device_,
      .tx_buf = req.tx_buf,
      .rx_buf = req.rx_buf,
      .len = req.len,
      .run_before = req.run_before,
      .run_after = req.run_after,
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
  SemaphoreHandle_t sem = *blocking_sem_;
  SpiManager::Event event(SpiManager::Event{
      .tag = SpiManager::Event::Tag::TRANSFER,
      .device = device_,
      .tx_buf = req.tx_buf,
      .rx_buf = req.rx_buf,
      .len = req.len,
      .run_before = req.run_before,
      .run_after =
          [sem, orig_run_after = req.run_after]() {
            if (orig_run_after) orig_run_after();
            CHECK(xSemaphoreGive(sem));
          },
  });
  if (!xQueueSendToBack(device_->spi_->event_queue_, &event,
                        ticks_to_wait_enqueue)) {
    return Result::ENQUEUE_TIMEOUT;
  }
  VLOG(1) << "TransferBlocking() enqueued device=" << device_->name_
          << " len=" << req.len;
  if (!xSemaphoreTake(sem, ticks_to_wait_transmit)) {
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

  SpiManager::Event event(SpiManager::Event{
      .tag = SpiManager::Event::Tag::END_TRANSACTION,
      .device = device_,
      .end_transaction_sem = *end_transaction_sem_,
  });
  VLOG(1) << "Dispose() enqueueing transaction end event";
  CHECK(xQueueSendToBack(device_->spi_->event_queue_, &event, portMAX_DELAY));

  if (flush_pending_) {
    // We need to consume any pending flush signal, otherwise the semaphore will
    // go out of sync and SpiManager will eventually get stuck.
    VLOG(1) << "Dispose() waiting to take flush_sem_ for pending flush";
    CHECK(xSemaphoreTake(device_->spi_->flush_sem_, portMAX_DELAY));
  }

  // Wait for SpiManager to reach the end of the queue and deselect this
  // device's CS line, before allowing another transaction to begin.
  VLOG(1) << "Dispose() waiting to take end_transaction_sem_";
  CHECK(xSemaphoreTake(*end_transaction_sem_, portMAX_DELAY));
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
    SpiManager::Event event(SpiManager::Event{
        .tag = SpiManager::Event::Tag::FLUSH,
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