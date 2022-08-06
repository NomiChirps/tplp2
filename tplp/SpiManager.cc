#include "tplp/SpiManager.h"

#include <chrono>
#include <cstdio>
#include <experimental/source_location>
#include <string>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/semphr.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "picolog/picolog.h"
#include "tplp/time.h"
#include "tplp/tplp_config.h"

using std::chrono_literals::operator""ms;

namespace tplp {

struct SpiManager::TransmitEvent {
  SpiDevice* device;
  const uint8_t* buf;
  size_t len;
  std::function<void()> run_before;
  std::function<void()> run_after;
};

struct SpiManager::ReceiveEvent {
  //
};

struct SpiManager::EndTransactionEvent {
  SpiDevice* device;
};

struct SpiManager::Event {
  Event() : tag(Tag::NOT_INITIALIZED), body() {}
  Event(const SpiManager::TransmitEvent& e) : tag(Tag::TRANSMIT), body(e) {}
  Event(const SpiManager::ReceiveEvent& e) : tag(Tag::RECEIVE), body(e) {}
  Event(const SpiManager::EndTransactionEvent& e)
      : tag(Tag::END_TRANSACTION), body(e) {}
  ~Event();

  enum class Tag {
    NOT_INITIALIZED = 0,
    TRANSMIT,
    RECEIVE,
    END_TRANSACTION
  } tag;

  union Body {
    struct Empty {
    } not_initialized;
    SpiManager::TransmitEvent transmit;
    SpiManager::ReceiveEvent receive;
    SpiManager::EndTransactionEvent end_transaction;

   private:
    Body() : not_initialized() {}
    Body(const SpiManager::TransmitEvent& e) : transmit(e) {}
    Body(const SpiManager::ReceiveEvent& e) : receive(e) {}
    Body(const SpiManager::EndTransactionEvent& e) : end_transaction(e) {}
    ~Body() {}  // handled by ~Event
    friend Event;
  } body;
};

SpiManager::Event::~Event() {
  switch (tag) {
    case Tag::NOT_INITIALIZED:
      body.not_initialized.~Empty();
      return;
    case Tag::TRANSMIT:
      body.transmit.~TransmitEvent();
      return;
    case Tag::RECEIVE:
      body.receive.~ReceiveEvent();
      return;
    case Tag::END_TRANSACTION:
      body.end_transaction.~EndTransactionEvent();
      return;
  }
  LOG(FATAL) << "Unexpected/corrupt Event tag " << static_cast<int>(tag);
}

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
    // Transmit DMA channel
    std::optional<dma_channel_t> dma_tx;
    // TODO: receive
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
      if (devices[irq_index][i].dma_tx) {
        if (dma_irqn_get_channel_status(irq_index,
                                        *devices[irq_index][i].dma_tx)) {
          dma_irqn_acknowledge_channel(irq_index,
                                       *devices[irq_index][i].dma_tx);
          vTaskNotifyGiveIndexedFromISR(devices[irq_index][i].task,
                                        kDmaFinishedNotificationIndex,
                                        &higher_priority_task_woken);
          break;
        }
      }
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }

  static void RegisterDevice(dma_irq_index_t irq_index,
                             std::optional<dma_channel_t> dma_tx,
                             TaskHandle_t task) {
    CHECK_GE(irq_index, 0);
    CHECK_LT(irq_index, kNumDmaIrqs);
    CHECK_LT(num_devices[irq_index], kMaxDevicesPerDmaIrq)
        << "Too many devices assigned to the SpiManager associated with DMA "
        << irq_index;
    devices[irq_index][num_devices[irq_index]++] = {.dma_tx = dma_tx,
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

}  // namespace

SpiManager* SpiManager::Init(int task_priority, spi_inst_t* spi, int freq_hz,
                             gpio_pin_t sclk, std::optional<gpio_pin_t> mosi,
                             std::optional<gpio_pin_t> miso) {
  int actual_freq_hz = spi_init(spi, freq_hz);
  LOG(INFO) << "SPI" << spi_get_index(spi) << " clock set to " << actual_freq_hz
            << "Hz";
  spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
  gpio_set_function(sclk, GPIO_FUNC_SPI);

  QueueHandle_t event_queue =
      CHECK_NOTNULL(xQueueCreate(kEventQueueDepth, sizeof(Event)));

  std::optional<dma_channel_t> dma_tx;
  if (mosi) {
    gpio_set_function(*mosi, GPIO_FUNC_SPI);
    dma_tx = dma_channel_t(dma_claim_unused_channel(false));
    LOG_IF(FATAL, dma_tx < 0) << "No free DMA channels available!";
    dma_channel_config c = dma_channel_get_default_config(*dma_tx);
    channel_config_set_dreq(&c, spi_get_dreq(spi, /*is_tx=*/true));
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_irq_quiet(&c, false);
    dma_channel_configure(*dma_tx, &c, /*write_addr=*/&spi_get_hw(spi)->dr,
                          /*read_addr=*/nullptr,
                          /*transfer_count=*/0,
                          /*trigger=*/false);
    LOG(INFO) << "DMA channel " << *dma_tx << " configured for SPI"
              << spi_get_index(spi) << " TX";
  }
  if (miso) {
    // TODO: dma_rx not implemented
    LOG(FATAL) << "dma_rx not implemented";
  }
  int spi_index = spi_get_index(spi);
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
  SpiManager* that =
      new SpiManager(spi, irq_index, irq_number, dma_tx, actual_freq_hz,
                     transaction_mutex, event_queue);

  // task_name remains allocated forever
  char* task_name = new char[16];
  snprintf(task_name, 16, "SpiManager%d", spi_get_index(spi));
  CHECK(xTaskCreate(&SpiManager::TaskFn, task_name, TaskStacks::kDefault, that,
                    task_priority, &that->task_));
  LOG(INFO) << "SPI" << spi_get_index(spi) << " initialization complete.";
  return that;
}

SpiManager::SpiManager(spi_inst_t* spi, dma_irq_index_t dma_irq_index,
                       dma_irq_number_t dma_irq_number,
                       std::optional<dma_channel_t> dma_tx,
                       int actual_frequency,
                       SemaphoreHandle_t transaction_mutex,
                       QueueHandle_t event_queue)
    : spi_(spi),
      dma_irq_index_(dma_irq_index),
      dma_irq_number_(dma_irq_number),
      dma_tx_(dma_tx),
      actual_frequency_(actual_frequency),
      transaction_mutex_(transaction_mutex),
      event_queue_(event_queue),
      active_device_(nullptr) {}

SpiDevice* SpiManager::AddDevice(gpio_pin_t cs, std::string_view name) {
  gpio_init(cs);
  gpio_set_dir(cs, GPIO_OUT);
  gpio_put(cs, 1);
  DmaFinishedNotifier::RegisterDevice(dma_irq_index_, dma_tx_, task_);
  return new SpiDevice(this, cs, name);
}

SpiDevice::SpiDevice(SpiManager* spi, gpio_pin_t cs, std::string_view name)
    : spi_(spi), cs_(cs), name_(name) {}

SpiTransaction SpiDevice::StartTransaction() {
  auto tx = StartTransaction(portMAX_DELAY);
  LOG_IF(FATAL, !tx) << "Unable to get SpiTransaction after waiting forever?";
  VLOG(1) << "StartTransaction() wrapper returning";
  return std::move(*tx);
}

void SpiManager::TaskFn(void* task_param) {
  SpiManager* self = static_cast<SpiManager*>(task_param);
  LOG(INFO) << "SpiManager task started.";

  // Enable the IRQ used for DmaFinishedNotifier.
  // TODO: set irq priority? the default middle-priority is probably fine
  if (self->dma_tx_) {
    dma_irqn_set_channel_enabled(self->dma_irq_index_, *self->dma_tx_, true);
    irq_set_enabled(self->dma_irq_number_, true);
    LOG(INFO) << "IRQ " << self->dma_irq_number_ << " enabled.";
  }

  Event event;
  for (;;) {
    VLOG(1) << "SpiManager waiting for event.";
    while (!xQueueReceive(self->event_queue_, &event, as_ticks(10'000ms))) {
      // Just keep waiting for events.
      VLOG(1) << "SpiManager" << spi_get_index(self->spi_) << " idle.";
    }
    VLOG(1) << "SpiManager got event: " << static_cast<int>(event.tag);

    switch (event.tag) {
      case Event::Tag::TRANSMIT:
        self->HandleEvent(event.body.transmit);
        break;
      case Event::Tag::RECEIVE:
        self->HandleEvent(event.body.receive);
        break;
      case Event::Tag::END_TRANSACTION:
        self->HandleEvent(event.body.end_transaction);
        break;
      default:
        LOG(ERROR) << "Uninitialized/corrupt event discarded. tag="
                   << static_cast<int>(event.tag);
    }
  }
}

void SpiManager::DoStartTransaction(SpiDevice* new_device) {
  CHECK_EQ(xSemaphoreGetMutexHolder(transaction_mutex_),
           xTaskGetCurrentTaskHandle());
  CHECK_EQ(uxQueueMessagesWaiting(event_queue_), 0u);
  CHECK_EQ(active_device_, nullptr)
      << "Got StartTransactionEvent for device=\"" << new_device->name_
      << "\" while a transaction is already in progress on device=\""
      << active_device_->name_ << "\"";
  CHECK_EQ(new_device->spi_, this);

  // Select the device.
  active_device_ = new_device;
  gpio_put(new_device->cs_, 0);
}

void SpiManager::HandleEvent(const TransmitEvent& event) {
  CHECK_EQ(active_device_, event.device)
      << "Got TransmitEvent for device=\"" << event.device->name_
      << "\" while a transaction is already in progress on device=\""
      << active_device_->name_ << "\"";
  CHECK_EQ(event.device->spi_, this);
  CHECK(event.device->spi_->dma_tx_);

  if (event.run_before) event.run_before();

  VLOG(1) << "Start DMA transfer channel=" << *event.device->spi_->dma_tx_
          << " buf=" << event.buf << " len=" << event.len;
  dma_channel_transfer_from_buffer_now(*event.device->spi_->dma_tx_, event.buf,
                                       event.len);
  // Wait for the DMA to complete before processing any more events.
  CHECK(ulTaskNotifyTakeIndexed(kDmaFinishedNotificationIndex, true,
                                portMAX_DELAY));
  VLOG(1) << "DMA finished on channel=" << *event.device->spi_->dma_tx_;

  // Make sure the SPI TX FIFO is fully drained before we call callbacks and/or
  // start another transfer on this bus. The FIFO is only 8 16-bit words deep
  // (and we only use 8 bits of each), so this shouldn't take too long. Doesn't
  // need to be a critical section.
  // TODO: Callbacks require this guarantee, but what about tx/rx?
  //       Could we proceed immediately if run_after isn't set?
  while (spi_is_busy(event.device->spi_->spi_)) tight_loop_contents();

  if (event.run_after) event.run_after();
}

void SpiManager::HandleEvent(const ReceiveEvent& event) {
  LOG(FATAL) << "not implemented";
}

void SpiManager::HandleEvent(const EndTransactionEvent& event) {
  VLOG(1) << "EndTransactionEvent device=" << event.device->name_;
  CHECK_EQ(active_device_, event.device)
      << "active_device_->name_=" << active_device_->name_
      << "; event.device->name_=" << event.device->name_;

  // Deselect the device.
  gpio_put(active_device_->cs_, 1);
  active_device_ = nullptr;

  // At this point the SpiTransaction will likely already have exited its
  // destructor, abandoning ownership of `transaction_mutex_` to us. We can
  // release it now, since all events related to this device should have
  // finished being processed.
  CHECK_EQ(uxQueueMessagesWaiting(event_queue_), 0u)
      << "Transaction finished but there are still events in the queue.";
  xSemaphoreGive(transaction_mutex_);
}

std::optional<SpiTransaction> SpiDevice::StartTransaction(
    TickType_t ticks_to_wait) {
  VLOG(1) << "StartTransaction() device=" << name_;
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
      blocking_mutex_(CHECK_NOTNULL(
          new SemaphoreHandle_t(CHECK_NOTNULL(xSemaphoreCreateBinary())))) {}

SpiTransaction::SpiTransaction(SpiTransaction&& other)
    : moved_from_(false),
      device_(other.device_),
      blocking_mutex_(std::move(other.blocking_mutex_)) {
  other.moved_from_ = true;
}

void SpiTransaction::SemaphoreDeleter::operator()(SemaphoreHandle_t* sem) {
  if (sem && *sem) vSemaphoreDelete(*sem);
  VLOG(1) << "SemaphoreDeleter OK " << sem;
}

SpiTransaction::Result SpiTransaction::Transmit(const TxMessage& msg,
                                                TickType_t ticks_to_wait) {
  VLOG(1) << "Transmit() device=" << device_->name_ << " len=" << msg.len;
  CHECK_EQ(device_->spi_->active_device_, device_);
  SpiManager::Event event(SpiManager::TransmitEvent{
      .device = device_,
      .buf = msg.buf,
      .len = msg.len,
      .run_before = msg.run_before,
      .run_after = msg.run_after,
  });
  if (xQueueSendToBack(device_->spi_->event_queue_, &event, ticks_to_wait)) {
    return Result::OK;
  }
  return Result::ENQUEUE_TIMEOUT;
}

SpiTransaction::Result SpiTransaction::TransmitBlocking(
    const TxMessage& msg, TickType_t ticks_to_wait_enqueue,
    TickType_t ticks_to_wait_transmit) {
  VLOG(1) << "TransmitBlocking() start device=" << device_->name_
          << " len=" << msg.len;
  CHECK_EQ(device_->spi_->active_device_, device_);
  SemaphoreHandle_t sem = *blocking_mutex_;
  SpiManager::Event event(SpiManager::TransmitEvent{
      .device = device_,
      .buf = msg.buf,
      .len = msg.len,
      .run_before = msg.run_before,
      .run_after =
          [sem, orig_run_after = msg.run_after]() {
            if (orig_run_after) orig_run_after();
            xSemaphoreGive(sem);
          },
  });
  if (!xQueueSendToBack(device_->spi_->event_queue_, &event,
                        ticks_to_wait_enqueue)) {
    return Result::ENQUEUE_TIMEOUT;
  }
  VLOG(1) << "TransmitBlocking() enqueued device=" << device_->name_
          << " len=" << msg.len;
  if (!xSemaphoreTake(sem, ticks_to_wait_transmit)) {
    return Result::TRANSMIT_TIMEOUT;
  }
  VLOG(1) << "TransmitBlocking() complete device=" << device_->name_
          << " len=" << msg.len;
  return Result::OK;
}

SpiTransaction::~SpiTransaction() {
  if (moved_from_) return;

  VLOG(1) << "~SpiTransaction device=" << device_->name_;
  LOG(FATAL) << "~SpiTransaction not implemented";

  // TODO: need to handle both cases: when transmits are pending and when
  // they're not. either way we need to deselect the device and release the
  // lock as soon as the queue empties.

  // XXX: idea: use QueueSets in TaskFn to wait on {transmit, receive, done}.
  // From here, send `done`. TaskFn checks all queues at the end of each
  // processing loop and does end-of-tx stuff when
}

}  // namespace tplp