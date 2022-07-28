#include "tplp/SpiManager.h"

#include <chrono>
#include <cstdio>
#include <experimental/source_location>
#include <string>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/semphr.h"
#include "fmt/format.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "tplp/assert.h"
#include "tplp/logging.h"
#include "tplp/time.h"

using std::chrono_literals::operator""ms;

namespace tplp {
namespace {

// We're gonna use DMA_IRQ_0 for SPI0 and DMA_IRQ_1 for SPI1,
// but this correspondence is not actually required.
dma_irq_index_t FromSpiIndex(int i) {
  switch (i) {
    case 0:
      return dma_irq_index_t::IRQ0;
    case 1:
      return dma_irq_index_t::IRQ1;
  }
  panic("FromSpiIndex(%d)\n", i);
}

uint ToIrqNum(dma_irq_index_t i) {
  switch (i) {
    case dma_irq_index_t::IRQ0:
      return DMA_IRQ_0;
    case dma_irq_index_t::IRQ1:
      return DMA_IRQ_1;
  }
  panic("ToIrqNum(%d)\n", i);
}

struct TransferRequestMessage {
  static QueueHandle_t queue;
  static constexpr int kTxQueueDepth = 3;

  bool transmit;  // if false, receive
  SpiDevice* device;
  uint8_t* buf;
  size_t len;
  SpiDevice::transmit_callback_t callback;
};

void InitGlobalsIfNecessary() {
  static bool is_init = false;
  if (is_init) return;

  TransferRequestMessage::queue = xQueueCreate(
      TransferRequestMessage::kTxQueueDepth, sizeof(TransferRequestMessage));
}

// Notification index 0 is reserved by the FreeRTOS message buffer
// implementation.
static constexpr int kTransferDoneNotificationIndex = 1;

class TransferDone {
 private:
  struct DeviceInfo {
    dma_channel_t dma_tx;
    TaskHandle_t task;
  };
  static constexpr int kMaxDevices = 3;
  static DeviceInfo devices[kMaxDevices];
  static int num_devices;

 public:
  template <int irq_index>
  static void ISR() {
    static_assert(irq_index == 0 || irq_index == 1, "bad irq_index");
    for (int i = 0; i < num_devices; ++i) {
      if (dma_irqn_get_channel_status(irq_index, devices[i].dma_tx)) {
        dma_irqn_acknowledge_channel(irq_index, devices[i].dma_tx);
        BaseType_t higher_priority_task_woken;
        vTaskNotifyGiveIndexedFromISR(devices[i].task,
                                      kTransferDoneNotificationIndex,
                                      &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
        return;
      }
    }
  }

  static void RegisterDevice(dma_channel_t dma_tx, TaskHandle_t task) {
    tplp_assert(num_devices < kMaxDevices);
    devices[num_devices++] = {.dma_tx = dma_tx, .task = task};
  }
};

QueueHandle_t TransferRequestMessage::queue = nullptr;
int TransferDone::num_devices = 0;
TransferDone::DeviceInfo TransferDone::devices[] = {};
template void TransferDone::ISR<0>();
template void TransferDone::ISR<1>();

}  // namespace

SpiManager* SpiManager::Init(int task_priority, spi_inst_t* spi, int freq_hz,
                             gpio_pin_t sclk, gpio_pin_t mosi,
                             gpio_pin_t miso) {
  InitGlobalsIfNecessary();

  int actual_freq_hz = spi_init(spi, freq_hz);
  DebugLog("SPI{} clock set to {} Hz", spi_get_index(spi), actual_freq_hz);
  spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

  gpio_set_function(sclk, GPIO_FUNC_SPI);

  dma_channel_t dma_tx = kDmaChannelInvalid;
  if (mosi) {
    gpio_set_function(mosi, GPIO_FUNC_SPI);
    dma_tx = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_tx);
    channel_config_set_dreq(&c, spi_get_dreq(spi, /*is_tx=*/true));
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_irq_quiet(&c, false);
    dma_channel_configure(dma_tx, &c, /*write_addr=*/&spi_get_hw(spi)->dr,
                          /*read_addr=*/nullptr,
                          /*transfer_count=*/0,
                          /*trigger=*/false);
    DebugLog("DMA channel {} configured for SPI{} TX", dma_tx,
             spi_get_index(spi));
  }
  if (miso) {
    // TODO: dma_rx not implemented
    panic("dma_rx not implemented");
  }
  dma_irq_index_t irq_index = FromSpiIndex(spi_get_index(spi));
  switch (irq_index) {
    case dma_irq_index_t::IRQ0:
      irq_add_shared_handler(ToIrqNum(irq_index), &TransferDone::ISR<0>,
                             PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
      break;
    case dma_irq_index_t::IRQ1:
      irq_add_shared_handler(ToIrqNum(irq_index), &TransferDone::ISR<1>,
                             PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
      break;
  };
  irq_set_enabled(ToIrqNum(irq_index), true);
  dma_channel_set_irq0_enabled(dma_tx, true);

  SpiManager* that = new SpiManager(spi, irq_index, dma_tx, actual_freq_hz);

  // task_name remains allocated forever
  std::string* task_name =
      new std::string(fmt::format("SpiManager{}", spi_get_index(spi)));
  xTaskCreate(&SpiManager::TaskFn, task_name->c_str(),
              TplpConfig::kDefaultTaskStackSize, nullptr, task_priority,
              &that->task_);
  return that;
}

SpiManager::SpiManager(spi_inst_t* spi, dma_irq_index_t dma_irq_index,
                       dma_channel_t dma_tx, int actual_frequency)
    : spi_(spi),
      dma_irq_index_(dma_irq_index),
      dma_tx_(dma_tx),
      actual_frequency_(actual_frequency) {}

void SpiManager::TaskFn(void*) {
  TransferRequestMessage request;
  DebugLog("SpiManager task started.");
  for (;;) {
    // TODO: memcpy'ing a std::function is not STRICTLY SPEAKING safe
    while (!xQueueReceive(TransferRequestMessage::queue, &request,
                          as_ticks(1'000ms))) {
      // just keep waiting for a request
      DebugLog("SpiManager task idle.");
    }
    if (!request.transmit) panic("rx not implemented");
    gpio_put(request.device->cs_, 0);
    dma_channel_transfer_from_buffer_now(request.device->spi_->dma_tx_,
                                         request.buf, request.len);
    while (!ulTaskNotifyTakeIndexed(kTransferDoneNotificationIndex, true,
                                    as_ticks(1'000ms))) {
      // TODO: transfer *still* not done...? what do? abort it?
      DebugLog("SPI transfer still not finished. CS={}, len={}",
               request.device->cs_, request.len);
    }

    // Make sure the SPI TX FIFO is fully drained before we deselect the chip
    // and potentially start another transfer on this bus. The FIFO is only 8
    // 16-bit words deep (and we only use 8 bits of each), so this shouldn't
    // take too long. Not a critical section.
    while (spi_is_busy(request.device->spi_->spi_)) tight_loop_contents();
    gpio_put(request.device->cs_, 1);
    if (request.callback) request.callback();
  }
}

SpiDevice* SpiManager::AddDevice(gpio_pin_t cs) {
  gpio_init(cs);
  gpio_set_dir(cs, GPIO_OUT);
  gpio_put(cs, 1);
  TransferDone::RegisterDevice(dma_tx_, task_);
  return new SpiDevice(this, cs);
}

SpiDevice::SpiDevice(SpiManager* spi, gpio_pin_t cs)
    : spi_(spi),
      cs_(cs),
      transmit_blocking_mutex_([]() { return xSemaphoreCreateBinary(); }) {}

bool SpiDevice::Transmit(const uint8_t* buf, uint32_t len,
                         TickType_t ticks_to_wait,
                         const transmit_callback_t& callback) {
  TransferRequestMessage req = {.transmit = true,
                                .device = this,
                                .buf = const_cast<uint8_t*>(buf),
                                .len = len,
                                .callback = callback};
  // TODO: memcpy'ing a std::function is not STRICTLY SPEAKING safe
  return xQueueSend(TransferRequestMessage::queue, &req, ticks_to_wait);
}

void SpiDevice::TransmitBlocking(const uint8_t* buf, uint32_t len) {
  tplp_assert(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);
  tplp_assert(xTaskGetCurrentTaskHandle());
  SemaphoreHandle_t sem = transmit_blocking_mutex_.get_or_init();
  Transmit(buf, len, portMAX_DELAY,
           [sem]() { xSemaphoreGive(static_cast<SemaphoreHandle_t>(sem)); });
  xSemaphoreTake(sem, portMAX_DELAY);
}

}  // namespace tplp