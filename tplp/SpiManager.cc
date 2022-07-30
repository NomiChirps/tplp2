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
#include "tplp/logging.h"
#include "tplp/time.h"

using std::chrono_literals::operator""ms;

namespace tplp {
namespace {

struct TransferRequestMessage {
  bool transmit;  // if false, receive
  SpiDevice* device;
  uint8_t* buf;
  size_t len;
  SpiDevice::transmit_callback_t callback;
};

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
  // index: (irq, device)
  static DeviceInfo devices[2][kMaxDevices];
  static int num_devices[2];

 public:
  template <int irq_index>
  static void ISR() {
    static_assert(irq_index == 0 || irq_index == 1, "bad irq_index");
    BaseType_t higher_priority_task_woken = 0;
    for (int i = 0; i < num_devices[irq_index]; ++i) {
      if (dma_irqn_get_channel_status(irq_index,
                                      devices[irq_index][i].dma_tx)) {
        dma_irqn_acknowledge_channel(irq_index, devices[irq_index][i].dma_tx);
        vTaskNotifyGiveIndexedFromISR(devices[irq_index][i].task,
                                      kTransferDoneNotificationIndex,
                                      &higher_priority_task_woken);
        break;
      }
    }
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }

  static void RegisterDevice(int irq_index, dma_channel_t dma_tx,
                             TaskHandle_t task) {
    tplp_assert(irq_index == 0 || irq_index == 1);
    tplp_assert(num_devices[irq_index] < kMaxDevices);
    devices[irq_index][num_devices[irq_index]++] = {.dma_tx = dma_tx,
                                                    .task = task};
  }
};

TransferDone::DeviceInfo TransferDone::devices[2][kMaxDevices] = {};
int TransferDone::num_devices[2] = {0, 0};
template void TransferDone::ISR<0>();
template void TransferDone::ISR<1>();

}  // namespace

SpiManager* SpiManager::Init(int task_priority, spi_inst_t* spi, int freq_hz,
                             gpio_pin_t sclk, gpio_pin_t mosi,
                             gpio_pin_t miso) {
  int actual_freq_hz = spi_init(spi, freq_hz);
  DebugLog("SPI%d clock set to %d Hz", spi_get_index(spi), actual_freq_hz);
  spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

  gpio_set_function(sclk, GPIO_FUNC_SPI);

  QueueHandle_t transmit_queue = nullptr;
  dma_channel_t dma_tx = kDmaChannelInvalid;
  if (mosi) {
    transmit_queue = xQueueCreate(TplpConfig::kSpiTransmitQueueDepth,
                                  sizeof(TransferRequestMessage));

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
    DebugLog("DMA channel %d configured for SPI%u TX", dma_tx,
             spi_get_index(spi));
  }
  if (miso) {
    // TODO: dma_rx not implemented
    panic("dma_rx not implemented");
  }
  int spi_index = spi_get_index(spi);
  // We're gonna use DMA_IRQ_0 for SPI0 and DMA_IRQ_1 for SPI1,
  // but this correspondence is not actually required.
  int irq_index;
  int irq_number;
  if (spi_index == 0) {
    irq_index = 0;
    irq_number = DMA_IRQ_0;
    irq_set_exclusive_handler(irq_number, &TransferDone::ISR<0>);
  } else if (spi_index == 1) {
    irq_index = 0;
    irq_number = DMA_IRQ_1;
    irq_set_exclusive_handler(irq_number, &TransferDone::ISR<1>);
  } else {
    panic("bad spi index");
  }
  // IRQ is enabled when the task starts up.

  SpiManager* that = new SpiManager(spi, irq_index, irq_number, dma_tx,
                                    actual_freq_hz, transmit_queue);

  // task_name remains allocated forever
  char* task_name = new char[16];
  snprintf(task_name, 16, "SpiManager%d", spi_get_index(spi));
  tplp_assert(xTaskCreate(&SpiManager::TaskFn, task_name, TaskStacks::kDefault,
                          that, task_priority, &that->task_) == pdPASS);
  return that;
}

SpiManager::SpiManager(spi_inst_t* spi, int dma_irq_index, int dma_irq_number,
                       dma_channel_t dma_tx, int actual_frequency,
                       QueueHandle_t transmit_queue)
    : spi_(spi),
      dma_irq_index_(dma_irq_index),
      dma_irq_number_(dma_irq_number),
      dma_tx_(dma_tx),
      actual_frequency_(actual_frequency),
      transmit_queue_(transmit_queue) {}

void SpiManager::TaskFn(void* task_param) {
  SpiManager* self = static_cast<SpiManager*>(task_param);

  // TODO: set irq priority? the default middle-priority is probably fine
  dma_irqn_set_channel_enabled(self->dma_irq_index_, self->dma_tx_, true);
  irq_set_enabled(self->dma_irq_number_, true);

  TransferRequestMessage request;
  DebugLog("SpiManager task started.");
  for (;;) {
    // TODO: memcpy'ing a std::function is not STRICTLY SPEAKING safe
    while (!xQueueReceive(self->transmit_queue_, &request, as_ticks(5'000ms))) {
      // just keep waiting for a request
      DebugLog("SpiManager%u idle.", spi_get_index(self->spi_));
    }
    if (!request.transmit) panic("rx not implemented");
    gpio_put(request.device->cs_, 0);
    dma_channel_transfer_from_buffer_now(request.device->spi_->dma_tx_,
                                         request.buf, request.len);
    while (!ulTaskNotifyTakeIndexed(kTransferDoneNotificationIndex, true,
                                    as_ticks(1'000ms))) {
      // TODO: transfer *still* not done...? what do? abort it?
      DebugLog("SPI transfer still not finished. device=%s, cs=%u, len=%u",
               request.device->name_, request.device->cs_, request.len);
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

SpiDevice* SpiManager::AddDevice(gpio_pin_t cs, std::string_view name) {
  gpio_init(cs);
  gpio_set_dir(cs, GPIO_OUT);
  gpio_put(cs, 1);
  TransferDone::RegisterDevice(dma_irq_index_, dma_tx_, task_);
  return new SpiDevice(this, cs, name);
}

SpiDevice::SpiDevice(SpiManager* spi, gpio_pin_t cs, std::string_view name)
    : spi_(spi),
      cs_(cs),
      name_(name),
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
  return xQueueSend(spi_->transmit_queue_, &req, ticks_to_wait);
}

int SpiDevice::TransmitBlocking(const uint8_t* buf, uint32_t len,
                                TickType_t ticks_to_wait_enqueue,
                                TickType_t ticks_to_wait_transmit) {
  tplp_assert(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);
  tplp_assert(xTaskGetCurrentTaskHandle());
  SemaphoreHandle_t sem = transmit_blocking_mutex_.get_or_init();
  if (Transmit(buf, len, ticks_to_wait_enqueue,
               [sem]() { xSemaphoreGive(sem); })) {
    if (xSemaphoreTake(sem, ticks_to_wait_transmit)) {
      return 0;
    }
    return 2;
  }
  return 1;
}

}  // namespace tplp