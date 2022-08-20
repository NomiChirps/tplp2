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
#include "tplp/bus/dma.h"
#include "tplp/rtos_util.h"

namespace tplp {
SpiController* SpiController::Init(spi_inst_t* spi, int freq_hz,
                                   gpio_pin_t sclk,
                                   std::optional<gpio_pin_t> mosi,
                                   std::optional<gpio_pin_t> miso,
                                   DmaController* dma) {
  const int spi_index = spi_get_index(spi);
  const int actual_freq_hz = spi_init(spi, freq_hz);
  LOG(INFO) << "SPI" << spi_index << " clock set to " << actual_freq_hz << "Hz";
  spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
  gpio_set_function(sclk, GPIO_FUNC_SPI);

  if (mosi) {
    gpio_set_function(*mosi, GPIO_FUNC_SPI);
  }
  if (miso) {
    gpio_set_function(*miso, GPIO_FUNC_SPI);
  }

  SemaphoreHandle_t transaction_mutex = CHECK_NOTNULL(xSemaphoreCreateMutex());
  SpiController* that = new SpiController(spi, dma, actual_freq_hz,
                                          transaction_mutex, mosi, miso);

  LOG(INFO) << "SPI" << spi_get_index(spi) << " initialization complete.";
  return that;
}

SpiController::SpiController(spi_inst_t* spi, DmaController* dma,
                             int actual_frequency,
                             SemaphoreHandle_t transaction_mutex,
                             std::optional<gpio_pin_t> mosi,
                             std::optional<gpio_pin_t> miso)
    : spi_(spi),
      dma_(dma),
      actual_frequency_(actual_frequency),
      transaction_mutex_(transaction_mutex) {}

SpiDevice* SpiController::AddDevice(gpio_pin_t cs, std::string_view name) {
  gpio_init(cs);
  gpio_set_dir(cs, GPIO_OUT);
  gpio_put(cs, 1);
  return new SpiDevice(this, cs, name);
}

SpiDevice::SpiDevice(SpiController* spi, gpio_pin_t cs, std::string_view name)
    : spi_(spi), cs_(cs), name_(name) {
  blocking_sem_ = xSemaphoreCreateBinary();
}

SpiTransaction SpiDevice::StartTransaction() {
  auto txn = StartTransaction(portMAX_DELAY);
  LOG_IF(FATAL, !txn) << "Unable to get SpiTransaction after waiting forever?";
  return std::move(*txn);
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

  // Select the device.
  VLOG(1) << "Active device=" << name_ << "; setting CS low";
  gpio_put(cs_, 0);

  return SpiTransaction(this);
}

SpiTransaction::SpiTransaction(SpiDevice* device)
    : moved_from_(false),
      device_(device),
      originating_task_(CHECK_NOTNULL(xTaskGetCurrentTaskHandle())) {}

SpiTransaction::SpiTransaction(SpiTransaction&& other)
    : moved_from_(false),
      device_(other.device_),
      originating_task_(other.originating_task_) {
  other.moved_from_ = true;
}

void SpiTransaction::TransferBlocking(const TransferConfig& req) {
  VLOG(1) << "TransferBlocking() device=" << device_->name_
          << " len=" << req.len;

  static uint32_t kDummyTxBuffer = 0x0a;
  static uint32_t kDummyRxBuffer = 0xa0;

  CHECK(!spi_is_busy(device_->spi_->spi_));

  if (req.len == 0) {
    VLOG(1) << "Skipping zero-length transfer";
    return;
  }

  volatile io_rw_32* spi_dr = &spi_get_hw(device_->spi_->spi_)->dr;
  DmaController::Request dma_req{
      // Both are always enabled, since our transfer are always full-duplex.
      .tx_enable = true,
      .tx_dreq = spi_get_dreq(device_->spi_->spi_, true),
      .tx_write = spi_dr,
      .tx_write_incr = false,

      .rx_enable = true,
      .rx_dreq = spi_get_dreq(device_->spi_->spi_, false),
      .rx_read = spi_dr,
      .rx_read_incr = false,

      // TODO: increase transfer width (8b,16b,32b) if buf is big and the right
      // size? alternatively, change tx_buf/rx_buf to void* and let caller
      // choose.
      .transfer_width = DmaController::TransferWidth::k8,
      .transfer_count = req.len,

      .action = {.give_semaphore = device_->blocking_sem_},
  };

  if (req.tx_buf) {
    dma_req.tx_read = req.tx_buf;
    dma_req.tx_read_incr = true;
  } else {
    dma_req.tx_read = &kDummyTxBuffer;
    dma_req.tx_read_incr = false;
  }

  if (req.rx_buf) {
    dma_req.rx_write = req.rx_buf;
    dma_req.rx_write_incr = true;
  } else {
    dma_req.rx_write = &kDummyRxBuffer;
    dma_req.rx_write_incr = false;
  }
  // Start TX and RX simultaneously so the SPI FIFOs don't overflow.
  device_->spi_->dma_->Transfer(dma_req);

  CHECK(xSemaphoreTake(device_->blocking_sem_, portMAX_DELAY));
  VLOG(1) << "TransferBlocking() complete device=" << device_->name_
          << " len=" << req.len;
}

SpiTransaction::~SpiTransaction() { Dispose(); }

void SpiTransaction::Dispose() {
  if (moved_from_) return;
  CHECK_EQ(originating_task_, xTaskGetCurrentTaskHandle())
      << "An SpiTransaction must be deleted by the same task that created it!";

  // Deselect the device.
  VLOG(1) << "Dispose() returning transaction_mutex_; setting CS low";
  gpio_put(device_->cs_, 1);
  CHECK(xSemaphoreGive(device_->spi_->transaction_mutex_));

  moved_from_ = true;
}

}  // namespace tplp