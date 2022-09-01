#include "tplp/bus/spi.h"

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
  SemaphoreHandle_t pending_transfers_sem =
      // maximum count is arbitrary. might as well be INT_MAX
      CHECK_NOTNULL(xSemaphoreCreateCounting(64, 0));
  SpiController* that = new SpiController(
      spi, dma, actual_freq_hz, transaction_mutex, pending_transfers_sem);

  LOG(INFO) << "SPI" << spi_get_index(spi) << " initialization complete.";
  return that;
}

SpiController::SpiController(spi_inst_t* spi, DmaController* dma,
                             int actual_frequency,
                             SemaphoreHandle_t transaction_mutex,
                             SemaphoreHandle_t pending_transfers_sem)
    : spi_(spi),
      dma_(dma),
      actual_frequency_(actual_frequency),
      transaction_mutex_(transaction_mutex),
      pending_transfers_sem_(pending_transfers_sem) {}

SpiDevice* SpiController::AddDevice(gpio_pin_t cs, std::string_view name) {
  gpio_init(cs);
  gpio_set_dir(cs, GPIO_OUT);
  gpio_put(cs, 1);
  return new SpiDevice(this, cs, name);
}

SpiDevice::SpiDevice(SpiController* spi, gpio_pin_t cs, std::string_view name)
    : spi_(spi), cs_(cs), name_(name) {}

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

  return SpiTransaction(spi_->spi_, spi_->dma_, spi_->pending_transfers_sem_,
                        cs_, spi_->transaction_mutex_);
}

SpiTransaction::SpiTransaction(spi_inst_t* spi, DmaController* dma,
                               SemaphoreHandle_t pending_transfers_sem,
                               gpio_pin_t cs,
                               SemaphoreHandle_t transaction_mutex)
    : spi_(spi),
      dma_(dma),
      pending_transfers_sem_(pending_transfers_sem),
      cs_(cs),
      transaction_mutex_(transaction_mutex),
      pending_transfer_count_(0),
      moved_from_(false) {
  CHECK_EQ(uxSemaphoreGetCount(pending_transfers_sem_), 0u);
}

SpiTransaction::SpiTransaction(SpiTransaction&& other)
    : spi_(other.spi_),
      dma_(other.dma_),
      pending_transfers_sem_(other.pending_transfers_sem_),
      cs_(other.cs_),
      transaction_mutex_(other.transaction_mutex_),
      pending_transfer_count_(other.pending_transfer_count_),
      moved_from_(false) {
  other.moved_from_ = true;
}

void SpiTransaction::Transfer(const TransferConfig& req) {
  VLOG(1) << "Transfer() trans_count=" << req.trans_count;

  static uint32_t kDummyTxBuffer = 0x0a;
  static uint32_t kDummyRxBuffer = 0xa0;

  CHECK(!spi_is_busy(spi_));

  if (req.trans_count == 0) {
    VLOG(1) << "Skipping zero-length transfer";
    return;
  }

  volatile io_rw_32* spi_dr = &spi_get_hw(spi_)->dr;
  // TODO: increase transfer width (8b,16b,32b) if buf is big and the right
  // size? alternatively, let the caller choose.
  // (is this even worth doing? would it make any difference?)
  DmaController::Request dma_req{
      // Both are always enabled, since our transfers are always full-duplex.
      .c0_enable = true,
      .c0_treq_sel = (uint8_t)spi_get_dreq(spi_, true),
      .c0_write_addr = spi_dr,
      .c0_write_incr = false,
      .c0_data_size = DmaController::DataSize::k8,
      .c0_trans_count = req.trans_count,

      .c1_enable = true,
      .c1_treq_sel = (uint8_t)spi_get_dreq(spi_, false),
      .c1_read_addr = spi_dr,
      .c1_read_incr = false,
      .c1_data_size = DmaController::DataSize::k8,
      .c1_trans_count = req.trans_count,

      .both_action =
          DmaController::Action{.toggle_gpio = req.toggle_gpio,
                                .give_semaphore = pending_transfers_sem_},
  };

  if (req.read_addr) {
    dma_req.c0_read_addr = req.read_addr;
    dma_req.c0_read_incr = true;
  } else {
    dma_req.c0_read_addr = &kDummyTxBuffer;
    dma_req.c0_read_incr = false;
  }

  if (req.write_addr) {
    dma_req.c1_write_addr = req.write_addr;
    dma_req.c1_write_incr = true;
  } else {
    dma_req.c1_write_addr = &kDummyRxBuffer;
    dma_req.c1_write_incr = false;
  }
  // Start TX and RX simultaneously so the SPI FIFOs don't overflow.
  dma_->Transfer(dma_req);
  pending_transfer_count_ += 1;

  VLOG(1) << "Transfer() queued trans_count=" << req.trans_count;
}

SpiTransaction::~SpiTransaction() { Dispose(); }

void SpiTransaction::Dispose() {
  if (moved_from_) return;

  // Wait for pending transfers.
  Flush();
  // Deselect the device.
  VLOG(1) << "Dispose() returning transaction_mutex_; setting CS high";
  gpio_put(cs_, 1);
  // Release the bus.
  CHECK(xSemaphoreGive(transaction_mutex_));

  moved_from_ = true;
}

void SpiTransaction::Flush() {
  while (pending_transfer_count_) {
    CHECK(xSemaphoreTake(pending_transfers_sem_, portMAX_DELAY));
    pending_transfer_count_--;
  }
}

}  // namespace tplp