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

  // TODO: verify that these pins have that function
  // for the given SPI peripheral index
  if (mosi) {
    gpio_set_function(*mosi, GPIO_FUNC_SPI);
  }
  if (miso) {
    gpio_set_function(*miso, GPIO_FUNC_SPI);
  }

  SemaphoreHandle_t bus_mutex = CHECK_NOTNULL(xSemaphoreCreateMutex());
  SemaphoreHandle_t unblock_semaphore = CHECK_NOTNULL(xSemaphoreCreateBinary());
  SpiController* self =
      new SpiController(spi, dma, actual_freq_hz, bus_mutex, unblock_semaphore);

  LOG(INFO) << "SPI" << spi_get_index(spi) << " initialization complete.";
  return self;
}

SpiController::SpiController(spi_inst_t* spi, DmaController* dma,
                             int actual_frequency, SemaphoreHandle_t bus_mutex,
                             SemaphoreHandle_t unblock_semaphore)
    : spi_(spi),
      dma_(dma),
      actual_frequency_(actual_frequency),
      bus_mutex_(bus_mutex),
      unblock_semaphore_(unblock_semaphore) {}

SpiDevice* SpiController::AddDevice(gpio_pin_t cs, std::string_view name) {
  gpio_init(cs);
  gpio_set_dir(cs, GPIO_OUT);
  gpio_put(cs, 1);
  return new SpiDevice(this, cs, name);
}

SpiDevice::SpiDevice(SpiController* spi, gpio_pin_t cs, std::string_view name)
    : spi_(spi), cs_(cs), name_(name) {}

SpiTransactionBuilder::SpiTransactionBuilder(SpiDevice* device)
    : device_(device), program_(device_->spi_->dma_->NewProgram()) {}

void SpiTransactionBuilder::AddTransfer(const TransferArg& arg,
                                        std::optional<Action> before,
                                        std::optional<Action> after) {
  volatile io_rw_32* spi_dr = &spi_get_hw(device_->spi_->spi_)->dr;
  bool null_write = arg.write_addr && *arg.write_addr == nullptr;
  bool null_read = arg.read_addr && *arg.read_addr == nullptr;
  if (arg.size) CHECK_GT(*arg.size, 0u);
  // TODO: increase transfer width (8b,16b,32b) if buf is the right size
  // XXX: AddOrExtendCommand ? for chain_length > 1 ...
  program_.AddCommand(
      {
          .enable = {1, 1},
          .transfers =
              {// RX channel
               ChannelConfig{
                   .ctrl = ChannelCtrl{.data_size = ChannelCtrl::DataSize::k8,
                                       .incr_read = false,
                                       .incr_write = !null_write,
                                       .treq_sel =
                                           static_cast<uint8_t>(spi_get_dreq(
                                               device_->spi_->spi_, false))},
                   .read_addr = spi_dr,
                   .write_addr = null_write ? &dummy_write_ : arg.write_addr,
                   .trans_count = arg.size},
               // TX channel
               ChannelConfig{
                   .ctrl = ChannelCtrl{.data_size = ChannelCtrl::DataSize::k8,
                                       .incr_read = !null_read,
                                       .incr_write = false,
                                       .treq_sel =
                                           static_cast<uint8_t>(spi_get_dreq(
                                               device_->spi_->spi_, true))},
                   .read_addr = null_read ? &dummy_read_ : arg.read_addr,
                   .write_addr = spi_dr,
                   .trans_count = arg.size}},
          .before = before,
          .after = after,
          .chain_length = 1,
      });
  args_.emplace_back();
  args_.emplace_back();
}

util::Status SpiTransactionBuilder::RunBlocking(
    std::initializer_list<TransferArg> args, TickType_t ticks_to_wait) {
  CHECK_EQ(program_.total_length(), args.size());
  int arg_index = 0;
  for (const TransferArg& arg : args) {
  bool null_write = arg.write_addr && *arg.write_addr == nullptr;
  bool null_read = arg.read_addr && *arg.read_addr == nullptr;
    if (arg.size) CHECK_GT(*arg.size, 0u);
    // Channel 0: RX
    args_[arg_index++] = {
        .write_addr = null_write ? std::nullopt : arg.write_addr,
        .trans_count = arg.size,
    };
    // Channel 1: TX
    args_[arg_index++] = {
        .read_addr = null_read ? std::nullopt : arg.read_addr,
        .trans_count = arg.size,
    };
  }
  program_.SetArgsChainMajor(args_.begin(), args_.end());
  // piggyback our semaphore onto the last command's Action
  CHECK_GT(program_.num_commands(), 0u);
  std::optional<Action>& after = program_.mutable_last_action();
  std::optional<Action> original_action = after;
  if (after && after->give_semaphore) {
    after->give_semaphore = device_->spi_->unblock_semaphore_;
  } else if (after) {
    after->give_semaphore = device_->spi_->unblock_semaphore_;
  } else {
    after = Action{.give_semaphore = device_->spi_->unblock_semaphore_};
  }

  VLOG(1) << "Claiming bus_mutex_ for device=" << device_->name_;
  if (!xSemaphoreTake(device_->spi_->bus_mutex_, ticks_to_wait)) {
    return util::UnavailableError("timed out");
  }
  gpio_put(device_->cs_, 0);
  device_->spi_->dma_->Enqueue(&program_);
  VLOG(1) << "Waiting for unblock_semaphore_";
  CHECK(xSemaphoreTake(device_->spi_->unblock_semaphore_, ticks_to_wait));
  if (original_action && original_action->give_semaphore) {
    CHECK(xSemaphoreGive(original_action->give_semaphore));
    after->give_semaphore = original_action->give_semaphore;
  }
  gpio_put(device_->cs_, 1);
  VLOG(1) << "Returning bus_mutex_";
  CHECK(xSemaphoreGive(device_->spi_->bus_mutex_));
  return util::OkStatus();
}

SpiTransactionBuilder SpiDevice::NewTransactionBuilder() {
  return SpiTransactionBuilder(this);
}

}  // namespace tplp