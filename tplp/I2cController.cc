#include "tplp/I2cController.h"

#include <chrono>

#include "FreeRTOS/semphr.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "picolog/picolog.h"
#include "tplp/config/tplp_config.h"
#include "tplp/rtos_utils.h"

namespace tplp {
namespace {
// I2C reserves any addresses of the form 000 0xxx or 111 1xxx
// for special purposes.
bool is_reserved(i2c_address_t addr) {
  return (addr.get() & 0x78) == 0 || (addr.get() & 0x78) == 0x78;
}

// notification index used by both IRQs to wake the main task
static constexpr int kNotificationIndex = 0;
static constexpr int kNumDmaIrqs = 2;

namespace NotificationBits {
// Layout matches the IC_TX_ABRT_SOURCE register.
enum Values : uint32_t {
  // This field indicates that the controller is in 7-bit addressing mode and
  // the address sent was not acknowledged by any target.
  ABRT_7B_ADDR_NOACK = 1 << 0,
  // This field indicates that the controller is in 10-bit address mode and the
  // first 10-bit address byte was not acknowledged by any target.
  ABRT_10ADDR1_NOACK = 1 << 1,
  // This field indicates that the controller is in 10-bit address mode and that
  // the second address byte of the 10-bit address was not acknowledged by any
  // target.
  ABRT_10ADDR2_NOACK = 1 << 2,
  // This field indicates the controller-mode only bit. When the controller
  // receives an acknowledgement for the address, but when it sends data byte(s)
  // following the address, it did not receive an acknowledge from the remote
  // target(s).
  ABRT_TXDATA_NOACK = 1 << 3,
  // This field indicates that DW_apb_i2c in controller mode has sent a General
  // Call and no target on the bus acknowledged the General Call.
  ABRT_GCALL_NO_ACK = 1 << 4,
  // This field indicates that DW_apb_i2c in the controller mode has sent a
  // General Call but the user programmed the byte following the General Call to
  // be a read from the bus (IC_DATA_CMD[9] is set to 1).
  ABRT_GCALL_READ = 1 << 5,
  // This field indicates that the controller is in High Speed mode and the High
  // Speed controller code was acknowledged (wrong behavior).
  ABRT_HS_ACKDET = 1 << 6,
  // This field indicates that the controller has sent a START Byte and the
  // START Byte was acknowledged (wrong behavior).
  ABRT_SBYTE_ACKDET = 1 << 7,
  // This field indicates that the restart is disabled (IC_RESTART_EN bit
  // (IC_CON[5]) =0) and the user is trying to use the controller to transfer
  // data in High Speed mode.
  ABRT_HS_NORSTRT = 1 << 8,
  // User trying to send START byte when RESTART disabled
  ABRT_SBYTE_NO_RSTRT = 1 << 9,
  // This field indicates that the restart is disabled (IC_RESTART_EN bit
  // (IC_CON[5]) =0) and the controller sends a read command in 10-bit
  // addressing mode
  ABRT_10B_RD_NORSTRT = 1 << 10,
  // This field indicates that the User tries to initiate a controller operation
  // with the controller mode disabled.
  ABRT_CONTROLLER_DIS = 1 << 11,
  // This field specifies that the controller has lost arbitration, or if
  // IC_TX_ABRT_SOURCE[14] is also set, then the target transmitter has lost
  // arbitration.
  ABRT_ARB_LOST = 1 << 12,
  // This field specifies that the target has received a read command and some
  // data exists in the TX FIFO, so the target issues a TX_ABRT interrupt to
  // flush old data in TX FIFO.
  ABRT_TRGFLUSH_TXFIFO = 1 << 13,
  // This field indicates that a target has lost the bus while transmitting data
  // to a remote controller. IC_TX_ABRT_SOURCE[12] is set at the same time.
  // Note: Even though the target never 'owns' the bus, something could go wrong
  // on the bus. This is a fail safe check. For instance, during a data
  // transmission at the low-to-high transition of SCL, if what is on the data
  // bus is not what is supposed to be transmitted, then DW_apb_i2c no longer
  // own the bus
  ABRT_TRG_ARBLOST = 1 << 14,
  // When the processor side responds to a target mode request for data to be
  // transmitted to a remote controller and user writes a 1 in CMD (bit 8) of
  // IC_DATA_CMD register
  ABRT_TRGRD_INTX = 1 << 15,
  // This is a controller-mode-only bit. Controller has detected the transfer
  // abort (IC_ENABLE[1])
  ABRT_USER_ABRT = 1 << 16,
  // RX DMA has filled its write buffer, meaning that an I2C read operation has
  // fully completed, presumably with success.
  DMA_RX_FINISHED = 1 << 17,
  // I2C RX FIFO overflow; more bytes were received on the bus than the RX DMA
  // could read. This should not happen and means that there's a bug in
  // I2cController.
  RX_OVER = 1 << 18,
  // I2C TX FIFO is empty and transmission of the most recently popped command
  // is complete.
  TX_EMPTY = 1 << 19,
  // Bits 20 to 22 are unused for now.
  // Bits 23 to 31 reserved for the TX_FLUSH_CNT value.
  // This field indicates the number of Tx FIFO Data
  // Commands which are flushed due to TX_ABRT interrupt. It
  // is cleared whenever I2C is disabled.
  TX_FLUSH_CNT_MASK = 0xff800000,

  // Bits 17 to 22, which are marked as reserved in the datasheet and which we
  // use for our own purposes.
  RESERVED_BITS_MASK = 0x007e0000,
};
static_assert(!(TX_FLUSH_CNT_MASK & RESERVED_BITS_MASK));
static_assert(!(RESERVED_BITS_MASK & ABRT_USER_ABRT));
static_assert(RESERVED_BITS_MASK & DMA_RX_FINISHED);
static_assert(RESERVED_BITS_MASK & TX_EMPTY);

util::Status AbortSourceToStatus(uint32_t bits) {
  if (bits & ABRT_7B_ADDR_NOACK) {
    return util::NotFoundError("Address NACK");
  }
  if (bits & ABRT_10ADDR1_NOACK) {
    return util::NotFoundError("Address NACK1");
  }
  if (bits & ABRT_10ADDR2_NOACK) {
    return util::NotFoundError("Address NACK2");
  }
  if (bits & ABRT_TXDATA_NOACK) {
    return util::DataLossError("Data not ACKed");
  }
  if (bits & ABRT_GCALL_NO_ACK) {
    return util::NotFoundError("General Call not ACKed by any target");
  }
  if (bits & ABRT_GCALL_READ) {
    return util::FailedPreconditionError(
        "General Call cannot be followed by a read");
  }
  if (bits & ABRT_HS_ACKDET) {
    return util::UnknownError("High Speed control code was ACKed");
  }
  if (bits & ABRT_SBYTE_ACKDET) {
    return util::UnknownError("START byte was ACKed");
  }
  if (bits & ABRT_HS_NORSTRT) {
    return util::InternalError("Restart disabled in High Speed mode");
  }
  if (bits & ABRT_SBYTE_NO_RSTRT) {
    return util::InternalError("Restart attempted while Restart is disabled");
  }
  if (bits & ABRT_10B_RD_NORSTRT) {
    return util::InternalError("Restart disabled in 10-bit address mode");
  }
  if (bits & ABRT_CONTROLLER_DIS) {
    return util::InternalError("Controller is disabled");
  }
  if (bits & ABRT_ARB_LOST) {
    // This should be retried
    return util::UnavailableError("Arbitration lost");
  }
  if (bits & ABRT_TRGFLUSH_TXFIFO) {
    // Handling for this scenario is not implemented.
    return util::UnimplementedError("ABRT_TRGFLUSH_TXFIFO");
  }
  if (bits & ABRT_TRG_ARBLOST) {
    return util::UnavailableError("Arbitration lost");
  }
  if (bits & ABRT_TRGRD_INTX) {
    return util::InternalError("ABRT_TRGRD_INTX");
  }
  if (bits & ABRT_USER_ABRT) {
    return util::AbortedError("User abort");
  }
  if (bits & DMA_RX_FINISHED) {
    // This should have been handled elsewhere.
    LOG(FATAL) << "Unhandled DMA_RX_FINISHED";
  }
  if (bits & RX_OVER) {
    return util::InternalError("RX FIFO overflow");
  }
  if (bits & TX_EMPTY) {
    // This should have been handled elsewhere.
    LOG(FATAL) << "Unhandled TX_EMPTY";
  }
  return util::UnknownError("No ABRT_SOURCE specified");
}
}  // namespace NotificationBits

// Size of the command buffer used as the source for the TX DMA.
static constexpr size_t kCommandBufferSize = 32;

class DmaFinishedNotifier {
 private:
  struct ManagerTaskInfo {
    dma_channel_t dma_rx;
    TaskHandle_t task = nullptr;
  };
  static ManagerTaskInfo manager_tasks[kNumDmaIrqs];

 public:
  template <int irq_index>
  static void ISR() {
    static_assert(irq_index >= 0 && irq_index < kNumDmaIrqs, "bad irq_index");
    if (dma_irqn_get_channel_status(irq_index,
                                    manager_tasks[irq_index].dma_rx)) {
      dma_irqn_acknowledge_channel(irq_index, manager_tasks[irq_index].dma_rx);
      BaseType_t higher_priority_task_woken = 0;
      xTaskNotifyIndexedFromISR(
          manager_tasks[irq_index].task, kNotificationIndex,
          static_cast<uint32_t>(NotificationBits::DMA_RX_FINISHED), eSetBits,
          &higher_priority_task_woken);
      portYIELD_FROM_ISR(higher_priority_task_woken);
    }
  }

  static void RegisterManagerTask(dma_irq_index_t irq_index,
                                  dma_channel_t dma_rx, TaskHandle_t task) {
    CHECK_GE(irq_index, 0);
    CHECK_LT(irq_index, kNumDmaIrqs);
    CHECK_EQ(manager_tasks[irq_index].task, nullptr)
        << "I2C manager task on DMA IRQ index " << irq_index
        << " already exists?";
    manager_tasks[irq_index] = {.dma_rx = dma_rx, .task = task};
  }
};

DmaFinishedNotifier::ManagerTaskInfo
    DmaFinishedNotifier::manager_tasks[kNumDmaIrqs] = {};

template void DmaFinishedNotifier::ISR<0>();
template void DmaFinishedNotifier::ISR<1>();

static dma_channel_config MakeChannelConfig(
    i2c_inst_t* spi, dma_channel_t dma, dma_channel_transfer_size transfer_size,
    bool is_tx, bool read_increment, bool write_increment, bool irq_quiet) {
  dma_channel_config c = dma_channel_get_default_config(dma);
  channel_config_set_dreq(&c, i2c_get_dreq(spi, is_tx));
  VLOG(1) << "DMA channel " << dma << " will be connected to I2C"
          << i2c_hw_index(spi) << " DREQ " << i2c_get_dreq(spi, is_tx);
  channel_config_set_transfer_data_size(&c, transfer_size);
  channel_config_set_read_increment(&c, read_increment);
  channel_config_set_write_increment(&c, write_increment);
  channel_config_set_irq_quiet(&c, irq_quiet);
  return c;
}

class I2cNotifier {
 private:
  static TaskHandle_t manager_tasks[2];

 public:
  template <int i2c_hw_index>
  static void ISR() {
    static_assert(i2c_hw_index == 0 || i2c_hw_index == 1);
    auto* hw = i2c_get_hw(i2c_hw_index ? i2c1 : i2c0);
    uint32_t notify = 0;
    if (hw->intr_stat & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
      // Block any further commands from being processed. The FIFOs are kept
      // empty until the TX_ABRT interrupt is cleared, but after that the DMAs
      // may push more commands. Aborting the DMAs from here would take too
      // long, so we pause them, block the command queue, and let the main task
      // handle cleanup.
      hw_clear_bits(&hw->dma_cr,
                    I2C_IC_DMA_CR_TDMAE_BITS | I2C_IC_DMA_CR_RDMAE_BITS);
      hw_set_bits(&hw->enable, I2C_IC_ENABLE_TX_CMD_BLOCK_BITS);
      // Copy over the abort reason.
      notify |= hw->tx_abrt_source & ~NotificationBits::RESERVED_BITS_MASK;
      // This clears the interrupt, unblocks the FIFOs, resumes command
      // processing, and clears the abort source register.
      (void)hw->clr_tx_abrt;
      // ABRT_SBYTE_NORSTRT cannot be cleared this way, but the only way it
      // would be set is if we disabled restarts.
    }
    if (hw->intr_stat & I2C_IC_INTR_STAT_R_RX_OVER_BITS) {
      (void)hw->clr_rx_over;
      notify |= NotificationBits::RX_OVER;
    }
    if (hw->intr_stat & I2C_IC_INTR_STAT_R_TX_EMPTY_BITS) {
      // TX_EMPTY is cleared by hardware! so we need to mask it here instead,
      // and rely on I2cController to unmask it when needed.
      hw_clear_bits(&hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);
      notify |= NotificationBits::TX_EMPTY;
    }
    BaseType_t higher_priority_task_woken = 0;
    xTaskNotifyIndexedFromISR(manager_tasks[i2c_hw_index], kNotificationIndex,
                              notify, eSetBits, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
  }

  static void RegisterManagerTask(int i2c_hw_index, TaskHandle_t task) {
    CHECK_GE(i2c_hw_index, 0);
    CHECK_LT(i2c_hw_index, 2);
    CHECK_EQ(manager_tasks[i2c_hw_index], nullptr)
        << "I2C manager task on I2C" << i2c_hw_index << " already exists?";
    manager_tasks[i2c_hw_index] = task;
  }
};

TaskHandle_t I2cNotifier::manager_tasks[] = {0, 0};

}  // namespace

constexpr int kEventQueueDepth = 8;
struct I2cController::Event {
  enum class Tag { INVALID, READ, WRITE } tag = Tag::INVALID;
  i2c_address_t addr = i2c_address_t(0);
  uint8_t* buf = nullptr;
  size_t len = 0;
  bool stop = true;
  // Must be true for the first event of each transaction.
  bool first_cmd_of_txn = false;

  // If not null, will be given after the operation completes.
  SemaphoreHandle_t blocking_sem = nullptr;

  // If not null, will be set to the result status of the operation.
  // FIXME: make sure these are correct
  // util::IsNotFound(): read/write was unacknowledged
  util::Status* out_result = nullptr;

  friend std::ostream& operator<<(std::ostream& stream, const Event::Tag& tag) {
    switch (tag) {
      case Tag::INVALID:
        return stream << "INVALID";
      case Tag::READ:
        return stream << "READ";
      case Tag::WRITE:
        return stream << "WRITE";
      default:
        return stream << (int)tag;
    }
  }

  friend std::ostream& operator<<(std::ostream& stream, const Event& event) {
    return stream << "{ " << event.tag << " addr=" << event.addr
                  << " buf=" << std::hex << (void*)event.buf << std::dec
                  << " len=" << event.len << " }";
  }
};

I2cController* I2cController::Init(int task_priority, i2c_inst_t* i2c_instance,
                                   gpio_pin_t scl, gpio_pin_t sda,
                                   int baudrate) {
  const int i2c_instance_index = i2c_hw_index(i2c_instance);
  CHECK_GT(baudrate, 0);
  // i2c_init configures it for fast mode, which the hardware considers
  // equivalent to fast mode plus, which has a maximum rate of 1000kbps.
  CHECK_LE(baudrate, 1'000'000);
  // TODO: implement full baudrate calculation instead of i2c_init()
  int actual_baudrate = i2c_init(i2c_instance, baudrate);
  LOG(INFO) << "I2C" << i2c_instance_index << " baudrate set to "
            << actual_baudrate << "Hz";

  gpio_set_function(scl, GPIO_FUNC_I2C);
  gpio_set_function(sda, GPIO_FUNC_I2C);
  gpio_pull_up(scl);
  gpio_pull_up(sda);

  I2cController* self = new I2cController();
  self->i2c_ = i2c_instance;
  self->event_queue_ =
      CHECK_NOTNULL(xQueueCreate(kEventQueueDepth, sizeof(Event)));
  self->txn_mutex_ = CHECK_NOTNULL(xSemaphoreCreateMutex());
  self->shared_blocking_sem_ = CHECK_NOTNULL(xSemaphoreCreateBinary());
  self->dma_rx_ = dma_channel_t(dma_claim_unused_channel(false));
  LOG_IF(FATAL, self->dma_rx_ < 0) << "No free DMA channels available";
  self->dma_tx_ = dma_channel_t(dma_claim_unused_channel(false));
  LOG_IF(FATAL, self->dma_tx_ < 0) << "No free DMA channels available";

  // Arbitrarily assign DMA_IRQ_0 to I2C0 and DMA_IRQ_1 to I2C1.
  self->dma_irq_index_ = dma_irq_index_t(i2c_instance_index);
  self->dma_irq_number_ =
      dma_irq_number_t(i2c_instance_index ? DMA_IRQ_1 : DMA_IRQ_0);
  if (self->dma_irq_index_) {
    irq_add_shared_handler(self->dma_irq_number_, &DmaFinishedNotifier::ISR<1>,
                           PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  } else {
    irq_add_shared_handler(self->dma_irq_number_, &DmaFinishedNotifier::ISR<0>,
                           PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  }
  // RX IRQ set to not-quiet.
  self->dma_rx_config_ = MakeChannelConfig(
      self->i2c_, self->dma_rx_, DMA_SIZE_8, false, true, false, false);
  // TX needs to be 16 bits wide because we use the upper 8 as control bits
  // when writing to IC_DATA_CMD.
  self->dma_tx_config_ = MakeChannelConfig(
      self->i2c_, self->dma_tx_, DMA_SIZE_16, true, false, true, true);

  if (i2c_instance_index) {
    irq_set_exclusive_handler(I2C1_IRQ, &I2cNotifier::ISR<1>);
  } else {
    irq_set_exclusive_handler(I2C0_IRQ, &I2cNotifier::ISR<0>);
  }

  // Mask all but the interrupts we care about.
  // We also care about TX_EMPTY, but that should remain masked until needed.
  hw_clear_bits(&self->i2c_->hw->intr_mask, 0x00001fff);
  hw_set_bits(&self->i2c_->hw->intr_mask, I2C_IC_INTR_MASK_M_RX_OVER_BITS |
                                              I2C_IC_INTR_MASK_M_TX_ABRT_BITS);

  // Configure TX_EMPTY interrupt: trigger when fully empty, and only after the
  // most recently popped command has completed.
  hw_set_bits(&self->i2c_->hw->con, I2C_IC_CON_TX_EMPTY_CTRL_BITS);
  self->i2c_->hw->tx_tl = 0;

  char* task_name = new char[8];
  snprintf(task_name, 16, "I2C %d", i2c_instance_index);
  xTaskCreate(&TaskFn, task_name, TaskStacks::kI2cController, self,
              task_priority, &self->task_);

  return self;
}

I2cController::I2cController(){};

void I2cController::TaskFn(void* task_param) {
  I2cController* self = static_cast<I2cController*>(task_param);
  LOG(INFO) << "I2cController task started.";

  int i2c_index = i2c_hw_index(self->i2c_);
  DmaFinishedNotifier::RegisterManagerTask(self->dma_irq_index_, self->dma_rx_,
                                           self->task_);
  I2cNotifier::RegisterManagerTask(i2c_index, self->task_);

  // Enable only the RX DMA IRQ for DmaFinishedNotifier; we use the I2C
  // interrupt to detect when writes are finished instead.
  dma_irqn_set_channel_enabled(self->dma_irq_index_, self->dma_tx_, false);
  dma_irqn_set_channel_enabled(self->dma_irq_index_, self->dma_rx_, true);

  irq_set_enabled(self->dma_irq_number_, true);
  LOG(INFO) << "IRQ " << self->dma_irq_number_ << " (DMA) enabled.";
  irq_set_enabled(i2c_index ? I2C1_IRQ : I2C0_IRQ, true);
  LOG(INFO) << "IRQ " << self->dma_irq_number_ << " (I2C) enabled.";

  // Keep the controller initially disabled.
  self->i2c_->hw->enable = 0;

  for (;;) {
    Event event;
    VLOG(2) << "Waiting for event.";
    CHECK(xQueueReceive(self->event_queue_, &event, portMAX_DELAY));
    VLOG(2) << "Event: " << event;

    switch (event.tag) {
      case Event::Tag::READ:
        self->DoRead(event);
        break;
      case Event::Tag::WRITE:
        self->DoWrite(event);
        break;
      default:
        LOG(FATAL) << "Invalid event received: " << event;
    }

    if (event.blocking_sem) CHECK(xSemaphoreGive(event.blocking_sem));
  }
}

void I2cController::DoRead(const Event& event) {
  const auto hw = i2c_->hw;

  CHECK_EQ(hw->enable_status & 1, 0u) << "Expected i2c hw to be disabled";
  CHECK(!(hw->status & I2C_IC_STATUS_RFNE_BITS))
      << "Expected receive FIFO to be empty";
  CHECK(hw->status & I2C_IC_STATUS_TFE_BITS)
      << "Expected transmit FIFO to be empty";
  CHECK(!dma_channel_is_busy(dma_rx_));
  CHECK(!dma_channel_is_busy(dma_tx_));
  uint32_t notify =
      ulTaskNotifyValueClearIndexed(nullptr, kNotificationIndex, 0xffffffff);
  CHECK_EQ(notify, 0u) << "Unexpected notification bits";
  CHECK(!xTaskNotifyStateClearIndexed(nullptr, kNotificationIndex))
      << "Unexpected notification state";
  CHECK_EQ(hw->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS, 0u)
      << "I2C hardware unexpectedly enabled";

  CHECK_LT(event.len, kCommandBufferSize)
      << "reads above command buffer size not implemented";

  hw->tar = event.addr.get();
  hw->enable = 1;

  // TODO allocate elsewhere
  uint16_t cmd_buf[kCommandBufferSize];
  // Setting the CMD bit indicates a read command.
  std::fill<uint16_t*, uint16_t>(cmd_buf, cmd_buf + event.len,
                                 I2C_IC_DATA_CMD_CMD_BITS);
  // Issue RESTART if this is not the first read of the txn.
  if (!event.first_cmd_of_txn) cmd_buf[0] |= I2C_IC_DATA_CMD_RESTART_BITS;
  // Issue STOP after the last byte if requested.
  if (event.stop) cmd_buf[event.len - 1] |= I2C_IC_DATA_CMD_STOP_BITS;

  // Mask the TX_EMPTY interrupt; for a read we don't want to be notified
  // when the TX FIFO is empty (i.e. when the last command has been executed),
  // but only when the RX DMA has finished (or on abort).
  hw_clear_bits(&hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);

  dma_channel_configure(dma_tx_, &dma_tx_config_, &hw->data_cmd, cmd_buf,
                        event.len, false);
  dma_channel_configure(dma_rx_, &dma_rx_config_, event.buf, &hw->data_cmd,
                        event.len, false);
  // Start TX and RX simultaneously so the I2C FIFOs don't overflow.
  dma_start_channel_mask((1u << dma_tx_) | (1u << dma_rx_));
  VLOG(2) << "DMA started; waiting";

  // Wait for the DMA to complete or the transaction to be aborted. Set a
  // generous timeout- no reasonable DMA or I2C transfer should take longer.
  // notify = ulTaskNotifyTakeIndexed(kNotificationIndex, true, portMAX_DELAY);
  notify =
      ulTaskNotifyTakeIndexed(kNotificationIndex, true, pdMS_TO_TICKS(10'000));
  if (!notify) {
    LOG(FATAL) << "Missed a notification or timed out waiting for DMA. tx_busy="
               << dma_channel_is_busy(dma_tx_)
               << " rx_busy=" << dma_channel_is_busy(dma_rx_) << " status=0x"
               << std::hex << hw->status << " raw_intr_stat=0x"
               << hw->raw_intr_stat;
  }
  VLOG(2) << "Got notification bits: " << std::hex << notify;

  if (notify & NotificationBits::DMA_RX_FINISHED) {
    // We expect that the read fully completed with success in this scenario.
    CHECK_EQ(notify & ~NotificationBits::DMA_RX_FINISHED, 0u)
        << "Unexpected abort reason in DMA_RX_FINISHED notification";
    *event.out_result = util::OkStatus();
    // Safely disable the controller for the next command.
    CHECK(!(hw->status & I2C_IC_STATUS_ACTIVITY_BITS))
        << "Expected I2C bus to be idle after successful read";
    hw->enable = 0;
    // The rest of the disable procedure is unnecessary because we've already
    // checked that the state machine is idle.
    CHECK_EQ(hw->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS, 0u)
        << "I2C controller did not respond to disable in a timely fashion";
  } else {
    // Safely abort any in-flight DMA transfers
    dma_irqn_set_channel_enabled(dma_irq_index_, dma_rx_, false);
    dma_irqn_set_channel_enabled(dma_irq_index_, dma_tx_, false);
    // NB: dma_channel_abort busy-waits!
    dma_channel_abort(dma_rx_);
    dma_channel_abort(dma_tx_);
    dma_irqn_acknowledge_channel(dma_irq_index_, dma_rx_);
    dma_irqn_acknowledge_channel(dma_irq_index_, dma_tx_);
    dma_irqn_set_channel_enabled(dma_irq_index_, dma_rx_, true);
    dma_irqn_set_channel_enabled(dma_irq_index_, dma_tx_, true);
    // Reenable the DMA controller interface, disabled by our interrupt handler.
    hw_set_bits(&hw->dma_cr,
                I2C_IC_DMA_CR_TDMAE_BITS | I2C_IC_DMA_CR_RDMAE_BITS);
    // If we cared to, we could measure how many bytes were lost
    // by checking `notify & TX_FLUSH_CNT_MASK` and the remaining
    // DMA transfer_count. But we don't.

    // TODO: not sure about this assumption.
    CHECK(!(hw->status & I2C_IC_STATUS_ACTIVITY_BITS))
        << "Expected I2C state machine to be idle after abort";
    // Clear the FIFOs and simultaneously
    // remove the block on command execution that was set in the I2cNotifier
    // ISR.
    hw->enable = 0;
    // The rest of the disable procedure is unnecessary because we've already
    // checked that the state machine is idle.
    CHECK_EQ(hw->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS, 0u)
        << "I2C controller did not respond to disable in a timely fashion";
    util::Status status = NotificationBits::AbortSourceToStatus(notify);
    LOG_IF(FATAL, util::IsInternal(status)) << status;
    *event.out_result = std::move(status);
  }
  // Done.
}

void I2cController::DoWrite(const Event& event) {
  int ret = i2c_write_blocking(i2c_, event.addr.get(), event.buf, event.len,
                               !event.stop);
  // FIXME: for now, assuming all errors are NotFoundError.

  // Unmask the TX_EMPTY interrupt; I2cNotifier masks it each time it's
  // triggered.
  // hw_clear_bits(&hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);

  if (event.out_result) {
    *event.out_result = ret <= 0 ? util::NotFoundError("") : util::OkStatus();
  }
}

util::Status I2cController::ScanBus(
    std::vector<i2c_address_t>* detected_addresses) {
  CHECK_NOTNULL(detected_addresses);
  uint8_t dummy;

  // addresses above 0x77 are reserved
  LOG(INFO) << "Starting I2C bus scan...";
  for (i2c_address_t addr(0); addr < i2c_address_t(0x78); ++addr) {
    if (is_reserved(addr)) continue;
    I2cTransaction txn = StartTransaction(addr);
    util::Status status = txn.ReadAndStop(&dummy, 1);

    VLOG(1) << "SCAN " << addr << ": " << status;
    if (util::IsNotFound(status)) {
      continue;
    } else if (!status.ok()) {
      return status;
    }
    detected_addresses->push_back(addr);
  }
  return util::OkStatus();
}

util::Status I2cController::ReadDeviceId(i2c_address_t target,
                                         I2cDeviceId* out) {
  return util::UnimplementedError("ReadDeviceId");
}

I2cTransaction I2cController::StartTransaction(i2c_address_t addr) {
  CHECK(xSemaphoreTake(txn_mutex_, portMAX_DELAY));
  return I2cTransaction(this, addr);
}

I2cTransaction::I2cTransaction(I2cController* controller, i2c_address_t addr)
    : controller_(controller),
      addr_(addr),
      stopped_(false),
      first_command_(true) {}

I2cTransaction::~I2cTransaction() {
  if (!stopped_) {
    util::Status s = Abort();
    LOG(WARNING) << "Transaction destroyed before STOP. Abort status: " << s;
    CHECK(xSemaphoreGive(controller_->txn_mutex_));
  }
}
I2cTransaction::I2cTransaction(I2cTransaction&& other)
    : controller_(other.controller_),
      addr_(other.addr_),
      stopped_(other.stopped_),
      first_command_(other.first_command_) {
  other.stopped_ = true;
}

util::Status I2cTransaction::Write(const uint8_t* buf, size_t len, bool stop) {
  CHECK(!stopped_);
  util::Status status;
  I2cController::Event event{
      .tag = I2cController::Event::Tag::WRITE,
      .addr = addr_,
      .buf = const_cast<uint8_t*>(buf),
      .len = len,
      .stop = stop,
      .first_cmd_of_txn = first_command_,
      .blocking_sem = controller_->shared_blocking_sem_,
      .out_result = &status,
  };
  CHECK(xQueueSendToBack(controller_->event_queue_, &event, portMAX_DELAY));
  CHECK(xSemaphoreTake(controller_->shared_blocking_sem_, portMAX_DELAY));
  if (stop) {
    AfterStop();
  }
  first_command_ = false;
  return status;
}

util::Status I2cTransaction::Read(uint8_t* buf, size_t len, bool stop) {
  CHECK(!stopped_);
  util::Status status;
  I2cController::Event event{
      .tag = I2cController::Event::Tag::READ,
      .addr = addr_,
      .buf = buf,
      .len = len,
      .stop = stop,
      .first_cmd_of_txn = first_command_,
      .blocking_sem = controller_->shared_blocking_sem_,
      .out_result = &status,
  };
  CHECK(xQueueSendToBack(controller_->event_queue_, &event, portMAX_DELAY));
  CHECK(xSemaphoreTake(controller_->shared_blocking_sem_, portMAX_DELAY));
  if (stop) {
    AfterStop();
  }
  first_command_ = false;
  return status;
}

void I2cTransaction::AfterStop() {
  stopped_ = true;
  xSemaphoreGive(controller_->txn_mutex_);
}

util::Status I2cTransaction::Abort() {
  // FIXME: implement
  // See RP2040 datasheet section 4.3.10.4
  return util::UnimplementedError("I2cTransaction::Abort()");
}

}  // namespace tplp