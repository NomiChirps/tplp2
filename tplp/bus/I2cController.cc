#include "tplp/bus/I2cController.h"

#include "FreeRTOS/semphr.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "picolog/picolog.h"
#include "picolog/status_macros.h"
#include "tplp/rtos_util.h"

namespace tplp {
namespace {
// I2C reserves any addresses of the form 000 0xxx or 111 1xxx
// for special purposes.
bool is_reserved(i2c_address_t addr) {
  return (addr.get() & 0x78) == 0 || (addr.get() & 0x78) == 0x78;
}

// notification index used by both IRQs to wake the main task
static constexpr int kNotificationIndex = 0;

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

  // IRQ_SOURCE_* flags indicate which interrupt causing this notification.

  // RX DMA has filled its write buffer, meaning that an I2C read operation has
  // fully completed, presumably with success.
  IRQ_SOURCE_DMA_RX_FINISHED = 1 << 17,
  // I2C RX FIFO overflow; more bytes were received on the bus than the RX DMA
  // could read. This should not happen and means that there's a bug in
  // I2cController.
  IRQ_SOURCE_RX_OVER = 1 << 18,
  // I2C TX FIFO is empty and transmission of the most recently popped command
  // is complete.
  IRQ_SOURCE_TX_EMPTY = 1 << 19,
  // Transaction was aborted and there should be a reason somewhere up there.
  IRQ_SOURCE_TX_ABRT = 1 << 20,
  // Bits 21 to 22 are unused for now.
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
static_assert(RESERVED_BITS_MASK & IRQ_SOURCE_DMA_RX_FINISHED);
static_assert(RESERVED_BITS_MASK & IRQ_SOURCE_TX_ABRT);

util::Status AbortSourceToStatus(uint32_t bits) {
  if (bits & ABRT_7B_ADDR_NOACK) {
    return util::NotFoundError("Address byte not ACKed by any target");
  }
  if (bits & ABRT_10ADDR1_NOACK) {
    return util::NotFoundError("Address byte 1 not ACKed by any target");
  }
  if (bits & ABRT_10ADDR2_NOACK) {
    return util::NotFoundError("Address byte 2 not ACKed by any target");
  }
  if (bits & ABRT_TXDATA_NOACK) {
    return util::DataLossError("Target failed to ACK data");
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
  if (bits & IRQ_SOURCE_DMA_RX_FINISHED) {
    // This should have been handled elsewhere.
    LOG(FATAL) << "Unhandled DMA_RX_FINISHED";
  }
  if (bits & IRQ_SOURCE_RX_OVER) {
    return util::InternalError("RX FIFO overflow");
  }
  if (bits & IRQ_SOURCE_TX_EMPTY) {
    // This should have been handled elsewhere.
    LOG(FATAL) << "Unhandled TX_EMPTY";
  }
  return util::UnknownError("No ABRT_SOURCE specified");
}
}  // namespace NotificationBits

// Size of the command buffer used as the source for the TX DMA.
static constexpr size_t kCommandBufferSize = 32;

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
      // Flag that we got this interrupt.
      notify |= NotificationBits::IRQ_SOURCE_TX_ABRT;
      // Copy over the abort reason, whatever it is.
      notify |= hw->tx_abrt_source & ~NotificationBits::RESERVED_BITS_MASK;
      // This clears the interrupt, unblocks the FIFOs, resumes command
      // processing, and clears the abort source register.
      (void)hw->clr_tx_abrt;
      // ABRT_SBYTE_NORSTRT cannot be cleared this way, but the only way it
      // would be set is if we disabled restarts.
    }
    if (hw->intr_stat & I2C_IC_INTR_STAT_R_RX_OVER_BITS) {
      (void)hw->clr_rx_over;
      notify |= NotificationBits::IRQ_SOURCE_RX_OVER;
    }
    if (hw->intr_stat & I2C_IC_INTR_STAT_R_TX_EMPTY_BITS) {
      // TX_EMPTY is cleared by hardware! so we need to mask it here instead,
      // and rely on I2cController to unmask it when needed.
      hw_clear_bits(&hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);
      notify |= NotificationBits::IRQ_SOURCE_TX_EMPTY;
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
  // Contents of the IC_TAR register.
  uint32_t tar = 0;
  // TODO: build the commands in I2cTransaction and put the command buffer
  //       here, instead of in TaskFn. that way the transaction can exactly
  //       control restarts and stops without a context switch & dma reinit
  //       in between. we might even be able to coalesce multiple read/write
  //       ops into one event!!
  uint8_t* buf = nullptr;
  size_t len = 0;
  bool restart = false;
  bool stop = true;

  // If not null, will be given after the operation completes.
  SemaphoreHandle_t blocking_sem = nullptr;

  // If not null, will be set to the result status of the operation.
  // See `NotificationBits::AbortSourceToStatus()`.
  // TODO: document those in the public header
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
    return stream << "{ " << event.tag << " tar=" << std::hex << event.tar
                  << " buf=" << (void*)event.buf << std::dec
                  << " len=" << event.len << " }";
  }
};

I2cController* I2cController::Init(DmaController* dma, int priority,
                                   int stack_depth, i2c_inst_t* i2c_instance,
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
  self->dma_ = dma;
  self->i2c_ = i2c_instance;
  self->event_queue_ =
      CHECK_NOTNULL(xQueueCreate(kEventQueueDepth, sizeof(Event)));
  self->txn_mutex_ = CHECK_NOTNULL(xSemaphoreCreateMutex());
  self->shared_blocking_sem_ = CHECK_NOTNULL(xSemaphoreCreateBinary());

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

  std::ostringstream task_name;
  task_name << "I2C" << i2c_instance_index;
  xTaskCreate(&TaskFn, task_name.str().c_str(), stack_depth, self, priority,
              &self->task_);

  return self;
}

I2cController::I2cController(){};

void I2cController::TaskFn(void* task_param) {
  I2cController* self = static_cast<I2cController*>(task_param);
  LOG(INFO) << "I2cController task started.";

  int i2c_index = i2c_hw_index(self->i2c_);
  I2cNotifier::RegisterManagerTask(i2c_index, self->task_);
  irq_set_enabled(i2c_index ? I2C1_IRQ : I2C0_IRQ, true);

  // Keep the controller initially disabled.
  self->i2c_->hw->enable = 0;
  self->new_transaction_ = true;

  for (;;) {
    Event event;
    VLOG(2) << "Waiting for event.";
    CHECK(xQueueReceive(self->event_queue_, &event, portMAX_DELAY));
    VLOG(2) << "Event: " << event;

    switch (event.tag) {
      case Event::Tag::READ:
      case Event::Tag::WRITE:
        self->DoTransfer(event);
        break;
      default:
        LOG(FATAL) << "Invalid event received: " << event;
    }

    if (event.blocking_sem) CHECK(xSemaphoreGive(event.blocking_sem));
  }
}

void I2cController::DoTransfer(const Event& event) {
  CHECK(event.tag == Event::Tag::READ || event.tag == Event::Tag::WRITE);
  const bool is_read = event.tag == Event::Tag::READ;
  const auto hw = i2c_->hw;

  uint32_t notify =
      ulTaskNotifyValueClearIndexed(nullptr, kNotificationIndex, 0xffffffff);
  CHECK_EQ(notify, 0u) << "Unexpected notification bits";
  CHECK(!xTaskNotifyStateClearIndexed(nullptr, kNotificationIndex))
      << "Unexpected notification state";

  CHECK_LT(event.len, kCommandBufferSize)
      << "transfers larger than command buffer size (" << kCommandBufferSize
      << ") not implemented";

  if (new_transaction_) {
    CHECK_EQ(hw->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS, 0u)
        << "Expected i2c hw to be disabled at start of transaction";
    CHECK(!(hw->status & I2C_IC_STATUS_RFNE_BITS))
        << "Expected receive FIFO to be empty";
    CHECK(hw->status & I2C_IC_STATUS_TFE_BITS)
        << "Expected transmit FIFO to be empty";

    hw->tar = event.tar;
    hw->enable = 1;

    new_transaction_ = false;
  }

  // TODO allocate elsewhere
  uint16_t cmd_buf[kCommandBufferSize];
  if (is_read) {
    // Setting the CMD bit indicates a read command.
    std::fill<uint16_t*, uint16_t>(cmd_buf, cmd_buf + event.len,
                                   I2C_IC_DATA_CMD_CMD_BITS);
  } else {
    // Copy user's bytes, leaving the CMD bit unset.
    std::copy(event.buf, event.buf + event.len, cmd_buf);
  }
  // Issue RESTART before the first byte if requested.
  if (event.restart) cmd_buf[0] |= I2C_IC_DATA_CMD_RESTART_BITS;
  // Issue STOP after the last byte if requested.
  if (event.stop) cmd_buf[event.len - 1] |= I2C_IC_DATA_CMD_STOP_BITS;

  if (is_read) {
    // Mask the TX_EMPTY interrupt; for a read we don't want to be notified
    // when the TX FIFO is empty (i.e. when the last command has been executed),
    // but only when the RX DMA has finished (or on abort).
    hw_clear_bits(&hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);
  } else {
    // Unmask TX_EMPTY so we're notified when the transfer completes.
    // I2cNotifier re-masks it each time, so this is necessary even if the
    // previous operation was also a write.
    hw_set_bits(&hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);
  }

  // c0 for tx, c1 for rx
  auto transfer_handle = dma_->Transfer({
      // TX channel
      .c0_enable = true,
      .c0_treq_sel = i2c_get_dreq(i2c_, /*is_tx=*/true),
      .c0_read_addr = cmd_buf,
      .c0_read_incr = true,
      .c0_write_addr = &hw->data_cmd,
      .c0_write_incr = false,
      // Upper 8 bits are used for I2C control.
      .c0_data_size = DmaController::DataSize::k16,
      .c0_trans_count = event.len,

      // RX channel
      .c1_enable = is_read,
      .c1_treq_sel = i2c_get_dreq(i2c_, /*is_tx=*/false),
      .c1_read_addr = &hw->data_cmd,
      .c1_read_incr = false,
      .c1_write_addr = event.buf,
      .c1_write_incr = true,
      .c1_data_size = DmaController::DataSize::k8,
      .c1_trans_count = event.len,

      .c1_action =
          DmaController::Action{
              .notify_task = task_,
              .notify_index = kNotificationIndex,
              .notify_value = NotificationBits::IRQ_SOURCE_DMA_RX_FINISHED,
              .notify_action = eSetBits,
          },
  });

  VLOG(2) << "DMA started; waiting";

  // Wait for the DMA(s) to complete or the transaction to be aborted. Set a
  // generous timeout- no reasonable DMA or I2C transfer should take longer.
  // notify = ulTaskNotifyTakeIndexed(kNotificationIndex, true, portMAX_DELAY);
  notify =
      ulTaskNotifyTakeIndexed(kNotificationIndex, true, pdMS_TO_TICKS(10'000));
  if (!notify) {
    LOG(FATAL)
        << "Missed a notification or timed out waiting for DMA. status=0x"
        << std::hex << hw->status << " raw_intr_stat=0x" << hw->raw_intr_stat;
  }
  VLOG(2) << "Got notification bits: " << std::hex << notify;

  if (notify & NotificationBits::IRQ_SOURCE_TX_ABRT) {
    CHECK(transfer_handle.started());
    // Safely abort any in-flight DMA transfers
    transfer_handle.Abort();
    // Reenable the DMA controller interface, disabled by our interrupt handler.
    hw_set_bits(&hw->dma_cr,
                I2C_IC_DMA_CR_TDMAE_BITS | I2C_IC_DMA_CR_RDMAE_BITS);
    // If we cared to, we could measure how many bytes were lost
    // by checking `notify & TX_FLUSH_CNT_MASK` and the remaining
    // DMA transfer_count. But we don't.

    // NB: not actually sure about this assumption.
    //     maybe it depends on the type of abort?
    CHECK(!(hw->status & I2C_IC_STATUS_ACTIVITY_BITS))
        << "Expected I2C state machine to be idle after abort";
    // Clear the FIFOs and simultaneously remove the block on command execution
    // that was set in the I2cNotifier ISR.
    hw->enable = 0;
    // Next request will necessarily be starting a new transaction.
    new_transaction_ = true;
    // The rest of the disable procedure is unnecessary because we've already
    // checked that the state machine is idle.
    CHECK_EQ(hw->enable_status & I2C_IC_ENABLE_STATUS_IC_EN_BITS, 0u)
        << "I2C controller did not respond to disable in a timely fashion";
    util::Status status = NotificationBits::AbortSourceToStatus(notify);
    LOG_IF(FATAL, util::IsInternal(status)) << status;
    *event.out_result = std::move(status);
  } else if (notify & NotificationBits::IRQ_SOURCE_DMA_RX_FINISHED) {
    // Read must have finished successfully.
    CHECK(is_read) << "Got DMA_RX_FINISHED for a write";
    // We expect that the read fully completed with success in this scenario.
    CHECK_EQ(notify & ~NotificationBits::IRQ_SOURCE_DMA_RX_FINISHED, 0u)
        << "Unexpected abort reason in DMA_RX_FINISHED notification";
    *event.out_result = util::OkStatus();
  } else if (notify & NotificationBits::IRQ_SOURCE_TX_EMPTY) {
    // Read must have finished successfully.
    CHECK(!is_read) << "Got TX_EMPTY for a read";
    // We expect that the write fully completed with success in this scenario.
    CHECK_EQ(notify & ~NotificationBits::IRQ_SOURCE_TX_EMPTY, 0u)
        << "Unexpected abort reason in TX_EMPTY notification";
    *event.out_result = util::OkStatus();
    // No need to re-mask the TX_EMPTY interrupt; I2cNotifier already did.
  } else {
    LOG(FATAL) << "Unclear notification bits: " << std::hex << notify;
  }
  if (event.stop) {
    // End of the transaction; disable the controller to prepare for the next
    // one. According to the datasheet, the controller can only be disabled if
    // the current command being processed has the STOP bit set to one.
    // We should have guaranteed this in the case of a read or write, but
    // in case of abort it's less clear. Furthermore, the datasheet recommends
    // we wait & poll when disabling in ALL cases.
    // TODO: is this ok? can we avoid this loop somehow? does it matter?
    hw->enable = 0;
    int n = 0;
    while (hw->enable_status & 1) {
      ++n;
      VLOG(1) << "waiting for disable... " << n;
      vTaskDelay(1);
      LOG_IF(FATAL, n > 999)
          << "Failed to disable I2C controller after " << n << " ticks.";
    }
    VLOG_IF(1, n) << "Waited " << n
                  << " ticks for the I2C controller to shut down.";
    new_transaction_ = true;
  }
  // Done.
}

util::Status I2cController::ScanBus(
    std::vector<i2c_address_t>* detected_addresses) {
  CHECK_NOTNULL(detected_addresses);
  uint8_t dummy;

  // addresses above 0x77 are reserved
  VLOG(1) << "Starting I2C bus scan...";
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

I2cTransaction I2cController::StartTransaction(i2c_address_t addr) {
  CHECK(xSemaphoreTake(txn_mutex_, portMAX_DELAY));
  return I2cTransaction(this, addr);
}

I2cTransaction::I2cTransaction(I2cController* controller, i2c_address_t addr)
    : controller_(controller), addr_(addr), stopped_(false), restart_(false) {}

I2cTransaction::~I2cTransaction() {
  if (!stopped_) {
    util::Status s = Abort();
    LOG(WARNING) << "Transaction ended before STOP. Abort status: " << s;
    CHECK(xSemaphoreGive(controller_->txn_mutex_));
  }
}
I2cTransaction::I2cTransaction(I2cTransaction&& other)
    : controller_(other.controller_),
      addr_(other.addr_),
      stopped_(other.stopped_),
      restart_(other.restart_) {
  other.stopped_ = true;
}

util::Status I2cTransaction::Write(const uint8_t* buf, size_t len, bool stop) {
  CHECK(!stopped_);
  util::Status status;
  I2cController::Event event{
      .tag = I2cController::Event::Tag::WRITE,
      .tar = addr_.get(),
      .buf = const_cast<uint8_t*>(buf),
      .len = len,
      .restart = restart_,
      .stop = stop,
      .blocking_sem = controller_->shared_blocking_sem_,
      .out_result = &status,
  };
  restart_ = false;
  CHECK(xQueueSendToBack(controller_->event_queue_, &event, portMAX_DELAY));
  CHECK(xSemaphoreTake(controller_->shared_blocking_sem_, portMAX_DELAY));
  if (stop) {
    AfterStop();
  }
  return status;
}

util::Status I2cTransaction::Read(uint8_t* buf, size_t len, bool stop) {
  CHECK(!stopped_);
  util::Status status;
  I2cController::Event event{
      .tag = I2cController::Event::Tag::READ,
      .tar = addr_.get(),
      .buf = buf,
      .len = len,
      .restart = restart_,
      .stop = stop,
      .blocking_sem = controller_->shared_blocking_sem_,
      .out_result = &status,
  };
  restart_ = false;
  CHECK(xQueueSendToBack(controller_->event_queue_, &event, portMAX_DELAY));
  CHECK(xSemaphoreTake(controller_->shared_blocking_sem_, portMAX_DELAY));
  if (stop) {
    AfterStop();
  }
  return status;
}

void I2cTransaction::Restart() { restart_ = true; }

void I2cTransaction::AfterStop() {
  stopped_ = true;
  xSemaphoreGive(controller_->txn_mutex_);
}

util::Status I2cTransaction::Abort() {
  // TODO: implement
  // See RP2040 datasheet section 4.3.10.4
  return util::UnimplementedError("I2cTransaction::Abort()");
}

}  // namespace tplp