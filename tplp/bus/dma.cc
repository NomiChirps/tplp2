#include "tplp/bus/dma.h"

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "picolog/picolog.h"

namespace tplp {
namespace {
static DmaController* controllers[NUM_DMA_CHANNELS] = {};

static inline void ExecuteActionFromISR(
    const std::optional<DmaController::Action>& action,
    BaseType_t* higher_priority_task_woken) {
  if (!action) return;
  if (action->toggle_gpio >= 0) {
    gpio_put(action->toggle_gpio, !gpio_get(action->toggle_gpio));
  }
  if (action->give_semaphore) {
    xSemaphoreGiveFromISR(action->give_semaphore, higher_priority_task_woken);
  }
  if (action->notify_task) {
    xTaskNotifyIndexedFromISR(action->notify_task, action->notify_index,
                              action->notify_value, action->notify_action,
                              higher_priority_task_woken);
  }
}

static inline void ConfigureAndLaunchImmediately(
    dma_channel_t c0, bool c0_enable, const uint32_t c0_config[4],
    dma_channel_t c1, bool c1_enable, const uint32_t c1_config[4]) {
  if (c0_enable) {
    dma_channel_hw_addr(c0)->read_addr = c0_config[0];
    dma_channel_hw_addr(c0)->write_addr = c0_config[1];
    dma_channel_hw_addr(c0)->transfer_count = c0_config[2];
    dma_channel_hw_addr(c0)->al1_ctrl = c0_config[3];
  }
  if (c1_enable) {
    dma_channel_hw_addr(c1)->read_addr = c1_config[0];
    dma_channel_hw_addr(c1)->write_addr = c1_config[1];
    dma_channel_hw_addr(c1)->transfer_count = c1_config[2];
    dma_channel_hw_addr(c1)->al1_ctrl = c1_config[3];
  }
  dma_start_channel_mask((c0_enable ? (1 << c0) : 0) |
                         (c1_enable ? (1 << c1) : 0));
}

}  // namespace

template <int irq_index>
[[gnu::hot]] void __not_in_flash_func(DmaController::DmaFinishedISR()) {
  static io_rw_32* const ints = irq_index ? &dma_hw->ints1 : &dma_hw->ints0;
  BaseType_t higher_priority_task_woken = 0;
  int ch;
  static_assert(NUM_DMA_CHANNELS <= 16);
  while ((ch = __builtin_ctz(*ints)) < 16) {
    DmaController* controller = controllers[ch];
    if (!controller) break;
    // Acknowledge the interrupt.
    // Workaround for https://github.com/raspberrypi/pico-sdk/issues/974
    *ints = 1u << ch;

    DmaController::TransferSlot* head = &controller->active_transfer_;
    // assertion: !head_->launch_ready

    if (ch == controller->c0_) {
      ::tplp::ExecuteActionFromISR(head->c0_action,
                                   &higher_priority_task_woken);
      head->c0_done = true;
    } else if (ch == controller->c1_) {
      ::tplp::ExecuteActionFromISR(head->c1_action,
                                   &higher_priority_task_woken);
      head->c1_done = true;
    }
    if (head->c0_done && head->c1_done) {
      ::tplp::ExecuteActionFromISR(head->both_action,
                                   &higher_priority_task_woken);
      // Launch the next queued transfer, if there is one.
      if (xQueueReceiveFromISR(controller->queue_,
                               &controller->active_transfer_,
                               &higher_priority_task_woken)) {
        ConfigureAndLaunchImmediately(controller->c0_, head->c0_enable,
                                      head->c0_config, controller->c1_,
                                      head->c1_enable, head->c1_config);
      }
      controller->completed_transfers_count_++;
    }
  }
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

template void DmaController::DmaFinishedISR<0>();
template void DmaController::DmaFinishedISR<1>();

DmaController* DmaController::Init(dma_irq_index_t irq_index) {
  CHECK(irq_index == 0 || irq_index == 1);
  static bool irq0_initialized = false;
  static bool irq1_initialized = false;
  if (irq_index == 0 && !irq0_initialized) {
    if (irq_get_exclusive_handler(DMA_IRQ_0) != &DmaFinishedISR<0>) {
      CHECK_EQ(irq_get_exclusive_handler(DMA_IRQ_0), nullptr)
          << "Interrupt handler already set for DMA_IRQ_0";
      irq_set_exclusive_handler(DMA_IRQ_0, &DmaFinishedISR<0>);
      irq_set_enabled(DMA_IRQ_0, true);
      LOG(INFO) << "IRQ " << DMA_IRQ_0 << " enabled";
    }
    irq0_initialized = true;
  }
  if (irq_index == 1 && !irq1_initialized) {
    if (irq_get_exclusive_handler(DMA_IRQ_0) != &DmaFinishedISR<1>) {
      CHECK_EQ(irq_get_exclusive_handler(DMA_IRQ_1), nullptr)
          << "Interrupt handler already set for DMA_IRQ_1";
      irq_set_exclusive_handler(DMA_IRQ_1, &DmaFinishedISR<1>);
      irq_set_enabled(DMA_IRQ_1, true);
      LOG(INFO) << "IRQ " << DMA_IRQ_1 << " enabled";
    }
    irq1_initialized = true;
  }

  dma_channel_t c0 = dma_channel_t(dma_claim_unused_channel(false));
  LOG_IF(FATAL, c0 < 0) << "Not enough free DMA channels available";
  LOG(INFO) << "Claimed DMA channel " << c0;
  dma_channel_t c1 = dma_channel_t(dma_claim_unused_channel(false));
  LOG_IF(FATAL, c1 < 0) << "Not enough free DMA channels available";
  LOG(INFO) << "Claimed DMA channel " << c1;
  dma_irqn_set_channel_enabled(irq_index, c0, true);
  dma_irqn_set_channel_enabled(irq_index, c1, true);

  DmaController* self = CHECK_NOTNULL(new DmaController(irq_index, c0, c1));

  CHECK(!controllers[c0]);
  controllers[c0] = self;
  CHECK(!controllers[c1]);
  controllers[c1] = self;

  return self;
}

DmaController::DmaController(dma_irq_index_t irq_index, dma_channel_t c0,
                             dma_channel_t c1)
    : irq_index_(irq_index),
      c0_(c0),
      c1_(c1),
      queue_(CHECK_NOTNULL(xQueueCreate(kQueueLength, sizeof(TransferSlot)))),
      transfer_mutex_(CHECK_NOTNULL(xSemaphoreCreateMutex())),
      active_transfer_(),
      queued_transfers_count_(0),
      completed_transfers_count_(0) {}

DmaController::TransferHandle DmaController::Transfer(const Request& req) {
  static_assert((int)DataSize::k8 == DMA_SIZE_8);
  static_assert((int)DataSize::k16 == DMA_SIZE_16);
  static_assert((int)DataSize::k32 == DMA_SIZE_32);
  static_assert(DREQ_FORCE == 0x3f);

  CHECK(req.c0_enable || req.c1_enable);

  TransferSlot cfg;
  cfg.c0_enable = req.c0_enable;
  cfg.c1_enable = req.c1_enable;
  cfg.c0_done = !req.c0_enable;
  cfg.c1_done = !req.c1_enable;

  dma_channel_config c = {};
  // Options not (yet?) exposed in the Request struct.
  channel_config_set_bswap(&c, false);
  channel_config_set_irq_quiet(&c, false);
  channel_config_set_sniff_enable(&c, false);
  channel_config_set_high_priority(&c, false);

  if (req.c0_enable && req.c1_enable &&
      (req.c0_treq_sel != DREQ_FORCE || req.c1_treq_sel != DREQ_FORCE)) {
    CHECK_NE(req.c0_treq_sel, req.c1_treq_sel)
        << "Two DMA channels must not use the same DREQ at the same time. "
           "treq_sel="
        << req.c0_treq_sel;
  }

  if (req.c0_enable) {
    CHECK_GT(req.c0_trans_count, 0u)
        << "enabled channel must have a nonzero trans_count";
    channel_config_set_enable(&c, req.c0_enable);
    channel_config_set_read_increment(&c, req.c0_read_incr);
    channel_config_set_write_increment(&c, req.c0_write_incr);
    channel_config_set_dreq(&c, req.c0_treq_sel);
    channel_config_set_chain_to(&c, c0_);  // self-chain disables it
    channel_config_set_transfer_data_size(
        &c, static_cast<dma_channel_transfer_size>(req.c0_data_size));
    channel_config_set_ring(&c, req.c0_ring_sel, req.c0_ring_size);
    CHECK(req.c0_read_addr);
    CHECK(req.c0_write_addr);
    cfg.c0_config[0] = reinterpret_cast<uint32_t>(req.c0_read_addr);
    cfg.c0_config[1] = reinterpret_cast<uint32_t>(req.c0_write_addr);
    cfg.c0_config[2] = req.c0_trans_count;
    cfg.c0_config[3] = c.ctrl;
    cfg.c0_action = req.c0_action;
  }

  if (req.c1_enable) {
    CHECK_GT(req.c1_trans_count, 0u)
        << "enabled channel must have a nonzero trans_count";
    channel_config_set_enable(&c, req.c1_enable);
    channel_config_set_read_increment(&c, req.c1_read_incr);
    channel_config_set_write_increment(&c, req.c1_write_incr);
    channel_config_set_dreq(&c, req.c1_treq_sel);
    channel_config_set_chain_to(&c, c1_);  // self-chain disables it
    channel_config_set_transfer_data_size(
        &c, static_cast<dma_channel_transfer_size>(req.c1_data_size));
    channel_config_set_ring(&c, req.c1_ring_sel, req.c1_ring_size);
    CHECK(req.c1_read_addr);
    CHECK(req.c1_write_addr);
    cfg.c1_config[0] = reinterpret_cast<uint32_t>(req.c1_read_addr);
    cfg.c1_config[1] = reinterpret_cast<uint32_t>(req.c1_write_addr);
    cfg.c1_config[2] = req.c1_trans_count;
    cfg.c1_config[3] = c.ctrl;
    cfg.c1_action = req.c1_action;
  }

  if (req.c0_enable && req.c1_enable) {
    cfg.both_action = req.both_action;
  }

  CHECK(xSemaphoreTake(transfer_mutex_, portMAX_DELAY));
  VLOG(2) << completed_transfers_count_ << " / " << queued_transfers_count_;
  CHECK(xQueueSendToBack(queue_, &cfg, portMAX_DELAY));
  if (queued_transfers_count_ == completed_transfers_count_) {
    // ISR isn't running. Launch queued thing (if it hasn't been already).
    if (xQueueReceive(queue_, &active_transfer_, 0)) {
      VLOG(1) << "Transfer(): immediate launch";
      CHECK(!dma_channel_is_busy(c0_));
      CHECK(!dma_channel_is_busy(c1_));
      ConfigureAndLaunchImmediately(c0_, cfg.c0_enable, cfg.c0_config, c1_,
                                    cfg.c1_enable, cfg.c1_config);
    } else {
      VLOG(1) << "Transfer(): already done";
    }
  } else {
    VLOG(1) << "Transfer(): pending (" << PeekQueueLength() << ")";
  }
  TransferHandle handle(this, queued_transfers_count_++);
  CHECK(xSemaphoreGive(transfer_mutex_));
  return handle;
}

std::array<uint32_t, 2> DmaController::TransferHandle::Abort() {
  if (finished()) {
    VLOG(1) << "Abort: already finished";
    return {0, 0};
  }
  CHECK(started())
      << "Aborting a queued but not started transfer is not supported";
  const uint32_t channels_mask = (1u << dma_->c0_) | (1u << dma_->c1_);
  volatile uint32_t* const inte =
      dma_->irq_index_ ? &dma_hw->inte1 : &dma_hw->inte0;
  volatile uint32_t* const ints =
      dma_->irq_index_ ? &dma_hw->ints1 : &dma_hw->ints0;
  std::array<uint32_t, 2> remaining_trans_count;
  // "Due to RP2040-E13, aborting a DMA channel that is making progress (i.e.
  // not stalled on an inactive DREQ) may cause a completion IRQ to assert. The
  // channel interrupt enable should be cleared before performing the abort, and
  // the interrupt should be checked and cleared after the abort."
  // Disable channel interrupts.
  hw_clear_bits(inte, channels_mask);
  if (finished()) {
    VLOG(1) << "Abort: already finished";
    // Re-enable interrupt
    hw_set_bits(inte, channels_mask);
    return {0, 0};
  }
  // Abort channels simultaneously.
  dma_hw->abort = (1u << dma_->c0_) | (1u << dma_->c1_);
  // Bit will go 0 once channel has reached safe state
  // (i.e. any in-flight transfers have retired)
  while (dma_hw->ch[dma_->c0_].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS) {
    tight_loop_contents();
  }
  while (dma_hw->ch[dma_->c1_].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS) {
    tight_loop_contents();
  }
  // Clear spurious completion interrupts (errata RP2040-E13)
  *ints = channels_mask;
  remaining_trans_count[0] = dma_channel_hw_addr(dma_->c0_)->transfer_count;
  remaining_trans_count[1] = dma_channel_hw_addr(dma_->c1_)->transfer_count;
  VLOG(1) << "Abort() remaining_trans_count = " << remaining_trans_count[0]
          << ", " << remaining_trans_count[1];
  // Skip this transfer's actions, but launch the next one if necessary
  CHECK(xSemaphoreTake(dma_->transfer_mutex_, portMAX_DELAY));
  // TODO: address code duplication between here and the ISR
  dma_->completed_transfers_count_++;
  if (xQueueReceive(dma_->queue_, &dma_->active_transfer_, 0)) {
    VLOG(1) << "Abort: launching next in queue";
    ConfigureAndLaunchImmediately(dma_->c0_, dma_->active_transfer_.c0_enable,
                                  dma_->active_transfer_.c0_config, dma_->c1_,
                                  dma_->active_transfer_.c1_enable,
                                  dma_->active_transfer_.c1_config);
  } else {
    VLOG(1) << "Abort: reached end of queue";
  }
  CHECK(xSemaphoreGive(dma_->transfer_mutex_));

  // Re-enable interrupt
  hw_set_bits(inte, channels_mask);
  return remaining_trans_count;
}

int DmaController::PeekQueueLength() const {
  return uxQueueMessagesWaiting(queue_);
}

}  // namespace tplp