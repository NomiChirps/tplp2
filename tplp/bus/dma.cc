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

// TODO: check out the disassembly
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

    DmaController::TransferSlot* head = controller->head_;
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
      // Move controller's head_ to the next channel pair
      head = controller->head_ = controller->RingNext(head);
      if (head->launch_ready) {
        // Launch the next queued DMA transfer!
        head->launch_ready = false;
        ConfigureAndLaunchImmediately(controller->c0_, head->c0_enable,
                                      head->c0_config, controller->c1_,
                                      head->c1_enable, head->c1_config);
      } else {
        // Reached the end of the queue.
        controller->active_ = false;
      }
      // Either way there's a new free spot in the queue now.
      controller->completed_transfers_count_++;
      xSemaphoreGiveFromISR(controller->free_slots_sem_,
                            &higher_priority_task_woken);
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
      tail_mutex_(CHECK_NOTNULL(xSemaphoreCreateMutex())),
      free_slots_sem_(
          CHECK_NOTNULL(xSemaphoreCreateCounting(kQueueLength, kQueueLength))),
      ring_(),
      ring_end_(ring_ + kQueueLength),
      head_(ring_),
      tail_(ring_),
      queued_transfers_count_(0),
      completed_transfers_count_(0),
      active_(false) {
  for (size_t i = 0; i < kQueueLength; ++i) {
    ring_[i] = TransferSlot();
  }
}

DmaController::TransferHandle DmaController::Transfer(const Request& req) {
  static_assert((int)DataSize::k8 == DMA_SIZE_8);
  static_assert((int)DataSize::k16 == DMA_SIZE_16);
  static_assert((int)DataSize::k32 == DMA_SIZE_32);
  static_assert(DREQ_FORCE == 0x3f);

  VLOG(2) << "Transfer() acquiring locks";
  CHECK(req.c0_enable || req.c1_enable);
  CHECK(xSemaphoreTake(tail_mutex_, portMAX_DELAY));
  CHECK(xSemaphoreTake(free_slots_sem_, portMAX_DELAY));
  VLOG(2) << "Transfer() configuring channels";

  TransferSlot* const tail = tail_;
  CHECK(!tail->launch_ready);
  tail->c0_enable = req.c0_enable;
  tail->c1_enable = req.c1_enable;
  tail->c0_done = !req.c0_enable;
  tail->c1_done = !req.c1_enable;

  dma_channel_config c = {};
  // Common options
  channel_config_set_ring(&c, false, 0);
  channel_config_set_bswap(&c, false);
  channel_config_set_irq_quiet(&c, false);
  channel_config_set_enable(&c, true);
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
    channel_config_set_read_increment(&c, req.c0_read_incr);
    channel_config_set_write_increment(&c, req.c0_write_incr);
    channel_config_set_dreq(&c, req.c0_treq_sel);
    channel_config_set_chain_to(&c, c0_);  // self-chain disables it
    channel_config_set_transfer_data_size(
        &c, static_cast<dma_channel_transfer_size>(req.c0_data_size));
    CHECK(req.c0_read_addr);
    CHECK(req.c0_write_addr);
    tail->c0_config[0] = reinterpret_cast<uint32_t>(req.c0_read_addr);
    tail->c0_config[1] = reinterpret_cast<uint32_t>(req.c0_write_addr);
    tail->c0_config[2] = req.c0_trans_count;
    tail->c0_config[3] = c.ctrl;
    tail->c0_action = req.c0_action;
  }

  if (req.c1_enable) {
    CHECK_GT(req.c1_trans_count, 0u)
        << "enabled channel must have a nonzero trans_count";
    channel_config_set_read_increment(&c, req.c1_read_incr);
    channel_config_set_write_increment(&c, req.c1_write_incr);
    channel_config_set_dreq(&c, req.c1_treq_sel);
    channel_config_set_chain_to(&c, c1_);  // self-chain disables it
    channel_config_set_transfer_data_size(
        &c, static_cast<dma_channel_transfer_size>(req.c1_data_size));
    CHECK(req.c1_read_addr);
    CHECK(req.c1_write_addr);
    tail->c1_config[0] = reinterpret_cast<uint32_t>(req.c1_read_addr);
    tail->c1_config[1] = reinterpret_cast<uint32_t>(req.c1_write_addr);
    tail->c1_config[2] = req.c1_trans_count;
    tail->c1_config[3] = c.ctrl;
    tail->c1_action = req.c1_action;
  }

  if (req.c0_enable && req.c1_enable) {
    tail->both_action = req.both_action;
  }

  VLOG(2) << "Transfer() checking for activity";
  if (active_) {
    VLOG(1) << "Enqueue @ slot " << (tail_ - ring_);
    tail->launch_ready = true;
  } else {
    VLOG(1) << "Immediate launch";
    CHECK_EQ(head_, tail) << "Queue not empty while inactive";
    CHECK(!tail->launch_ready);
    CHECK(!dma_channel_is_busy(c0_));
    CHECK(!dma_channel_is_busy(c1_));
    active_ = true;
    ConfigureAndLaunchImmediately(c0_, tail->c0_enable, tail->c0_config, c1_,
                                  tail->c1_enable, tail->c1_config);
  }
  VLOG(2) << "Transfer() advancing tail";
  tail_ = RingNext(tail);
  TransferHandle handle(this, queued_transfers_count_++);
  VLOG(2) << "Transfer() releasing tail mutex";
  xSemaphoreGive(tail_mutex_);
  VLOG(2) << "Transfer() done";
  return handle;
}

void DmaController::TransferHandle::Abort() {
  if (finished()) {
    VLOG(1) << "Abort: already finished";
    return;
  }
  CHECK(started())
      << "Aborting a queued but not started transfer is not supported";
  // Disable interrupts (narrowly)
  dma_irqn_set_channel_enabled(dma_->irq_index_, dma_->c0_, false);
  dma_irqn_set_channel_enabled(dma_->irq_index_, dma_->c1_, false);
  if (finished()) {
    VLOG(1) << "Abort: already finished";
    // Nothing to do already!
  } else {
    // NB: dma_channel_abort busy-waits!
    dma_channel_abort(dma_->c0_);
    dma_channel_abort(dma_->c1_);
    // Clear spurious completion interrupts (errata RP2040-E13)
    dma_irqn_acknowledge_channel(dma_->irq_index_, dma_->c0_);
    dma_irqn_acknowledge_channel(dma_->irq_index_, dma_->c1_);
    // Skip this transfer's actions, but launch the next one if necessary
    // TODO: address code duplication between here and the ISR
    dma_->head_ = dma_->RingNext(dma_->head_);
    if (dma_->head_->launch_ready) {
      VLOG(1) << "Abort: launching next in queue";
      ConfigureAndLaunchImmediately(
          dma_->c0_, dma_->head_->c0_enable, dma_->head_->c0_config, dma_->c1_,
          dma_->head_->c1_enable, dma_->head_->c1_config);
    } else {
      VLOG(1) << "Abort: reached end of queue";
      // Reached the end of the queue.
      dma_->active_ = false;
    }
    dma_->completed_transfers_count_++;
    xSemaphoreGive(dma_->free_slots_sem_);
  }
  // Reenable interrupts
  dma_irqn_set_channel_enabled(dma_->irq_index_, dma_->c0_, true);
  dma_irqn_set_channel_enabled(dma_->irq_index_, dma_->c1_, true);
}

int DmaController::PeekQueueLength() const {
  return kQueueLength - uxSemaphoreGetCount(free_slots_sem_);
}

}  // namespace tplp