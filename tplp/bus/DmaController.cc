#include "tplp/bus/DmaController.h"

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "picolog/picolog.h"

namespace tplp {
namespace {
static DmaController* controllers[DmaController::kMaxNumControllers] = {};

static inline void ExecuteActionFromISR(
    const DmaController::Action& action,
    BaseType_t* higher_priority_task_woken) {
  if (action.toggle_gpio >= 0) {
    gpio_put(action.toggle_gpio, !gpio_get(action.toggle_gpio));
  }
  if (action.give_semaphore) {
    xSemaphoreGiveFromISR(action.give_semaphore, higher_priority_task_woken);
  }
  if (action.notify_task) {
    xTaskNotifyIndexedFromISR(action.notify_task, action.notify_index,
                              action.notify_value, action.notify_action,
                              higher_priority_task_woken);
  }
}

}  // namespace

// TODO: check out the disassembly
template <int irq_index>
[[gnu::hot]] void __not_in_flash_func(DmaController::DmaFinishedISR()) {
  BaseType_t higher_priority_task_woken = 0;
  for (int n = 0; n < kMaxNumControllers; ++n) {
    if (!controllers[n]) break;
    DmaController::ChannelPair* head = controllers[n]->head_;
    // assertion: !head_->launch_ready
    uint32_t ints = irq_index ? dma_hw->ints1 : dma_hw->ints0;

    if (ints & (1u << head->tx)) {
      // SDK bug workaround! dma_irqn_acknowledge uses hw_set_bits,
      // which is buggy for the DMA INTS0/1 registers.
      (irq_index ? dma_hw->ints1 : dma_hw->ints0) = 1u << head->tx;
      // dma_irqn_acknowledge_channel(irq_index, head->tx);
      head->tx_done = true;
    }
    if (ints & (1u << head->rx)) {
      // SDK bug workaround! dma_irqn_acknowledge uses hw_set_bits,
      // which is buggy for the DMA INTS0/1 registers.
      (irq_index ? dma_hw->ints1 : dma_hw->ints0) = 1u << head->rx;
      // dma_irqn_acknowledge_channel(irq_index, head->rx);
      head->rx_done = true;
    }
    if (head->tx_done && head->rx_done) {
      ::tplp::ExecuteActionFromISR(head->action, &higher_priority_task_woken);
      // Move controller's head_ to the next channel pair
      head = controllers[n]->head_ = controllers[n]->RingNext(head);
      if (head->launch_ready) {
        // Launch the next queued DMA transfer!
        head->launch_ready = false;
        dma_start_channel_mask((head->tx_enable ? (1 << head->tx) : 0) |
                               (head->rx_enable ? (1 << head->rx) : 0));
      } else {
        // Reached the end of the queue.
        controllers[n]->active_ = false;
      }
      // Either way there's a new free spot in the queue now.
      xSemaphoreGiveFromISR(controllers[n]->free_slots_sem_,
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
    CHECK_EQ(irq_get_exclusive_handler(DMA_IRQ_0), nullptr)
        << "Interrupt handler already set for DMA_IRQ_0";
    irq_set_exclusive_handler(DMA_IRQ_0, &DmaFinishedISR<0>);
    irq_set_enabled(DMA_IRQ_0, true);
    LOG(INFO) << "IRQ " << DMA_IRQ_0 << " enabled";
    irq0_initialized = true;
  }
  if (irq_index == 1 && !irq1_initialized) {
    CHECK_EQ(irq_get_exclusive_handler(DMA_IRQ_1), nullptr)
        << "Interrupt handler already set for DMA_IRQ_1";
    irq_set_exclusive_handler(DMA_IRQ_1, &DmaFinishedISR<1>);
    irq_set_enabled(DMA_IRQ_1, true);
    LOG(INFO) << "IRQ " << DMA_IRQ_1 << " enabled";
    irq1_initialized = true;
  }

  DmaController* self = CHECK_NOTNULL(new DmaController());

  for (int i = 0; i < kNumChannelPairs; ++i) {
    ChannelPair* pair = &self->ring_[i];
    pair->launch_ready = false;
    pair->tx = dma_channel_t(dma_claim_unused_channel(false));
    LOG_IF(FATAL, pair->tx < 0) << "Not enough free DMA channels available";
    pair->rx = dma_channel_t(dma_claim_unused_channel(false));
    LOG_IF(FATAL, pair->rx < 0) << "Not enough free DMA channels available";
    dma_irqn_set_channel_enabled(irq_index, pair->tx, true);
    dma_irqn_set_channel_enabled(irq_index, pair->rx, true);
    LOG(INFO) << "Claimed DMA channel " << pair->tx << " for TX";
    LOG(INFO) << "Claimed DMA channel " << pair->rx << " for RX";
    CHECK(xSemaphoreGive(self->free_slots_sem_));
  }

  bool ok = false;
  for (int i = 0; i < kMaxNumControllers; ++i) {
    if (!controllers[i]) {
      controllers[i] = self;
      ok = true;
      break;
    }
  }
  CHECK(ok) << "Too many DmaControllers; increase kMaxNumControllers from "
            << kMaxNumControllers;
  return self;
}

DmaController::DmaController()
    : tail_mutex_(CHECK_NOTNULL(xSemaphoreCreateMutex())),
      free_slots_sem_(
          CHECK_NOTNULL(xSemaphoreCreateCounting(kNumChannelPairs, 0))),
      ring_(),
      ring_end_(ring_ + kNumChannelPairs),
      head_(ring_),
      tail_(ring_),
      active_(false) {
  for (int i = 0; i < kNumChannelPairs; ++i) {
    ring_[i] = ChannelPair();
  }
}

void DmaController::Transfer(const Request& req) {
  static_assert((int)TransferWidth::k8 == DMA_SIZE_8);
  static_assert((int)TransferWidth::k16 == DMA_SIZE_16);
  static_assert((int)TransferWidth::k32 == DMA_SIZE_32);
  static_assert(DREQ_FORCE == 0x3f);

  VLOG(2) << "Transfer() acquiring locks";
  CHECK(req.tx_enable || req.rx_enable);
  CHECK(xSemaphoreTake(tail_mutex_, portMAX_DELAY));
  CHECK(xSemaphoreTake(free_slots_sem_, portMAX_DELAY));
  VLOG(2) << "Transfer() configuring channels";

  ChannelPair* const tail = tail_;
  CHECK(!tail->launch_ready);
  tail->tx_enable = req.tx_enable;
  tail->rx_enable = req.rx_enable;
  tail->tx_done = !req.tx_enable;
  tail->rx_done = !req.rx_enable;
  tail->action = req.action;

  dma_channel_config c = {};
  // Common options
  channel_config_set_ring(&c, false, 0);
  channel_config_set_bswap(&c, false);
  channel_config_set_irq_quiet(&c, false);
  channel_config_set_enable(&c, true);
  channel_config_set_sniff_enable(&c, false);
  channel_config_set_high_priority(&c, false);
  channel_config_set_transfer_data_size(
      &c, static_cast<dma_channel_transfer_size>(req.transfer_width));
  CHECK_GT(req.transfer_count, 0u);

  if (req.tx_enable && req.rx_enable &&
      (req.tx_dreq != DREQ_FORCE || req.rx_dreq != DREQ_FORCE)) {
    CHECK_NE(req.tx_dreq, req.rx_dreq);
  }

  if (req.tx_enable) {
    channel_config_set_read_increment(&c, req.tx_read_incr);
    channel_config_set_write_increment(&c, req.tx_write_incr);
    channel_config_set_dreq(&c, req.tx_dreq);
    channel_config_set_chain_to(&c, tail->tx);  // self-chain disables it
    dma_channel_set_config(tail->tx, &c, false);

    CHECK(req.tx_read);
    CHECK(req.tx_write);
    dma_channel_set_read_addr(tail->tx, req.tx_read, false);
    dma_channel_set_write_addr(tail->tx, req.tx_write, false);
    dma_channel_set_trans_count(tail->tx, req.transfer_count, false);
  }

  if (req.rx_enable) {
    channel_config_set_read_increment(&c, req.rx_read_incr);
    channel_config_set_write_increment(&c, req.rx_write_incr);
    channel_config_set_dreq(&c, req.rx_dreq);
    channel_config_set_chain_to(&c, tail->rx);  // self-chain disables it
    dma_channel_set_config(tail->rx, &c, false);

    CHECK(req.tx_read);
    CHECK(req.tx_write);
    dma_channel_set_read_addr(tail->rx, req.rx_read, false);
    dma_channel_set_write_addr(tail->rx, req.rx_write, false);
    dma_channel_set_trans_count(tail->rx, req.transfer_count, false);
  }

  VLOG(2) << "Transfer() checking for activity";
  if (active_) {
    VLOG(1) << "Enqueue, count=" << req.transfer_count << " @ slot "
            << (tail_ - ring_);
    tail->launch_ready = true;
  } else {
    VLOG(1) << "Immediate launch, transfer count=" << req.transfer_count;
    CHECK_EQ(head_, tail) << "Queue not empty while inactive";
    CHECK(!tail->launch_ready);
    CHECK(!dma_channel_is_busy(tail->tx));
    CHECK(!dma_channel_is_busy(tail->rx));
    active_ = true;

    dma_start_channel_mask((tail->tx_enable ? (1 << tail->tx) : 0) |
                           (tail->rx_enable ? (1 << tail->rx) : 0));
  }
  VLOG(2) << "Transfer() advancing tail";
  tail_ = RingNext(tail);
  VLOG(2) << "Transfer() releasing tail mutex";
  xSemaphoreGive(tail_mutex_);
  VLOG(2) << "Transfer() done";
}

int DmaController::PeekQueueLength() const {
  return kNumChannelPairs - uxSemaphoreGetCount(free_slots_sem_);
}

}  // namespace tplp