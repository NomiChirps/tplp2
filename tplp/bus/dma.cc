#include "tplp/bus/dma.h"

#include <cstring>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "picolog/picolog.h"

namespace tplp {
namespace {

// A couple of assertions that depend on pico-sdk headers
static_assert((int)ChannelCtrl::DataSize::k8 == DMA_SIZE_8);
static_assert((int)ChannelCtrl::DataSize::k16 == DMA_SIZE_16);
static_assert((int)ChannelCtrl::DataSize::k32 == DMA_SIZE_32);

// Each DmaController will allocate one queue and add a pointer to it here for
// every channel it manages.
static DmaController* controllers[NUM_DMA_CHANNELS] = {};

void __not_in_flash_func(RunActionFromISR)(
    const Action& action, BaseType_t* higher_priority_task_woken) {
  if (action.toggle_gpio >= 0) {
    gpio_put(action.toggle_gpio, !gpio_get(action.toggle_gpio));
  }
  if (action.give_semaphore) {
    xSemaphoreGiveFromISR(action.give_semaphore, higher_priority_task_woken);
  }
  if (action.notify_task) {
    xTaskGenericNotifyFromISR(action.notify_task, action.notify_index,
                              action.notify_value, action.notify_action,
                              nullptr, higher_priority_task_woken);
  }
}

}  // namespace

template <int irq_index>
[[gnu::hot]] void __not_in_flash_func(DmaController_ProgramFinishedISR)() {
  constexpr io_rw_32* ints = irq_index ? &dma_hw->ints1 : &dma_hw->ints0;
  uint32_t ints_val;
  BaseType_t higher_priority_task_woken = 0;
  while ((ints_val = *ints)) {
    int ch = __builtin_ctz(ints_val);
    // acknowledge interrupt
    *ints = 1 << ch;
    DmaController* controller = controllers[ch];
    if (!--controller->active_chain_pending_channels_) {
      // This chain is done!
      if (controller->active_chain_->after) {
        RunActionFromISR(*controller->active_chain_->after,
                         &higher_priority_task_woken);
      }
      CompiledChain0* next;
      if (xQueueReceiveFromISR(controller->queue_, &next,
                               &higher_priority_task_woken)) {
        controller->LaunchImmediateFromISR(next, &higher_priority_task_woken);
      }
    }
  }
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

DmaController::DmaController(
    const std::array<uint8_t, kMaxSimultaneousTransfers>& programmer_channels,
    const std::array<uint8_t, kMaxSimultaneousTransfers>& execution_channels,
    QueueHandle_t queue, SemaphoreHandle_t active_chain_mutex)
    : programmer_channels_(programmer_channels),
      execution_channels_(execution_channels),
      queue_(queue),
      active_chain_mutex_(active_chain_mutex),
      active_chain_(nullptr) {}

DmaController* DmaController::Init(dma_irq_index_t irq_index) {
  QueueHandle_t queue = CHECK_NOTNULL(
      // NOLINTNEXTLINE(bugprone-sizeof-expression)
      xQueueCreate(kQueueLength, sizeof(CompiledChain0*)));

  std::array<uint8_t, kMaxSimultaneousTransfers> pcs;
  std::array<uint8_t, kMaxSimultaneousTransfers> ecs;
  for (int t = 0; t < kMaxSimultaneousTransfers; ++t) {
    pcs[t] = dma_claim_unused_channel(false);
    CHECK_GE(pcs[t], 0) << "not enough free DMA channels";
    ecs[t] = dma_claim_unused_channel(false);
    CHECK_GE(ecs[t], 0) << "not enough free DMA channels";

    // Only the execution channels will generate interrupts,
    // as both channels are set to irq_quiet mode, and only
    // the execution channels ever get a null-trigger.
    dma_irqn_set_channel_enabled(irq_index, ecs[t], true);
  }
  irq_set_enabled(irq_index ? DMA_IRQ_1 : DMA_IRQ_0, true);

  SemaphoreHandle_t active_chain_mutex = CHECK_NOTNULL(xSemaphoreCreateMutex());
  DmaController* controller =
      CHECK_NOTNULL(new DmaController(pcs, ecs, queue, active_chain_mutex));
  for (int t = 0; t < kMaxSimultaneousTransfers; ++t) {
    CHECK(!controllers[pcs[t]]) << "twice-claimed dma channel?";
    CHECK(!controllers[ecs[t]]) << "twice-claimed dma channel?";
    controllers[pcs[t]] = controller;
    controllers[ecs[t]] = controller;
  }
  return controller;
}

DmaProgram DmaController::NewProgram() {
  return DmaProgram(programmer_channels_, execution_channels_);
}

void DmaController::LaunchImmediate(CompiledChain0* cc) {
  CHECK_EQ(active_chain_pending_channels_, 0);
  CHECK_EQ(active_chain_, nullptr);
  BaseType_t ignored;
  LaunchImmediateFromISR(cc, &ignored);
}

void __not_in_flash_func(DmaController::LaunchImmediateFromISR)(
    CompiledChain0* cc, BaseType_t* higher_priority_task_woken) {
  // assuming active_chain_pending_channels_ == 0
  for (int t = 0; t < kMaxSimultaneousTransfers; ++t) {
    if (cc->enable[t]) {
      active_chain_pending_channels_++;
    }
  }
  active_chain_ = cc;
  // Make sure we don't trigger any channels before the count is complete!
  __compiler_memory_barrier();

  if (cc->before) {
    RunActionFromISR(*cc->before, higher_priority_task_woken);
  }
  for (int t = 0; t < kMaxSimultaneousTransfers; ++t) {
    if (cc->enable[t]) {
      // Write initial config of execution channel, without triggering it.
      uint32_t* initial_config_read = cc->initial_config[t];
      uint32_t* initial_config_write = cc->initial_config_write_addr[t];
      switch (cc->initial_config_write_length[t]) {
        case 3:
          *initial_config_write++ = *initial_config_read++;
        case 2:
          *initial_config_write++ = *initial_config_read++;
        case 1:
          *initial_config_write++ = *initial_config_read++;
      }

      // Fully configure and trigger the programmer channel!
      dma_channel_hw_t* p = dma_channel_hw_addr(programmer_channels_[t]);
      p->read_addr = cc->programmer_config[t].read_addr;
      p->write_addr = cc->programmer_config[t].write_addr;
      p->transfer_count = cc->programmer_config[t].trans_count;
      p->ctrl_trig = cc->programmer_config[t].ctrl_trig;
    }
  }
}

void DmaController::Enqueue(const DmaProgram* p) {
  for (const CompiledChain1& cc1 : p->contents()) {
    CHECK(xQueueSendToBack(queue_, &cc1.cc0, portMAX_DELAY));
  }
  if (!active_chain_) {
    xSemaphoreTake(active_chain_mutex_, portMAX_DELAY);
    if (!active_chain_) {
      CompiledChain0* head;
      if (xQueueReceive(queue_, &head, 0)) {
        LaunchImmediate(head);
      } else {
        // Someone else must have started and finished the queue in between us
        // pushing our last chain and checking active_chain_. Nothing to do.
      }
    }
    xSemaphoreGive(active_chain_mutex_);
  }
}

}  // namespace tplp