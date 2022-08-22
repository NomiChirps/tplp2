#ifndef TPLP_BUS_DMACONTROLLER_H_
#define TPLP_BUS_DMACONTROLLER_H_

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"
#include "tplp/bus/types.h"

namespace tplp {

class DmaController {
 public:
  // TODO: what should this be?
  static constexpr size_t kQueueLength = 4;

 public:
  // `irq_index` must be 0 or 1; this sets an interrupt handler on that
  // DMA unit's IRQ. Do not attempt to initialize multiple DmaControllers
  // concurrently.
  static DmaController* Init(dma_irq_index_t irq_index);

  enum class DataSize { k8 = 0, k16 = 1, k32 = 2 };
  struct Action {
    gpio_pin_t toggle_gpio = gpio_pin_t(-1);
    SemaphoreHandle_t give_semaphore = nullptr;

    TaskHandle_t notify_task = nullptr;
    UBaseType_t notify_index = 0;
    uint32_t notify_value = 0;
    eNotifyAction notify_action = eNoAction;
  };

  // Either c0_enable or c1_enable or both must be true.
  // trans_count must be greater than zero.
  struct Request {
    bool c0_enable = false;
    // Default value of 0x3f means an unpaced transfer.
    uint32_t c0_treq_sel = 0x3f;
    volatile const void* c0_read_addr = nullptr;
    bool c0_read_incr = true;
    volatile void* c0_write_addr = nullptr;
    bool c0_write_incr = true;

    bool c1_enable = false;
    // Default value of 0x3f means an unpaced transfer.
    uint32_t c1_treq_sel = 0x3f;
    volatile const void* c1_read_addr = nullptr;
    bool c1_read_incr = true;
    volatile void* c1_write_addr = nullptr;
    bool c1_write_incr = true;

    DataSize data_size = DataSize::k8;
    // Number of `data_size`-width transfers to perform.
    size_t trans_count = 0;

    // Action to take from the interrupt handler when this
    // pair of DMAs completes.
    Action action;
  };

  // Queues up a DMA with the given configuration.
  //
  // Returns immediately after enqueueing the transfer. `action` specifies an
  // optional set of actions to take when the transfer has been completed. The
  // actions run in an interrupt context, as soon as feasible after the DMA has
  // completed and before the next one is started.
  void Transfer(const Request& req);

  // Returns the current length of the transfer request queue, approximately.
  int PeekQueueLength() const;

 private:
  explicit DmaController(dma_channel_t c0, dma_channel_t c1);
  ~DmaController() = delete;
  DmaController(const DmaController&) = delete;
  DmaController& operator=(const DmaController&) = delete;

  template <int irq_index>
  static void DmaFinishedISR();

 private:
  const dma_channel_t c0_;
  const dma_channel_t c1_;
  const SemaphoreHandle_t tail_mutex_;
  const SemaphoreHandle_t free_slots_sem_;
  struct ChannelPair {
    // Whether channel 0 should run.
    bool c0_enable = 0;
    // Whether channel 1 should run.
    bool c1_enable = 0;

    // Control register values:
    // [read_addr, write_addr, trans_count, ctrl]
    uint32_t c0_config[4];
    uint32_t c1_config[4];

    // True if channel 0 finished or was not enabled.
    bool c0_done = 0;
    // True if channel 1 finished or was not enabled.
    bool c1_done = 0;

    // What to do after all enabled channels finish.
    Action action;

    // If true, channel configs are filled out.
    bool launch_ready = 0;
  };
  ChannelPair ring_[kQueueLength];
  const ChannelPair* const ring_end_;
  inline ChannelPair* RingNext(ChannelPair* p) {
    if (p + 1 == ring_end_)
      return ring_;
    else
      return p + 1;
  }

  // Currently active channel pair. Never null.
  // Readers: DmaFinishedISR() only
  // Writers: DmaFinishedISR() only
  ChannelPair* head_;
  // Next free channel pair.
  // Readers: Transfer() only
  // Writers: Transfer() only
  ChannelPair* tail_;

  // If false, no transfer is in progress, and DmaFinishedISR() cannot trigger.
  // If true, it's more complicated than that.
  bool active_;
};

}  // namespace tplp

#endif  // TPLP_BUS_DMACONTROLLER_H_