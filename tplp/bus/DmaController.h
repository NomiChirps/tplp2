#ifndef TPLP_BUS_DMACONTROLLER_H_
#define TPLP_BUS_DMACONTROLLER_H_

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"
#include "tplp/bus/types.h"

namespace tplp {

// TODO: generalize from channel pairs to tuples...?
// we could also in principle support dma chains within a request
// (but probably not across requests)
// TODO: allow multiple channel-pairs to be active at once
// TODO: this is okay i guess, but we could also use dma chaining to skip even
// the interrupt handler
class DmaController {
 public:
  // TODO: not clear what this should be
  // does it make sense for it to be configurable per controller?
  // would it slow things down if it was non-constexpr? (probably not)
  static constexpr int kNumChannelPairs = 2;
  static constexpr int kMaxNumControllers = NUM_DMA_CHANNELS / (kNumChannelPairs*2);
  static_assert(kMaxNumControllers > 0);

 public:
  // `irq_index` must be 0 or 1; this sets an interrupt handler on that
  // DMA unit's IRQ. Do not attempt to initialize multiple DmaControllers
  // concurrently.
  static DmaController* Init(dma_irq_index_t irq_index);

  enum class TransferWidth { k8 = 0, k16 = 1, k32 = 2 };
  struct Action {
    gpio_pin_t toggle_gpio = gpio_pin_t(-1);
    SemaphoreHandle_t give_semaphore = nullptr;

    TaskHandle_t notify_task = nullptr;
    UBaseType_t notify_index = 0;
    uint32_t notify_value = 0;
    eNotifyAction notify_action = eNoAction;
  };

  // Either tx_enable or rx_enable or both must be true. Default value of
  // tx_dreq/rx_dreq is 0xffffffff, which indicates that no dreq should be used.
  struct Request {
    bool tx_enable = false;
    uint32_t tx_dreq = 0x3f;  // unpaced transfer
    volatile const void* tx_read = nullptr;
    bool tx_read_incr = true;
    volatile void* tx_write = nullptr;
    bool tx_write_incr = true;

    bool rx_enable = false;
    uint32_t rx_dreq = 0x3f;  // unpaced transfer
    volatile const void* rx_read = nullptr;
    bool rx_read_incr = true;
    volatile void* rx_write = nullptr;
    bool rx_write_incr = true;

    TransferWidth transfer_width = TransferWidth::k8;
    size_t transfer_count = 0;

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
  explicit DmaController();
  ~DmaController() = delete;
  DmaController(const DmaController&) = delete;
  DmaController& operator=(const DmaController&) = delete;

  template <int irq_index>
  static void DmaFinishedISR();

 private:
  const SemaphoreHandle_t tail_mutex_;
  const SemaphoreHandle_t free_slots_sem_;
  struct ChannelPair {
    dma_channel_t tx;
    dma_channel_t rx;

    // Whether the tx channel should run.
    bool tx_enable = 0;
    // Whether the rx channel should run.
    bool rx_enable = 0;

    // True if tx dma finished or was not enabled.
    bool tx_done = 0;
    // True if rx dma finished or was not enabled.
    bool rx_done = 0;

    // What to do after all enabled channels finish.
    Action action;

    // If true, request is filled out and channels are configured.
    bool launch_ready = 0;
  };
  ChannelPair ring_[kNumChannelPairs];
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