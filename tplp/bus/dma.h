#ifndef TPLP_BUS_DMACONTROLLER_H_
#define TPLP_BUS_DMACONTROLLER_H_

#include <optional>

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
    uint8_t c0_treq_sel = 0x3f;
    volatile const void* c0_read_addr = nullptr;
    bool c0_read_incr = true;
    volatile void* c0_write_addr = nullptr;
    bool c0_write_incr = true;
    DataSize c0_data_size = DataSize::k8;
    size_t c0_trans_count = 0;
    uint8_t c0_ring_size = 0;
    // 0 = read wrap, 1 = write wrap
    bool c0_ring_sel = false;

    bool c1_enable = false;
    // Default value of 0x3f means an unpaced transfer.
    uint8_t c1_treq_sel = 0x3f;
    volatile const void* c1_read_addr = nullptr;
    bool c1_read_incr = true;
    volatile void* c1_write_addr = nullptr;
    bool c1_write_incr = true;
    DataSize c1_data_size = DataSize::k8;
    size_t c1_trans_count = 0;
    uint8_t c1_ring_size = 0;
    bool c1_ring_sel = false;

    // Action to take from the interrupt handler when c0 is finished.
    std::optional<Action> c0_action;
    // Action to take from the interrupt handler when c1 is finished.
    std::optional<Action> c1_action;
    // Action to take from the interrupt handler when both channels are
    // finished.
    std::optional<Action> both_action;
  };

  class TransferHandle {
   public:
    // Returns true if this transfer has started or finished or aborted;
    // false if it's still queued.
    bool started() const { return id_ <= dma_->completed_transfers_count_; }
    // Returns true if this transfer has finished or aborted.
    bool finished() const { return id_ < dma_->completed_transfers_count_; }

    // TODO: allow aborting queued transfer
    // Attempts to abort the transfer if it has started. Aborting a queued but
    // not started transfer is not supported and will CHECK-fail. If the
    // transfer has already finished, does nothing. An aborted transfer may or
    // may not execute its `DmaController::Action`(s). In any case, finished()
    // will be true after calling this.
    //
    // Note that this temporarily disables DMA interrupts for the pair of
    // channels managed by this DmaController and may busy-wait for a short
    // time.
    //
    // There is no guarantee about whether or not any configured
    // DmaController::Action will be executed for an aborted transaction.
    //
    // Returns the remaining trans_count for each channel.
    std::array<uint32_t, 2> Abort();

    // Returns the current remaining trans_count value for each channel.
    // The transfer must already have started. This is necessarily imprecise,
    // because it queries the channels one after the other and not
    // simultaneously.
    std::array<uint32_t, 2> trans_count() const;

    // Returns an invalid instance of TransferHandle that will crash if any
    // methods are called on it. Use with caution.
    static TransferHandle Invalid() { return TransferHandle(nullptr, 0); }

   private:
    friend class DmaController;
    TransferHandle(DmaController* dma, uint32_t id) : dma_(dma), id_(id) {}

    DmaController* dma_;
    uint32_t id_;
  };

  // Queues up a DMA with the given configuration. Thread-safe.
  //
  // Returns immediately after enqueueing the transfer. `action` specifies an
  // optional set of actions to take when the transfer has been completed. The
  // actions run in an interrupt context, as soon as feasible after the DMA has
  // completed and before the next one is started.
  TransferHandle Transfer(const Request& req);

  // Returns the current length of the transfer request queue, approximately.
  int PeekQueueLength() const;

 private:
  explicit DmaController(dma_irq_index_t irq_index, dma_channel_t c0,
                         dma_channel_t c1);
  ~DmaController() = delete;
  DmaController(const DmaController&) = delete;
  DmaController& operator=(const DmaController&) = delete;

  template <int irq_index>
  static void DmaFinishedISR();

 private:
  friend class TransferHandle;

  const dma_irq_index_t irq_index_;
  const dma_channel_t c0_;
  const dma_channel_t c1_;
  struct TransferSlot {
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

    std::optional<Action> c0_action;
    std::optional<Action> c1_action;
    std::optional<Action> both_action;
  };
  QueueHandle_t queue_;

  // transfer_mutex_ protects:
  // - active_transfer_
  // - queued_transfers_count_
  // - completed_transfers_count_
  // (except the ISR doesn't participate)
  SemaphoreHandle_t transfer_mutex_;
  TransferSlot active_transfer_;
  volatile uint32_t queued_transfers_count_;
  volatile uint32_t completed_transfers_count_;
};

}  // namespace tplp

#endif  // TPLP_BUS_DMACONTROLLER_H_