#ifndef TPLP_BUS_DMA_H_
#define TPLP_BUS_DMA_H_

#include "pico/platform.h"
#include "picolog/status.h"
#include "tplp/bus/dma_program.h"
#include "tplp/bus/types.h"

namespace tplp {

// just a sketch...
class DmaController {
 public:
  static DmaController* Init(dma_irq_index_t irq_index);

  DmaProgram NewProgram();

  // Enqueues all chains in the program for execution, waiting indefinitely for
  // space on the queue if necessary. Caller retains ownership of `program`,
  // which must remain valid until completely executed.
  void Enqueue(const DmaProgram* program);

 private:
  // Arbitrary limit
  static constexpr int kQueueLength = 4;

  explicit DmaController(
      const std::array<uint8_t, kMaxSimultaneousTransfers>& programmer_channels,
      const std::array<uint8_t, kMaxSimultaneousTransfers>& execution_channels,
      QueueHandle_t queue, SemaphoreHandle_t active_chain_mutex);

  template <int irq_index>
  friend void __not_in_flash_func(DmaController_ProgramFinishedISR)();

  void LaunchImmediate(CompiledChain0* cc);
  void LaunchImmediateFromISR(CompiledChain0* cc, BaseType_t* higher_priority_task_woken);

 private:
  const std::array<uint8_t, kMaxSimultaneousTransfers> programmer_channels_;
  const std::array<uint8_t, kMaxSimultaneousTransfers> execution_channels_;
  const QueueHandle_t queue_;
  // Coordinates multiple tasks calling Enqueue() so that only one of them
  // starts the DMAs at the head of the queue, when none are active.
  const SemaphoreHandle_t active_chain_mutex_;
  CompiledChain0* volatile active_chain_;

  // Number of channels in the active chain that we're waiting on to complete.
  volatile int active_chain_pending_channels_;
};

}  // namespace tplp

#endif  // TPLP_BUS_DMA_H_