#ifndef TPLP_BUS_DMACONTROLLER_H_
#define TPLP_BUS_DMACONTROLLER_H_

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"
#include "tplp/bus/types.h"

namespace tplp {

class DmaController {
 public:
  static constexpr int kNumChannelPairs = 2;

 public:
  static DmaController* Init(dma_irq_index_t irq_index);

  void Transfer(const void* tx, void* rx, size_t transfer_count);

 private:
  explicit DmaController();
  ~DmaController() = delete;
  DmaController(const DmaController&) = delete;
  DmaController& operator=(const DmaController&) = delete;

  void TaskFn();

 private:
  const TaskHandle_t task_;

  struct Request {
    //
  };
  Request next_request_;
  SemaphoreHandle_t next_request_mutex_;
  SemaphoreHandle_t ready_for_next_request_sem_;

  struct ChannelPair {
    dma_channel_t tx;
    dma_channel_t rx;
  };
  ChannelPair ring_[kNumChannelPairs];
  // Currently active channel pair.
  int head_;
  // Next free channel pair.
  int tail_;
};

}  // namespace tplp

#endif  // TPLP_BUS_DMACONTROLLER_H_