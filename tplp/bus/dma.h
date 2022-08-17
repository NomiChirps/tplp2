#ifndef TPLP_BUS_DMA_H_
#define TPLP_BUS_DMA_H_

#include <vector>
#include "tplp/bus/types.h"
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"

// Just putting down some thoughts on version 2.0 ......



namespace tplp {

constexpr int kMaxSimultaneousTransfers = 2;

enum class TransferWidth { k8 = 0, k16 = 1, k32 = 2 };
struct Action {
  gpio_pin_t toggle_gpio = gpio_pin_t(-1);
  SemaphoreHandle_t give_semaphore = nullptr;

  TaskHandle_t notify_task = nullptr;
  UBaseType_t notify_index = 0;
  uint32_t notify_value = 0;
  eNotifyAction notify_action = eNoAction;
};
struct ChannelConfig {
  bool enable;
  volatile const void* read_buf;
  bool read_incr;
  volatile void* write_buf;
  bool write_incr;
  uint32_t dreq;
};
struct DmaCommand {
  ChannelConfig transfers[kMaxSimultaneousTransfers];

  int transfer_width;
  uint32_t transfer_count;
  DmaController::Action action;
};
struct DmaProgram {
    std::vector<DmaCommand> commands;
};

// a DmaProgram can be compiled into a sequence of DMA control blocks

}  // namespace tplp

#endif  // TPLP_BUS_DMA_H_