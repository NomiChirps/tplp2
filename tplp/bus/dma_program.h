#ifndef TPLP_BUS_DMA_PROGRAM_H_
#define TPLP_BUS_DMA_PROGRAM_H_

#include <optional>
#include <variant>
#include <vector>

// XXX: use forward decls and remove dependency?
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"

namespace tplp {

// not sure there's ever a need for more than 2! but why the hell not
constexpr int kMaxSimultaneousTransfers = 2;

struct Action {
  int toggle_gpio = -1;

  SemaphoreHandle_t give_semaphore = nullptr;

  TaskHandle_t notify_task = nullptr;
  UBaseType_t notify_index = 0;
  uint32_t notify_value = 0;
  eNotifyAction notify_action = eNoAction;
};

struct ChannelCtrl {
  // enable must be specified at compile time
  bool high_priority;
  uint8_t data_size;
  bool incr_read;
  bool incr_write;
  uint8_t ring_size;
  bool ring_sel;
  // chain_to is used internally
  uint8_t treq_sel;
  // irq_quiet is used internally
  bool bswap;
  bool sniff_en;
};

// Configuration for one of the channels in a simultaneous transfer. All fields
// with non-nullopt values are fixed. Any omitted fields must be supplied at
// runtime. At least one field must be omitted.
struct ChannelConfig {
  std::optional<ChannelCtrl> ctrl;
  std::optional<volatile const void*> read_addr;
  std::optional<volatile void*> write_addr;
  std::optional<uint32_t> trans_count;
};

// A MultiTransfer can be chained with another iff:
//   - the first has no after Action
//   - the second one has no before Action
//   - they have the same set of enabled channels
//   - each enabled channel has the same set of config holes as its counterpart
// Caution! Because channels may proceed at different rates, you cannot rely on
// all of the transfers in one MultiTransfer completing before some of the
// transfers in the next MultiTransfer begin. If you require this sequencing
// guarantee, insert an Action between them.
struct MultiTransfer {
  bool enable[kMaxSimultaneousTransfers];
  ChannelConfig transfers[kMaxSimultaneousTransfers];
  std::optional<Action> before;
  std::optional<Action> after;
};

// user-facing stuff above
// compiler stuff below

// The control block aliases provide many possible combinations of config
// updates. Actually, it's all possible combinations! Wow.
struct ControlAlias0 {
  volatile const void* read_addr;
  volatile void* write_addr;
  uint32_t trans_count;
  uint32_t ctrl_trig;
};
struct ControlAlias1 {
  uint32_t ctrl;
  volatile const void* read_addr;
  volatile void* write_addr;
  uint32_t trans_count_trig;
};
struct ControlAlias2 {
  uint32_t ctrl;
  uint32_t trans_count;
  volatile const void* read_addr;
  volatile void* write_addr_trig;
};
struct ControlAlias3 {
  uint32_t ctrl;
  volatile void* write_addr;
  uint32_t trans_count;
  volatile const void* read_addr_trig;
};
union ControlAliasAny {
  ControlAlias0 al0;
  ControlAlias1 al1;
  ControlAlias2 al2;
  ControlAlias3 al3;
};

// This structure will be used in the interrupt handler to launch a queued
// chain. It should be efficient for that purpose. All of the hole data
// should be filled in already.
struct CompiledChain0 {
  bool enabled[kMaxSimultaneousTransfers];
  // Packed sequence of null-trigger-terminated chains of control blocks.
  // Each sequence may have different widths. `programmer_config` will
  // contain pointers into this structure.
  uint32_t* const ctrl_buf;
  ControlAlias0 programmer_config[kMaxSimultaneousTransfers];

  std::optional<Action> before;
  std::optional<Action> after;
};

// Used in task code to fill in the holes with fresh data before either
// launching it or adding it to the queue.
// XXX: use an actual FreeRTOS queue-- it is in fact implemented as a ring
// buffer with counting semaphore, and doesn't need to be reinvented. you
// buffoon. <3
struct CompiledChain1 {
  CompiledChain0 cc;
  // Pointers into cc.ctrl_buf.
  const std::vector<uint32_t*> holes;
};

void test() {
  uint32_t a;
  // the interrupt handler can use this to see which channel(s) are triggered,
  // then index into a global array that says what to  do in each case...
  int n = __builtin_clz(a);
}

class DmaProgram {
 public:
  // XXX: what's the nicest way to pass in hole data?
  void Execute(...);
 private:
  const std::vector<CompiledChain1> chains_;
};

class DmaProgramCompiler {
 public:
  // Builds up a list of CompiledChain1, pushing a new one when necessary.
  void Add(const MultiTransfer& cmd);

 private:
};

// a DmaProgram can be compiled into a sequence of DMA control blocks
// which would use a mix of interrupt handlers and dma chaining, depending on
// what actions are specified and which transfers are enabled.

// note! chained DmaCommands with dreqs will not necessarily have a sync barrier
// between them anymore.
//
// we have kMaxSimultaneousTransfers channels as execution channels,
// and the same number of channels reserved as programmers.
//
// programmers are configured to read from a sequence of control blocks and
// write to the corresponding execution channel's config registers, one control
// block at a time. if the control block specifies chain_to = programmer, then
// chaining continues. the last control block in a sequence must end the chain,
// raising an irq_quiet interrupt on that channel.
//
// each program needs a number of "holes" for the user to fill in parameters
// that change at runtime. in compiled form, this can be an array of offsets
// into the control block sequence!

}  // namespace tplp

#endif  // TPLP_BUS_DMA_PROGRAM_H_