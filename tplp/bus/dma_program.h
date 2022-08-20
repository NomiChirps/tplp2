#ifndef TPLP_BUS_DMA_PROGRAM_H_
#define TPLP_BUS_DMA_PROGRAM_H_

#include <array>
#include <memory>
#include <optional>
#include <variant>
#include <vector>

// XXX: use forward decls and remove dependency?
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"
#include "picolog/picolog.h"

namespace tplp {

// not sure there's ever a need for more than 2! but why the hell not
constexpr int kMaxSimultaneousTransfers = 2;

// XXX: rename or move, this is too generic
struct Action {
  int gpio_toggle = -1;

  SemaphoreHandle_t give_semaphore = nullptr;

  TaskHandle_t notify_task = nullptr;
  UBaseType_t notify_index = 0;
  uint32_t notify_value = 0;
  eNotifyAction notify_action = eNoAction;
};

struct ChannelCtrl {
  enum DataSize { k8 = 0, k16 = 1, k32 = 2 };
  bool high_priority = 0;
  DataSize data_size = k8;
  bool incr_read = 1;
  bool incr_write = 1;
  // Size of address wrap region. If 0, don’t wrap. For values n > 0, only the
  // lower n bits of the address will change. This wraps the address on a
  // (1 << n) byte boundary, facilitating access to naturally-aligned ring
  // buffers. Valid values are 0 to 15.
  uint8_t ring_size = 0;
  // Select whether RING_SIZE applies to read or write addresses. If 0, read
  // addresses are wrapped on a (1 << RING_SIZE) boundary. If 1, write addresses
  // are wrapped.
  bool ring_sel = 0;
  // Set to 0x3f to disable.
  uint8_t treq_sel = 0x3f;
  bool bswap = false;
  bool sniff_en = false;

  static constexpr uint32_t kStaticBitsMask = 0xff207801;
  // Packs fields into the register format, omitting enable, chain_to, and
  // irq_quiet (which are selected by kStaticBitsMask).
  uint32_t Pack(bool enable, uint8_t chain_to, bool irq_quiet) const {
    return enable | (high_priority << 1) | ((data_size & 0x3) << 2) |
           (incr_read << 4) | (incr_write << 5) | ((ring_size & 0xf) << 6) |
           (ring_sel << 10) | ((chain_to & 0xf) << 11) |
           ((treq_sel & 0x3f) << 15) | (irq_quiet << 21) | (bswap << 22) |
           (sniff_en << 23);
  }

  // enable must be specified at compile time.
  // chain_to is used internally.
  // irq_quiet is used internally.
  // kStaticBitsMask selects those fields in the packed representation
  // (plus the read-only and reserved bits of the register).
};

// Configuration for one of the channels in a simultaneous transfer. All fields
// with non-nullopt values are fixed. Any omitted fields must be supplied at
// runtime.
//
// Forbidden combinations of omitted fields (due to hardware limitations):
//   - ctrl, write_addr
//   - ctrl, read_addr
//   - ctrl, read_addr, trans_count
//   - ctrl, read_addr, write_addr
struct ChannelConfig {
  std::optional<ChannelCtrl> ctrl;
  std::optional<volatile const void*> read_addr;
  std::optional<volatile void*> write_addr;
  // Caution! trans_count MUST NOT be zero.
  std::optional<uint32_t> trans_count;

  enum class Field { kInvalid, kCtrl, kReadAddr, kWriteAddr, kTransCount };

  // Returns a 4-bit mask with a 1 for each field that is present.
  // ctrl is MSB, trans_count is LSB.
  uint8_t mask() const {
    return trans_count.has_value() | (write_addr.has_value() << 1) |
           (read_addr.has_value() << 2) | (ctrl.has_value() << 3);
  }
  // Returns the number of fields that are nullopt.
  uint8_t num_params() const {
    return !ctrl.has_value() + !read_addr.has_value() +
           !write_addr.has_value() + !trans_count.has_value();
  }
  // Returns the number of fields that are not nullopt.
  uint8_t num_fields_set() const { return 4 - num_params(); }
};

// Caution! Because channels may proceed at different rates, you cannot rely on
// them being in lockstep throughout a chain of transfers. If you require such a
// sequencing guarantee, use two DmaCommands.
struct DmaCommand {
  bool enable[kMaxSimultaneousTransfers] = {0};
  ChannelConfig transfers[kMaxSimultaneousTransfers] = {};
  std::optional<Action> before = std::nullopt;
  std::optional<Action> after = std::nullopt;
  // TODO: document
  int chain_length = 0;
};

// user-facing stuff above
// compiler stuff below

// The control block aliases provide many possible combinations of config
// updates.
struct ControlAlias0 {
  uint32_t read_addr;
  uint32_t write_addr;
  uint32_t trans_count;
  uint32_t ctrl_trig;

  static ChannelConfig::Field field_type(int index) {
    switch (index) {
      case 0:
        return ChannelConfig::Field::kReadAddr;
      case 1:
        return ChannelConfig::Field::kWriteAddr;
      case 2:
        return ChannelConfig::Field::kTransCount;
      case 3:
        return ChannelConfig::Field::kCtrl;
    }
    return ChannelConfig::Field::kInvalid;
  }
};
struct ControlAlias1 {
  uint32_t ctrl;
  uint32_t read_addr;
  uint32_t write_addr;
  uint32_t trans_count_trig;

  static ChannelConfig::Field field_type(int index) {
    switch (index) {
      case 0:
        return ChannelConfig::Field::kCtrl;
      case 1:
        return ChannelConfig::Field::kReadAddr;
      case 2:
        return ChannelConfig::Field::kWriteAddr;
      case 3:
        return ChannelConfig::Field::kTransCount;
    }
    return ChannelConfig::Field::kInvalid;
  }
};
struct ControlAlias2 {
  uint32_t ctrl;
  uint32_t trans_count;
  uint32_t read_addr;
  uint32_t write_addr_trig;

  static ChannelConfig::Field field_type(int index) {
    switch (index) {
      case 0:
        return ChannelConfig::Field::kCtrl;
      case 1:
        return ChannelConfig::Field::kTransCount;
      case 2:
        return ChannelConfig::Field::kReadAddr;
      case 3:
        return ChannelConfig::Field::kWriteAddr;
    }
    return ChannelConfig::Field::kInvalid;
  }
};
struct ControlAlias3 {
  uint32_t ctrl;
  uint32_t write_addr;
  uint32_t trans_count;
  uint32_t read_addr_trig;

  static ChannelConfig::Field field_type(int index) {
    switch (index) {
      case 0:
        return ChannelConfig::Field::kCtrl;
      case 1:
        return ChannelConfig::Field::kWriteAddr;
      case 2:
        return ChannelConfig::Field::kTransCount;
      case 3:
        return ChannelConfig::Field::kReadAddr;
    }
    return ChannelConfig::Field::kInvalid;
  }
};
union ControlAliasAny {
  ControlAlias0 al0;
  ControlAlias1 al1;
  ControlAlias2 al2;
  ControlAlias3 al3;

  static ChannelConfig::Field field_type(int alias, int index) {
    switch (alias) {
      case 0:
        return ControlAlias0::field_type(index);
      case 1:
        return ControlAlias1::field_type(index);
      case 2:
        return ControlAlias2::field_type(index);
      case 3:
        return ControlAlias3::field_type(index);
    }
    return ChannelConfig::Field::kInvalid;
  }

  inline uint32_t field_value(int alias, int index) {
    return *(reinterpret_cast<uint32_t*>(&al0) + index);
  }
};
static_assert(sizeof(ControlAliasAny) == 16);

// This structure will be used in the interrupt handler to launch a queued
// chain. It should be efficient for that purpose. All of the hole data
// should be filled in already.
struct CompiledChain0 {
  bool enable[kMaxSimultaneousTransfers];
  // Packed sequence of null-trigger-terminated chains of control blocks.
  // Each sequence may have different widths. `programmer_config` will
  // contain pointers into this structure. Do not modify after compilation,
  // as this will invalidate the pointers in programmer_config.
  std::unique_ptr<uint32_t[]> program;
  ControlAlias0 programmer_config[kMaxSimultaneousTransfers];

  uint32_t initial_config[kMaxSimultaneousTransfers][3];
  uint8_t
      initial_config_write_length[kMaxSimultaneousTransfers];  // number of
                                                               // words, 0 to 3
  uint32_t* initial_config_write_addr[kMaxSimultaneousTransfers];

  std::optional<Action> before;
  std::optional<Action> after;
  int chain_length;
};

// Used in task code to fill in the holes with fresh data before either
// launching it or adding it to the queue.
struct CompiledChain1 {
  CompiledChain0 cc0;

  struct AliasInfo {
    // Which ControlAlias* structure? Valid values 0, 1, 2, or 3.
    int_fast8_t num;
    // Offset into the ControlAlias* structure where the dynamic parameters
    // begin, i.e. those that are supplied at runtime and may be different for
    // each transfer in the chain. Valid values 0, 1, 2, or 3.
    int_fast8_t offset;
  };
  AliasInfo alias[kMaxSimultaneousTransfers];

  struct HoleInfo {
    ChannelConfig::Field type;
    // Indices into cc.program
    size_t program_index;
  };
  // stored in (channel,chain,field) order
  std::vector<HoleInfo> holes_channel_major;
  // stored in (chain,channel,field) order
  std::vector<HoleInfo> holes_chain_major;
  uint8_t num_params[kMaxSimultaneousTransfers];
};

class DmaProgram {
 public:
  explicit DmaProgram(
      const std::array<uint8_t, kMaxSimultaneousTransfers>& programmer_channels,
      const std::array<uint8_t, kMaxSimultaneousTransfers>& execution_channels);

  void AddCommand(const DmaCommand& cmd);

  // The input list of ChannelConfig(s) must include ONLY the enabled channels.
  // All ChannelConfig fields that were omitted in AddCommand() MUST be provided
  // now. The length of channel_configs must be equal to the number of enabled
  // channels multiplied by the chain length. Configs must be provided in
  // channel-major order, i.e. for each command, for each enabled channel,
  // provide each chain's config for that channel. For example:
  //   cmd0-channel0-chain0
  //   cmd0-channel0-chain1
  //   cmd0-channel1-chain0
  //   cmd0-channel1-chain1
  //   cmd1-channel0-chain0
  //   cmd1-channel1-chain0
  //   ...
  // CHECK-fails if these requirements are not met.
  template <class InputIterator>
  void SetArgsChannelMajor(InputIterator channel_configs_begin,
                           InputIterator channel_configs_end);

  void SetArgsChannelMajor(std::initializer_list<ChannelConfig> args) {
    SetArgsChannelMajor(args.begin(), args.end());
  }

  // As SetArgsChannelMajor, but args are provided in chain-major order,
  // i.e. for each command, for each chain, provide each enabled channel's
  // config in order. For example:
  //   cmd0-chain0-channel0
  //   cmd0-chain0-channel1
  //   cmd0-chain1-channel0
  //   cmd0-chain1-channel1
  //   cmd1-chain0-channel0
  //   cmd1-chain0-channel1
  //   ...
  // CHECK-fails if these requirements are not met.
  template <class InputIterator>
  void SetArgsChainMajor(InputIterator channel_configs_begin,
                         InputIterator channel_configs_end);
  void SetArgsChainMajor(std::initializer_list<ChannelConfig> args) {
    SetArgsChainMajor(args.begin(), args.end());
  }

  size_t num_commands() const { return chains_.size(); }
  // Total number of simultaneous transfers, that is,
  // the sum of chain_length across all commands.
  size_t total_length() const;

  // num_commands must be > 0
  std::optional<Action>& mutable_last_action() {
    return chains_.back().cc0.after;
  }

  const std::vector<CompiledChain1>& contents() const { return chains_; }

 private:
  const std::array<uint8_t, kMaxSimultaneousTransfers> programmer_channels_;
  const std::array<uint8_t, kMaxSimultaneousTransfers> execution_channels_;

  // Each DmaCommand corresponds to exactly one compiled chain.
  std::vector<CompiledChain1> chains_;
};

template <class InputIterator>
void DmaProgram::SetArgsChannelMajor(InputIterator channel_configs_begin,
                                     InputIterator channel_configs_end) {
  auto channel_config = channel_configs_begin;
  int channel_config_index = 0;
  int total_num_enabled_channels = 0;
  for (size_t command_index = 0; command_index < chains_.size();
       ++command_index) {
    for (int t = 0; t < kMaxSimultaneousTransfers; ++t) {
      if (chains_[command_index].cc0.enable[t]) total_num_enabled_channels++;
    }
  }
  for (size_t command_index = 0; command_index < chains_.size();
       ++command_index) {
    CompiledChain1& chain = chains_[command_index];
    auto hole = chain.holes_channel_major.begin();
    int hole_index = 0;
    for (int t = 0; t < kMaxSimultaneousTransfers; ++t) {
      if (!chain.cc0.enable[t]) continue;
      for (int index_in_chain = 0; index_in_chain < chain.cc0.chain_length;
           ++index_in_chain) {
        CHECK(channel_config != channel_configs_end)
            << "Argument list too short: expected exactly "
            << (total_num_enabled_channels * total_length());
        CHECK_EQ(chain.num_params[t], channel_config->num_fields_set())
            << "incorrect number of non-nullopt fields in channel_configs["
            << channel_config_index << "] (configuring channel " << t << ")";
        for (int n = 0; n < chain.num_params[t]; ++n) {
          CHECK(hole != chain.holes_channel_major.end())
              << "too many arguments; ran out of config holes";
          switch (hole->type) {
            case ChannelConfig::Field::kCtrl:
              CHECK(channel_config->ctrl)
                  << "channel_config[" << channel_config_index
                  << "].ctrl must be set";
              chain.cc0.program[hole->program_index] =
                  channel_config->ctrl->Pack(true, programmer_channels_[t],
                                             true);
              break;
            case ChannelConfig::Field::kReadAddr:
              CHECK(channel_config->read_addr)
                  << "channel_config[" << channel_config_index
                  << "].read_addr must be set";
              chain.cc0.program[hole->program_index] = static_cast<uint32_t>(
                  reinterpret_cast<intptr_t>(*channel_config->read_addr));
              break;
            case ChannelConfig::Field::kWriteAddr:
              CHECK(channel_config->write_addr)
                  << "channel_config[" << channel_config_index
                  << "].write_addr must be set";
              chain.cc0.program[hole->program_index] = static_cast<uint32_t>(
                  reinterpret_cast<intptr_t>(*channel_config->write_addr));
              break;
            case ChannelConfig::Field::kTransCount:
              CHECK(channel_config->trans_count)
                  << "channel_config[" << channel_config_index
                  << "].trans_count must be set";
              chain.cc0.program[hole->program_index] =
                  *channel_config->trans_count;
              break;
            case ChannelConfig::Field::kInvalid:
              LOG(FATAL) << "internal error";
          }
          VLOG(1) << "filled hole " << hole_index << " from channel_config["
                  << channel_config_index << "]";
          ++hole;
          ++hole_index;
        }
        ++channel_config;
        ++channel_config_index;
      }
    }
  }
  CHECK(channel_config == channel_configs_end)
      << "Argument list too long: expected exactly "
      << (total_num_enabled_channels * total_length());
}

template <class InputIterator>
void DmaProgram::SetArgsChainMajor(InputIterator channel_configs_begin,
                                   InputIterator channel_configs_end) {
  auto channel_config = channel_configs_begin;
  int channel_config_index = 0;
  int total_num_enabled_channels = 0;
  for (size_t command_index = 0; command_index < chains_.size();
       ++command_index) {
    for (int t = 0; t < kMaxSimultaneousTransfers; ++t) {
      if (chains_[command_index].cc0.enable[t]) total_num_enabled_channels++;
    }
  }
  for (size_t command_index = 0; command_index < chains_.size();
       ++command_index) {
    CompiledChain1& chain = chains_[command_index];
    auto hole = chain.holes_chain_major.begin();
    int hole_index = 0;
    for (int index_in_chain = 0; index_in_chain < chain.cc0.chain_length;
         ++index_in_chain) {
      for (int t = 0; t < kMaxSimultaneousTransfers; ++t) {
        if (!chain.cc0.enable[t]) continue;
        CHECK(channel_config != channel_configs_end)
            << "Argument list too short: expected exactly "
            << (total_num_enabled_channels * total_length());
        CHECK_EQ(chain.num_params[t], channel_config->num_fields_set())
            << "incorrect number of non-nullopt fields in channel_configs["
            << channel_config_index << "] (configuring channel " << t << ")";
        for (int n = 0; n < chain.num_params[t]; ++n) {
          CHECK(hole != chain.holes_chain_major.end())
              << "too many arguments; ran out of config holes";
          switch (hole->type) {
            case ChannelConfig::Field::kCtrl:
              CHECK(channel_config->ctrl)
                  << "channel_config[" << channel_config_index
                  << "].ctrl must be set";
              chain.cc0.program[hole->program_index] =
                  channel_config->ctrl->Pack(true, programmer_channels_[t],
                                             true);
              break;
            case ChannelConfig::Field::kReadAddr:
              CHECK(channel_config->read_addr)
                  << "channel_config[" << channel_config_index
                  << "].read_addr must be set";
              chain.cc0.program[hole->program_index] = static_cast<uint32_t>(
                  reinterpret_cast<intptr_t>(*channel_config->read_addr));
              break;
            case ChannelConfig::Field::kWriteAddr:
              CHECK(channel_config->write_addr)
                  << "channel_config[" << channel_config_index
                  << "].write_addr must be set";
              chain.cc0.program[hole->program_index] = static_cast<uint32_t>(
                  reinterpret_cast<intptr_t>(*channel_config->write_addr));
              break;
            case ChannelConfig::Field::kTransCount:
              CHECK(channel_config->trans_count)
                  << "channel_config[" << channel_config_index
                  << "].trans_count must be set";
              chain.cc0.program[hole->program_index] =
                  *channel_config->trans_count;
              break;
            case ChannelConfig::Field::kInvalid:
              LOG(FATAL) << "internal error";
          }
          VLOG(1) << "filled hole " << hole_index << " from channel_config["
                  << channel_config_index << "]";
          ++hole;
          ++hole_index;
        }
        ++channel_config;
        ++channel_config_index;
      }
    }
  }
  CHECK(channel_config == channel_configs_end)
      << "Argument list too long: expected exactly "
      << (total_num_enabled_channels * total_length());
}

}  // namespace tplp

#endif  // TPLP_BUS_DMA_PROGRAM_H_