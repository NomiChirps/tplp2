#include "tplp/bus/dma_program.h"

#include <cstring>

#include "picolog/picolog.h"

namespace tplp {
namespace {
CompiledChain0::AliasInfo SelectAlias(const ChannelConfig& cfg) {
  // mask() gives a bitmask of the fields that are *present*
  // negate to get the omitted ones
  switch ((~cfg.mask()) & 0xf) {
    case 0x0:
      // all fields are provided. arbitrarily choose trans_count
      // to be the trigger
      return {1, 3};
    case 0x1:
      // trans_count
      return {1, 3};
    case 0x2:
      // write_addr
      return {2, 3};
    case 0x3:
      // write_addr, trans_count
      return {1, 2};
    case 0x4:
      // read_addr
      return {3, 3};
    case 0x5:
      // read_addr, trans_count
      return {3, 2};
    case 0x6:
      // read_addr, write_addr
      return {2, 2};
    case 0x7:
      // read_addr, write_addr, trans_count
      return {1, 1};
    case 0x8:
      // ctrl
      return {0, 3};
    case 0x9:
      // ctrl, trans_count
      return {0, 2};
    case 0xa:
      // ctrl, write_addr
      // TODO: we actually could support these if we tried
      LOG(FATAL) << "unsupported combination of omitted fields in "
                    "ChannelConfig: ctrl, write_addr";
    case 0xb:
      // ctrl, write_addr, trans_count
      return {0, 1};
    case 0xc:
      // ctrl, read_addr
      LOG(FATAL) << "unsupported combination of omitted fields in "
                    "ChannelConfig: ctrl, read_addr";
    case 0xd:
      // ctrl, read_addr, trans_count
      LOG(FATAL) << "unsupported combination of omitted fields in "
                    "ChannelConfig: ctrl, read_addr, trans_count";
    case 0xe:
      // ctrl, read_addr, write_addr
      LOG(FATAL) << "unsupported combination of omitted fields in "
                    "ChannelConfig: ctrl, read_addr, write_addr";
    case 0xf:
      // all fields are omitted; we can use any alias.
      return {0, 0};
  }
  LOG(FATAL) << "invalid ChannelConfig::mask() value: " << cfg.mask();
}

void PackInitialConfig(const ChannelConfig& cfg,
                       CompiledChain0::AliasInfo alias, uint8_t chain_to,
                       ControlAliasAny* out) {
  uint32_t ctrl = cfg.ctrl ? cfg.ctrl->Pack(true, chain_to, true) : 0;
  switch (alias.num) {
    case 0:
      if (cfg.ctrl) out->al0.ctrl_trig = ctrl;
      if (cfg.read_addr)
        out->al0.read_addr = reinterpret_cast<intptr_t>(*cfg.read_addr);
      if (cfg.write_addr)
        out->al0.write_addr = reinterpret_cast<intptr_t>(*cfg.write_addr);
      if (cfg.trans_count) out->al0.trans_count = *cfg.trans_count;
      break;
    case 1:
      if (cfg.ctrl) out->al1.ctrl = ctrl;
      if (cfg.read_addr)
        out->al1.read_addr = reinterpret_cast<intptr_t>(*cfg.read_addr);
      if (cfg.write_addr)
        out->al1.write_addr = reinterpret_cast<intptr_t>(*cfg.write_addr);
      if (cfg.trans_count) out->al1.trans_count_trig = *cfg.trans_count;
      break;
    case 2:
      if (cfg.ctrl) out->al2.ctrl = ctrl;
      if (cfg.read_addr)
        out->al2.read_addr = reinterpret_cast<intptr_t>(*cfg.read_addr);
      if (cfg.write_addr)
        out->al2.write_addr_trig = reinterpret_cast<intptr_t>(*cfg.write_addr);
      if (cfg.trans_count) out->al2.trans_count = *cfg.trans_count;
      break;
    case 3:
      if (cfg.ctrl) out->al3.ctrl = ctrl;
      if (cfg.read_addr)
        out->al3.read_addr_trig = reinterpret_cast<intptr_t>(*cfg.read_addr);
      if (cfg.write_addr)
        out->al3.write_addr = reinterpret_cast<intptr_t>(*cfg.write_addr);
      if (cfg.trans_count) out->al3.trans_count = *cfg.trans_count;
      break;
    default:
      LOG(FATAL) << "internal error";
  }
}

}  // namespace

DmaProgram::DmaProgram(
    const std::array<uint8_t, kMaxSimultaneousTransfers>& programmer_channels,
    const std::array<uint8_t, kMaxSimultaneousTransfers>& execution_channels)
    : programmer_channels_(programmer_channels),
      execution_channels_(execution_channels) {}

void DmaProgram::SetArg(size_t command_index,
                        std::initializer_list<ChannelConfig> channel_configs) {
  auto channel_config = channel_configs.begin();
  CHECK_GE(command_index, 0u);
  CHECK_LT(command_index, chains_.size());
  CompiledChain1& chain = chains_[command_index];
  int num_enabled = 0;
  for (int t = 0; t < kMaxSimultaneousTransfers; ++t) {
    if (!chain.cc0.enable[t]) continue;
    ++num_enabled;
    CHECK_NE(channel_config, channel_configs.end())
        << "Argument list too short: expected at least " << num_enabled;
    for (const CompiledChain1::HoleInfo& hole : chain.holes) {
      switch (hole.type) {
        case ChannelConfig::Field::kCtrl:
          CHECK(channel_config->ctrl) << "ctrl must not be nullopt";
          chain.cc0.program[hole.program_index] =
              channel_config->ctrl->Pack(true, programmer_channels_[t], true);
          break;
        case ChannelConfig::Field::kReadAddr:
          CHECK(channel_config->read_addr) << "read_addr must not be nullopt";
          chain.cc0.program[hole.program_index] = static_cast<uint32_t>(
              reinterpret_cast<intptr_t>(*channel_config->read_addr));
          break;
        case ChannelConfig::Field::kWriteAddr:
          CHECK(channel_config->write_addr) << "write_addr must not be nullopt";
          chain.cc0.program[hole.program_index] = static_cast<uint32_t>(
              reinterpret_cast<intptr_t>(*channel_config->write_addr));
          break;
        case ChannelConfig::Field::kTransCount:
          CHECK(channel_config->trans_count)
              << "trans_count must not be nullopt";
          chain.cc0.program[hole.program_index] = *channel_config->trans_count;
          break;
        case ChannelConfig::Field::kInvalid:
          LOG(FATAL) << "internal error";
      }
    }
    ++channel_config;
  }
  CHECK_EQ(channel_config, channel_configs.end())
      << "Argument list too long: expected exactly " << num_enabled;
}

void DmaProgram::AddCommand(const DmaCommand& cmd) {
  CompiledChain1& cc1 = chains_.emplace_back();
  CompiledChain0& cc0 = cc1.cc0;
  cc0.before = cmd.before;
  cc0.after = cmd.after;
  std::vector<uint32_t> tmp_program;
  for (int t = 0; t < kMaxSimultaneousTransfers; ++t) {
    cc0.enable[t] = cmd.enable[t];
    // FIXME ? sure this is right for a disabled channel?
    if (!cmd.enable[t]) continue;

    const int num_params = cmd.transfers[t].num_params();
    CHECK_GE(num_params, 0);
    CHECK_LE(num_params, 4);
    // a.k.a. programmer's transfer count
    uint8_t control_block_length;
    uint8_t ring_size;
    // We can only write in power-of-2 chunks due to ring_size, so promote
    // 3 params to 4. The fixed 4th (0th) value was added to the program above.
    // Also if num_params is zero, we still need to write at least one word
    // to trigger the channel.
    switch (num_params) {
      case 0:
      case 1:
        control_block_length = 1;
        // 4 bytes of parameters
        ring_size = 2;
        break;
      case 2:
        control_block_length = 2;
        // 8 bytes of parameters
        ring_size = 3;
      case 3:
      case 4:
        control_block_length = 4;
        // 12 or 16 bytes of parameters
        ring_size = 4;
    }

    cc0.alias[t] = SelectAlias(cmd.transfers[t]);
    ControlAliasAny initial_config;
    // Execution always chains to the programmer channel.
    PackInitialConfig(cmd.transfers[t], cc0.alias[t], programmer_channels_[t],
                      &initial_config);
    // (4 - control_block_length) determines the boundary between the initial
    // config and the runtime config; write of the initial config always starts
    // at the 0th field of the alias.
    cc0.initial_config_write_addr[t] = reinterpret_cast<uint32_t*>(
        0x50000000 + 0x40 * execution_channels_[t] + 0x10 * cc0.alias[t].num);
    cc0.initial_config_write_length[t] = 4 - control_block_length;
    std::memcpy(cc0.initial_config[t], &initial_config, 3 * sizeof(uint32_t));

    for (int index_in_chain = 0; index_in_chain < cmd.chain_length;
         ++index_in_chain) {
      for (int param = 0; param < num_params; ++param) {
        cc1.holes.push_back({
            .type = ControlAliasAny::field_type(
                cc1.cc0.alias[t].num, cc1.cc0.alias[t].offset + param),
            // TODO: these could be pointers too, to save
            //       a tiny bit of arithmetic at runtime
            .program_index = tmp_program.size(),
        });
        // placeholder value to be filled in at runtime
        tmp_program.push_back(0xbaadf00d);
      }
      if (num_params == 0) {
        // We need at least one non-null-trigger to run the program.
        // Use the one arbitrarily chosen in SelectAlias, which will
        // have been placed into the last slot of initial_config
        // by PackInitialConfig.
        CHECK_EQ(cc0.alias[t].offset, 3);
        tmp_program.push_back(initial_config.field_value(cc0.alias[t].num, 3));
      }
      if (num_params == 3) {
        // We need to write an extra, fixed parameter here, because ring_size
        // for the programmer's writes can only be a power of 2. note that in
        // this case the fixed parameter has index 0, so each cycle of the
        // programmer writes: 1, 2, 3(trigger), [ring loops around], 0.
        tmp_program.push_back(initial_config.field_value(cc0.alias[t].num, 0));
      }
    }
    // Add a dummy control block with a null trigger to terminate the chain.
    for (int i = 0; i < control_block_length; ++i) {
      tmp_program.push_back(0);
    }

    // Programmer reads from the program, writes to the appropriate alias of the
    // executor channel's control registers, and chains to the executor.
    // See RP2040 datasheet section 2.5.7 for these offsets.
    cc0.programmer_config[t].read_addr = 0; // filled in afterwards
    cc0.programmer_config[t].write_addr =
        0x50000000 + 0x40 * execution_channels_[t] + 0x10 * cc0.alias[t].num +
        0x4 * cc0.alias[t].offset;
    cc0.programmer_config[t].trans_count = control_block_length;
    ChannelCtrl pctrl{
        .high_priority = cmd.transfers[t].ctrl->high_priority,
        .data_size = ChannelCtrl::DataSize::k32,
        .incr_read = true,
        .incr_write = true,
        .ring_size = ring_size,
        .ring_sel = 1,     // wrap writes
        .treq_sel = 0x3f,  // unpaced
        .bswap = false,
        .sniff_en = false,
    };
    cc0.programmer_config[t].ctrl_trig =
        pctrl.Pack(true, execution_channels_[t], true);
  }
  // Finalize the program so we can make pointers into it.
  cc0.program = std::make_unique<uint32_t[]>(tmp_program.size());
  std::copy(tmp_program.begin(), tmp_program.end(), cc0.program.get());
  for (int t = 0; t < kMaxSimultaneousTransfers; ++t) {
    cc0.programmer_config[t].read_addr =
        reinterpret_cast<intptr_t>(&cc0.program[0]);
  }
}

}  // namespace tplp
