#include "tplp/bus/dma_program.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "picolog/picolog.h"

using namespace tplp;

using testing::ElementsAre;
using testing::SizeIs;

namespace tplp {

std::ostream& operator<<(std::ostream& out, ChannelConfig::Field type) {
  switch (type) {
    case ChannelConfig::Field::kCtrl:
      return out << "kCtrl";
    case ChannelConfig::Field::kReadAddr:
      return out << "kReadAddr";
    case ChannelConfig::Field::kWriteAddr:
      return out << "kWriteAddr";
    case ChannelConfig::Field::kTransCount:
      return out << "kTransCount";
    default:
      return out << "<" << (int)type << ">";
  }
}

std::ostream& operator<<(std::ostream& out,
                         const CompiledChain1::HoleInfo& hole) {
  return out << "{ .type=" << hole.type
             << ", .program_index=" << hole.program_index << " }";
}

}  // namespace tplp

std::array<uint8_t, kMaxSimultaneousTransfers> kProgrammerChannels = {0x3, 0x4};
std::array<uint8_t, kMaxSimultaneousTransfers> kExecutorChannels = {0x5, 0x6};

TEST(ChannelCtrl, Pack1) {
  ChannelCtrl ctrl{
      .high_priority = 0,
      .data_size = ChannelCtrl::DataSize::k16,
      .incr_read = 1,
      .incr_write = 0,
      .ring_size = 4,
      .ring_sel = 0,
      .treq_sel = 0x1c,
      .bswap = 0,
      .sniff_en = 0,
  };
  EXPECT_EQ(ctrl.Pack(true, 0x5, true), 1 | (0x1 << 2) | (1 << 4) | (4 << 6) |
                                            (0x5 << 11) | (0x1c << 15) |
                                            (1 << 21));
}

TEST(ChannelCtrl, Pack2) {
  ChannelCtrl ctrl{
      .high_priority = 1,
      .data_size = ChannelCtrl::DataSize::k8,
      .incr_read = 0,
      .incr_write = 1,
      .ring_size = 15,
      .ring_sel = 1,
      .treq_sel = 0xff,  // out of range
      .bswap = 1,
      .sniff_en = 1,
  };
  EXPECT_EQ(ctrl.Pack(false, 0xf, false),
            (1 << 1) | (1 << 5) | (0xf << 6) | (1 << 10) | (0xf << 11) |
                (0x3f << 15) | (1 << 22) | (1 << 23));
}

TEST(ChannelConfig, methods) {
  ChannelConfig cfg{
      .ctrl = ChannelCtrl(),
      .read_addr = nullptr,
      .write_addr = nullptr,
      .trans_count = 0,
  };
  EXPECT_EQ(cfg.num_params(), 0);
  EXPECT_EQ(cfg.mask(), 0b1111);
  cfg.trans_count = std::nullopt;
  EXPECT_EQ(cfg.num_params(), 1);
  EXPECT_EQ(cfg.mask(), 0b1110);
  cfg.write_addr = std::nullopt;
  EXPECT_EQ(cfg.num_params(), 2);
  EXPECT_EQ(cfg.mask(), 0b1100);
  cfg.read_addr = std::nullopt;
  EXPECT_EQ(cfg.num_params(), 3);
  EXPECT_EQ(cfg.mask(), 0b1000);
  cfg.ctrl = std::nullopt;
  EXPECT_EQ(cfg.num_params(), 4);
  EXPECT_EQ(cfg.mask(), 0b0000);
}

TEST(ControlAliasAny, field_value) {
  ControlAliasAny x{
      .al1 =
          {
              .ctrl = 0xdeadbeef,
              .read_addr = 0xcafebabe,
              .write_addr = 0xdecafbad,
              .trans_count_trig = 0xbaadf00d,
          },
  };
  EXPECT_EQ(x.field_value(1, 0), 0xdeadbeef);
  EXPECT_EQ(x.field_value(1, 1), 0xcafebabe);
  EXPECT_EQ(x.field_value(1, 2), 0xdecafbad);
  EXPECT_EQ(x.field_value(1, 3), 0xbaadf00d);
}

TEST(DmaProgram, InvalidCombination) {
  DmaProgram p(kProgrammerChannels, kExecutorChannels);
  DmaCommand cmd;
  cmd = {
      .enable{1},
      .transfers{
          {
              .ctrl = std::nullopt,
              .read_addr = &cmd,
              .write_addr = std::nullopt,
              .trans_count = 42,
          },
      },
      .before = std::nullopt,
      .after = std::nullopt,
      .chain_length = 1,
  };
  ASSERT_EQ(cmd.transfers[0].mask(), 0b0101);
  ASSERT_DEATH(p.AddCommand(cmd), "unsupported combination of omitted fields");
}

TEST(DmaProgram, AllProvided) {
  DmaProgram p(kProgrammerChannels, kExecutorChannels);
  DmaCommand cmd;
  cmd = {
      .enable{1},
      .transfers{
          {
              .ctrl = ChannelCtrl{},
              .read_addr = reinterpret_cast<volatile const void*>(0xdeadbeef),
              .write_addr = reinterpret_cast<volatile void*>(0xcafebabe),
              .trans_count = 42,
          },
      },
      .before = std::nullopt,
      .after = std::nullopt,
      .chain_length = 1,
  };
  ASSERT_EQ(cmd.transfers[0].num_params(), 0);
  p.AddCommand(cmd);
  const std::vector<CompiledChain1>& chains = p.contents();
  EXPECT_THAT(chains, SizeIs(1));
  const CompiledChain1& cc1 = chains[0];
  const CompiledChain0& cc0 = cc1.cc0;
  // All fields were provided, but a trigger is still needed. SelectAlias() uses
  // trans_count (even though it's saved between triggers anyway).
  EXPECT_EQ(cc0.alias[0].num, 1);
  EXPECT_EQ(cc0.alias[0].offset, 3);
  EXPECT_THAT(cc1.holes, SizeIs(0));
  EXPECT_THAT(cc0.enable, ElementsAre(1, 0));
  EXPECT_EQ(cc0.program[0], 42);
  EXPECT_EQ(cc0.program[1], 0);
  EXPECT_EQ(cc0.initial_config[0][0],
            cmd.transfers[0].ctrl->Pack(true, kProgrammerChannels[0], true));
  EXPECT_EQ(cc0.initial_config[0][1], 0xdeadbeef);
  EXPECT_EQ(cc0.initial_config[0][2], 0xcafebabe);
  EXPECT_EQ(cc0.initial_config_write_length[0], 3);
  EXPECT_EQ(cc0.initial_config_write_addr[0],
            // channel kExecutorChannels[0], alias 1, offset 0
            reinterpret_cast<uint32_t*>(0x50000000 +
                                        kExecutorChannels[0] * 0x40 + 0x10));
  EXPECT_EQ(cc0.programmer_config[0].read_addr,
            static_cast<uint32_t>(reinterpret_cast<intptr_t>(&cc0.program[0])));
  EXPECT_EQ(cc0.programmer_config[0].write_addr,
            // channel kExecutorChannels[0], alias 1, offset 3
            0x50000000 + kExecutorChannels[0] * 0x40 + 0x1c);
  // 1 word, just the trigger
  EXPECT_EQ(cc0.programmer_config[0].trans_count, 1);
  uint32_t expected_programmer_ctrl(
      ChannelCtrl{
          .high_priority = 0,
          .data_size = ChannelCtrl::DataSize::k32,
          .incr_read = 1,
          .incr_write = 1,
          // 1-word = 4-byte ring boundary = lower 2 bits
          .ring_size = 2,
          // write address is wrapped
          .ring_sel = 1,
      }
          .Pack(true, kExecutorChannels[0], true));
  EXPECT_EQ(cc0.programmer_config[0].ctrl_trig, expected_programmer_ctrl);
  EXPECT_EQ(cc0.before, std::nullopt);
  EXPECT_EQ(cc0.after, std::nullopt);
}

TEST(DmaProgram, NoneProvided) {
  DmaProgram p(kProgrammerChannels, kExecutorChannels);
  DmaCommand cmd;
  cmd = {
      .enable{1},
      .transfers{
          {
              .ctrl = std::nullopt,
              .read_addr = std::nullopt,
              .write_addr = std::nullopt,
              .trans_count = std::nullopt,
          },
      },
      .before = std::nullopt,
      .after = std::nullopt,
      .chain_length = 1,
  };
  ASSERT_EQ(cmd.transfers[0].num_params(), 4);
  p.AddCommand(cmd);
  const std::vector<CompiledChain1>& chains = p.contents();
  EXPECT_THAT(chains, SizeIs(1));
  const CompiledChain1& cc1 = chains[0];
  const CompiledChain0& cc0 = cc1.cc0;
  // We could use any alias, so zero it is.
  EXPECT_EQ(cc0.alias[0].num, 0);
  EXPECT_EQ(cc0.alias[0].offset, 0);
  EXPECT_THAT(cc1.holes, SizeIs(4));
  // holes in alias order
  EXPECT_EQ(cc1.holes[0].type, ChannelConfig::Field::kReadAddr);
  EXPECT_EQ(cc1.holes[1].type, ChannelConfig::Field::kWriteAddr);
  EXPECT_EQ(cc1.holes[2].type, ChannelConfig::Field::kTransCount);
  EXPECT_EQ(cc1.holes[3].type, ChannelConfig::Field::kCtrl);
  EXPECT_EQ(cc1.holes[0].program_index, 0);
  EXPECT_EQ(cc1.holes[1].program_index, 1);
  EXPECT_EQ(cc1.holes[2].program_index, 2);
  EXPECT_EQ(cc1.holes[3].program_index, 3);
  EXPECT_THAT(cc0.enable, ElementsAre(1, 0));
  // placeholder value for args
  EXPECT_EQ(cc0.program[0], 0xbaadf00d);
  EXPECT_EQ(cc0.program[1], 0xbaadf00d);
  EXPECT_EQ(cc0.program[2], 0xbaadf00d);
  EXPECT_EQ(cc0.program[3], 0xbaadf00d);
  EXPECT_EQ(cc0.program[4], 0);
  EXPECT_EQ(cc0.program[5], 0);
  EXPECT_EQ(cc0.program[6], 0);
  EXPECT_EQ(cc0.program[7], 0);
  // Don't care about the contents of initial_config because
  // the length is zero.
  EXPECT_EQ(cc0.initial_config_write_length[0], 0);
  EXPECT_EQ(
      cc0.initial_config_write_addr[0],
      // channel kExecutorChannels[0], alias 0, offset 0
      reinterpret_cast<uint32_t*>(0x50000000 + kExecutorChannels[0] * 0x40));
  EXPECT_EQ(cc0.programmer_config[0].read_addr,
            static_cast<uint32_t>(reinterpret_cast<intptr_t>(&cc0.program[0])));
  EXPECT_EQ(cc0.programmer_config[0].write_addr,
            // channel kExecutorChannels[0], alias 0, offset 0
            0x50000000 + kExecutorChannels[0] * 0x40);
  // 1 word, just the trigger
  EXPECT_EQ(cc0.programmer_config[0].trans_count, 4);
  uint32_t expected_programmer_ctrl(
      ChannelCtrl{
          .high_priority = 0,
          .data_size = ChannelCtrl::DataSize::k32,
          .incr_read = 1,
          .incr_write = 1,
          // 4-word = 16-byte ring boundary = lower 4 bits
          .ring_size = 4,
          // write address is wrapped
          .ring_sel = 1,
      }
          .Pack(true, kExecutorChannels[0], true));
  EXPECT_EQ(cc0.programmer_config[0].ctrl_trig, expected_programmer_ctrl);
  EXPECT_EQ(cc0.before, std::nullopt);
  EXPECT_EQ(cc0.after, std::nullopt);

  // Make sure the program is filled out correctly.
  ChannelCtrl ctrl_arg{};
  p.SetArg(0, {{.ctrl = ChannelCtrl{},
                .read_addr = reinterpret_cast<volatile const void*>(0xcafebabe),
                .write_addr = reinterpret_cast<volatile void*>(0xfeedbeef),
                .trans_count = 42}});
  // As we're writing to alias 0, they should go in a different
  // order to the ChannelConfigArg field order.
  // read_addr
  EXPECT_EQ(cc0.program[0], 0xcafebabe);
  // write_addr
  EXPECT_EQ(cc0.program[1], 0xfeedbeef);
  // trans_count
  EXPECT_EQ(cc0.program[2], 42);
  // ctrl
  EXPECT_EQ(cc0.program[3], ctrl_arg.Pack(true, kProgrammerChannels[0], true));
}

// Three is tricky because of the non-power-of-2 alignment.
TEST(DmaProgram, ThreeProvided) {
  DmaProgram p(kProgrammerChannels, kExecutorChannels);
  DmaCommand cmd;
  cmd = {
      .enable{1},
      .transfers{
          {
              .ctrl =
                  ChannelCtrl{
                      .treq_sel = 0x0b,
                  },
              .read_addr = std::nullopt,
              .write_addr = reinterpret_cast<volatile void*>(0xbeefbabe),
              .trans_count = 42,
          },
      },
      .before = std::nullopt,
      .after = std::nullopt,
      .chain_length = 1,
  };
  ASSERT_EQ(cmd.transfers[0].num_params(), 1);
  p.AddCommand(cmd);
  const std::vector<CompiledChain1>& chains = p.contents();
  EXPECT_THAT(chains, SizeIs(1));
  const CompiledChain1& cc1 = chains[0];
  const CompiledChain0& cc0 = cc1.cc0;
  EXPECT_EQ(cc0.alias[0].num, 3);
  EXPECT_EQ(cc0.alias[0].offset, 3);
  EXPECT_THAT(cc1.holes, SizeIs(1));
  EXPECT_EQ(cc1.holes[0].type, ChannelConfig::Field::kReadAddr);
  EXPECT_EQ(cc1.holes[0].program_index, 0);
  EXPECT_THAT(cc0.enable, ElementsAre(1, 0));
  // placeholder value for args
  EXPECT_EQ(cc0.program[0], 0xbaadf00d);
  EXPECT_EQ(cc0.program[1], 0);
  EXPECT_EQ(cc0.initial_config_write_length[0], 3);
  EXPECT_EQ(cc0.initial_config_write_addr[0],
            // channel kExecutorChannels[0], alias 3, offset 0
            reinterpret_cast<uint32_t*>(0x50000000 +
                                        kExecutorChannels[0] * 0x40 + 0x30));
  EXPECT_EQ(cc0.initial_config[0][0],
            cmd.transfers[0].ctrl->Pack(true, kProgrammerChannels[0], true));
  EXPECT_EQ(cc0.initial_config[0][1], 0xbeefbabe);
  EXPECT_EQ(cc0.initial_config[0][2], 42);
  EXPECT_EQ(cc0.programmer_config[0].read_addr,
            static_cast<uint32_t>(reinterpret_cast<intptr_t>(&cc0.program[0])));
  EXPECT_EQ(cc0.programmer_config[0].write_addr,
            // channel kExecutorChannels[0], alias 3, offset 3
            0x50000000 + kExecutorChannels[0] * 0x40 + 0x3c);
  // 1 word, just the trigger
  EXPECT_EQ(cc0.programmer_config[0].trans_count, 1);
  uint32_t expected_programmer_ctrl(
      ChannelCtrl{
          .high_priority = 0,
          .data_size = ChannelCtrl::DataSize::k32,
          .incr_read = 1,
          .incr_write = 1,
          // 1-word = 4-byte ring boundary = lower 2 bits
          .ring_size = 2,
          // write address is wrapped
          .ring_sel = 1,
      }
          .Pack(true, kExecutorChannels[0], true));
  EXPECT_EQ(cc0.programmer_config[0].ctrl_trig, expected_programmer_ctrl);
  EXPECT_EQ(cc0.before, std::nullopt);
  EXPECT_EQ(cc0.after, std::nullopt);

  p.SetArg(0,
           {{.read_addr = reinterpret_cast<const volatile void*>(0x12345678)}});
  EXPECT_EQ(cc0.program[0], 0x12345678);
  EXPECT_EQ(cc0.program[1], 0);
}

TEST(DmaProgram, LongChainTwoChannels) {
  const int kChainLength = 3;
  DmaProgram p(kProgrammerChannels, kExecutorChannels);
  DmaCommand cmd;
  cmd = {
      .enable{1, 1},
      .transfers{{
                     .ctrl =
                         ChannelCtrl{
                             .treq_sel = 0x0c,
                         },
                     .read_addr = std::nullopt,
                     .write_addr = std::nullopt,
                     .trans_count = 42,
                 },
                 {
                     .ctrl =
                         ChannelCtrl{
                             .treq_sel = 0x1f,
                         },
                     .read_addr = reinterpret_cast<volatile void*>(0xcafebaba),
                     .write_addr = std::nullopt,
                     .trans_count = 56,
                 }},
      .before = std::nullopt,
      .after = std::nullopt,
      .chain_length = kChainLength,
  };
  ASSERT_EQ(cmd.transfers[0].num_params(), 2);
  ASSERT_EQ(cmd.transfers[1].num_params(), 1);
  p.AddCommand(cmd);
  const std::vector<CompiledChain1>& chains = p.contents();
  EXPECT_THAT(chains, SizeIs(1));
  const CompiledChain1& cc1 = chains[0];
  const CompiledChain0& cc0 = cc1.cc0;
  EXPECT_EQ(cc0.alias[0].num, 2);
  EXPECT_EQ(cc0.alias[0].offset, 2);
  EXPECT_EQ(cc0.alias[1].num, 2);
  EXPECT_EQ(cc0.alias[1].offset, 3);
  EXPECT_THAT(cc1.holes, SizeIs(3 * kChainLength));
  // kChainLength * 2 placeholders for channel 0 holes
  int pc = 0;
  EXPECT_EQ(cc0.program[pc++], 0xbaadf00d);
  EXPECT_EQ(cc0.program[pc++], 0xbaadf00d);
  EXPECT_EQ(cc0.program[pc++], 0xbaadf00d);
  EXPECT_EQ(cc0.program[pc++], 0xbaadf00d);
  EXPECT_EQ(cc0.program[pc++], 0xbaadf00d);
  EXPECT_EQ(cc0.program[pc++], 0xbaadf00d);
  // channel 0 terminator
  EXPECT_EQ(cc0.program[pc++], 0);
  EXPECT_EQ(cc0.program[pc++], 0);
  const int channel1_program_start = pc;
  // kChainLength * 1 placeholders for channel 1 holes
  EXPECT_EQ(cc0.program[pc++], 0xbaadf00d);
  EXPECT_EQ(cc0.program[pc++], 0xbaadf00d);
  EXPECT_EQ(cc0.program[pc++], 0xbaadf00d);
  // channel 1 terminator
  EXPECT_EQ(cc0.program[pc++], 0);

  // holes in channel-major order
  int hole_num = 0;
  pc = 0;
  // check holes for channel 0
  {
    for (int i = 0; i < kChainLength; ++i) {
      EXPECT_EQ(cc1.holes[hole_num].type, ChannelConfig::Field::kReadAddr)
          << hole_num;
      EXPECT_EQ(cc1.holes[hole_num].program_index, pc++);
      hole_num++;
      EXPECT_EQ(cc1.holes[hole_num].type, ChannelConfig::Field::kWriteAddr)
          << hole_num;
      EXPECT_EQ(cc1.holes[hole_num].program_index, pc++);
      hole_num++;
    }
  }
  // skip channel 0 terminator
  pc += 2;
  // check holes for channel 1
  {
    for (int i = 0; i < kChainLength; ++i) {
      EXPECT_EQ(cc1.holes[hole_num].type, ChannelConfig::Field::kWriteAddr)
          << hole_num;
      EXPECT_EQ(cc1.holes[hole_num].program_index, pc++);
      hole_num++;
    }
  }

  EXPECT_THAT(cc0.enable, ElementsAre(1, 1));
  EXPECT_EQ(cc0.initial_config_write_length[0], 2);
  EXPECT_EQ(cc0.initial_config_write_addr[0],
            // channel kExecutorChannels[0], alias 2, offset 0
            reinterpret_cast<uint32_t*>(0x50000000 +
                                        kExecutorChannels[0] * 0x40 + 0x20));
  EXPECT_EQ(cc0.initial_config_write_length[1], 3);
  EXPECT_EQ(cc0.initial_config_write_addr[1],
            // channel kExecutorChannels[1], alias 2, offset 0
            reinterpret_cast<uint32_t*>(0x50000000 +
                                        kExecutorChannels[1] * 0x40 + 0x20));
  EXPECT_EQ(cc0.initial_config[0][0],
            cmd.transfers[0].ctrl->Pack(true, kProgrammerChannels[0], true));
  EXPECT_EQ(cc0.initial_config[0][1], 42);
  EXPECT_EQ(cc0.initial_config[1][0],
            cmd.transfers[1].ctrl->Pack(true, kProgrammerChannels[1], true));
  EXPECT_EQ(cc0.initial_config[1][1], 56);
  EXPECT_EQ(cc0.initial_config[1][2], 0xcafebaba);

  EXPECT_EQ(cc0.programmer_config[0].read_addr,
            static_cast<uint32_t>(reinterpret_cast<intptr_t>(&cc0.program[0])));
  EXPECT_EQ(cc0.programmer_config[1].read_addr,
            static_cast<uint32_t>(reinterpret_cast<intptr_t>(
                &cc0.program[channel1_program_start])));
  EXPECT_EQ(cc0.programmer_config[0].write_addr,
            // channel kExecutorChannels[0], alias 2, offset 2
            0x50000000 + kExecutorChannels[0] * 0x40 + 0x28);
  EXPECT_EQ(cc0.programmer_config[1].write_addr,
            // channel kExecutorChannels[1], alias 2, offset 3
            0x50000000 + kExecutorChannels[1] * 0x40 + 0x2c);
  EXPECT_EQ(cc0.programmer_config[0].trans_count, 2);
  EXPECT_EQ(cc0.programmer_config[1].trans_count, 1);
  uint32_t expected_programmer0_ctrl(
      ChannelCtrl{
          .high_priority = 0,
          .data_size = ChannelCtrl::DataSize::k32,
          .incr_read = 1,
          .incr_write = 1,
          // 2-word = 8-byte ring boundary = lower 3 bits
          .ring_size = 3,
          .ring_sel = 1,
      }
          .Pack(true, kExecutorChannels[0], true));
  uint32_t expected_programmer1_ctrl(
      ChannelCtrl{
          .high_priority = 0,
          .data_size = ChannelCtrl::DataSize::k32,
          .incr_read = 1,
          .incr_write = 1,
          // 1-word = 4-byte ring boundary = lower 2 bits
          .ring_size = 2,
          .ring_sel = 1,
      }
          .Pack(true, kExecutorChannels[1], true));
  EXPECT_EQ(cc0.programmer_config[0].ctrl_trig, expected_programmer0_ctrl);
  EXPECT_EQ(cc0.programmer_config[1].ctrl_trig, expected_programmer1_ctrl);
  EXPECT_EQ(cc0.before, std::nullopt);
  EXPECT_EQ(cc0.after, std::nullopt);

  p.SetArg(0, {
                  {.read_addr = reinterpret_cast<const volatile void*>(1),
                   .write_addr = reinterpret_cast<volatile void*>(2)},
                  {.read_addr = reinterpret_cast<const volatile void*>(3),
                   .write_addr = reinterpret_cast<volatile void*>(4)},
                  {.read_addr = reinterpret_cast<const volatile void*>(5),
                   .write_addr = reinterpret_cast<volatile void*>(6)},
                  {.write_addr = reinterpret_cast<volatile void*>(7)},
                  {.write_addr = reinterpret_cast<volatile void*>(8)},
                  {.write_addr = reinterpret_cast<volatile void*>(9)},
              });
  pc = 0;
  EXPECT_EQ(cc0.program[pc++], 1);
  EXPECT_EQ(cc0.program[pc++], 2);
  EXPECT_EQ(cc0.program[pc++], 3);
  EXPECT_EQ(cc0.program[pc++], 4);
  EXPECT_EQ(cc0.program[pc++], 5);
  EXPECT_EQ(cc0.program[pc++], 6);
  EXPECT_EQ(cc0.program[pc++], 0);
  EXPECT_EQ(cc0.program[pc++], 0);
  EXPECT_EQ(cc0.program[pc++], 7);
  EXPECT_EQ(cc0.program[pc++], 8);
  EXPECT_EQ(cc0.program[pc++], 9);
  EXPECT_EQ(cc0.program[pc++], 0);
}