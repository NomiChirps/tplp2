#include "tplp/hx711/hx711.h"

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "picolog/picolog.h"
#include "tplp/hx711/hx711.pio.h"

// Concept:
// 1. PIO program continuously samples the HX711 at a given rate.
// 2. a single DMA channel continuously empties the RX FIFO into a volatile
// int32_t in system memory, which can be read atomically at any time by
// whomever.
// Optionally: timer of some sort periodically checks to make sure the DMA is
// still running (and, therefore, that the PIO is still running).
namespace tplp {
namespace {}  // namespace

HX711* HX711::Init(pio_hw_t* pio, gpio_pin_t sck, gpio_pin_t dout) {
  CHECK(pio_can_add_program(pio, &hx711_program));
  auto offset = pio_add_program(pio, &hx711_program);
  auto sm = pio_claim_unused_sm(pio, false);
  LOG_IF(FATAL, sm < 0) << "No free state machines in PIO"
                        << pio_get_index(pio);
  pio_sm_config piocfg = hx711_program_get_default_config(offset);

  // SCK
  pio_gpio_init(pio, sck);
  pio_sm_set_consecutive_pindirs(pio, sm, sck, 1, true);
  sm_config_set_sideset_pins(&piocfg, sck);
  sm_config_set_sideset(&piocfg, 1, /*optional=*/false, /*pindirs=*/false);

  // DOUT
  pio_gpio_init(pio, dout);
  pio_sm_set_consecutive_pindirs(pio, sm, dout, 1, false);
  sm_config_set_in_pins(&piocfg, dout);
  sm_config_set_in_shift(&piocfg, /*push_right=*/false, /*autopush=*/true,
                         /*push_threshold*/ 24);

  // max frequency for HX711 is 10MHz. closest without going over wins.
  auto sys_hz = clock_get_hz(clk_sys);
  auto clock_divider = sys_hz / 10'000'000 + (sys_hz % 10'000'000 != 0);
  LOG(INFO) << "HX711 clock divider = " << clock_divider
            << "; Hz = " << sys_hz / clock_divider;
  CHECK_LE(sys_hz / clock_divider, 10'000'000u);
  sm_config_set_clkdiv_int_frac(&piocfg, clock_divider, 0);

  HX711* self = CHECK_NOTNULL(new HX711());
  pio_sm_init(pio, sm, offset, &piocfg);
  pio_sm_set_enabled(pio, sm, true);

  // Start two DMA channels that chain into each other in an endless loop.
  // This is really overkill because even at the maximum of 80 samples per
  // second, a single channel will still take literally years to count down
  // from a transfer_count of uint32_max.
  dma_channel_t c0 = dma_channel_t(dma_claim_unused_channel(false));
  CHECK_GE(c0, 0) << "Not enough free DMA channels available";
  dma_channel_t c1 = dma_channel_t(dma_claim_unused_channel(false));
  CHECK_GE(c1, 0) << "Not enough free DMA channels available";
  dma_channel_config c = {};
  channel_config_set_ring(&c, false, 0);
  channel_config_set_bswap(&c, false);
  channel_config_set_irq_quiet(&c, true);
  channel_config_set_enable(&c, true);
  channel_config_set_sniff_enable(&c, false);
  channel_config_set_high_priority(&c, false);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, false);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
  channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

  // c0
  channel_config_set_chain_to(&c, c1);
  dma_channel_configure(c0, &c, &self->sampled_value_, &pio->rxf[sm],
                        std::numeric_limits<uint32_t>::max(),
                        /*trigger=*/false);
  // c1 & start
  channel_config_set_chain_to(&c, c0);
  dma_channel_configure(c0, &c, &self->sampled_value_, &pio->rxf[sm],
                        std::numeric_limits<uint32_t>::max(),
                        /*trigger=*/true);

  // Now the PIO is running, filling pio->rxf, and a DMA is running forever,
  // copying words out into the sampling location as fast as they arrive.

  return self;
}

HX711::HX711() : sampled_value_(0) {}

}  // namespace tplp