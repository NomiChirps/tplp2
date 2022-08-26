#ifndef TPLP_HX711_HX711_H_
#define TPLP_HX711_HX711_H_

#include "hardware/dma.h"
#include "hardware/pio.h"
#include "tplp/bus/types.h"

namespace tplp {

// This HX711 driver uses a PIO machine and 2 DMA channels to continuously read
// samples from the HX711 as fast as it can supply them, at precisely the
// maximum rate specified in the datasheet. One DMA at a time continuously reads
// from the PIO's RX FIFO (waiting on the relevant DREQ) and copies the latest
// value to a fixed memory location, `sampled_value_`. The DMAs chain into each
// other to ensure that operation continues if transfer_count reaches zero.
//
// All the CPU has to do is read the latest value from `sampled_value_` at its
// leisure. The only way this could be improved is if we squeezed the
// sign-extension bit-twiddling into the PIO code, but the necessary quantity of
// PIO instructions is arguably more precious than a few CPU cycles.
class HX711 {
 public:
  static HX711* Init(pio_hw_t* pio, gpio_pin_t sck, gpio_pin_t dout);

  inline int32_t current_value() const {
    return signextend<int32_t, 24>(sampled_value_);
  }

 private:
  explicit HX711();
  // Not copyable or movable.
  HX711(const HX711&) = delete;
  HX711& operator=(const HX711&) = delete;

  // GCC implements this with 2 instructions: `LSLS r0,#8; ASRS r0,#8`.
  template <typename T, unsigned B>
  inline static T signextend(const T x) {
    struct {
      T x : B;
    } s;
    return s.x = x;
  }

 private:
  volatile int32_t sampled_value_;
};

};  // namespace tplp

#endif  // TPLP_HX711_HX711_H_