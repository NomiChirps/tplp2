#ifndef TPLP_TYPES_H_
#define TPLP_TYPES_H_

#include "pico/types.h"

// Why pico-sdk doesn't declare these I do not know.
// TODO: consider giving up on these typedefs :(
//       type safety just not happening without a major wrapper layer.
using gpio_pin_t = uint;
using dma_channel_t = int;
static constexpr dma_channel_t kDmaChannelInvalid = -1;

#endif  // TPLP_TYPES_H_