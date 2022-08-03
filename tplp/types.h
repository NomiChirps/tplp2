#ifndef TPLP_TYPES_H_
#define TPLP_TYPES_H_

#include "NamedType/named_type.hpp"
#include "pico/types.h"

namespace tplp {

using gpio_pin_t =
    fluent::NamedType<uint, struct GpioPinTag, fluent::FunctionCallable>;
using dma_channel_t =
    fluent::NamedType<int, struct DmaChannelTag, fluent::FunctionCallable>;

// Either 0 or 1, referring to the DMA unit.
using dma_irq_index_t =
    fluent::NamedType<int, struct DmaIrqIndexTag, fluent::FunctionCallable>;
// Refers to a physical IRQ number.
using dma_irq_number_t =
    fluent::NamedType<int, struct DmaIrqNumberTag, fluent::FunctionCallable>;

}  // namespace tplp

#endif  // TPLP_TYPES_H_