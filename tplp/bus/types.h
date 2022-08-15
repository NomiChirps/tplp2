#ifndef TPLP_BUS_TYPES_H_
#define TPLP_BUS_TYPES_H_

#include <ostream>

#include "NamedType/named_type.hpp"

namespace tplp {

using gpio_pin_t = fluent::NamedType<unsigned int, struct GpioPinTag,
                                     fluent::FunctionCallable>;

using dma_channel_t =
    fluent::NamedType<int, struct DmaChannelTag, fluent::FunctionCallable>;

// Either 0 or 1, referring to the DMA unit.
using dma_irq_index_t =
    fluent::NamedType<int, struct DmaIrqIndexTag, fluent::FunctionCallable>;
// Refers to a physical IRQ number.
using dma_irq_number_t =
    fluent::NamedType<int, struct DmaIrqNumberTag, fluent::FunctionCallable>;

// Numeric address, 0<=value<256.
using i2c_address_t =
    fluent::NamedType<uint8_t, struct I2cAddressTag, fluent::PreIncrementable,
                      fluent::Comparable>;

std::ostream& operator<<(std::ostream& s, const i2c_address_t& addr);

}  // namespace tplp

#endif  // TPLP_BUS_TYPES_H_