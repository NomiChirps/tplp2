#include "tplp/bus/types.h"

#include <iomanip>

namespace tplp {

std::ostream& operator<<(std::ostream& s, const i2c_address_t& addr) {
  static_assert(sizeof(i2c_address_t::UnderlyingType) == 1);
  return s << "0x" << std::hex << std::setw(2) << std::setfill('0')
           << (int)addr.get();
}

}  // namespace tplp