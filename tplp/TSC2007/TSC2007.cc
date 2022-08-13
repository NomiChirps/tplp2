#include "tplp/TSC2007/TSC2007.h"

#include <iomanip>

#include "picolog/picolog.h"
#include "picolog/status_macros.h"

namespace tplp {
namespace {

enum tsc2007_function : uint8_t {
  MEASURE_TEMP0 = 0,
  MEASURE_AUX = 2,
  MEASURE_TEMP1 = 4,
  ACTIVATE_X = 8,
  ACTIVATE_Y = 9,
  ACTIVATE_YPLUS_X = 10,
  SETUP_COMMAND = 11,
  MEASURE_X = 12,
  MEASURE_Y = 13,
  MEASURE_Z1 = 14,
  MEASURE_Z2 = 15
};

enum tsc2007_power : uint8_t {
  POWERDOWN_IRQON = 0,
  ADON_IRQOFF = 1,
  ADOFF_IRQON = 2,
};

enum tsc2007_resolution : uint8_t {
  ADC_12BIT = 0,
  ADC_8BIT = 1,
};

util::Status Command(I2cTransaction& txn, tsc2007_function func,
                     tsc2007_power pwr, tsc2007_resolution res,
                     bool stop = false) {
  uint8_t cmd = func << 4;
  cmd |= pwr << 2;
  cmd |= res << 1;
  return txn.Write(&cmd, 1, stop);
}

int16_t Convert12BitValue(const uint8_t buf[2]) {
  return (static_cast<uint16_t>(buf[0]) << 4) |
         (static_cast<uint16_t>(buf[1]) >> 4);
}

}  // namespace

// "In both cases previously listed, it is recommended that whenever the host
// writes to the TSC2007, the controller processor masks the interrupt
// associated to PENIRQ. This masking prevents false triggering of interrupts
// when the PENIRQ line is disabled in the cases previously listed."

TSC2007::TSC2007(I2cDeviceHandle device) : i2c_(device) {}

util::Status TSC2007::Setup() {
  I2cTransaction txn = i2c_.StartTransaction();
  return Command(txn, MEASURE_TEMP0, POWERDOWN_IRQON, ADC_12BIT, true);
}

util::Status TSC2007::ReadPosition(int16_t* x, int16_t* y, int16_t* z1,
                                   int16_t* z2) {
  // "For best performance, the I2C bus should remain in an idle state while
  // an A/D conversion is taking place. This idling prevents digital clock noise
  // from affecting the bit decisions being made by the TSC2007. The controller
  // should wait for at least 10us before attempting to read data from the
  // TSC2007 to realize this best performance. However, the controller does not
  // need to wait for a completed conversion before beginning a read from the
  // target, if full 12-bit performance is not necessary."
  uint8_t buf[2];
  // TODO: it would be so nice if I2cTransaction supported nonblocking ops
  //       and we could queue up this whole thing in one go!
  I2cTransaction txn = i2c_.StartTransaction();
  if (x) {
    RETURN_IF_ERROR(Command(txn, MEASURE_X, ADON_IRQOFF, ADC_12BIT));
    RETURN_IF_ERROR(txn.Read(buf, 2));
    VLOG(1) << std::hex << std::setw(2) << std::setfill('0') << "X = [0x"
            << (int)buf[0] << ", 0x" << (int)buf[1] << "]";
    *x = Convert12BitValue(buf);
  }
  if (y) {
    RETURN_IF_ERROR(Command(txn, MEASURE_Y, ADON_IRQOFF, ADC_12BIT));
    RETURN_IF_ERROR(txn.Read(buf, 2));
    VLOG(1) << std::hex << std::setw(2) << std::setfill('0') << "Y = [0x"
            << (int)buf[0] << ", 0x" << (int)buf[1] << "]";
    *y = Convert12BitValue(buf);
  }
  if (z1) {
    RETURN_IF_ERROR(Command(txn, MEASURE_Z1, ADON_IRQOFF, ADC_8BIT));
    RETURN_IF_ERROR(txn.Read(buf, 1));
    VLOG(1) << std::hex << std::setw(2) << std::setfill('0') << "Z1 = 0x"
            << (int)buf[0];
    *z1 = buf[0];
  }
  if (z2) {
    RETURN_IF_ERROR(Command(txn, MEASURE_Z1, ADON_IRQOFF, ADC_8BIT));
    RETURN_IF_ERROR(txn.Read(buf, 1));
    VLOG(1) << std::hex << std::setw(2) << std::setfill('0') << "Z2 = 0x"
            << (int)buf[0];
    *z2 = buf[0];
  }
  return Command(txn, MEASURE_TEMP0, POWERDOWN_IRQON, ADC_12BIT, true);
}

}  // namespace tplp
