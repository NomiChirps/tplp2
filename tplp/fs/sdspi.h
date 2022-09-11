#ifndef TPLP_FS_SDSPI_H_
#define TPLP_FS_SDSPI_H_

#include "picolog/status.h"
#include "picolog/statusor.h"
#include "tplp/bus/spi.h"

namespace tplp {

// Provides block-device operations on an SD card via an SPI interface.
class SdSpi {
 public:
  explicit SdSpi(SpiDevice* spi);

  util::Status Init();
  bool initialized() const { return initialized_; }

  enum class CardType {
    SD1,
    SD2,
    SDHC,
  };
  CardType type() const { return type_; }

 private:
  enum R1 : uint8_t {
    R1_PARAMETER_ERROR = 0b01000000,
    R1_ADDRESS_ERROR = 0b00100000,
    R1_ERASE_SEQUENCE_ERROR = 0b00010000,
    R1_COM_CRC_ERROR = 0b00001000,
    R1_ILLEGAL_COMMAND = 0b00000100,
    R1_ERASE_RESET = 0b00000010,
    R1_IN_IDLE_STATE = 0b00000001,
  };

  static constexpr int kDefaultBusyTimeoutMs = 300;
  // returns first non-0xff byte read
  util::StatusOr<uint8_t> WaitNotBusy(SpiTransaction& txn,
                                      int timeout_ms = kDefaultBusyTimeoutMs,
                                      int delay_ms = 0);

  util::StatusOr<R1> CardCommand(SpiTransaction& txn, uint8_t cmd,
                                 uint32_t arg);
  util::StatusOr<R1> CardACommand(SpiTransaction& txn, uint8_t acmd,
                                  uint32_t arg);
  util::StatusOr<R1> CardCommandNoWait(SpiTransaction& txn, uint8_t cmd,
                                       uint32_t arg);
  util::Status R1ToStatus(R1 r1);

  static constexpr uint8_t kDummyTxBuf[8] = {0xff, 0xff, 0xff, 0xff,
                                             0xff, 0xff, 0xff, 0xff};

 private:
  SpiDevice* const spi_;
  CardType type_;
  bool initialized_;
};

}  // namespace tplp

#endif  // TPLP_FS_SDSPI_H_