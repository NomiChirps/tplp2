#include "tplp/fs/sdspi.h"

#include <cstring>
#include <iomanip>

#include "picolog/picolog.h"
#include "picolog/status_macros.h"
#include "tplp/fs/crc16.h"
#include "tplp/fs/crc7.h"
#include "tplp/rtos_util.h"

namespace tplp {
namespace {

util::Status WaitUntil(uint32_t timeout_ms, uint32_t delay_ms,
                       const std::function<util::StatusOr<bool>()>& done) {
  uint32_t start_time = to_ms_since_boot(get_absolute_time());
  while (to_ms_since_boot(get_absolute_time()) - start_time < timeout_ms) {
    auto status = done();
    if (!status.ok()) {
      return std::move(status).status();
    }
    if (status.value()) {
      return util::OkStatus();
    }
    if (delay_ms) {
      vTaskDelay(MillisToTicks(delay_ms));
    }
  }
  return util::DeadlineExceededError("timed out");
}

util::Status CheckCrc16(uint16_t given, uint16_t computed) {
  if (computed != given) {
    LOG(ERROR) << "CRC error, given != computed: 0x" << std::hex
               << std::setfill('0') << std::setw(4) << given << " != 0x"
               << std::setw(4) << computed;
    return util::DataLossError("CRC error");
  }
  return util::OkStatus();
}

}  // namespace

SdSpi::SdSpi(SpiDevice* spi) : spi_(spi), initialized_(false) {}

util::Status SdSpi::Init() {
  initialized_ = false;

  // For initialization sequence details see Figure 7-2 in the
  // Physical Layer Simplified Specification Version 2.0
  SpiTransaction txn = spi_->StartTransaction();
  util::Status status;
  R1 r1;

  // bizarrely, an SD card needs to see at least 74 clock cycles with CS *high*
  CHECK(txn.ToggleCS());
  for (int i = 0; i < 2; ++i) {
    constexpr int k = 5;
    txn.TransferImmediate({
        .read_addr = &kDummyTxBuf,
        .read_incr = false,
        .trans_count = k,
    });
  }
  CHECK(!txn.ToggleCS());

  // send CMD0 to enter SPI mode
  ASSIGN_OR_RETURN(r1, CardCommandNoWait(txn, 0, 0));
  if (r1 != R1_IN_IDLE_STATE) {
    // keep trying until it returns in_idle_state (bit 0)
    RETURN_IF_ERROR(WaitUntil(/*timeout=*/2000, /*delay=*/0,
                              [this, &txn]() -> util::StatusOr<bool> {
                                R1 r1;
                                ASSIGN_OR_RETURN(r1, CardCommand(txn, 0, 0));
                                return r1 == R1_IN_IDLE_STATE;
                              }));
  }

  // check SD version
  ASSIGN_OR_RETURN(r1, CardCommand(txn, 8, 0x1aa));
  if (r1 & R1_ILLEGAL_COMMAND) {
    LOG(INFO) << "Detected SD1 card.";
    type_ = CardType::SD1;
  } else {
    uint8_t r7[4];
    txn.TransferImmediate({
        .read_addr = &kDummyTxBuf,
        .read_incr = false,
        .write_addr = r7,
        .trans_count = 4,
    });
    if (r7[3] != 0xaa) {
      LOG(ERROR) << "Expected 0xaa echoed but got response: " << std::hex
                 << std::setw(2) << std::setfill('0') << (int)r7[0] << " "
                 << (int)r7[1] << " " << (int)r7[2] << " " << (int)r7[3];
      return util::UnknownError("device failed to echo check pattern");
    }
    // actually it might be SDHC, we'll check in a sec.
    type_ = CardType::SD2_SC;
  }

  // Enable card-side CRC verification (probably?)
  ASSIGN_OR_RETURN(r1, CardCommand(txn, 59, 1));

  // Issue ACMD41 "Initialization command"
  status = WaitUntil(
      /*timeout=*/1000, /*delay=*/0,
      [this, &txn, &r1]() -> util::StatusOr<bool> {
        ASSIGN_OR_RETURN(
            r1,
            CardACommand(txn, 41, type_ == CardType::SD2_SC ? 0x40000000 : 0));
        return r1 == 0;
      });
  if (!status.ok()) {
    LOG(ERROR) << "Timed out waiting for ACMD41. Last error: "
               << R1ToStatus(r1);
    return R1ToStatus(r1);
  }

  // if SD2 read OCR register to check for SDHC card
  if (type() == CardType::SD2_SC) {
    ASSIGN_OR_RETURN(r1, CardCommand(txn, 58, 0));
    RETURN_IF_ERROR(R1ToStatus(r1));
    uint8_t ocr[4];
    txn.TransferImmediate({
        .read_addr = &kDummyTxBuf,
        .read_incr = false,
        .write_addr = ocr,
        .trans_count = 4,
    });
    if ((ocr[0] & 0xc0) == 0xc0) {
      LOG(INFO) << "Detected SDHC card.";
      type_ = CardType::SD2_HC;
    } else {
      LOG(INFO) << "Detected SDSC card.";
    }
  } else {
    LOG(INFO) << "Detected SD 1.x card.";
  }
  // Done with low-level init sequence.
  txn.Dispose();

  ASSIGN_OR_RETURN(size_, GetSize());
  cid_t cid;
  ASSIGN_OR_RETURN(cid, ReadCID());
  LOG(INFO) << "Initialized card: "
            << std::string_view(cid.oid, sizeof(cid.oid)) << " "
            << std::string_view(cid.pnm, sizeof(cid.pnm)) << " SN#" << cid.psn
            << "; " << size_ << " blocks";

  initialized_ = true;
  return util::OkStatus();
}

util::StatusOr<uint8_t> SdSpi::WaitNotBusy(SpiTransaction& txn, int timeout_ms,
                                           int delay_ms) {
  uint8_t buf;
  RETURN_IF_ERROR(
      WaitUntil(timeout_ms, delay_ms, [&txn, &buf]() -> util::StatusOr<bool> {
        txn.TransferImmediate({
            .read_addr = &kDummyTxBuf,
            .read_incr = false,
            .write_addr = &buf,
            .trans_count = 1,
        });
        VLOG(1) << "WaitNotBusy: 0x" << std::hex << std::setw(2)
                << std::setfill('0') << (int)buf;
        return buf == 0xff;
      }));
  return buf;
}

util::StatusOr<SdSpi::R1> SdSpi::CardCommand(SpiTransaction& txn, uint8_t cmd,
                                             uint32_t arg) {
  RETURN_IF_ERROR(WaitNotBusy(txn).status());
  return CardCommandNoWait(txn, cmd, arg);
}

util::StatusOr<SdSpi::R1> SdSpi::CardACommand(SpiTransaction& txn, uint8_t acmd,
                                              uint32_t arg) {
  RETURN_IF_ERROR(CardCommand(txn, 55, 0).status());
  return CardCommand(txn, acmd, arg);
}

util::StatusOr<SdSpi::R1> SdSpi::CardCommandNoWait(SpiTransaction& txn,
                                                   uint8_t cmd, uint32_t arg) {
  uint8_t buf[6];
  CHECK_LT(cmd, 0x40);
  buf[0] = 0x40 | cmd;
  buf[1] = arg >> 24;
  buf[2] = arg >> 16;
  buf[3] = arg >> 8;
  buf[4] = arg >> 0;
  buf[5] = (crc7(0, buf, 5) << 1) | 0x01;
  txn.TransferImmediate({
      .read_addr = buf,
      .trans_count = 6,
  });
  // wait for response
  R1 r1;
  RETURN_IF_ERROR(WaitUntil(kDefaultBusyTimeoutMs, 0,
                            [&txn, &r1]() -> util::StatusOr<bool> {
                              txn.TransferImmediate({
                                  .read_addr = &kDummyTxBuf,
                                  .read_incr = false,
                                  .write_addr = &r1,
                                  .trans_count = 1,
                              });
                              return !(r1 & 0x80);
                            }));
  VLOG(1) << std::hex << std::setw(2) << std::setfill('0') << "CardCommand(0x"
          << (int)cmd << ", 0x" << arg << "): " << (int)buf[0] << " "
          << (int)buf[1] << " " << (int)buf[2] << " " << (int)buf[3] << " "
          << (int)buf[4] << " " << (int)buf[5] << " -> " << (int)r1;
  return r1;
}

util::Status SdSpi::R1ToStatus(R1 r1) {
  if (r1 & 0x80) {
    // MSB is supposed to always be zero
    return util::UnknownError("R1 MSB was set");
  } else if (r1 & 0x40) {
    return util::InvalidArgumentError("SD protocol parameter error");
  } else if (r1 & 0x20) {
    return util::InvalidArgumentError("SD protocol address error");
  } else if (r1 & 0x10) {
    return util::FailedPreconditionError("SD protocol erase sequence error");
  } else if (r1 & 0x08) {
    return util::DataLossError("SD protocol CRC error");
  } else if (r1 & 0x04) {
    return util::InvalidArgumentError("SD protocol illegal command");
  } else if (r1 & 0x02) {
    return util::FailedPreconditionError("SD protocol erase reset error");
  } else if (r1 & 0x01) {
    // This isn't actually an error.
    return util::UnknownError("card is in idle state; protocol error?");
  } else {
    return util::OkStatus();
  }
  return util::OkStatus();
}

util::StatusOr<uint32_t> SdSpi::GetSize() {
  csd_t csd;
  ASSIGN_OR_RETURN(csd, ReadCSD());
  if (csd.v1.csd_ver == 0) {
    uint8_t read_bl_len = csd.v1.read_bl_len;
    uint16_t c_size = (csd.v1.c_size_high << 10) | (csd.v1.c_size_mid << 2) |
                      csd.v1.c_size_low;
    uint8_t c_size_mult =
        (csd.v1.c_size_mult_high << 1) | csd.v1.c_size_mult_low;
    return (uint32_t)(c_size + 1) << (c_size_mult + read_bl_len - 7);
  } else if (csd.v2.csd_ver == 1) {
    uint32_t c_size = ((uint32_t)csd.v2.c_size_high << 16) |
                      (csd.v2.c_size_mid << 8) | csd.v2.c_size_low;
    return (c_size + 1) << 10;
  } else {
    return util::UnknownError("Bad CSD");
  }
}

util::StatusOr<SdSpi::csd_t> SdSpi::ReadCSD() {
  csd_t csd;
  static_assert(sizeof(csd) == 16);
  RETURN_IF_ERROR(ReadRegister(9, reinterpret_cast<uint8_t*>(&csd)));
  return csd;
}

util::StatusOr<SdSpi::cid_t> SdSpi::ReadCID() {
  cid_t cid;
  static_assert(sizeof(cid) == 16);
  RETURN_IF_ERROR(ReadRegister(10, reinterpret_cast<uint8_t*>(&cid)));
  return cid;
}

util::Status SdSpi::ReadRegister(uint8_t cmd, uint8_t buf[16]) {
  SpiTransaction txn = spi_->StartTransaction();
  RETURN_IF_ERROR(CardCommand(txn, cmd, 0).status());
  RETURN_IF_ERROR(WaitStartBlock(txn));
  txn.Transfer({
      .read_addr = &kDummyTxBuf,
      .read_incr = false,
      .write_addr = buf,
      .trans_count = 16,
  });
  uint8_t crc_buf[2];
  txn.Transfer({
      .read_addr = &kDummyTxBuf,
      .read_incr = false,
      .write_addr = &crc_buf,
      .trans_count = 2,
  });
  uint16_t crc = (crc_buf[0] << 8) | crc_buf[1];
  txn.Dispose();
  RETURN_IF_ERROR(CheckCrc16(crc, crc16(0, buf, 16)));
  return util::OkStatus();
}

util::Status SdSpi::WaitStartBlock(SpiTransaction& txn, int timeout_ms,
                                   int delay_ms) {
  uint8_t buf;
  RETURN_IF_ERROR(
      WaitUntil(timeout_ms, delay_ms, [&txn, &buf]() -> util::StatusOr<bool> {
        txn.TransferImmediate({
            .read_addr = &kDummyTxBuf,
            .read_incr = false,
            .write_addr = &buf,
            .trans_count = 1,
        });
        VLOG(1) << "WaitStartBlock: 0x" << std::hex << std::setw(2)
                << std::setfill('0') << (int)buf;
        return buf != 0xff;
      }));
  // 0xfe is the start block token
  if (buf != 0xfe) {
    LOG(ERROR) << "Got R1 = 0x" << std::hex << std::setw(2) << std::setfill('0')
               << (int)buf << " instead of start block token";
    return util::UnknownError(
        "timed out waiting for start block token; protocol error?");
  }
  return util::OkStatus();
}

util::Status SdSpi::ReadBlock(uint32_t block, void* buf) {
  SpiTransaction txn = spi_->StartTransaction();
  R1 r1;
  if (type_ == CardType::SD2_SC) {
    ASSIGN_OR_RETURN(r1, CardCommand(txn, 17, block * kBlockSize));
  } else {
    ASSIGN_OR_RETURN(r1, CardCommand(txn, 17, block));
  }
  if (r1) return R1ToStatus(r1);
  RETURN_IF_ERROR(WaitStartBlock(txn));
  txn.Transfer({
      .read_addr = &kDummyTxBuf,
      .read_incr = false,
      .write_addr = buf,
      .trans_count = 512,
  });
  uint8_t crc_buf[2];
  txn.Transfer({
      .read_addr = &kDummyTxBuf,
      .read_incr = false,
      .write_addr = crc_buf,
      .trans_count = 2,
  });
  txn.Dispose();
  uint16_t crc = (crc_buf[0] << 8) | crc_buf[1];
  RETURN_IF_ERROR(CheckCrc16(crc, crc16(0, buf, 512)));
  return util::OkStatus();
}

util::Status SdSpi::WriteBlock(uint32_t block, const void* buf) {
  SpiTransaction txn = spi_->StartTransaction();
  R1 r1;
  if (type_ == CardType::SD2_SC) {
    ASSIGN_OR_RETURN(r1, CardCommand(txn, 24, block * kBlockSize));
  } else {
    ASSIGN_OR_RETURN(r1, CardCommand(txn, 24, block));
  }
  if (r1) return R1ToStatus(r1);
  constexpr uint8_t kStartBlockToken = 0xfe;
  txn.Transfer({
      .read_addr = &kStartBlockToken,
      .trans_count = 1,
  });
  txn.Transfer({
      .read_addr = buf,
      .trans_count = 512,
  });
  uint8_t crc_buf[2];
  uint16_t crc = crc16(0, buf, 512);
  crc_buf[0] = crc >> 8;
  crc_buf[1] = crc >> 0;
  txn.Transfer({
      .read_addr = crc_buf,
      .trans_count = 2,
  });
  txn.Transfer({
      .read_addr = &kDummyTxBuf,
      .read_incr = false,
      .write_addr = &r1,
      .trans_count = 1,
  });
  txn.Flush();
  VLOG(1) << "WriteBlock response token: 0x" << std::hex << std::setfill('0')
          << std::setw(2) << (int)r1;
  // parse response token
  switch (r1 & 0x1f) {
    case 0b00101:
      // data accepted
      break;
    case 0b01011:
      // data rejected due to crc error
      return util::DataLossError("SD card rejected data due to CRC error");
    case 0b01101:
      // data rejected due to write error
      return util::UnknownError("SD card reported a write error");
      default:
      LOG(ERROR) << "Bad WriteBlock response token: 0x" << std::hex << (int)r1;
      return util::UnknownError("Bad WriteBlock response token");
  }
  // wait for flash programming to complete
  RETURN_IF_ERROR(WaitNotBusy(txn).status());
  ASSIGN_OR_RETURN(r1, CardCommand(txn, 13, 0));
  if (r1) {
    LOG(ERROR) << "CMD13 error 0x" << std::hex << (int)r1;
    return R1ToStatus(r1);
  }
  uint8_t r2;
  txn.TransferImmediate({
      .read_addr = &kDummyTxBuf,
      .read_incr = false,
      .write_addr = &r2,
      .trans_count = 1,
  });
  if (r2) {
    LOG(ERROR) << "Write error (R2 format): 0x" << (int)r2;
    // TODO: translate error codes
    return util::UnknownError("Write error (see log)");
  }

  return util::OkStatus();
}

}  // namespace tplp