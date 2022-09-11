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

  enum class CardType;
  CardType type() const { return type_; }

  // Returns the size of the card in 512-byte blocks.
  uint32_t size() const;

  union csd_t;
  util::StatusOr<csd_t> ReadCSD();
  struct cid_t;
  util::StatusOr<cid_t> ReadCID();

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

  // TODO: add dma ring support to SPI interface
  static constexpr uint8_t kDummyTxBuf[16] = {
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

  util::StatusOr<uint32_t> GetSize();
  // Read CID or CSR register (16 bytes)
  util::Status ReadRegister(uint8_t cmd, uint8_t buf[16]);
  util::Status WaitStartBlock(SpiTransaction& txn,
                              int timeout_ms = kDefaultBusyTimeoutMs,
                              int delay_ms = 0);

 private:
  SpiDevice* const spi_;
  CardType type_;
  bool initialized_;

  uint32_t size_;  // blocks

 public:
  // Data blocks are 512 bytes.
  static constexpr uint32_t kBlockSize = 512;

  enum class CardType {
    SD1,
    SD2,
    SDHC,
  };

  struct cid_t {
    // byte 0
    uint8_t mid;  // Manufacturer ID
    // byte 1-2
    char oid[2];  // OEM/Application ID
    // byte 3-7
    char pnm[4];  // Product name
    // byte 8
    unsigned prv_m : 4;  // Product revision n.m
    unsigned prv_n : 4;
    // byte 9-12
    uint32_t psn;  // Product serial number
    // byte 13
    unsigned mdt_year_high : 4;  // Manufacturing date
    unsigned reserved : 4;
    // byte 14
    unsigned mdt_month : 4;
    unsigned mdt_year_low : 4;
    // byte 15
    unsigned always1 : 1;
    unsigned crc : 7;
  };

  // CSD for version 1.00 cards
  struct csd1_t {
    // byte 0
    unsigned reserved1 : 6;
    unsigned csd_ver : 2;
    // byte 1
    uint8_t taac;
    // byte 2
    uint8_t nsac;
    // byte 3
    uint8_t tran_speed;
    // byte 4
    uint8_t ccc_high;
    // byte 5
    unsigned read_bl_len : 4;
    unsigned ccc_low : 4;
    // byte 6
    unsigned c_size_high : 2;
    unsigned reserved2 : 2;
    unsigned dsr_imp : 1;
    unsigned read_blk_misalign : 1;
    unsigned write_blk_misalign : 1;
    unsigned read_bl_partial : 1;
    // byte 7
    uint8_t c_size_mid;
    // byte 8
    unsigned vdd_r_curr_max : 3;
    unsigned vdd_r_curr_min : 3;
    unsigned c_size_low : 2;
    // byte 9
    unsigned c_size_mult_high : 2;
    unsigned vdd_w_cur_max : 3;
    unsigned vdd_w_curr_min : 3;
    // byte 10
    unsigned sector_size_high : 6;
    unsigned erase_blk_en : 1;
    unsigned c_size_mult_low : 1;
    // byte 11
    unsigned wp_grp_size : 7;
    unsigned sector_size_low : 1;
    // byte 12
    unsigned write_bl_len_high : 2;
    unsigned r2w_factor : 3;
    unsigned reserved3 : 2;
    unsigned wp_grp_enable : 1;
    // byte 13
    unsigned reserved4 : 5;
    unsigned write_partial : 1;
    unsigned write_bl_len_low : 2;
    // byte 14
    unsigned reserved5 : 2;
    unsigned file_format : 2;
    unsigned tmp_write_protect : 1;
    unsigned perm_write_protect : 1;
    unsigned copy : 1;
    unsigned file_format_grp : 1;
    // byte 15
    unsigned always1 : 1;
    unsigned crc : 7;
  };

  // CSD for version 2.00 cards
  struct csd2_t {
    // byte 0
    unsigned reserved1 : 6;
    unsigned csd_ver : 2;
    // byte 1
    uint8_t taac;
    // byte 2
    uint8_t nsac;
    // byte 3
    uint8_t tran_speed;
    // byte 4
    uint8_t ccc_high;
    // byte 5
    unsigned read_bl_len : 4;
    unsigned ccc_low : 4;
    // byte 6
    unsigned reserved2 : 4;
    unsigned dsr_imp : 1;
    unsigned read_blk_misalign : 1;
    unsigned write_blk_misalign : 1;
    unsigned read_bl_partial : 1;
    // byte 7
    unsigned reserved3 : 2;
    unsigned c_size_high : 6;
    // byte 8
    uint8_t c_size_mid;
    // byte 9
    uint8_t c_size_low;
    // byte 10
    unsigned sector_size_high : 6;
    unsigned erase_blk_en : 1;
    unsigned reserved4 : 1;
    // byte 11
    unsigned wp_grp_size : 7;
    unsigned sector_size_low : 1;
    // byte 12
    unsigned write_bl_len_high : 2;
    unsigned r2w_factor : 3;
    unsigned reserved5 : 2;
    unsigned wp_grp_enable : 1;
    // byte 13
    unsigned reserved6 : 5;
    unsigned write_partial : 1;
    unsigned write_bl_len_low : 2;
    // byte 14
    unsigned reserved7 : 2;
    unsigned file_format : 2;
    unsigned tmp_write_protect : 1;
    unsigned perm_write_protect : 1;
    unsigned copy : 1;
    unsigned file_format_grp : 1;
    // byte 15
    unsigned always1 : 1;
    unsigned crc : 7;
  };

  // union of old and new style CSD register
  union csd_t {
    csd1_t v1;
    csd2_t v2;
  };
};

}  // namespace tplp

#endif  // TPLP_FS_SDSPI_H_