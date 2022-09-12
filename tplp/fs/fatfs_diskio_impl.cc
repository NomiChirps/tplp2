#include "tplp/fs/fatfs_diskio_impl.h"

#include <cstdint>

#include "fatfs/ff.h"
#include "picolog/picolog.h"
#include "tplp/fs/sdspi.h"

// diskio.h must not be read before ff.h
#include "fatfs/diskio.h"

// TODO: consider using the implementation checker:
//       http://elm-chan.org/fsw/ff/res/app4.c

static tplp::SdSpi* disk_ = nullptr;

void tplp::fs::SetGlobalDiskForFatFs(tplp::SdSpi* card) { disk_ = card; }

DSTATUS disk_initialize(BYTE pdrv) {
  VLOG(1) << "disk_initialize(" << (int)pdrv << ")";
  if (!disk_) {
    LOG(ERROR) << "disk_initialize called before SdSpi instance was assigned";
    return STA_NOINIT;
  }
  util::Status status = disk_->Init();
  if (!status.ok()) {
    LOG(ERROR) << "Error initializing disk: " << status;
    return STA_NOINIT;
  }
  return 0;
}

DSTATUS disk_status(BYTE pdrv) {
  VLOG(1) << "disk_status(" << (int)pdrv << ")";
  if (disk_ && disk_->initialized()) {
    return 0;
  }
  return STA_NOINIT;
}

DRESULT disk_read(BYTE pdrv, BYTE* buff, LBA_t sector, UINT count) {
  VLOG(1) << "disk_read(" << (int)pdrv << ", " << buff << ", " << sector << ", "
          << count << ")";
  if (!disk_ || !disk_->initialized()) {
    return RES_NOTRDY;
  }
  if (sector >= disk_->size()) {
    return RES_PARERR;
  }
  for (uint32_t block = sector; block < sector + count; ++block) {
    util::Status status = disk_->ReadBlock(block, buff);
    if (!status.ok()) {
      LOG(ERROR) << "Error reading sector " << block << ": " << status;
      return RES_ERROR;
    }
    buff += tplp::SdSpi::kBlockSize;
  }
  return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count) {
  VLOG(1) << "disk_write(" << (int)pdrv << ", " << buff << ", " << sector
          << ", " << count << ")";
  if (!disk_ || !disk_->initialized()) {
    return RES_NOTRDY;
  }
  if (sector >= disk_->size()) {
    return RES_PARERR;
  }
  for (uint32_t block = sector; block < sector + count; ++block) {
    util::Status status = disk_->WriteBlock(block, buff);
    if (!status.ok()) {
      LOG(ERROR) << "Error writing sector " << block << ": " << status;
      return RES_ERROR;
    }
    buff += tplp::SdSpi::kBlockSize;
  }
  return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff) {
  VLOG(1) << "disk_ioctl(" << (int)pdrv << ", " << (int)cmd << ", " << buff
          << ")";
  if (!disk_ || !disk_->initialized()) {
    return RES_NOTRDY;
  }
  switch (cmd) {
    case CTRL_SYNC:
      // we don't use async writes (yet?)
      return RES_OK;
    case GET_SECTOR_COUNT:
      *static_cast<LBA_t*>(buff) = disk_->size();
      return RES_OK;
    case GET_SECTOR_SIZE:
      *static_cast<WORD*>(buff) = tplp::SdSpi::kBlockSize;
      return RES_OK;
    case GET_BLOCK_SIZE:
      // TODO: might want to set this up for large erases
      //       see http://elm-chan.org/fsw/ff/doc/dioctl.html
      *static_cast<DWORD*>(buff) = 1;
      return RES_OK;
    case CTRL_TRIM:
      // SD card doesn't support Discard command in SPI mode,
      // as far as I can tell...
      return RES_OK;
    default:
      return RES_PARERR;
  }
}

DWORD get_fattime() {
  // Current local time shall be returned as bit-fields packed into a DWORD
  // value. The bit fields are as follows:
  // bit31:25 Year origin from the 1980 (0..127, e.g. 37 for 2017)
  // bit24:21 Month (1..12)
  // bit20:16 Day of the month (1..31)
  // bit15:11 Hour (0..23)
  // bit10:5  Minute (0..59)
  // bit4:0   Second / 2 (0..29, e.g. 25 for 50)

  // I suppose in theory we could use the RTC. But why bother?
  return (11 << 25) | (9 << 21) | (6 << 16) | (4 << 11) | (20 << 5) | (0 << 0);
}
