#include "tplp/fs/fatfs_diskio_impl.h"

#include <cstdint>

#include "fatfs/ff.h"
#include "hardware/rtc.h"
#include "picolog/picolog.h"
#include "tplp/fs/sdspi.h"

// diskio.h must not be read before ff.h
#include "fatfs/diskio.h"

// TODO: consider using the implementation checker:
//       http://elm-chan.org/fsw/ff/res/app4.c

DSTATUS disk_initialize(BYTE pdrv) {
  // TODO
}

DSTATUS disk_status(BYTE pdrv) {
  // TODO
}

DRESULT disk_read(BYTE pdrv, BYTE* buff, LBA_t sector, UINT count) {
  VLOG(1) << "disk_read(" << (int)pdrv << ", " << buff << ", " << sector << ", "
          << count << ")";
  // TODO
}

DRESULT disk_write(BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count) {
  VLOG(1) << "disk_write(" << (int)pdrv << ", " << buff << ", " << sector
          << ", " << count << ")";
  // TODO
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff) {
  VLOG(1) << "disk_ioctl(" << (int)pdrv << ", " << (int)cmd << ", " << buff
          << ")";
  // TODO
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
