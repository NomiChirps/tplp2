#ifndef TPLP_FS_FATFS_DISKIO_IMPL_H_
#define TPLP_FS_FATFS_DISKIO_IMPL_H_

#include "tplp/fs/sdspi.h"

namespace tplp {
namespace fs {

void SetGlobalDiskForFatFs(SdSpi* card);

}  // namespace fs
}  // namespace tplp

#endif  // TPLP_FS_FATFS_DISKIO_IMPL_H_