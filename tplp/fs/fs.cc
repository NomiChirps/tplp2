#include "tplp/fs/fs.h"

#include "fatfs/ff.h"
#include "picolog/picolog.h"
#include "picolog/status_macros.h"
#include "tplp/fs/fatfs_diskio_impl.h"

namespace tplp {
namespace fs {
namespace {

util::Status ToStatus(FRESULT c) {
  switch (c) {
    case FR_OK:
      return util::OkStatus();
    case FR_DISK_ERR:
      return util::DataLossError("Disk error");
    case FR_INT_ERR:
      return util::InternalError("FatFs internal error");
    case FR_NOT_READY:
      return util::FailedPreconditionError("Disk not ready");
    case FR_NO_FILE:
      return util::NotFoundError("File not found");
    case FR_NO_PATH:
      return util::NotFoundError("Path not found");
    case FR_INVALID_NAME:
      return util::InvalidArgumentError("Invalid name");
    case FR_DENIED:
      return util::FailedPreconditionError("Access denied");
    case FR_EXIST:
      return util::AlreadyExistsError("File already exists");
    case FR_INVALID_OBJECT:
      return util::FailedPreconditionError("Invalid object");
    case FR_WRITE_PROTECTED:
      return util::PermissionDeniedError("Media is write protected");
    case FR_INVALID_DRIVE:
      return util::InvalidArgumentError("Invalid drive number");
    case FR_NOT_ENABLED:
      return util::FailedPreconditionError("Filesystem not mounted");
    case FR_NO_FILESYSTEM:
      return util::NotFoundError("Valid FAT volume could not be found");
    case FR_MKFS_ABORTED:
      return util::FailedPreconditionError(
          "Cannot create filesystem with the given conditions");
    case FR_TIMEOUT:
      return util::DeadlineExceededError("Timeout");
    case FR_LOCKED:
      return util::AbortedError("Object is locked");
    case FR_NOT_ENOUGH_CORE:
      return util::ResourceExhaustedError("Not enough memory");
    case FR_TOO_MANY_OPEN_FILES:
      return util::ResourceExhaustedError("Too many open files");
    case FR_INVALID_PARAMETER:
      return util::InvalidArgumentError("Invalid parameter");
    default:
      LOG(ERROR) << "FatFs unknown error code: " << (int)c;
      return util::UnknownError("Unknown error code");
  }
}

static FATFS global_fs_work_area;

}  // namespace

util::Status Init(SdSpi* card) {
  SetGlobalDiskForFatFs(card);
  FRESULT res = f_mount(&global_fs_work_area, "0:", 1);
  if (res == FR_NO_FILESYSTEM) {
    LOG(INFO) << "Filesystem not found. Formatting...";
    BYTE work[FF_MAX_SS];
    // Create with default options.
    RETURN_IF_ERROR(ToStatus(
        f_mkfs("0:", /*opt=*/nullptr, work, sizeof(work) / sizeof(work[0]))));
    // Try to mount again.
    RETURN_IF_ERROR(ToStatus(f_mount(&global_fs_work_area, "0:", 1)));
  } else if (res != FR_OK) {
    return ToStatus(res);
  }
  LOG(INFO) << "FAT filesystem mounted. " << global_fs_work_area.free_clst
            << " free clusters.";
  return util::OkStatus();
}

util::StatusOr<size_t> GetContents(const char* path, char* buf, size_t n) {
  VLOG(1) << "ReadContents(" << path << ", " << (void*)buf << ", " << n << ")";
  static_assert(sizeof(UINT) == sizeof(size_t));
  FIL fp;
  FRESULT res;
  res = f_open(&fp, path, FA_READ);
  VLOG(1) << "f_open: " << ToStatus(res);
  if (res) return ToStatus(res);
  size_t count;
  res = f_read(&fp, buf, n, &count);
  VLOG(1) << "f_read: " << ToStatus(res);
  if (res) {
    FRESULT res_close = f_close(&fp);
    LOG_IF(ERROR, res_close)
        << "Error closing file after read error: " << ToStatus(res_close);
    return ToStatus(res);
  }
  res = f_close(&fp);
  if (res) return ToStatus(res);
  return count;
}

util::Status SetContents(const char* path, const char* buf, size_t n) {
  VLOG(1) << "SetContents(" << path << ", " << (void*)buf << ", " << n << ")";
  FIL fp;
  FRESULT res;
  res = f_open(&fp, path, FA_WRITE | FA_CREATE_ALWAYS);
  VLOG(1) << "f_open: " << ToStatus(res);
  if (res) return ToStatus(res);
  size_t count;
  res = f_write(&fp, buf, n, &count);
  VLOG(1) << "f_write: " << ToStatus(res);
  if (res) {
    FRESULT res_close = f_close(&fp);
    LOG_IF(ERROR, res_close)
        << "Error closing file after write error: " << ToStatus(res_close);
    return ToStatus(res);
  }
  return ToStatus(f_close(&fp));
}

}  // namespace fs
}  // namespace tplp