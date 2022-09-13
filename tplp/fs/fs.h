#ifndef TPLP_FS_FS_H_
#define TPLP_FS_FS_H_

#include <string_view>

#include "picolog/status.h"
#include "tplp/fs/sdspi.h"

namespace tplp {
namespace fs {

// Opens the FAT filesystem on the given device, formatting it and creating one
// if it doesn't exist.
util::Status Init(SdSpi* card);

// Reads the contents of a file into `buf`, up to at most `n` bytes. Returns the
// number of bytes read or an error.
util::StatusOr<size_t> ReadContents(const char* path, char* buf,
                                    size_t n);

// Sets the contents of a file to the first `n` bytes of `buf`.
util::Status SetContents(const char* path, const char* buf, size_t n);

}  // namespace fs
}  // namespace tplp

#endif  // TPLP_FS_FS_H_