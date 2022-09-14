
#include "tplp/config/params_storage.h"

#include "picolog/picolog.h"
#include "picolog/status_macros.h"
#include "tplp/config/params.h"
#include "tplp/fs/fs.h"

namespace tplp {
namespace config {
namespace {
static constexpr size_t kMaxSerializedValueLen = 512;
}

util::Status SaveParameter(const ParameterBase* param) {
  char buf[kMaxSerializedValueLen];
  auto bytes_count = param->Serialize(buf, kMaxSerializedValueLen);
  if (!bytes_count.ok()) {
    LOG(ERROR) << "Error serializing parameter value " << param->name() << ": "
               << bytes_count.status();
    return util::InternalError("serialization error");
  }
  util::Status status = fs::SetContents(param->file_path(), buf, *bytes_count);
  if (!status.ok()) {
    LOG(ERROR) << "Error writing " << param->file_path() << ": " << status;
    return status;
  }
  LOG(INFO) << "Saved parameter: " << param->name() << " = "
            << param->DebugString();
  return util::OkStatus();
}

util::Status LoadParameter(ParameterBase* param) {
  char buf[kMaxSerializedValueLen];
  auto bytes_read =
      fs::GetContents(param->file_path(), buf, kMaxSerializedValueLen);
  if (!bytes_read.ok()) {
    LOG(ERROR) << "Error reading " << param->file_path() << ": "
               << bytes_read.status();
    return std::move(bytes_read).status();
  }
  auto status = param->Parse(std::string_view(buf, *bytes_read));
  if (!status.ok()) {
    LOG(ERROR) << "Error parsing parameter value " << param->name() << ": "
               << status;
    return status;
  }
  LOG(INFO) << "Loaded parameter: " << param->name() << " = "
            << param->DebugString();
  return util::OkStatus();
}

util::Status LoadAllParameters() {
  util::Status status;
  util::Status last_error;
  for (ParameterBase* param : AllParameters()) {
    status = LoadParameter(param);
    if (!status.ok()) {
      // already logged above. just save it.
      last_error = std::move(status);
    }
  }
  return last_error;
}

util::Status SaveAllParameters() {
  util::Status status;
  util::Status last_error;
  for (ParameterBase* param : AllParameters()) {
    status = SaveParameter(param);
    if (!status.ok()) {
      // already logged above. just save it.
      last_error = std::move(status);
    }
  }
  return last_error;
}

}  // namespace config
}  // namespace tplp