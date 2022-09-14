
#include "tplp/config/params_storage.h"

#include <map>

#include "absl/strings/str_cat.h"
#include "picolog/picolog.h"
#include "picolog/status_macros.h"
#include "tplp/config/params.h"
#include "tplp/fs/fs.h"

namespace tplp {
namespace config {
namespace {
static constexpr size_t kMaxSerializedValueLen = 512;
// Directory on the filesystem where parameter values shall be stored.
static const char* const kConfigDirectory = "params";

// Generates an 8.3 file path loosely inspired by the given parameter name.
static std::string GenerateFilePath(std::string_view param_name) {
  if (param_name.size() <= 8) {
    return absl::StrCat(kConfigDirectory, "/", param_name, ".prm");
  } else {
    return absl::StrCat(kConfigDirectory, "/", param_name.substr(0, 4), "~",
                        param_name.substr(param_name.size() - 4, 3), ".prm");
  }
}
}  // namespace

util::Status InitParameterStorage() {
  RETURN_IF_ERROR(fs::MkDir(kConfigDirectory));
  // Check for name collisions.
  std::map<std::string, ParameterBase*> names;
  for (ParameterBase* param : AllParameters()) {
    std::string path = GenerateFilePath(param->name());
    if (names.count(path)) {
      LOG(ERROR) << "File path collision: " << param->name() << " vs. "
                 << names[path]->name();
      return util::InternalError(
          "Generate parameter filename collision. Please improve the "
          "algorithm.");
    }
    names[path] = param;
  }
  return util::OkStatus();
}

util::Status SaveParameter(const ParameterBase* param) {
  char buf[kMaxSerializedValueLen];
  auto bytes_count = param->Serialize(buf, kMaxSerializedValueLen);
  if (!bytes_count.ok()) {
    LOG(ERROR) << "Error serializing parameter value " << param->name() << ": "
               << bytes_count.status();
    return util::InternalError("serialization error");
  }
  std::string file_path = GenerateFilePath(param->name());
  util::Status status = fs::SetContents(file_path.c_str(), buf, *bytes_count);
  if (!status.ok()) {
    LOG(ERROR) << "Error writing " << file_path << ": " << status;
    return status;
  }
  LOG(INFO) << "Saved parameter: " << param->name() << " = "
            << param->DebugString();
  return util::OkStatus();
}

util::Status LoadParameter(ParameterBase* param) {
  char buf[kMaxSerializedValueLen];
  std::string file_path = GenerateFilePath(param->name());
  auto bytes_read =
      fs::GetContents(file_path.c_str(), buf, kMaxSerializedValueLen);
  if (!bytes_read.ok()) {
    LOG(ERROR) << "Error reading " << file_path << ": " << bytes_read.status();
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