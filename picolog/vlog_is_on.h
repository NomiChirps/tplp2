#include <array>
#include <string>
#include <string_view>

namespace picolog {
namespace detail {
struct VmoduleConfig_1 {
  std::string_view module;
  int verboselevel;

  constexpr bool operator==(const VmoduleConfig_1& other) const {
    return module == other.module && verboselevel == other.verboselevel;
  }
};

typedef std::array<VmoduleConfig_1, 8> VmoduleConfig;

constexpr int ParseInt(std::string_view str) {
  int expt = 1;
  int acc = 0;
  for (int i = str.size() - 1; i >= 0; --i) {
    int digit = str[i] - '0';
    if (digit < 0 || digit > 9) {
      // skip it i guess
      continue;
    }
    acc += digit * expt;
    expt *= 10;
  }
  return acc;
}

constexpr VmoduleConfig_1 ParseVmoduleConfig_1(std::string_view vmodule) {
  size_t equals = vmodule.find_first_of('=');
  if (equals == std::string_view::npos) {
    return {"", 0};
  }
  return {vmodule.substr(0, equals), ParseInt(vmodule.substr(equals))};
}

constexpr VmoduleConfig ParseVmoduleConfig(std::string_view vmodule_def) {
  VmoduleConfig config({});
  size_t vmodule_start = 0;
  size_t vmodule_end = std::string_view::npos;
  for (size_t i = 0;; ++i) {
    vmodule_end = vmodule_def.find_first_of(',', vmodule_start);
    std::string_view vmodule =
        vmodule_def.substr(vmodule_start, vmodule_end == std::string_view::npos
                                              ? std::string_view::npos
                                              : vmodule_end - vmodule_start);
    config[i] = ParseVmoduleConfig_1(vmodule);
    if (vmodule_end == std::string_view::npos) break;

    vmodule_start = vmodule_end + 1;  // skip the comma
  }
  return config;
}

static constexpr VmoduleConfig kVmoduleConfig(
    ParseVmoduleConfig(PICOLOG_VMODULE));

constexpr std::string_view FilenameToModuleName(std::string_view file) {
  // Strip everything after and including the last '.' (file extension)
  size_t dot = file.find_last_of('.');
  if (dot != std::string_view::npos) {
    file.remove_suffix(file.size() - dot);
  }
  // Strip everything up to and including the last slash (in case the file path
  // was absolute).
  size_t slash = file.find_last_of("\\/");
  if (slash != std::string_view::npos) {
    file.remove_prefix(slash + 1);
  }
  return file;
}

constexpr bool VlogIsOnImpl(std::string_view file, int verboselevel) {
  // 0 turns off VLOG entirely, even VLOG(0).
  if (verboselevel <= 0) return false;

  std::string_view module = FilenameToModuleName(file);
  for (size_t i = 0; i < kVmoduleConfig.size(); ++i) {
    if (kVmoduleConfig[i].module == module) {
      return verboselevel <= kVmoduleConfig[i].verboselevel;
    }
  }
  return false;
}
}  // namespace detail
}  // namespace picolog

// TODO: make it also possible to mute VLOGs at runtime?
#define VLOG_IS_ON(verboselevel) \
  ::picolog::detail::VlogIsOnImpl(__FILE__, verboselevel)
