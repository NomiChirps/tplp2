#ifndef TPLP_CONFIG_PARAMS_STORAGE_H_
#define TPLP_CONFIG_PARAMS_STORAGE_H_

#include "picolog/status.h"
#include "tplp/config/params.h"

namespace tplp {
namespace config {

// Saves a parameter's current value to the filesystem.
util::Status SaveParameter(const ParameterBase* param);
// Loads a parameter's value from the filesystem, replacing the current value.
util::Status LoadParameter(ParameterBase* param);

// Saves all parameters to the filesystem with their current values.
util::Status SaveAllParameters();
// Loads all parameters values from the filesystem, replacing their current
// values.
util::Status LoadAllParameters();

}  // namespace config
}  // namespace tplp

#endif  // TPLP_CONFIG_PARAMS_STORAGE_H_
