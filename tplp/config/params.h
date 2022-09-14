#ifndef TPLP_CONFIG_PARAMS_H_
#define TPLP_CONFIG_PARAMS_H_

#include <functional>

#include "picolog/status.h"
#include "picolog/statusor.h"

// Defines a persistent parameter. It contains a default value until
// Load() or LoadAllParameters() is used.
#define TPLP_PARAM(Type, name, default_value, help)    \
  extern ::tplp::config::Parameter<Type> PARAM_##name; \
  namespace tplp {}                                    \
  tplp::config::Parameter<Type> PARAM_##name { #name, default_value, help }

// Declares a persistent parameter which is defined in another module.
#define TPLP_DECLARE_PARAM(Type, name) \
  extern ::tplp::config::Parameter<Type> PARAM_##name;

namespace tplp {
namespace config {

// If non-OK, there was an error during static initialization.
const util::Status& DeferredInitError();

class ParameterBase {
 public:
  ParameterBase(const ParameterBase&) = delete;
  ParameterBase& operator=(const ParameterBase&) = delete;
  virtual ~ParameterBase();

  // Sets this parameter's value from a string representation, which should have
  // been previously generated by Serialize().
  virtual util::Status Parse(std::string_view str) = 0;

  // Writes a string representation of this parameter's value into the given
  // buffer. Returns the number of bytes written, which shall be at most `n`.
  virtual util::StatusOr<size_t> Serialize(char* buf, size_t n) const = 0;

  const char* name() const { return name_; }
  const char* help() const { return help_; }
  const char* file_path() const { return file_path_.c_str(); }

  virtual std::string DebugString() const = 0;

 protected:
  explicit ParameterBase(const char* name, const char* help);

  const char* const name_;
  const char* const help_;
  const std::string file_path_;
};

template <typename T>
class Parameter : public ParameterBase {
 public:
  explicit Parameter(const char* name, const T& default_value,
                     const char* help);

  const T& Get() const { return value_; }
  void Set(const T& new_value) { value_ = new_value; }

  util::Status Parse(std::string_view str) override;
  util::StatusOr<size_t> Serialize(char* buf, size_t n) const override;
  std::string DebugString() const override = 0;

 private:
  T value_;
};

// "Container" object which iterates over all registered parameters.
// Usage: for (ParameterBase* param : AllParameters()) { ... }
struct AllParameters {
  class iterator_t {
   public:
    explicit iterator_t(int index) : index_(index) {}
    ParameterBase* operator*();
    bool operator==(const iterator_t& other) const {
      return index_ == other.index_;
    }
    bool operator!=(const iterator_t& other) const {
      return index_ != other.index_;
    }
    iterator_t& operator++() {
      index_++;
      return *this;
    }

   private:
    int index_;
  };

  iterator_t begin() const;
  iterator_t end() const;
};

}  // namespace config
}  // namespace tplp

#endif  // TPLP_CONFIG_PARAMS_H_