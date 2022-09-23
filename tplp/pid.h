#ifndef TPLP_PID_H_
#define TPLP_PID_H_

#include "tplp/config/params.h"

namespace tplp {

// PID controller.
class PID {
 public:
  PID(const char* name, const config::Parameter<int32_t>* pn,
      const config::Parameter<int32_t>* pd,
      const config::Parameter<int32_t>* in,
      const config::Parameter<int32_t>* id,
      const config::Parameter<int32_t>* dn,
      const config::Parameter<int32_t>* dd,
      const config::Parameter<int32_t>* i_deadband);

  // Runs a loop iteration. Returns the PID output.
  // sp: setpoint,
  // pv: process variable,
  // dt: timestep since last update
  int32_t Update(int32_t sp, int32_t pv, int32_t dt);

 private:
 const char* const name_;
  const config::Parameter<int32_t>* const pn_;
  const config::Parameter<int32_t>* const pd_;
  const config::Parameter<int32_t>* const in_;
  const config::Parameter<int32_t>* const id_;
  const config::Parameter<int32_t>* const dn_;
  const config::Parameter<int32_t>* const dd_;
  const config::Parameter<int32_t>* const i_deadband_;

  int32_t integral;
  int32_t last_error;
  int32_t last_nonzero_error;
};

}  // namespace tplp

#endif  // TPLP_PID_H_