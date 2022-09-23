#include "tplp/pid.h"

#include "picolog/picolog.h"
#include "tplp/numbers.h"

namespace tplp {

PID::PID(const char* name, const config::Parameter<int32_t>* pn,
         const config::Parameter<int32_t>* pd,
         const config::Parameter<int32_t>* in,
         const config::Parameter<int32_t>* id,
         const config::Parameter<int32_t>* dn,
         const config::Parameter<int32_t>* dd,
         const config::Parameter<int32_t>* i_deadband)
    : name_(name),
      pn_(pn),
      pd_(pd),
      in_(in),
      id_(id),
      dn_(dn),
      dd_(dd),
      i_deadband_(i_deadband) {
  integral = 0;
  last_error = 0;
  last_nonzero_error = 0;
}

int32_t PID::Update(int32_t sp, int32_t pv, int32_t dt) {
  int32_t error = sp - pv;
  int32_t i_deadband = i_deadband_->Get();
//   if (i_deadband && error != 0 &&
//       util::signum(last_nonzero_error) != util::signum(error)) {
//     // we just crossed zero; clear the integral to prevent windup
//     integral = 0;
//   }
  if (std::abs(error) > i_deadband) {
    // trapezoid rule to update the integral
    // TODO: we might drop this /2 and just put it in the parameter
    integral += dt * (error + last_error) / 2;
  }
  int32_t proportional_term = (pn_->Get() * error) / pd_->Get();
  int32_t integral_term = (in_->Get() * integral) / id_->Get();
  int32_t derivative_term =
      (dn_->Get() * (error - last_error)) / (dd_->Get() * dt);

  int32_t op = proportional_term + integral_term + derivative_term;

  VLOG(1) << name_ << " sp=" << sp << " pv=" << pv << " P=" << proportional_term
          << " I=" << integral_term << " D=" << derivative_term
          << "; error = " << error << " op = " << op;
  last_error = error;
  if (error) last_nonzero_error = error;
  return op;
}

}  // namespace tplp