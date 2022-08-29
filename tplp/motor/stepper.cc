#include "tplp/motor/stepper.h"

#include "hardware/gpio.h"
#include "hardware/pio.h"

namespace tplp {

StepperMotor* StepperMotor::Init(const Hardware& hw) {
  gpio_init(hw.a1);
  gpio_init(hw.a2);
  gpio_init(hw.b1);
  gpio_init(hw.b2);
  gpio_set_dir(hw.a1, GPIO_OUT);
  gpio_set_dir(hw.a2, GPIO_OUT);
  gpio_set_dir(hw.b1, GPIO_OUT);
  gpio_set_dir(hw.b2, GPIO_OUT);

  return new StepperMotor(hw);
}

StepperMotor::StepperMotor(const Hardware& hw) : hw_(hw) {}

void StepperMotor::Move(int32_t count) {
  //
}

void StepperMotor::Stop(bool brake) {
  //
}

void StepperMotor::RunPioTest() {
  gpio_put(hw_.a1, !gpio_get(hw_.a1));
  gpio_put(hw_.a2, !gpio_get(hw_.a2));
  gpio_put(hw_.b1, !gpio_get(hw_.b1));
  gpio_put(hw_.b2, !gpio_get(hw_.b2));
}

}  // namespace tplp