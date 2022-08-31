#ifndef TPLP_MOTOR_STEPPER_H_
#define TPLP_MOTOR_STEPPER_H_

#include "hardware/pio.h"
#include "tplp/bus/types.h"

// Concept:
//
// PWM sin-cos microstepping. 8 bits resolution is probably more than enough.

// A full cycle, with B 90 degrees ahead of A:
//
// phase A1 A2 phase B1 B2
//   0   HI HI   +   HI LO
//   +   HI LO   0   HI HI
//   0   HI HI   -   LO HI
//   -   LO HI   0   HI HI
//
// Each level transition corresponds to a quarter-cycle of a sinusoid written
// to the PWM peripheral. In more detail:
//
// phase A1  A2  phase B1  B2
//   0   255 255   +   255 0
//   ^   255 dn    v   255 up
//   +   255 0     0   255 255
//   v   255 up    v   dn  255
//   0   255 255   -   0   255
//   v   dn  255   ^   up  255
//   -   0   255   0   255 255
//   ^   up  255   ^   255 dn
//   wrap around   wrap around
//
// Storage cost options:
// * single quarter-wave: 64 bytes
// * one entire column of the table: 256 bytes (260?)
//
// Actually, one column should be all we need, if cleverly offsetting the
// DMA read addresses into it. The question is how many PWM slices are needed.
// If we can make do with one, then we can easily change speed on-the-fly!
//
// A slice has the following relevant controls:
//   - counter wrap value
//   - counter compare value for channel A
//   - counter compare value for channel B
//   - invert output A
//   - invert output B
//
// PWM slice 1 controls A1, A2 targeting TB6612's max pwm frequency = 100kHz.
// PWM slice 2 controls B1, B2 at the same frequency.
// DMA channel 1 writes waveform to PWM slice 1, paced by PIO SM 1 TX
// DMA channel 2 writes waveform to PWM slice 2, paced by PIO SM 1 RX
//
// PIO SM 1 has autopull and autopush both enabled with a threshold of 1,
// and periodically executes "OUT ISR, 1" to generate simultaneous DREQs
// on the TX and RX.
//
// XXX: Fuck, that won't work; autopush/autopull will stall if no one
//      services the FIFOs.
//
// Now we can write SMx_CLOCKDIV to change the step speed on the fly,
// theoretically without any glitches! Another way to control speed
// would be to force-write one of the scratch registers, which the program
// would use to reset its counter each iteration.
//
// Wanna go ~super~ fancy? DMA a minimum-jerk curve into SMx_CLOCKDIV.

// ok maybe the PWM peripheral won't work directly.
// can we use PIO effectively instead? almost certainly yes.
// sleep on it please. seriously,  GO TO BED

namespace tplp {

// Designed to drive a bipolar stepper motor via the TB6612FNG motor driver,
// which short-brakes when both ends of a coil are set high.
class StepperMotor {
 public:
  struct Hardware {
    // a1, a2, b1, b2 must be consecutive GPIO numbers.
    gpio_pin_t a1;
    gpio_pin_t a2;
    gpio_pin_t b1;
    gpio_pin_t b2;

    int pwm_freq_hz;
  };

  static StepperMotor* Init(PIO pio, const Hardware& hw);

  void RunPioTest();

  // Step forward or backward by a certain number of microsteps.
  void Move(int32_t count);

  // Set the speed of the next move, or change the speed of the move in progress.
  void SetSpeed(/*TODO*/);

  enum class StopType {
    // Motor coils remain energized to whatever state they were in
    // at the time of the stop. High holding torque.
    HOLD,
    // Motor coils are shorted together. Medium-low holding torque.
    SHORT_BRAKE,
    // Motor outputs are set to high impedance. Lowest holding torque.
    FREEWHEEL
  };
  // Abort any move in progress and apply the given stop type.
  void Stop(StopType type);

 private:
  explicit StepperMotor();
  void InitCommands();
  void SendCommand(uint32_t);

 private:
  PIO pio_;
  int sm_;
  // TX FIFO register.
  volatile uint32_t* txf_;

  int offset_;
  // Iteration count.
  uint16_t pwm_period_;
  Hardware hw_;

  std::vector<uint32_t> commands_;
  uint32_t shortbrake_command_;
  uint32_t freewheel_command_;

 private:
  // Not copyable or moveable.
  StepperMotor(const StepperMotor&) = delete;
  StepperMotor& operator=(const StepperMotor&) = delete;
};

}  // namespace tplp

#endif  // TPLP_MOTOR_STEPPER_H_