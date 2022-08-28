#ifndef TPLP_ADAFRUIT_SEESAW_DEFS_H_
#define TPLP_ADAFRUIT_SEESAW_DEFS_H_

#include <cstdint>

namespace tplp {
namespace seesaw {

// TODO rename the rest of these to be a little less verbose/redundant

// The module base addresses for different seesaw modules.
enum class ModuleBaseAddress : uint8_t {
  STATUS = 0x00,
  GPIO = 0x01,
  SERCOM0 = 0x02,

  TIMER = 0x08,
  ADC = 0x09,
  DAC = 0x0A,
  INTERRUPT = 0x0B,
  DAP = 0x0C,
  EEPROM = 0x0D,
  NEOPIXEL = 0x0E,
  TOUCH = 0x0F,
  KEYPAD = 0x10,
  ENCODER = 0x11,
  SPECTRUM = 0x12,
};

/** GPIO module function address registers
 */
enum class GpioFunction : uint8_t {
  DIRSET_BULK = 0x02,
  DIRCLR_BULK = 0x03,
  BULK = 0x04,
  BULK_SET = 0x05,
  BULK_CLR = 0x06,
  BULK_TOGGLE = 0x07,
  INTENSET = 0x08,
  INTENCLR = 0x09,
  INTFLAG = 0x0A,
  PULLENSET = 0x0B,
  PULLENCLR = 0x0C,
};

/** status module function address registers
 */
enum class StatusFunction : uint8_t {
  HW_ID = 0x01,
  VERSION = 0x02,
  OPTIONS = 0x03,
  TEMP = 0x04,
  SWRST = 0x7F,
};

/** timer module function address registers
 */
enum class TimerFunction : uint8_t {
  STATUS = 0x00,
  PWM = 0x01,
  FREQ = 0x02,
};

/** ADC module function address registers
 */
enum class AdcFunction : uint8_t {
  STATUS = 0x00,
  INTEN = 0x02,
  INTENCLR = 0x03,
  WINMODE = 0x04,
  WINTHRESH = 0x05,
  CHANNEL_OFFSET = 0x07,
};

/** Sercom module function address registers
 */
enum class SercomFunction : uint8_t {
  STATUS = 0x00,
  INTEN = 0x02,
  INTENCLR = 0x03,
  BAUD = 0x04,
  DATA = 0x05,
};

/** neopixel module function address registers
 */
enum class NeopixelFunction : uint8_t {
  STATUS = 0x00,
  PIN = 0x01,
  SPEED = 0x02,
  BUF_LENGTH = 0x03,
  BUF = 0x04,
  SHOW = 0x05,
};

/** touch module function address registers
 */
enum class TouchFunction : uint8_t {
  CHANNEL_OFFSET = 0x10,
};

/** keypad module function address registers
 */
enum class KeypadFunction : uint8_t {
  STATUS = 0x00,
  EVENT = 0x01,
  INTENSET = 0x02,
  INTENCLR = 0x03,
  COUNT = 0x04,
  FIFO = 0x10,
};

/** keypad module edge definitions
 */
enum class KeypadEdge : uint8_t {
  HIGH = 0,
  LOW,
  FALLING,
  RISING,
};

/** encoder module function address registers
 */
enum class EncoderFunction : uint8_t {
  STATUS = 0x00,
  INTENSET = 0x10,
  INTENCLR = 0x20,
  POSITION = 0x30,
  DELTA = 0x40,
};

/** Audio spectrum module function address registers
 */
enum class SpectrumFunction : uint8_t {
  RESULTS_LOWER = 0x00,  // Audio spectrum bins 0-31
  RESULTS_UPPER = 0x01,  // Audio spectrum bins 32-63
  // If some future device supports a larger spectrum, can add additional
  // "bins" working upward from here. Configurable setting registers then
  // work downward from the top to avoid collision between spectrum bins
  // and configurables.
  CHANNEL = 0xFD,
  RATE = 0xFE,
  STATUS = 0xFF,
};

static constexpr uint8_t ADC_INPUT_0_PIN = 2;  ///< default ADC input pin
static constexpr uint8_t ADC_INPUT_1_PIN = 3;  ///< default ADC input pin
static constexpr uint8_t ADC_INPUT_2_PIN = 4;  ///< default ADC input pin
static constexpr uint8_t ADC_INPUT_3_PIN = 5;  ///< default ADC input pin

static constexpr uint8_t PWM_0_PIN = 4;  ///< default PWM output pin
static constexpr uint8_t PWM_1_PIN = 5;  ///< default PWM output pin
static constexpr uint8_t PWM_2_PIN = 6;  ///< default PWM output pin
static constexpr uint8_t PWM_3_PIN = 7;  ///< default PWM output pin

static constexpr uint8_t INPUT_PULLDOWN = 0x03;

/*=========================================================================*/

static constexpr uint8_t SEESAW_HW_ID_CODE_SAMD09 =
    0x55;  ///< seesaw HW ID code for SAMD09

static constexpr uint8_t SEESAW_HW_ID_CODE_TINY8X7 =
    0x87;  ///< seesaw HW ID code for ATtiny817

static constexpr uint8_t SEESAW_EEPROM_I2C_ADDR =
    0x3F;  ///< EEPROM address of i2c address to start up with (for devices that
           ///< support this feature)

/** raw key event stucture for keypad module */
union key_event_raw_t {
  struct {
    uint8_t EDGE : 2;  ///< the edge that was triggered
    uint8_t NUM : 6;   ///< the event number
  } bit;               ///< bitfield format
  uint8_t reg;         ///< register format
};

/** extended key event stucture for keypad module */
union key_event_t {
  struct {
    uint8_t EDGE : 2;   ///< the edge that was triggered
    uint16_t NUM : 14;  ///< the event number
  } bit;                ///< bitfield format
  uint16_t reg;         ///< register format
};

/** key state struct that will be written to seesaw chip keypad module */
union key_state_t {
  struct {
    uint8_t STATE : 1;   ///< the current state of the key
    uint8_t ACTIVE : 4;  ///< the registered events for that key
  } bit;                 ///< bitfield format
  uint8_t reg;           ///< register format
};

}  // namespace seesaw
}  // namespace tplp

#endif  // TPLP_ADAFRUIT_SEESAW_DEFS_H_