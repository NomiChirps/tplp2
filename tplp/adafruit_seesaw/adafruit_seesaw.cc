#include "tplp/adafruit_seesaw/adafruit_seesaw.h"

#include <cstring>

#include "picolog/picolog.h"
#include "picolog/status_macros.h"
#include "tplp/adafruit_seesaw/defs.h"

// bring in stuff from defs.h
using namespace ::tplp::seesaw;

namespace tplp {

/// Base

template <>
util::Status SeesawBase::Write<uint8_t>(uint8_t reg, uint8_t func,
                                        uint8_t data) {
  VLOG(1) << "Write<uint8_t> reg=0x" << std::hex << (int)reg << " func=0x"
          << (int)func << " data=0x" << (int)data;
  auto txn = i2c_.StartTransaction();
  uint8_t cmd[2] = {reg, func};
  RETURN_IF_ERROR(txn.Write(cmd, 2));
  RETURN_IF_ERROR(txn.WriteAndStop(&data, sizeof(data)));
  return util::OkStatus();
}

template <>
util::Status SeesawBase::Write<uint16_t>(uint8_t reg, uint8_t func,
                                         uint16_t data) {
  VLOG(1) << "Write<uint16_t> reg=0x" << std::hex << (int)reg << " func=0x"
          << (int)func << " data=0x" << data;
  auto txn = i2c_.StartTransaction();
  uint8_t cmd[2] = {reg, func};
  RETURN_IF_ERROR(txn.Write(cmd, 2));
  uint8_t buf[] = {(uint8_t)(data >> 8), (uint8_t)data};
  RETURN_IF_ERROR(txn.WriteAndStop(buf, sizeof(buf)));
  return util::OkStatus();
}

template <>
util::Status SeesawBase::Write<uint32_t>(uint8_t reg, uint8_t func,
                                         uint32_t data) {
  VLOG(1) << "Write<uint32_t> reg=0x" << std::hex << (int)reg << " func=0x"
          << (int)func << " data=0x" << data;
  auto txn = i2c_.StartTransaction();
  uint8_t cmd[2] = {reg, func};
  RETURN_IF_ERROR(txn.Write(cmd, 2));
  uint8_t buf[] = {(uint8_t)(data >> 24), (uint8_t)(data >> 16),
                   (uint8_t)(data >> 8), (uint8_t)data};
  RETURN_IF_ERROR(txn.WriteAndStop(buf, sizeof(buf)));
  return util::OkStatus();
}

template <>
util::Status SeesawBase::Write<int32_t>(uint8_t reg, uint8_t func,
                                        int32_t data) {
  return Write<uint32_t>(reg, func, static_cast<uint32_t>(data));
}

template <>
util::StatusOr<uint32_t> SeesawBase::Read<uint32_t>(uint8_t reg, uint8_t func) {
  VLOG(2) << "Read<uint32_t> reg=0x" << std::hex << (int)reg << " func=0x"
          << (int)func;
  auto txn = i2c_.StartTransaction();
  uint8_t cmd[2] = {reg, func};
  RETURN_IF_ERROR(txn.Write(cmd, 2));
  uint8_t buf[4];
  RETURN_IF_ERROR(txn.ReadAndStop(buf, sizeof(buf)));
  uint32_t val = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                 ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
  VLOG(1) << "Read<uint32_t> reg=0x" << std::hex << (int)reg << " func=0x"
          << (int)func << " value=0x" << val;
  return val;
}

template <>
util::StatusOr<int32_t> SeesawBase::Read<int32_t>(uint8_t reg, uint8_t func) {
  uint32_t val;
  ASSIGN_OR_RETURN(val, Read<uint32_t>(reg, func));
  return static_cast<int32_t>(val);
}

template <>
util::StatusOr<uint8_t> SeesawBase::Read<uint8_t>(uint8_t reg, uint8_t func) {
  VLOG(2) << "Read<uint8_t> reg=0x" << std::hex << (int)reg << " func=0x"
          << (int)func;
  auto txn = i2c_.StartTransaction();
  uint8_t cmd[2] = {reg, func};
  RETURN_IF_ERROR(txn.Write(cmd, 2));
  uint8_t val;
  RETURN_IF_ERROR(txn.ReadAndStop(&val, sizeof(val)));
  VLOG(1) << "Read<uint8_t> reg=0x" << std::hex << (int)reg << " func=0x"
          << (int)func << " value=0x" << (int)val;
  return val;
}

util::Status SeesawBase::Init() {
  // Make sure the device isn't still booting up
  util::Status status = i2c_.WaitForDevice(pdMS_TO_TICKS(1000));
  if (!status.ok()) {
    return util::NotFoundError("Seesaw device did not respond");
  }
  RETURN_IF_ERROR(SoftwareReset());
  // Wait for it to wake up again
  status = i2c_.WaitForDevice(pdMS_TO_TICKS(1000));
  if (!status.ok()) {
    return util::UnknownError(
        "Seesaw device did not wake up after software reset");
  }

  bool found = false;
  for (int attempts = 0; attempts < 10; ++attempts) {
    uint8_t hw_id;
    ASSIGN_OR_RETURN(hw_id,
                     Read<uint8_t>(SEESAW_STATUS_BASE, SEESAW_STATUS_HW_ID));
    VLOG(1) << "hw_id = 0x" << std::hex << (int)hw_id;
    if (hw_id == SEESAW_HW_ID_CODE_SAMD09 ||
        hw_id == SEESAW_HW_ID_CODE_TINY8X7) {
      found = true;
      break;
    } else {
      vTaskDelay(1);
    }
  }
  if (!found) {
    return util::UnknownError("Seesaw device has an unknown hardware id?");
  }
  return util::OkStatus();
}

util::StatusOr<uint32_t> SeesawBase::GetVersion() {
  VLOG(1) << "GetVersion()";
  return Read<uint32_t>(SEESAW_STATUS_BASE, SEESAW_STATUS_VERSION);
}

util::StatusOr<uint32_t> SeesawBase::GetOptions() {
  VLOG(1) << "GetOptions()";
  return Read<uint32_t>(SEESAW_STATUS_BASE, SEESAW_STATUS_OPTIONS);
}

util::Status SeesawBase::SoftwareReset() {
  VLOG(1) << "SoftwareReset()";
  return Write<uint8_t>(SEESAW_STATUS_BASE, SEESAW_STATUS_SWRST, 0xff);
}

util::Status SeesawBase::SetPinMode(uint8_t pin, PinMode mode) {
  if (pin >= 32) {
    return util::UnimplementedError("SetPinMode not implemented for pin >= 32");
  }
  return SetPinModes(1 << pin, mode);
}

util::Status SeesawBase::SetPinModes(uint32_t pins_mask, PinMode mode) {
  switch (mode) {
    // clang-format off
    case PinMode::OUTPUT:
      RETURN_IF_ERROR(Write<uint32_t>(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRSET_BULK, pins_mask));
      break;
    case PinMode::INPUT:
      RETURN_IF_ERROR(Write<uint32_t>(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, pins_mask));
      break;
    case PinMode::INPUT_PULLUP:
      RETURN_IF_ERROR(Write<uint32_t>(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, pins_mask));
      RETURN_IF_ERROR(Write<uint32_t>(SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, pins_mask));
      RETURN_IF_ERROR(Write<uint32_t>(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_SET, pins_mask));
      break;
    case PinMode::INPUT_PULLDOWN:
      RETURN_IF_ERROR(Write<uint32_t>(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, pins_mask));
      RETURN_IF_ERROR(Write<uint32_t>(SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, pins_mask));
      RETURN_IF_ERROR(Write<uint32_t>(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_CLR, pins_mask));
      break;
    // clang-format on
    default:
      return util::InternalError("switch fell through in " __FILE__);
  }
  return util::OkStatus();
}

util::Status SeesawBase::SetGpioInterruptsEnabled(uint32_t pins_mask,
                                                  bool enabled) {
  if (enabled) {
    return Write<uint32_t>(SEESAW_GPIO_BASE, SEESAW_GPIO_INTENSET, pins_mask);
  } else {
    return Write<uint32_t>(SEESAW_GPIO_BASE, SEESAW_GPIO_INTENCLR, pins_mask);
  }
}

util::StatusOr<bool> SeesawBase::DigitalRead(uint8_t pin) {
  if (pin >= 32) {
    return util::UnimplementedError(
        "DigitalRead for pin >= 32 not implemented");
  }
  return DigitalReadMask(1 << pin);
}

util::StatusOr<uint32_t> SeesawBase::DigitalReadMask(uint32_t pin_mask) {
  uint32_t state;
  ASSIGN_OR_RETURN(state, Read<uint32_t>(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK));
  return state & pin_mask;
}

/// Encoder

util::StatusOr<int32_t> SeesawEncoder::GetPosition() {
  return Read<int32_t>(SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION + index_);
}
util::StatusOr<int32_t> SeesawEncoder::GetDelta() {
  return Read<int32_t>(SEESAW_ENCODER_BASE, SEESAW_ENCODER_DELTA + index_);
}
util::Status SeesawEncoder::EnableInterruptPin() {
  return Write<uint8_t>(SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENSET + index_,
                        1);
}
util::Status SeesawEncoder::DisableInterruptPin() {
  return Write<uint8_t>(SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENCLR + index_,
                        1);
}
util::Status SeesawEncoder::SetPosition(int32_t pos) {
  // TODO: this is untested (not sure if int32_t write impl is correct)
  return Write<int32_t>(SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION + index_,
                        pos);
}

/// Neopixel

SeesawNeopixel::SeesawNeopixel(I2cDeviceHandle i2c, uint16_t num_leds,
                               uint8_t pin, Type type, bool khz800)
    : SeesawBase(i2c),
      num_leds_(num_leds),
      pin_(pin),
      type_(type),
      khz800_(khz800),
      r_offset_(1),
      g_offset_(0),
      b_offset_(2),
      w_offset_(1),
      pixels_(nullptr),
      num_bytes_(0) {}

util::Status SeesawNeopixel::Init() {
  VLOG(1) << "SeesawNeopixel::Init()";
  RETURN_IF_ERROR(UpdateType());
  VLOG(1) << "UpdateType OK";
  RETURN_IF_ERROR(UpdateLength(num_leds_));
  VLOG(1) << "UpdateLength OK";
  RETURN_IF_ERROR(SetPin(pin_));
  VLOG(1) << "SetPin OK";
  return util::OkStatus();
}

util::Status SeesawNeopixel::UpdateType() {
  VLOG(1) << "UpdateType() type=0x" << std::hex << (int)type_;
  uint8_t t = static_cast<uint8_t>(type_);
  bool old_three_bytes_per_pixel = (w_offset_ == r_offset_);  // false if RGBW

  w_offset_ = (t >> 6) & 0b11;  // See notes in header file
  r_offset_ = (t >> 4) & 0b11;  // regarding R/G/B/W offsets
  g_offset_ = (t >> 2) & 0b11;
  b_offset_ = t & 0b11;

  RETURN_IF_ERROR(
      Write<uint8_t>(SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_SPEED, khz800_));

  // If bytes-per-pixel has changed (and pixel data was previously
  // allocated), re-allocate to new size.  Will clear any data.
  if (pixels_) {
    bool new_three_bytes_per_pixel = (w_offset_ == r_offset_);
    if (new_three_bytes_per_pixel != old_three_bytes_per_pixel)
      RETURN_IF_ERROR(UpdateLength(num_leds_));
  }
  return util::OkStatus();
}
util::Status SeesawNeopixel::UpdateLength(uint16_t num_leds) {
  VLOG(1) << "UpdateLength(" << num_leds << ")";
  if (pixels_) free(pixels_);  // Free existing data (if any)

  // Allocate new data -- note: ALL PIXELS ARE CLEARED
  num_bytes_ = num_leds * ((w_offset_ == r_offset_) ? 3 : 4);
  if ((pixels_ = (uint8_t*)malloc(num_bytes_))) {
    std::memset(pixels_, 0, num_bytes_);
    num_leds_ = num_leds;
  } else {
    num_leds_ = num_bytes_ = 0;
    return util::ResourceExhaustedError("Failed to allocate pixel buffer");
  }

  return Write<uint16_t>(SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_BUF_LENGTH,
                         num_bytes_);
}
util::Status SeesawNeopixel::SetPin(uint8_t pin) {
  VLOG(1) << "SetPin(" << (int)pin << ")";
  RETURN_IF_ERROR(
      Write<uint8_t>(SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_PIN, pin));
  pin_ = pin;
  return util::OkStatus();
}

/// Adafruit 4991 Encoder+Neopixel breakout board

Adafruit4991::Adafruit4991(I2cDeviceHandle i2c)
    : SeesawBase(i2c),
      SeesawEncoder(i2c, /*index=*/0),
      SeesawNeopixel(i2c, /*num_leds=*/1, /*pin=*/kNeopixelPin,
                     SeesawNeopixel::Type::NEO_GRB, /*khz800=*/true) {}

util::Status Adafruit4991::Init() {
  // Extremely fiddly init sequence.
  VLOG(1) << "Adafruit4991: First reset";
  RETURN_IF_ERROR(this->SeesawBase::Init());
  VLOG(1) << "Adafruit4991: Second reset";
  RETURN_IF_ERROR(this->SeesawBase::Init());
  VLOG(1) << "Adafruit4991: Neopixel init";
  RETURN_IF_ERROR(this->SeesawNeopixel::Init());

  uint32_t version;
  // First time we get the version it always returns garbage.
  ASSIGN_OR_RETURN(version, GetVersion());
  ASSIGN_OR_RETURN(version, GetVersion());
  ASSIGN_OR_RETURN(version, GetVersion());
  ASSIGN_OR_RETURN(version, GetVersion());
  ASSIGN_OR_RETURN(version, GetVersion());
  ASSIGN_OR_RETURN(version, GetVersion());
  LOG(INFO) << "Adafruit4991 got version = 0x" << std::hex << version;
  uint32_t firmware_version = (version >> 16) & 0xffff;
  if (firmware_version != kExpectedFirmwareVersion) {
    LOG(ERROR) << "Wrong firmware version for Adafruit4991: expected "
               << kExpectedFirmwareVersion << ", got " << firmware_version;
    return util::FailedPreconditionError(
        "Target device has wrong firmware version?");
  }

  // The switch is wired to gpio 24 on the encoder breakout board.
  RETURN_IF_ERROR(SetPinMode(kSwitchPin, PinMode::INPUT_PULLUP));
  VLOG(1) << "SetPinMode OK";

  // Read initial position. Adafruit driver does this, and the
  // init sequence is extremely goddamn flaky, so...
  //RETURN_IF_ERROR(GetPosition().status());

  RETURN_IF_ERROR(SetGpioInterruptsEnabled(1 << kSwitchPin, true));
  VLOG(1) << "SetGpioInterruptsEnabled OK";
  RETURN_IF_ERROR(EnableInterruptPin());
  VLOG(1) << "EnableInterruptPin OK";
  return util::OkStatus();
}

util::StatusOr<bool> Adafruit4991::GetSwitchState() {
  return DigitalRead(kSwitchPin);
}

}  // namespace tplp