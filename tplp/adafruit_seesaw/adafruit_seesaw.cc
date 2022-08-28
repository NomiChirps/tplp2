#include "tplp/adafruit_seesaw/adafruit_seesaw.h"

#include "picolog/picolog.h"
#include "picolog/status_macros.h"
#include "tplp/adafruit_seesaw/defs.h"

// bring in stuff from defs.h
using namespace ::tplp::seesaw;

namespace tplp {

Seesaw::Seesaw(I2cDeviceHandle i2c) : SeesawBase(i2c) {}

util::StatusOr<Seesaw*> Seesaw::Init(I2cDeviceHandle i2c_device) {
  auto self = std::unique_ptr<Seesaw>(new Seesaw(i2c_device));
  RETURN_IF_ERROR(self->SoftwareReset());
  LOG(INFO) << "Adafruit Seesaw device at " << i2c_device << " reset.";
  return self.release();
}

template <>
util::Status SeesawBase::Write<uint8_t>(uint8_t reg, uint8_t func,
                                        uint8_t data) {
  auto txn = i2c_.StartTransaction();
  uint8_t buf1[2] = {reg, func};
  RETURN_IF_ERROR(txn.Write(buf1, 2));
  RETURN_IF_ERROR(txn.WriteAndStop(&data, sizeof(data)));
  return util::OkStatus();
}

template <>
util::StatusOr<int32_t> SeesawBase::Read<int32_t>(uint8_t reg, uint8_t func) {
  auto txn = i2c_.StartTransaction();
  uint8_t buf1[2] = {reg, func};
  RETURN_IF_ERROR(txn.Write(buf1, 2));
  uint8_t buf[4];
  RETURN_IF_ERROR(txn.ReadAndStop(buf, sizeof(buf)));
  return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
         ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
}

util::Status Seesaw::SoftwareReset() {
  return Write<uint8_t>(SEESAW_STATUS_BASE, SEESAW_STATUS_SWRST, 0xff);
}

std::unique_ptr<SeesawEncoder> Seesaw::NewEncoder(uint8_t index) {
  LOG(INFO) << "Initializing encoder " << index << " on Seesaw " << i2c_;
  return std::unique_ptr<SeesawEncoder>(
      CHECK_NOTNULL(new SeesawEncoder(i2c_, index)));
}

/// Encoder

util::StatusOr<int32_t> SeesawEncoder::GetPosition() {
  return Read<int32_t>(SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION + index_);
}
util::StatusOr<int32_t> SeesawEncoder::GetDelta() {
  return Read<int32_t>(SEESAW_ENCODER_BASE, SEESAW_ENCODER_DELTA + index_);
}
util::Status SeesawEncoder::EnableInterrupt() {
  return Write<uint8_t>(SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENSET + index_,
                        1);
}
util::Status SeesawEncoder::DisableInterrupt() {
  return Write<uint8_t>(SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENCLR + index_,
                        1);
}
util::Status SeesawEncoder::SetPosition(int32_t pos) {
  return Write<int32_t>(SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION + index_,
                        pos);
}

}  // namespace tplp