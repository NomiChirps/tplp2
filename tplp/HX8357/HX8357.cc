#include "tplp/HX8357/HX8357.h"

#include <chrono>
#include <iomanip>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "hardware/gpio.h"
#include "picolog/picolog.h"
#include "tplp/time.h"

using std::chrono_literals::operator""ms;

#define CHECK_OK(expr) CHECK_EQ((expr), SpiTransaction::Result::OK)

// - see commands and init sequence at
//   https://github.com/adafruit/Adafruit_HX8357_Library/blob/master/Adafruit_HX8357.h
// - read diagnostics as in
//   https://github.com/adafruit/Adafruit_HX8357_Library/blob/master/examples/graphicstest/graphicstest.ino
// - intermediate class; see especially startWrite()/endWrite()
//   https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Adafruit_SPITFT.h
// - see high-level (non-virtual) entry points at
//   https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Adafruit_GFX.h
namespace tplp {
namespace {

constexpr int HX8357_TFTWIDTH = 320;   ///< 320 pixels wide
constexpr int HX8357_TFTHEIGHT = 480;  ///< 480 pixels tall

constexpr uint8_t HX8357_NOP = 0x00;      ///< No op
constexpr uint8_t HX8357_SWRESET = 0x01;  ///< software reset
constexpr uint8_t HX8357_RDDID = 0x04;    ///< Read ID
constexpr uint8_t HX8357_RDDST = 0x09;    ///< (unknown)

constexpr uint8_t HX8357_RDPOWMODE = 0x0A;  ///< Read power mode Read power mode
constexpr uint8_t HX8357_RDMADCTL = 0x0B;   ///< Read MADCTL
constexpr uint8_t HX8357_RDCOLMOD = 0x0C;   ///< Column entry mode
constexpr uint8_t HX8357_RDDIM = 0x0D;      ///< Read display image mode
constexpr uint8_t HX8357_RDDSDR = 0x0F;     ///< Read dosplay signal mode

constexpr uint8_t HX8357_SLPIN = 0x10;   ///< Enter sleep mode
constexpr uint8_t HX8357_SLPOUT = 0x11;  ///< Exit sleep mode
constexpr uint8_t HX8357B_PTLON = 0x12;  ///< Partial mode on
constexpr uint8_t HX8357B_NORON = 0x13;  ///< Normal mode

constexpr uint8_t HX8357_INVOFF = 0x20;   ///< Turn off invert
constexpr uint8_t HX8357_INVON = 0x21;    ///< Turn on invert
constexpr uint8_t HX8357_DISPOFF = 0x28;  ///< Display on
constexpr uint8_t HX8357_DISPON = 0x29;   ///< Display off

constexpr uint8_t HX8357_CASET = 0x2A;  ///< Column addr set
constexpr uint8_t HX8357_PASET = 0x2B;  ///< Page addr set
constexpr uint8_t HX8357_RAMWR = 0x2C;  ///< Write VRAM
constexpr uint8_t HX8357_RAMRD = 0x2E;  ///< Read VRAm

constexpr uint8_t HX8357B_PTLAR = 0x30;    ///< (unknown)
constexpr uint8_t HX8357_TEON = 0x35;      ///< Tear enable on
constexpr uint8_t HX8357_TEARLINE = 0x44;  ///< (unknown)
constexpr uint8_t HX8357_MADCTL = 0x36;    ///< Memory access control
constexpr uint8_t HX8357_COLMOD = 0x3A;    ///< Color mode

constexpr uint8_t HX8357_SETOSC = 0xB0;       ///< Set oscillator
constexpr uint8_t HX8357_SETPWR1 = 0xB1;      ///< Set power control
constexpr uint8_t HX8357B_SETDISPLAY = 0xB2;  ///< Set display mode
constexpr uint8_t HX8357_SETRGB = 0xB3;       ///< Set RGB interface
constexpr uint8_t HX8357D_SETCOM = 0xB6;      ///< Set VCOM voltage

constexpr uint8_t HX8357B_SETDISPMODE = 0xB4;  ///< Set display mode
constexpr uint8_t HX8357D_SETCYC = 0xB4;       ///< Set display cycle reg
constexpr uint8_t HX8357B_SETOTP = 0xB7;       ///< Set OTP memory
constexpr uint8_t HX8357D_SETC = 0xB9;         ///< Enable extension command

constexpr uint8_t HX8357B_SET_PANEL_DRIVING = 0xC0;  ///< Set panel drive mode
constexpr uint8_t HX8357D_SETSTBA = 0xC0;            ///< Set source option
constexpr uint8_t HX8357B_SETDGC = 0xC1;             ///< Set DGC settings
constexpr uint8_t HX8357B_SETID = 0xC3;              ///< Set ID
constexpr uint8_t HX8357B_SETDDB = 0xC4;             ///< Set DDB
constexpr uint8_t HX8357B_SETDISPLAYFRAME = 0xC5;    ///< Set display frame
constexpr uint8_t HX8357B_GAMMASET = 0xC8;           ///< Set Gamma correction
constexpr uint8_t HX8357B_SETCABC = 0xC9;            ///< Set CABC
constexpr uint8_t HX8357_SETPANEL = 0xCC;            ///< Set Panel

constexpr uint8_t HX8357B_SETPOWER = 0xD0;      ///< Set power control
constexpr uint8_t HX8357B_SETVCOM = 0xD1;       ///< Set VCOM
constexpr uint8_t HX8357B_SETPWRNORMAL = 0xD2;  ///< Set power normal

constexpr uint8_t HX8357B_RDID1 = 0xDA;  ///< Read ID #1
constexpr uint8_t HX8357B_RDID2 = 0xDB;  ///< Read ID #2
constexpr uint8_t HX8357B_RDID3 = 0xDC;  ///< Read ID #3
constexpr uint8_t HX8357B_RDID4 = 0xDD;  ///< Read ID #4

constexpr uint8_t HX8357D_SETGAMMA = 0xE0;  ///< Set Gamma

constexpr uint8_t HX8357B_SETGAMMA = 0xC8;         ///< Set Gamma
constexpr uint8_t HX8357B_SETPANELRELATED = 0xE9;  ///< Set panel related

constexpr uint8_t MADCTL_MY = 0x80;   ///< Bottom to top
constexpr uint8_t MADCTL_MX = 0x40;   ///< Right to left
constexpr uint8_t MADCTL_MV = 0x20;   ///< Reverse Mode
constexpr uint8_t MADCTL_ML = 0x10;   ///< LCD refresh Bottom to top
constexpr uint8_t MADCTL_RGB = 0x00;  ///< Red-Green-Blue pixel order
constexpr uint8_t MADCTL_BGR = 0x08;  ///< Blue-Green-Red pixel order
constexpr uint8_t MADCTL_MH = 0x04;   ///< LCD refresh right to left

// Color definitions
constexpr int HX8357_BLACK = 0x0000;    ///< BLACK color for drawing graphics
constexpr int HX8357_BLUE = 0x001F;     ///< BLUE color for drawing graphics
constexpr int HX8357_RED = 0xF800;      ///< RED color for drawing graphics
constexpr int HX8357_GREEN = 0x07E0;    ///< GREEN color for drawing graphics
constexpr int HX8357_CYAN = 0x07FF;     ///< CYAN color for drawing graphics
constexpr int HX8357_MAGENTA = 0xF81F;  ///< MAGENTA color for drawing graphics
constexpr int HX8357_YELLOW = 0xFFE0;   ///< YELLOW color for drawing graphics
constexpr int HX8357_WHITE = 0xFFFF;    ///< WHITE color for drawing graphics

static constexpr uint8_t kInitB[] = {
    HX8357B_SETPOWER,
    3,
    0x44,
    0x41,
    0x06,
    HX8357B_SETVCOM,
    2,
    0x40,
    0x10,
    HX8357B_SETPWRNORMAL,
    2,
    0x05,
    0x12,
    HX8357B_SET_PANEL_DRIVING,
    5,
    0x14,
    0x3b,
    0x00,
    0x02,
    0x11,
    HX8357B_SETDISPLAYFRAME,
    1,
    0x0c,  // 6.8mhz
    HX8357B_SETPANELRELATED,
    1,
    0x01,  // BGR
    0xEA,
    3,  // seq_undefined1, 3 args
    0x03,
    0x00,
    0x00,
    0xEB,
    4,  // undef2, 4 args
    0x40,
    0x54,
    0x26,
    0xdb,
    HX8357B_SETGAMMA,
    12,
    0x00,
    0x15,
    0x00,
    0x22,
    0x00,
    0x08,
    0x77,
    0x26,
    0x66,
    0x22,
    0x04,
    0x00,
    HX8357_MADCTL,
    1,
    0xC0,
    HX8357_COLMOD,
    1,
    0x55,
    HX8357_PASET,
    4,
    0x00,
    0x00,
    0x01,
    0xDF,
    HX8357_CASET,
    4,
    0x00,
    0x00,
    0x01,
    0x3F,
    HX8357B_SETDISPMODE,
    1,
    0x00,  // CPU (DBI) and internal oscillation ??
    HX8357_SLPOUT,
    0x80 + 120 / 5,  // Exit sleep, then delay 120 ms
    HX8357_DISPON,
    0x80 + 10 / 5,  // Main screen turn on, delay 10 ms
    0               // END OF COMMAND LIST
};

static constexpr uint8_t kInitD[] = {
    HX8357_SWRESET,
    0x80 + 100 / 5,  // Soft reset, then delay 10 ms
    HX8357D_SETC,
    3,
    0xFF,
    0x83,
    0x57,
    0xFF,
    0x80 + 500 / 5,  // No command, just delay 300 ms
    HX8357_SETRGB,
    4,
    0x80,
    0x00,
    0x06,
    0x06,  // 0x80 enables SDO pin (0x00 disables)
    HX8357D_SETCOM,
    1,
    0x25,  // -1.52V
    HX8357_SETOSC,
    1,
    0x68,  // Normal mode 70Hz, Idle mode 55 Hz
    HX8357_SETPANEL,
    1,
    0x05,  // BGR, Gate direction swapped
    HX8357_SETPWR1,
    6,
    0x00,  // Not deep standby
    0x15,  // BT
    0x1C,  // VSPR
    0x1C,  // VSNR
    0x83,  // AP
    0xAA,  // FS
    HX8357D_SETSTBA,
    6,
    0x50,  // OPON normal
    0x50,  // OPON idle
    0x01,  // STBA
    0x3C,  // STBA
    0x1E,  // STBA
    0x08,  // GEN
    HX8357D_SETCYC,
    7,
    0x02,  // NW 0x02
    0x40,  // RTN
    0x00,  // DIV
    0x2A,  // DUM
    0x2A,  // DUM
    0x0D,  // GDON
    0x78,  // GDOFF
    HX8357D_SETGAMMA,
    34,
    0x02,
    0x0A,
    0x11,
    0x1d,
    0x23,
    0x35,
    0x41,
    0x4b,
    0x4b,
    0x42,
    0x3A,
    0x27,
    0x1B,
    0x08,
    0x09,
    0x03,
    0x02,
    0x0A,
    0x11,
    0x1d,
    0x23,
    0x35,
    0x41,
    0x4b,
    0x4b,
    0x42,
    0x3A,
    0x27,
    0x1B,
    0x08,
    0x09,
    0x03,
    0x00,
    0x01,
    HX8357_COLMOD,
    1,
    0x55,  // 16 bit
    HX8357_MADCTL,
    1,
    0xC0,
    HX8357_TEON,
    1,
    0x00,  // TW off
    HX8357_TEARLINE,
    2,
    0x00,
    0x02,
    HX8357_SLPOUT,
    0x80 + 150 / 5,  // Exit Sleep, then delay 150 ms
    HX8357_DISPON,
    0x80 + 50 / 5,  // Main screen turn on, delay 50 ms
    0,              // END OF COMMAND LIST
};

}  // namespace

enum class HX8357::DisplayType { HX8357D = 0xD, HX8357B = 0xB };

HX8357::HX8357(DisplayType type, SpiManager* spi, gpio_pin_t cs, gpio_pin_t dc)
    : type_(type), spi_(spi), cs_(cs), dc_(dc) {}

HX8357B::HX8357B(SpiManager* spi, gpio_pin_t cs, gpio_pin_t dc)
    : HX8357(DisplayType::HX8357B, spi, cs, dc) {}

HX8357D::HX8357D(SpiManager* spi, gpio_pin_t cs, gpio_pin_t dc)
    : HX8357(DisplayType::HX8357D, spi, cs, dc) {}

void HX8357::Begin() {
  spi_device_ = spi_->AddDevice(cs_, "HX8357");
  width_ = HX8357_TFTWIDTH;  // Screen dimensions for default rotation 0
  height_ = HX8357_TFTHEIGHT;
  gpio_init(dc_);
  gpio_set_dir(dc_, GPIO_OUT);
  SendInitSequence();
}

void HX8357::SendInitSequence() {
  LOG(INFO) << "HX8357 init sequence start";
  const uint8_t* addr = (type_ == DisplayType::HX8357B) ? kInitB : kInitD;
  uint8_t cmd, x, numArgs;
  while ((cmd = *(addr++)) > 0) {  // '0' command ends list
    x = *(addr++);
    numArgs = x & 0x7F;
    if (cmd != 0xFF) {  // '255' is ignored
      if (x & 0x80) {   // If high bit set, numArgs is a delay time
        SendCommand(cmd, nullptr, 0);
      } else {
        SendCommand(cmd, addr, numArgs);
        addr += numArgs;
      }
    }
    if (x & 0x80) {  // If high bit set...
      // numArgs is actually a delay time (5ms units)
      int t = as_ticks_ceil(std::chrono::milliseconds(numArgs * 5));
      VLOG(1) << "Sleep at least " << (numArgs * 5)
              << "ms <= " << t * portTICK_PERIOD_MS << "ms as ticks";
      vTaskDelay(t);
    }
  }
}

void HX8357::SendCommand(uint8_t command, const uint8_t* data, uint8_t len) {
  VLOG(1) << std::hex << std::setw(2) << std::setfill('0') << "SendCommand(0x"
          << (int)command << ", [" << std::dec << (int)len << " bytes])";
  // DC is low for a command byte, then high for the data.
  SpiTransaction txn = spi_device_->StartTransaction();
  gpio_put(dc_, 0);
  CHECK_OK(txn.TransferBlocking({
      .tx_buf = &command,
      .len = 1,
  }));
  if (data && len) {
    gpio_put(dc_, 1);
    CHECK_OK(txn.Transfer({
        .tx_buf = data,
        .len = len,
    }));
  }
}

uint8_t HX8357::RDDSDR() {
  SpiTransaction txn = spi_device_->StartTransaction();
  VLOG(1) << "Issuing RDDSDR";
  const uint8_t command = HX8357_RDDSDR;
  // Datasheet claims that the first byte of the reply should be a "dummy read",
  // with the actual result being in the 2nd byte. I have determined that that
  // was a lie.
  uint8_t result = 0;
  gpio_put(dc_, 0);
  CHECK_OK(txn.Transfer({
      .tx_buf = &command,
      .len = 1,
  }));
  CHECK_OK(txn.Transfer({
      .rx_buf = &result,
      .len = 1,
  }));
  gpio_put(dc_, 1);
  txn.Flush();
  VLOG(1) << std::hex << std::setw(2) << std::setfill('0')
          << "RDDSDR response 0x" << (int)result;
  return result;
}

bool HX8357::SelfTest() {
  uint8_t dsdr1 = RDDSDR();
  SendCommand(HX8357_SLPOUT);
  vTaskDelay(as_ticks_ceil(120ms));
  uint8_t dsdr2 = RDDSDR();

  // DSDR register protocol is that a successful self-test of a module inverts
  // the corresponding bit.
  bool ok = true;
  if ((dsdr1 & 0x80) == (dsdr2 & 0x80)) {
    LOG(ERROR) << "Self-test: Register Loading FAIL";
    ok = false;
  }
  if ((dsdr1 & 0x40) == (dsdr2 & 0x40)) {
    LOG(ERROR) << "Self-test: TFT Functionality FAIL";
    ok = false;
  }
  // For the checksum test, there's no inversion; 0 is success.
  if (dsdr1 & 0x01) {
    LOG(ERROR) << "Self-test: Checksum FAIL";
    ok = false;
  }
  LOG(INFO) << std::hex << std::setw(2) << std::setfill('0')
            << "Self-test overall result: 0x" << (int)dsdr1 << ", 0x"
            << (int)dsdr2 << " -> " << (ok ? "OK" : "FAIL");
  return ok;
}
}  // namespace tplp