#include "tplp/hx8357/hx8357.h"

#include <iomanip>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "hardware/gpio.h"
#include "picolog/picolog.h"
#include "tplp/rtos_util.h"

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

constexpr int16_t HX8357_TFTWIDTH = 320;   ///< 320 pixels wide
constexpr int16_t HX8357_TFTHEIGHT = 480;  ///< 480 pixels tall

constexpr uint8_t HX8357_NOP = 0x00;      ///< No op
constexpr uint8_t HX8357_SWRESET = 0x01;  ///< software reset
constexpr uint8_t HX8357_RDDID = 0x04;    ///< Read ID
constexpr uint8_t HX8357_RDDST = 0x09;    ///< (unknown)

constexpr uint8_t HX8357_RDPOWMODE = 0x0A;  ///< Read power mode Read power mode
constexpr uint8_t HX8357_RDMADCTL = 0x0B;   ///< Read MADCTL
constexpr uint8_t HX8357_RDCOLMOD = 0x0C;   ///< Column entry mode
constexpr uint8_t HX8357_RDDIM = 0x0D;      ///< Read display image mode
constexpr uint8_t HX8357_RDDSDR = 0x0F;     ///< Read display signal mode

constexpr uint8_t HX8357_SLPIN = 0x10;   ///< Enter sleep mode
constexpr uint8_t HX8357_SLPOUT = 0x11;  ///< Exit sleep mode
constexpr uint8_t HX8357B_PTLON = 0x12;  ///< Partial mode on
constexpr uint8_t HX8357B_NORON = 0x13;  ///< Normal mode

constexpr uint8_t HX8357_INVOFF = 0x20;   ///< Turn off invert
constexpr uint8_t HX8357_INVON = 0x21;    ///< Turn on invert
constexpr uint8_t HX8357_DISPOFF = 0x28;  ///< Display off
constexpr uint8_t HX8357_DISPON = 0x29;   ///< Display on

constexpr uint8_t HX8357_CASET = 0x2A;  ///< Column addr set
constexpr uint8_t HX8357_PASET = 0x2B;  ///< Page addr set
constexpr uint8_t HX8357_RAMWR = 0x2C;  ///< Write VRAM
constexpr uint8_t HX8357_RAMRD = 0x2E;  ///< Read VRAM

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
    // Soft reset, then delay
    HX8357_SWRESET, 0x80 + 100 / 5,
    // Enable extended command set
    HX8357D_SETC, 3, 0xFF, 0x83, 0x57,
    // No-op and delay (nothing is sent, 0xFF is interpreted by SendCommand)
    0xFF, 0x80 + 500 / 5,
    // Set low-level RGB interface parameters
    // 0x80: enable SDO pin, disable BYPASS, set 65k color mode data format
    //       5-6-5 to insert 6th LSB equal to the MSB, set RAM access interface
    //       to DBI/MPU, set display sync to internal clock.
    // 0x00: set DOTCLK polarity to rising edge, set VSYNC pin to active low,
    //       set HSYNC pin to active low, set ENABLE pin to active high
    // 0x06: set RGB mode 1 (VS+HS+DE), set delay period from falling edge of
    //       HSYNC signal to first valid data in DPI I/F mode 2 = 6 DOTCLK
    //       cycles.
    // 0x06: Set the delay period from falling edge of VSYNC signal to first
    //       valid line in DPI I/F mode 2 = 6 DOTCLK cycles.
    // (but those timings don't matter much because we're in DBI mode, not DPI)
    HX8357_SETRGB, 4, 0x80, 0x00, 0x06, 0x06,
    // Set VCOM voltage = -1.52V
    HX8357D_SETCOM, 1, 0x25,
    // Set internal oscillator frequency (controls display framerate)
    // Normal mode 70Hz, Idle mode 55 Hz
    HX8357_SETOSC, 1, 0x68,
    // Set panel characteristics
    // Normally white, BGR, Gate direction swapped, Source direction normal
    HX8357_SETPANEL, 1, 0x05,
    // Arcane low-level panel voltage stuff
    HX8357_SETPWR1, 6,
    0x00,  // Not deep standby
    0x15,  // TRI, BT=5
    0x1C,  // VSPR
    0x1C,  // VSNR
    0x83,  // AP
    0xAA,  // FS
    // Arcane low-level panel timing and bias current stuff
    HX8357D_SETSTBA, 6,
    0x50,  // OPON normal
    0x50,  // OPON idle
    0x01,  // STBA
    0x3C,  // STBA
    0x1E,  // STBA
    0x08,  // GEN
    // Arcane low-level panel clock timing stuff
    HX8357D_SETCYC, 7,
    0x02,  // NW 0x02
    0x40,  // RTN
    0x00,  // DIV
    0x2A,  // DUM
    0x2A,  // DUM
    0x0D,  // GDON
    0x78,  // GDOFF
    // Set the gamma curve. Where did this come from?
    HX8357D_SETGAMMA, 34, 0x02, 0x0A, 0x11, 0x1d, 0x23, 0x35, 0x41, 0x4b, 0x4b,
    0x42, 0x3A, 0x27, 0x1B, 0x08, 0x09, 0x03, 0x02, 0x0A, 0x11, 0x1d, 0x23,
    0x35, 0x41, 0x4b, 0x4b, 0x42, 0x3A, 0x27, 0x1B, 0x08, 0x09, 0x03, 0x00,
    0x01,
    // Set 16 bits/pixel for DBI and DPI interfaces
    HX8357_COLMOD, 1, 0x55,
    // Define read/write scanning direction of frame memory.
    // MX, MY control rotation of the display. Initial rotation MX=1, MY=1,
    // matching the initial 320x480 value for display_width_ and
    // display_height_.
    // Also set RGB-BGR order for an RGB color filter panel.
    HX8357_MADCTL, 1, 0xC0,
    // Tearing-effect output line off.
    // The Adafruit board doesn't bring it out.
    HX8357_TEON, 1, 0x00,
    // Tearing-effect line timing
    // Pointless(?)
    HX8357_TEARLINE, 2, 0x00, 0x02,
    // Exit sleep, then delay 150ms
    HX8357_SLPOUT, 0x80 + 150 / 5,
    // Main screen turn on
    HX8357_DISPON, 0x80 + 50 / 5,
    0,  // END OF COMMAND LIST
};

}  // namespace

enum class HX8357::DisplayType { HX8357D = 0xD, HX8357B = 0xB };

HX8357::HX8357(DisplayType type, SpiController* spi, gpio_pin_t cs,
               gpio_pin_t dc)
    : type_(type), spi_(spi), cs_(cs), dc_(dc) {}

HX8357B::HX8357B(SpiController* spi, gpio_pin_t cs, gpio_pin_t dc)
    : HX8357(DisplayType::HX8357B, spi, cs, dc) {}

HX8357D::HX8357D(SpiController* spi, gpio_pin_t cs, gpio_pin_t dc)
    : HX8357(DisplayType::HX8357D, spi, cs, dc) {}

void HX8357::Begin() {
  spi_device_ = spi_->AddDevice(cs_, "HX8357");
  // Screen dimensions for default rotation (1,1,0)
  width_ = HX8357_TFTWIDTH;
  height_ = HX8357_TFTHEIGHT;
  gpio_init(dc_);
  gpio_set_dir(dc_, GPIO_OUT);
  SendInitSequence();
}

void HX8357::SendInitSequence() {
  LOG(INFO) << "HX8357 init sequence start";
  // The display needs some time to warm up after power-on, before we start
  // sending commands. Otherwise things go Wrong(tm).
  EnsureTimeSinceBootMillis(50);

  // Hog the SPI bus for the duration.
  SpiTransaction txn = spi_device_->StartTransaction();
  gpio_put(dc_, 0);

  const uint8_t* addr = (type_ == DisplayType::HX8357B) ? kInitB : kInitD;
  uint8_t cmd, x, numArgs;
  while ((cmd = *(addr++)) > 0) {  // '0' command ends list
    x = *(addr++);
    numArgs = x & 0x7F;
    if (cmd != 0xFF) {  // '255' is ignored
      if (x & 0x80) {   // If high bit set, numArgs is a delay time
        SendCommand(txn, cmd, nullptr, 0);
      } else {
        SendCommand(txn, cmd, addr, numArgs);
        addr += numArgs;
      }
    }
    if (x & 0x80) {  // If high bit set...
      // numArgs is actually a delay time in milliseconds
      int t = MillisToTicks(numArgs * 5);
      VLOG(1) << "Sleep at least " << (numArgs * 5)
              << "ms <= " << t * portTICK_PERIOD_MS << "ms as ticks";
      vTaskDelay(t);
    }
  }
  LOG(INFO) << "HX8357 init sequence finished";
}

void HX8357::SendCommand(SpiTransaction& txn, uint8_t command,
                         const uint8_t* data, uint8_t len) {
  VLOG(1) << std::hex << std::setw(2) << std::setfill('0') << "SendCommand(0x"
          << (int)command << ", [" << std::dec << (int)len << " bytes])";
  txn.Transfer({
      .read_addr = &command,
      .trans_count = 1,
      .toggle_gpio = (data && len) ? dc_ : gpio_pin_t(-1),
  });
  if (data && len) {
    txn.Transfer({
        .read_addr = data,
        .trans_count = len,
        .toggle_gpio = dc_,
    });
  }
}

uint8_t HX8357::RDDSDR() {
  uint8_t buf0[2] = {HX8357_RDDSDR, 0};
  uint8_t buf1[2] = {0, 0};
  gpio_put(dc_, 0);
  SpiTransaction txn = spi_device_->StartTransaction();
  txn.Transfer({
      .read_addr = buf0,
      .write_addr = buf1,
      .trans_count = 2,
  });
  VLOG(1) << std::hex << std::setw(2) << std::setfill('0')
          << "RDDSDR response 0x" << (int)buf1[1];
  return buf1[1];
}

bool HX8357::SelfTest() {
  uint8_t dsdr1 = RDDSDR();
  {
    SpiTransaction txn = spi_device_->StartTransaction();
    gpio_put(dc_, 0);
    SendCommand(txn, HX8357_SLPOUT);
  }
  vTaskDelay(MillisToTicks(140));
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

// TODO: add the automatic brightness control self-diagnostic
// see command RDABCSDR, page 177.

namespace {
// Please don't send a negative number
uint8_t upper_half(int16_t n) { return (n & 0xff00) >> 8; }
uint8_t lower_half(int16_t n) { return n & 0x00ff; }
}  // namespace

void HX8357::Blit(const uint16_t* pixels, int16_t x1, int16_t y1, int16_t x2,
                  int16_t y2) {
  CHECK_NOTNULL(pixels);
  CHECK_GE(x1, 0);
  CHECK_GE(y1, 0);
  CHECK_GE(x2, 0);
  CHECK_GE(y2, 0);
  CHECK_LT(x1, width_);
  CHECK_LT(x2, width_);
  CHECK_LT(y1, height_);
  CHECK_LT(y2, height_);
  CHECK_LE(x1, x2);
  CHECK_LE(y1, y2);

  uint64_t t1 = 0;
  if (VLOG_IS_ON(1)) t1 = to_us_since_boot(get_absolute_time());

  const uint8_t xcoords[4] = {upper_half(x1), lower_half(x1), upper_half(x2),
                              lower_half(x2)};
  const uint8_t ycoords[4] = {upper_half(y1), lower_half(y1), upper_half(y2),
                              lower_half(y2)};
  SpiTransaction txn = spi_device_->StartTransaction();
  gpio_put(dc_, 0);
  SendCommand(txn, HX8357_CASET, xcoords, 4);
  SendCommand(txn, HX8357_PASET, ycoords, 4);
  txn.Transfer({
      .read_addr = &HX8357_RAMWR,
      .trans_count = 1,
      .toggle_gpio = dc_,
  });
  // dc_ will high for the next transfer
  // TODO: use 16-bit transfer width! :) but watch out for byte order
  uint32_t len = static_cast<uint32_t>(x2 - x1 + 1) *
                 static_cast<uint32_t>(y2 - y1 + 1) * 2u;
  txn.Transfer({
      .read_addr = reinterpret_cast<const uint8_t*>(pixels),
      .trans_count = len,
  });
  txn.Dispose();

  uint64_t t2 = 0;
  if (VLOG_IS_ON(1)) t2 = to_us_since_boot(get_absolute_time());
  VLOG(1) << "Blit() finished in " << (t2 - t1) << "us ~"
          << ((1'000ll * len) / (t2 - t1)) << "kB/s";
}

void HX8357::SetRotation(bool mx, bool my, bool mv) {
  uint8_t param = MADCTL_RGB;
  if (mx) param |= MADCTL_MX;
  if (my) param |= MADCTL_MY;
  if (mv) {
    param |= MADCTL_MV;
    width_ = HX8357_TFTHEIGHT;
    height_ = HX8357_TFTWIDTH;
  } else {
    width_ = HX8357_TFTWIDTH;
    height_ = HX8357_TFTHEIGHT;
  }
  LOG(INFO) << "Setting rotation to (" << mx << "," << my << "," << mv
            << "). New dimensions " << width_ << "x" << height_;

  SpiTransaction txn = spi_device_->StartTransaction();
  gpio_put(dc_, 0);
  SendCommand(txn, HX8357_MADCTL, &param, 1);
}

void HX8357::SetInvertedColors(bool invert) {
  SpiTransaction txn = spi_device_->StartTransaction();
  gpio_put(dc_, 0);
  SendCommand(txn, invert ? HX8357_INVON : HX8357_INVOFF);
}
}  // namespace tplp