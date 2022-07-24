#ifndef TPLP_SHARPLCD_H_
#define TPLP_SHARPLCD_H_

#include "tplp/SpiManager.h"

namespace tplp {

// Driver for the monochrome LCD display (model LS013B4DN04) from Sharp.
class SharpLCD {
 public:
  class FrameBuffer {
    friend class SharpLCD;

   public:
    FrameBuffer(FrameBuffer &) = delete;
    FrameBuffer(FrameBuffer &&) = default;
    FrameBuffer &operator=(const FrameBuffer &) = delete;

    FrameBuffer(uint8_t *fb);

    void Clear(bool black = false);

    // Copy over a bitmap to a specified location into the framebuffer. The
    // placement of the target bitmap is limited to the LCD's
    // boundary--otherwise this routine fails.
    void BitBlit(const uint8_t *bitmap, unsigned int width, unsigned int height,
                 unsigned int posx, unsigned int posy);

    uint8_t GetRowByte(unsigned int row, unsigned int byteIndex) const {
      return buffer_[RowColToIndex(row, byteIndex)];
    }

    void SetRowByte(unsigned int row, unsigned int byteIndex, uint8_t pixels) {
      buffer_[RowColToIndex(row, byteIndex)] = pixels;
    }

   private:
    static unsigned RowColToIndex(unsigned row, unsigned col);

   private:
    uint8_t *const buffer_;
  };

 public:
  SharpLCD(SharpLCD &) = delete;
  SharpLCD(SharpLCD &&) = default;
  SharpLCD &operator=(const SharpLCD &) = delete;

  explicit SharpLCD(SpiManager *spi);

  // Configure the SPI hardware.
  //
  // `cs`: Chip select pin. `SpiManager` will expect this to be active LOW, but
  // the Sharp LCD chip expects active HIGH. Make sure you've got an inverter in
  // the circuit.
  void Begin(gpio_pin_t cs);

  // Clear the LCD's display to all-white. More efficient than writing a
  // framebuffer.
  void Clear();

  // Pushes the given framebuffer out to the display over SPI.
  void DrawFrameBufferBlocking(const FrameBuffer &fb);

  static FrameBuffer AllocateNewFrameBuffer();

  // Toggle the VCOM mode of the LCD; it is recommended to trigger this
  // periodically. Check the datasheet.
  // FIXME: should start a low priority task to do this
  void ToggleVCOM();

 private:
  void WriteBufferBlocking(const uint8_t *buffer, unsigned len);

 private:
  SpiManager *const spi_;
  SpiDevice *spi_device_;
};

}  // namespace tplp

#endif  // TPLP_SHARPLCD_H_