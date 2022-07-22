#ifndef LIB_SHARPLCD_H_
#define LIB_SHARPLCD_H_

#include <cstdint>

#include "hardware/spi.h"

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

  // Free of side effects.
  SharpLCD(spi_inst_t *spi, int sclk, int mosi, int cs);

  // Configure the SPI hardware.
  void Begin();

  // Clear the LCD's display to all-white. More efficient than writing a
  // framebuffer.
  void Clear();

  void DrawFrameBufferBlocking(const FrameBuffer &fb);

  static FrameBuffer AllocateNewFrameBuffer();

  // Toggle the VCOM mode of the LCD; it is recommended to trigger this
  // periodically. Check the datasheet.
  void ToggleVCOM();

 private:
  // Helper function to write out a buffer onto the LCD's SPI channel.
  void WriteBufferBlocking(const uint8_t *buffer, unsigned len);

 private:
  spi_inst_t *const spi_;
  const int sclk_;
  const int mosi_;
  const int cs_;
};

#endif  // LIB_SHARPLCD_H_