#ifndef TPLP_SHARPLCD_H_
#define TPLP_SHARPLCD_H_

#include "tplp/SpiManager.h"

namespace tplp {

// Driver for the monochrome LCD display (model LS013B4DN04) from Sharp,
// 144x168px.
class SharpLCD {
 public:
  class FrameBuffer {
    friend class SharpLCD;

   public:
    // No copies.
    FrameBuffer(FrameBuffer &) = delete;
    FrameBuffer &operator=(const FrameBuffer &) = delete;

    // Moves are OK.
    FrameBuffer(FrameBuffer &&) = default;
    FrameBuffer &operator=(FrameBuffer &&) = default;

    explicit FrameBuffer(uint8_t *fb);

    void Clear(bool black = false);

    // Copy over a bitmap to a specified location into the framebuffer. The
    // placement of the target bitmap is limited to the LCD's
    // boundary--otherwise this routine fails. Note that `bitmap` is indeed
    // tightly packed bits; its size should be (width*height/8).
    void BitBlit(const uint8_t *bitmap, unsigned int width, unsigned int height,
                 unsigned int posx, unsigned int posy);

    template <typename Int>
    uint8_t GetRowByte(Int row, Int byteIndex) const {
      return buffer_[RowColToIndex(row, byteIndex)];
    }

    template <typename Int>
    void SetRowByte(Int row, Int byteIndex, uint8_t pixels) {
      buffer_[RowColToIndex(row, byteIndex)] = pixels;
    }

   public:
    static constexpr unsigned kLcdWidth = 144;
    static constexpr unsigned kLcdHeight = 168;

   private:
    template <typename Int>
    static unsigned RowColToIndex(Int row, Int col) {
      return (row * kLcdFramebufferSizeofScanLine) +
             kLcdFramebufferSizeofScanLineMetadata + col;
    }
    static constexpr unsigned kLcdEndOfDummySize = 2;
    static constexpr unsigned kLcdFramebufferSizeofScanLineMetadata =
        (1 + /* mode byte in SPI update command */
         1 /* addr byte in SPI update command */);
    static constexpr unsigned kLcdFramebufferSizeofScanLine =
        (kLcdFramebufferSizeofScanLineMetadata + (kLcdWidth / 8));

    static constexpr unsigned kSizeofFramebuffer =
        (kLcdHeight * kLcdFramebufferSizeofScanLine);
    static constexpr unsigned kSizeofFramebufferForAlloc =
        kSizeofFramebuffer + kLcdEndOfDummySize;

   private:
    uint8_t *buffer_;
  };

 public:
  SharpLCD(SharpLCD &) = delete;
  SharpLCD(SharpLCD &&) = default;
  SharpLCD &operator=(const SharpLCD &) = delete;

  // Caller retains ownership of `spi`.
  explicit SharpLCD(SpiManager *spi);

  // Configure the SPI hardware and start a background task to toggle VCOM
  // periodically.
  //
  // `cs`: Chip select pin. `SpiManager` will expect this to be active LOW, but
  // the Sharp LCD chip expects active HIGH. Make sure you've got an inverter in
  // the circuit.
  void Begin(gpio_pin_t cs);

  // Clear the LCD's display to all-white. More efficient than writing a
  // framebuffer.
  void Clear();

  // Pushes the given framebuffer out to the display over SPI, returning
  // immediately. Returns true if successful. Returns false if the frame was
  // dropped due to the transmit queue being full.
  bool DrawFrameBuffer(
      const FrameBuffer &fb,
      const SpiDevice::transmit_callback_t &callback = nullptr);

  // Pushes the given framebuffer out to the display over SPI,
  // waiting until finished.
  void DrawFrameBufferBlocking(const FrameBuffer &fb);

  // Toggle the VCOM mode of the LCD; it is recommended to trigger this
  // periodically. Check the datasheet.
  void ToggleVCOM();

  static FrameBuffer AllocateNewFrameBuffer();

  // Returns display width in pixels.
  static int width() { return FrameBuffer::kLcdWidth; }
  // Returns display height in pixels.
  static int height() { return FrameBuffer::kLcdHeight; }

 private:
  static constexpr uint8_t kM0Flag = 0x80;
  static constexpr uint8_t kM1Flag = 0x40;
  static constexpr uint8_t kM2Flag = 0x20;
  static constexpr uint8_t kDummy8 = 0x00;

 private:
  void WriteBufferBlocking(const uint8_t *buffer, unsigned len);

 private:
  SpiManager *const spi_;
  SpiDevice *spi_device_;
};

}  // namespace tplp

#endif  // TPLP_SHARPLCD_H_