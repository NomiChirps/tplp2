#include "tplp/SharpLCD/SharpLCD.h"

#include <chrono>
#include <cstdio>
#include <utility>
#include <vector>

#include "tplp/assert.h"
#include "tplp/logging.h"
#include "tplp/time.h"
#include "tplp/tplp_config.h"

using std::chrono_literals::operator""ms;

namespace tplp {
namespace {

inline uint8_t BitReverse8(uint8_t b) {
  // http://graphics.stanford.edu/~seander/bithacks.html#ReverseByteWith32Bits
  return ((b * 0x0802LU & 0x22110LU) | (b * 0x8020LU & 0x88440LU)) *
             0x10101LU >>
         16;
  // Alas, if only...
  // return __builtin_arm_rbit(byte) >> (8 * sizeof(unsigned int) - 8);
}

}  // namespace

SharpLCD::FrameBuffer::FrameBuffer(uint8_t* fb) : buffer_(fb) {
  tplp_assert(fb);
  Clear();

  /* Make sure the dummy bytes are clear */
  ((uint8_t*)buffer_)[kSizeofFramebuffer] = kDummy8;
  ((uint8_t*)buffer_)[kSizeofFramebuffer + 1] = kDummy8;
}

void SharpLCD::FrameBuffer::Clear(bool black) {
  uint8_t* pfb;
  unsigned int row;
  unsigned int index;

  /* initialize the frame buffer */
  pfb = buffer_;
  for (row = 0; row < kLcdHeight; row++) {
    for (index = 0; index < kLcdFramebufferSizeofScanLine; index++) {
      switch (index) {
        case 0:
          *pfb++ = kM0Flag; /* update line command */
          break;
        case 1:
          *pfb++ = BitReverse8(row + 1);
          break;
        default:
          *pfb++ = black ? 0x00 : 0xFF;
          break;
      }
    }
  }
}

void
SharpLCD::FrameBuffer::BitBlit(const uint8_t *bitmap,
                               unsigned int   width,  /* width of the bitmap */
                               unsigned int   height, /* height of the bitmap */
                               unsigned int   posx,   /* x-offset of the top-left
                                                       * corner of the bitmap
                                                       * w.r.t. the top-left
                                                       * corner of the screen */
                               unsigned int   posy    /* y-offset of the top-left
                                                       * corner of the bitmap
                                                       * w.r.t. the top-left
                                                       * corner of the screen */)
{
  unsigned int row;
  unsigned int col;
  unsigned int bitsToBlitInRow;
  unsigned int destByteIndex;    /* within the current row */
  unsigned int destBitIndexMod8; /* bits to the right of this index in
                                  * 'destByte' need to be blitted next;
                                  * MSB has index 0. */
  unsigned int srcByteIndex;     /* byte index within the source bitmap */
  unsigned int srcBitIndexMod8;  /* bits to the right of this index in
                                  * source byte (at index srcByteIndex)
                                  * need to be blitted to the dest; MSB
                                  * has index 0. */
  const unsigned int endRow = posy + height;
  uint8_t destByte;
  uint8_t srcBits;

  tplp_assert(bitmap);
  tplp_assert(width > 0);
  tplp_assert(height > 0);
  tplp_assert(posx + width <= kLcdWidth);
  tplp_assert(posy + height <= kLcdHeight);

#define SHIFT_INTO_SRC_BITS_FROM_SINGLE_SRC_BYTE(M)                            \
  do {                                                                         \
    uint8_t mask;                                                              \
                                                                               \
    mask = ((uint8_t)((uint8_t)1 << (M)) - 1);                                 \
    srcBits <<= (M);                                                           \
    srcBits |= (bitmap[srcByteIndex] >> (8 - (srcBitIndexMod8 + (M)))) & mask; \
                                                                               \
    /* update the indices */                                                   \
    srcBitIndexMod8 += (M);                                                    \
    if (srcBitIndexMod8 == 8) {                                                \
      srcBitIndexMod8 = 0;                                                     \
      srcByteIndex++;                                                          \
    }                                                                          \
  } while (0)

/* Left-shift N bits into srcBits; fetching them from the source
 * bitmap. */
#define SHIFT_INTO_SRC_BITS(N)                               \
  do {                                                       \
    uint8_t bitsToShift;                                     \
                                                             \
    bitsToShift = (N);                                       \
                                                             \
    if ((srcBitIndexMod8 + (N)) > 8) {                       \
      bitsToShift = 8 - srcBitIndexMod8;                     \
      SHIFT_INTO_SRC_BITS_FROM_SINGLE_SRC_BYTE(bitsToShift); \
      bitsToShift = (N)-bitsToShift;                         \
    }                                                        \
                                                             \
    SHIFT_INTO_SRC_BITS_FROM_SINGLE_SRC_BYTE(bitsToShift);   \
  } while (0)

  srcByteIndex = 0;
  srcBitIndexMod8 = 0;
  for (row = posy; row < endRow; row++) {
    col = posx;
    bitsToBlitInRow = width;
    destBitIndexMod8 = col & 0x7 /* col % 8 */;
    destByteIndex = col >> 3 /* col / 8 */;
    srcBits = 0;

    /* While there are bits in this row remaining to be blitted to
     * the destination, ... */
    while (bitsToBlitInRow) {
      if ((destBitIndexMod8 == 0) && (bitsToBlitInRow >= 8)) {
        /* We know that destBitIndexMod8 == 0, which means that
         * the destination is byte aligned and we can simply
         * do the equivalent of a memcpy. */
        while (bitsToBlitInRow >= 8) {
          SHIFT_INTO_SRC_BITS(8);
          SetRowByte(row, destByteIndex, srcBits);

          bitsToBlitInRow -= 8;
          destByteIndex++;
        }
      } else {
        uint8_t blit; /* number of bits to blit in this iteration */
        uint8_t mask;
        uint8_t leftShift;

        /* This will be a read-modify-write operation, so we
         * need to fetch the destination byte. */
        destByte = GetRowByte(row, destByteIndex);

        if ((destBitIndexMod8 + bitsToBlitInRow) >= 8) {
          blit = 8 - destBitIndexMod8;
          leftShift = 0;
        } else {
          blit = bitsToBlitInRow;
          leftShift = (8 - (destBitIndexMod8 + bitsToBlitInRow));
        }

        SHIFT_INTO_SRC_BITS(blit);
        mask = ((uint8_t)((uint8_t)1 << blit) - 1) << leftShift;

        /* update destByte */
        destByte &= ~mask;
        destByte |= ((uint8_t)(srcBits << leftShift) & mask);

        SetRowByte(row, destByteIndex, destByte);

        /* update dest indices */
        bitsToBlitInRow -= blit;
        destBitIndexMod8 += blit;
        if (destBitIndexMod8 == 8) {
          destBitIndexMod8 = 0;
          destByteIndex++;
        }
      }
    }

    /* potentially update srcByteIndex */
    if (srcBitIndexMod8 != 0) {
      srcBitIndexMod8 = 0;
      srcByteIndex++;
    }
  }
}

SharpLCD::SharpLCD(SpiManager* spi) : spi_(spi) {
  tplp_assert(spi_->GetActualFrequency() <= 2'000'000);
}

void SharpLCD::Begin(gpio_pin_t cs, int toggle_vcom_task_priority) {
  spi_device_ = spi_->AddDevice(cs);
  tplp_assert(xTaskCreate(&SharpLCD::ToggleVcomTask, "ToggleVCOM",
                          TplpConfig::kDefaultTaskStackSize, this,
                          toggle_vcom_task_priority, nullptr) == pdPASS);
}

SharpLCD::FrameBuffer SharpLCD::AllocateNewFrameBuffer() {
  return FrameBuffer(new uint8_t[FrameBuffer::kSizeofFramebufferForAlloc]);
}

void SharpLCD::WriteBufferBlocking(const uint8_t* buffer, unsigned len) {
  tplp_assert(spi_device_);
  int ret = spi_device_->TransmitBlocking(buffer, len, as_ticks(1'000ms),
                                          as_ticks(1'000ms));
  if (ret) {
    DebugLog("WriteBufferBlocking() timed out. ret=%s", ret);
  }
}

void SharpLCD::Clear() {
  const uint8_t buf[2] = {kM2Flag, kDummy8};

  WriteBufferBlocking(buf, sizeof(buf));
}

bool SharpLCD::DrawFrameBuffer(const FrameBuffer& fb,
                               const SpiDevice::transmit_callback_t& callback) {
  return spi_device_->Transmit(
      fb.buffer_, FrameBuffer::kSizeofFramebufferForAlloc, 0, callback);
}

void SharpLCD::DrawFrameBufferBlocking(const FrameBuffer& fb) {
  WriteBufferBlocking(fb.buffer_, FrameBuffer::kSizeofFramebufferForAlloc);
}

void SharpLCD::ToggleVCOM(void) {
  static bool frameInversion = false;
  uint8_t buf[2] = {0, kDummy8};

  uint8_t mode = 0x0;
  if (frameInversion) {
    mode |= kM1Flag;
  }
  // toggle frameInversion in preparation for the next call
  frameInversion = !frameInversion;

  buf[0] = mode;
  DebugLog("Sending ToggleVCOM");
  WriteBufferBlocking(buf, sizeof(buf));
}

void SharpLCD::ToggleVcomTask(void* param) {
  DebugLog("SharpLCD::ToggleVcomTask started.");
  for (;;) {
    static_cast<SharpLCD*>(param)->ToggleVCOM();
    vTaskDelay(as_ticks(std::chrono::minutes(60)));
  }
}

}  // namespace tplp