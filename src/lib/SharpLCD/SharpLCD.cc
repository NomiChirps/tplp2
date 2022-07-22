#include "src/lib/SharpLCD/SharpLCD.h"

#include <cstdio>
#include <utility>
#include <vector>

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

namespace {

static constexpr unsigned kLcdWidth = 144;
static constexpr unsigned kLcdHeight = 168;
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

static constexpr uint8_t kM0Flag = 0x80;
static constexpr uint8_t kM1Flag = 0x40;
static constexpr uint8_t kM2Flag = 0x20;
static constexpr uint8_t kDummy8 = 0x00;

inline uint8_t BitReverse8(uint8_t b) {
  // http://graphics.stanford.edu/~seander/bithacks.html#ReverseByteWith32Bits
  return ((b * 0x0802LU & 0x22110LU) | (b * 0x8020LU & 0x88440LU)) *
             0x10101LU >>
         16;
  // Alas, if only...
  // return __builtin_arm_rbit(byte) >> (8 * sizeof(unsigned int) - 8);
}

static std::pair<int, SharpLCD *> DMA_INSTANCE[4];
static int DMA_INSTANCE_COUNT = 0;

void RegisterInstance(int dma, SharpLCD *instance) {
  DMA_INSTANCE[DMA_INSTANCE_COUNT++] = std::make_pair(dma, instance);
}

}  // namespace

void SharpLCD::InitISRStuff() {
  static bool is_init = false;
  if (!is_init) {
    irq_add_shared_handler(DMA_IRQ_0, &SharpLCD::TransferDone_ISR,
                           PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);
    is_init = true;
  }
}

void SharpLCD::TransferDone_ISR() {
  // TODO(chirps): This is a rather large ISR...
  // what we really need here is a RTOS task managing the SPI bus.
  for (int i = 0; i < DMA_INSTANCE_COUNT; ++i) {
    if (dma_channel_get_irq0_status(DMA_INSTANCE[i].first)) {
      // FIXME(chirps): This happens too early; the SPI FIFO is still full.
      gpio_put(DMA_INSTANCE[i].second->cs_, 0);
      dma_channel_acknowledge_irq0(DMA_INSTANCE[i].first);
      if (DMA_INSTANCE[i].second->ready_isr_) {
        DMA_INSTANCE[i].second->ready_isr_();
      }
    }
  }
}

SharpLCD::FrameBuffer::FrameBuffer(uint8_t *fb) : buffer_(fb) {
  Clear();

  /* Make sure the dummy bytes are clear */
  ((uint8_t *)buffer_)[kSizeofFramebuffer] = kDummy8;
  ((uint8_t *)buffer_)[kSizeofFramebuffer + 1] = kDummy8;
}

void SharpLCD::FrameBuffer::Clear(bool black) {
  uint8_t *pfb;
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

  // TODO(chirps): throw exceptions...? no good assert mechanism otherwise

  // ASSERT(bitmap != NULL, "%s: passed in a NULL bitmap", __FUNCTION__);
  // ASSERT(width > 0, "%s: passed in an invalid width", __FUNCTION__);
  // ASSERT(height > 0, "%s: passed in an invalid height", __FUNCTION__);
  // ASSERT(posx + width <= LCD_WIDTH,
  //        "%s: bitmap will exceed the screen width", __FUNCTION__);
  // ASSERT(posy + height <= LCD_HEIGHT,
  //        "%s: bitmap will exceed the screen height", __FUNCTION__);

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

unsigned SharpLCD::FrameBuffer::RowColToIndex(unsigned row, unsigned col) {
  return (row * kLcdFramebufferSizeofScanLine) +
         kLcdFramebufferSizeofScanLineMetadata + col;
}

SharpLCD::SharpLCD(spi_inst_t *spi, int sclk, int mosi, int cs)
    : spi_(spi), sclk_(sclk), mosi_(mosi), cs_(cs) {}

void SharpLCD::Begin() {
  // TODO(chirps): this is gonna be sharing an SPI bus with other devices,
  // soooooo... this won't fly.
  unsigned int actual_freq = spi_init(spi_, 2'000'000);
  printf("SPI initialized with frequency = %u\n", actual_freq);
  gpio_set_function(sclk_, GPIO_FUNC_SPI);
  gpio_set_function(mosi_, GPIO_FUNC_SPI);
  gpio_init(cs_);
  gpio_set_dir(cs_, GPIO_OUT);
  gpio_put(cs_, 0);

  // init dma
  dma_ = dma_claim_unused_channel(true);
  dma_channel_config c = dma_channel_get_default_config(dma_);
  channel_config_set_dreq(&c, spi_get_dreq(spi_, /*is_tx=*/true));
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);
  channel_config_set_irq_quiet(&c, false);
  dma_channel_set_config(dma_, &c, /*trigger=*/false);
  dma_channel_configure(dma_, &c, /*write_addr=*/&spi_get_hw(spi_)->dr,
                        /*read_addr=*/nullptr,
                        /*transfer_count=*/0,
                        /*trigger=*/false);

  InitISRStuff();
  RegisterInstance(dma_, this);
  dma_channel_set_irq0_enabled(dma_, true);
}

SharpLCD::FrameBuffer SharpLCD::AllocateNewFrameBuffer() {
  return FrameBuffer(new uint8_t[kSizeofFramebufferForAlloc]);
}

void SharpLCD::WriteBufferBlocking(const uint8_t *buffer, unsigned len) {
  dma_channel_wait_for_finish_blocking(dma_);
  // Also wait for the SPI TX FIFO to drain.
  while (spi_is_busy(spi_)) tight_loop_contents();
  gpio_put(cs_, 1);
  dma_channel_transfer_from_buffer_now(dma_, buffer, len);
}

void SharpLCD::Clear() {
  const uint8_t buf[2] = {kM2Flag, kDummy8};

  WriteBufferBlocking(buf, sizeof(buf));
}

void SharpLCD::DrawFrameBufferBlocking(const FrameBuffer &fb) {
  WriteBufferBlocking(fb.buffer_, kSizeofFramebufferForAlloc);
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
  WriteBufferBlocking(buf, sizeof(buf));
}