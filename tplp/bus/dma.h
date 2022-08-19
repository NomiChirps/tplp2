#ifndef TPLP_BUS_DMA_H_
#define TPLP_BUS_DMA_H_

#include "tplp/bus/dma_program.h"
#include "tplp/bus/types.h"

namespace tplp {

// just a sketch...
class DmaController {
 public:
  static DmaController* Init(dma_irq_index_t irq_index);

  DmaProgram NewProgram();
  void Enqueue(const DmaProgram&);

 private:
  explicit DmaController();
};

}  // namespace tplp

#endif  // TPLP_BUS_DMA_H_