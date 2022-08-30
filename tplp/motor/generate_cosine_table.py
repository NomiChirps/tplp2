#!/usr/bin/env python

import math
import sys

print("""
#include "tplp/motor/cosine_table.h"

namespace tplp {

const uint16_t kCosineTable[] = {
""")

# Prints a table of the given size containing unsigned 16-bit values of
# cosine from 0 to pi (inclusive).
N = int(sys.argv[1])
for i in range(0,N-1):
    print(32768 + int(((1<<15) - 1)*math.cos((math.pi*i)/(N-1))), end="")
    print(",")
print(int(0))

print("""
};

}  // namespace tplp
""")
