#!/usr/bin/env python

import math
import sys

print("""
#include "tplp/motor/sin_table.h"

namespace tplp {

const uint16_t kSinTable[] = {
""")

# Prints a table of the given size containing unsigned 16-bit values of
# sin from 0 to pi/2 (exclusive)
N = int(sys.argv[1])
for i in range(0,N):
    print(round(((1<<16)-1)*math.sin((math.pi*i)/(2*N))), end=",\n")

print("""
};

}  // namespace tplp
""")
