#include "tplp/backtrace.h"

#include <unwind.h>

namespace {

// From GCC implementation comments for -mpoke-function-name:
// https://gcc.gnu.org/legacy-ml/gcc-patches/1999-04n/msg01000.html
// https://github.com/gcc-mirror/gcc/blob/releases/gcc-11.3.0/gcc/config/arm/arm.c#L21480
//
//    Write the function name into the code section, directly preceding
//    the function prologue.
//
//    Code will be output similar to this:
//        t0
//          .ascii "arm_poke_function_name", 0
//          .align
//        t1
//          .word 0xff000000 + (t1 - t0)
//        arm_poke_function_name
//          mov     ip, sp
//          stmfd   sp!, {fp, ip, lr, pc}
//          sub     fp, ip, #4
//    When performing a stack backtrace, code can inspect the value
//    of 'pc' stored at 'fp' + 0.  If the trace function then looks
//    at location pc - 12 and the top 8 bits are set, then we know
//    that there is a function name embedded immediately preceding this
//    location and has length ((pc[-3]) & 0xff000000).
//
//    We assume that pc is declared as a pointer to an unsigned long.
//
//    It is of no benefit to output the function name if we are assembling
//    a leaf function.  These function types will not contain a stack
//    backtrace structure, therefore it is not possible to determine the
//    function name.
//
// However... empirical testing has shown that whatever is happening below works
// more reliably ¯\_(ツ)_/¯. Maybe I simply don't understand what that person
// wrote.
static const char* unwind_get_function_name(const _Unwind_Word* pc) {
  static_assert(sizeof(_Unwind_Word) == 4);
  if (!pc) {
    return "<bad pc>";
  }
  if (((*(pc - 1)) & 0xff000000) != 0xff000000) {
    return "<unknown>";
  }
  int name_length = (*(pc - 1)) ^ 0xff000000;
  const char* name = reinterpret_cast<const char*>(pc - 1) - name_length;
  // consistency check. it's guaranteed to be null-terminated.
  if (name[name_length] == 0) {
    return "<bad name data>";
  }
  return name;
}

struct trace_func_param_t {
  backtrace_frame_t* const buffer;
  const int size;
  int count;
};

// Callback from the unwind backtrace function.
_Unwind_Reason_Code trace_func(struct _Unwind_Context* ctx, void* raw_param) {
  trace_func_param_t* param = static_cast<trace_func_param_t*>(raw_param);
  if (param->count >= param->size) {
    // Shouldn't get here, but who knows?
    return _URC_END_OF_STACK;
  }
  backtrace_frame_t* out_frame = &param->buffer[param->count];
  out_frame->ip = reinterpret_cast<void*>(_Unwind_GetIP(ctx));

  const _Unwind_Word* pc =
      reinterpret_cast<_Unwind_Word*>(_Unwind_GetRegionStart(ctx));
  if (!pc) {
    out_frame->name = "<bad pc>";
  } else {
    out_frame->name = unwind_get_function_name(pc);
  }
  param->count++;
  if (param->count < param->size) {
    // Continue unwinding.
    // For some reason returning _URC_CONTINUE_UNWIND does not do that.
    return _URC_NO_REASON;
  }
  // Stop unwinding.
  return _URC_END_OF_STACK;
}

}  // namespace

int TakeBacktrace(backtrace_frame_t* buffer, int size) {
  if (!buffer) return 0;
  if (size <= 0) return 0;
  trace_func_param_t param = {
      .buffer = buffer,
      .size = size,
      .count = 0,
  };
  _Unwind_Backtrace(&trace_func, &param);
  return param.count;
}