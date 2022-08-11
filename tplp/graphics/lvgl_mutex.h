#ifndef TPLP_GRAPHICS_LVGL_MUTEX_H_
#define TPLP_GRAPHICS_LVGL_MUTEX_H_

namespace tplp {

// This is a scoped mutex guard. It MUST be held while calling ANY lvgl
// function. Do not use until after calling InitOnce().
class LvglMutex {
 public:
  explicit LvglMutex();
  ~LvglMutex();

  LvglMutex(const LvglMutex&) = delete;
  LvglMutex& operator=(const LvglMutex&) = delete;

  static void InitOnce();
};

}  // namespace tplp

#endif  // TPLP_GRAPHICS_LVGL_MUTEX_H_