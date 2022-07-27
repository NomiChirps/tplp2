#ifndef TPLP_GRAPHICS_LVGL_MUTEX_H_
#define TPLP_GRAPHICS_LVGL_MUTEX_H_

namespace tplp {

// This is a scoped mutex guard. It MUST be held while calling ANY lvgl
// function. Do not use until after calling InitOnce().
class LvglLock {
 public:
  explicit LvglLock();
  ~LvglLock();

  LvglLock(const LvglLock&) = delete;
  LvglLock& operator=(const LvglLock&) = delete;

  static void InitOnce();
};

}  // namespace tplp

#endif  // TPLP_GRAPHICS_LVGL_MUTEX_H_