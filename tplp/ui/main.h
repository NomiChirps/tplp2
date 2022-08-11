#ifndef TPLP_UI_MAIN_H_
#define TPLP_UI_MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

// Caller must be holding the LVGL mutex.
void ui_main(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif  // TPLP_UI_MAIN_H_