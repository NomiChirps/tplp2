#include "tplp/ui/main.h"

#include <SDL2/SDL.h>
#include <assert.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"
#include "FreeRTOS/task.h"
#include "lv_drivers/indev/keyboard.h"
#include "lv_drivers/indev/mouse.h"
#include "lv_drivers/indev/mousewheel.h"
#include "lv_drivers/sdl/sdl.h"
#include "lvgl/lvgl.h"
#include "simulator/console.h"
#include "simulator/hooks.h"
#include "tplp/config/params.h"

// Some dummy parameters for testing that part of the UI.
#define ADD_TEST_TPLP_PARAM(n) \
  TPLP_PARAM(int32_t, test_numeric_param_##n, n, "Blah, blah, blah: " #n)
ADD_TEST_TPLP_PARAM(1);
ADD_TEST_TPLP_PARAM(2);
ADD_TEST_TPLP_PARAM(3);
ADD_TEST_TPLP_PARAM(99999999);

// Define params the UI has a hard dependency on.
TPLP_PARAM(int32_t, loadcell_offset, 100000, "Loadcell offset");
TPLP_PARAM(int32_t, loadcell_scale, 200, "Loadcell scale");

// Initialize the Hardware Abstraction Layer (HAL) for LVGL
static void hal_init(void) {
  SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1");
  sdl_init();

  /*Create a display buffer*/
  static lv_disp_draw_buf_t disp_buf1;
  static lv_color_t buf1_1[SDL_HOR_RES * 100];
  static lv_color_t buf1_2[SDL_HOR_RES * 100];
  lv_disp_draw_buf_init(&disp_buf1, buf1_1, buf1_2, SDL_HOR_RES * 100);

  /*Create a display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv); /*Basic initialization*/
  disp_drv.draw_buf = &disp_buf1;
  disp_drv.flush_cb = sdl_display_flush;
  disp_drv.hor_res = SDL_HOR_RES;
  disp_drv.ver_res = SDL_VER_RES;
  disp_drv.antialiasing = 1;

  lv_disp_t* disp = lv_disp_drv_register(&disp_drv);

  lv_theme_t* th = lv_theme_default_init(
      disp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
      LV_THEME_DEFAULT_DARK, LV_FONT_DEFAULT);
  lv_disp_set_theme(disp, th);

  lv_group_t* g = lv_group_create();
  lv_group_set_default(g);

  static lv_indev_drv_t indev_drv_1;
  lv_indev_drv_init(&indev_drv_1); /*Basic initialization*/
  indev_drv_1.type = LV_INDEV_TYPE_POINTER;

  /*This function will be called periodically (by the library) to get the mouse
   * position and state*/
  indev_drv_1.read_cb = sdl_mouse_read;
  lv_indev_t* mouse_indev = lv_indev_drv_register(&indev_drv_1);

  static lv_indev_drv_t indev_drv_2;
  lv_indev_drv_init(&indev_drv_2); /*Basic initialization*/
  indev_drv_2.type = LV_INDEV_TYPE_KEYPAD;
  indev_drv_2.read_cb = sdl_keyboard_read;
  lv_indev_t* kb_indev = lv_indev_drv_register(&indev_drv_2);
  lv_indev_set_group(kb_indev, g);

  static lv_indev_drv_t indev_drv_3;
  lv_indev_drv_init(&indev_drv_3); /*Basic initialization*/
  indev_drv_3.type = LV_INDEV_TYPE_ENCODER;
  indev_drv_3.read_cb = sdl_mousewheel_read;

  lv_indev_t* enc_indev = lv_indev_drv_register(&indev_drv_3);
  lv_indev_set_group(enc_indev, g);

  /*Set a cursor for the mouse*/
  LV_IMG_DECLARE(mouse_cursor_icon); /*Declare the image file.*/
  lv_obj_t* cursor_obj =
      lv_img_create(lv_scr_act()); /*Create an image object for the cursor */
  lv_img_set_src(cursor_obj, &mouse_cursor_icon); /*Set the image source*/
  lv_indev_set_cursor(mouse_indev,
                      cursor_obj); /*Connect the image  object to the driver*/
}

// A task to measure the elapsed time for LVGL
static void tick_task(void* unused) {
  console_print("tick task started\n");

  long n = 0;
  while (1) {
    // not accurate if this task was delayed
    lv_tick_inc(5);

    vTaskDelay(pdMS_TO_TICKS(5));
    if (++n % 500 == 0) {
      // console_print("lvgl tick %ld\n", n);
    }
  }
}

// Initializes everything we need for FreeRTOS and starts the scheduler.
// This function will be run as an ordinary pthread.
static int FreeRTOSStartupPthread(void*) {
  const int kDefaultTaskStack = 2048;

  int ok;
  ok = xTaskCreate(&tick_task, "tick", kDefaultTaskStack, NULL, /*priority=*/1,
                   NULL);
  assert(ok);

  vTaskStartScheduler();
  // should never reach here
  assert(0);
}

int main() {
  // SIGINT is not blocked by the FreeRTOS posix port
  signal(SIGINT, handle_sigint);
  // Set up the global mutex for stdio. printf is not thread-safe, so we need to
  // print everything through the console_print() wrapper.
  console_init();

  // Initialize LVGL
  lv_init();
  lv_log_register_print_cb(&lvgl_log_callback);

  // Initialize the HAL (display, input devices, tick) for LVGL
  hal_init();

  // Create all the UI widgets
  // Use the default do-nothing impl of TplpInterface.
  tplp::ui::ui_main(new tplp::ui::TplpInterface());

  // Fork off a thread to run the FreeRTOS scheduler.
  SDL_CreateThread(&FreeRTOSStartupPthread, "FreeRTOS Scheduler", NULL);

  // Run the combined LVGL/SDL event loop.
  // It's necessary to do this in the main thread, and not in a FreeRTOS task,
  // because of *waves hands* something about MacOS. It's also necessary to do
  // this in the same thread that SDL was initialized in, due to *waves hands*
  // GL context owner reasons.
  long n = 0;
  for (;;) {
    // No mutex needed because the only other thread calling LVGL functions is
    // tick_task, and lv_tick_inc() is already thread-safe.
    lv_timer_handler();

    usleep(5000);
    if (++n % 500 == 0) {
      // console_print("lvgl timer handler call count %ld\n", n);
    }
  }

  return 0;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
