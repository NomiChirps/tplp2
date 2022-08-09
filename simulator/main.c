
/**
 * @file main
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <assert.h>
#include <stdio.h>
#include <SDL2/SDL.h>
#include "lvgl/lvgl.h"
#include "lv_drivers/sdl/sdl.h"
#include "lv_drivers/indev/mouse.h"
#include "lv_drivers/indev/keyboard.h"
#include "lv_drivers/indev/mousewheel.h"
#include "ui/main.h"
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/semphr.h"
#include "simulator/hooks.h"
#include "simulator/console.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void hal_init(void);
static void tick_task(void *data);
static void timer_task(void *data);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *      VARIABLES
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

SemaphoreHandle_t GLOBAL_LVGL_MUTEX;
StaticSemaphore_t GLOBAL_LVGL_MUTEX_BUFFER;

 void StartWithFreeRTOS() {
  const int kDefaultTaskStack = 2048;

  GLOBAL_LVGL_MUTEX = xSemaphoreCreateMutexStatic( &GLOBAL_LVGL_MUTEX_BUFFER );

  int ok;
  ok = xTaskCreate(&tick_task, "tick", kDefaultTaskStack, NULL, /*priority=*/1, NULL);
  assert(ok);

  /* Periodically call the lv_task handler. */
  ok = xTaskCreate(&timer_task, "timer", kDefaultTaskStack, NULL, /*priority=*/1, NULL);
  assert(ok);

  vTaskStartScheduler();
 }

 void StartWithoutThreads() {
  for(;;) {
    lv_timer_handler();
    usleep(5000);
    lv_tick_inc(5);
  }
 }

int main(int argc, char **argv)
{
  (void)argc; /*Unused*/
  (void)argv; /*Unused*/

  /* SIGINT is not blocked by the posix port */
  signal( SIGINT, handle_sigint );
  console_init();

  //
  //
  // FIXME: StartWithoutThreads works. StartWithFreeRTOS doesn't...
  //
  //
  StartWithFreeRTOS();
  //StartWithoutThreads();

  return 0;
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * Initialize the Hardware Abstraction Layer (HAL) for the LVGL graphics
 * library
 */
static void hal_init(void)
{
  SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1");
  /* Use the 'monitor' driver which creates window on PC's monitor to simulate a display*/
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

  lv_disp_t * disp = lv_disp_drv_register(&disp_drv);

  lv_theme_t * th = lv_theme_default_init(disp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), LV_THEME_DEFAULT_DARK, LV_FONT_DEFAULT);
  lv_disp_set_theme(disp, th);

  lv_group_t * g = lv_group_create();
  lv_group_set_default(g);

  static lv_indev_drv_t indev_drv_1;
  lv_indev_drv_init(&indev_drv_1); /*Basic initialization*/
  indev_drv_1.type = LV_INDEV_TYPE_POINTER;

  /*This function will be called periodically (by the library) to get the mouse position and state*/
  indev_drv_1.read_cb = sdl_mouse_read;
  lv_indev_t *mouse_indev = lv_indev_drv_register(&indev_drv_1);

  static lv_indev_drv_t indev_drv_2;
  lv_indev_drv_init(&indev_drv_2); /*Basic initialization*/
  indev_drv_2.type = LV_INDEV_TYPE_KEYPAD;
  indev_drv_2.read_cb = sdl_keyboard_read;
  lv_indev_t *kb_indev = lv_indev_drv_register(&indev_drv_2);
  lv_indev_set_group(kb_indev, g);

  static lv_indev_drv_t indev_drv_3;
  lv_indev_drv_init(&indev_drv_3); /*Basic initialization*/
  indev_drv_3.type = LV_INDEV_TYPE_ENCODER;
  indev_drv_3.read_cb = sdl_mousewheel_read;

  lv_indev_t * enc_indev = lv_indev_drv_register(&indev_drv_3);
  lv_indev_set_group(enc_indev, g);

  /*Set a cursor for the mouse*/
  LV_IMG_DECLARE(mouse_cursor_icon); /*Declare the image file.*/
  lv_obj_t * cursor_obj = lv_img_create(lv_scr_act()); /*Create an image object for the cursor */
  lv_img_set_src(cursor_obj, &mouse_cursor_icon);           /*Set the image source*/
  lv_indev_set_cursor(mouse_indev, cursor_obj);             /*Connect the image  object to the driver*/
}

/**
 * A task to measure the elapsed time for LVGL
 * @param data unused
 */
static void tick_task(void *data) {
  (void)data;
  console_print("tick task started\n");

  long n = 0;
  while(1) {
    xSemaphoreTake(GLOBAL_LVGL_MUTEX, portMAX_DELAY);
    // not accurate if this task was delayed
    lv_tick_inc(5);
    xSemaphoreGive(GLOBAL_LVGL_MUTEX);

    vTaskDelay(pdMS_TO_TICKS(5));
    if (++n % 500 == 0){
      console_print("lvgl tick %ld\n", n);
    }
  }
}

static void timer_task(void* data) {
  (void)data;
  console_print("init/timer task started\n");

  /*Initialize LVGL*/
  lv_init();
  lv_log_register_print_cb(&lvgl_log_callback);

  /*Initialize the HAL (display, input devices, tick) for LVGL*/
  hal_init();

  ui_main();

  long n = 0;
  while(1) {
    xSemaphoreTake(GLOBAL_LVGL_MUTEX, portMAX_DELAY);
    lv_timer_handler();
    xSemaphoreGive(GLOBAL_LVGL_MUTEX);

    vTaskDelay(pdMS_TO_TICKS(5));
    if (++n % 500 == 0) {
      console_print("lvgl timer handler call count %ld\n", n);
    }
  }
}
