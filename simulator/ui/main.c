#include "lvgl/lvgl.h"
#include "screens/home_screen.h"

void ui_main(void)
{
    ui_home_screen_create(lv_scr_act());
}
