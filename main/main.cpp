#include <stdio.h>
#include "esp32_s3_szp.h"
#include "logo_en_240x240_lcd.h"
#include "zibo.h"
#include "MCyaping.h"
#include "demos/lv_demos.h"
#include "../components/LVGL_Face/generated/gui_guider.h"
#include "../lvgl__lvgl/lvgl.h"
#include "who_human_face_recognition.hpp"

lv_ui guider_ui;
extern "C" void app_main(void)
{
    bsp_i2c_init();
    pca9557_init();
    // lcd_init();  // 液晶屏初始化
    button_Init();
    // lcd_draw_picture(0, 0, 320, 240, gImage_zibo);
    bsp_lvgl_start();
    bsp_camera_init(PIXFORMAT_RGB565, FRAMESIZE_240X240, 20);         //第三个参数为2，表示有两个帧的缓存
    setup_ui(&guider_ui);
    app_camera_ai_lcd(); 
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // 保持主任务存活
    }


}