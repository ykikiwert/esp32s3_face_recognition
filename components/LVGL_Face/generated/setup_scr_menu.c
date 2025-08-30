/*
* Copyright 2025 NXP
* NXP Confidential and Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "lvgl.h"
#include <stdio.h>
#include "gui_guider.h"
#include "events_init.h"
#include "widgets_init.h"
#include "custom.h"


void setup_scr_menu(lv_ui *ui)
{
	//Write codes menu
	ui->menu = lv_obj_create(NULL);
	lv_obj_set_size(ui->menu, 320, 240);

	//Write style for menu, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_bg_opa(ui->menu, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_img_src(ui->menu, &_facebalk_320x240, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_bg_img_opa(ui->menu, 255, LV_PART_MAIN|LV_STATE_DEFAULT);

	//Write codes menu_imgbtn_1
	ui->menu_imgbtn_1 = lv_imgbtn_create(ui->menu);
	lv_obj_add_flag(ui->menu_imgbtn_1, LV_OBJ_FLAG_CHECKABLE);
	lv_imgbtn_set_src(ui->menu_imgbtn_1, LV_IMGBTN_STATE_RELEASED, NULL, &_face_alpha_37x44, NULL);
	ui->menu_imgbtn_1_label = lv_label_create(ui->menu_imgbtn_1);
	lv_label_set_text(ui->menu_imgbtn_1_label, "");
	lv_label_set_long_mode(ui->menu_imgbtn_1_label, LV_LABEL_LONG_WRAP);
	lv_obj_align(ui->menu_imgbtn_1_label, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_pad_all(ui->menu_imgbtn_1, 0, LV_STATE_DEFAULT);
	lv_obj_set_pos(ui->menu_imgbtn_1, 272, 50);
	lv_obj_set_size(ui->menu_imgbtn_1, 37, 44);

	//Write style for menu_imgbtn_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->menu_imgbtn_1, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->menu_imgbtn_1, &lv_font_montserratMedium_12, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->menu_imgbtn_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->menu_imgbtn_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

	//Write style for menu_imgbtn_1, Part: LV_PART_MAIN, State: LV_STATE_PRESSED.
	lv_obj_set_style_img_opa(ui->menu_imgbtn_1, 255, LV_PART_MAIN|LV_STATE_PRESSED);
	lv_obj_set_style_text_color(ui->menu_imgbtn_1, lv_color_hex(0xFF33FF), LV_PART_MAIN|LV_STATE_PRESSED);
	lv_obj_set_style_text_font(ui->menu_imgbtn_1, &lv_font_montserratMedium_12, LV_PART_MAIN|LV_STATE_PRESSED);
	lv_obj_set_style_shadow_width(ui->menu_imgbtn_1, 0, LV_PART_MAIN|LV_STATE_PRESSED);

	//Write style for menu_imgbtn_1, Part: LV_PART_MAIN, State: LV_STATE_CHECKED.
	lv_obj_set_style_img_opa(ui->menu_imgbtn_1, 255, LV_PART_MAIN|LV_STATE_CHECKED);
	lv_obj_set_style_text_color(ui->menu_imgbtn_1, lv_color_hex(0xFF33FF), LV_PART_MAIN|LV_STATE_CHECKED);
	lv_obj_set_style_text_font(ui->menu_imgbtn_1, &lv_font_montserratMedium_12, LV_PART_MAIN|LV_STATE_CHECKED);
	lv_obj_set_style_shadow_width(ui->menu_imgbtn_1, 0, LV_PART_MAIN|LV_STATE_CHECKED);

	//Write style for menu_imgbtn_1, Part: LV_PART_MAIN, State: LV_IMGBTN_STATE_RELEASED.
	lv_obj_set_style_img_opa(ui->menu_imgbtn_1, 255, LV_PART_MAIN|LV_IMGBTN_STATE_RELEASED);

	//Write codes menu_imgbtn_2
	ui->menu_imgbtn_2 = lv_imgbtn_create(ui->menu);
	lv_obj_add_flag(ui->menu_imgbtn_2, LV_OBJ_FLAG_CHECKABLE);
	lv_imgbtn_set_src(ui->menu_imgbtn_2, LV_IMGBTN_STATE_RELEASED, NULL, &_save_alpha_37x44, NULL);
	ui->menu_imgbtn_2_label = lv_label_create(ui->menu_imgbtn_2);
	lv_label_set_text(ui->menu_imgbtn_2_label, "");
	lv_label_set_long_mode(ui->menu_imgbtn_2_label, LV_LABEL_LONG_WRAP);
	lv_obj_align(ui->menu_imgbtn_2_label, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_style_pad_all(ui->menu_imgbtn_2, 0, LV_STATE_DEFAULT);
	lv_obj_set_pos(ui->menu_imgbtn_2, 272, 157);
	lv_obj_set_size(ui->menu_imgbtn_2, 37, 44);

	//Write style for menu_imgbtn_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
	lv_obj_set_style_text_color(ui->menu_imgbtn_2, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(ui->menu_imgbtn_2, &lv_font_montserratMedium_12, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_text_align(ui->menu_imgbtn_2, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
	lv_obj_set_style_shadow_width(ui->menu_imgbtn_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

	//Write style for menu_imgbtn_2, Part: LV_PART_MAIN, State: LV_STATE_PRESSED.
	lv_obj_set_style_img_opa(ui->menu_imgbtn_2, 255, LV_PART_MAIN|LV_STATE_PRESSED);
	lv_obj_set_style_text_color(ui->menu_imgbtn_2, lv_color_hex(0xFF33FF), LV_PART_MAIN|LV_STATE_PRESSED);
	lv_obj_set_style_text_font(ui->menu_imgbtn_2, &lv_font_montserratMedium_12, LV_PART_MAIN|LV_STATE_PRESSED);
	lv_obj_set_style_shadow_width(ui->menu_imgbtn_2, 0, LV_PART_MAIN|LV_STATE_PRESSED);

	//Write style for menu_imgbtn_2, Part: LV_PART_MAIN, State: LV_STATE_CHECKED.
	lv_obj_set_style_img_opa(ui->menu_imgbtn_2, 255, LV_PART_MAIN|LV_STATE_CHECKED);
	lv_obj_set_style_text_color(ui->menu_imgbtn_2, lv_color_hex(0xFF33FF), LV_PART_MAIN|LV_STATE_CHECKED);
	lv_obj_set_style_text_font(ui->menu_imgbtn_2, &lv_font_montserratMedium_12, LV_PART_MAIN|LV_STATE_CHECKED);
	lv_obj_set_style_shadow_width(ui->menu_imgbtn_2, 0, LV_PART_MAIN|LV_STATE_CHECKED);

	//Write style for menu_imgbtn_2, Part: LV_PART_MAIN, State: LV_IMGBTN_STATE_RELEASED.
	lv_obj_set_style_img_opa(ui->menu_imgbtn_2, 255, LV_PART_MAIN|LV_IMGBTN_STATE_RELEASED);

	//Update current screen layout.
	lv_obj_update_layout(ui->menu);

	
	//Init events for screen.
	events_init_menu(ui);
}
