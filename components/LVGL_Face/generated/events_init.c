/*
* Copyright 2025 NXP
* NXP Confidential and Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "events_init.h"
#include <stdio.h>
#include "lvgl.h"
#include "../main/who_human_face_recognition.hpp"

extern QueueHandle_t xQueueEvent;

static void menu_imgbtn_1_event_handler (lv_event_t *e)
{
	lv_event_code_t code = lv_event_get_code(e);

	switch (code) {
	case LV_EVENT_CLICKED:
	{
		lv_obj_set_style_bg_color(guider_ui.menu_imgbtn_1, lv_color_hex(0xdbaeae), LV_PART_MAIN);
		recognizer_state_t state = RECOGNIZE;
                if (xQueueSend(xQueueEvent, &state, 0) != pdPASS) {
                    printf("发送 RECOGNIZE 失败（队列已满）\n");
                } else {
                    printf("已发送 RECOGNIZE 命令\n");
                }
		break;
	}
	default:
		break;
	}
}
static void menu_imgbtn_2_event_handler (lv_event_t *e)
{
	lv_event_code_t code = lv_event_get_code(e);

	switch (code) {
	case LV_EVENT_CLICKED:
	{
		lv_obj_set_style_bg_color(guider_ui.menu_imgbtn_2, lv_color_hex(0x000000), LV_PART_MAIN);
		recognizer_state_t state = ENROLL;
                if (xQueueSend(xQueueEvent, &state, 0) != pdPASS) {
                    printf("发送 ENROLL 失败（队列已满）\n");
                } else {
                    printf("已发送 ENROLL 命令\n");
                }
		break;
	}
	default:
		break;
	}
}
void events_init_menu(lv_ui *ui)
{
	lv_obj_add_event_cb(ui->menu_imgbtn_1, menu_imgbtn_1_event_handler, LV_EVENT_ALL, NULL);
	lv_obj_add_event_cb(ui->menu_imgbtn_2, menu_imgbtn_2_event_handler, LV_EVENT_ALL, NULL);
}

void events_init(lv_ui *ui)
{

}
