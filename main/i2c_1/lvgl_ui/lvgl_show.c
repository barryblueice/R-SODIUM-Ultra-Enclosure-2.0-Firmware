#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

#include "lvgl_init.h"
#include "init_func/init_func.h"

LV_FONT_DECLARE(pixelmplus);
LV_IMG_DECLARE(logo);

lv_obj_t *labels[5];

typedef struct {
    int label_index;
    char new_text[64];
} label_update_msg_t;

QueueHandle_t label_update_queue;

void update_label_text(int label_index, const char *new_text) {
    if (label_update_queue != NULL) {
        label_update_msg_t msg;
        msg.label_index = label_index;
        strncpy(msg.new_text, new_text, sizeof(msg.new_text) - 1);
        msg.new_text[sizeof(msg.new_text) - 1] = '\0';
        
        xQueueSend(label_update_queue, &msg, portMAX_DELAY);
    }
}

void update_labels_task(lv_timer_t *timer) {
    label_update_msg_t msg;
    
    while (xQueueReceive(label_update_queue, &msg, 0) == pdTRUE) {
        if (msg.label_index >= 0 && msg.label_index < 5) {
            lv_label_set_text(labels[msg.label_index], msg.new_text);
        }
    }
}

void _main_gui(lv_display_t *disp) {
    lv_coord_t w = lv_display_get_horizontal_resolution(disp);
    lv_coord_t h = lv_display_get_vertical_resolution(disp);

    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cont, w, h);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_pad_all(cont, 0, 0);
    lv_obj_set_style_pad_row(cont, 0, 0);

    const char *texts[] = {
        "",
        "",
        "",
        "",
        ""
    };
    for (int i = 0; i < 5; i++) {
        labels[i] = lv_label_create(cont);
        lv_label_set_long_mode(labels[i], LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_width(labels[i], w - 2);
        lv_label_set_text(labels[i], texts[i]);
        lv_obj_set_style_text_font(labels[i], &pixelmplus, 0);
        lv_obj_set_style_text_line_space(labels[i], 0, 0);
    }

    label_update_queue = xQueueCreate(10, sizeof(label_update_msg_t));
    lv_timer_create(update_labels_task, 50, NULL);
}

void _startup_logo(lv_display_t *disp) {
    lv_obj_t *img = lv_img_create(lv_scr_act());
    lv_img_set_src(img, &logo);

    lv_coord_t disp_w = lv_display_get_horizontal_resolution(disp);
    lv_coord_t disp_h = lv_display_get_vertical_resolution(disp);

    uint16_t scale_x = (uint16_t)((256.0 * disp_w) / logo.header.w);
    uint16_t scale_y = (uint16_t)((256.0 * disp_h) / logo.header.h);
    uint16_t scale = scale_x < scale_y ? scale_x : scale_y;

    lv_image_set_scale(img, scale);
    lv_obj_center(img);
}

void update_ui(int index, const char *text) {
    if (index < 0 || index >= 5) return;
    lv_label_set_text(labels[index], text);
    lv_obj_invalidate(labels[index]);
}