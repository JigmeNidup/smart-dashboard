// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.2
// LVGL version: 8.3.11
// Project name: SquareLine_Project_ESP32_DASHBOARD

#include "ui.h"


// COMPONENT item

lv_obj_t * ui_item_create(lv_obj_t * comp_parent)
{

    lv_obj_t * cui_item;
    cui_item = lv_obj_create(comp_parent);
    lv_obj_set_width(cui_item, 189);
    lv_obj_set_height(cui_item, LV_SIZE_CONTENT);    /// 50
    lv_obj_set_x(cui_item, 124);
    lv_obj_set_y(cui_item, -66);
    lv_obj_set_align(cui_item, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(cui_item, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(cui_item, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(cui_item, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    lv_obj_t * cui_name1;
    cui_name1 = lv_label_create(cui_item);
    lv_obj_set_width(cui_name1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(cui_name1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(cui_name1, -56);
    lv_obj_set_y(cui_name1, 0);
    lv_obj_set_align(cui_name1, LV_ALIGN_CENTER);
    lv_label_set_text(cui_name1, "New Jeans are so hot");

    lv_obj_t * cui_price1;
    cui_price1 = lv_label_create(cui_item);
    lv_obj_set_width(cui_price1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(cui_price1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(cui_price1, 77);
    lv_obj_set_y(cui_price1, 1);
    lv_obj_set_align(cui_price1, LV_ALIGN_RIGHT_MID);
    lv_label_set_text(cui_price1, "4");
    lv_obj_set_style_text_color(cui_price1, lv_color_hex(0x808080), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(cui_price1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_t ** children = lv_mem_alloc(sizeof(lv_obj_t *) * _UI_COMP_ITEM_NUM);
    children[UI_COMP_ITEM_ITEM] = cui_item;
    children[UI_COMP_ITEM_NAME1] = cui_name1;
    children[UI_COMP_ITEM_PRICE1] = cui_price1;
    lv_obj_add_event_cb(cui_item, get_component_child_event_cb, LV_EVENT_GET_COMP_CHILD, children);
    lv_obj_add_event_cb(cui_item, del_component_child_event_cb, LV_EVENT_DELETE, children);
    ui_comp_item_create_hook(cui_item);
    return cui_item;
}
