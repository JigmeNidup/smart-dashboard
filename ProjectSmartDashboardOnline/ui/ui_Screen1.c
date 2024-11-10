// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.2
// LVGL version: 8.3.11
// Project name: SquareLine_Project_ESP32_DASHBOARD

#include "ui.h"

void ui_Screen1_screen_init(void)
{
    ui_Screen1 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_HomePanel = lv_obj_create(ui_Screen1);
    lv_obj_set_width(ui_HomePanel, lv_pct(100));
    lv_obj_set_height(ui_HomePanel, lv_pct(100));
    lv_obj_set_align(ui_HomePanel, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_HomePanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_TemperaturePanel = lv_obj_create(ui_HomePanel);
    lv_obj_set_height(ui_TemperaturePanel, 125);
    lv_obj_set_width(ui_TemperaturePanel, lv_pct(50));
    lv_obj_clear_flag(ui_TemperaturePanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_TemperaturePanel, lv_color_hex(0x007CFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_TemperaturePanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_TemperatureImage = lv_img_create(ui_TemperaturePanel);
    lv_img_set_src(ui_TemperatureImage, &ui_img_temp_icon_png);
    lv_obj_set_width(ui_TemperatureImage, LV_SIZE_CONTENT);   /// 100
    lv_obj_set_height(ui_TemperatureImage, LV_SIZE_CONTENT);    /// 100
    lv_obj_set_align(ui_TemperatureImage, LV_ALIGN_LEFT_MID);
    lv_obj_add_flag(ui_TemperatureImage, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_TemperatureImage, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_TempValPanel = lv_obj_create(ui_TemperaturePanel);
    lv_obj_remove_style_all(ui_TempValPanel);
    lv_obj_set_width(ui_TempValPanel, 100);
    lv_obj_set_height(ui_TempValPanel, 100);
    lv_obj_set_align(ui_TempValPanel, LV_ALIGN_RIGHT_MID);
    lv_obj_clear_flag(ui_TempValPanel, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_tempVal = lv_label_create(ui_TempValPanel);
    lv_obj_set_width(ui_tempVal, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_tempVal, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_tempVal, LV_ALIGN_CENTER);
    lv_label_set_text(ui_tempVal, "0 °C");
    lv_obj_set_style_text_font(ui_tempVal, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tempLabel = lv_label_create(ui_TempValPanel);
    lv_obj_set_width(ui_tempLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_tempLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_tempLabel, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_tempLabel, "Temperature");

    ui_HumidityPanel = lv_obj_create(ui_HomePanel);
    lv_obj_set_height(ui_HumidityPanel, 125);
    lv_obj_set_width(ui_HumidityPanel, lv_pct(50));
    lv_obj_set_align(ui_HumidityPanel, LV_ALIGN_TOP_RIGHT);
    lv_obj_clear_flag(ui_HumidityPanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_HumidityPanel, lv_color_hex(0x007CFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_HumidityPanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_HumidityImage = lv_img_create(ui_HumidityPanel);
    lv_img_set_src(ui_HumidityImage, &ui_img_img_humi_png);
    lv_obj_set_width(ui_HumidityImage, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_HumidityImage, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_HumidityImage, -53);
    lv_obj_set_y(ui_HumidityImage, 1);
    lv_obj_set_align(ui_HumidityImage, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_HumidityImage, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_HumidityImage, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_HumidityContainer = lv_obj_create(ui_HumidityPanel);
    lv_obj_remove_style_all(ui_HumidityContainer);
    lv_obj_set_width(ui_HumidityContainer, 110);
    lv_obj_set_height(ui_HumidityContainer, 108);
    lv_obj_set_align(ui_HumidityContainer, LV_ALIGN_RIGHT_MID);
    lv_obj_clear_flag(ui_HumidityContainer, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_humiArc = lv_arc_create(ui_HumidityContainer);
    lv_obj_set_width(ui_humiArc, 100);
    lv_obj_set_height(ui_humiArc, 100);
    lv_obj_set_x(ui_humiArc, 0);
    lv_obj_set_y(ui_humiArc, 12);
    lv_obj_set_align(ui_humiArc, LV_ALIGN_RIGHT_MID);
    lv_arc_set_value(ui_humiArc, 0);

    lv_obj_set_style_bg_color(ui_humiArc, lv_color_hex(0x0094FF), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_humiArc, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_humiArc, lv_color_hex(0x0094FF), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_humiArc, 255, LV_PART_KNOB | LV_STATE_DEFAULT);

    ui_humiVal = lv_label_create(ui_humiArc);
    lv_obj_set_width(ui_humiVal, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_humiVal, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_humiVal, LV_ALIGN_CENTER);
    lv_label_set_text(ui_humiVal, "0 %");
    lv_obj_set_style_text_font(ui_humiVal, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_HumidityLabel = lv_label_create(ui_HumidityContainer);
    lv_obj_set_width(ui_HumidityLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_HumidityLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_HumidityLabel, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_HumidityLabel, "Humidity");

    ui_SpeakerPanel = lv_obj_create(ui_HomePanel);
    lv_obj_set_height(ui_SpeakerPanel, 150);
    lv_obj_set_width(ui_SpeakerPanel, lv_pct(30));
    lv_obj_set_align(ui_SpeakerPanel, LV_ALIGN_BOTTOM_RIGHT);
    lv_obj_clear_flag(ui_SpeakerPanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_SpeakerPanel, lv_color_hex(0x007CFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SpeakerPanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_lightON = lv_img_create(ui_SpeakerPanel);
    lv_img_set_src(ui_lightON, &ui_img_bulb_on_png);
    lv_obj_set_width(ui_lightON, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lightON, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lightON, 0);
    lv_obj_set_y(ui_lightON, -25);
    lv_obj_set_align(ui_lightON, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_lightON, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_lightON, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_SwitchRelay = lv_switch_create(ui_SpeakerPanel);
    lv_obj_set_width(ui_SwitchRelay, 80);
    lv_obj_set_height(ui_SwitchRelay, 40);
    lv_obj_set_x(ui_SwitchRelay, 0);
    lv_obj_set_y(ui_SwitchRelay, 35);
    lv_obj_set_align(ui_SwitchRelay, LV_ALIGN_CENTER);


    ui_lightOFF = lv_img_create(ui_SpeakerPanel);
    lv_img_set_src(ui_lightOFF, &ui_img_bulb_off_png);
    lv_obj_set_width(ui_lightOFF, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lightOFF, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lightOFF, 0);
    lv_obj_set_y(ui_lightOFF, -25);
    lv_obj_set_align(ui_lightOFF, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_lightOFF, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_lightOFF, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_CO2Panel = lv_obj_create(ui_HomePanel);
    lv_obj_set_height(ui_CO2Panel, 150);
    lv_obj_set_width(ui_CO2Panel, lv_pct(35));
    lv_obj_set_x(ui_CO2Panel, 11);
    lv_obj_set_y(ui_CO2Panel, 0);
    lv_obj_set_align(ui_CO2Panel, LV_ALIGN_BOTTOM_MID);
    lv_obj_clear_flag(ui_CO2Panel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_CO2Panel, lv_color_hex(0x007CFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_CO2Panel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_mq9Arc = lv_arc_create(ui_CO2Panel);
    lv_obj_set_width(ui_mq9Arc, 120);
    lv_obj_set_height(ui_mq9Arc, 120);
    lv_obj_set_x(ui_mq9Arc, 0);
    lv_obj_set_y(ui_mq9Arc, 15);
    lv_obj_set_align(ui_mq9Arc, LV_ALIGN_CENTER);
    lv_arc_set_range(ui_mq9Arc, 0, 4096);
    lv_arc_set_value(ui_mq9Arc, 0);

    lv_obj_set_style_bg_color(ui_mq9Arc, lv_color_hex(0x00DEFF), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_mq9Arc, 255, LV_PART_KNOB | LV_STATE_DEFAULT);

    ui_mq9Val = lv_label_create(ui_mq9Arc);
    lv_obj_set_width(ui_mq9Val, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_mq9Val, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_mq9Val, LV_ALIGN_CENTER);
    lv_label_set_text(ui_mq9Val, "0 PPM");

    ui_CO2Label = lv_label_create(ui_CO2Panel);
    lv_obj_set_width(ui_CO2Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_CO2Label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_CO2Label, 0);
    lv_obj_set_y(ui_CO2Label, -8);
    lv_obj_set_align(ui_CO2Label, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_CO2Label, "CO2");

    ui_AirQualityPanel = lv_obj_create(ui_HomePanel);
    lv_obj_set_height(ui_AirQualityPanel, 150);
    lv_obj_set_width(ui_AirQualityPanel, lv_pct(35));
    lv_obj_set_align(ui_AirQualityPanel, LV_ALIGN_BOTTOM_LEFT);
    lv_obj_clear_flag(ui_AirQualityPanel, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_AirQualityPanel, lv_color_hex(0x007CFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_AirQualityPanel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_mq135Arc = lv_arc_create(ui_AirQualityPanel);
    lv_obj_set_width(ui_mq135Arc, 120);
    lv_obj_set_height(ui_mq135Arc, 120);
    lv_obj_set_x(ui_mq135Arc, 0);
    lv_obj_set_y(ui_mq135Arc, 15);
    lv_obj_set_align(ui_mq135Arc, LV_ALIGN_CENTER);
    lv_arc_set_range(ui_mq135Arc, 0, 4096);
    lv_arc_set_value(ui_mq135Arc, 0);

    lv_obj_set_style_bg_color(ui_mq135Arc, lv_color_hex(0x00FFD1), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_mq135Arc, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_mq135Arc, lv_color_hex(0x00FFD1), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_mq135Arc, 255, LV_PART_KNOB | LV_STATE_DEFAULT);

    ui_mq135Val = lv_label_create(ui_mq135Arc);
    lv_obj_set_width(ui_mq135Val, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_mq135Val, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_mq135Val, LV_ALIGN_CENTER);
    lv_label_set_text(ui_mq135Val, "0 PPM");

    ui_AirQualityLabel = lv_label_create(ui_AirQualityPanel);
    lv_obj_set_width(ui_AirQualityLabel, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_AirQualityLabel, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_AirQualityLabel, 0);
    lv_obj_set_y(ui_AirQualityLabel, -8);
    lv_obj_set_align(ui_AirQualityLabel, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_AirQualityLabel, "Air Quality");

    lv_obj_add_event_cb(ui_humiArc, ui_event_humiArc, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_SwitchRelay, ui_event_SwitchRelay, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_mq9Arc, ui_event_mq9Arc, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_mq135Arc, ui_event_mq135Arc, LV_EVENT_ALL, NULL);

}
