SET(SOURCES ui_comp_item.c
    ui_comp.c
    ui_Screen1.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_img_temp_icon_png.c
    ui_img_img_humi_png.c
    ui_img_bulb_on_png.c
    ui_img_bulb_off_png.c)

add_library(ui ${SOURCES})
