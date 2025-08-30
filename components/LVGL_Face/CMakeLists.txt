set(UI_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/)

file(GLOB_RECURSE SOURCES ${UI_SOURCE_DIR}/custom/*.c ${UI_SOURCE_DIR}/generated/*.c ${UI_SOURCE_DIR}/generated/guider_fonts/*.c ${UI_SOURCE_DIR}/generated/images/*.c)

set(include_dirs 
    ${UI_SOURCE_DIR}/custom
    ${UI_SOURCE_DIR}/generated
    ${UI_SOURCE_DIR}/generated/guider_customer_fonts
    ${UI_SOURCE_DIR}/generated/guider_fonts   
    # ${CMAKE_CURRENT_LIST_DIR}/../lvgl
    # ${CMAKE_CURRENT_LIST_DIR}/../lvgl/src/font  
    
    )
idf_component_register(SRCS ${SOURCES}
                    PRIV_REQUIRES
                    spi_flash 
                    driver
                    esp_lcd
                    lvgl          # 确保 LVGL 依赖
                    esp_lvgl_port  # 如果需要 LVGL 官方集成组件
                    esp32-camera
                    esp_lcd_touch_ft5x06
                    esp_lcd_touch

                    INCLUDE_DIRS ${include_dirs}
                    REQUIRES lvgl main)
