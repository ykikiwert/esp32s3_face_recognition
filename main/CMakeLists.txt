idf_component_register(
    SRCS 
        "who_human_face_recognition.cpp" 
        "who_ai_utils.cpp" 
        "esp32_s3_szp.c" 
        "main.cpp"
       
    PRIV_REQUIRES 
        spi_flash 
        driver
        esp_lcd
        lvgl          # 确保 LVGL 依赖
        esp_lvgl_port  # 如果需要 LVGL 官方集成组件

    INCLUDE_DIRS 
        "."
)