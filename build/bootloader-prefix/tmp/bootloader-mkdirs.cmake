# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Software/ESPIDF/ESPIDF5.1.4/Espressif/frameworks/esp-idf-v5.1.4/components/bootloader/subproject"
  "D:/00project/00ESP32/01Face_recognition/face/03Lcd_Lvgl_key_faceyeska/build/bootloader"
  "D:/00project/00ESP32/01Face_recognition/face/03Lcd_Lvgl_key_faceyeska/build/bootloader-prefix"
  "D:/00project/00ESP32/01Face_recognition/face/03Lcd_Lvgl_key_faceyeska/build/bootloader-prefix/tmp"
  "D:/00project/00ESP32/01Face_recognition/face/03Lcd_Lvgl_key_faceyeska/build/bootloader-prefix/src/bootloader-stamp"
  "D:/00project/00ESP32/01Face_recognition/face/03Lcd_Lvgl_key_faceyeska/build/bootloader-prefix/src"
  "D:/00project/00ESP32/01Face_recognition/face/03Lcd_Lvgl_key_faceyeska/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/00project/00ESP32/01Face_recognition/face/03Lcd_Lvgl_key_faceyeska/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/00project/00ESP32/01Face_recognition/face/03Lcd_Lvgl_key_faceyeska/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
