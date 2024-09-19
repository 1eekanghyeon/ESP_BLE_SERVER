# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/HiWi4/esp/v5.3.1/esp-idf/components/bootloader/subproject"
  "C:/leekanghyeon/ESP32/gatt_server_with_image_Real5/build/bootloader"
  "C:/leekanghyeon/ESP32/gatt_server_with_image_Real5/build/bootloader-prefix"
  "C:/leekanghyeon/ESP32/gatt_server_with_image_Real5/build/bootloader-prefix/tmp"
  "C:/leekanghyeon/ESP32/gatt_server_with_image_Real5/build/bootloader-prefix/src/bootloader-stamp"
  "C:/leekanghyeon/ESP32/gatt_server_with_image_Real5/build/bootloader-prefix/src"
  "C:/leekanghyeon/ESP32/gatt_server_with_image_Real5/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/leekanghyeon/ESP32/gatt_server_with_image_Real5/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/leekanghyeon/ESP32/gatt_server_with_image_Real5/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
