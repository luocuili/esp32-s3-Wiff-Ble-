# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/ESP32/Espressif/frameworks/esp-idf-v5.3.1/components/bootloader/subproject"
  "E:/Microsoft_LCL/Snap-POC-Dev/esp32-s3-Wiff-Ble/build/bootloader"
  "E:/Microsoft_LCL/Snap-POC-Dev/esp32-s3-Wiff-Ble/build/bootloader-prefix"
  "E:/Microsoft_LCL/Snap-POC-Dev/esp32-s3-Wiff-Ble/build/bootloader-prefix/tmp"
  "E:/Microsoft_LCL/Snap-POC-Dev/esp32-s3-Wiff-Ble/build/bootloader-prefix/src/bootloader-stamp"
  "E:/Microsoft_LCL/Snap-POC-Dev/esp32-s3-Wiff-Ble/build/bootloader-prefix/src"
  "E:/Microsoft_LCL/Snap-POC-Dev/esp32-s3-Wiff-Ble/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/Microsoft_LCL/Snap-POC-Dev/esp32-s3-Wiff-Ble/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "E:/Microsoft_LCL/Snap-POC-Dev/esp32-s3-Wiff-Ble/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
