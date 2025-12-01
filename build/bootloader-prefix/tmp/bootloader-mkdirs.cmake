# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Espressif/frameworks/esp-idf-v5.3.1/components/bootloader/subproject"
  "E:/Learning materials/Extended routine-IDF(v5.3.x)/WiFi/03_WiFi_AP/build/bootloader"
  "E:/Learning materials/Extended routine-IDF(v5.3.x)/WiFi/03_WiFi_AP/build/bootloader-prefix"
  "E:/Learning materials/Extended routine-IDF(v5.3.x)/WiFi/03_WiFi_AP/build/bootloader-prefix/tmp"
  "E:/Learning materials/Extended routine-IDF(v5.3.x)/WiFi/03_WiFi_AP/build/bootloader-prefix/src/bootloader-stamp"
  "E:/Learning materials/Extended routine-IDF(v5.3.x)/WiFi/03_WiFi_AP/build/bootloader-prefix/src"
  "E:/Learning materials/Extended routine-IDF(v5.3.x)/WiFi/03_WiFi_AP/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "E:/Learning materials/Extended routine-IDF(v5.3.x)/WiFi/03_WiFi_AP/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "E:/Learning materials/Extended routine-IDF(v5.3.x)/WiFi/03_WiFi_AP/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
