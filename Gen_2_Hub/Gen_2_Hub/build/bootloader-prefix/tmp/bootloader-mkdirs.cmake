# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/1xlr8/esp/esp-idf-v5.1.4/components/bootloader/subproject"
  "C:/Users/1xlr8/Desktop/ASDL/ASDL_Gen2.0_Arduino/Gen_2_Hub/Gen_2_Hub/build/bootloader"
  "C:/Users/1xlr8/Desktop/ASDL/ASDL_Gen2.0_Arduino/Gen_2_Hub/Gen_2_Hub/build/bootloader-prefix"
  "C:/Users/1xlr8/Desktop/ASDL/ASDL_Gen2.0_Arduino/Gen_2_Hub/Gen_2_Hub/build/bootloader-prefix/tmp"
  "C:/Users/1xlr8/Desktop/ASDL/ASDL_Gen2.0_Arduino/Gen_2_Hub/Gen_2_Hub/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/1xlr8/Desktop/ASDL/ASDL_Gen2.0_Arduino/Gen_2_Hub/Gen_2_Hub/build/bootloader-prefix/src"
  "C:/Users/1xlr8/Desktop/ASDL/ASDL_Gen2.0_Arduino/Gen_2_Hub/Gen_2_Hub/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/1xlr8/Desktop/ASDL/ASDL_Gen2.0_Arduino/Gen_2_Hub/Gen_2_Hub/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/1xlr8/Desktop/ASDL/ASDL_Gen2.0_Arduino/Gen_2_Hub/Gen_2_Hub/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
