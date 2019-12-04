# Install script for directory: C:/ncs.sojkk/zephyr

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/Zephyr-Kernel")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/arch/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/lib/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/soc/arm/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/boards/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/ext/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/subsys/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/drivers/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/nrf/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/nffs/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/segger/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/mbedtls/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/mcuboot/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/mcumgr/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/tinycbor/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/nrfxlib/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/civetweb/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/fatfs/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/nordic/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/st/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/libmetal/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/lvgl/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/open-amp/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/openthread/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/modules/littlefs/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/kernel/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/cmake/flash/cmake_install.cmake")
  include("C:/ncs.sojkk/nrf/training_and_handson/hello_world/build_nrf52840_pca10056/zephyr/cmake/reports/cmake_install.cmake")

endif()

