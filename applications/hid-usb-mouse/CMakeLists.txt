# SPDX-License-Identifier: Apache-2.0

set(REMOTE_ZEPHYR_DIR ${CMAKE_CURRENT_BINARY_DIR}/rpmsg_service_remote-prefix/src/rpmsg_service_remote-build/zephyr)

if("${BOARD}" STREQUAL "nrf5340dk_nrf5340_cpuapp")
  set(BOARD_REMOTE "nrf5340dk_nrf5340_cpunet")
endif()

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hid-usb-mouse)

include(ExternalProject)

ExternalProject_Add(
  rpmsg_service_remote
  SOURCE_DIR ${APPLICATION_SOURCE_DIR}/remote
  INSTALL_COMMAND ""      # This particular build system has no install command
  CMAKE_CACHE_ARGS -DBOARD:STRING=${BOARD_REMOTE}
  CMAKE_CACHE_ARGS -DDTC_OVERLAY_FILE:STRING=${DTC_OVERLAY_FILE}
  BUILD_BYPRODUCTS "${REMOTE_ZEPHYR_DIR}/${KERNEL_BIN_NAME}"
  # NB: Do we need to pass on more CMake variables?
  BUILD_ALWAYS True
)


FILE(GLOB app_sources src/main.c)
target_sources(app PRIVATE ${app_sources})
target_include_directories(app PRIVATE lib)