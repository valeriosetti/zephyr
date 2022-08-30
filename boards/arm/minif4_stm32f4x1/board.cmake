# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=STM32F401CC" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

# set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-DUSE_HAL_DRIVER")
# set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-DSTM32F401xC")
