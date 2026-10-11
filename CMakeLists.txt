# CMakeLists.txt - G2FD gs_usb firmware (AT32F456, ARM GCC build)
#
# Mirrors the Keil MDK project in mdk_v5/ (same sources, same defines).
# The GCC startup file is used here; the MDK startup file is ARMCC syntax.
#
# Build:
#   cmake -B gcc/build          # add -DARM_GCC_TOOLCHAIN_DIR=... if not on PATH
#   cmake --build gcc/build
#
# Outputs: gcc/build/g2fd.elf / .hex / .bin
#
# Flashing: see README.md (the image is linked at 0x08000000 and overwrites
# the on-board bootloader).

cmake_minimum_required(VERSION 3.15)
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/arm-gcc.cmake" CACHE STRING "ARM GCC toolchain file")
project(g2fd LANGUAGES C ASM)

# ---------------------------------------------------------------- CPU / flags
set(CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

add_compile_options(${CPU_FLAGS}
    -Wall
    -ffunction-sections
    -fdata-sections
)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)

add_compile_definitions(
    AT32F456RET7
    USE_STDPERIPH_DRIVER
    AT_START_F456_V1
)

include_directories(
    inc
    libraries/cmsis/cm4/core_support
    libraries/cmsis/cm4/device_support
    libraries/drivers/inc
    middlewares/usb_drivers/inc
    middlewares/usbd_class/winusb
)

# ------------------------------------------------------------------- sources
# Application (src/ + board files)
set(APP_SOURCES
    src/at32f45x_int.c
    src/at32f45x_clock.c
    src/at32f45x_board.c
    src/main.c
    src/bsp_can.c
    src/bsp_can_init.c
    src/bsp_usb.c
    src/can_ring.c
    src/gs_usb_protocol.c
    src/led.c
)

# Artery peripheral driver library
set(DRIVER_SOURCES
    libraries/drivers/src/at32f45x_gpio.c
    libraries/drivers/src/at32f45x_misc.c
    libraries/drivers/src/at32f45x_crm.c
    libraries/drivers/src/at32f45x_usart.c
    libraries/drivers/src/at32f45x_tmr.c
    libraries/drivers/src/at32f45x_dma.c
    libraries/drivers/src/at32f45x_adc.c
    libraries/drivers/src/at32f45x_can.c
    libraries/drivers/src/at32f45x_pwc.c
    libraries/drivers/src/at32f45x_flash.c
    libraries/drivers/src/at32f45x_usb.c
    libraries/drivers/src/at32f45x_acc.c
    libraries/drivers/src/at32f45x_exint.c
    libraries/drivers/src/at32f45x_scfg.c
)

# CMSIS device support (GCC startup file, not the MDK one)
set(CMSIS_SOURCES
    libraries/cmsis/cm4/device_support/system_at32f45x.c
    libraries/cmsis/cm4/device_support/startup/gcc/startup_at32f45x.s
)

# Artery USB device stack + gs_usb (candleLight) class
set(USB_SOURCES
    middlewares/usb_drivers/src/usb_core.c
    middlewares/usb_drivers/src/usbd_core.c
    middlewares/usb_drivers/src/usbd_int.c
    middlewares/usb_drivers/src/usbd_sdr.c
    middlewares/usbd_class/winusb/winusb_class.c
    middlewares/usbd_class/winusb/winusb_desc.c
)

# ------------------------------------------------------------------- target
add_executable(${PROJECT_NAME}.elf
    ${APP_SOURCES}
    ${DRIVER_SOURCES}
    ${CMSIS_SOURCES}
    ${USB_SOURCES}
)

set(LD_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/gcc/g2fd_at32f456.ld")

target_link_options(${PROJECT_NAME}.elf PRIVATE
    ${CPU_FLAGS}
    -T ${LD_SCRIPT}
    -Wl,--gc-sections
    -Wl,-Map=${PROJECT_NAME}.map
    --specs=nano.specs
    -Wl,--cref
)

# Keil build uses -O1 (<Optim>1</Optim>); keep GCC at -Og for debuggability,
# override with -DCMAKE_C_FLAGS_RELEASE="-O2 -g" if size matters more.
set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-O2 -g")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ----------------------------------------------------- post-build artifacts
if(CMAKE_OBJCOPY)
    add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
        COMMENT "Generating ${PROJECT_NAME}.hex / ${PROJECT_NAME}.bin"
        VERBATIM
    )
endif()
if(CMAKE_SIZE)
    add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
        COMMENT "Size:"
        VERBATIM
    )
endif()
