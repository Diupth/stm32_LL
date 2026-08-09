cmake_minimum_required(VERSION 3.22)

# Set compile standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Project Name
set(CMAKE_PROJECT_NAME Low_layer)
project(${CMAKE_PROJECT_NAME} C ASM)

# Enable export compile commands for tooling (clangd, etc.)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Definitions
add_compile_definitions(
    STM32H562xx
    USE_HAL_DRIVER
    CFG_TUSB_MCU=OPT_MCU_STM32H5
)

# Source files
set(SOURCES
    # Core sources
    Core/Src/main.c
    Core/Startup/system_stm32h5xx.c
    Core/Startup/startup_stm32h562xx.s
    Core/Src/ComMgr.c

    # ST HAL Driver sources needed for USB and minimal GPIO/RCC
    Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal.c
    Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_cortex.c
    Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rcc.c
    Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rcc_ex.c
    Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_gpio.c
    Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pcd.c
    Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pcd_ex.c
    Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_ll_usb.c
    Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_dma.c
    Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_dma_ex.c
    Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pwr.c
    Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pwr_ex.c
    Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_flash.c
    Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_flash_ex.c
    Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_exti.c

    # TinyUSB sources
    Drivers/tinyusb/src/tusb.c
    Drivers/tinyusb/src/common/tusb_fifo.c
    Drivers/tinyusb/src/device/usbd.c
    Drivers/tinyusb/src/class/cdc/cdc_device.c
    Drivers/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
    Drivers/tinyusb/src/portable/st/stm32_fsdev/fsdev_common.c
)

# Include directories
set(INC_DIRS
    Core/Inc
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32H5xx/Include
    Drivers/STM32H5xx_HAL_Driver/Inc
    Drivers/STM32H5xx_HAL_Driver/Inc/Legacy
    Drivers/tinyusb/src
)

# Target executable
add_executable(${CMAKE_PROJECT_NAME} ${SOURCES})

# Include paths for target
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${INC_DIRS})

# Link libraries
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE m)

# Auto generate hex/bin files after build
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generating .bin and .hex output files..."
)
