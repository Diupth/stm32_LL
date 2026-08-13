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

add_definitions(
    -DSHOW_SAMPLING_LOG
    # -DSHOW_ADC_DAC_DEBUG
)

# Source files
set(SOURCES
    # Application & System sources
    Src/main.c
    Startup/system_stm32h5xx.c
    Startup/startup_stm32h562xx.s

    # Applications
    Src/Apps/Receiver.c
    Src/Apps/Transmitter.c
    Src/Apps/SyncSignalApp.c

    # Services
    Src/Services/ComMgr.c
    Src/Services/SystemClock.c
    Src/Services/SyncSignal.c
    Src/Services/ADCService.c
    Src/Services/DACService.c

    # ST HAL Driver sources needed for USB and minimal GPIO/RCC
    Src/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal.c
    Src/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_cortex.c
    Src/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rcc.c
    Src/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rcc_ex.c
    Src/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_gpio.c
    Src/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pcd.c
    Src/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pcd_ex.c
    Src/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_ll_usb.c
    Src/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_dma.c
    Src/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_dma_ex.c
    Src/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pwr.c
    Src/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pwr_ex.c
    Src/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_flash.c
    Src/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_flash_ex.c
    Src/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_exti.c

    # TinyUSB sources
    Src/Drivers/tinyusb/src/tusb.c
    Src/Drivers/tinyusb/src/common/tusb_fifo.c
    Src/Drivers/tinyusb/src/device/usbd.c
    Src/Drivers/tinyusb/src/class/cdc/cdc_device.c
    Src/Drivers/tinyusb/src/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
    Src/Drivers/tinyusb/src/portable/st/stm32_fsdev/fsdev_common.c
)

# Include directories
set(INC_DIRS
    Inc
    Src/Apps
    Src/Services
    Src/Drivers/CMSIS/Include
    Src/Drivers/CMSIS/Device/ST/STM32H5xx/Include
    Src/Drivers/STM32H5xx_HAL_Driver/Inc
    Src/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy
    Src/Drivers/tinyusb/src
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
