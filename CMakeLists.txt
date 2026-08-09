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

# Definitions (Chỉ dùng CMSIS, không dùng HAL)
add_compile_definitions(
    STM32H562xx
)

# Chỉ compile các file cần thiết trong Core/Src
file(GLOB SOURCES
    "Core/Src/*.c"
    "Core/Src/*.s"
)

# Include directories cho CMSIS
set(INC_DIRS
    "Drivers/CMSIS/Include"
    "Drivers/CMSIS/Device/ST/STM32H5xx/Include"
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

