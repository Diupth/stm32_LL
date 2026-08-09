#!/bin/zsh

# Dừng thực thi nếu có lỗi xảy ra
set -e

# Thiết lập đường dẫn Toolchain từ STM32CubeCLT
export PATH="/opt/ST/STM32CubeCLT_1.22.0/GNU-tools-for-STM32/bin:/opt/ST/STM32CubeCLT_1.22.0/CMake/bin:/opt/ST/STM32CubeCLT_1.22.0/Ninja/bin:$PATH"

# Thư mục chứa script
SCRIPT_DIR="${0:A:h}"
cd "$SCRIPT_DIR"

# Tạo thư mục build nếu chưa có và cấu hình bằng CMake sử dụng Ninja
echo "=== Configuring project with CMake ==="
cmake -DCMAKE_TOOLCHAIN_FILE=arm-none-eabi-gcc.cmake -B build -G Ninja

# Biên dịch dự án
echo "=== Building project ==="
cmake --build build

echo "=== Build completed successfully ==="
echo "Output files:"
echo "  - build/Low_layer.elf"
echo "  - build/Low_layer.bin"
echo "  - build/Low_layer.hex"
