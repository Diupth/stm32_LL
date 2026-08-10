#!/bin/zsh

# Dừng thực thi nếu có lỗi xảy ra
set -e

# Dùng STM32CubeCLT nếu được cài ở vị trí mặc định trên macOS.
if [[ -d "/opt/ST/STM32CubeCLT_1.22.0/GNU-tools-for-STM32/bin" ]]; then
	export PATH="/opt/ST/STM32CubeCLT_1.22.0/GNU-tools-for-STM32/bin:/opt/ST/STM32CubeCLT_1.22.0/CMake/bin:/opt/ST/STM32CubeCLT_1.22.0/Ninja/bin:$PATH"
fi

command -v cmake >/dev/null || { echo "Error: cmake is not in PATH" >&2; exit 1; }
command -v ninja >/dev/null || { echo "Error: ninja is not in PATH" >&2; exit 1; }
command -v arm-none-eabi-gcc >/dev/null || { echo "Error: arm-none-eabi-gcc is not in PATH" >&2; exit 1; }

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
