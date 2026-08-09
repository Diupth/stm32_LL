#!/bin/zsh

# Thư mục chứa script
SCRIPT_DIR="${0:A:h}"
cd "$SCRIPT_DIR"

# Đường dẫn tới STM32_Programmer_CLI từ STM32CubeCLT
PROGRAMMER_CLI="/opt/ST/STM32CubeCLT_1.22.0/STM32CubeProgrammer/bin/STM32_Programmer_CLI"

# Kiểm tra file binary có tồn tại hay không
BIN_FILE="build/Low_layer.bin"
if [ ! -f "$BIN_FILE" ]; then
    echo "Lỗi: Không tìm thấy file binary tại $BIN_FILE."
    echo "Vui lòng chạy './build.sh' trước để biên dịch dự án."
    exit 1
fi

echo "=== Đang chuẩn bị nạp chương trình qua cổng USB DFU ==="
echo "Hướng dẫn kết nối:"
echo "1. Nhấn giữ nút BOOT (hoặc nút BT0) trên board WeAct."
echo "2. Nhấn nút RST (Reset) và thả ra."
echo "3. Thả nút BOOT. Board sẽ chuyển sang chế độ DFU bootloader."
echo "4. Cắm cáp USB-C kết nối trực tiếp từ board vào máy tính."
echo ""
echo "Đang cố gắng nạp file: $BIN_FILE vào địa chỉ Flash 0x08000000..."

# Chạy lệnh nạp chương trình qua giao tiếp USB DFU (không dùng -rst vì giao thức DFU không hỗ trợ reset phần cứng)
"$PROGRAMMER_CLI" -c port=USB1 -d "$BIN_FILE" 0x08000000 -v
