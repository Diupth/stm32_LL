#include "LogService.h"
#include "ComMgr.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define LOG_MAX_BUFFER_SIZE 256U

// Trích xuất tên file gọn (bỏ đường dẫn thư mục)
static const char *LogService_GetFileName(const char *path)
{
    if (path == NULL)
    {
        return "unknown";
    }
    const char *filename = path;
    const char *p = path;
    while (*p != '\0')
    {
        if (*p == '/' || *p == '\\')
        {
            filename = p + 1;
        }
        p++;
    }
    return filename;
}

void LogService_Init(void)
{
    // LogService sử dụng ComMgr để truyền nhận, đảm bảo ComMgr được khởi tạo trước
}

// Hàm phụ trợ ghi số nguyên unsigned 32-bit thành chuỗi ASCII cực nhanh (không dùng chia chậm của printf)
static inline char *LogService_U32ToAscii(char *buf, uint32_t val)
{
    char temp[11];
    int idx = 0;
    if (val == 0U)
    {
        *buf++ = '0';
        return buf;
    }
    while (val > 0U)
    {
        uint32_t q = (val * 0xCCCCCCCDULL) >> 35; // Tối ưu phép chia cho 10 bằng phép nhân hằng số + dịch bit (Reciprocal division)
        uint32_t r = val - q * 10U;
        temp[idx++] = (char)('0' + r);
        val = q;
    }
    while (idx > 0)
    {
        *buf++ = temp[--idx];
    }
    return buf;
}

uint64_t LogService_GetTimestampUs(void)
{
    uint32_t ms1, ms2, val;
    uint32_t load = SysTick->LOAD;

    if (load == 0U)
    {
        return ((uint64_t)HAL_GetTick() * 1000ULL);
    }

    // Đọc an toàn tránh xung đột ngắt SysTick (đọc ms trước và sau khi đọc counter)
    do {
        ms1 = HAL_GetTick();
        val = SysTick->VAL;
        ms2 = HAL_GetTick();
    } while (ms1 != ms2);

    uint32_t elapsed = (load >= val) ? (load - val) : 0U;

    // Tối ưu hóa: Thay vì chia (elapsed * 1000) / (load + 1)
    // Với SystemCoreClock = 240 MHz (load = 239999), phép chia cho 240 được thực hiện bằng
    // phép nhân nghịch đảo: (elapsed * 2796203) >> 26 hoặc (ms << 10) - (ms << 4) - (ms << 3)
    // Để tổng quát hóa với load bất kỳ, ta tính:
    // ms * 1000 = (ms << 10) - (ms << 4) - (ms << 3) = ms * 1024 - ms * 16 - ms * 8
    uint64_t ms_to_us = ((uint64_t)ms1 << 10) - ((uint64_t)ms1 << 4) - ((uint64_t)ms1 << 3);

    // sub_us: Tại 240MHz (load = 240000 - 1), 1 us = 240 chu kỳ
    // Phép chia elapsed / 240 tương đương (elapsed * 2796203ULL) >> 26
    uint32_t sub_us;
    if (load >= 239000U && load <= 241000U)
    {
        sub_us = (uint32_t)(((uint64_t)elapsed * 2796203ULL) >> 26);
    }
    else
    {
        sub_us = (uint32_t)(((uint64_t)elapsed * 1000ULL) / (uint64_t)(load + 1U));
    }

    return ms_to_us + (uint64_t)sub_us;
}

void LogService_Log(LogLevel_t level, const char *file, uint32_t line, const char *fmt, ...)
{
    char log_buffer[LOG_MAX_BUFFER_SIZE];
    uint64_t stamp_us = LogService_GetTimestampUs();
    const char *tag = (level == LOG_LEVEL_DEV) ? "LOGD" : "LOGP";
    const char *short_file = LogService_GetFileName(file);

    // Ghép prefix [stamp][tag][file:line] nhanh chóng bằng thao tác con trỏ chuỗi
    char *p = log_buffer;
    *p++ = '[';
    
    // Tách 64-bit timestamp thành sec và us_rem
    // stamp_us = sec * 1000000 + us_rem
    uint32_t sec = (uint32_t)(stamp_us / 1000000ULL);
    uint32_t us_rem = (uint32_t)(stamp_us % 1000000ULL);

    if (sec > 0U)
    {
        p = LogService_U32ToAscii(p, sec);
        // Padding 6 số 0 cho us_rem
        uint32_t temp = us_rem;
        int digits = (temp == 0U) ? 1 : 0;
        while (temp > 0U) { digits++; temp /= 10U; }
        for (int i = 0; i < (6 - digits); i++)
        {
            *p++ = '0';
        }
        if (us_rem > 0U)
        {
            p = LogService_U32ToAscii(p, us_rem);
        }
        else
        {
            *p++ = '0';
        }
    }
    else
    {
        p = LogService_U32ToAscii(p, us_rem);
    }

    *p++ = ']';
    *p++ = '[';
    while (*tag != '\0') { *p++ = *tag++; }
    *p++ = ']';
    *p++ = '[';
    while (*short_file != '\0') { *p++ = *short_file++; }
    *p++ = ':';
    p = LogService_U32ToAscii(p, line);
    *p++ = ']';
    *p++ = ' ';
    *p = '\0';

    uint32_t header_len = (uint32_t)(p - log_buffer);

    if (header_len < 0)
    {
        return;
    }

    if ((uint32_t)header_len < sizeof(log_buffer))
    {
        va_list args;
        va_start(args, fmt);
        int msg_len = vsnprintf(log_buffer + header_len, sizeof(log_buffer) - (uint32_t)header_len, fmt, args);
        va_end(args);

        if (msg_len > 0)
        {
            uint32_t total_len = (uint32_t)header_len + (uint32_t)msg_len;
            if (total_len >= sizeof(log_buffer) - 2U)
            {
                total_len = sizeof(log_buffer) - 3U;
            }
            // Đảm bảo kết thúc bằng \r\n
            log_buffer[total_len] = '\r';
            log_buffer[total_len + 1] = '\n';
            log_buffer[total_len + 2] = '\0';
        }
        else
        {
            // Nếu không có message hoặc lỗi snprintf, gắn \r\n vào cuối header
            uint32_t total_len = (uint32_t)header_len;
            if (total_len >= sizeof(log_buffer) - 2U)
            {
                total_len = sizeof(log_buffer) - 3U;
            }
            log_buffer[total_len] = '\r';
            log_buffer[total_len + 1] = '\n';
            log_buffer[total_len + 2] = '\0';
        }
    }

    // Gửi log qua ComMgr
    ComMgr_SendString(log_buffer);
}
