#include "Receiver.h"
#include <math.h>

#include "ADCService.h"
#include "ComMgr.h"
#include "stm32h5xx.h"
#define FRAME_SAMPLES          2048U
#define FRAME_HEADER_SIZE      16U
#define RECEIVER_FRAME_SIZE    (FRAME_HEADER_SIZE + (FRAME_SAMPLES * 2U))

#define ADC_BIAS               2048
#define ADC_MAX_VAL            4095

#define BPF_GAIN_SHIFT         3U
#define BPF_FEEDBACK_SHIFT     2U

#define FS_HZ                  160000U
#define LOG_INTERVAL_MS        1000U
#define LOG_INTERVAL_FRAMES    ((FS_HZ * LOG_INTERVAL_MS) / (FRAME_SAMPLES * 1000U))
#define FALLBACK_CLOCK_MHZ     250U

#define DSP_HEADER_SIZE        8U
#define DSP_VALUE_COUNT        5U
#define DSP_FRAME_SIZE         (DSP_HEADER_SIZE + (DSP_VALUE_COUNT * 4U))

typedef struct {
    int32_t x1;
    int32_t x2;
    int32_t y1;
    int32_t y2;
} BPF_State_t;

static uint8_t receiver_frame[RECEIVER_FRAME_SIZE];
static int16_t adc1_frame_buffer[FRAME_SAMPLES];
static int16_t adc2_frame_buffer[FRAME_SAMPLES];

// BPF Filter States - Mảng chứa trạng thái bộ lọc BPF 35-45 kHz cho 2 kênh
static BPF_State_t bpf_states[2] = { {0, 0, 0, 0}, {0, 0, 0, 0} };
static int16_t raw_active_buffer[FRAME_SAMPLES];

// IQ Demodulated Buffers - Mảng lưu trữ tín hiệu phức cho 2 kênh (tối ưu hóa bộ nhớ, không dùng buffer độ lớn phụ)
static Complex_t complex_buffers[2][FRAME_SAMPLES];

#ifdef SHOW_SAMPLING_LOG
static uint32_t log_counter = 0U;
#endif

// Hàm tính căn bậc hai số nguyên nhanh (Integer Square Root) không dùng float
static uint32_t int_sqrt(uint32_t val)
{
    uint32_t temp, g = 0;
    for (uint32_t bit = 1U << 11; bit > 0; bit >>= 1) {
        temp = g + bit;
        if (temp * temp <= val) {
            g = temp;
        }
    }
    return g;
}

static void Receiver_FilterBPF(int16_t *buffer, uint32_t length, BPF_State_t *state)
{
    // Load state into local registers to avoid RAM access inside loop
    int32_t sx1 = state->x1;
    int32_t sx2 = state->x2;
    int32_t sy1 = state->y1;
    int32_t sy2 = state->y2;

    for (uint32_t i = 0U; i < length; i++)
    {
        // Loại bỏ DC bias
        int32_t x_q16 = ((int32_t)buffer[i] - ADC_BIAS) << 16;

        // Phương trình sai phân: y[n] = (x[n] - x[n-2]) / 8 - 0.75 * y[n-2]
        int32_t diff_x = x_q16 - sx2;
        int32_t term1 = diff_x >> BPF_GAIN_SHIFT;
        int32_t term2 = sy2 - (sy2 >> BPF_FEEDBACK_SHIFT);
        int32_t y_q16 = term1 - term2;

        // Cập nhật trạng thái lịch sử
        sx2 = sx1;
        sx1 = x_q16;

        sy2 = sy1;
        sy1 = y_q16;

        // Đưa về dạng 12-bit nguyên bản và cộng lại DC bias
        int32_t bp = (y_q16 >> 16) + ADC_BIAS;

        // Giới hạn trong dải ADC 12-bit [0, 4095]
        if (bp > ADC_MAX_VAL)
        {
            bp = ADC_MAX_VAL;
        }
        else if (bp < 0)
        {
            bp = 0;
        }

        buffer[i] = (int16_t)bp;
    }

    // Save state back to RAM
    state->x1 = sx1;
    state->x2 = sx2;
    state->y1 = sy1;
    state->y2 = sy2;
}

// Bộ lọc giải điều chế IQ tối ưu hóa sử dụng thanh ghi dịch (Shift Register) để tránh load lại bộ nhớ và rẽ nhánh
static void Receiver_IQDemodulate(const int16_t *input, Complex_t *output, uint32_t length)
{
    int32_t v0 = 0;
    int32_t v1 = 0;
    int32_t v2 = 0;
    int32_t v3 = 0;

    for (uint32_t i = 0U; i < length; i++)
    {
        // Cập nhật thanh ghi dịch
        v3 = v2;
        v2 = v1;
        v1 = v0;
        v0 = (int32_t)input[i] - ADC_BIAS;

        int32_t A = (v0 - v2) >> 2;
        int32_t B = (v1 - v3) >> 2;

        int16_t I = 0;
        int16_t Q = 0;

        switch (i & 3U)
        {
            case 0U:
                I = (int16_t)A;
                Q = (int16_t)(-B);
                break;
            case 1U:
                I = (int16_t)B;
                Q = (int16_t)(-A);
                break;
            case 2U:
                I = (int16_t)(-A);
                Q = (int16_t)B;
                break;
            case 3U:
                I = (int16_t)(-B);
                Q = (int16_t)A;
                break;
        }

        output[i].re = I;
        output[i].im = Q;
    }
}

Complex_t* Receiver_GetComplexBuffer(uint32_t channel)
{
    if (channel >= 1U && channel <= 2U)
    {
        return complex_buffers[channel - 1U];
    }
    return NULL;
}



void Receiver_Init(void)
{
    ADCService_Init(1U);
    ADCService_Init(2U);

    receiver_frame[0] = 'F';
    receiver_frame[1] = 'R';
    receiver_frame[2] = 'X';
    receiver_frame[3] = '1';
    receiver_frame[4] = (uint8_t)(FRAME_SAMPLES & 0xFFU);
    receiver_frame[5] = (uint8_t)((FRAME_SAMPLES >> 8U) & 0xFFU);
}

void Receiver_Process(void)
{
#ifdef SHOW_SAMPLING_LOG
    uint32_t t_start = DWT->CYCCNT;
    uint32_t t_read_start = DWT->CYCCNT;
#endif

    bool has_frame[2] = {
        ADCService_ReadFrame(1U, adc1_frame_buffer),
        ADCService_ReadFrame(2U, adc2_frame_buffer)
    };

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_read_end = DWT->CYCCNT;
    uint32_t t_bpf_start = DWT->CYCCNT;
#endif

    int16_t *buffers[2] = { adc1_frame_buffer, adc2_frame_buffer };
    uint32_t rx_chan = ComMgr_GetRxSelect();
    bool active_has_frame = (rx_chan >= 1U && rx_chan <= 2U && has_frame[rx_chan - 1U]);

    // Lưu lại tín hiệu thô trước khi lọc BPF cho kênh đang chọn
    if (active_has_frame)
    {
        for (uint32_t i = 0U; i < FRAME_SAMPLES; i++)
        {
            raw_active_buffer[i] = buffers[rx_chan - 1U][i];
        }
    }

    // Luôn tiến hành lọc BPF đầy đủ trên cả 2 kênh để tính toán thời gian DSP chuẩn xác
    for (uint32_t i = 0U; i < 2U; i++)
    {
        if (has_frame[i])
        {
            Receiver_FilterBPF(buffers[i], FRAME_SAMPLES, &bpf_states[i]);
        }
    }

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_bpf_end = DWT->CYCCNT;
    uint32_t t_demod_start = DWT->CYCCNT;
#endif

    // Luôn luôn thực hiện IQ Demodulate trên cả 2 kênh
    for (uint32_t i = 0U; i < 2U; i++)
    {
        if (has_frame[i])
        {
            Receiver_IQDemodulate(buffers[i], complex_buffers[i], FRAME_SAMPLES);
        }
    }

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_demod_end = DWT->CYCCNT;
#endif

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_send_start = 0U;
    uint32_t t_send_end = 0U;
#endif
    bool sent = false;

    if (active_has_frame)
    {
#ifdef SHOW_SAMPLING_LOG
        t_send_start = DWT->CYCCNT;
#endif
        receiver_frame[3] = (uint8_t)('0' + rx_chan);
        StreamMode_t mode = ComMgr_GetStreamMode();
        if (mode == STREAM_MODE_DEMOD)
        {
            const Complex_t *chan_buf = complex_buffers[rx_chan - 1U];
            for (uint32_t i = 0U; i < FRAME_SAMPLES; i++)
            {
                int32_t re = (int32_t)chan_buf[i].re;
                int32_t im = (int32_t)chan_buf[i].im;
                uint32_t norm = (uint32_t)re * (uint32_t)re + (uint32_t)im * (uint32_t)im;
                int16_t val = (int16_t)int_sqrt(norm);
                receiver_frame[FRAME_HEADER_SIZE + i * 2] = (uint8_t)(val & 0xFF);
                receiver_frame[FRAME_HEADER_SIZE + i * 2 + 1] = (uint8_t)((val >> 8) & 0xFF);
            }
        }
        else if (mode == STREAM_MODE_BPF)
        {
            const int16_t *send_buffer = buffers[rx_chan - 1U];
            for (uint32_t i = 0U; i < FRAME_SAMPLES; i++)
            {
                int16_t val = send_buffer[i];
                receiver_frame[FRAME_HEADER_SIZE + i * 2] = (uint8_t)(val & 0xFF);
                receiver_frame[FRAME_HEADER_SIZE + i * 2 + 1] = (uint8_t)((val >> 8) & 0xFF);
            }
        }
        else
        {
            for (uint32_t i = 0U; i < FRAME_SAMPLES; i++)
            {
                int16_t val = raw_active_buffer[i];
                receiver_frame[FRAME_HEADER_SIZE + i * 2] = (uint8_t)(val & 0xFF);
                receiver_frame[FRAME_HEADER_SIZE + i * 2 + 1] = (uint8_t)((val >> 8) & 0xFF);
            }
        }
        ComMgr_SendData(receiver_frame, sizeof(receiver_frame));
#ifdef SHOW_SAMPLING_LOG
        t_send_end = DWT->CYCCNT;
#endif
        sent = true;
    }

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_end = DWT->CYCCNT;

    if (has_frame[0] || has_frame[1])
    {
        log_counter++;
        if (log_counter >= LOG_INTERVAL_FRAMES)
        {
            log_counter = 0U; // Đã khôi phục câu lệnh reset biến đếm log
            uint32_t clock_mhz = SystemCoreClock / 1000000U;
            if (clock_mhz == 0U)
            {
                clock_mhz = FALLBACK_CLOCK_MHZ;
            }

            uint32_t total_cycles = t_end - t_start;
            uint32_t read_cycles = t_read_end - t_read_start;
            uint32_t bpf_cycles = t_bpf_end - t_bpf_start;
            uint32_t demod_cycles = t_demod_end - t_demod_start;
            uint32_t send_cycles = sent ? (t_send_end - t_send_start) : 0U;

            uint32_t total_us = total_cycles / clock_mhz;
            uint32_t read_us = read_cycles / clock_mhz;
            uint32_t bpf_us = bpf_cycles / clock_mhz;
            uint32_t demod_us = demod_cycles / clock_mhz;
            uint32_t send_us = send_cycles / clock_mhz;

            // Sắp xếp bản tin nhị phân gửi đi
            uint8_t dsp_frame[DSP_FRAME_SIZE];
            dsp_frame[0] = 'D';
            dsp_frame[1] = 'S';
            dsp_frame[2] = 'P';
            dsp_frame[3] = '1';

            // Sequence (Little-Endian)
            dsp_frame[4] = (uint8_t)(log_counter & 0xFFU);
            dsp_frame[5] = (uint8_t)((log_counter >> 8U) & 0xFFU);
            dsp_frame[6] = (uint8_t)((log_counter >> 16U) & 0xFFU);
            dsp_frame[7] = (uint8_t)((log_counter >> 24U) & 0xFFU);

            // Ghi các giá trị payload (mỗi giá trị 4 bytes)
            uint32_t values[DSP_VALUE_COUNT] = { total_us, read_us, bpf_us, demod_us, send_us };
            for (uint32_t v = 0U; v < DSP_VALUE_COUNT; v++)
            {
                for (uint32_t b = 0U; b < 4U; b++)
                {
                    dsp_frame[DSP_HEADER_SIZE + v * 4U + b] = (uint8_t)(values[v] >> (b * 8U));
                }
            }

            ComMgr_SendData(dsp_frame, sizeof(dsp_frame));
        }
    }
#endif
}