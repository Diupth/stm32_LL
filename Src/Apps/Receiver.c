#pragma GCC optimize ("O3")
#include "Receiver.h"
#include <math.h>
#include <string.h>

#include "ADCService.h"
#include "ComMgr.h"
#include "Transmitter.h"
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
#define DSP_VALUE_COUNT        7U
#define DSP_FRAME_SIZE         (DSP_HEADER_SIZE + (DSP_VALUE_COUNT * 4U))

typedef struct {
    int32_t x1;
    int32_t x2;
    int32_t y1;
    int32_t y2;
} BPF_State_t;

static __attribute__((aligned(4))) uint8_t receiver_frame[RECEIVER_FRAME_SIZE];
static __attribute__((aligned(4))) int16_t adc1_frame_buffer[FRAME_SAMPLES];
static __attribute__((aligned(4))) int16_t adc2_frame_buffer[FRAME_SAMPLES];

// BPF Filter States - Mảng chứa trạng thái bộ lọc BPF 35-45 kHz cho 2 kênh
static BPF_State_t bpf_states[2] = { {0, 0, 0, 0}, {0, 0, 0, 0} };
static __attribute__((aligned(4))) int16_t raw_active_buffer[FRAME_SAMPLES];

// IQ Demodulated Buffers - Mảng lưu trữ tín hiệu phức cho 2 kênh (tối ưu hóa bộ nhớ, không dùng buffer độ lớn phụ)
static __attribute__((aligned(4))) Complex_t complex_buffers[2][FRAME_SAMPLES];
static __attribute__((aligned(4))) Complex_t compressed_buffers[2][FRAME_SAMPLES];

// 8-cycle slow time complex sum accumulation buffer
static __attribute__((aligned(4))) Complex_t slow_time_accumulation[8][FRAME_SAMPLES];
// Real sum and diff norm accumulation buffers (fixed-point uint32_t)
static uint32_t accumulated_sum_norm[FRAME_SAMPLES];
static uint32_t accumulated_diff_norm[FRAME_SAMPLES];
static uint32_t pulse_idx = 0U;
static bool cached_has_frame[2] = {false, false};

// Fast integer square root
static uint32_t int_sqrt(uint32_t x)
{
    uint32_t res = 0;
    uint32_t bit = 1UL << 30;
    while (bit > x)
    {
        bit >>= 2;
    }
    while (bit != 0)
    {
        if (x >= res + bit)
        {
            x -= res + bit;
            res = (res >> 1) + bit;
        }
        else
        {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

#ifdef SHOW_SAMPLING_LOG
static uint32_t log_counter = 0U;
#endif




static void Receiver_FilterBPF(int16_t *buffer, uint32_t length, BPF_State_t *state)
{
    // Load state into local registers to avoid RAM access inside loop
    int32_t sx1 = state->x1;
    int32_t sx2 = state->x2;
    int32_t sy1 = state->y1;
    int32_t sy2 = state->y2;

    for (uint32_t i = 0U; i < length; i += 2U)
    {
        // Mẫu 0 (i + 0)
        int32_t x_q16_0 = ((int32_t)buffer[i] - ADC_BIAS) << 16;
        int32_t diff_x_0 = x_q16_0 - sx2;
        int32_t term1_0 = diff_x_0 >> BPF_GAIN_SHIFT;
        int32_t term2_0 = sy2 - (sy2 >> BPF_FEEDBACK_SHIFT);
        int32_t y_q16_0 = term1_0 - term2_0;

        int32_t bp_0 = (y_q16_0 >> 16) + ADC_BIAS;
        if (bp_0 > ADC_MAX_VAL)
        {
            bp_0 = ADC_MAX_VAL;
        }
        else if (bp_0 < 0)
        {
            bp_0 = 0;
        }
        buffer[i] = (int16_t)bp_0;

        // Mẫu 1 (i + 1)
        int32_t x_q16_1 = ((int32_t)buffer[i + 1] - ADC_BIAS) << 16;
        int32_t diff_x_1 = x_q16_1 - sx1;
        int32_t term1_1 = diff_x_1 >> BPF_GAIN_SHIFT;
        int32_t term2_1 = sy1 - (sy1 >> BPF_FEEDBACK_SHIFT);
        int32_t y_q16_1 = term1_1 - term2_1;

        int32_t bp_1 = (y_q16_1 >> 16) + ADC_BIAS;
        if (bp_1 > ADC_MAX_VAL)
        {
            bp_1 = ADC_MAX_VAL;
        }
        else if (bp_1 < 0)
        {
            bp_1 = 0;
        }
        buffer[i + 1] = (int16_t)bp_1;

        // Cập nhật trạng thái lịch sử
        sx2 = x_q16_0;
        sx1 = x_q16_1;
        sy2 = y_q16_0;
        sy1 = y_q16_1;
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

    for (uint32_t i = 0U; i < length; i += 4U)
    {
        // Mẫu i
        v3 = v2;
        v2 = v1;
        v1 = v0;
        v0 = (int32_t)input[i] - ADC_BIAS;
        output[i].re = (int16_t)((v0 - v2) >> 2);
        output[i].im = (int16_t)(-(v1 - v3) >> 2);

        // Mẫu i + 1
        v3 = v2;
        v2 = v1;
        v1 = v0;
        v0 = (int32_t)input[i + 1] - ADC_BIAS;
        output[i + 1].re = (int16_t)((v1 - v3) >> 2);
        output[i + 1].im = (int16_t)(-(v0 - v2) >> 2);

        // Mẫu i + 2
        v3 = v2;
        v2 = v1;
        v1 = v0;
        v0 = (int32_t)input[i + 2] - ADC_BIAS;
        output[i + 2].re = (int16_t)(-(v0 - v2) >> 2);
        output[i + 2].im = (int16_t)((v1 - v3) >> 2);

        // Mẫu i + 3
        v3 = v2;
        v2 = v1;
        v1 = v0;
        v0 = (int32_t)input[i + 3] - ADC_BIAS;
        output[i + 3].re = (int16_t)(-(v1 - v3) >> 2);
        output[i + 3].im = (int16_t)((v0 - v2) >> 2);
    }
}

static void Receiver_MatchedFilter(const Complex_t * restrict input, Complex_t * restrict output, uint32_t length, Transmitter_PulseType pulse_type)
{
    static int32_t S_packed[FRAME_SAMPLES];
    int32_t sum_packed = 0;

    const int32_t * restrict in_ptr = (const int32_t *)input;
    int32_t * restrict s_ptr = S_packed;

    // Tính tổng dịch chuyển (moving sum) của 8 mẫu cho cả phần thực và ảo song song dùng SIMD
    for (uint32_t i = 0U; i < 8U; i++)
    {
        sum_packed = __SADD16(sum_packed, in_ptr[i]);
        s_ptr[i] = sum_packed;
    }
    for (uint32_t i = 8U; i < length; i++)
    {
        sum_packed = __SADD16(sum_packed, in_ptr[i]);
        sum_packed = __SSUB16(sum_packed, in_ptr[i - 8U]);
        s_ptr[i] = sum_packed;
    }

    if (pulse_type == TRANSMITTER_PULSE_BARKER13)
    {
        // 13 chíp Barker, mỗi chíp dài 8 mẫu. 
        // Lọc tương thích hoàn toàn dùng phép cộng/trừ với mã Barker-13 đảo thời gian:
        // +1, -1, +1, -1, +1, +1, -1, -1, +1, +1, +1, +1, +1
        for (uint32_t i = 0U; i < 103U; i++)
        {
            output[i].re = 0;
            output[i].im = 0;
        }
        for (uint32_t i = 103U; i < length; i++)
        {
            int32_t acc = s_ptr[i];
            acc = __SSUB16(acc, s_ptr[i - 8U]);
            acc = __SADD16(acc, s_ptr[i - 16U]);
            acc = __SSUB16(acc, s_ptr[i - 24U]);
            acc = __SADD16(acc, s_ptr[i - 32U]);
            acc = __SADD16(acc, s_ptr[i - 40U]);
            acc = __SSUB16(acc, s_ptr[i - 48U]);
            acc = __SSUB16(acc, s_ptr[i - 56U]);
            acc = __SADD16(acc, s_ptr[i - 64U]);
            acc = __SADD16(acc, s_ptr[i - 72U]);
            acc = __SADD16(acc, s_ptr[i - 80U]);
            acc = __SADD16(acc, s_ptr[i - 88U]);
            acc = __SADD16(acc, s_ptr[i - 96U]);

            // Shift right by 3 (scale down / 8) cho cả Re và Im đóng gói trong 32-bit int
            output[i].re = (int16_t)((int16_t)acc >> 3);
            output[i].im = (int16_t)((int16_t)(acc >> 16) >> 3);
        }
    }
    else
    {
        // Xung đơn Single (dài 8 mẫu), matched filter đơn giản chỉ là tổng dịch chuyển 8 mẫu chia cho 8
        for (uint32_t i = 0U; i < 7U; i++)
        {
            output[i].re = 0;
            output[i].im = 0;
        }
        for (uint32_t i = 7U; i < length; i++)
        {
            int32_t acc = s_ptr[i];
            output[i].re = (int16_t)((int16_t)acc >> 3);
            output[i].im = (int16_t)((int16_t)(acc >> 16) >> 3);
        }
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

static void Receiver_SendAccumulatedFrame(uint32_t rx_chan)
{
    // Write channel ID to the 4th byte of the header ('0' for Rx Sum, '3' for Rx Diff)
    receiver_frame[3] = (uint8_t)('0' + rx_chan);
    int16_t *dest_payload = (int16_t *)&receiver_frame[FRAME_HEADER_SIZE];
    
    if (rx_chan == 0U)
    {
        for (uint32_t i = 0U; i < FRAME_SAMPLES; i++)
        {
            dest_payload[i] = (int16_t)int_sqrt(accumulated_sum_norm[i] >> 3); // sqrt of average energy
        }
    }
    else if (rx_chan == 3U)
    {
        for (uint32_t i = 0U; i < FRAME_SAMPLES; i++)
        {
            dest_payload[i] = (int16_t)int_sqrt(accumulated_diff_norm[i] >> 3); // sqrt of average energy
        }
    }
    
    // Gửi gói tin qua ComMgr
    ComMgr_SendData(receiver_frame, sizeof(receiver_frame));
}

// Gửi frame dữ liệu của kênh đang chọn dựa trên chế độ truyền tải (Stream Mode)
static void Receiver_SendActiveFrame(uint32_t rx_chan, int16_t *buffers[2])
{
    // Ghi số hiệu kênh nhận (1 hoặc 2) vào byte thứ 4 của header
    receiver_frame[3] = (uint8_t)('0' + rx_chan);
    StreamMode_t mode = ComMgr_GetStreamMode();

    if (mode == STREAM_MODE_COMPRESSED)
    {
        // Chế độ Compressed: Tính biên độ tín hiệu phức từ bộ lọc tương thích (Magnitude = sqrt(I^2 + Q^2))
        const Complex_t *chan_buf = compressed_buffers[rx_chan - 1U];
        int16_t *dest_payload = (int16_t *)&receiver_frame[FRAME_HEADER_SIZE];
        for (uint32_t i = 0U; i < FRAME_SAMPLES; i++)
        {
            // Sử dụng SIMD lệnh __SMLAD để nhân song song 16-bit và cộng dồn trong 1 chu kỳ máy
            int32_t packed_val = *(const int32_t *)&chan_buf[i];
            int32_t norm = __SMLAD(packed_val, packed_val, 0);
            dest_payload[i] = (int16_t)sqrtf((float)norm);
        }
    }
    else if (mode == STREAM_MODE_DEMOD)
    {
        // Chế độ IQ Demodulation: Tính biên độ tín hiệu phức (Magnitude = sqrt(I^2 + Q^2))
        const Complex_t *chan_buf = complex_buffers[rx_chan - 1U];
        int16_t *dest_payload = (int16_t *)&receiver_frame[FRAME_HEADER_SIZE];
        for (uint32_t i = 0U; i < FRAME_SAMPLES; i++)
        {
            // Sử dụng SIMD lệnh __SMLAD để nhân song song 16-bit và cộng dồn trong 1 chu kỳ máy
            int32_t packed_val = *(const int32_t *)&chan_buf[i];
            int32_t norm = __SMLAD(packed_val, packed_val, 0);
            dest_payload[i] = (int16_t)sqrtf((float)norm);
        }
    }
    else if (mode == STREAM_MODE_BPF)
    {
        // Chế độ Bandpass Filter: Gửi trực tiếp tín hiệu đã qua bộ lọc BPF
        const int16_t *send_buffer = buffers[rx_chan - 1U];
        memcpy(&receiver_frame[FRAME_HEADER_SIZE], send_buffer, FRAME_SAMPLES * sizeof(int16_t));
    }
    else
    {
        // Chế độ Raw: Gửi tín hiệu thô được lưu trữ trước khi lọc BPF
        memcpy(&receiver_frame[FRAME_HEADER_SIZE], raw_active_buffer, FRAME_SAMPLES * sizeof(int16_t));
    }
    
    // Gửi gói tin hoàn chỉnh qua UART/USB qua ComMgr
    ComMgr_SendData(receiver_frame, sizeof(receiver_frame));
}

static void Receiver_AccumulateAndProcess(uint32_t rx_chan, bool *sent, uint32_t *t_send_start, uint32_t *t_send_end, uint32_t *t_accum_cycles)
{
    if (pulse_idx == 0U)
    {
        memset(accumulated_sum_norm, 0, sizeof(accumulated_sum_norm));
        memset(accumulated_diff_norm, 0, sizeof(accumulated_diff_norm));
    }

#ifdef SHOW_SAMPLING_LOG
    uint32_t loop_start = DWT->CYCCNT;
#endif

    for (uint32_t i = 0U; i < FRAME_SAMPLES; i++)
    {
        // Safe type-punned union to avoid strict-aliasing rules
        union { Complex_t c; int32_t val; } u_rx1, u_rx2, u_sum;
        u_rx1.c = compressed_buffers[0][i];
        u_rx2.c = compressed_buffers[1][i];
        int32_t packed_rx1 = u_rx1.val;
        int32_t packed_rx2 = u_rx2.val;

        // SIMD sum and diff of 16-bit packed components
        int32_t sum_packed = __SADD16(packed_rx1, packed_rx2);
        int32_t diff_packed = __SSUB16(packed_rx1, packed_rx2);

        // Store complex sum to slow time accumulation buffer
        u_sum.val = sum_packed;
        slow_time_accumulation[pulse_idx][i] = u_sum.c;

        // Compute squared magnitudes using SIMD __SMLAD (re^2 + im^2)
        int32_t sum_norm = __SMLAD(sum_packed, sum_packed, 0);
        int32_t diff_norm = __SMLAD(diff_packed, diff_packed, 0);

        // Accumulate squared magnitudes directly (fast additions)
        accumulated_sum_norm[i] += (uint32_t)sum_norm;
        accumulated_diff_norm[i] += (uint32_t)diff_norm;
    }

#ifdef SHOW_SAMPLING_LOG
    uint32_t loop_end = DWT->CYCCNT;
    if (t_accum_cycles)
    {
        *t_accum_cycles = loop_end - loop_start;
    }
#endif

    // If Rx Sum (RX_CHANNEL_SUM) or Rx Diff (RX_CHANNEL_DIFF) is selected, send frame ONLY on the 8th pulse
    if ((rx_chan == RX_CHANNEL_SUM || rx_chan == RX_CHANNEL_DIFF) && (pulse_idx == 7U))
    {
#ifdef SHOW_SAMPLING_LOG
        if (t_send_start)
        {
            *t_send_start = DWT->CYCCNT;
        }
#endif
        Receiver_SendAccumulatedFrame(rx_chan);
#ifdef SHOW_SAMPLING_LOG
        if (t_send_end)
        {
            *t_send_end = DWT->CYCCNT;
        }
        if (sent)
        {
            *sent = true;
        }
#endif
    }

    pulse_idx = (pulse_idx + 1) % 8U;
}

#ifdef SHOW_SAMPLING_LOG
static void Receiver_SendDSPLog(uint32_t t_start, uint32_t t_read_start, uint32_t t_read_end, 
                                 uint32_t t_bpf_start, uint32_t t_bpf_end, uint32_t t_demod_start, uint32_t t_demod_end,
                                 uint32_t t_mfilt_start, uint32_t t_mfilt_end,
                                 bool sent, uint32_t t_send_start, uint32_t t_send_end,
                                 uint32_t t_accum_cycles)
{
    uint32_t t_end = DWT->CYCCNT;

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
            uint32_t mfilt_cycles = t_mfilt_end - t_mfilt_start;
            uint32_t send_cycles = sent ? (t_send_end - t_send_start) : 0U;
            uint32_t accum_cycles = t_accum_cycles;

            uint32_t total_us = total_cycles / clock_mhz;
            uint32_t read_us = read_cycles / clock_mhz;
            uint32_t bpf_us = bpf_cycles / clock_mhz;
            uint32_t demod_us = demod_cycles / clock_mhz;
            uint32_t mfilt_us = mfilt_cycles / clock_mhz;
            uint32_t send_us = send_cycles / clock_mhz;
            uint32_t accum_us = accum_cycles / clock_mhz;

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
            uint32_t values[DSP_VALUE_COUNT] = { total_us, read_us, bpf_us, demod_us, mfilt_us, send_us, accum_us };
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

    if (!cached_has_frame[0])
    {
        cached_has_frame[0] = ADCService_ReadFrame(1U, adc1_frame_buffer);
    }
    if (!cached_has_frame[1])
    {
        cached_has_frame[1] = ADCService_ReadFrame(2U, adc2_frame_buffer);
    }

    if (!cached_has_frame[0] || !cached_has_frame[1])
    {
        return;
    }

    // Tiêu thụ các frame đã cache
    cached_has_frame[0] = false;
    cached_has_frame[1] = false;

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_read_end = DWT->CYCCNT;
    uint32_t t_bpf_start = DWT->CYCCNT;
#endif

    int16_t *buffers[2] = { adc1_frame_buffer, adc2_frame_buffer };
    uint32_t rx_chan = ComMgr_GetRxSelect();
    bool active_has_frame = (rx_chan >= 1U && rx_chan <= 2U);

    // Lưu lại tín hiệu thô trước khi lọc BPF cho kênh đang chọn
    if (active_has_frame)
    {
        memcpy(raw_active_buffer, buffers[rx_chan - 1U], FRAME_SAMPLES * sizeof(int16_t));
    }

    // Luôn tiến hành lọc BPF đầy đủ trên cả 2 kênh để tính toán thời gian DSP chuẩn xác
    for (uint32_t i = 0U; i < 2U; i++)
    {
        Receiver_FilterBPF(buffers[i], FRAME_SAMPLES, &bpf_states[i]);
    }

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_bpf_end = DWT->CYCCNT;
    uint32_t t_demod_start = DWT->CYCCNT;
#endif

    // Luôn luôn thực hiện IQ Demodulate trên cả 2 kênh
    for (uint32_t i = 0U; i < 2U; i++)
    {
        Receiver_IQDemodulate(buffers[i], complex_buffers[i], FRAME_SAMPLES);
    }

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_demod_end = DWT->CYCCNT;
    uint32_t t_mfilt_start = DWT->CYCCNT;
#endif

    // Luôn luôn thực hiện Matched Filter trên cả 2 kênh
    Transmitter_PulseType pulse_type = Transmitter_GetPulseType();
    for (uint32_t i = 0U; i < 2U; i++)
    {
        Receiver_MatchedFilter(complex_buffers[i], compressed_buffers[i], FRAME_SAMPLES, pulse_type);
    }

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_mfilt_end = DWT->CYCCNT;
#endif

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_send_start = 0U;
    uint32_t t_send_end = 0U;
    uint32_t t_accum_cycles = 0U;
#endif
    bool sent = false;

#ifdef SHOW_SAMPLING_LOG
    Receiver_AccumulateAndProcess(rx_chan, &sent, &t_send_start, &t_send_end, &t_accum_cycles);
#else
    Receiver_AccumulateAndProcess(rx_chan, NULL, NULL, NULL, NULL);
#endif

    if (active_has_frame)
    {
#ifdef SHOW_SAMPLING_LOG
        t_send_start = DWT->CYCCNT;
#endif
        Receiver_SendActiveFrame(rx_chan, buffers);
#ifdef SHOW_SAMPLING_LOG
        t_send_end = DWT->CYCCNT;
        sent = true;
#endif
    }

#ifdef SHOW_SAMPLING_LOG
    Receiver_SendDSPLog(t_start, t_read_start, t_read_end, t_bpf_start, t_bpf_end, 
                        t_demod_start, t_demod_end, t_mfilt_start, t_mfilt_end, sent, t_send_start, t_send_end,
                        t_accum_cycles);
#endif
}