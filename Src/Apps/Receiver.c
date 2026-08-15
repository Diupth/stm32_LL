#pragma GCC optimize ("O3")
#include "Receiver.h"
#include <math.h>
#include <string.h>

#include "ADCService.h"
#include "ComMgr.h"
#include "Transmitter.h"
#include "stm32h5xx.h"

// System Configuration Constants
#define CHANNEL_COUNT                 2U
#define ACCUMULATION_PULSES           8U

#define FRAME_SAMPLES                 2048U
#define FRAME_HEADER_SIZE             16U
#define RECEIVER_FRAME_SIZE           (FRAME_HEADER_SIZE + (FRAME_SAMPLES * 2U))

// ADC Resolution Constants
#define ADC_RESOLUTION_BITS           12U
#define ADC_MAX_VAL                   ((1U << ADC_RESOLUTION_BITS) - 1U)
#define ADC_BIAS                      (1U << (ADC_RESOLUTION_BITS - 1U))

// DSP Filter Constants
#define BPF_GAIN_SHIFT                3U
#define BPF_FEEDBACK_SHIFT            2U
#define BPF_SAMPLES_PER_ITER          2U

#define IQ_DEMOD_SAMPLES_PER_ITER     4U
#define IQ_DEMOD_SHIFT                2U

#define MOVING_SUM_WINDOW_SIZE        8U

#define BARKER13_CHIP_COUNT           13U
#define BARKER13_CHIP_SAMPLES         8U
#define BARKER13_TOTAL_PREFIX_SAMPLES ((BARKER13_CHIP_COUNT * BARKER13_CHIP_SAMPLES) - 1U)
#define MATCHED_FILTER_SCALE_SHIFT    3U

#define SINGLE_PULSE_PREFIX_SAMPLES   (BARKER13_CHIP_SAMPLES - 1U)

#define ACCUM_NORM_SCALE_SHIFT        14U
#define STREAM_NORM_SCALE_SHIFT       12U

#define BARKER_OFF(n)                 (-(int32_t)((n) * BARKER13_CHIP_SAMPLES))

// Telemetry & Timing Constants
#define US_PER_SEC                    1000000U
#define MS_PER_SEC                    1000U
#define FS_HZ                         160000U
#define LOG_INTERVAL_MS               1000U
#define LOG_INTERVAL_FRAMES           ((FS_HZ * LOG_INTERVAL_MS) / (FRAME_SAMPLES * MS_PER_SEC))
#define FALLBACK_CLOCK_MHZ            250U

#define DSP_HEADER_SIZE               8U
#define DSP_VALUE_COUNT               8U
#define DSP_FRAME_SIZE                (DSP_HEADER_SIZE + (DSP_VALUE_COUNT * 4U))

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
static BPF_State_t bpf_states[CHANNEL_COUNT] = { {0, 0, 0, 0}, {0, 0, 0, 0} };
static __attribute__((aligned(4))) int16_t raw_active_buffer[FRAME_SAMPLES];

// IQ Demodulated Buffers - Mảng lưu trữ tín hiệu phức cho 2 kênh
static __attribute__((aligned(4))) Complex_t complex_buffers[CHANNEL_COUNT][FRAME_SAMPLES];
static __attribute__((aligned(4))) Complex_t compressed_buffers[CHANNEL_COUNT][FRAME_SAMPLES];

// 8-cycle slow time complex sum accumulation buffer
static __attribute__((aligned(4))) Complex_t slow_time_accumulation[ACCUMULATION_PULSES][FRAME_SAMPLES];
// Real sum and diff norm accumulation buffers (fixed-point uint32_t)
static uint32_t accumulated_sum_norm[FRAME_SAMPLES];
static uint32_t accumulated_diff_norm[FRAME_SAMPLES];
static uint32_t pulse_idx = 0U;
static bool cached_has_frame[CHANNEL_COUNT] = {false, false};

#ifdef SHOW_SAMPLING_LOG
static uint32_t t_send_start = 0U;
static uint32_t t_send_end = 0U;
static uint32_t t_accum_cycles = 0U;
static uint32_t t_detect_cycles = 0U;
static bool sent = false;
#endif

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

static void Receiver_FilterBPF(int16_t *buffer, BPF_State_t *state)
{
    // Load state into local registers to avoid RAM access inside loop
    int32_t sx1 = state->x1;
    int32_t sx2 = state->x2;
    int32_t sy1 = state->y1;
    int32_t sy2 = state->y2;

    for (uint32_t i = 0U; i < FRAME_SAMPLES; i += BPF_SAMPLES_PER_ITER)
    {
        // Load two 16-bit samples in a single 32-bit read instruction
        int32_t samples = *(const int32_t *)&buffer[i];
        int32_t sample0 = (int32_t)((int16_t)samples);
        int32_t sample1 = (int32_t)((int16_t)(samples >> 16));

        // Mẫu 0 (i + 0)
        int32_t x_q16_0 = (sample0 - ADC_BIAS) << 16;
        int32_t diff_x_0 = x_q16_0 - sx2;
        int32_t term1_0 = diff_x_0 >> BPF_GAIN_SHIFT;
        int32_t term2_0 = sy2 - (sy2 >> BPF_FEEDBACK_SHIFT);
        int32_t y_q16_0 = term1_0 - term2_0;

        int32_t bp_0 = (y_q16_0 >> 16) + ADC_BIAS;
        buffer[i] = (int16_t)__USAT(bp_0, ADC_RESOLUTION_BITS);

        // Mẫu 1 (i + 1)
        int32_t x_q16_1 = (sample1 - ADC_BIAS) << 16;
        int32_t diff_x_1 = x_q16_1 - sx1;
        int32_t term1_1 = diff_x_1 >> BPF_GAIN_SHIFT;
        int32_t term2_1 = sy1 - (sy1 >> BPF_FEEDBACK_SHIFT);
        int32_t y_q16_1 = term1_1 - term2_1;

        int32_t bp_1 = (y_q16_1 >> 16) + ADC_BIAS;
        buffer[i + 1] = (int16_t)__USAT(bp_1, ADC_RESOLUTION_BITS);

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
static void Receiver_IQDemodulate(const int16_t *input, Complex_t *output)
{
    int32_t v0 = 0;
    int32_t v1 = 0;
    int32_t v2 = 0;
    int32_t v3 = 0;

    for (uint32_t i = 0U; i < FRAME_SAMPLES; i += IQ_DEMOD_SAMPLES_PER_ITER)
    {
        // Mẫu i
        v3 = v2;
        v2 = v1;
        v1 = v0;
        v0 = (int32_t)input[i] - ADC_BIAS;
        output[i].re = (int16_t)((v0 - v2) >> IQ_DEMOD_SHIFT);
        output[i].im = (int16_t)(-(v1 - v3) >> IQ_DEMOD_SHIFT);

        // Mẫu i + 1
        v3 = v2;
        v2 = v1;
        v1 = v0;
        v0 = (int32_t)input[i + 1] - ADC_BIAS;
        output[i + 1].re = (int16_t)((v1 - v3) >> IQ_DEMOD_SHIFT);
        output[i + 1].im = (int16_t)(-(v0 - v2) >> IQ_DEMOD_SHIFT);

        // Mẫu i + 2
        v3 = v2;
        v2 = v1;
        v1 = v0;
        v0 = (int32_t)input[i + 2] - ADC_BIAS;
        output[i + 2].re = (int16_t)(-(v0 - v2) >> IQ_DEMOD_SHIFT);
        output[i + 2].im = (int16_t)((v1 - v3) >> IQ_DEMOD_SHIFT);

        // Mẫu i + 3
        v3 = v2;
        v2 = v1;
        v1 = v0;
        v0 = (int32_t)input[i + 3] - ADC_BIAS;
        output[i + 3].re = (int16_t)(-(v1 - v3) >> IQ_DEMOD_SHIFT);
        output[i + 3].im = (int16_t)((v0 - v2) >> IQ_DEMOD_SHIFT);
    }
}

static void Receiver_MatchedFilter(const Complex_t * restrict input, Complex_t * restrict output)
{
    static int32_t S_packed[FRAME_SAMPLES];
    int32_t sum_packed = 0;

    const int32_t * restrict in_ptr = (const int32_t *)input;
    int32_t * restrict s_ptr = S_packed;

    // Tính tổng dịch chuyển (moving sum) của 8 mẫu cho cả phần thực và ảo song song dùng SIMD và con trỏ
    for (uint32_t i = 0U; i < MOVING_SUM_WINDOW_SIZE; i++)
    {
        sum_packed = __SADD16(sum_packed, in_ptr[i]);
        s_ptr[i] = sum_packed;
    }
    
    const int32_t *p_in = in_ptr + MOVING_SUM_WINDOW_SIZE;
    const int32_t *p_in_prev = in_ptr;
    int32_t *p_out = s_ptr + MOVING_SUM_WINDOW_SIZE;
    for (uint32_t i = MOVING_SUM_WINDOW_SIZE; i < FRAME_SAMPLES; i++)
    {
        sum_packed = __SADD16(sum_packed, *p_in++);
        sum_packed = __SSUB16(sum_packed, *p_in_prev++);
        *p_out++ = sum_packed;
    }

    Transmitter_PulseType pulse_type = Transmitter_GetPulseType();

    if (pulse_type == TRANSMITTER_PULSE_BARKER13)
    {
        // Zero-initialize the prefix
        for (uint32_t i = 0U; i < BARKER13_TOTAL_PREFIX_SAMPLES; i++)
        {
            output[i].re = 0;
            output[i].im = 0;
        }

        // Single base pointer with constant negative offsets to avoid both register spilling and index math
        const int32_t *p = &s_ptr[BARKER13_TOTAL_PREFIX_SAMPLES];
        Complex_t *out = &output[BARKER13_TOTAL_PREFIX_SAMPLES];

        for (uint32_t i = BARKER13_TOTAL_PREFIX_SAMPLES; i < FRAME_SAMPLES; i++)
        {
            int32_t acc = p[0];
            acc = __SSUB16(acc, p[BARKER_OFF(1)]);
            acc = __SADD16(acc, p[BARKER_OFF(2)]);
            acc = __SSUB16(acc, p[BARKER_OFF(3)]);
            acc = __SADD16(acc, p[BARKER_OFF(4)]);
            acc = __SADD16(acc, p[BARKER_OFF(5)]);
            acc = __SSUB16(acc, p[BARKER_OFF(6)]);
            acc = __SSUB16(acc, p[BARKER_OFF(7)]);
            acc = __SADD16(acc, p[BARKER_OFF(8)]);
            acc = __SADD16(acc, p[BARKER_OFF(9)]);
            acc = __SADD16(acc, p[BARKER_OFF(10)]);
            acc = __SADD16(acc, p[BARKER_OFF(11)]);
            acc = __SADD16(acc, p[BARKER_OFF(12)]);

            out->re = (int16_t)((int16_t)acc >> MATCHED_FILTER_SCALE_SHIFT);
            out->im = (int16_t)((int16_t)(acc >> 16) >> MATCHED_FILTER_SCALE_SHIFT);
            
            p++;
            out++;
        }
    }
    else
    {
        // Xung đơn Single (dài 8 mẫu), matched filter đơn giản chỉ là tổng dịch chuyển 8 mẫu chia cho 8
        for (uint32_t i = 0U; i < SINGLE_PULSE_PREFIX_SAMPLES; i++)
        {
            output[i].re = 0;
            output[i].im = 0;
        }
        for (uint32_t i = SINGLE_PULSE_PREFIX_SAMPLES; i < FRAME_SAMPLES; i++)
        {
            int32_t acc = s_ptr[i];
            output[i].re = (int16_t)((int16_t)acc >> MATCHED_FILTER_SCALE_SHIFT);
            output[i].im = (int16_t)((int16_t)(acc >> 16) >> MATCHED_FILTER_SCALE_SHIFT);
        }
    }
}

Complex_t* Receiver_GetComplexBuffer(uint32_t channel)
{
    if (channel >= 1U && channel <= CHANNEL_COUNT)
    {
        return complex_buffers[channel - 1U];
    }
    return NULL;
}

static void Receiver_SendAccumulatedFrame(void)
{
    uint32_t rx_chan = ComMgr_GetRxSelect();
    // Write channel ID to the 4th byte of the header ('0' for Rx Sum, '3' for Rx Diff)
    receiver_frame[3] = (uint8_t)('0' + rx_chan);
    int16_t *dest_payload = (int16_t *)&receiver_frame[FRAME_HEADER_SIZE];
    
    if (rx_chan == RX_CHANNEL_SUM)
    {
        for (uint32_t i = 0U; i < FRAME_SAMPLES; i++)
        {
            dest_payload[i] = (int16_t)(accumulated_sum_norm[i] >> ACCUM_NORM_SCALE_SHIFT);
        }
    }
    else if (rx_chan == RX_CHANNEL_DIFF)
    {
        for (uint32_t i = 0U; i < FRAME_SAMPLES; i++)
        {
            dest_payload[i] = (int16_t)(accumulated_diff_norm[i] >> ACCUM_NORM_SCALE_SHIFT);
        }
    }
    
    // Gửi gói tin qua ComMgr
    ComMgr_SendData(receiver_frame, sizeof(receiver_frame));
}

// Gửi frame dữ liệu của kênh đang chọn dựa trên chế độ truyền tải (Stream Mode)
static void Receiver_SendActiveFrame(void)
{
    uint32_t rx_chan = ComMgr_GetRxSelect();
    // Ghi số hiệu kênh nhận (1 hoặc 2) vào byte thứ 4 của header
    receiver_frame[3] = (uint8_t)('0' + rx_chan);
    StreamMode_t mode = ComMgr_GetStreamMode();

    if (mode == STREAM_MODE_COMPRESSED)
    {
        // Chế độ Compressed: Gửi năng lượng bình phương phức từ bộ lọc tương thích
        const Complex_t *chan_buf = compressed_buffers[rx_chan - 1U];
        int16_t *dest_payload = (int16_t *)&receiver_frame[FRAME_HEADER_SIZE];
        for (uint32_t i = 0U; i < FRAME_SAMPLES; i++)
        {
            int32_t packed_val = *(const int32_t *)&chan_buf[i];
            int32_t norm = __SMLAD(packed_val, packed_val, 0);
            dest_payload[i] = (int16_t)(norm >> STREAM_NORM_SCALE_SHIFT);
        }
    }
    else if (mode == STREAM_MODE_DEMOD)
    {
        // Chế độ IQ Demodulation: Gửi năng lượng bình phương phức giải điều chế
        const Complex_t *chan_buf = complex_buffers[rx_chan - 1U];
        int16_t *dest_payload = (int16_t *)&receiver_frame[FRAME_HEADER_SIZE];
        for (uint32_t i = 0U; i < FRAME_SAMPLES; i++)
        {
            int32_t packed_val = *(const int32_t *)&chan_buf[i];
            int32_t norm = __SMLAD(packed_val, packed_val, 0);
            dest_payload[i] = (int16_t)(norm >> STREAM_NORM_SCALE_SHIFT);
        }
    }
    else if (mode == STREAM_MODE_BPF)
    {
        // Chế độ Bandpass Filter: Gửi trực tiếp tín hiệu đã qua bộ lọc BPF từ static buffer
        const int16_t *send_buffer = (rx_chan == 1U) ? adc1_frame_buffer : adc2_frame_buffer;
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

static void Receiver_DetectAndSendTargets(void)
{
#ifdef SHOW_SAMPLING_LOG
    uint32_t t_detect_start = DWT->CYCCNT;
#endif

    const uint32_t *accum_buf = accumulated_sum_norm;

    // Tìm đỉnh tín hiệu trong vùng tín hiệu khả dụng (bỏ qua vùng nhiễu crosstalk phát ban đầu)
    uint32_t start_idx = 120U;
    uint32_t max_idx = start_idx;
    uint32_t max_val = accum_buf[start_idx];

    for (uint32_t i = start_idx + 1U; i < FRAME_SAMPLES; i++)
    {
        if (accum_buf[i] > max_val)
        {
            max_val = accum_buf[i];
            max_idx = i;
        }
    }

    // Trừ độ trễ nhóm của bộ lọc tương thích (Matched Filter Group Delay) để cự li chính xác tuyệt đối
    Transmitter_PulseType pulse_type = Transmitter_GetPulseType();
    uint32_t filter_delay = (pulse_type == TRANSMITTER_PULSE_BARKER13) ? BARKER13_TOTAL_PREFIX_SAMPLES : SINGLE_PULSE_PREFIX_SAMPLES;
    uint32_t true_idx = (max_idx > filter_delay) ? (max_idx - filter_delay) : 0U;

    // Tính Cự li (mét): Range = (true_idx * SPEED_OF_SOUND) / (2 * FS)
    // SPEED_OF_SOUND = 343.0 m/s, FS = 160000.0 Hz -> (343.0 / 320000.0) = 0.001071875
    float range_m = ((float)true_idx * 343.0f) / (2.0f * 160000.0f);

    // Tính Cường độ (dBV): Quy đổi giá trị tích lũy về điện thế RMS và tính dBV
    uint32_t avg_norm = max_val >> ACCUM_NORM_SCALE_SHIFT;
    float mag = sqrtf((float)avg_norm * 2048.0f);
    float voltage = (mag / 8192.0f) * 3.3f;
    float strength_dbv = 20.0f * log10f(fmaxf(voltage, 1e-4f));

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_detect_end = DWT->CYCCNT;
    t_detect_cycles = t_detect_end - t_detect_start;
#endif

    // Đóng gói khung dữ liệu mục tiêu TGT1
    TargetFrame_t target_frame;
    target_frame.header[0] = 'T';
    target_frame.header[1] = 'G';
    target_frame.header[2] = 'T';
    target_frame.header[3] = '1';
    target_frame.target_count = 1U;
    target_frame.reserved = 0U;
    target_frame.targets[0].range_m = range_m;
    target_frame.targets[0].strength_dbv = strength_dbv;
    target_frame.targets[0].angle_deg = 90;       // Mặc định góc 90 độ (sẽ bổ sung góc quét sau)
    target_frame.targets[0].reserved = 0;
    target_frame.targets[0].velocity_mps = 0.0f;  // Mặc định 0.0 m/s (sẽ bổ sung vận tốc sau)

    ComMgr_SendData(&target_frame, sizeof(target_frame));
}

static void Receiver_AccumulateAndProcess(void)
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
        union { Complex_t c; int32_t val; } u_rx1, u_rx2, u_sum;
        u_rx1.c = compressed_buffers[0][i];
        u_rx2.c = compressed_buffers[1][i];
        int32_t packed_rx1 = u_rx1.val;
        int32_t packed_rx2 = u_rx2.val;

        int32_t sum_packed = __SADD16(packed_rx1, packed_rx2);
        int32_t diff_packed = __SSUB16(packed_rx1, packed_rx2);

        u_sum.val = sum_packed;
        slow_time_accumulation[pulse_idx][i] = u_sum.c;

        int32_t sum_norm = __SMLAD(sum_packed, sum_packed, 0);
        int32_t diff_norm = __SMLAD(diff_packed, diff_packed, 0);

        accumulated_sum_norm[i] += (uint32_t)sum_norm;
        accumulated_diff_norm[i] += (uint32_t)diff_norm;
    }

#ifdef SHOW_SAMPLING_LOG
    uint32_t loop_end = DWT->CYCCNT;
    t_accum_cycles = loop_end - loop_start;
#endif

    uint32_t rx_chan = ComMgr_GetRxSelect();
    if ((pulse_idx == (ACCUMULATION_PULSES - 1U)) && (rx_chan == RX_CHANNEL_SUM || rx_chan == RX_CHANNEL_DIFF))
    {
#ifdef SHOW_SAMPLING_LOG
        t_send_start = DWT->CYCCNT;
#endif
        Receiver_SendAccumulatedFrame();
#ifdef SHOW_SAMPLING_LOG
        t_send_end = DWT->CYCCNT;
        sent = true;
#endif
    }

    pulse_idx = (pulse_idx + 1U) % ACCUMULATION_PULSES;
}

#ifdef SHOW_SAMPLING_LOG
static void Receiver_SendDSPLog(uint32_t t_start, uint32_t t_read_start, uint32_t t_read_end, 
                                 uint32_t t_bpf_start, uint32_t t_bpf_end, uint32_t t_demod_start, uint32_t t_demod_end,
                                 uint32_t t_mfilt_start, uint32_t t_mfilt_end,
                                 bool sent, uint32_t t_send_start, uint32_t t_send_end,
                                 uint32_t t_accum_cycles, uint32_t t_detect_cycles)
{
    uint32_t t_end = DWT->CYCCNT;

    log_counter++;
    if (log_counter >= LOG_INTERVAL_FRAMES)
    {
        log_counter = 0U;
        uint32_t clock_mhz = SystemCoreClock / US_PER_SEC;
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
        uint32_t detect_cycles = t_detect_cycles;

        uint32_t total_us = total_cycles / clock_mhz;
        uint32_t read_us = read_cycles / clock_mhz;
        uint32_t bpf_us = bpf_cycles / clock_mhz;
        uint32_t demod_us = demod_cycles / clock_mhz;
        uint32_t mfilt_us = mfilt_cycles / clock_mhz;
        uint32_t send_us = send_cycles / clock_mhz;
        uint32_t accum_us = accum_cycles / clock_mhz;
        uint32_t detect_us = detect_cycles / clock_mhz;

        uint8_t dsp_frame[DSP_FRAME_SIZE];
        dsp_frame[0] = 'D';
        dsp_frame[1] = 'S';
        dsp_frame[2] = 'P';
        dsp_frame[3] = '1';

        dsp_frame[4] = (uint8_t)(log_counter & 0xFFU);
        dsp_frame[5] = (uint8_t)((log_counter >> 8U) & 0xFFU);
        dsp_frame[6] = (uint8_t)((log_counter >> 16U) & 0xFFU);
        dsp_frame[7] = (uint8_t)((log_counter >> 24U) & 0xFFU);

        uint32_t values[DSP_VALUE_COUNT] = { total_us, read_us, bpf_us, demod_us, mfilt_us, send_us, accum_us, detect_us };
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
    for (uint32_t ch = 1U; ch <= CHANNEL_COUNT; ch++)
    {
        ADCService_Init(ch);
    }

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

    int16_t *buffers[CHANNEL_COUNT] = { adc1_frame_buffer, adc2_frame_buffer };
    uint32_t rx_chan = ComMgr_GetRxSelect();
    bool active_has_frame = (rx_chan >= 1U && rx_chan <= CHANNEL_COUNT);

    // Lưu lại tín hiệu thô trước khi lọc BPF cho kênh đang chọn
    if (active_has_frame)
    {
        memcpy(raw_active_buffer, buffers[rx_chan - 1U], FRAME_SAMPLES * sizeof(int16_t));
    }

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_read_end = DWT->CYCCNT;
    uint32_t t_bpf_start = t_read_end;
#endif

    // Luôn tiến hành lọc BPF đầy đủ trên cả 2 kênh để tính toán thời gian DSP chuẩn xác
    for (uint32_t i = 0U; i < CHANNEL_COUNT; i++)
    {
        Receiver_FilterBPF(buffers[i], &bpf_states[i]);
    }

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_bpf_end = DWT->CYCCNT;
    uint32_t t_demod_start = DWT->CYCCNT;
#endif

    // Luôn luôn thực hiện IQ Demodulate trên cả 2 kênh
    for (uint32_t i = 0U; i < CHANNEL_COUNT; i++)
    {
        Receiver_IQDemodulate(buffers[i], complex_buffers[i]);
    }

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_demod_end = DWT->CYCCNT;
    uint32_t t_mfilt_start = DWT->CYCCNT;
#endif

    // Luôn luôn thực hiện Matched Filter trên cả 2 kênh
    for (uint32_t i = 0U; i < CHANNEL_COUNT; i++)
    {
        Receiver_MatchedFilter(complex_buffers[i], compressed_buffers[i]);
    }

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_mfilt_end = DWT->CYCCNT;
#endif

#ifdef SHOW_SAMPLING_LOG
    t_send_start = 0U;
    t_send_end = 0U;
    t_accum_cycles = 0U;
    sent = false;
#endif

    Receiver_AccumulateAndProcess();

    // Gọi nhận diện và truyền thông tin mục tiêu trực tiếp từ Receiver_Process sau khi hoàn tất tích lũy xung thứ 8
    if (pulse_idx == 0U)
    {
        Receiver_DetectAndSendTargets();
    }

    if (active_has_frame)
    {
#ifdef SHOW_SAMPLING_LOG
        t_send_start = DWT->CYCCNT;
#endif
        Receiver_SendActiveFrame();
#ifdef SHOW_SAMPLING_LOG
        t_send_end = DWT->CYCCNT;
        sent = true;
#endif
    }

#ifdef SHOW_SAMPLING_LOG
    Receiver_SendDSPLog(t_start, t_read_start, t_read_end, t_bpf_start, t_bpf_end, 
                        t_demod_start, t_demod_end, t_mfilt_start, t_mfilt_end, sent, t_send_start, t_send_end,
                        t_accum_cycles, t_detect_cycles);
#endif
}