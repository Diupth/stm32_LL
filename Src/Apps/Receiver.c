#include "Receiver.h"
#include <string.h>
#include <math.h>

#include "ADCService.h"
#include "ComMgr.h"
#include "Transmitter.h"
#include "SyncSignalApp.h"
#ifdef SHOW_TIMING_LOG
#include "DWTService.h"
#endif

#define RECEIVER_FRAME_HEADER_SIZE 16U
#define RECEIVER_FRAME_PAYLOAD_SIZE (ADC_FRAME_SAMPLE_COUNT * sizeof(int16_t))
#define RECEIVER_FRAME_SIZE (RECEIVER_FRAME_HEADER_SIZE + RECEIVER_FRAME_PAYLOAD_SIZE)

#define ADC_BIAS 2048
#define USE_FFT_MATCHED_FILTER 1

static uint8_t receiver_frame[RECEIVER_FRAME_SIZE] __attribute__((aligned(4)));
static int16_t adc1_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t adc2_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t bpf1_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t bpf2_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static Complex_q31 iq1_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(8)));
static Complex_q31 iq2_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(8)));
static int16_t demod1_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t demod2_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t filtered1_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t filtered2_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));

#define BPF_NUM_STAGES 2U
static const float bpf_coeffs[BPF_NUM_STAGES * 5U] = {
    /* Stage 1: b0, b1, b2, -a1, -a2 */
    0.01440144f, -0.02880288f, 0.01440144f, -1.47902943f, -0.80560112f,
    /* Stage 2: b0, b1, b2, -a1, -a2 */
    1.00000000f,  2.00000000f,  1.00000000f, -1.69438393f, -0.85724673f
};
static float bpf_state[2][BPF_NUM_STAGES * 4U];

#define LPF_NUM_STAGES 2U
#define LPF_NUM_CHANNELS 4U  /* 0: Ch1_I, 1: Ch1_Q, 2: Ch2_I, 3: Ch2_Q */

/* Hệ số LPF Butterworth bậc 4, Fc = 2.0 kHz @ Fs = 96 kHz dạng float */
static const float lpf_coeffs_float[LPF_NUM_STAGES * 5U] = {
    /* Stage 1: b0, b1, b2, -a1, -a2 */
    0.00381725f, 0.00763449f, 0.00381725f,  1.76950435f, -0.78477333f,
    /* Stage 2: b0, b1, b2, -a1, -a2 */
    0.00407407f, 0.00814814f, 0.00407407f,  1.88855595f, -0.90485223f
};
static float lpf_state_float[LPF_NUM_CHANNELS][LPF_NUM_STAGES * 4U];

/* Cosine / Sine LUT cho sóng mang fc = 40.0 kHz @ Fs = 96 kHz (chu kỳ 12 mẫu) ở định dạng Q15 (1.0 = 32767) */
static const int16_t cos_carrier_lut_q15[12] = {
    32767, -28377, 16384, 0, -16383, 28377, -32767, 28377, -16384, 0, 16383, -28377
};
static const int16_t sin_carrier_lut_q15[12] = {
    0, 16383, -28377, 32767, -28377, 16384, 0, -16384, 28377, -32767, 28377, -16383
};

#ifdef SHOW_TIMING_LOG
static uint32_t dsp_log_sequence = 0U;
static uint32_t last_dsp_log_tick = 0U;
static uint32_t read_cycles = 0U;
static uint32_t bpf_cycles = 0U;
static uint32_t demod_cycles = 0U;
static uint32_t mfilt_cycles = 0U;
static uint32_t send_cycles = 0U;
static uint32_t total_cycles = 0U;

static void Receiver_SendTimingLog(void)
{
    uint8_t dsp_frame[40] = {'D', 'S', 'P', '1'};
    uint32_t accum_us = 0U;
    uint32_t detect_us = 0U;

    uint32_t values[9] = {
        dsp_log_sequence++,
        DWTService_CyclesToUs(total_cycles),
        DWTService_CyclesToUs(read_cycles),
        DWTService_CyclesToUs(bpf_cycles),
        DWTService_CyclesToUs(demod_cycles),
        DWTService_CyclesToUs(mfilt_cycles),
        DWTService_CyclesToUs(send_cycles),
        accum_us,
        detect_us
    };

    for (uint32_t i = 0U; i < 9U; i++)
    {
        for (uint32_t byte = 0U; byte < 4U; byte++)
        {
            dsp_frame[4U + i * 4U + byte] = (uint8_t)(values[i] >> (byte * 8U));
        }
    }

    ComMgr_SendData(dsp_frame, sizeof(dsp_frame));
}
#endif

static void FFT_InitCMSIS(void);

void Receiver_Init(void)
{
#ifdef SHOW_TIMING_LOG
    DWTService_Init();
    last_dsp_log_tick = HAL_GetTick();
#endif
    ADCService_Init(1U);
    ADCService_Init(2U);

    memset(bpf_state, 0, sizeof(bpf_state));
    memset(lpf_state_float, 0, sizeof(lpf_state_float));

    FFT_InitCMSIS();

    receiver_frame[0] = 'F';
    receiver_frame[1] = 'R';
    receiver_frame[2] = 'X';
    receiver_frame[3] = '1';
    receiver_frame[4] = (uint8_t)(ADC_FRAME_SAMPLE_COUNT & 0xFFU);
    receiver_frame[5] = (uint8_t)((ADC_FRAME_SAMPLE_COUNT >> 8U) & 0xFFU);
}

/**
 * @brief Bộ lọc thông dải số (Bandpass Filter) IIR Butterworth bậc 4 (38 - 42 kHz @ Fs = 96 kHz)
 * @details 
 *  - Cấu trúc: 2 tầng Biquad nối tầng (Cascaded Direct Form I SOS - Second Order Sections).
 *  - Dải thông: 38.0 kHz đến 42.0 kHz tại tần số lấy mẫu Fs = 96.0 kHz.
 *  - Phương trình sai phân cho mỗi tầng:
 *      y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] + (-a1)*y[n-1] + (-a2)*y[n-2]
 *  - Tự động nhận diện buffer input (kênh 1 hoặc kênh 2) để lưu trạng thái trễ riêng biệt giữa các frame.
 * 
 * @param input Con trỏ mảng mẫu thô đầu vào từ ADC (2048 mẫu, 12-bit)
 * @param output Con trỏ mảng mẫu kết quả sau lọc (2048 mẫu, 12-bit)
 */
void Receiver_BPF(const int16_t *input, int16_t *output)
{
    if (input == NULL || output == NULL)
    {
        return;
    }

    /* Xác định kênh dựa trên địa chỉ buffer đầu vào để lưu state trễ riêng */
    uint32_t idx = (input == adc2_frame_buffer) ? 1U : 0U;
    float *st = bpf_state[idx];

    /* Tải trạng thái trễ của tầng 1 và tầng 2 vào thanh ghi để tối ưu tốc độ */
    float s1_x1 = st[0], s1_x2 = st[1], s1_y1 = st[2], s1_y2 = st[3];
    float s2_x1 = st[4], s2_x2 = st[5], s2_y1 = st[6], s2_y2 = st[7];

    /* Hệ số SOS Biquad tầng 1 */
    const float b0_1 = bpf_coeffs[0], b1_1 = bpf_coeffs[1], b2_1 = bpf_coeffs[2];
    const float a1_1 = bpf_coeffs[3], a2_1 = bpf_coeffs[4];

    /* Hệ số SOS Biquad tầng 2 */
    const float b0_2 = bpf_coeffs[5], b1_2 = bpf_coeffs[6], b2_2 = bpf_coeffs[7];
    const float a1_2 = bpf_coeffs[8], a2_2 = bpf_coeffs[9];

    for (uint32_t n = 0U; n < ADC_FRAME_SAMPLE_COUNT; n++)
    {
        /* 1. Trừ mức phân cực một chiều ADC_BIAS (2048) để chuyển về tín hiệu xoay chiều quanh 0 */
        float x0 = (float)((int32_t)input[n] - ADC_BIAS);

        /* 2. Lọc Biquad Tầng 1 */
        float y1_val = b0_1 * x0 + b1_1 * s1_x1 + b2_1 * s1_x2 + a1_1 * s1_y1 + a2_1 * s1_y2;
        s1_x2 = s1_x1;
        s1_x1 = x0;
        s1_y2 = s1_y1;
        s1_y1 = y1_val;

        /* 3. Lọc Biquad Tầng 2 (Nhận ngõ ra của Tầng 1 làm ngõ vào) */
        float y2_val = b0_2 * y1_val + b1_2 * s2_x1 + b2_2 * s2_x2 + a1_2 * s2_y1 + a2_2 * s2_y2;
        s2_x2 = s2_x1;
        s2_x1 = y1_val;
        s2_y2 = s2_y1;
        s2_y1 = y2_val;

        /* 4. Khôi phục lại mức phân cực ADC_BIAS và bão hòa an toàn trong dải 12-bit [0, 4095] */
        int32_t result = (int32_t)y2_val + ADC_BIAS;
        output[n] = (int16_t)__USAT(result, 12U);
    }

    /* Lưu lại trạng thái trễ để lọc tiếp tục liền mạch cho frame kế tiếp */
    st[0] = s1_x1; st[1] = s1_x2; st[2] = s1_y1; st[3] = s1_y2;
    st[4] = s2_x1; st[5] = s2_x2; st[6] = s2_y1; st[7] = s2_y2;
}

/**
 * @brief Bộ lọc thông thấp IIR Butterworth bậc 4 (Fc = 2 kHz @ Fs = 96 kHz) xử lý tín hiệu
 * @details 
 *  - Cấu trúc: 2 tầng Biquad nối tầng (Cascaded Direct Form I SOS), FPU Cortex-M33 phần cứng.
 *  - Đầu vào và đầu ra giữ nguyên kiểu int32 để tương thích toàn bộ pipeline phức int32.
 * 
 * @param input Mảng tín hiệu đầu vào int32
 * @param output Mảng tín hiệu đầu ra int32 sau lọc
 * @param state_id Chỉ số kênh (0: Ch1_I, 1: Ch1_Q, 2: Ch2_I, 3: Ch2_Q)
 */
void Receiver_LPF(const int32_t *input, int32_t *output, uint32_t state_id)
{
    if (input == NULL || output == NULL || state_id >= LPF_NUM_CHANNELS)
    {
        return;
    }

    float *st = lpf_state_float[state_id];

    float s1_x1 = st[0], s1_x2 = st[1], s1_y1 = st[2], s1_y2 = st[3];
    float s2_x1 = st[4], s2_x2 = st[5], s2_y1 = st[6], s2_y2 = st[7];

    /* Hệ số tầng 1 */
    const float b0_1 = lpf_coeffs_float[0], b1_1 = lpf_coeffs_float[1], b2_1 = lpf_coeffs_float[2];
    const float a1_1 = lpf_coeffs_float[3], a2_1 = lpf_coeffs_float[4];

    /* Hệ số tầng 2 */
    const float b0_2 = lpf_coeffs_float[5], b1_2 = lpf_coeffs_float[6], b2_2 = lpf_coeffs_float[7];
    const float a1_2 = lpf_coeffs_float[8], a2_2 = lpf_coeffs_float[9];

    for (uint32_t n = 0U; n < ADC_FRAME_SAMPLE_COUNT; n++)
    {
        float x0 = (float)input[n];

        /* Tầng 1: Direct Form I SOS */
        float y1_val = b0_1 * x0 + b1_1 * s1_x1 + b2_1 * s1_x2 + a1_1 * s1_y1 + a2_1 * s1_y2;
        s1_x2 = s1_x1;
        s1_x1 = x0;
        s1_y2 = s1_y1;
        s1_y1 = y1_val;

        /* Tầng 2: Direct Form I SOS */
        float y2_val = b0_2 * y1_val + b1_2 * s2_x1 + b2_2 * s2_x2 + a1_2 * s2_y1 + a2_2 * s2_y2;
        s2_x2 = s2_x1;
        s2_x1 = y1_val;
        s2_y2 = s2_y1;
        s2_y1 = y2_val;

        output[n] = (int32_t)y2_val;
    }

    st[0] = s1_x1; st[1] = s1_x2; st[2] = s1_y1; st[3] = s1_y2;
    st[4] = s2_x1; st[5] = s2_x2; st[6] = s2_y1; st[7] = s2_y2;
}

/**
 * @brief Bộ giải điều chế số I/Q (Quadrature Demodulator)
 * @details
 *  - Chuyển phổ tín hiệu từ [39 kHz, 41 kHz] về [-1 kHz, +1 kHz] (Baseband quanh 0 Hz).
 *  - Tần số sóng mang cục bộ: fc = 40.0 kHz, tần số lấy mẫu Fs = 96.0 kHz.
 *  - Toàn bộ tính toán nhân trộn sóng và lọc LPF dùng số nguyên int32 / fixed-point.
 *  - Đầu ra phức (iq_output) lưu trữ trực tiếp (I, Q) dạng int32/Q31 để phục vụ matched filter / FFT vận tốc tiếp theo.
 *  - Đầu ra mag_output (nếu khác NULL) tính biên độ bao và đưa về 12-bit ADC bias phục vụ hiển thị / stream.
 *
 * @param input Con trỏ mảng mẫu đầu vào từ BPF (2048 mẫu, 12-bit)
 * @param iq_output Con trỏ mảng mẫu số phức int32 đầu ra (2048 phần tử Complex_q31)
 * @param mag_output Con trỏ mảng mẫu biên độ 12-bit (có thể NULL nếu không cần hiển thị)
 */
static int32_t mix_i[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int32_t mix_q[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int32_t lpf_i[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int32_t lpf_q[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));

void Receiver_IQDemodulator(const int16_t *input, Complex_q31 *iq_output, int16_t *mag_output)
{
    if (input == NULL)
    {
        return;
    }

    /* Xác định kênh: Kênh 1 (state 0 cho I, 1 cho Q), Kênh 2 (state 2 cho I, 3 cho Q) */
    uint32_t chan_idx = (input == bpf2_frame_buffer) ? 1U : 0U;
    uint32_t state_id_i = chan_idx * 2U;
    uint32_t state_id_q = chan_idx * 2U + 1U;

    /* Bước 1: Trộn tần số hạ dải (Down-mixing về Baseband) dạng int32 fixed-point */
    uint32_t lut_idx = 0U;
    for (uint32_t n = 0U; n < ADC_FRAME_SAMPLE_COUNT; n++)
    {
        int32_t x = (int32_t)input[n] - ADC_BIAS;
        int32_t cos_val = (int32_t)cos_carrier_lut_q15[lut_idx];
        int32_t sin_val = (int32_t)sin_carrier_lut_q15[lut_idx];

        lut_idx++;
        if (lut_idx >= 12U)
        {
            lut_idx = 0U;
        }

        /* Nhân với 2 và chia cho 32768 (Q15): (x * cos_val * 2) >> 15 = (x * cos_val) >> 14 */
        mix_i[n] = (x * cos_val) >> 14;
        mix_q[n] = (-x * sin_val) >> 14;
    }

    /* Bước 2: Lọc thông thấp IIR số nguyên int32 cho từng nhánh I và Q */
    Receiver_LPF(mix_i, lpf_i, state_id_i);
    Receiver_LPF(mix_q, lpf_q, state_id_q);

    /* Bước 3: Ghi nhận tín hiệu số phức int32 (Baseband I/Q) */
    if (iq_output != NULL)
    {
        for (uint32_t n = 0U; n < ADC_FRAME_SAMPLE_COUNT; n++)
        {
            iq_output[n].real = lpf_i[n];
            iq_output[n].imag = lpf_q[n];
        }
    }

    /* Bước 4: Tính biên độ bao 12-bit nếu có yêu cầu xuất ra mag_output */
    if (mag_output != NULL)
    {
        for (uint32_t n = 0U; n < ADC_FRAME_SAMPLE_COUNT; n++)
        {
            float fi = (float)lpf_i[n];
            float fq = (float)lpf_q[n];
            int32_t env = (int32_t)sqrtf(fi * fi + fq * fq);
            int32_t result = env + ADC_BIAS;
            mag_output[n] = (int16_t)__USAT(result, 12U);
        }
    }
}

static int16_t h_coeffs[TRANSMITTER_LFM_LENGTH] __attribute__((aligned(4)));
static uint32_t last_ref_len = 0U;
static const uint16_t *last_waveform_ptr = NULL;
static int16_t in_biased[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));

void Receiver_MatchedFilter(const int16_t *input, int16_t *output)
{
    const uint16_t *ref_waveform = NULL;
    uint32_t ref_len = Transmitter_GetActiveWaveform(&ref_waveform);

    if (input == NULL || output == NULL || ref_waveform == NULL || ref_len == 0U)
    {
        return;
    }

    /* Template h_coeffs[k] = ref_waveform[k] - ADC_BIAS để có thể tính tích chập x[n - ref_len + 1 + k] * h[k] */
    if (ref_len != last_ref_len || ref_waveform != last_waveform_ptr)
    {
        for (uint32_t i = 0U; i < ref_len; i++)
        {
            h_coeffs[i] = (int16_t)((int32_t)ref_waveform[i] - ADC_BIAS);
        }
        last_ref_len = ref_len;
        last_waveform_ptr = ref_waveform;
    }

    /* Bước 1: Trừ ADC_BIAS trước một lần duy nhất cho toàn bộ frame bằng SIMD __SSUB16 */
    const uint32_t *in_u32 = (const uint32_t *)(const void *)input;
    uint32_t *biased_u32 = (uint32_t *)(void *)in_biased;
    const uint32_t bias_pair = ((uint32_t)(uint16_t)ADC_BIAS) | (((uint32_t)(uint16_t)ADC_BIAS) << 16U);

    for (uint32_t i = 0U; i < ADC_FRAME_SAMPLE_COUNT / 2U; i++)
    {
        biased_u32[i] = __SSUB16(in_u32[i], bias_pair);
    }

    const int32_t divisor = (int32_t)(ref_len * 1024U);
    const uint32_t *h_ptr32 = (const uint32_t *)(const void *)h_coeffs;

    /* Bước 2: Đoạn đầu (n < ref_len - 1), tín hiệu chưa đi hết bộ lọc (tích chập từng phần) */
    for (uint32_t n = 0U; n < ref_len - 1U && n < ADC_FRAME_SAMPLE_COUNT; n++)
    {
        int32_t acc = 0;
        uint32_t k_len = n + 1U;
        const int16_t *x_ptr = &in_biased[0];
        const int16_t *h_sub = &h_coeffs[ref_len - k_len];

        uint32_t k = 0U;
        for (; k + 3U < k_len; k += 4U)
        {
            acc += (int32_t)x_ptr[k] * (int32_t)h_sub[k];
            acc += (int32_t)x_ptr[k + 1U] * (int32_t)h_sub[k + 1U];
            acc += (int32_t)x_ptr[k + 2U] * (int32_t)h_sub[k + 2U];
            acc += (int32_t)x_ptr[k + 3U] * (int32_t)h_sub[k + 3U];
        }
        for (; k < k_len; k++)
        {
            acc += (int32_t)x_ptr[k] * (int32_t)h_sub[k];
        }

        int32_t scaled = acc / divisor;
        int32_t result = scaled + ADC_BIAS;
        output[n] = (int16_t)__USAT(result, 12U);
    }

    /* Bước 3: Đoạn chính (n >= ref_len - 1), bộ lọc khớp hoàn toàn, sử dụng SIMD kép SMLAD unroll 16 mẫu */
    const uint32_t pairs = ref_len >> 1U;
    for (uint32_t n = ref_len - 1U; n < ADC_FRAME_SAMPLE_COUNT; n++)
    {
        int32_t acc = 0;
        const int16_t *x_sub = &in_biased[n - (ref_len - 1U)];
        const uint32_t *x_ptr32 = (const uint32_t *)(const void *)x_sub;

        /* Unroll 16 mẫu (8 lệnh SMLAD) mỗi vòng lặp */
        uint32_t j = 0U;
        for (; j + 7U < pairs; j += 8U)
        {
            acc = (int32_t)__SMLAD(x_ptr32[j],      h_ptr32[j],      (uint32_t)acc);
            acc = (int32_t)__SMLAD(x_ptr32[j + 1U], h_ptr32[j + 1U], (uint32_t)acc);
            acc = (int32_t)__SMLAD(x_ptr32[j + 2U], h_ptr32[j + 2U], (uint32_t)acc);
            acc = (int32_t)__SMLAD(x_ptr32[j + 3U], h_ptr32[j + 3U], (uint32_t)acc);
            acc = (int32_t)__SMLAD(x_ptr32[j + 4U], h_ptr32[j + 4U], (uint32_t)acc);
            acc = (int32_t)__SMLAD(x_ptr32[j + 5U], h_ptr32[j + 5U], (uint32_t)acc);
            acc = (int32_t)__SMLAD(x_ptr32[j + 6U], h_ptr32[j + 6U], (uint32_t)acc);
            acc = (int32_t)__SMLAD(x_ptr32[j + 7U], h_ptr32[j + 7U], (uint32_t)acc);
        }
        for (; j < pairs; j++)
        {
            acc = (int32_t)__SMLAD(x_ptr32[j], h_ptr32[j], (uint32_t)acc);
        }
        if (ref_len & 1U)
        {
            acc += (int32_t)x_sub[ref_len - 1U] * (int32_t)h_coeffs[ref_len - 1U];
        }

        int32_t scaled = acc / divisor;
        int32_t result = scaled + ADC_BIAS;
        output[n] = (int16_t)__USAT(result, 12U);
    }
}

/* ========================================================================= */
/*                   FREQUENCY DOMAIN MATCHED FILTER (CMSIS-DSP)            */
/* ========================================================================= */
#include "arm_math.h"
#include "arm_const_structs.h"

#define FFT_SIZE 4096U

static arm_rfft_fast_instance_f32 rfft_instance;
static bool rfft_initialized = false;

static float fft_in[FFT_SIZE] __attribute__((aligned(4)));
static float fft_out[FFT_SIZE] __attribute__((aligned(4)));
static float fft_h_buf[FFT_SIZE] __attribute__((aligned(4)));
static float fft_x_buf[FFT_SIZE] __attribute__((aligned(4)));
static float fft_prod[FFT_SIZE] __attribute__((aligned(4)));

static uint32_t last_fft_ref_len = 0U;
static const uint16_t *last_fft_waveform_ptr = NULL;

static void FFT_InitCMSIS(void)
{
    if (!rfft_initialized)
    {
        arm_rfft_fast_init_f32(&rfft_instance, FFT_SIZE);
        rfft_initialized = true;
    }
}

void Receiver_MatchedFilterFFT(const int16_t *input, int16_t *output)
{
    const uint16_t *ref_waveform = NULL;
    uint32_t ref_len = Transmitter_GetActiveWaveform(&ref_waveform);

    if (input == NULL || output == NULL || ref_waveform == NULL || ref_len == 0U)
    {
        return;
    }

    if (!rfft_initialized)
    {
        FFT_InitCMSIS();
    }

    /* 1. Tiền tính toán phổ liên hợp H*(f) của tín hiệu mẫu khi mẫu thay đổi */
    if (ref_len != last_fft_ref_len || ref_waveform != last_fft_waveform_ptr)
    {
        memset(fft_in, 0, sizeof(fft_in));
        for (uint32_t i = 0U; i < ref_len; i++)
        {
            fft_in[i] = (float)((int32_t)ref_waveform[i] - ADC_BIAS);
        }

        /* Forward RFFT của mẫu phát */
        arm_rfft_fast_f32(&rfft_instance, fft_in, fft_h_buf, 0);

        /* Lấy liên hợp phức: H*(f) = Re(H) - j * Im(H) */
        for (uint32_t i = 3U; i < FFT_SIZE; i += 2U)
        {
            fft_h_buf[i] = -fft_h_buf[i];
        }

        last_fft_ref_len = ref_len;
        last_fft_waveform_ptr = ref_waveform;
    }

    /* 2. Nạp tín hiệu vào, trừ bias ADC và zero-pad */
    for (uint32_t i = 0U; i < ADC_FRAME_SAMPLE_COUNT; i++)
    {
        fft_in[i] = (float)((int32_t)input[i] - ADC_BIAS);
    }
    memset(&fft_in[ADC_FRAME_SAMPLE_COUNT], 0, (FFT_SIZE - ADC_FRAME_SAMPLE_COUNT) * sizeof(float));

    /* 3. Forward RFFT của tín hiệu thu X(f) */
    arm_rfft_fast_f32(&rfft_instance, fft_in, fft_x_buf, 0);

    /* 4. Nhân chập miền tần số (Tương quan phức): Y(f) = X(f) * H*(f) */
    /* Bin DC và Nyquist là số thực thuần */
    fft_prod[0] = fft_x_buf[0] * fft_h_buf[0];
    fft_prod[1] = fft_x_buf[1] * fft_h_buf[1];

    /* Các bin phức từ index 2 đến FFT_SIZE-1 */
    arm_cmplx_mult_cmplx_f32(&fft_x_buf[2], &fft_h_buf[2], &fft_prod[2], (FFT_SIZE - 2U) / 2U);

    /* 5. Inverse RFFT: khôi phục tín hiệu miền thời gian ra buffer riêng fft_out */
    arm_rfft_fast_f32(&rfft_instance, fft_prod, fft_out, 1);

    /* 6. Chuẩn hóa scale biên độ và khôi phục bias 12-bit ADC */
    const float scale = 1.0f / (float)(ref_len * 1024U);
    for (uint32_t n = 0U; n < ADC_FRAME_SAMPLE_COUNT; n++)
    {
        float val = fft_out[n] * scale;
        int32_t result = (int32_t)val + ADC_BIAS;
        output[n] = (int16_t)__USAT(result, 12U);
    }
}

static void Receiver_SendFrame(int16_t *const raw_buffers[2],
                               int16_t *const bpf_buffers[2],
                               int16_t *const demod_buffers[2],
                               int16_t *const filtered_buffers[2])
{
    uint32_t rx_select = ComMgr_GetRxSelect();
    ComMgr_StreamMode mode = ComMgr_GetStreamMode();
    int16_t *const *active_buffers = raw_buffers;

    if (mode == COMMGR_STREAM_BPF)
    {
        active_buffers = bpf_buffers;
    }
    else if (mode == COMMGR_STREAM_DEMODULATED)
    {
        active_buffers = demod_buffers;
    }
    else if (mode == COMMGR_STREAM_COMPRESSED)
    {
        active_buffers = filtered_buffers;
    }

    const int16_t *send_buf = NULL;
    static int16_t calc_buf[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));

    if (rx_select == 1U || rx_select == 2U)
    {
        send_buf = active_buffers[rx_select - 1U];
    }
    else if (rx_select == 0U)
    {
        // Rx Sum = (Rx1 + Rx2) / 2
        const int16_t *b1 = active_buffers[0];
        const int16_t *b2 = active_buffers[1];
        for (uint32_t i = 0U; i < ADC_FRAME_SAMPLE_COUNT; i++)
        {
            calc_buf[i] = (int16_t)(((int32_t)b1[i] + (int32_t)b2[i]) / 2);
        }
        send_buf = calc_buf;
    }
    else if (rx_select == 3U)
    {
        // Rx Diff = (Rx1 - Rx2) / 2 + ADC_BIAS
        const int16_t *b1 = active_buffers[0];
        const int16_t *b2 = active_buffers[1];
        for (uint32_t i = 0U; i < ADC_FRAME_SAMPLE_COUNT; i++)
        {
            calc_buf[i] = (int16_t)(((int32_t)b1[i] - (int32_t)b2[i]) / 2 + ADC_BIAS);
        }
        send_buf = calc_buf;
    }

    if (send_buf != NULL)
    {
        receiver_frame[3] = (uint8_t)('0' + rx_select);
        memcpy(&receiver_frame[RECEIVER_FRAME_HEADER_SIZE], send_buf, RECEIVER_FRAME_PAYLOAD_SIZE);
        ComMgr_SendData(receiver_frame, sizeof(receiver_frame));
    }
}

void Receiver_Process(void)
{
    /* Kiểm tra và đồng bộ đủ 2 kênh ADC từ SyncSignalApp trước khi bắt đầu xử lý DSP */
    if (!SyncSignalApp_WaitForFrames())
    {
        ComMgr_Process();
        return;
    }

    int16_t *adc_buffers[2] = {adc1_frame_buffer, adc2_frame_buffer};
    int16_t *bpf_buffers[2] = {bpf1_frame_buffer, bpf2_frame_buffer};
    Complex_q31 *iq_buffers[2] = {iq1_frame_buffer, iq2_frame_buffer};
    int16_t *demod_buffers[2] = {demod1_frame_buffer, demod2_frame_buffer};
    int16_t *filtered_buffers[2] = {filtered1_frame_buffer, filtered2_frame_buffer};

#ifdef SHOW_TIMING_LOG
    uint32_t t_start = DWTService_GetCycles();
    read_cycles = 0U;
    bpf_cycles = 0U;
    demod_cycles = 0U;
    mfilt_cycles = 0U;
    send_cycles = 0U;
#endif

    /* 1. Thu thập dữ liệu ADC, lọc BPF, giải điều chế I/Q trên cả 2 kênh thu */
    for (uint32_t chan = 1U; chan <= 2U; chan++)
    {
#ifdef SHOW_TIMING_LOG
        uint32_t t_read_start = DWTService_GetCycles();
        ADCService_ReadFrame(chan, adc_buffers[chan - 1U]);
        read_cycles += (DWTService_GetCycles() - t_read_start);

        uint32_t t_bpf_start = DWTService_GetCycles();
        Receiver_BPF(adc_buffers[chan - 1U], bpf_buffers[chan - 1U]);
        bpf_cycles += (DWTService_GetCycles() - t_bpf_start);

        uint32_t t_demod_start = DWTService_GetCycles();
        Receiver_IQDemodulator(bpf_buffers[chan - 1U], iq_buffers[chan - 1U], demod_buffers[chan - 1U]);
        demod_cycles += (DWTService_GetCycles() - t_demod_start);

        /* Tạm thời comment đoạn xử lý matched filter */
        /*
        uint32_t t_mfilt_start = DWTService_GetCycles();
#if defined(USE_FFT_MATCHED_FILTER)
        Receiver_MatchedFilterFFT(bpf_buffers[chan - 1U], filtered_buffers[chan - 1U]);
#else
        Receiver_MatchedFilter(bpf_buffers[chan - 1U], filtered_buffers[chan - 1U]);
#endif
        mfilt_cycles += (DWTService_GetCycles() - t_mfilt_start);
        */
#else
        ADCService_ReadFrame(chan, adc_buffers[chan - 1U]);
        Receiver_BPF(adc_buffers[chan - 1U], bpf_buffers[chan - 1U]);
        Receiver_IQDemodulator(bpf_buffers[chan - 1U], iq_buffers[chan - 1U], demod_buffers[chan - 1U]);
        /* Tạm thời comment đoạn xử lý matched filter */
        /*
#if defined(USE_FFT_MATCHED_FILTER)
        Receiver_MatchedFilterFFT(bpf_buffers[chan - 1U], filtered_buffers[chan - 1U]);
#else
        Receiver_MatchedFilter(bpf_buffers[chan - 1U], filtered_buffers[chan - 1U]);
#endif
        */
#endif
    }

    /* 2. Gửi tín hiệu theo cấu hình Rx select (1 hoặc 2) và Stream Mode */
#ifdef SHOW_TIMING_LOG
    uint32_t t_send_start = DWTService_GetCycles();
#endif
    Receiver_SendFrame(adc_buffers, bpf_buffers, demod_buffers, filtered_buffers);
#ifdef SHOW_TIMING_LOG
    send_cycles = DWTService_GetCycles() - t_send_start;
    total_cycles = DWTService_GetCycles() - t_start;

    uint32_t now = HAL_GetTick();
    if (now - last_dsp_log_tick >= 1000U)
    {
        last_dsp_log_tick = now;
        Receiver_SendTimingLog();
    }
#endif

    /* 3. Xử lý truyền thông USB */
    ComMgr_Process();
}