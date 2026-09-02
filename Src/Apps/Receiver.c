#include "Receiver.h"
#include <string.h>
#include <math.h>
#include "arm_math.h"
#include "arm_const_structs.h"

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
#define RECEIVER_NUM_CHANNELS 2U
#define USE_FFT_MATCHED_FILTER 1

/* Buffer gửi UART luôn cấp đủ cho khung lớn nhất (2048 mẫu * 2 byte + header) */
static uint8_t receiver_frame[RECEIVER_FRAME_SIZE] __attribute__((aligned(4)));
static int16_t adc1_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t adc2_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t bpf1_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t bpf2_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static Complex_q31 iq1_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(8)));
static Complex_q31 iq2_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(8)));
static Complex_q31 ds_iq1_frame_buffer[RECEIVER_DOWNSAMPLED_SAMPLE_COUNT] __attribute__((aligned(8)));
static Complex_q31 ds_iq2_frame_buffer[RECEIVER_DOWNSAMPLED_SAMPLE_COUNT] __attribute__((aligned(8)));
static Complex_q31 filtered_iq1_frame_buffer[RECEIVER_DOWNSAMPLED_SAMPLE_COUNT] __attribute__((aligned(8)));
static Complex_q31 filtered_iq2_frame_buffer[RECEIVER_DOWNSAMPLED_SAMPLE_COUNT] __attribute__((aligned(8)));

/* 8 mảng phức tổng và 8 mảng phức hiệu sau Matched Filter (mỗi mảng 128 mẫu @ 6 kHz) */
static Complex_q31 accumulated_sum_pulses[RECEIVER_ACCUMULATED_PULSE_COUNT][RECEIVER_DOWNSAMPLED_SAMPLE_COUNT] __attribute__((aligned(8)));
static Complex_q31 accumulated_diff_pulses[RECEIVER_ACCUMULATED_PULSE_COUNT][RECEIVER_DOWNSAMPLED_SAMPLE_COUNT] __attribute__((aligned(8)));
static uint32_t accumulated_pulse_idx = 0U;
static bool accumulation_complete = false;

/* Ma trận độ lớn Range-Doppler tổng 16x128 (16 slow-time Doppler bins x 128 range bins) dạng uint16 */
static uint16_t rd_sum_mag_matrix[RECEIVER_DOPPLER_BIN_COUNT][RECEIVER_DOWNSAMPLED_SAMPLE_COUNT] __attribute__((aligned(4)));
static uint8_t rd_receiver_frame[RECEIVER_FRAME_HEADER_SIZE + RECEIVER_RD_MATRIX_SIZE * sizeof(uint16_t)] __attribute__((aligned(4)));

static int16_t demod1_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t demod2_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t ds1_frame_buffer[RECEIVER_DOWNSAMPLED_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t ds2_frame_buffer[RECEIVER_DOWNSAMPLED_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t filtered1_frame_buffer[RECEIVER_DOWNSAMPLED_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t filtered2_frame_buffer[RECEIVER_DOWNSAMPLED_SAMPLE_COUNT] __attribute__((aligned(4)));
static uint32_t last_cmplx_fft_ref_len = 0U;

#define BPF_NUM_STAGES 2U
static int32_t bpf_state_fx[2][BPF_NUM_STAGES * 4U];

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
static uint32_t ds_cycles = 0U;
static uint32_t mfilt_cycles = 0U;
static uint32_t rd_cycles = 0U;
static uint32_t last_rd_cycles = 0U;
static uint32_t send_cycles = 0U;
static uint32_t total_cycles = 0U;

static void Receiver_SendTimingLog(void)
{
    uint8_t dsp_frame[40] = {'D', 'S', 'P', '1'};

    uint32_t values[9] = {
        dsp_log_sequence++,
        DWTService_CyclesToUs(total_cycles),
        DWTService_CyclesToUs(read_cycles),
        DWTService_CyclesToUs(bpf_cycles),
        DWTService_CyclesToUs(demod_cycles),
        DWTService_CyclesToUs(mfilt_cycles),
        DWTService_CyclesToUs(send_cycles),
        DWTService_CyclesToUs(ds_cycles),
        DWTService_CyclesToUs(last_rd_cycles)
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

void Receiver_ResetAccumulation(void)
{
    accumulated_pulse_idx = 0U;
    accumulation_complete = false;
    memset(accumulated_sum_pulses, 0, sizeof(accumulated_sum_pulses));
    memset(accumulated_diff_pulses, 0, sizeof(accumulated_diff_pulses));
}

bool Receiver_IsAccumulationComplete(void)
{
    return accumulation_complete;
}

uint32_t Receiver_GetAccumulatedPulseCount(void)
{
    return accumulated_pulse_idx;
}

const Complex_q31 (*Receiver_GetSumPulses(void))[RECEIVER_DOWNSAMPLED_SAMPLE_COUNT]
{
    return (const Complex_q31 (*)[RECEIVER_DOWNSAMPLED_SAMPLE_COUNT])accumulated_sum_pulses;
}

const Complex_q31 (*Receiver_GetDiffPulses(void))[RECEIVER_DOWNSAMPLED_SAMPLE_COUNT]
{
    return (const Complex_q31 (*)[RECEIVER_DOWNSAMPLED_SAMPLE_COUNT])accumulated_diff_pulses;
}

const uint16_t *Receiver_GetRangeDopplerMagSum(void)
{
    return (const uint16_t *)rd_sum_mag_matrix;
}

/**
 * @brief Tính ma trận Range-Doppler từ 8 xung phức tổng bằng 8-point Complex FFT dọc theo slow-time
 * @details
 *  - Với mỗi Range bin r (0..127): Lấy đúng 8 mẫu phức qua 8 xung [0..7] (không zero-padding).
 *  - Nhân cửa sổ Hamming 8 điểm để triệt tiêu búp phụ Doppler sidelobes.
 *  - Thực hiện 8-point Forward Complex FFT trực tiếp (Radix-2 DIT tối ưu FPU Cortex-M33).
 *  - Áp dụng fftshift: 8 Doppler bins [-4, -3, -2, -1, 0, 1, 2, 3] ứng với bins [4, 5, 6, 7, 0, 1, 2, 3].
 *  - Tính độ lớn biên độ bao (magnitude) lưu vào rd_sum_mag_matrix[8][128].
 */
void Receiver_ComputeRangeDoppler(void)
{
    const float SQRT1_2 = 0.7071067811865475f; /* cos(pi/4) / sin(pi/4) */
    static const float hamming8[8] = {
        0.08000000f, 0.25319469f, 0.64236894f, 0.95443637f,
        0.95443637f, 0.64236894f, 0.25319469f, 0.08000000f
    };
    static const uint8_t shift_map[8] = {4, 5, 6, 7, 0, 1, 2, 3};

    for (uint32_t r = 0U; r < RECEIVER_DOWNSAMPLED_SAMPLE_COUNT; r++)
    {
        /* 1. Nạp 8 mẫu phức và nhân cửa sổ Hamming */
        float x_r[8];
        float x_i[8];
        for (uint32_t p = 0U; p < RECEIVER_ACCUMULATED_PULSE_COUNT; p++)
        {
            float w = hamming8[p];
            x_r[p] = ((float)accumulated_sum_pulses[p][r].real) * w;
            x_i[p] = ((float)accumulated_sum_pulses[p][r].imag) * w;
        }

        /* 2. Thực thi 8-point Forward Complex FFT (Radix-2 DIT) */
        /* Stage 1: 2-point butterflies */
        float a0_r = x_r[0] + x_r[4], a0_i = x_i[0] + x_i[4];
        float a1_r = x_r[0] - x_r[4], a1_i = x_i[0] - x_i[4];
        float a2_r = x_r[2] + x_r[6], a2_i = x_i[2] + x_i[6];
        float a3_r = x_r[2] - x_r[6], a3_i = x_i[2] - x_i[6];
        float a4_r = x_r[1] + x_r[5], a4_i = x_i[1] + x_i[5];
        float a5_r = x_r[1] - x_r[5], a5_i = x_i[1] - x_i[5];
        float a6_r = x_r[3] + x_r[7], a6_i = x_i[3] + x_i[7];
        float a7_r = x_r[3] - x_r[7], a7_i = x_i[3] - x_i[7];

        /* Stage 2: 4-point butterflies */
        float b0_r = a0_r + a2_r, b0_i = a0_i + a2_i;
        float b1_r = a1_r + a3_i, b1_i = a1_i - a3_r; /* nhân -j */
        float b2_r = a0_r - a2_r, b2_i = a0_i - a2_i;
        float b3_r = a1_r - a3_i, b3_i = a1_i + a3_r; /* nhân +j */

        float b4_r = a4_r + a6_r, b4_i = a4_i + a6_i;
        float b5_r = a5_r + a7_i, b5_i = a5_i - a7_r; /* nhân -j */
        float b6_r = a4_r - a6_r, b6_i = a4_i - a6_i;
        float b7_r = a5_r - a7_i, b7_i = a5_i + a7_r; /* nhân +j */

        /* Twiddle nhân với W8^1, W8^2, W8^3 cho nhóm lẻ */
        /* W8^1 = (1 - j)/sqrt(2) */
        float t5_r = (b5_r + b5_i) * SQRT1_2;
        float t5_i = (b5_i - b5_r) * SQRT1_2;

        /* W8^2 = -j */
        float t6_r = b6_i;
        float t6_i = -b6_r;

        /* W8^3 = (-1 - j)/sqrt(2) */
        float t7_r = (-b7_r + b7_i) * SQRT1_2;
        float t7_i = (-b7_i - b7_r) * SQRT1_2;

        /* Stage 3: Kết hợp 8-point FFT output X[0..7] */
        float X_r[8], X_i[8];
        X_r[0] = b0_r + b4_r; X_i[0] = b0_i + b4_i;
        X_r[1] = b1_r + t5_r; X_i[1] = b1_i + t5_i;
        X_r[2] = b2_r + t6_r; X_i[2] = b2_i + t6_i;
        X_r[3] = b3_r + t7_r; X_i[3] = b3_i + t7_i;

        X_r[4] = b0_r - b4_r; X_i[4] = b0_i - b4_i;
        X_r[5] = b1_r - t5_r; X_i[5] = b1_i - t5_i;
        X_r[6] = b2_r - t6_r; X_i[6] = b2_i - t6_i;
        X_r[7] = b3_r - t7_r; X_i[7] = b3_i - t7_i;

        /* 3. FFT Shift và tính độ lớn Magnitude:
         * 8 Doppler bins đối xứng từ -4 đến +3:
         * d=0 (Doppler -4) -> X[4]
         * d=1 (Doppler -3) -> X[5]
         * d=2 (Doppler -2) -> X[6]
         * d=3 (Doppler -1) -> X[7]
         * d=4 (Doppler  0) -> X[0] (0 Hz nằm ở giữa bin 4)
         * d=5 (Doppler +1) -> X[1]
         * d=6 (Doppler +2) -> X[2]
         * d=7 (Doppler +3) -> X[3]
         */
        for (uint32_t d = 0U; d < RECEIVER_DOPPLER_BIN_COUNT; d++)
        {
            uint8_t k = shift_map[d];
            float re = X_r[k];
            float im = X_i[k];
            float mag = sqrtf(re * re + im * im);
            /* Chuẩn hóa chia cho 8 (Doppler processing gain) */
            uint32_t mag_u32 = (uint32_t)lroundf(mag / 8.0f);
            rd_sum_mag_matrix[d][r] = (uint16_t)__USAT(mag_u32, 16U);
        }
    }
}

void Receiver_Init(void)
{
#ifdef SHOW_TIMING_LOG
    DWTService_Init();
    last_dsp_log_tick = HAL_GetTick();
#endif
    ADCService_Init(1U);
    ADCService_Init(2U);

    memset(bpf_state_fx, 0, sizeof(bpf_state_fx));
    memset(lpf_state_float, 0, sizeof(lpf_state_float));
    last_cmplx_fft_ref_len = 0U;
    Receiver_ResetAccumulation();

    FFT_InitCMSIS();

    receiver_frame[0] = 'F';
    receiver_frame[1] = 'R';
    receiver_frame[2] = 'X';
    receiver_frame[3] = '1';
    receiver_frame[4] = (uint8_t)(ADC_FRAME_SAMPLE_COUNT & 0xFFU);
    receiver_frame[5] = (uint8_t)((ADC_FRAME_SAMPLE_COUNT >> 8U) & 0xFFU);

    memset(rd_receiver_frame, 0, RECEIVER_FRAME_HEADER_SIZE);
    rd_receiver_frame[0] = 'F';
    rd_receiver_frame[1] = 'R';
    rd_receiver_frame[2] = 'X';
    rd_receiver_frame[3] = '0';
    rd_receiver_frame[4] = (uint8_t)(RECEIVER_RD_MATRIX_SIZE & 0xFFU);
    rd_receiver_frame[5] = (uint8_t)((RECEIVER_RD_MATRIX_SIZE >> 8U) & 0xFFU);
}

/*
 * Đóng gói hệ số vào các cặp 32-bit (chứa 2 hệ số int16_t) để tận dụng lệnh SIMD __SMLAD:
 *   __SMLAD(val1, val2, acc): nhân đồng thời 2 cặp 16-bit và cộng dồn vào thanh ghi 32-bit trong 1 chu kỳ.
 */
static const uint32_t bpf_h1_b1b2 = ((uint32_t)(uint16_t)(-472)) | (((uint32_t)(uint16_t)236) << 16U);
static const uint32_t bpf_h1_a1a2 = ((uint32_t)(uint16_t)(-24232)) | (((uint32_t)(uint16_t)(-13199)) << 16U);

static const uint32_t bpf_h2_b1b2 = ((uint32_t)(uint16_t)32767) | (((uint32_t)(uint16_t)16384) << 16U);
static const uint32_t bpf_h2_a1a2 = ((uint32_t)(uint16_t)(-27761)) | (((uint32_t)(uint16_t)(-14045)) << 16U);

/**
 * @brief Bộ lọc thông dải số (Bandpass Filter) SIMD Q14/Q15 trên ARM Cortex-M33
 * @details 
 *  - Sử dụng lệnh SIMD ghép đôi __SMLAD (Signed Multiply and Add Dual) thực thi 2 phép nhân 16-bit 
 *    và cộng dồn vào accumulator 32-bit chỉ trong 1 chu kỳ clock.
 *  - Đóng gói các biến trạng thái theo từng cặp (x1, x2) và (y1, y2) trong word 32-bit.
 *  - Mở rộng vòng lặp unroll 4x tối ưu thông lượng pipeline CPU.
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

    uint32_t idx = (input == adc2_frame_buffer) ? 1U : 0U;
    int32_t *st = bpf_state_fx[idx];

    /* Tải các biến trạng thái vào CPU registers */
    int32_t s1_x1 = st[0], s1_x2 = st[1], s1_y1 = st[2], s1_y2 = st[3];
    int32_t s2_x1 = st[4], s2_x2 = st[5], s2_y1 = st[6], s2_y2 = st[7];

    const uint32_t h1_b1b2 = bpf_h1_b1b2;
    const uint32_t h1_a1a2 = bpf_h1_a1a2;
    const uint32_t h2_b1b2 = bpf_h2_b1b2;
    const uint32_t h2_a1a2 = bpf_h2_a1a2;

    for (uint32_t n = 0U; n < ADC_FRAME_SAMPLE_COUNT; n += 4U)
    {
        /* ===== Mẫu 1 (n) ===== */
        int32_t x0_0 = (int32_t)input[n] - ADC_BIAS;
        uint32_t s1_x1x2_0 = ((uint32_t)(uint16_t)s1_x1) | (((uint32_t)(uint16_t)s1_x2) << 16U);
        uint32_t s1_y1y2_0 = ((uint32_t)(uint16_t)s1_y1) | (((uint32_t)(uint16_t)s1_y2) << 16U);
        
        int32_t acc1_0 = 236 * x0_0;
        acc1_0 = (int32_t)__SMLAD(s1_x1x2_0, h1_b1b2, (uint32_t)acc1_0);
        acc1_0 = (int32_t)__SMLAD(s1_y1y2_0, h1_a1a2, (uint32_t)acc1_0);
        int32_t y1_0 = acc1_0 >> 14;
        s1_x2 = s1_x1; s1_x1 = x0_0;
        s1_y2 = s1_y1; s1_y1 = y1_0;

        uint32_t s2_x1x2_0 = ((uint32_t)(uint16_t)s2_x1) | (((uint32_t)(uint16_t)s2_x2) << 16U);
        uint32_t s2_y1y2_0 = ((uint32_t)(uint16_t)s2_y1) | (((uint32_t)(uint16_t)s2_y2) << 16U);
        int32_t acc2_0 = 16384 * y1_0;
        acc2_0 = (int32_t)__SMLAD(s2_x1x2_0, h2_b1b2, (uint32_t)acc2_0);
        acc2_0 = (int32_t)__SMLAD(s2_y1y2_0, h2_a1a2, (uint32_t)acc2_0);
        int32_t y2_0 = acc2_0 >> 14;
        s2_x2 = s2_x1; s2_x1 = y1_0;
        s2_y2 = s2_y1; s2_y1 = y2_0;
        output[n] = (int16_t)__SSAT(y2_0, 16U);

        /* ===== Mẫu 2 (n + 1) ===== */
        int32_t x0_1 = (int32_t)input[n + 1U] - ADC_BIAS;
        uint32_t s1_x1x2_1 = ((uint32_t)(uint16_t)s1_x1) | (((uint32_t)(uint16_t)s1_x2) << 16U);
        uint32_t s1_y1y2_1 = ((uint32_t)(uint16_t)s1_y1) | (((uint32_t)(uint16_t)s1_y2) << 16U);
        
        int32_t acc1_1 = 236 * x0_1;
        acc1_1 = (int32_t)__SMLAD(s1_x1x2_1, h1_b1b2, (uint32_t)acc1_1);
        acc1_1 = (int32_t)__SMLAD(s1_y1y2_1, h1_a1a2, (uint32_t)acc1_1);
        int32_t y1_1 = acc1_1 >> 14;
        s1_x2 = s1_x1; s1_x1 = x0_1;
        s1_y2 = s1_y1; s1_y1 = y1_1;

        uint32_t s2_x1x2_1 = ((uint32_t)(uint16_t)s2_x1) | (((uint32_t)(uint16_t)s2_x2) << 16U);
        uint32_t s2_y1y2_1 = ((uint32_t)(uint16_t)s2_y1) | (((uint32_t)(uint16_t)s2_y2) << 16U);
        int32_t acc2_1 = 16384 * y1_1;
        acc2_1 = (int32_t)__SMLAD(s2_x1x2_1, h2_b1b2, (uint32_t)acc2_1);
        acc2_1 = (int32_t)__SMLAD(s2_y1y2_1, h2_a1a2, (uint32_t)acc2_1);
        int32_t y2_1 = acc2_1 >> 14;
        s2_x2 = s2_x1; s2_x1 = y1_1;
        s2_y2 = s2_y1; s2_y1 = y2_1;
        output[n + 1U] = (int16_t)__SSAT(y2_1, 16U);

        /* ===== Mẫu 3 (n + 2) ===== */
        int32_t x0_2 = (int32_t)input[n + 2U] - ADC_BIAS;
        uint32_t s1_x1x2_2 = ((uint32_t)(uint16_t)s1_x1) | (((uint32_t)(uint16_t)s1_x2) << 16U);
        uint32_t s1_y1y2_2 = ((uint32_t)(uint16_t)s1_y1) | (((uint32_t)(uint16_t)s1_y2) << 16U);
        
        int32_t acc1_2 = 236 * x0_2;
        acc1_2 = (int32_t)__SMLAD(s1_x1x2_2, h1_b1b2, (uint32_t)acc1_2);
        acc1_2 = (int32_t)__SMLAD(s1_y1y2_2, h1_a1a2, (uint32_t)acc1_2);
        int32_t y1_2 = acc1_2 >> 14;
        s1_x2 = s1_x1; s1_x1 = x0_2;
        s1_y2 = s1_y1; s1_y1 = y1_2;

        uint32_t s2_x1x2_2 = ((uint32_t)(uint16_t)s2_x1) | (((uint32_t)(uint16_t)s2_x2) << 16U);
        uint32_t s2_y1y2_2 = ((uint32_t)(uint16_t)s2_y1) | (((uint32_t)(uint16_t)s2_y2) << 16U);
        int32_t acc2_2 = 16384 * y1_2;
        acc2_2 = (int32_t)__SMLAD(s2_x1x2_2, h2_b1b2, (uint32_t)acc2_2);
        acc2_2 = (int32_t)__SMLAD(s2_y1y2_2, h2_a1a2, (uint32_t)acc2_2);
        int32_t y2_2 = acc2_2 >> 14;
        s2_x2 = s2_x1; s2_x1 = y1_2;
        s2_y2 = s2_y1; s2_y1 = y2_2;
        output[n + 2U] = (int16_t)__SSAT(y2_2, 16U);

        /* ===== Mẫu 4 (n + 3) ===== */
        int32_t x0_3 = (int32_t)input[n + 3U] - ADC_BIAS;
        uint32_t s1_x1x2_3 = ((uint32_t)(uint16_t)s1_x1) | (((uint32_t)(uint16_t)s1_x2) << 16U);
        uint32_t s1_y1y2_3 = ((uint32_t)(uint16_t)s1_y1) | (((uint32_t)(uint16_t)s1_y2) << 16U);
        
        int32_t acc1_3 = 236 * x0_3;
        acc1_3 = (int32_t)__SMLAD(s1_x1x2_3, h1_b1b2, (uint32_t)acc1_3);
        acc1_3 = (int32_t)__SMLAD(s1_y1y2_3, h1_a1a2, (uint32_t)acc1_3);
        int32_t y1_3 = acc1_3 >> 14;
        s1_x2 = s1_x1; s1_x1 = x0_3;
        s1_y2 = s1_y1; s1_y1 = y1_3;

        uint32_t s2_x1x2_3 = ((uint32_t)(uint16_t)s2_x1) | (((uint32_t)(uint16_t)s2_x2) << 16U);
        uint32_t s2_y1y2_3 = ((uint32_t)(uint16_t)s2_y1) | (((uint32_t)(uint16_t)s2_y2) << 16U);
        int32_t acc2_3 = 16384 * y1_3;
        acc2_3 = (int32_t)__SMLAD(s2_x1x2_3, h2_b1b2, (uint32_t)acc2_3);
        acc2_3 = (int32_t)__SMLAD(s2_y1y2_3, h2_a1a2, (uint32_t)acc2_3);
        int32_t y2_3 = acc2_3 >> 14;
        s2_x2 = s2_x1; s2_x1 = y1_3;
        s2_y2 = s2_y1; s2_y1 = y2_3;
        output[n + 3U] = (int16_t)__SSAT(y2_3, 16U);
    }

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
        int32_t x = (int32_t)input[n];
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
            mag_output[n] = (int16_t)__USAT(env, 12U);
        }
    }
}

/**
 * @brief Giảm tần số lấy mẫu (Downsampling) từ 96 kHz xuống 6 kHz (Factor = 16) trên tín hiệu số phức I/Q
 * @details 
 *  - Đầu vào: 2048 mẫu số phức I/Q (Complex_q31) sau Demodulation
 *  - Đầu ra: 128 mẫu số phức I/Q (Complex_q31) @ 6 kHz (mỗi mẫu là trung bình 16 mẫu I/Q liên tiếp)
 *  - Đầu ra mag_output: Mảng 128 mẫu biên độ bao 12-bit (với bias ADC_BIAS) phục vụ gửi hiển thị / UART
 * 
 * @param input Con trỏ mảng 2048 mẫu số phức I/Q đầu vào
 * @param output Con trỏ mảng 128 mẫu số phức I/Q đầu ra (có thể NULL nếu không cần)
 * @param mag_output Con trỏ mảng 128 mẫu biên độ 12-bit đầu ra (có thể NULL nếu không cần)
 */
void Receiver_DownSampling(const Complex_q31 *input, Complex_q31 *output, int16_t *mag_output)
{
    if (input == NULL)
    {
        return;
    }

    /* Hệ số làm tròn khi chia nguyên bằng phép dịch bit: (RECEIVER_DOWNSAMPLE_RATIO / 2 = 8) */
    const int32_t round_val = (int32_t)(RECEIVER_DOWNSAMPLE_RATIO >> 1U);

    for (uint32_t i = 0U; i < RECEIVER_DOWNSAMPLED_SAMPLE_COUNT; i++)
    {
        /* Vị trí bắt đầu của block 16 mẫu: i * 16 (tương đương i << 4) */
        uint32_t base = i << RECEIVER_DOWNSAMPLE_SHIFT;
        int32_t sum_i = 0;
        int32_t sum_q = 0;

        /* Tính tổng 16 mẫu liên tiếp của thành phần I và Q */
        for (uint32_t k = 0U; k < RECEIVER_DOWNSAMPLE_RATIO; k++)
        {
            sum_i += input[base + k].real;
            sum_q += input[base + k].imag;
        }

        /* Lấy trung bình cộng (sum / 16) bằng phép dịch bit kèm làm tròn số học */
        int32_t avg_i = (sum_i + round_val) >> RECEIVER_DOWNSAMPLE_SHIFT;
        int32_t avg_q = (sum_q + round_val) >> RECEIVER_DOWNSAMPLE_SHIFT;

        if (output != NULL)
        {
            output[i].real = avg_i;
            output[i].imag = avg_q;
        }

        if (mag_output != NULL)
        {
            float fi = (float)avg_i;
            float fq = (float)avg_q;
            int32_t env = (int32_t)sqrtf(fi * fi + fq * fq);
            mag_output[i] = (int16_t)__USAT(env, 12U);
        }
    }
}

/* ========================================================================= */
/*                     COMPLEX MATCHED FILTER (TIME DOMAIN)                  */
/* ========================================================================= */

#define DOWNSAMPLED_LFM_LENGTH    (TRANSMITTER_LFM_LENGTH / RECEIVER_DOWNSAMPLE_RATIO)       /* 18 mẫu */
#define DOWNSAMPLED_SINGLE_LENGTH (TRANSMITTER_SINGLE_LENGTH / RECEIVER_DOWNSAMPLE_RATIO)    /* 3 mẫu */

/* Mẫu phát tín hiệu LFM phức sau giải điều chế IQ và Downsampling (18 mẫu @ 6 kHz) */
static const Complex_q31 ref_lfm_template[DOWNSAMPLED_LFM_LENGTH] = {
    {      0,       0}, /* [00] */
    {     96,      41}, /* [01] */
    {    902,     116}, /* [02] */
    {   1696,    -532}, /* [03] */
    {    853,   -1648}, /* [04] */
    {   -712,   -1855}, /* [05] */
    {  -1695,   -1067}, /* [06] */
    {  -1986,     -63}, /* [07] */
    {  -1869,     726}, /* [08] */
    {  -1591,    1226}, /* [09] */
    {  -1356,    1477}, /* [10] */
    {  -1275,    1540}, /* [11] */
    {  -1368,    1449}, /* [12] */
    {  -1597,    1175}, /* [13] */
    {  -1856,     655}, /* [14] */
    {  -1947,    -136}, /* [15] */
    {  -1603,   -1079}, /* [16] */
    {   -641,   -1801}  /* [17] */
};

/* Mẫu phát tín hiệu đơn xung phức sau giải điều chế IQ và Downsampling (3 mẫu @ 6 kHz) */
static const Complex_q31 ref_single_template[DOWNSAMPLED_SINGLE_LENGTH] = {
    {     68,     154}, /* [00] */
    {    792,    1817}, /* [01] */
    {    740,    1702}  /* [02] */
};

/**
 * @brief Bộ lọc phối hợp miền thời gian trên tín hiệu số phức I/Q sau Downsampling
 * @details
 *  - Tương quan phức nhân chập (Complex Cross-Correlation):
 *      y[n] = sum_{k=0}^{L-1} x[n - L + 1 + k] * conj(s[k])
 *      y_real = sum (x_r * s_r + x_i * s_i)
 *      y_imag = sum (x_i * s_r - x_r * s_i)
 *  - Chuẩn hóa biên độ năng lượng mẫu để giữ nguyên thang đo biên độ của tín hiệu.
 *  - Xuất ra Complex_q31 (128 mẫu) và int16_t envelope 12-bit (128 mẫu) với bias ADC_BIAS.
 *
 * @param input Con trỏ mảng 128 mẫu số phức I/Q đầu vào (ds_iq_buffers)
 * @param output Con trỏ mảng 128 mẫu số phức I/Q đầu ra
 * @param mag_output Con trỏ mảng 128 mẫu biên độ bao 12-bit đầu ra
 */
void Receiver_MatchedFilter(const Complex_q31 *input, Complex_q31 *output, int16_t *mag_output)
{
    if (input == NULL)
    {
        return;
    }

    const uint16_t *ref_waveform = NULL;
    uint32_t raw_ref_len = Transmitter_GetActiveWaveform(&ref_waveform);
    const Complex_q31 *template_ptr = ref_single_template;
    uint32_t ref_len = DOWNSAMPLED_SINGLE_LENGTH;

    if (raw_ref_len == TRANSMITTER_LFM_LENGTH)
    {
        template_ptr = ref_lfm_template;
        ref_len = DOWNSAMPLED_LFM_LENGTH;
    }

    /* Chuẩn hóa theo biên độ đỉnh của mẫu phát (2000), bảo toàn tự nhiên độ lợi tích lũy nén xung */
    const float norm_scale = 1.0f / 2000.0f;

    for (uint32_t n = 0U; n < RECEIVER_DOWNSAMPLED_SAMPLE_COUNT; n++)
    {
        int64_t acc_r = 0;
        int64_t acc_i = 0;

        for (uint32_t k = 0U; k < ref_len; k++)
        {
            int32_t idx = (int32_t)n - (int32_t)ref_len + 1 + (int32_t)k;
            if (idx >= 0 && idx < (int32_t)RECEIVER_DOWNSAMPLED_SAMPLE_COUNT)
            {
                int64_t xr = (int64_t)input[idx].real;
                int64_t xi = (int64_t)input[idx].imag;
                int64_t sr = (int64_t)template_ptr[k].real;
                int64_t si = (int64_t)template_ptr[k].imag;

                /* (xr + j*xi) * (sr - j*si) = (xr*sr + xi*si) + j*(xi*sr - xr*si) */
                acc_r += (xr * sr + xi * si);
                acc_i += (xi * sr - xr * si);
            }
        }

        float out_r_f = (float)acc_r * norm_scale;
        float out_i_f = (float)acc_i * norm_scale;

        if (output != NULL)
        {
            output[n].real = (int32_t)lroundf(out_r_f);
            output[n].imag = (int32_t)lroundf(out_i_f);
        }

        if (mag_output != NULL)
        {
            int32_t env = (int32_t)lroundf(sqrtf(out_r_f * out_r_f + out_i_f * out_i_f));
            mag_output[n] = (int16_t)(uint16_t)__USAT(env, 16U);
        }
    }
}

/* ========================================================================= */
/*            FREQUENCY DOMAIN COMPLEX MATCHED FILTER (CMSIS-DSP)            */
/* ========================================================================= */
#define CMPLX_FFT_SIZE 256U

static float cmplx_fft_x[CMPLX_FFT_SIZE * 2U] __attribute__((aligned(4)));
static float cmplx_fft_h[CMPLX_FFT_SIZE * 2U] __attribute__((aligned(4)));
static float cmplx_fft_prod[CMPLX_FFT_SIZE * 2U] __attribute__((aligned(4)));

static void FFT_InitCMSIS(void)
{
    /* CFFT instance CMSIS-DSP arm_cfft_sR_f32_len256 là const struct có sẵn */
}

/**
 * @brief Bộ lọc phối hợp miền tần số trên tín hiệu số phức I/Q (CMSIS-DSP 256-point Complex FFT)
 * @details
 *  - Biến đổi CFFT 256 điểm trên tín hiệu phức 128 mẫu @ 6 kHz.
 *  - Nhân chập miền tần số: Y(f) = X(f) * H*(f)
 *  - Biến đổi ngược CIFFT 256 điểm khôi phục tín hiệu phức miền thời gian.
 *
 * @param input Con trỏ mảng 128 mẫu số phức I/Q đầu vào (ds_iq_buffers)
 * @param output Con trỏ mảng 128 mẫu số phức I/Q đầu ra
 * @param mag_output Con trỏ mảng 128 mẫu biên độ bao 12-bit đầu ra
 */
void Receiver_MatchedFilterFFT(const Complex_q31 *input, Complex_q31 *output, int16_t *mag_output)
{
    if (input == NULL)
    {
        return;
    }

    const uint16_t *ref_waveform = NULL;
    uint32_t raw_ref_len = Transmitter_GetActiveWaveform(&ref_waveform);
    const Complex_q31 *template_ptr = ref_single_template;
    uint32_t ref_len = DOWNSAMPLED_SINGLE_LENGTH;

    if (raw_ref_len == TRANSMITTER_LFM_LENGTH)
    {
        template_ptr = ref_lfm_template;
        ref_len = DOWNSAMPLED_LFM_LENGTH;
    }

    /* 1. Tiền tính phổ liên hợp H*(f) khi dạng sóng phát thay đổi */
    if (raw_ref_len != last_cmplx_fft_ref_len)
    {
        memset(cmplx_fft_h, 0, sizeof(cmplx_fft_h));
        for (uint32_t k = 0U; k < ref_len; k++)
        {
            cmplx_fft_h[2U * k]      = (float)template_ptr[k].real;
            cmplx_fft_h[2U * k + 1U] = (float)template_ptr[k].imag;
        }

        /* Forward CFFT cho template phát */
        arm_cfft_f32(&arm_cfft_sR_f32_len256, cmplx_fft_h, 0, 1);

        /* Liên hợp phức trong miền tần số: H*(f) = Re(H) - j * Im(H) */
        for (uint32_t k = 0U; k < CMPLX_FFT_SIZE; k++)
        {
            cmplx_fft_h[2U * k + 1U] = -cmplx_fft_h[2U * k + 1U];
        }

        last_cmplx_fft_ref_len = raw_ref_len;
    }

    /* 2. Nạp tín hiệu thu phức (128 mẫu) và zero-pad đến 256 mẫu */
    for (uint32_t n = 0U; n < RECEIVER_DOWNSAMPLED_SAMPLE_COUNT; n++)
    {
        cmplx_fft_x[2U * n]      = (float)input[n].real;
        cmplx_fft_x[2U * n + 1U] = (float)input[n].imag;
    }
    memset(&cmplx_fft_x[RECEIVER_DOWNSAMPLED_SAMPLE_COUNT * 2U], 0, 
           (CMPLX_FFT_SIZE - RECEIVER_DOWNSAMPLED_SAMPLE_COUNT) * 2U * sizeof(float));

    /* 3. Forward CFFT tín hiệu thu: X(f) */
    arm_cfft_f32(&arm_cfft_sR_f32_len256, cmplx_fft_x, 0, 1);

    /* 4. Nhân tương quan phức miền tần số: Y(f) = X(f) * H*(f) */
    arm_cmplx_mult_cmplx_f32(cmplx_fft_x, cmplx_fft_h, cmplx_fft_prod, CMPLX_FFT_SIZE);

    /* 5. Inverse CFFT khôi phục tín hiệu phức miền thời gian */
    arm_cfft_f32(&arm_cfft_sR_f32_len256, cmplx_fft_prod, 1, 1);

    /* 6. Chuẩn hóa theo biên độ đỉnh của mẫu phát (2000), bảo toàn tự nhiên độ lợi tích lũy nén xung */
    const float norm_scale = 1.0f / 2000.0f;

    for (uint32_t n = 0U; n < RECEIVER_DOWNSAMPLED_SAMPLE_COUNT; n++)
    {
        float out_r_f = cmplx_fft_prod[2U * n] * norm_scale;
        float out_i_f = cmplx_fft_prod[2U * n + 1U] * norm_scale;

        if (output != NULL)
        {
            output[n].real = (int32_t)lroundf(out_r_f);
            output[n].imag = (int32_t)lroundf(out_i_f);
        }

        if (mag_output != NULL)
        {
            int32_t env = (int32_t)lroundf(sqrtf(out_r_f * out_r_f + out_i_f * out_i_f));
            mag_output[n] = (int16_t)(uint16_t)__USAT(env, 16U);
        }
    }
}

static void Receiver_SendFrame(int16_t *const raw_buffers[2],
                               int16_t *const bpf_buffers[2],
                               int16_t *const demod_buffers[2],
                               int16_t *const ds_buffers[2],
                               int16_t *const filtered_buffers[2])
{
    uint32_t rx_select = ComMgr_GetRxSelect();
    ComMgr_StreamMode mode = ComMgr_GetStreamMode();
    int16_t *const *active_buffers = raw_buffers;
    uint32_t sample_count = ADC_FRAME_SAMPLE_COUNT;

    if (mode == COMMGR_STREAM_BPF)
    {
        active_buffers = bpf_buffers;
    }
    else if (mode == COMMGR_STREAM_DEMODULATED)
    {
        active_buffers = demod_buffers;
    }
    else if (mode == COMMGR_STREAM_DOWNSAMPLING)
    {
        active_buffers = ds_buffers;
        sample_count = RECEIVER_DOWNSAMPLED_SAMPLE_COUNT;
    }
    else if (mode == COMMGR_STREAM_COMPRESSED)
    {
        active_buffers = filtered_buffers;
        sample_count = RECEIVER_DOWNSAMPLED_SAMPLE_COUNT;
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
        for (uint32_t i = 0U; i < sample_count; i++)
        {
            calc_buf[i] = (int16_t)(((int32_t)b1[i] + (int32_t)b2[i]) / 2);
        }
        send_buf = calc_buf;
    }
    else if (rx_select == 3U)
    {
        // Rx Diff = (Rx1 - Rx2) / 2 (+ ADC_BIAS neu o RAW mode)
        int32_t bias = (mode == COMMGR_STREAM_RAW) ? ADC_BIAS : 0;
        const int16_t *b1 = active_buffers[0];
        const int16_t *b2 = active_buffers[1];
        for (uint32_t i = 0U; i < sample_count; i++)
        {
            calc_buf[i] = (int16_t)(((int32_t)b1[i] - (int32_t)b2[i]) / 2 + bias);
        }
        send_buf = calc_buf;
    }

    if (send_buf != NULL)
    {
        receiver_frame[3] = (uint8_t)('0' + rx_select);
        receiver_frame[4] = (uint8_t)(sample_count & 0xFFU);
        receiver_frame[5] = (uint8_t)((sample_count >> 8U) & 0xFFU);
        uint32_t payload_size = sample_count * sizeof(int16_t);
        memcpy(&receiver_frame[RECEIVER_FRAME_HEADER_SIZE], send_buf, payload_size);
        ComMgr_SendData(receiver_frame, RECEIVER_FRAME_HEADER_SIZE + payload_size);
    }
}

static void Receiver_SendRangeDopplerFrame(void)
{
    rd_receiver_frame[0] = 'F';
    rd_receiver_frame[1] = 'R';
    rd_receiver_frame[2] = 'X';
    rd_receiver_frame[3] = '0'; /* '0' = Rx Sum */
    rd_receiver_frame[4] = (uint8_t)(RECEIVER_RD_MATRIX_SIZE & 0xFFU);
    rd_receiver_frame[5] = (uint8_t)((RECEIVER_RD_MATRIX_SIZE >> 8U) & 0xFFU);
    uint32_t payload_size = RECEIVER_RD_MATRIX_SIZE * sizeof(uint16_t);
    memcpy(&rd_receiver_frame[RECEIVER_FRAME_HEADER_SIZE], rd_sum_mag_matrix, payload_size);
    ComMgr_SendData(rd_receiver_frame, RECEIVER_FRAME_HEADER_SIZE + payload_size);
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
    Complex_q31 *ds_iq_buffers[2] = {ds_iq1_frame_buffer, ds_iq2_frame_buffer};
    int16_t *demod_buffers[2] = {demod1_frame_buffer, demod2_frame_buffer};
    int16_t *ds_buffers[2] = {ds1_frame_buffer, ds2_frame_buffer};
    Complex_q31 *filtered_iq_buffers[2] = {filtered_iq1_frame_buffer, filtered_iq2_frame_buffer};
    int16_t *filtered_buffers[2] = {filtered1_frame_buffer, filtered2_frame_buffer};

#ifdef SHOW_TIMING_LOG
    uint32_t t_start = DWTService_GetCycles();
    read_cycles = 0U;
    bpf_cycles = 0U;
    demod_cycles = 0U;
    ds_cycles = 0U;
    mfilt_cycles = 0U;
    send_cycles = 0U;
#endif

    /* 1. Thu thập dữ liệu ADC, lọc BPF, giải điều chế I/Q và Downsampling trên cả 2 kênh thu */
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

        uint32_t t_ds_start = DWTService_GetCycles();
        Receiver_DownSampling(iq_buffers[chan - 1U], ds_iq_buffers[chan - 1U], ds_buffers[chan - 1U]);
        ds_cycles += (DWTService_GetCycles() - t_ds_start);

        uint32_t t_mfilt_start = DWTService_GetCycles();
#if (USE_FFT_MATCHED_FILTER == 1)
        Receiver_MatchedFilterFFT(ds_iq_buffers[chan - 1U], filtered_iq_buffers[chan - 1U], filtered_buffers[chan - 1U]);
#else
        Receiver_MatchedFilter(ds_iq_buffers[chan - 1U], filtered_iq_buffers[chan - 1U], filtered_buffers[chan - 1U]);
#endif
        mfilt_cycles += (DWTService_GetCycles() - t_mfilt_start);
#else
        ADCService_ReadFrame(chan, adc_buffers[chan - 1U]);
        Receiver_BPF(adc_buffers[chan - 1U], bpf_buffers[chan - 1U]);
        Receiver_IQDemodulator(bpf_buffers[chan - 1U], iq_buffers[chan - 1U], demod_buffers[chan - 1U]);
        Receiver_DownSampling(iq_buffers[chan - 1U], ds_iq_buffers[chan - 1U], ds_buffers[chan - 1U]);
#if (USE_FFT_MATCHED_FILTER == 1)
        Receiver_MatchedFilterFFT(ds_iq_buffers[chan - 1U], filtered_iq_buffers[chan - 1U], filtered_buffers[chan - 1U]);
#else
        Receiver_MatchedFilter(ds_iq_buffers[chan - 1U], filtered_iq_buffers[chan - 1U], filtered_buffers[chan - 1U]);
#endif
#endif
    }

    /* 2. Tính tổng và hiệu phức giữa 2 kênh (sau Matched Filter) lưu vào 8 mảng phức tích lũy */
#ifdef SHOW_TIMING_LOG
    uint32_t t_rd_start = DWTService_GetCycles();
#endif
    for (uint32_t n = 0U; n < RECEIVER_DOWNSAMPLED_SAMPLE_COUNT; n++)
    {
        /* Tổng phức 2 kênh: Sum = Ch1 + Ch2 */
        accumulated_sum_pulses[accumulated_pulse_idx][n].real = filtered_iq1_frame_buffer[n].real + filtered_iq2_frame_buffer[n].real;
        accumulated_sum_pulses[accumulated_pulse_idx][n].imag = filtered_iq1_frame_buffer[n].imag + filtered_iq2_frame_buffer[n].imag;

        /* Hiệu phức 2 kênh: Diff = Ch1 - Ch2 */
        accumulated_diff_pulses[accumulated_pulse_idx][n].real = filtered_iq1_frame_buffer[n].real - filtered_iq2_frame_buffer[n].real;
        accumulated_diff_pulses[accumulated_pulse_idx][n].imag = filtered_iq1_frame_buffer[n].imag - filtered_iq2_frame_buffer[n].imag;
    }

    accumulated_pulse_idx++;
    bool rd_ready = false;
    if (accumulated_pulse_idx >= RECEIVER_ACCUMULATED_PULSE_COUNT)
    {
        accumulation_complete = true;

        /* Thực hiện tính toán ma trận Range-Doppler (8-point CFFT theo slow-time) độc lập với stream mode */
        Receiver_ComputeRangeDoppler();
        rd_ready = true;

        /* Sau khi xử lý xong đợt 8 xung, reset lại để tích lũy đợt 8 xung tiếp theo */
        Receiver_ResetAccumulation();
    }
#ifdef SHOW_TIMING_LOG
    rd_cycles = DWTService_GetCycles() - t_rd_start;
    if (rd_ready)
    {
        last_rd_cycles = rd_cycles;
    }
#endif

    /* 3. Gửi tín hiệu theo cấu hình Rx select (1 hoặc 2) và Stream Mode */
#ifdef SHOW_TIMING_LOG
    uint32_t t_send_start = DWTService_GetCycles();
#endif
    ComMgr_StreamMode current_mode = ComMgr_GetStreamMode();
    if (current_mode == COMMGR_STREAM_RANGE_DOPPLER)
    {
        /* Ở chế độ Range-Doppler, chỉ gửi ma trận 8x128 khi đủ 8 xung */
        if (rd_ready)
        {
            Receiver_SendRangeDopplerFrame();
        }
    }
    else
    {
        Receiver_SendFrame(adc_buffers, bpf_buffers, demod_buffers, ds_buffers, filtered_buffers);
    }
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

    /* 4. Xử lý truyền thông USB */
    ComMgr_Process();
}