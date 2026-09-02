#include "Transmitter.h"

#include "DACService.h"
#include <math.h>
#include <string.h>

#define TRANSMITTER_SAMPLE_COUNT DAC_SAMPLE_COUNT
#define TRANSMITTER_BIAS 2048U
#define TRANSMITTER_SINGLE_LENGTH 48U
#define TRANSMITTER_SINGLE_FREQ 40000.0f
#define TRANSMITTER_SINGLE_AMPLITUDE 2000.0f

#define TRANSMITTER_FS_HZ 96000U
#define TRANSMITTER_FS ((float)TRANSMITTER_FS_HZ)

#define TRANSMITTER_LFM_F0_HZ 39000U
#define TRANSMITTER_LFM_F1_HZ 41000U
#define TRANSMITTER_LFM_F0 ((float)TRANSMITTER_LFM_F0_HZ)
#define TRANSMITTER_LFM_F1 ((float)TRANSMITTER_LFM_F1_HZ)
#define TRANSMITTER_LFM_AMPLITUDE 2000.0f

#ifdef SIMULATION_MODE
#define TRANSMITTER_SIMULATION_DELAY          1000U
/* Giá trị tối đa DAC 12-bit */
#define TRANSMITTER_DAC_MAX_VALUE             4095.0f

#ifdef SIMULATION_NOISE
/* Biên độ nhiễu (đơn vị LSB DAC). Tăng để tăng mức nhiễu trên tín hiệu phát */
#define TRANSMITTER_SIMULATION_NOISE_AMPLITUDE 400.0f
/* Hằng số LCG (Knuth / Numerical Recipes) */
#define TRANSMITTER_LCG_MULTIPLIER            1664525UL
#define TRANSMITTER_LCG_INCREMENT             1013904223UL
#define TRANSMITTER_LCG_SEED                  0x12345678U
/* 2^31: dùng chuẩn hoá output LCG 31-bit về [0, 1] */
#define TRANSMITTER_LCG_NORM                  2147483648.0f
#endif /* SIMULATION_NOISE */
#endif /* SIMULATION_MODE */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Pre-computed short waveform buffers */
static uint16_t transmitter_single_waveform[TRANSMITTER_SINGLE_LENGTH];
static uint16_t transmitter_lfm_waveform[TRANSMITTER_LFM_LENGTH];

/* DMA buffer */
static volatile uint16_t transmitter_samples[TRANSMITTER_SAMPLE_COUNT]
    __attribute__((aligned(32)));

#if defined(SIMULATION_MODE) && defined(SIMULATION_NOISE)
/* ---------------------------------------------------------------
 * Bộ tạo số ngẫu nhiên giả (LCG – Linear Congruential Generator)
 * Tham số: Knuth / Numerical Recipes
 * --------------------------------------------------------------- */
static uint32_t lcg_state = TRANSMITTER_LCG_SEED;

static float Transmitter_RandUniform(void)
{
    lcg_state = lcg_state * TRANSMITTER_LCG_MULTIPLIER + TRANSMITTER_LCG_INCREMENT;
    /* Chuẩn hoá về [0, 1]: dùng 31 bit cao để tránh bias bit thấp */
    return (float)(lcg_state >> 1) / TRANSMITTER_LCG_NORM;
}

/**
 * Trả về mẫu nhiễu Gaussian xấp xỉ bằng cách cộng N mẫu uniform[0,1]
 * rồi trừ N/2 để căn giữa về 0 (Định lý giới hạn trung tâm – CLT).
 * N=12 → sigma ≈ 1, trung bình = 0.
 */
#define TRANSMITTER_NOISE_CLT_N 12U
static float Transmitter_RandGaussian(void)
{
    float sum = 0.0f;
    for (uint32_t i = 0U; i < TRANSMITTER_NOISE_CLT_N; i++)
    {
        sum += Transmitter_RandUniform();
    }
    /* CLT: E[sum]=6, Var[sum]=1  →  sum - 6 ~ N(0,1) */
    return sum - (float)(TRANSMITTER_NOISE_CLT_N / 2U);
}
#endif /* SIMULATION_MODE && SIMULATION_NOISE */

static void Transmitter_GenerateSingleWaveform(void)
{
    const float two_pi = 2.0f * (float)M_PI;

    for (uint32_t n = 0U; n < TRANSMITTER_SINGLE_LENGTH; n++)
    {
        float t = (float)n / TRANSMITTER_FS;
        float phase = two_pi * TRANSMITTER_SINGLE_FREQ * t;
        float sample = (float)TRANSMITTER_BIAS + TRANSMITTER_SINGLE_AMPLITUDE * sinf(phase);
        transmitter_single_waveform[n] = (uint16_t)lroundf(sample);
    }
}

static void Transmitter_GenerateLfmWaveform(void)
{
    const float duration = (float)TRANSMITTER_LFM_LENGTH / TRANSMITTER_FS;
    const float chirp_rate = (TRANSMITTER_LFM_F1 - TRANSMITTER_LFM_F0) / duration;
    const float two_pi = 2.0f * (float)M_PI;

    for (uint32_t n = 0U; n < TRANSMITTER_LFM_LENGTH; n++)
    {
        float t = (float)n / TRANSMITTER_FS;
        float phase = two_pi * (TRANSMITTER_LFM_F0 * t + 0.5f * chirp_rate * t * t);
        float sample = (float)TRANSMITTER_BIAS + TRANSMITTER_LFM_AMPLITUDE * sinf(phase);
        transmitter_lfm_waveform[n] = (uint16_t)lroundf(sample);
    }
}

static Transmitter_PulseType current_pulse_type = TRANSMITTER_PULSE_SINGLE;

void Transmitter_Init(void)
{
    Transmitter_GenerateSingleWaveform();
    Transmitter_GenerateLfmWaveform();

    /* Nạp mặc định xung đơn vào buffer DMA */
    Transmitter_SetPulseType(TRANSMITTER_PULSE_SINGLE);

    DACService_Init((const uint16_t *)transmitter_samples, TRANSMITTER_SAMPLE_COUNT);
}

void Transmitter_SetPulseType(Transmitter_PulseType pulse_type)
{
    current_pulse_type = pulse_type;
    const uint16_t *src = NULL;
    uint32_t active_length = 0U;

    switch (pulse_type)
    {
        case TRANSMITTER_PULSE_LFM:
            src = transmitter_lfm_waveform;
            active_length = TRANSMITTER_LFM_LENGTH;
            break;
        case TRANSMITTER_PULSE_SINGLE:
        default:
            src = transmitter_single_waveform;
            active_length = TRANSMITTER_SINGLE_LENGTH;
            break;
    }

    if (src != NULL)
    {
#ifdef SIMULATION_MODE
#ifdef SIMULATION_NOISE
        /* Điền toàn bộ buffer: nhiễu Gaussian trên nền BIAS, xung được cộng thêm vào
         * vùng [DELAY, DELAY + active_length). Phản ánh đúng thực tế: nhiễu môi trường
         * luôn hiện diện trên toàn bộ cửa sổ thời gian, không chỉ riêng vùng xung. */
        for (uint32_t n = 0U; n < TRANSMITTER_SAMPLE_COUNT; n++)
        {
            float base;
            if ((n >= TRANSMITTER_SIMULATION_DELAY) &&
                (n < (TRANSMITTER_SIMULATION_DELAY + active_length)))
            {
                /* Vùng xung: tín hiệu + nhiễu */
                base = (float)src[n - TRANSMITTER_SIMULATION_DELAY];
            }
            else
            {
                /* Vùng im lặng: chỉ BIAS + nhiễu */
                base = (float)TRANSMITTER_BIAS;
            }

            float noisy = base + TRANSMITTER_SIMULATION_NOISE_AMPLITUDE * Transmitter_RandGaussian();
            /* Giữ trong dải hợp lệ [0, TRANSMITTER_DAC_MAX_VALUE] */
            if (noisy < 0.0f)                       { noisy = 0.0f; }
            if (noisy > TRANSMITTER_DAC_MAX_VALUE)   { noisy = TRANSMITTER_DAC_MAX_VALUE; }
            transmitter_samples[n] = (uint16_t)lroundf(noisy);
        }
#else
        /* Giả lập không nhiễu: đặt xung tại vị trí DELAY trên nền BIAS tĩnh */
        for (uint32_t n = 0U; n < TRANSMITTER_SAMPLE_COUNT; n++)
        {
            if ((n >= TRANSMITTER_SIMULATION_DELAY) &&
                (n < (TRANSMITTER_SIMULATION_DELAY + active_length)))
            {
                transmitter_samples[n] = src[n - TRANSMITTER_SIMULATION_DELAY];
            }
            else
            {
                transmitter_samples[n] = TRANSMITTER_BIAS;
            }
        }
#endif /* SIMULATION_NOISE */
#else
        /* Chép phần tín hiệu xung ngắn */
        (void)memcpy((void *)transmitter_samples, src, active_length * sizeof(uint16_t));

        /* Đặt phần còn lại về mức trung vị BIAS */
        for (uint32_t n = active_length; n < TRANSMITTER_SAMPLE_COUNT; n++)
        {
            transmitter_samples[n] = TRANSMITTER_BIAS;
        }
#endif
    }
}

uint32_t Transmitter_GetActiveWaveform(const uint16_t **waveform)
{
    if (current_pulse_type == TRANSMITTER_PULSE_LFM)
    {
        if (waveform != NULL)
        {
            *waveform = transmitter_lfm_waveform;
        }
        return TRANSMITTER_LFM_LENGTH;
    }
    else
    {
        if (waveform != NULL)
        {
            *waveform = transmitter_single_waveform;
        }
        return TRANSMITTER_SINGLE_LENGTH;
    }
}

#if defined(SIMULATION_MODE) && defined(SIMULATION_NOISE)
/* Số frame DMA đã xử lý ở lần cuối refill nhiễu */
static uint32_t transmitter_last_dac_count = 0U;

/**
 * Tính lại nhiễu cho toàn bộ buffer simulation.
 * Gọi nội bộ mỗi khi DMA hoàn thành một frame.
 */
static void Transmitter_RefillNoise(void)
{
    const uint16_t *src = (current_pulse_type == TRANSMITTER_PULSE_LFM)
                          ? transmitter_lfm_waveform
                          : transmitter_single_waveform;
    const uint32_t active_length = (current_pulse_type == TRANSMITTER_PULSE_LFM)
                                   ? TRANSMITTER_LFM_LENGTH
                                   : TRANSMITTER_SINGLE_LENGTH;

    for (uint32_t n = 0U; n < TRANSMITTER_SAMPLE_COUNT; n++)
    {
        float base;
        if ((n >= TRANSMITTER_SIMULATION_DELAY) &&
            (n < (TRANSMITTER_SIMULATION_DELAY + active_length)))
        {
            base = (float)src[n - TRANSMITTER_SIMULATION_DELAY];
        }
        else
        {
            base = (float)TRANSMITTER_BIAS;
        }

        float noisy = base + TRANSMITTER_SIMULATION_NOISE_AMPLITUDE * Transmitter_RandGaussian();
        /* Giữ trong dải hợp lệ [0, TRANSMITTER_DAC_MAX_VALUE] */
        if (noisy < 0.0f)                       { noisy = 0.0f; }
        if (noisy > TRANSMITTER_DAC_MAX_VALUE)   { noisy = TRANSMITTER_DAC_MAX_VALUE; }
        transmitter_samples[n] = (uint16_t)lroundf(noisy);
    }
}
#endif /* SIMULATION_MODE && SIMULATION_NOISE */

/**
 * Gọi từ main loop.
 * Trong SIMULATION_MODE kèm SIMULATION_NOISE: cập nhật nhiễu mỗi khi DMA hoàn thành một frame,
 * giúp nhiễu thay đổi theo thời gian thực thay vì bị đóng băng.
 * Ngoài SIMULATION_NOISE: hàm rỗng, không tốn tài nguyên.
 */
void Transmitter_Process(void)
{
#if defined(SIMULATION_MODE) && defined(SIMULATION_NOISE)
    uint32_t current_count = DACService_GetCompletedCount();
    if (current_count != transmitter_last_dac_count)
    {
        transmitter_last_dac_count = current_count;
        Transmitter_RefillNoise();
    }
#endif
}