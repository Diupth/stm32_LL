#include "Transmitter.h"

#include "DACService.h"
#include <math.h>
#include <string.h>

#define TRANSMITTER_SAMPLE_COUNT DAC_SAMPLE_COUNT
#define TRANSMITTER_BIAS 2048U
#define TRANSMITTER_SINGLE_LENGTH 80U
#define TRANSMITTER_PULSE_HIGH 4048U
#define TRANSMITTER_PULSE_LOW 48U

#define TRANSMITTER_LFM_LENGTH 640U
#define TRANSMITTER_FS 160000.0f
#define TRANSMITTER_LFM_F0 39000.0f
#define TRANSMITTER_LFM_F1 41000.0f
#define TRANSMITTER_LFM_AMPLITUDE 2000.0f

#ifdef SIMULATION_MODE
#define TRANSMITTER_SIMULATION_DELAY 1000U
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Pre-computed short waveform buffers */
static uint16_t transmitter_single_waveform[TRANSMITTER_SINGLE_LENGTH];
static uint16_t transmitter_lfm_waveform[TRANSMITTER_LFM_LENGTH];

/* DMA buffer */
static volatile uint16_t transmitter_samples[TRANSMITTER_SAMPLE_COUNT]
    __attribute__((aligned(32)));

static void Transmitter_GenerateSingleWaveform(void)
{
    const uint16_t pattern[4] = {
        TRANSMITTER_BIAS,
        TRANSMITTER_PULSE_HIGH,
        TRANSMITTER_BIAS,
        TRANSMITTER_PULSE_LOW
    };

    for (uint32_t n = 0U; n < TRANSMITTER_SINGLE_LENGTH; n++)
    {
        transmitter_single_waveform[n] = pattern[n % 4U];
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
        /* Đặt phần đầu trước delay về mức BIAS */
        for (uint32_t n = 0U; n < TRANSMITTER_SIMULATION_DELAY; n++)
        {
            transmitter_samples[n] = TRANSMITTER_BIAS;
        }

        /* Chép phần tín hiệu xung sau độ trễ */
        (void)memcpy((void *)&transmitter_samples[TRANSMITTER_SIMULATION_DELAY], src, active_length * sizeof(uint16_t));

        /* Đặt phần còn lại về mức trung vị BIAS */
        for (uint32_t n = TRANSMITTER_SIMULATION_DELAY + active_length; n < TRANSMITTER_SAMPLE_COUNT; n++)
        {
            transmitter_samples[n] = TRANSMITTER_BIAS;
        }
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