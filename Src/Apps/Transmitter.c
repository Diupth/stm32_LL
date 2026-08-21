#include "Transmitter.h"

#include "DACService.h"
#include <math.h>
#include <string.h>

#define TRANSMITTER_SAMPLE_COUNT 2048U
#define TRANSMITTER_BIAS 2048U
#define TRANSMITTER_SINGLE_LENGTH 8U
#define TRANSMITTER_PULSE_HIGH 4048U
#define TRANSMITTER_PULSE_LOW 48U

#define TRANSMITTER_LFM_LENGTH 320U
#define TRANSMITTER_FS 160000.0f
#define TRANSMITTER_LFM_F0 39000.0f
#define TRANSMITTER_LFM_F1 41000.0f
#define TRANSMITTER_LFM_AMPLITUDE 2000.0f

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const uint16_t transmitter_single_waveform[TRANSMITTER_SAMPLE_COUNT] = {
    TRANSMITTER_BIAS, TRANSMITTER_PULSE_HIGH, TRANSMITTER_BIAS, TRANSMITTER_PULSE_LOW,
    TRANSMITTER_BIAS, TRANSMITTER_PULSE_HIGH, TRANSMITTER_BIAS, TRANSMITTER_PULSE_LOW,
    [TRANSMITTER_SINGLE_LENGTH ... TRANSMITTER_SAMPLE_COUNT - 1U] = TRANSMITTER_BIAS
};

/* Global buffer computed once in Transmitter_Init */
static uint16_t transmitter_lfm_waveform[TRANSMITTER_SAMPLE_COUNT];

static volatile uint16_t transmitter_samples[TRANSMITTER_SAMPLE_COUNT]
    __attribute__((aligned(32)));

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

    for (uint32_t n = TRANSMITTER_LFM_LENGTH; n < TRANSMITTER_SAMPLE_COUNT; n++)
    {
        transmitter_lfm_waveform[n] = TRANSMITTER_BIAS;
    }
}

void Transmitter_Init(void)
{
    Transmitter_GenerateLfmWaveform();

    (void)memcpy((void *)transmitter_samples, transmitter_single_waveform, sizeof(transmitter_samples));

    DACService_Init((const uint16_t *)transmitter_samples, TRANSMITTER_SAMPLE_COUNT);
}

void Transmitter_SetPulseType(Transmitter_PulseType pulse_type)
{
    const uint16_t *src = NULL;

    switch (pulse_type)
    {
        case TRANSMITTER_PULSE_LFM:
            src = transmitter_lfm_waveform;
            break;
        case TRANSMITTER_PULSE_SINGLE:
        default:
            src = transmitter_single_waveform;
            break;
    }

    if (src != NULL)
    {
        (void)memcpy((void *)transmitter_samples, src, sizeof(transmitter_samples));
    }
}