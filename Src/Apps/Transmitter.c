#include "Transmitter.h"

#include "DACService.h"
#include <math.h>

#define TRANSMITTER_SAMPLE_COUNT 2048U
#define TRANSMITTER_SAMPLE_RATE 160000.0f
#define TRANSMITTER_FREQUENCY 300.0f
#define TRANSMITTER_PI 3.14159265358979323846f

static uint16_t transmitter_samples[TRANSMITTER_SAMPLE_COUNT]
    __attribute__((aligned(32)));

void Transmitter_Init(void)
{
    for (uint32_t i = 0U; i < TRANSMITTER_SAMPLE_COUNT; i++)
    {
        float value = 2048.0f + 2000.0f * sinf(
            2.0f * TRANSMITTER_PI * TRANSMITTER_FREQUENCY * (float)i / TRANSMITTER_SAMPLE_RATE);
        transmitter_samples[i] = (uint16_t)value;
    }

    DACService_Init(transmitter_samples, TRANSMITTER_SAMPLE_COUNT);
}