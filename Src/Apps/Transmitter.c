#include "Transmitter.h"

#include "DACService.h"
#include <string.h>

#define TRANSMITTER_SAMPLE_COUNT 2048U
#define TRANSMITTER_BIAS 2048U

static const uint16_t transmitter_waveform[TRANSMITTER_SAMPLE_COUNT] = {
    2048U, 4048U, 2048U, 48U,
    2048U, 4048U, 2048U, 48U,
    [8U ... TRANSMITTER_SAMPLE_COUNT - 1U] = TRANSMITTER_BIAS
};

static uint16_t transmitter_samples[TRANSMITTER_SAMPLE_COUNT]
    __attribute__((aligned(32)));

void Transmitter_Init(void)
{
    (void)memcpy(transmitter_samples, transmitter_waveform, sizeof(transmitter_samples));

    DACService_Init(transmitter_samples, TRANSMITTER_SAMPLE_COUNT);
}