#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <stdint.h>

#define TRANSMITTER_SINGLE_LENGTH 48U
#define TRANSMITTER_FS_HZ 96000U
#define TRANSMITTER_LFM_BANDWIDTH_HZ 2000U
#define TRANSMITTER_COMPRESSION_RATIO 6U
#define TRANSMITTER_LFM_LENGTH ((TRANSMITTER_COMPRESSION_RATIO * TRANSMITTER_FS_HZ) / TRANSMITTER_LFM_BANDWIDTH_HZ)

typedef enum
{
	TRANSMITTER_PULSE_SINGLE = 0U,
	TRANSMITTER_PULSE_LFM
} Transmitter_PulseType;

void Transmitter_Init(void);
void Transmitter_SetPulseType(Transmitter_PulseType pulse_type);
uint32_t Transmitter_GetActiveWaveform(const uint16_t **waveform);
void Transmitter_Process(void);

#endif /* TRANSMITTER_H */