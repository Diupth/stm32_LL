#ifndef TRANSMITTER_H
#define TRANSMITTER_H

typedef enum {
  TRANSMITTER_PULSE_SINGLE = 0U,
  TRANSMITTER_PULSE_BARKER13
} Transmitter_PulseType;

void Transmitter_Init(void);
void Transmitter_SetPulseType(Transmitter_PulseType pulse_type);
#ifdef SIMULATION_MODE
void Transmitter_UpdateNoise(void);
#endif

#endif /* TRANSMITTER_H */