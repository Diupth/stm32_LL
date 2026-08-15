#ifndef RECEIVER_H
#define RECEIVER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t re;
    int16_t im;
} Complex_t;

typedef enum {
    RX_CHANNEL_SUM = 0,
    RX_CHANNEL_1 = 1,
    RX_CHANNEL_2 = 2,
    RX_CHANNEL_DIFF = 3
} RxChannel_t;

void Receiver_Init(void);
void Receiver_Process(void);
Complex_t* Receiver_GetComplexBuffer(uint32_t channel);

#endif /* RECEIVER_H */