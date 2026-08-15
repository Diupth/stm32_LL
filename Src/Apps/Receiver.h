#ifndef RECEIVER_H
#define RECEIVER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t re;
    int16_t im;
} Complex_t;

void Receiver_Init(void);
void Receiver_Process(void);
Complex_t* Receiver_GetComplexBuffer(uint32_t channel);

#endif /* RECEIVER_H */