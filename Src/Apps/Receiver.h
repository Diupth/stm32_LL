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

typedef struct __attribute__((packed)) {
    float range_m;
    float strength_dbv;
    int16_t angle_deg;
    int16_t reserved;
    float velocity_mps;
} TargetEntry_t;

typedef struct __attribute__((packed)) {
    char header[4];       // "TGT1"
    uint16_t target_count;// Số lượng mục tiêu
    uint16_t reserved;
    TargetEntry_t targets[1];
} TargetFrame_t;

void Receiver_Init(void);
void Receiver_Process(void);
Complex_t* Receiver_GetComplexBuffer(uint32_t channel);

#endif /* RECEIVER_H */