#ifndef RECEIVER_H
#define RECEIVER_H

#include <stdbool.h>
#include <stdint.h>

void Receiver_Init(void);
void Receiver_Process(void);
void Receiver_MatchedFilter(const int16_t *input, int16_t *output);

#endif /* RECEIVER_H */