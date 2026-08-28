#ifndef RECEIVER_H
#define RECEIVER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Cấu trúc số phức 32-bit (Q31 / int32 fixed point)
 */
typedef struct
{
    int32_t real;  /**< Thành phần thực / In-phase (I) */
    int32_t imag;  /**< Thành phần ảo / Quadrature (Q) */
} Complex_q31;

void Receiver_Init(void);
void Receiver_Process(void);
void Receiver_BPF(const int16_t *input, int16_t *output);
void Receiver_LPF(const int32_t *input, int32_t *output, uint32_t state_id);
void Receiver_IQDemodulator(const int16_t *input, Complex_q31 *iq_output, int16_t *mag_output);
void Receiver_MatchedFilter(const int16_t *input, int16_t *output);
void Receiver_MatchedFilterFFT(const int16_t *input, int16_t *output);

#endif /* RECEIVER_H */