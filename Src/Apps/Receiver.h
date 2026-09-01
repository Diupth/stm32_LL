#ifndef RECEIVER_H
#define RECEIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "ADCService.h"

/**
 * @brief Cấu trúc số phức 32-bit (Q31 / int32 fixed point)
 */
typedef struct
{
    int32_t real;  /**< Thành phần thực / In-phase (I) */
    int32_t imag;  /**< Thành phần ảo / Quadrature (Q) */
} Complex_q31;

#define RECEIVER_DOWNSAMPLE_RATIO         16U  /* Hệ số giảm tần số lấy mẫu: 96 kHz / 6 kHz = 16 */
#define RECEIVER_DOWNSAMPLE_SHIFT         4U   /* Số bit dịch tương đương chia cho 16: (1 << 4 = 16) */
#define RECEIVER_DOWNSAMPLED_SAMPLE_COUNT (ADC_FRAME_SAMPLE_COUNT / RECEIVER_DOWNSAMPLE_RATIO)  /* 128 mẫu @ 6 kHz */

#define RECEIVER_ACCUMULATED_PULSE_COUNT  8U   /* Số lượng xung tích lũy cho xử lý mảng (Slow-time bins) */
#define RECEIVER_DOPPLER_BIN_COUNT        8U   /* 8-point CFFT cho trục Doppler (không zero-padding) */
#define RECEIVER_RD_MATRIX_SIZE           (RECEIVER_DOPPLER_BIN_COUNT * RECEIVER_DOWNSAMPLED_SAMPLE_COUNT) /* 8 x 128 = 1024 mẫu */

void Receiver_Init(void);
void Receiver_Process(void);
void Receiver_ResetAccumulation(void);
bool Receiver_IsAccumulationComplete(void);
uint32_t Receiver_GetAccumulatedPulseCount(void);
const Complex_q31 (*Receiver_GetSumPulses(void))[RECEIVER_DOWNSAMPLED_SAMPLE_COUNT];
const Complex_q31 (*Receiver_GetDiffPulses(void))[RECEIVER_DOWNSAMPLED_SAMPLE_COUNT];
const uint16_t *Receiver_GetRangeDopplerMagSum(void);

void Receiver_ComputeRangeDoppler(void);

void Receiver_BPF(const int16_t *input, int16_t *output);
void Receiver_LPF(const int32_t *input, int32_t *output, uint32_t state_id);
void Receiver_IQDemodulator(const int16_t *input, Complex_q31 *iq_output, int16_t *mag_output);
void Receiver_DownSampling(const Complex_q31 *input, Complex_q31 *output, int16_t *mag_output);
void Receiver_MatchedFilter(const Complex_q31 *input, Complex_q31 *output, int16_t *mag_output);
void Receiver_MatchedFilterFFT(const Complex_q31 *input, Complex_q31 *output, int16_t *mag_output);

#endif /* RECEIVER_H */