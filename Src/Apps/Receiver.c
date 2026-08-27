#include "Receiver.h"
#include <string.h>
#include <math.h>

#include "ADCService.h"
#include "ComMgr.h"
#include "Transmitter.h"
#include "SyncSignalApp.h"
#ifdef SHOW_TIMING_LOG
#include "DWTService.h"
#endif

#define RECEIVER_FRAME_HEADER_SIZE 16U
#define RECEIVER_FRAME_PAYLOAD_SIZE (ADC_FRAME_SAMPLE_COUNT * sizeof(int16_t))
#define RECEIVER_FRAME_SIZE (RECEIVER_FRAME_HEADER_SIZE + RECEIVER_FRAME_PAYLOAD_SIZE)

#define ADC_BIAS 2048
#define USE_FFT_MATCHED_FILTER 1

static uint8_t receiver_frame[RECEIVER_FRAME_SIZE] __attribute__((aligned(4)));
static int16_t adc1_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t adc2_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t filtered1_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));
static int16_t filtered2_frame_buffer[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));

#ifdef SHOW_TIMING_LOG
static uint32_t dsp_log_sequence = 0U;
static uint32_t last_dsp_log_tick = 0U;
static uint32_t read_cycles = 0U;
static uint32_t mfilt_cycles = 0U;
static uint32_t send_cycles = 0U;
static uint32_t total_cycles = 0U;

static void Receiver_SendTimingLog(void)
{
    uint8_t dsp_frame[40] = {'D', 'S', 'P', '1'};
    uint32_t bpf_us = 0U;
    uint32_t demod_us = 0U;
    uint32_t accum_us = 0U;
    uint32_t detect_us = 0U;

    uint32_t values[9] = {
        dsp_log_sequence++,
        DWTService_CyclesToUs(total_cycles),
        DWTService_CyclesToUs(read_cycles),
        bpf_us,
        demod_us,
        DWTService_CyclesToUs(mfilt_cycles),
        DWTService_CyclesToUs(send_cycles),
        accum_us,
        detect_us
    };

    for (uint32_t i = 0U; i < 9U; i++)
    {
        for (uint32_t byte = 0U; byte < 4U; byte++)
        {
            dsp_frame[4U + i * 4U + byte] = (uint8_t)(values[i] >> (byte * 8U));
        }
    }

    ComMgr_SendData(dsp_frame, sizeof(dsp_frame));
}
#endif

static void FFT_InitCMSIS(void);

void Receiver_Init(void)
{
#ifdef SHOW_TIMING_LOG
    DWTService_Init();
    last_dsp_log_tick = HAL_GetTick();
#endif
    ADCService_Init(1U);
    ADCService_Init(2U);

    FFT_InitCMSIS();

    receiver_frame[0] = 'F';
    receiver_frame[1] = 'R';
    receiver_frame[2] = 'X';
    receiver_frame[3] = '1';
    receiver_frame[4] = (uint8_t)(ADC_FRAME_SAMPLE_COUNT & 0xFFU);
    receiver_frame[5] = (uint8_t)((ADC_FRAME_SAMPLE_COUNT >> 8U) & 0xFFU);
}

static int16_t h_coeffs[TRANSMITTER_LFM_LENGTH] __attribute__((aligned(4)));
static uint32_t last_ref_len = 0U;
static const uint16_t *last_waveform_ptr = NULL;
static int16_t in_biased[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));

void Receiver_MatchedFilter(const int16_t *input, int16_t *output)
{
    const uint16_t *ref_waveform = NULL;
    uint32_t ref_len = Transmitter_GetActiveWaveform(&ref_waveform);

    if (input == NULL || output == NULL || ref_waveform == NULL || ref_len == 0U)
    {
        return;
    }

    /* Template h_coeffs[k] = ref_waveform[k] - ADC_BIAS để có thể tính tích chập x[n - ref_len + 1 + k] * h[k] */
    if (ref_len != last_ref_len || ref_waveform != last_waveform_ptr)
    {
        for (uint32_t i = 0U; i < ref_len; i++)
        {
            h_coeffs[i] = (int16_t)((int32_t)ref_waveform[i] - ADC_BIAS);
        }
        last_ref_len = ref_len;
        last_waveform_ptr = ref_waveform;
    }

    /* Bước 1: Trừ ADC_BIAS trước một lần duy nhất cho toàn bộ frame bằng SIMD __SSUB16 */
    const uint32_t *in_u32 = (const uint32_t *)(const void *)input;
    uint32_t *biased_u32 = (uint32_t *)(void *)in_biased;
    const uint32_t bias_pair = ((uint32_t)(uint16_t)ADC_BIAS) | (((uint32_t)(uint16_t)ADC_BIAS) << 16U);

    for (uint32_t i = 0U; i < ADC_FRAME_SAMPLE_COUNT / 2U; i++)
    {
        biased_u32[i] = __SSUB16(in_u32[i], bias_pair);
    }

    const int32_t divisor = (int32_t)(ref_len * 1024U);
    const uint32_t *h_ptr32 = (const uint32_t *)(const void *)h_coeffs;

    /* Bước 2: Đoạn đầu (n < ref_len - 1), tín hiệu chưa đi hết bộ lọc (tích chập từng phần) */
    for (uint32_t n = 0U; n < ref_len - 1U && n < ADC_FRAME_SAMPLE_COUNT; n++)
    {
        int32_t acc = 0;
        uint32_t k_len = n + 1U;
        const int16_t *x_ptr = &in_biased[0];
        const int16_t *h_sub = &h_coeffs[ref_len - k_len];

        uint32_t k = 0U;
        for (; k + 3U < k_len; k += 4U)
        {
            acc += (int32_t)x_ptr[k] * (int32_t)h_sub[k];
            acc += (int32_t)x_ptr[k + 1U] * (int32_t)h_sub[k + 1U];
            acc += (int32_t)x_ptr[k + 2U] * (int32_t)h_sub[k + 2U];
            acc += (int32_t)x_ptr[k + 3U] * (int32_t)h_sub[k + 3U];
        }
        for (; k < k_len; k++)
        {
            acc += (int32_t)x_ptr[k] * (int32_t)h_sub[k];
        }

        int32_t scaled = acc / divisor;
        int32_t result = scaled + ADC_BIAS;
        output[n] = (int16_t)__USAT(result, 12U);
    }

    /* Bước 3: Đoạn chính (n >= ref_len - 1), bộ lọc khớp hoàn toàn, sử dụng SIMD kép SMLAD unroll 16 mẫu */
    const uint32_t pairs = ref_len >> 1U;
    for (uint32_t n = ref_len - 1U; n < ADC_FRAME_SAMPLE_COUNT; n++)
    {
        int32_t acc = 0;
        const int16_t *x_sub = &in_biased[n - (ref_len - 1U)];
        const uint32_t *x_ptr32 = (const uint32_t *)(const void *)x_sub;

        /* Unroll 16 mẫu (8 lệnh SMLAD) mỗi vòng lặp */
        uint32_t j = 0U;
        for (; j + 7U < pairs; j += 8U)
        {
            acc = (int32_t)__SMLAD(x_ptr32[j],      h_ptr32[j],      (uint32_t)acc);
            acc = (int32_t)__SMLAD(x_ptr32[j + 1U], h_ptr32[j + 1U], (uint32_t)acc);
            acc = (int32_t)__SMLAD(x_ptr32[j + 2U], h_ptr32[j + 2U], (uint32_t)acc);
            acc = (int32_t)__SMLAD(x_ptr32[j + 3U], h_ptr32[j + 3U], (uint32_t)acc);
            acc = (int32_t)__SMLAD(x_ptr32[j + 4U], h_ptr32[j + 4U], (uint32_t)acc);
            acc = (int32_t)__SMLAD(x_ptr32[j + 5U], h_ptr32[j + 5U], (uint32_t)acc);
            acc = (int32_t)__SMLAD(x_ptr32[j + 6U], h_ptr32[j + 6U], (uint32_t)acc);
            acc = (int32_t)__SMLAD(x_ptr32[j + 7U], h_ptr32[j + 7U], (uint32_t)acc);
        }
        for (; j < pairs; j++)
        {
            acc = (int32_t)__SMLAD(x_ptr32[j], h_ptr32[j], (uint32_t)acc);
        }
        if (ref_len & 1U)
        {
            acc += (int32_t)x_sub[ref_len - 1U] * (int32_t)h_coeffs[ref_len - 1U];
        }

        int32_t scaled = acc / divisor;
        int32_t result = scaled + ADC_BIAS;
        output[n] = (int16_t)__USAT(result, 12U);
    }
}

/* ========================================================================= */
/*                   FREQUENCY DOMAIN MATCHED FILTER (CMSIS-DSP)            */
/* ========================================================================= */
#include "arm_math.h"
#include "arm_const_structs.h"

#define FFT_SIZE 4096U

static arm_rfft_fast_instance_f32 rfft_instance;
static bool rfft_initialized = false;

static float fft_in[FFT_SIZE] __attribute__((aligned(4)));
static float fft_out[FFT_SIZE] __attribute__((aligned(4)));
static float fft_h_buf[FFT_SIZE] __attribute__((aligned(4)));
static float fft_x_buf[FFT_SIZE] __attribute__((aligned(4)));
static float fft_prod[FFT_SIZE] __attribute__((aligned(4)));

static uint32_t last_fft_ref_len = 0U;
static const uint16_t *last_fft_waveform_ptr = NULL;

static void FFT_InitCMSIS(void)
{
    if (!rfft_initialized)
    {
        arm_rfft_fast_init_f32(&rfft_instance, FFT_SIZE);
        rfft_initialized = true;
    }
}

void Receiver_MatchedFilterFFT(const int16_t *input, int16_t *output)
{
    const uint16_t *ref_waveform = NULL;
    uint32_t ref_len = Transmitter_GetActiveWaveform(&ref_waveform);

    if (input == NULL || output == NULL || ref_waveform == NULL || ref_len == 0U)
    {
        return;
    }

    if (!rfft_initialized)
    {
        FFT_InitCMSIS();
    }

    /* 1. Tiền tính toán phổ liên hợp H*(f) của tín hiệu mẫu khi mẫu thay đổi */
    if (ref_len != last_fft_ref_len || ref_waveform != last_fft_waveform_ptr)
    {
        memset(fft_in, 0, sizeof(fft_in));
        for (uint32_t i = 0U; i < ref_len; i++)
        {
            fft_in[i] = (float)((int32_t)ref_waveform[i] - ADC_BIAS);
        }

        /* Forward RFFT của mẫu phát */
        arm_rfft_fast_f32(&rfft_instance, fft_in, fft_h_buf, 0);

        /* Lấy liên hợp phức: H*(f) = Re(H) - j * Im(H) */
        for (uint32_t i = 3U; i < FFT_SIZE; i += 2U)
        {
            fft_h_buf[i] = -fft_h_buf[i];
        }

        last_fft_ref_len = ref_len;
        last_fft_waveform_ptr = ref_waveform;
    }

    /* 2. Nạp tín hiệu vào, trừ bias ADC và zero-pad */
    for (uint32_t i = 0U; i < ADC_FRAME_SAMPLE_COUNT; i++)
    {
        fft_in[i] = (float)((int32_t)input[i] - ADC_BIAS);
    }
    memset(&fft_in[ADC_FRAME_SAMPLE_COUNT], 0, (FFT_SIZE - ADC_FRAME_SAMPLE_COUNT) * sizeof(float));

    /* 3. Forward RFFT của tín hiệu thu X(f) */
    arm_rfft_fast_f32(&rfft_instance, fft_in, fft_x_buf, 0);

    /* 4. Nhân chập miền tần số (Tương quan phức): Y(f) = X(f) * H*(f) */
    /* Bin DC và Nyquist là số thực thuần */
    fft_prod[0] = fft_x_buf[0] * fft_h_buf[0];
    fft_prod[1] = fft_x_buf[1] * fft_h_buf[1];

    /* Các bin phức từ index 2 đến FFT_SIZE-1 */
    arm_cmplx_mult_cmplx_f32(&fft_x_buf[2], &fft_h_buf[2], &fft_prod[2], (FFT_SIZE - 2U) / 2U);

    /* 5. Inverse RFFT: khôi phục tín hiệu miền thời gian ra buffer riêng fft_out */
    arm_rfft_fast_f32(&rfft_instance, fft_prod, fft_out, 1);

    /* 6. Chuẩn hóa scale biên độ và khôi phục bias 12-bit ADC */
    const float scale = 1.0f / (float)(ref_len * 1024U);
    for (uint32_t n = 0U; n < ADC_FRAME_SAMPLE_COUNT; n++)
    {
        float val = fft_out[n] * scale;
        int32_t result = (int32_t)val + ADC_BIAS;
        output[n] = (int16_t)__USAT(result, 12U);
    }
}

static void Receiver_SendFrame(int16_t *const raw_buffers[2], int16_t *const filtered_buffers[2])
{
    uint32_t rx_select = ComMgr_GetRxSelect();
    ComMgr_StreamMode mode = ComMgr_GetStreamMode();
    const int16_t *send_buf = NULL;
    static int16_t calc_buf[ADC_FRAME_SAMPLE_COUNT] __attribute__((aligned(4)));

    if (rx_select == 1U || rx_select == 2U)
    {
        send_buf = (mode == COMMGR_STREAM_COMPRESSED)
                       ? filtered_buffers[rx_select - 1U]
                       : raw_buffers[rx_select - 1U];
    }
    else if (rx_select == 0U)
    {
        // Rx Sum = (Rx1 + Rx2) / 2
        const int16_t *b1 = (mode == COMMGR_STREAM_COMPRESSED) ? filtered_buffers[0] : raw_buffers[0];
        const int16_t *b2 = (mode == COMMGR_STREAM_COMPRESSED) ? filtered_buffers[1] : raw_buffers[1];
        for (uint32_t i = 0U; i < ADC_FRAME_SAMPLE_COUNT; i++)
        {
            calc_buf[i] = (int16_t)(((int32_t)b1[i] + (int32_t)b2[i]) / 2);
        }
        send_buf = calc_buf;
    }
    else if (rx_select == 3U)
    {
        // Rx Diff = (Rx1 - Rx2) / 2 + ADC_BIAS
        const int16_t *b1 = (mode == COMMGR_STREAM_COMPRESSED) ? filtered_buffers[0] : raw_buffers[0];
        const int16_t *b2 = (mode == COMMGR_STREAM_COMPRESSED) ? filtered_buffers[1] : raw_buffers[1];
        for (uint32_t i = 0U; i < ADC_FRAME_SAMPLE_COUNT; i++)
        {
            calc_buf[i] = (int16_t)(((int32_t)b1[i] - (int32_t)b2[i]) / 2 + ADC_BIAS);
        }
        send_buf = calc_buf;
    }

    if (send_buf != NULL)
    {
        receiver_frame[3] = (uint8_t)('0' + rx_select);
        memcpy(&receiver_frame[RECEIVER_FRAME_HEADER_SIZE], send_buf, RECEIVER_FRAME_PAYLOAD_SIZE);
        ComMgr_SendData(receiver_frame, sizeof(receiver_frame));
    }
}

void Receiver_Process(void)
{
    /* Kiểm tra và đồng bộ đủ 2 kênh ADC từ SyncSignalApp trước khi bắt đầu xử lý DSP */
    if (!SyncSignalApp_WaitForFrames())
    {
        ComMgr_Process();
        return;
    }

    int16_t *adc_buffers[2] = {adc1_frame_buffer, adc2_frame_buffer};
    int16_t *filtered_buffers[2] = {filtered1_frame_buffer, filtered2_frame_buffer};

#ifdef SHOW_TIMING_LOG
    uint32_t t_start = DWTService_GetCycles();
    read_cycles = 0U;
    mfilt_cycles = 0U;
    send_cycles = 0U;
#endif

    /* 1. Thu thập dữ liệu ADC và xử lý Matched Filter song song cho cả 2 kênh */
    for (uint32_t chan = 1U; chan <= 2U; chan++)
    {
#ifdef SHOW_TIMING_LOG
        uint32_t t_read_start = DWTService_GetCycles();
#endif
        ADCService_ReadFrame(chan, adc_buffers[chan - 1U]);
#ifdef SHOW_TIMING_LOG
        read_cycles += (DWTService_GetCycles() - t_read_start);
        uint32_t t_mfilt_start = DWTService_GetCycles();
#endif
        /* Bạn có thể chọn Receiver_MatchedFilter (Direct SIMD) hoặc Receiver_MatchedFilterFFT (Frequency domain) */
#if defined(USE_FFT_MATCHED_FILTER)
        Receiver_MatchedFilterFFT(adc_buffers[chan - 1U], filtered_buffers[chan - 1U]);
#else
        Receiver_MatchedFilter(adc_buffers[chan - 1U], filtered_buffers[chan - 1U]);
#endif
#ifdef SHOW_TIMING_LOG
        mfilt_cycles += (DWTService_GetCycles() - t_mfilt_start);
#endif
    }

    /* 2. Gửi tín hiệu theo cấu hình Rx select (1 hoặc 2) và Stream Mode */
#ifdef SHOW_TIMING_LOG
    uint32_t t_send_start = DWTService_GetCycles();
#endif
    Receiver_SendFrame(adc_buffers, filtered_buffers);
#ifdef SHOW_TIMING_LOG
    send_cycles = DWTService_GetCycles() - t_send_start;
    total_cycles = DWTService_GetCycles() - t_start;

    uint32_t now = HAL_GetTick();
    if (now - last_dsp_log_tick >= 1000U)
    {
        last_dsp_log_tick = now;
        Receiver_SendTimingLog();
    }
#endif

    /* 3. Xử lý truyền thông USB */
    ComMgr_Process();
}