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

static uint8_t receiver_frame[RECEIVER_FRAME_SIZE];
static int16_t adc1_frame_buffer[ADC_FRAME_SAMPLE_COUNT];
static int16_t adc2_frame_buffer[ADC_FRAME_SAMPLE_COUNT];
static int16_t filtered1_frame_buffer[ADC_FRAME_SAMPLE_COUNT];
static int16_t filtered2_frame_buffer[ADC_FRAME_SAMPLE_COUNT];

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

void Receiver_Init(void)
{
#ifdef SHOW_TIMING_LOG
    DWTService_Init();
    last_dsp_log_tick = HAL_GetTick();
#endif
    ADCService_Init(1U);
    ADCService_Init(2U);

    receiver_frame[0] = 'F';
    receiver_frame[1] = 'R';
    receiver_frame[2] = 'X';
    receiver_frame[3] = '1';
    receiver_frame[4] = (uint8_t)(ADC_FRAME_SAMPLE_COUNT & 0xFFU);
    receiver_frame[5] = (uint8_t)((ADC_FRAME_SAMPLE_COUNT >> 8U) & 0xFFU);
}

static int16_t h_coeffs[TRANSMITTER_LFM_LENGTH];
static uint32_t last_ref_len = 0U;
static const uint16_t *last_waveform_ptr = NULL;

void Receiver_MatchedFilter(const int16_t *input, int16_t *output)
{
    const uint16_t *ref_waveform = NULL;
    uint32_t ref_len = Transmitter_GetActiveWaveform(&ref_waveform);

    if (input == NULL || output == NULL || ref_waveform == NULL || ref_len == 0U)
    {
        return;
    }

    /* Chỉ tính toán lại mảng h_coeffs khi waveform thay đổi */
    if (ref_len != last_ref_len || ref_waveform != last_waveform_ptr)
    {
        for (uint32_t i = 0U; i < ref_len; i++)
        {
            h_coeffs[i] = (int16_t)((int32_t)ref_waveform[ref_len - 1U - i] - ADC_BIAS);
        }
        last_ref_len = ref_len;
        last_waveform_ptr = ref_waveform;
    }

    const int32_t divisor = (int32_t)(ref_len * 1024U);

    for (uint32_t n = 0U; n < ADC_FRAME_SAMPLE_COUNT; n++)
    {
        int32_t acc = 0;
        uint32_t k_max = (n + 1U < ref_len) ? (n + 1U) : ref_len;
        const int16_t *in_ptr = &input[n];
        const int16_t *h_ptr = h_coeffs;

        uint32_t k = 0U;
        for (; k + 3U < k_max; k += 4U)
        {
            acc += ((int32_t)in_ptr[-((int32_t)k)] - ADC_BIAS) * (int32_t)h_ptr[k];
            acc += ((int32_t)in_ptr[-((int32_t)(k + 1U))] - ADC_BIAS) * (int32_t)h_ptr[k + 1U];
            acc += ((int32_t)in_ptr[-((int32_t)(k + 2U))] - ADC_BIAS) * (int32_t)h_ptr[k + 2U];
            acc += ((int32_t)in_ptr[-((int32_t)(k + 3U))] - ADC_BIAS) * (int32_t)h_ptr[k + 3U];
        }
        for (; k < k_max; k++)
        {
            acc += ((int32_t)in_ptr[-((int32_t)k)] - ADC_BIAS) * (int32_t)h_ptr[k];
        }

        int32_t scaled = acc / divisor;
        int32_t result = scaled + ADC_BIAS;

        if (result < 0)
        {
            result = 0;
        }
        else if (result > 4095)
        {
            result = 4095;
        }

        output[n] = (int16_t)result;
    }
}

static void Receiver_SendFrame(int16_t *const raw_buffers[2], int16_t *const filtered_buffers[2])
{
    uint32_t rx_select = ComMgr_GetRxSelect();
    if (rx_select == 1U || rx_select == 2U)
    {
        ComMgr_StreamMode mode = ComMgr_GetStreamMode();
        const int16_t *send_buf = (mode == COMMGR_STREAM_COMPRESSED)
                                      ? filtered_buffers[rx_select - 1U]
                                      : raw_buffers[rx_select - 1U];

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

    /* 1. Thu thập và xử lý Matched Filter song song trên cả 2 kênh */
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
        Receiver_MatchedFilter(adc_buffers[chan - 1U], filtered_buffers[chan - 1U]);
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