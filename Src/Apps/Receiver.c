#include "Receiver.h"
#include <string.h>
#include <math.h>

#include "ADCService.h"
#include "ComMgr.h"
#include "Transmitter.h"

#define RECEIVER_FRAME_HEADER_SIZE 16U
#define RECEIVER_FRAME_PAYLOAD_SIZE (ADC_FRAME_SAMPLE_COUNT * sizeof(int16_t))
#define RECEIVER_FRAME_SIZE (RECEIVER_FRAME_HEADER_SIZE + RECEIVER_FRAME_PAYLOAD_SIZE)

#define ADC_BIAS 2048

static uint8_t receiver_frame[RECEIVER_FRAME_SIZE];
static int16_t adc1_frame_buffer[ADC_FRAME_SAMPLE_COUNT];
static int16_t adc2_frame_buffer[ADC_FRAME_SAMPLE_COUNT];
static int16_t filtered1_frame_buffer[ADC_FRAME_SAMPLE_COUNT];
static int16_t filtered2_frame_buffer[ADC_FRAME_SAMPLE_COUNT];

void Receiver_Init(void)
{
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

void Receiver_Process(void)
{
    uint32_t rx_select = ComMgr_GetRxSelect();
    ComMgr_StreamMode mode = ComMgr_GetStreamMode();
    int16_t *adc_buffers[2] = {adc1_frame_buffer, adc2_frame_buffer};
    int16_t *filtered_buffers[2] = {filtered1_frame_buffer, filtered2_frame_buffer};

    for (uint32_t chan = 1U; chan <= 2U; chan++)
    {
        if (ADCService_ReadFrame(chan, adc_buffers[chan - 1U]))
        {
            /* Luôn thực hiện matched filter trên cả 2 kênh bất kể cấu hình */
            Receiver_MatchedFilter(adc_buffers[chan - 1U], filtered_buffers[chan - 1U]);

            /* Cấu hình chỉ phục vụ mục đích truyền thông lên SonarViewer */
            if (rx_select == chan)
            {
                int16_t *send_buf = (mode == COMMGR_STREAM_COMPRESSED) ? filtered_buffers[chan - 1U] : adc_buffers[chan - 1U];

                receiver_frame[3] = (uint8_t)('0' + chan);
                memcpy(&receiver_frame[RECEIVER_FRAME_HEADER_SIZE], send_buf, RECEIVER_FRAME_PAYLOAD_SIZE);
                ComMgr_SendData(receiver_frame, sizeof(receiver_frame));
            }

            /* Nhường quyền xử lý ngay cho USB CDC truyền dữ liệu và xử lý tud_task */
            ComMgr_Process();
        }
    }
}