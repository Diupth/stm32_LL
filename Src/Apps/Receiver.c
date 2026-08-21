#include "Receiver.h"
#include <string.h>

#include "ADCService.h"
#include "ComMgr.h"

#define RECEIVER_FRAME_HEADER_SIZE 16U
#define RECEIVER_FRAME_PAYLOAD_SIZE (ADC_FRAME_SAMPLE_COUNT * sizeof(int16_t))
#define RECEIVER_FRAME_SIZE (RECEIVER_FRAME_HEADER_SIZE + RECEIVER_FRAME_PAYLOAD_SIZE)

static uint8_t receiver_frame[RECEIVER_FRAME_SIZE];
static int16_t adc1_frame_buffer[ADC_FRAME_SAMPLE_COUNT];
static int16_t adc2_frame_buffer[ADC_FRAME_SAMPLE_COUNT];

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

void Receiver_Process(void)
{
    bool has_frame1 = ADCService_ReadFrame(1U, adc1_frame_buffer);
    bool has_frame2 = ADCService_ReadFrame(2U, adc2_frame_buffer);

    uint32_t rx_chan = ComMgr_GetRxSelect();
    int16_t *active_buffer = NULL;

    if (rx_chan == 1U && has_frame1)
    {
        active_buffer = adc1_frame_buffer;
    }
    else if (rx_chan == 2U && has_frame2)
    {
        active_buffer = adc2_frame_buffer;
    }

    if (active_buffer != NULL)
    {
        receiver_frame[3] = (uint8_t)('0' + rx_chan);
        memcpy(&receiver_frame[RECEIVER_FRAME_HEADER_SIZE], active_buffer, RECEIVER_FRAME_PAYLOAD_SIZE);
        ComMgr_SendData(receiver_frame, sizeof(receiver_frame));
    }
}