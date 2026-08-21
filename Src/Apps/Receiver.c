#include "Receiver.h"
#include <string.h>

#include "ADCService.h"
#include "ComMgr.h"

static uint8_t receiver_frame[4112];
static int16_t adc1_frame_buffer[2048];
static int16_t adc2_frame_buffer[2048];

void Receiver_Init(void)
{
    ADCService_Init(1U);
    ADCService_Init(2U);

    receiver_frame[0] = 'F';
    receiver_frame[1] = 'R';
    receiver_frame[2] = 'X';
    receiver_frame[3] = '1';
    receiver_frame[4] = 0x00U;
    receiver_frame[5] = 0x08U;
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
        memcpy(&receiver_frame[16], active_buffer, 2048U * sizeof(int16_t));
        ComMgr_SendData(receiver_frame, sizeof(receiver_frame));
    }
}