#include "Receiver.h"

#include "ADCService.h"
#include "ComMgr.h"

static uint8_t receiver_frame[4112];

void Receiver_Init(void)
{
    ADCService_Init();

    receiver_frame[0] = 'F';
    receiver_frame[1] = 'R';
    receiver_frame[2] = 'X';
    receiver_frame[3] = '1';
    receiver_frame[4] = 0x00U;
    receiver_frame[5] = 0x08U;
}

void Receiver_Process(void)
{
    if (ADCService_ReadFrame((int16_t *)&receiver_frame[16]))
    {
        ComMgr_SendData(receiver_frame, sizeof(receiver_frame));
    }
}