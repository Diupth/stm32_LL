#include "FakedReceiver.h"
#include "ComMgr.h"
#include "stm32h5xx_hal.h"

#include <stdint.h>

#define FAKED_RECEIVER_SAMPLE_COUNT 2048U
#define FAKED_RECEIVER_FRAME_SIZE (8U + (FAKED_RECEIVER_SAMPLE_COUNT * 2U))
#define FAKED_RECEIVER_FRAME_PERIOD_MS 13U

static uint8_t frame[FAKED_RECEIVER_FRAME_SIZE];
static uint32_t next_frame_tick;

static void FakedReceiver_WriteU16(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value & 0xffU);
    destination[1] = (uint8_t)(value >> 8U);
}

void FakedReceiver_Init(void)
{
    frame[0] = 'F';
    frame[1] = 'R';
    frame[2] = 'X';
    frame[3] = '1';
    FakedReceiver_WriteU16(&frame[4], FAKED_RECEIVER_SAMPLE_COUNT);
    frame[6] = 0U;
    frame[7] = 0U;

    for (uint32_t sample_index = 0U; sample_index < FAKED_RECEIVER_SAMPLE_COUNT; sample_index++)
    {
        int16_t sample;
        switch (sample_index & 3U)
        {
        case 0U:
            sample = 16384;
            break;
        case 1U:
            sample = 32767;
            break;
        case 2U:
            sample = 16384;
            break;
        default:
            sample = 0;
            break;
        }

        FakedReceiver_WriteU16(&frame[8U + (sample_index * 2U)], (uint16_t)sample);
    }

    next_frame_tick = HAL_GetTick();
}

void FakedReceiver_Process(void)
{
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - next_frame_tick) < 0)
    {
        return;
    }

    ComMgr_SendData(frame, FAKED_RECEIVER_FRAME_SIZE);
    next_frame_tick = now + FAKED_RECEIVER_FRAME_PERIOD_MS;
}