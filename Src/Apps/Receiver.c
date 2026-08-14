#include "Receiver.h"

#include "ADCService.h"
#include "ComMgr.h"
#include "stm32h5xx.h"
#define FRAME_SAMPLES          2048U
#define FRAME_HEADER_SIZE      16U
#define RECEIVER_FRAME_SIZE    (FRAME_HEADER_SIZE + (FRAME_SAMPLES * 2U))

#define ADC_BIAS               2048
#define ADC_MAX_VAL            4095

#define BPF_GAIN_SHIFT         3U
#define BPF_FEEDBACK_SHIFT     2U

#define FS_HZ                  160000U
#define LOG_INTERVAL_MS        1000U
#define LOG_INTERVAL_FRAMES    ((FS_HZ * LOG_INTERVAL_MS) / (FRAME_SAMPLES * 1000U))
#define FALLBACK_CLOCK_MHZ     250U

#define DSP_HEADER_SIZE        8U
#define DSP_VALUE_COUNT        4U
#define DSP_FRAME_SIZE         (DSP_HEADER_SIZE + (DSP_VALUE_COUNT * 4U))

typedef struct {
    int32_t x1;
    int32_t x2;
    int32_t y1;
    int32_t y2;
} BPF_State_t;

static uint8_t receiver_frame[RECEIVER_FRAME_SIZE];
static int16_t adc1_frame_buffer[FRAME_SAMPLES];
static int16_t adc2_frame_buffer[FRAME_SAMPLES];

// BPF Filter States - Mảng chứa trạng thái bộ lọc BPF 35-45 kHz cho 2 kênh
static BPF_State_t bpf_states[2] = { {0, 0, 0, 0}, {0, 0, 0, 0} };

#ifdef SHOW_SAMPLING_LOG
static uint32_t log_counter = 0U;
#endif

// Bộ lọc thông dải IIR bậc 2 dải thông 35 kHz - 45 kHz (tại FS = 160 kHz)
static void Receiver_FilterBPF(int16_t *buffer, uint32_t length, BPF_State_t *state)
{
    for (uint32_t i = 0U; i < length; i++)
    {
        // Loại bỏ DC bias
        int32_t x_q16 = ((int32_t)buffer[i] - ADC_BIAS) << 16;
        
        // Phương trình sai phân: y[n] = (x[n] - x[n-2]) / 8 - 0.75 * y[n-2]
        int32_t diff_x = x_q16 - (state->x2);
        int32_t term1 = diff_x >> BPF_GAIN_SHIFT;
        int32_t term2 = (state->y2) - ((state->y2) >> BPF_FEEDBACK_SHIFT);
        int32_t y_q16 = term1 - term2;
        
        // Cập nhật trạng thái lịch sử
        state->x2 = state->x1;
        state->x1 = x_q16;
        
        state->y2 = state->y1;
        state->y1 = y_q16;
        
        // Đưa về dạng 12-bit nguyên bản và cộng lại DC bias
        int32_t bp = (y_q16 >> 16) + ADC_BIAS;
        
        // Giới hạn trong dải ADC 12-bit [0, 4095]
        if (bp > ADC_MAX_VAL)
        {
            bp = ADC_MAX_VAL;
        }
        else if (bp < 0)
        {
            bp = 0;
        }
        
        buffer[i] = (int16_t)bp;
    }
}

void Receiver_Init(void)
{
    ADCService_Init(1U);
    ADCService_Init(2U);

    receiver_frame[0] = 'F';
    receiver_frame[1] = 'R';
    receiver_frame[2] = 'X';
    receiver_frame[3] = '1';
    receiver_frame[4] = (uint8_t)(FRAME_SAMPLES & 0xFFU);
    receiver_frame[5] = (uint8_t)((FRAME_SAMPLES >> 8U) & 0xFFU);
}

void Receiver_Process(void)
{
#ifdef SHOW_SAMPLING_LOG
    uint32_t t_start = DWT->CYCCNT;
    uint32_t t_read_start = DWT->CYCCNT;
#endif

    bool has_frame[2] = {
        ADCService_ReadFrame(1U, adc1_frame_buffer),
        ADCService_ReadFrame(2U, adc2_frame_buffer)
    };

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_read_end = DWT->CYCCNT;
    uint32_t t_bpf_start = DWT->CYCCNT;
#endif

    int16_t *buffers[2] = { adc1_frame_buffer, adc2_frame_buffer };
    for (uint32_t i = 0U; i < 2U; i++)
    {
        if (has_frame[i])
        {
            Receiver_FilterBPF(buffers[i], FRAME_SAMPLES, &bpf_states[i]);
        }
    }

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_bpf_end = DWT->CYCCNT;
#endif
    uint32_t rx_chan = ComMgr_GetRxSelect();
    int16_t *active_buffer = NULL;

    if (rx_chan >= 1U && rx_chan <= 2U && has_frame[rx_chan - 1U])
    {
        active_buffer = buffers[rx_chan - 1U];
    }

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_send_start = 0U;
    uint32_t t_send_end = 0U;
#endif
    bool sent = false;

    if (active_buffer != NULL)
    {
#ifdef SHOW_SAMPLING_LOG
        t_send_start = DWT->CYCCNT;
#endif
        receiver_frame[3] = (uint8_t)('0' + rx_chan);
        for (uint32_t i = 0U; i < FRAME_SAMPLES; i++)
        {
            // Copy 16-bit values into byte array (little-endian)
            receiver_frame[FRAME_HEADER_SIZE + i * 2] = (uint8_t)(active_buffer[i] & 0xFF);
            receiver_frame[FRAME_HEADER_SIZE + i * 2 + 1] = (uint8_t)((active_buffer[i] >> 8) & 0xFF);
        }
        ComMgr_SendData(receiver_frame, sizeof(receiver_frame));
#ifdef SHOW_SAMPLING_LOG
        t_send_end = DWT->CYCCNT;
#endif
        sent = true;
    }

#ifdef SHOW_SAMPLING_LOG
    uint32_t t_end = DWT->CYCCNT;

    if (has_frame[0] || has_frame[1])
    {
        log_counter++;
        if (log_counter >= LOG_INTERVAL_FRAMES)
        {
            log_counter = 0U; // Đã khôi phục câu lệnh reset biến đếm log
            uint32_t clock_mhz = SystemCoreClock / 1000000U;
            if (clock_mhz == 0U)
            {
                clock_mhz = FALLBACK_CLOCK_MHZ;
            }

            uint32_t total_cycles = t_end - t_start;
            uint32_t read_cycles = t_read_end - t_read_start;
            uint32_t bpf_cycles = t_bpf_end - t_bpf_start;
            uint32_t send_cycles = sent ? (t_send_end - t_send_start) : 0U;

            uint32_t total_us = total_cycles / clock_mhz;
            uint32_t read_us = read_cycles / clock_mhz;
            uint32_t bpf_us = bpf_cycles / clock_mhz;
            uint32_t send_us = send_cycles / clock_mhz;

            // Sắp xếp bản tin nhị phân gửi đi
            uint8_t dsp_frame[DSP_FRAME_SIZE];
            dsp_frame[0] = 'D';
            dsp_frame[1] = 'S';
            dsp_frame[2] = 'P';
            dsp_frame[3] = '1';

            // Sequence (Little-Endian)
            dsp_frame[4] = (uint8_t)(log_counter & 0xFFU);
            dsp_frame[5] = (uint8_t)((log_counter >> 8U) & 0xFFU);
            dsp_frame[6] = (uint8_t)((log_counter >> 16U) & 0xFFU);
            dsp_frame[7] = (uint8_t)((log_counter >> 24U) & 0xFFU);

            // Ghi các giá trị payload (mỗi giá trị 4 bytes)
            uint32_t values[DSP_VALUE_COUNT] = { total_us, read_us, bpf_us, send_us };
            for (uint32_t v = 0U; v < DSP_VALUE_COUNT; v++)
            {
                for (uint32_t b = 0U; b < 4U; b++)
                {
                    dsp_frame[DSP_HEADER_SIZE + v * 4U + b] = (uint8_t)(values[v] >> (b * 8U));
                }
            }

            ComMgr_SendData(dsp_frame, sizeof(dsp_frame));
        }
    }
#endif
}