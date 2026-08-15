#include "Transmitter.h"

#include "DACService.h"
#include "stm32h5xx.h"
#include <string.h>

#define TRANSMITTER_SAMPLE_COUNT 2048U
#define TRANSMITTER_BIAS 2048U
#define TRANSMITTER_SINGLE_LENGTH 8U
#define TRANSMITTER_PULSE_HIGH 4048U
#define TRANSMITTER_PULSE_LOW 48U
#define TRANSMITTER_BARKER13_LENGTH 104U
#define TRANSMITTER_BARKER13_INVERTED_MIN 148U
#define TRANSMITTER_BARKER13_INVERTED_MAX 3948U
#ifdef SIMULATION_MODE
#define TRANSMITTER_SIMULATION_DELAY 500U
#define TRANSMITTER_SIMULATION_UPDATE_MS 100U

#define TRANSMITTER_DAC_RESOLUTION 12U
#define TRANSMITTER_DAC_MIN 0
#define TRANSMITTER_DAC_MAX ((1U << TRANSMITTER_DAC_RESOLUTION) - 1U)

#define TRANSMITTER_LCG_MULTIPLIER 1103515245U
#define TRANSMITTER_LCG_INCREMENT 12345U
#define TRANSMITTER_LCG_SCALE 65536U
#define TRANSMITTER_NOISE_RANGE 512U
#define TRANSMITTER_NOISE_OFFSET ((int16_t)(TRANSMITTER_NOISE_RANGE / 2U))

#define TRANSMITTER_ATTENUATION_SHIFT 1U // 50% amplitude (shift right by 1)
#define TRANSMITTER_RISE_SAMPLES 4U      // 4 samples rise time
#define TRANSMITTER_DECAY_SAMPLES 32U    // 32 samples ringing decay time
#endif

static const uint16_t transmitter_single_waveform[TRANSMITTER_SAMPLE_COUNT] = {
    TRANSMITTER_BIAS,
    TRANSMITTER_PULSE_HIGH,
    TRANSMITTER_BIAS,
    TRANSMITTER_PULSE_LOW,
    TRANSMITTER_BIAS,
    TRANSMITTER_PULSE_HIGH,
    TRANSMITTER_BIAS,
    TRANSMITTER_PULSE_LOW,
    [TRANSMITTER_SINGLE_LENGTH... TRANSMITTER_SAMPLE_COUNT - 1U] =
        TRANSMITTER_BIAS};

#define TRANSMITTER_BARKER_POSITIVE_CHIP                                       \
  TRANSMITTER_BIAS, TRANSMITTER_PULSE_HIGH, TRANSMITTER_BIAS,                  \
      TRANSMITTER_PULSE_LOW, TRANSMITTER_BIAS, TRANSMITTER_PULSE_HIGH,         \
      TRANSMITTER_BIAS, TRANSMITTER_PULSE_LOW
#define TRANSMITTER_BARKER_NEGATIVE_CHIP                                       \
  TRANSMITTER_BIAS, TRANSMITTER_BARKER13_INVERTED_MIN, TRANSMITTER_BIAS,       \
      TRANSMITTER_BARKER13_INVERTED_MAX, TRANSMITTER_BIAS,                     \
      TRANSMITTER_BARKER13_INVERTED_MIN, TRANSMITTER_BIAS,                     \
      TRANSMITTER_BARKER13_INVERTED_MAX

static const uint16_t transmitter_barker13_waveform[TRANSMITTER_SAMPLE_COUNT] =
    {TRANSMITTER_BARKER_POSITIVE_CHIP,
     TRANSMITTER_BARKER_POSITIVE_CHIP,
     TRANSMITTER_BARKER_POSITIVE_CHIP,
     TRANSMITTER_BARKER_POSITIVE_CHIP,
     TRANSMITTER_BARKER_POSITIVE_CHIP,
     TRANSMITTER_BARKER_NEGATIVE_CHIP,
     TRANSMITTER_BARKER_NEGATIVE_CHIP,
     TRANSMITTER_BARKER_POSITIVE_CHIP,
     TRANSMITTER_BARKER_POSITIVE_CHIP,
     TRANSMITTER_BARKER_NEGATIVE_CHIP,
     TRANSMITTER_BARKER_POSITIVE_CHIP,
     TRANSMITTER_BARKER_NEGATIVE_CHIP,
     TRANSMITTER_BARKER_POSITIVE_CHIP,
     [TRANSMITTER_BARKER13_LENGTH... TRANSMITTER_SAMPLE_COUNT - 1U] =
         TRANSMITTER_BIAS};

static volatile uint16_t transmitter_samples[TRANSMITTER_SAMPLE_COUNT]
    __attribute__((aligned(32)));

#ifdef SIMULATION_MODE
static uint32_t noise_seed = TRANSMITTER_LCG_INCREMENT;
static const uint16_t *active_waveform_src = transmitter_single_waveform;
static uint32_t active_waveform_length = TRANSMITTER_SINGLE_LENGTH;

static int16_t get_random_noise(void) {
  noise_seed =
      noise_seed * TRANSMITTER_LCG_MULTIPLIER + TRANSMITTER_LCG_INCREMENT;
  return (int16_t)((noise_seed >> 16U) & (TRANSMITTER_NOISE_RANGE - 1U)) -
         TRANSMITTER_NOISE_OFFSET;
}

static void fill_samples_with_noise_and_pulse_doppler(const uint16_t *src,
                                                      uint32_t active_length,
                                                      uint32_t delay) {
  uint32_t i = 0U;

  // 1. Noise before pulse (branchless)
  for (; i < delay; i++) {
    transmitter_samples[i] = (uint16_t)(TRANSMITTER_BIAS + get_random_noise());
  }

  // 2. Active pulse with rise envelope and attenuation + noise (branchless)
  uint32_t end_pulse = delay + active_length;
  for (; i < end_pulse; i++) {
    uint32_t idx = i - delay;
    int32_t dev = (int32_t)src[idx] - TRANSMITTER_BIAS;

    // Méo trước: rise envelope
    if (idx < TRANSMITTER_RISE_SAMPLES) {
      dev = (dev * (int32_t)idx) >> 2; // dev * idx / 4
    }

    // Suy hao
    dev = dev >> TRANSMITTER_ATTENUATION_SHIFT;

    int32_t val = TRANSMITTER_BIAS + dev + get_random_noise();
    if (val < TRANSMITTER_DAC_MIN)
      val = TRANSMITTER_DAC_MIN;
    if (val > TRANSMITTER_DAC_MAX)
      val = TRANSMITTER_DAC_MAX;
    transmitter_samples[i] = (uint16_t)val;
  }

  // 3. Ringing tail (méo sau / decay tail) + noise (branchless)
  uint32_t end_decay = end_pulse + TRANSMITTER_DECAY_SAMPLES;
  for (; i < end_decay; i++) {
    uint32_t idx = i - end_pulse;
    int32_t osc_dev = 0;
    uint32_t phase = idx & 3U; // idx % 4
    if (phase == 0U) {
      osc_dev = (int32_t)TRANSMITTER_PULSE_HIGH - TRANSMITTER_BIAS;
    } else if (phase == 2U) {
      osc_dev = (int32_t)TRANSMITTER_PULSE_LOW - TRANSMITTER_BIAS;
    }

    // Méo sau: decay envelope
    osc_dev = (osc_dev * (int32_t)(TRANSMITTER_DECAY_SAMPLES - idx)) >>
              5; // osc_dev * (32 - idx) / 32

    // Suy hao
    osc_dev = osc_dev >> TRANSMITTER_ATTENUATION_SHIFT;

    int32_t val = TRANSMITTER_BIAS + osc_dev + get_random_noise();
    if (val < TRANSMITTER_DAC_MIN)
      val = TRANSMITTER_DAC_MIN;
    if (val > TRANSMITTER_DAC_MAX)
      val = TRANSMITTER_DAC_MAX;
    transmitter_samples[i] = (uint16_t)val;
  }

  // 4. Noise after pulse and tail (branchless)
  for (; i < TRANSMITTER_SAMPLE_COUNT; i++) {
    transmitter_samples[i] = (uint16_t)(TRANSMITTER_BIAS + get_random_noise());
  }
}
#endif

void Transmitter_Init(void) {
#ifdef SIMULATION_MODE
  fill_samples_with_noise_and_pulse_doppler(transmitter_single_waveform,
                                            TRANSMITTER_SINGLE_LENGTH,
                                            TRANSMITTER_SIMULATION_DELAY);
#else
  (void)memcpy((void *)transmitter_samples, transmitter_single_waveform,
               sizeof(transmitter_samples));
#endif

  DACService_Init((const uint16_t *)transmitter_samples,
                  TRANSMITTER_SAMPLE_COUNT);
}

static volatile Transmitter_PulseType current_pulse_type = TRANSMITTER_PULSE_SINGLE;

Transmitter_PulseType Transmitter_GetPulseType(void) {
  return current_pulse_type;
}

void Transmitter_SetPulseType(Transmitter_PulseType pulse_type) {
  current_pulse_type = pulse_type;
  const uint16_t *src = NULL;
#ifdef SIMULATION_MODE
  uint32_t active_length = 0U;
#endif

  switch (pulse_type) {
  case TRANSMITTER_PULSE_BARKER13:
    src = transmitter_barker13_waveform;
#ifdef SIMULATION_MODE
    active_length = TRANSMITTER_BARKER13_LENGTH;
#endif
    break;
  case TRANSMITTER_PULSE_SINGLE:
  default:
    src = transmitter_single_waveform;
#ifdef SIMULATION_MODE
    active_length = TRANSMITTER_SINGLE_LENGTH;
#endif
    break;
  }

  if (src != NULL) {
#ifdef SIMULATION_MODE
    active_waveform_src = src;
    active_waveform_length = active_length;
    fill_samples_with_noise_and_pulse_doppler(src, active_length,
                                              TRANSMITTER_SIMULATION_DELAY);
#else
    (void)memcpy((void *)transmitter_samples, src, sizeof(transmitter_samples));
#endif
  }
}
#ifdef SIMULATION_MODE
void Transmitter_UpdateNoise(void) {

  static uint32_t last_completed_count = 0U;
  uint32_t current_completed = DACService_GetCompletedCount();

  // Check if the DMA has finished transmitting a frame
  if (current_completed != last_completed_count) {
    last_completed_count = current_completed;

    // Coherent Doppler shift: commented out as phase shift between pulses is
    // not needed yet
    static uint32_t doppler_shift_samples = 0U;
    // doppler_shift_samples = (doppler_shift_samples + 1U) & 3U;

    if (active_waveform_src != NULL) {
      fill_samples_with_noise_and_pulse_doppler(
          active_waveform_src, active_waveform_length,
          TRANSMITTER_SIMULATION_DELAY + doppler_shift_samples);
    }
  }
}
#endif