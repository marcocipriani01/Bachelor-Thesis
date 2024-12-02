#ifndef DMM_DSP_H
#define DMM_DSP_H

#include "arm_math.h"

#include <stdint.h>
#include <stdbool.h>

#define DMM_BUF_SIZE 4096
#define FFT_MAG_COUNT (DMM_BUF_SIZE / 2)
#define FFT_PEAKS_MAX (FFT_MAG_COUNT / 2)

#define FFT_PEAK_SEARCH_STEP 5
#define FFT_PEAK_FACTOR 0.5f

#define FFT_FREQ_TOLERANCE 0.05f
#define FFT_AMPLITUDE_TOLERANCE 0.1f
#define FFT_NOISE_THRESHOLD 0.1f


typedef enum {
    RECT, HANNINING, HAMMING, BLACKMAN, BARTLETT, WELCH, FLAT_TOP
} FFTWindow;

typedef enum {
    UNKNOWN_WAVE = 0,
    DC = 1,
    SINE = 2,
    RECTIFIED_SINE = 3,
    SQUARE = 4,
    TRIANGLE = 5,
    SAWTOOTH = 6
} Waveform;

extern float dmmDataMin;
extern float dmmDataMax;
extern float dmmDataSum;
extern float dmmDataRMSSq;

void DMM_DSP_Init(FFTWindow window);

void DMM_DSP_AddADCData(const uint16_t* data, float offset, float factor);

const float32_t* DMM_DSP_Process(void);

static inline float DMM_DSP_GetMax(void) {
    return dmmDataMax;
}

static inline float DMM_DSP_GetMin(void) {
    return dmmDataMin;
}

static inline float DMM_DSP_GetAvg(void) {
    return (dmmDataSum / DMM_BUF_SIZE);
}

static inline float DMM_DSP_GetRMS(void) {
    return sqrtf(dmmDataRMSSq / DMM_BUF_SIZE);
}

static inline float DMM_DSP_IndexToFreq(const uint16_t i, const float samplingRate) {
    return ((float) i) * samplingRate / ((float) DMM_BUF_SIZE);
}

uint16_t DMM_DSP_FindPeaks(uint16_t* peaks);

uint16_t DMM_DSP_FindFundamental(const uint16_t* peaks, size_t count);

bool DMM_DSP_IsDC(const uint16_t* peaks, size_t count);

bool DMM_DSP_IsPureSine(uint16_t fundamental, const uint16_t* peaks, size_t count);

uint16_t DMM_DSP_FindNthHarmonic(uint16_t fundamental, const uint16_t* peaks, size_t count, uint16_t n);

bool DMM_DSP_MatchHarmonicCoeff(uint16_t fundamental, const uint16_t* peaks, size_t count, uint16_t n, float coeff);

Waveform DMM_DSP_DetectWaveform(uint16_t fundamental, const uint16_t* peaks, size_t count);

#endif
