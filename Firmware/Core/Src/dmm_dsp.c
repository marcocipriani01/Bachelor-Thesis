#include "dmm_dsp.h"


float dmmDataMin = 0.0f;
float dmmDataMax = 0.0f;
float dmmDataSum = 0.0f;
float dmmDataRMSSq = 0.0f;

static arm_rfft_fast_instance_f32 fftInst;
static float32_t fftWindow[DMM_BUF_SIZE];
static float32_t fftInputBuf[DMM_BUF_SIZE];
static float32_t fftMagnitudes[FFT_MAG_COUNT];
static float fftScaling;

void DMM_DSP_Init(const FFTWindow window) {
    arm_rfft_fast_init_f32(&fftInst, DMM_BUF_SIZE);

    switch (window) {
        case HANNINING:
            arm_hanning_f32(fftWindow, DMM_BUF_SIZE);
            break;
        case HAMMING:
            arm_hamming_f32(fftWindow, DMM_BUF_SIZE);
            break;
        case BLACKMAN:
            arm_blackman_harris_92db_f32(fftWindow, DMM_BUF_SIZE);
            break;
        case BARTLETT:
            arm_bartlett_f32(fftWindow, DMM_BUF_SIZE);
            break;
        case WELCH:
            arm_welch_f32(fftWindow, DMM_BUF_SIZE);
            break;
        case FLAT_TOP:
            arm_hft95_f32(fftWindow, DMM_BUF_SIZE);
            break;
        case RECT:
        default: {
            for (size_t i = 0; i < DMM_BUF_SIZE; ++i) {
                fftWindow[i] = 1.0f;
            }
            break;
        }
    }
    float sum;
    arm_accumulate_f32(fftWindow, DMM_BUF_SIZE, &sum);
    fftScaling = sum * (2.0f / (DMM_BUF_SIZE * DMM_BUF_SIZE));
}

void DMM_DSP_AddADCData(const uint16_t* data, const float offset, const float factor) {
    dmmDataMin = FLT_MAX;
    dmmDataMax = FLT_MIN;
    dmmDataSum = 0.0f;
    dmmDataRMSSq = 0.0f;
    for (size_t i = 0; i < DMM_BUF_SIZE; ++i) {
        const float val = (((float) data[i]) - offset) * factor;
        if (val < dmmDataMin) dmmDataMin = val;
        if (val > dmmDataMax) dmmDataMax = val;
        dmmDataSum += val;
        dmmDataRMSSq += (val * val);
        fftInputBuf[i] = fftWindow[i] * val;
    }
}

const float32_t* DMM_DSP_Process(void) {
    float fftBuf[DMM_BUF_SIZE];
    arm_rfft_fast_f32(&fftInst, fftInputBuf, fftBuf, 0);
    for (size_t i = 0; i < FFT_MAG_COUNT; ++i) {
        const float real = fftBuf[i * 2];
        const float imag = fftBuf[(i * 2) + 1];
        fftMagnitudes[i] = sqrtf((real * real) + (imag * imag)) * fftScaling;
    }
    return fftMagnitudes;
}

uint16_t DMM_DSP_FindPeaks(uint16_t* peaks) {
    memset(peaks, 0, FFT_PEAKS_MAX);
    uint16_t count = 0;
    for (size_t i = FFT_PEAK_SEARCH_STEP; i < (FFT_MAG_COUNT - FFT_PEAK_SEARCH_STEP); ++i) {
        if (fftMagnitudes[i - 1] < fftMagnitudes[i]) {
            size_t j = i + 1;
            while ((j < (FFT_MAG_COUNT - FFT_PEAK_SEARCH_STEP)) && (fftMagnitudes[j] == fftMagnitudes[i])) {
                j++;
            }

            if (fftMagnitudes[j] < fftMagnitudes[i]) {
                const uint16_t midIdx = (i + j - 1) / 2;
                const float peakVal = fftMagnitudes[i] * FFT_PEAK_FACTOR;
                if ((peakVal > fftMagnitudes[midIdx - FFT_PEAK_SEARCH_STEP]) &&
                    (peakVal > fftMagnitudes[midIdx + FFT_PEAK_SEARCH_STEP])) {
                    peaks[count++] = midIdx;
                }
                i = j;
            }
        }
    }
    return count;
}

uint16_t DMM_DSP_FindFundamental(const uint16_t* peaks, const size_t count) {
    uint32_t fundamental = 0;
    float32_t max = 0.0f;
    for (size_t i = 0; i < count; ++i) {
        const uint32_t peak = peaks[i];
        const float val = fftMagnitudes[peak];
        if (val > max) {
            fundamental = peak;
            max = val;
        }
    }
    return fundamental;
}

bool DMM_DSP_IsDC(const uint16_t* peaks, const size_t count) {
    if (count == 0) return true;

    const float noiseThreshold = fftMagnitudes[0] * FFT_NOISE_THRESHOLD;

    for (size_t i = 0; i < count; ++i) {
        if (fftMagnitudes[peaks[i]] > noiseThreshold) return false;
    }
    return true;
}

bool DMM_DSP_IsPureSine(const uint16_t fundamental, const uint16_t* peaks, const size_t count) {
    if (count == 0) return false;
    if (count == 1) return true;

    const float noiseThreshold = fftMagnitudes[fundamental] * FFT_NOISE_THRESHOLD;

    for (size_t i = 0; i < count; ++i) {
        if ((peaks[i] != fundamental) && (fftMagnitudes[peaks[i]] > noiseThreshold))
            return false;
    }
    return true;
}

uint16_t DMM_DSP_FindNthHarmonic(const uint16_t fundamental, const uint16_t* peaks,
                                 const size_t count, const uint16_t n) {
    if (count < 2) return false;

    const float minIdx = ((float) (n * fundamental)) * (1.0f - FFT_FREQ_TOLERANCE);
    const float maxIdx = ((float) (n * fundamental)) * (1.0f + FFT_FREQ_TOLERANCE);

    for (size_t i = 1; i < count; ++i) {
        const float peakIdx = peaks[i];
        if (peakIdx >= maxIdx) return 0;
        if (peakIdx >= minIdx) return peaks[i];
    }
    return 0;
}

bool DMM_DSP_MatchHarmonicCoeff(const uint16_t fundamental, const uint16_t* peaks, const size_t count,
                                const uint16_t n, const float coeff) {
    if (count < 2) return false;

    const float minIdx = ((float) (n * fundamental)) * (1.0f - FFT_FREQ_TOLERANCE);
    const float maxIdx = ((float) (n * fundamental)) * (1.0f + FFT_FREQ_TOLERANCE);
    const float minVal = (fftMagnitudes[fundamental] * coeff) * (1.0f - FFT_AMPLITUDE_TOLERANCE);
    const float maxVal = (fftMagnitudes[fundamental] * coeff) * (1.0f + FFT_AMPLITUDE_TOLERANCE);

    for (size_t i = 1; i < count; ++i) {
        const float peakIdx = peaks[i];
        if (peakIdx >= maxIdx) return false;
        if (peakIdx >= minIdx) {
            const float peakVal = fftMagnitudes[peaks[i]];
            if ((peakVal >= minVal) && (peakVal <= maxVal)) return true;
        }
    }
    return false;
}

Waveform DMM_DSP_DetectWaveform(const uint16_t fundamental, const uint16_t* peaks, const size_t count) {
    if (DMM_DSP_IsDC(peaks, count))
        return DC;
    if (DMM_DSP_IsPureSine(fundamental, peaks, count))
        return SINE;
    if (DMM_DSP_MatchHarmonicCoeff(fundamental, peaks, count, 3, 1.0f / 3.0f)) {
        // Sawtooth wave Fourier series: x(t) = (-2/π) * Σ [((-1)^k)/k * sin(2πkt)]
        if (DMM_DSP_MatchHarmonicCoeff(fundamental, peaks, count, 2, 1.0f / 2.0f))
            return SAWTOOTH;
        // Square wave Fourier series: x(t) = (4/π) * Σ [1/(2k-1) * sin(2π(2k-1)t)]
        return SQUARE;
    }
    // Triangle wave Fourier series: x(t) = (-8/π^2) * Σ [((-1)^k)/(2k-1)^2 * sin(2π(2k-1)t)]
    if (DMM_DSP_MatchHarmonicCoeff(fundamental, peaks, count, 3, 1.0f / 9.0f))
        return TRIANGLE;
    // Rectified sine wave harmonic coefficients: a_k = (-2)/(π*(4(k^2)-1))
    if (DMM_DSP_MatchHarmonicCoeff(fundamental, peaks, count, 2, 1.0f / (4.0f * (2.0f * 2.0f) - 1.0f)))
        return RECTIFIED_SINE;
    return UNKNOWN_WAVE;
}
