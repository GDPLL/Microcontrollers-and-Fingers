/*
 * ESP32 Sixth Finger - EMG Signal Processor Implementation
 * Author: Qoder
 * Date: 2025
 */

#include "emg_processor.h"
#include <math.h>

// ============================================
// Constructor
// ============================================

EMGProcessor::EMGProcessor() 
    : _filtered_ch1(0), _filtered_ch2(0)
    , _prev_raw_ch1(0), _prev_raw_ch2(0)
    , _prev_filtered_ch1(0), _prev_filtered_ch2(0)
    , _envelope_ch1(0), _envelope_ch2(0)
    , _mavg_index(0)
{
    // Initialize moving average buffers
    for (int i = 0; i < MAVG_SIZE; i++) {
        _mavg_buffer_ch1[i] = 0;
        _mavg_buffer_ch2[i] = 0;
    }
}

// ============================================
// Initialization
// ============================================

void EMGProcessor::begin() {
    // Configure ADC
    analogReadResolution(12);  // 12-bit ADC (0-4095)
    analogSetAttenuation(ADC_11db);  // Full scale voltage range
    
    // Initialize filter states
    _filtered_ch1 = 0;
    _filtered_ch2 = 0;
    _prev_raw_ch1 = 0;
    _prev_raw_ch2 = 0;
    _prev_filtered_ch1 = 0;
    _prev_filtered_ch2 = 0;
    _envelope_ch1 = 0;
    _envelope_ch2 = 0;
    _mavg_index = 0;
    
    Serial.println("EMG Processor initialized");
}

// ============================================
// Raw Reading
// ============================================

void EMGProcessor::readRaw(uint16_t& ch1, uint16_t& ch2) {
    ch1 = analogRead(EMG_CH1_PIN);
    ch2 = analogRead(EMG_CH2_PIN);
}

// ============================================
// Filtering
// ============================================

void EMGProcessor::filter(uint16_t raw_ch1, uint16_t raw_ch2, 
                          float& filtered_ch1, float& filtered_ch2) {
    // Convert to signed value (center around 2048 for 12-bit ADC)
    float input_ch1 = (float)raw_ch1 - 2048.0f;
    float input_ch2 = (float)raw_ch2 - 2048.0f;
    
    // Apply high-pass filter (remove DC offset)
    static float hp_state_ch1[2] = {0};  // [prev_input, prev_output]
    static float hp_state_ch2[2] = {0};
    
    float hp_ch1 = highPassFilter(input_ch1, hp_state_ch1[0], hp_state_ch1[1]);
    float hp_ch2 = highPassFilter(input_ch2, hp_state_ch2[0], hp_state_ch2[1]);
    
    // Update high-pass filter state
    hp_state_ch1[0] = input_ch1;
    hp_state_ch1[1] = hp_ch1;
    hp_state_ch2[0] = input_ch2;
    hp_state_ch2[1] = hp_ch2;
    
    // Apply notch filter (50Hz power line interference)
    static float notch_state_ch1[4] = {0};  // [x_n1, x_n2, y_n1, y_n2]
    static float notch_state_ch2[4] = {0};
    
    float notch_ch1 = notchFilter(hp_ch1, notch_state_ch1[0], notch_state_ch1[1], 
                                   notch_state_ch1[2], notch_state_ch1[3]);
    float notch_ch2 = notchFilter(hp_ch2, notch_state_ch2[0], notch_state_ch2[1], 
                                   notch_state_ch2[2], notch_state_ch2[3]);
    
    // Update notch filter state
    notch_state_ch1[1] = notch_state_ch1[0];
    notch_state_ch1[0] = hp_ch1;
    notch_state_ch1[3] = notch_state_ch1[2];
    notch_state_ch1[2] = notch_ch1;
    notch_state_ch2[1] = notch_state_ch2[0];
    notch_state_ch2[0] = hp_ch2;
    notch_state_ch2[3] = notch_state_ch2[2];
    notch_state_ch2[2] = notch_ch2;
    
    // Apply moving average filter
    filtered_ch1 = movingAverage(notch_ch1, _mavg_buffer_ch1, _mavg_index);
    filtered_ch2 = movingAverage(notch_ch2, _mavg_buffer_ch2, _mavg_index);
    
    // Store filtered values
    _filtered_ch1 = filtered_ch1;
    _filtered_ch2 = filtered_ch2;
    
    // Update envelope
    updateEnvelope(filtered_ch1, filtered_ch2);
    
    // Store previous values
    _prev_raw_ch1 = input_ch1;
    _prev_raw_ch2 = input_ch2;
    _prev_filtered_ch1 = filtered_ch1;
    _prev_filtered_ch2 = filtered_ch2;
}

// ============================================
// Filter Implementations
// ============================================

float EMGProcessor::highPassFilter(float input, float& prev_input, float& prev_output) {
    // First-order high-pass filter
    // Cutoff frequency: 10Hz at 1000Hz sampling rate
    // H(z) = (1 - alpha) * (z - 1) / (z - alpha)
    const float alpha = 0.9391f;  // For 10Hz cutoff at 1000Hz
    
    float output = alpha * (prev_output + input - prev_input);
    return output;
}

float EMGProcessor::lowPassFilter(float input, float prev_output) {
    // First-order low-pass filter
    // Cutoff frequency: 450Hz at 1000Hz sampling rate
    const float alpha = 0.7488f;  // For 450Hz cutoff at 1000Hz
    
    return alpha * prev_output + (1.0f - alpha) * input;
}

float EMGProcessor::notchFilter(float input, float& x_n1, float& x_n2, 
                                 float& y_n1, float& y_n2) {
    // Second-order IIR notch filter for 50Hz power line interference
    // Sampling rate: 1000Hz
    
    // Filter coefficients for 50Hz notch at 1000Hz
    const float r = 0.95f;  // Pole radius (bandwidth control)
    const float cos_w0 = 0.9511f;  // cos(2*pi*50/1000)
    
    // Transfer function: H(z) = (1 - 2*cos(w0)*z^-1 + z^-2) / (1 - 2*r*cos(w0)*z^-1 + r^2*z^-2)
    float a0 = 1.0f;
    float a1 = -2.0f * cos_w0;
    float a2 = 1.0f;
    float b1 = -2.0f * r * cos_w0;
    float b2 = r * r;
    
    float output = (a0 * input + a1 * x_n1 + a2 * x_n2 - b1 * y_n1 - b2 * y_n2) / a0;
    
    return output;
}

float EMGProcessor::movingAverage(float input, float* buffer, uint8_t& index) {
    // Update buffer
    buffer[index] = input;
    index = (index + 1) % MAVG_SIZE;
    
    // Calculate average
    float sum = 0;
    for (int i = 0; i < MAVG_SIZE; i++) {
        sum += buffer[i];
    }
    
    return sum / MAVG_SIZE;
}

// ============================================
// Envelope Detection
// ============================================

void EMGProcessor::updateEnvelope(float filtered_ch1, float filtered_ch2) {
    // Rectify (absolute value)
    float rect_ch1 = fabsf(filtered_ch1);
    float rect_ch2 = fabsf(filtered_ch2);
    
    // Low-pass filter for envelope (smooth rectified signal)
    const float envelope_alpha = 0.9f;  // Time constant ~10ms at 1000Hz
    
    _envelope_ch1 = envelope_alpha * _envelope_ch1 + (1.0f - envelope_alpha) * rect_ch1;
    _envelope_ch2 = envelope_alpha * _envelope_ch2 + (1.0f - envelope_alpha) * rect_ch2;
}

// ============================================
// Feature Extraction
// ============================================

EMGFeatures EMGProcessor::extractFeatures(float* data, uint16_t length) {
    EMGFeatures features;
    
    // Mean Absolute Value (MAV)
    float sum_abs = 0;
    for (uint16_t i = 0; i < length; i++) {
        sum_abs += fabsf(data[i]);
    }
    features.mav = sum_abs / length;
    
    // Root Mean Square (RMS)
    float sum_sq = 0;
    for (uint16_t i = 0; i < length; i++) {
        sum_sq += data[i] * data[i];
    }
    features.rms = sqrtf(sum_sq / length);
    
    // Waveform Length (WL)
    float wl = 0;
    for (uint16_t i = 1; i < length; i++) {
        wl += fabsf(data[i] - data[i-1]);
    }
    features.wl = wl;
    
    // Zero Crossing (ZC)
    uint16_t zc = 0;
    for (uint16_t i = 1; i < length; i++) {
        if ((data[i] * data[i-1]) < 0) {
            zc++;
        }
    }
    features.zc = zc;
    
    // Slope Sign Changes (SSC)
    uint16_t ssc = 0;
    for (uint16_t i = 2; i < length; i++) {
        float diff1 = data[i-1] - data[i-2];
        float diff2 = data[i] - data[i-1];
        if ((diff1 * diff2) < 0) {
            ssc++;
        }
    }
    features.ssc = ssc;
    
    // Variance
    float mean = 0;
    for (uint16_t i = 0; i < length; i++) {
        mean += data[i];
    }
    mean /= length;
    
    float var = 0;
    for (uint16_t i = 0; i < length; i++) {
        float diff = data[i] - mean;
        var += diff * diff;
    }
    features.var = var / length;
    
    return features;
}

// ============================================
// Calibration
// ============================================

void EMGProcessor::setCalibration(const CalibrationData& cal) {
    _calibration = cal;
}

float EMGProcessor::getNormalizedCh1() const {
    if (!_calibration.calibrated) return 0.0f;
    
    float range = _calibration.ch1_max - _calibration.ch1_rest;
    if (range < 1.0f) range = 1.0f;
    
    float value = fabsf(_envelope_ch1);
    float normalized = (value - _calibration.ch1_rest) / range;
    
    // Clamp to 0-1 range
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    
    return normalized;
}

float EMGProcessor::getNormalizedCh2() const {
    if (!_calibration.calibrated) return 0.0f;
    
    float range = _calibration.ch2_max - _calibration.ch2_rest;
    if (range < 1.0f) range = 1.0f;
    
    float value = fabsf(_envelope_ch2);
    float normalized = (value - _calibration.ch2_rest) / range;
    
    // Clamp to 0-1 range
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    
    return normalized;
}

// ============================================
// Signal Quality Check
// ============================================

bool EMGProcessor::checkSignalQuality(uint16_t raw_ch1, uint16_t raw_ch2) {
    // Check for saturation
    if (raw_ch1 > EMG_SATURATION_HIGH || raw_ch1 < EMG_SATURATION_LOW ||
        raw_ch2 > EMG_SATURATION_HIGH || raw_ch2 < EMG_SATURATION_LOW) {
        return false;
    }
    
    // Check for disconnected sensor (stuck at same value)
    static uint16_t prev_ch1 = 0;
    static uint16_t prev_ch2 = 0;
    static uint16_t stuck_count = 0;
    
    if (raw_ch1 == prev_ch1 && raw_ch2 == prev_ch2) {
        stuck_count++;
        if (stuck_count > 100) {  // Stuck for 100ms
            return false;
        }
    } else {
        stuck_count = 0;
    }
    
    prev_ch1 = raw_ch1;
    prev_ch2 = raw_ch2;
    
    return true;
}
