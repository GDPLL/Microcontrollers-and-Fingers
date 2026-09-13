/*
 * ESP32 Sixth Finger - EMG Signal Processor
 * Author: Qoder
 * Date: 2025
 */

#ifndef EMG_PROCESSOR_H
#define EMG_PROCESSOR_H

#include "config.h"

// ============================================
// EMG Processor Class
// ============================================

class EMGProcessor {
public:
    EMGProcessor();
    
    // Initialize the processor
    void begin();
    
    // Read raw ADC values
    void readRaw(uint16_t& ch1, uint16_t& ch2);
    
    // Apply filters to raw values
    void filter(uint16_t raw_ch1, uint16_t raw_ch2, float& filtered_ch1, float& filtered_ch2);
    
    // Extract features from a window of data
    EMGFeatures extractFeatures(float* data, uint16_t length);
    
    // Get filtered values (after calling filter())
    float getFilteredCh1() const { return _filtered_ch1; }
    float getFilteredCh2() const { return _filtered_ch2; }
    
    // Get envelope (smoothed amplitude)
    float getEnvelopeCh1() const { return _envelope_ch1; }
    float getEnvelopeCh2() const { return _envelope_ch2; }
    
    // Set calibration data
    void setCalibration(const CalibrationData& cal);
    
    // Get normalized values (0.0 to 1.0) based on calibration
    float getNormalizedCh1() const;
    float getNormalizedCh2() const;
    
    // Check for signal quality issues
    bool checkSignalQuality(uint16_t raw_ch1, uint16_t raw_ch2);

private:
    // Filter state variables
    float _filtered_ch1;
    float _filtered_ch2;
    float _prev_raw_ch1;
    float _prev_raw_ch2;
    float _prev_filtered_ch1;
    float _prev_filtered_ch2;
    
    // Envelope tracking
    float _envelope_ch1;
    float _envelope_ch2;
    
    // Moving average buffers
    float _mavg_buffer_ch1[MAVG_SIZE];
    float _mavg_buffer_ch2[MAVG_SIZE];
    uint8_t _mavg_index;
    
    // Calibration
    CalibrationData _calibration;
    
    // High-pass filter (DC removal)
    float highPassFilter(float input, float& prev_input, float& prev_output);
    
    // Low-pass filter (smoothing)
    float lowPassFilter(float input, float prev_output);
    
    // Notch filter (50Hz power line)
    float notchFilter(float input, float& x_n1, float& x_n2, float& y_n1, float& y_n2);
    
    // Moving average filter
    float movingAverage(float input, float* buffer, uint8_t& index);
    
    // Envelope detection
    void updateEnvelope(float filtered_ch1, float filtered_ch2);
};

#endif // EMG_PROCESSOR_H
