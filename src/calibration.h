/*
 * ESP32 Sixth Finger - Calibration Manager
 * Author: Qoder
 * Date: 2025
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "config.h"
#include "emg_processor.h"

// ============================================
// Calibration States
// ============================================

enum CalibrationState {
    CAL_IDLE = 0,
    CAL_PROMPT_REST,
    CAL_RECORDING_REST,
    CAL_PROMPT_CH1,
    CAL_RECORDING_CH1,
    CAL_PROMPT_CH2,
    CAL_RECORDING_CH2,
    CAL_COMPLETE,
    CAL_ERROR
};

// ============================================
// Calibration Manager Class
// ============================================

class CalibrationManager {
public:
    CalibrationManager(EMGProcessor& processor);
    
    // Start calibration process
    void start();
    
    // Stop calibration
    void stop();
    
    // Update calibration state (call in main loop)
    void update();
    
    // Check if calibration is complete
    bool isComplete() const { return _state == CAL_COMPLETE; }
    
    // Check if calibration is in progress
    bool isInProgress() const { return _state != CAL_IDLE && _state != CAL_COMPLETE; }
    
    // Get calibration data
    CalibrationData getCalibrationData() const { return _data; }
    
    // Get current state
    CalibrationState getState() const { return _state; }
    
    // Save calibration to EEPROM/flash
    bool saveCalibration();
    
    // Load calibration from EEPROM/flash
    bool loadCalibration();
    
    // Reset calibration data
    void resetCalibration();

private:
    EMGProcessor& _processor;
    CalibrationData _data;
    CalibrationState _state;
    
    uint32_t _stateStartTime;
    uint32_t _sampleCount;
    float _sum_ch1;
    float _sum_ch2;
    float _max_ch1;
    float _max_ch2;
    
    // State handlers
    void handleIdle();
    void handlePromptRest();
    void handleRecordingRest();
    void handlePromptCh1();
    void handleRecordingCh1();
    void handlePromptCh2();
    void handleRecordingCh2();
    void handleComplete();
    void handleError();
    
    // Transition to next state
    void transitionTo(CalibrationState newState);
    
    // Print calibration instructions
    void printInstructions(const char* instruction);
};

#endif // CALIBRATION_H
