/*
 * ESP32 Sixth Finger - Calibration Manager Implementation
 * Author: Qoder
 * Date: 2025
 */

#include "calibration.h"
#include <EEPROM.h>

// ============================================
// Constructor
// ============================================

CalibrationManager::CalibrationManager(EMGProcessor& processor)
    : _processor(processor)
    , _state(CAL_IDLE)
    , _stateStartTime(0)
    , _sampleCount(0)
    , _sum_ch1(0)
    , _sum_ch2(0)
    , _max_ch1(0)
    , _max_ch2(0)
{
}

// ============================================
// Control Functions
// ============================================

void CalibrationManager::start() {
    if (_state != CAL_IDLE) {
        Serial.println("Calibration already in progress!");
        return;
    }
    
    Serial.println("\n========================================");
    Serial.println("    CALIBRATION STARTED");
    Serial.println("========================================\n");
    
    transitionTo(CAL_PROMPT_REST);
}

void CalibrationManager::stop() {
    if (_state != CAL_IDLE) {
        Serial.println("\nCalibration stopped by user.");
        _state = CAL_IDLE;
    }
}

void CalibrationManager::resetCalibration() {
    _data = CalibrationData();
    _data.calibrated = false;
    Serial.println("Calibration data reset.");
}

// ============================================
// State Machine
// ============================================

void CalibrationManager::update() {
    switch (_state) {
        case CAL_IDLE:
            handleIdle();
            break;
        case CAL_PROMPT_REST:
            handlePromptRest();
            break;
        case CAL_RECORDING_REST:
            handleRecordingRest();
            break;
        case CAL_PROMPT_CH1:
            handlePromptCh1();
            break;
        case CAL_RECORDING_CH1:
            handleRecordingCh1();
            break;
        case CAL_PROMPT_CH2:
            handlePromptCh2();
            break;
        case CAL_RECORDING_CH2:
            handleRecordingCh2();
            break;
        case CAL_COMPLETE:
            handleComplete();
            break;
        case CAL_ERROR:
            handleError();
            break;
    }
}

// ============================================
// State Handlers
// ============================================

void CalibrationManager::handleIdle() {
    // Do nothing, waiting for start command
}

void CalibrationManager::handlePromptRest() {
    printInstructions(
        "STEP 1/3: REST POSITION\n"
        "Please relax your muscles completely.\n"
        "Press BOOT button when ready..."
    );
    
    // Wait for button press or timeout
    if (digitalRead(BUTTON_CAL) == LOW) {
        delay(200);  // Debounce
        transitionTo(CAL_RECORDING_REST);
    }
}

void CalibrationManager::handleRecordingRest() {
    uint16_t raw_ch1, raw_ch2;
    _processor.readRaw(raw_ch1, raw_ch2);
    
    _sum_ch1 += raw_ch1;
    _sum_ch2 += raw_ch2;
    _sampleCount++;
    
    // Progress indicator
    if (_sampleCount % 50 == 0) {
        int progress = (_sampleCount * 100) / CALIBRATION_SAMPLES;
        Serial.printf("Recording rest position... %d%%\r", progress);
    }
    
    // Check if recording complete
    if (_sampleCount >= CALIBRATION_SAMPLES) {
        _data.ch1_rest = (uint16_t)(_sum_ch1 / _sampleCount);
        _data.ch2_rest = (uint16_t)(_sum_ch2 / _sampleCount);
        
        Serial.printf("\nRest position recorded: CH1=%d, CH2=%d\n", 
                      _data.ch1_rest, _data.ch2_rest);
        
        // Reset counters
        _sampleCount = 0;
        _sum_ch1 = 0;
        _sum_ch2 = 0;
        
        transitionTo(CAL_PROMPT_CH1);
    }
}

void CalibrationManager::handlePromptCh1() {
    printInstructions(
        "STEP 2/3: CHANNEL 1 (FLEXOR) MAX\n"
        "Contract your flexor muscle maximally.\n"
        "Press BOOT button when ready..."
    );
    
    if (digitalRead(BUTTON_CAL) == LOW) {
        delay(200);
        transitionTo(CAL_RECORDING_CH1);
    }
}

void CalibrationManager::handleRecordingCh1() {
    uint16_t raw_ch1, raw_ch2;
    _processor.readRaw(raw_ch1, raw_ch2);
    
    // Track maximum values
    if (raw_ch1 > _max_ch1) _max_ch1 = raw_ch1;
    
    _sampleCount++;
    
    if (_sampleCount % 50 == 0) {
        int progress = (_sampleCount * 100) / CALIBRATION_SAMPLES;
        Serial.printf("Recording CH1 max... %d%% (current max: %.0f)\r", 
                      progress, _max_ch1);
    }
    
    if (_sampleCount >= CALIBRATION_SAMPLES) {
        _data.ch1_max = (uint16_t)_max_ch1;
        
        Serial.printf("\nCH1 max recorded: %d\n", _data.ch1_max);
        
        _sampleCount = 0;
        _max_ch1 = 0;
        
        transitionTo(CAL_PROMPT_CH2);
    }
}

void CalibrationManager::handlePromptCh2() {
    printInstructions(
        "STEP 3/3: CHANNEL 2 (EXTENSOR) MAX\n"
        "Contract your extensor muscle maximally.\n"
        "Press BOOT button when ready..."
    );
    
    if (digitalRead(BUTTON_CAL) == LOW) {
        delay(200);
        transitionTo(CAL_RECORDING_CH2);
    }
}

void CalibrationManager::handleRecordingCh2() {
    uint16_t raw_ch1, raw_ch2;
    _processor.readRaw(raw_ch1, raw_ch2);
    
    if (raw_ch2 > _max_ch2) _max_ch2 = raw_ch2;
    
    _sampleCount++;
    
    if (_sampleCount % 50 == 0) {
        int progress = (_sampleCount * 100) / CALIBRATION_SAMPLES;
        Serial.printf("Recording CH2 max... %d%% (current max: %.0f)\r", 
                      progress, _max_ch2);
    }
    
    if (_sampleCount >= CALIBRATION_SAMPLES) {
        _data.ch2_max = (uint16_t)_max_ch2;
        
        Serial.printf("\nCH2 max recorded: %d\n", _data.ch2_max);
        
        transitionTo(CAL_COMPLETE);
    }
}

void CalibrationManager::handleComplete() {
    _data.calibrated = true;
    _processor.setCalibration(_data);
    
    Serial.println("\n========================================");
    Serial.println("    CALIBRATION COMPLETE!");
    Serial.println("========================================");
    Serial.printf("CH1 Rest: %d, Max: %d\n", _data.ch1_rest, _data.ch1_max);
    Serial.printf("CH2 Rest: %d, Max: %d\n", _data.ch2_rest, _data.ch2_max);
    Serial.println("========================================\n");
    
    // Save to EEPROM
    if (saveCalibration()) {
        Serial.println("Calibration saved to memory.");
    }
    
    transitionTo(CAL_IDLE);
}

void CalibrationManager::handleError() {
    Serial.println("\nCalibration error occurred!");
    Serial.println("Please restart calibration.");
    transitionTo(CAL_IDLE);
}

// ============================================
// Helper Functions
// ============================================

void CalibrationManager::transitionTo(CalibrationState newState) {
    _state = newState;
    _stateStartTime = millis();
}

void CalibrationManager::printInstructions(const char* instruction) {
    static const char* lastInstruction = nullptr;
    
    // Only print if instruction changed
    if (instruction != lastInstruction) {
        Serial.println("\n----------------------------------------");
        Serial.println(instruction);
        Serial.println("----------------------------------------\n");
        lastInstruction = instruction;
    }
}

// ============================================
// EEPROM Storage
// ============================================

bool CalibrationManager::saveCalibration() {
    EEPROM.begin(512);
    
    // Write calibration data
    int addr = 0;
    EEPROM.put(addr, _data);
    
    // Write magic number to indicate valid data
    uint32_t magic = 0xDEADBEEF;
    addr += sizeof(CalibrationData);
    EEPROM.put(addr, magic);
    
    bool success = EEPROM.commit();
    EEPROM.end();
    
    return success;
}

bool CalibrationManager::loadCalibration() {
    EEPROM.begin(512);
    
    // Check magic number
    uint32_t magic;
    int addr = sizeof(CalibrationData);
    EEPROM.get(addr, magic);
    
    if (magic != 0xDEADBEEF) {
        Serial.println("No calibration data found in memory.");
        EEPROM.end();
        return false;
    }
    
    // Load calibration data
    addr = 0;
    EEPROM.get(addr, _data);
    EEPROM.end();
    
    if (_data.calibrated) {
        _processor.setCalibration(_data);
        Serial.println("Calibration loaded from memory.");
        Serial.printf("CH1 Rest: %d, Max: %d\n", _data.ch1_rest, _data.ch1_max);
        Serial.printf("CH2 Rest: %d, Max: %d\n", _data.ch2_rest, _data.ch2_max);
        return true;
    }
    
    return false;
}
