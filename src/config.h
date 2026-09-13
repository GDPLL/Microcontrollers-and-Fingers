/*
 * ESP32 Sixth Finger Prosthetic Control - Configuration
 * Author: Qoder
 * Date: 2025
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================
// Hardware Pin Configuration
// ============================================

// EMG Sensor Pins (2 channels)
#define EMG_CH1_PIN     4      // ADC1_CH6 - Flexor muscle
#define EMG_CH2_PIN     5      // ADC1_CH7 - Extensor muscle

// Servo Pins
#define SERVO_PIN       6      // PWM capable pin for servo

// LED Indicators (optional)
#define LED_STATUS      2       // Built-in LED on most ESP32 boards
#define LED_CH1_ACTIVE  7      // Channel 1 activation indicator
#define LED_CH2_ACTIVE  8      // Channel 2 activation indicator

// Button Pins (for calibration/mode)
#define BUTTON_CAL      0       // Boot button for calibration
#define BUTTON_MODE     9       // Mode switch button

// ============================================
// EMG Signal Processing Parameters
// ============================================

// Sampling parameters
#define SAMPLING_RATE   1000    // Hz - EMG sampling rate
#define SAMPLE_INTERVAL_US  1000    // Microseconds between samples (1000Hz)

// Window parameters for feature extraction
#define WINDOW_SIZE     100     // 100ms window at 1000Hz
#define STEP_SIZE       50      // 50% overlap

// Filter parameters
#define FILTER_LOWCUT   10      // Hz - High-pass filter cutoff
#define FILTER_HIGHCUT  450     // Hz - Low-pass filter cutoff
#define NOTCH_FREQ      50      // Hz - Power line interference

// Moving average filter size
#define MAVG_SIZE       10

// ============================================
// Servo Control Parameters
// ============================================

// Servo angle limits
#define SERVO_MIN_ANGLE 0       // Minimum angle (fully flexed)
#define SERVO_MAX_ANGLE 180     // Maximum angle (fully extended)
#define SERVO_REST_ANGLE 90     // Rest position

// Smooth motion parameters
#define SERVO_SMOOTHING_FACTOR  0.15f   // Lower = smoother but slower
#define SERVO_MIN_UPDATE_MS     20      // Minimum update interval (ms)

// Dead zone to prevent jitter
#define SERVO_DEAD_ZONE     2       // Degrees

// ============================================
// Control Mode Parameters
// ============================================

// Control modes
enum ControlMode {
    MODE_DIRECT = 0,        // Direct EMG to servo mapping
    MODE_THRESHOLD = 1,     // Threshold-based on/off control
    MODE_PROPORTIONAL = 2,  // Proportional control with gain
    MODE_GESTURE = 3        // ML-based gesture recognition
};

#define DEFAULT_CONTROL_MODE    MODE_PROPORTIONAL

// Threshold mode parameters
#define THRESHOLD_CH1_ON    300     // Channel 1 activation threshold
#define THRESHOLD_CH1_OFF   200     // Channel 1 deactivation threshold
#define THRESHOLD_CH2_ON    300     // Channel 2 activation threshold
#define THRESHOLD_CH2_OFF   200     // Channel 2 deactivation threshold

// Proportional control parameters
#define PROP_GAIN_CH1       0.5f    // Channel 1 gain
#define PROP_GAIN_CH2       0.5f    // Channel 2 gain
#define PROP_DEADZONE       50      // EMG deadzone

// ============================================
// Calibration Parameters
// ============================================

#define CALIBRATION_SAMPLES 500     // Number of samples for calibration
#define CALIBRATION_TIMEOUT 10000   // Calibration timeout (ms)

// ============================================
// Communication Parameters
// ============================================

#define SERIAL_BAUDRATE     115200
#define BLE_ENABLED         false   // Set to true to enable BLE

// Data output format for serial plotter
#define OUTPUT_RAW          true    // Output raw EMG values
#define OUTPUT_FILTERED     true    // Output filtered values
#define OUTPUT_FEATURES     false   // Output extracted features
#define OUTPUT_ANGLE        true    // Output servo angle

// ============================================
// Safety Parameters
// ============================================

#define WATCHDOG_TIMEOUT    1000    // ms - Reset if no valid signal
#define MAX_SERVO_SPEED     100     // degrees per second
#define EMG_SATURATION_HIGH 4000    // ADC saturation high
#define EMG_SATURATION_LOW  100     // ADC saturation low

// ============================================
// Debug Configuration
// ============================================

#define DEBUG_LEVEL         2       // 0=none, 1=error, 2=info, 3=debug
#define DEBUG_EMG           true
#define DEBUG_SERVO         true
#define DEBUG_TIMING        false

// ============================================
// Structure Definitions
// ============================================

// Calibration data structure
struct CalibrationData {
    uint16_t ch1_rest;      // Resting value for channel 1
    uint16_t ch2_rest;      // Resting value for channel 2
    uint16_t ch1_max;       // Maximum contraction for channel 1
    uint16_t ch2_max;       // Maximum contraction for channel 2
    bool calibrated;        // Calibration status
    
    CalibrationData() : ch1_rest(2048), ch2_rest(2048), 
                        ch1_max(3000), ch2_max(3000), 
                        calibrated(false) {}
};

// EMG features structure
struct EMGFeatures {
    float mav;          // Mean Absolute Value
    float rms;          // Root Mean Square
    float wl;           // Waveform Length
    uint16_t zc;        // Zero Crossing count
    uint16_t ssc;       // Slope Sign Changes
    float var;          // Variance
};

// Control state structure
struct ControlState {
    ControlMode mode;
    int targetAngle;
    int currentAngle;
    bool ch1Active;
    bool ch2Active;
    uint32_t lastUpdate;
    
    ControlState() : mode(DEFAULT_CONTROL_MODE), targetAngle(90),
                     currentAngle(90), ch1Active(false), 
                     ch2Active(false), lastUpdate(0) {}
};

#endif // CONFIG_H
