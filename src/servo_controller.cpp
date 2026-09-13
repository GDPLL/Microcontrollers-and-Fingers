/*
 * ESP32 Sixth Finger - Servo Controller Implementation
 * Author: Qoder
 * Date: 2025
 */

#include "servo_controller.h"

// ============================================
// Constructor
// ============================================

ServoController::ServoController()
    : _currentAngle(SERVO_REST_ANGLE)
    , _targetAngle(SERVO_REST_ANGLE)
    , _smoothingFactor(SERVO_SMOOTHING_FACTOR)
    , _enabled(false)
    , _lastUpdateTime(0)
{
}

// ============================================
// Initialization
// ============================================

bool ServoController::begin() {
    // Configure PWM timer for ESP32Servo library
    //ESP32PWM::allocateTimer(0);
    //ESP32PWM::allocateTimer(1);
    //ESP32PWM::allocateTimer(2);
    //ESP32PWM::allocateTimer(3);
    
    // Attach servo to pin
    _servo.setPeriodHertz(50);  // Standard 50Hz servo
    _servo.attach(SERVO_PIN);  // Min/max pulse width in microseconds
    
    // Move to rest position
    _currentAngle = SERVO_REST_ANGLE;
    _targetAngle = SERVO_REST_ANGLE;
    _servo.write(_currentAngle);
    
    _enabled = true;
    _lastUpdateTime = millis();
    
    Serial.println("Servo controller initialized");
    Serial.printf("Rest position: %d degrees\n", SERVO_REST_ANGLE);
    
    return true;
}

// ============================================
// Angle Control
// ============================================

void ServoController::setTargetAngle(int angle) {
    _targetAngle = constrainAngle(angle);
}

void ServoController::update() {
    if (!_enabled) return;
    
    uint32_t currentTime = millis();
    
    // Limit update rate
    if (currentTime - _lastUpdateTime < SERVO_MIN_UPDATE_MS) {
        return;
    }
    _lastUpdateTime = currentTime;
    
    // Apply smoothing
    int newAngle = smoothAngle(_targetAngle, _currentAngle);
    
    // Only update if angle changed significantly
    if (abs(newAngle - _currentAngle) > SERVO_DEAD_ZONE) {
        _currentAngle = newAngle;
        _servo.write(_currentAngle);
    }
}

// ============================================
// Utility Functions
// ============================================

int ServoController::constrainAngle(int angle) {
    if (angle < SERVO_MIN_ANGLE) return SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) return SERVO_MAX_ANGLE;
    return angle;
}

int ServoController::smoothAngle(int target, int current) {
    // Exponential smoothing
    float smoothed = _smoothingFactor * target + (1.0f - _smoothingFactor) * current;
    return (int)round(smoothed);
}

void ServoController::goToRest() {
    setTargetAngle(SERVO_REST_ANGLE);
    // Force immediate update
    _currentAngle = SERVO_REST_ANGLE;
    if (_enabled) {
        _servo.write(_currentAngle);
    }
}

void ServoController::emergencyStop() {
    _targetAngle = _currentAngle;  // Stop at current position
    _enabled = false;
    _servo.detach();  // Release servo
    Serial.println("EMERGENCY STOP - Servo disabled");
}

void ServoController::setSmoothingFactor(float factor) {
    if (factor < 0.0f) factor = 0.0f;
    if (factor > 1.0f) factor = 1.0f;
    _smoothingFactor = factor;
}

void ServoController::enable(bool enabled) {
    _enabled = enabled;
    if (enabled && !_servo.attached()) {
        _servo.attach(SERVO_PIN);
    } else if (!enabled && _servo.attached()) {
        _servo.detach();
    }
}
