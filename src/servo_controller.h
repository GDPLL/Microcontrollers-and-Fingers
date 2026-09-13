/*
 * ESP32 Sixth Finger - Servo Controller
 * Author: Qoder
 * Date: 2025
 */

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include "config.h"
#include <ESP32Servo.h>

// ============================================
// Servo Controller Class
// ============================================

class ServoController {
public:
    ServoController();
    
    // Initialize servo
    bool begin();
    
    // Set target angle (0-180 degrees)
    void setTargetAngle(int angle);
    
    // Update servo position (call in main loop)
    void update();
    
    // Get current angle
    int getCurrentAngle() const { return _currentAngle; }
    
    // Get target angle
    int getTargetAngle() const { return _targetAngle; }
    
    // Check if servo has reached target
    bool isAtTarget() const { return abs(_currentAngle - _targetAngle) <= SERVO_DEAD_ZONE; }
    
    // Move to rest position
    void goToRest();
    
    // Emergency stop
    void emergencyStop();
    
    // Set smoothing factor (0.0-1.0)
    void setSmoothingFactor(float factor);
    
    // Enable/disable servo
    void enable(bool enabled);
    bool isEnabled() const { return _enabled; }

private:
    Servo _servo;
    int _currentAngle;
    int _targetAngle;
    float _smoothingFactor;
    bool _enabled;
    uint32_t _lastUpdateTime;
    
    // Constrain angle to valid range
    int constrainAngle(int angle);
    
    // Apply smoothing
    int smoothAngle(int target, int current);
};

#endif // SERVO_CONTROLLER_H
