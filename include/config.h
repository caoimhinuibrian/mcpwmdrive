#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- WIFI ---
#define WIFI_SSID "TeamhairnaRi"
#define WIFI_PASS "bodhisattva00"

// --- PINS ---
#define STEP_PIN          4   
#define DIR_PIN           10   
#define SOLENOID_PIN      12
#define PCNT_INPUT_PIN 18  // The "Ear" pin
#define LIMIT_SWITCH_PIN 9

// --- MOTION ---
const float MM_PER_REV    = 8.0;
const int STEPS_PER_REV   = 3200; 
const float STEPS_PER_MM  = (float)STEPS_PER_REV / MM_PER_REV;

const float FORWARD_VELOCITY_MMS = 60.0; 
const uint32_t SOLENOID_LAG_MS  = 18;   
const uint32_t SOLENOID_PULSE_MS = 40;  

const uint32_t RAMP_TIME_MS = 300;     
const uint32_t RAMP_INTERVAL_MS = 10;
const float RETURN_VELOCITY_MMS = 60.0; // Keep it same as forward for safety

enum SystemState { IDLE, MOVE_WITH_TARGETS, MOVE, RETURNING };
struct Target {
    float positionMM;
    int32_t fireStep;
    bool completed;
};

#endif
