#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <driver/mcpwm.h>
#include <driver/pcnt.h>
#include "config.h"
#include "esp_timer.h"
#include "soc/mcpwm_periph.h"
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "soc/mcpwm_periph.h"
#include "math.h"

// --- Global State ---
WebServer server(80); 
SystemState currentState = IDLE;
Target targets[20];
int activeTargetCount = 0;
//char command = ' ';
//int32_t commandSteps=0;
//int32_t totalDistanceMM=0;
uint32_t totalSteps=0;
bool motorDirection=true;

// High-precision position tracking (Handles the 90cm travel)
uint32_t overflowCount = 0; 
uint32_t currentSteps = 0;
static uint16_t last_hw_count = 0;

// Solenoid Timer Handle
esp_timer_handle_t solenoid_timer;
// --- 1. Solenoid "Off" Interrupt Handler ---
// This runs in IRAM to ensure the 40ms pulse is exact regardless of WiFi activity

void IRAM_ATTR onSolenoidTimerTimeout(void* arg) {
    digitalWrite(SOLENOID_PIN, LOW);
}

// --- Function Prototypes ---
void initHardware();
void initPCNT();
void returnHome();
void handleNewTargets();
void moveWithRamp(uint32_t steps, bool forward);
void stopMovement();
void updatePosition();
void fireSolenoid();

void setup() {
    Serial.begin(115200);
    initHardware();

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting WiFi");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nIP: " + WiFi.localIP().toString());

    server.on("/run", HTTP_POST, handleNewTargets);
    server.begin();

    // Set software zero at current boot position (Temp until Limit Switch arrives)
    overflowCount = 0;
    pcnt_counter_clear(PCNT_UNIT_0);
    currentSteps = 0;
    Serial.println("System Ready. Carriage is at ZERO.");
          //digitalWrite(5, LOW);
          //delay(1000);
          //digitalWrite(5, HIGH);

}
void foo() {

}
void loop() {
    server.handleClient(); // Keep the web server alive

    switch (currentState) {
        case IDLE:
            // Waiting for the "curl" command to set targets and change state
            break;

        case MOVE_WITH_TARGETS:
            Serial.println("Ramping Motor ...");
            moveWithRamp(totalSteps, true); 
            // Once moveWithRamp returns, we are DONE moving forward
            currentState = RETURNING;
            break;

        case MOVE:
            Serial.printf("Ramping Motor for move by %d steps ...",totalSteps);
            moveWithRamp(totalSteps, motorDirection); 
            // Once moveWithRamp returns, we are DONE moving forward
            currentState = IDLE;
            break;

        case RETURNING:
            Serial.println("Initiating Return Journey...");
            returnHome(); 
            
            currentState = IDLE;
            Serial.println("Sequence Complete. System IDLE.");
            break;
    }
}

void updatePosition() {
    int16_t current_hw_count = 0;
    
    pcnt_get_counter_value(PCNT_UNIT_0, &current_hw_count);
    //Serial.printf("current_hw_count=%d,overflowCount=%d\n",current_hw_count,overflowCount);
    // Hardware jump detection: 32767 -> 0
    // The gap is ~32768. 
    // We check if the difference is "huge and negative"
    // Calculate the difference using signed math to catch the jump
    int16_t diff = current_hw_count - last_hw_count;

    if (diff < -20000) {
        //Serial.printf("positive overflow overflow=%dlast=%d, current=%d\n",overflowCount,last_hw_count,current_hw_count);
        overflowCount += 32768; 
    } 
    // If you ever move backwards: 0 -> 32767
    else if (diff > 20000) {
        // Reverse jump: 0 -> 32767
        // Only subtract if we have something to subtract from
        if (overflowCount >= 32768) {
            overflowCount -= 32768;
        } else {
            overflowCount = 0;
        }
    }
    last_hw_count = current_hw_count;
    //Serial.printf("updatePosition: previous steps=%d current steps=%d\n",currentSteps,overflowCount+current_hw_count);
    currentSteps = overflowCount + (uint16_t)current_hw_count;
}

void fireSolenoid() {
    digitalWrite(SOLENOID_PIN, HIGH);
    esp_timer_start_once(solenoid_timer, 40000); // 40ms in microseconds
}

void resetPosition() {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    
    // You MUST zero all three of these
    overflowCount = 0;
    currentSteps = 0;
    last_hw_count = 0; // If you forget this, the next move "jumps" 32k
    
    pcnt_counter_resume(PCNT_UNIT_0);
}

void returnHome() {
    updatePosition();
    // Calculate distance based on current position
    //float mmToReturn = (float)currentSteps / STEPS_PER_MM;
    
    if (currentSteps > 0) {
        Serial.printf("Returning Home:  (%d steps)\n", currentSteps);
        moveWithRamp(currentSteps, false); // false = backward
    }

    // Final Zeroing
    resetPosition(); 
    Serial.println("System Re-Zeroed. Ready.");
}

void initHardware() {
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(SOLENOID_PIN, OUTPUT);
    //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, STEP_PIN); 
    // 1. Configure the physical pin as a generic output
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << STEP_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // 2. Manually route MCPWM Unit 0, Generator A (PWM0A) to the STEP_PIN
    // '65' is the internal signal index for MCPWM0_OUT0A on the ESP32-S3
    gpio_matrix_out(STEP_PIN, 65, false, false);
    //   esp_rom_gpio_connect_out_signal(STEP_PIN, mcpwm_periph_signals.groups[0].operators[0].generators[0].pwm_sig_out, false, false);
    initPCNT(); 

    const esp_timer_create_args_t timer_args = {
        .callback = &onSolenoidTimerTimeout,
        .name = "solenoid_off"
    };
    esp_timer_create(&timer_args, &solenoid_timer);

    
    // --- FIXED MCPWM INIT ---
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, STEP_PIN);
    
    mcpwm_config_t pwm_config;
    // CRITICAL: Do NOT use 0 here. Use a placeholder frequency.
    pwm_config.frequency = 1000; 
    pwm_config.cmpr_a = 0;       // 0% duty cycle means no pulses will be sent
    pwm_config.cmpr_b = 0;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    // Ensure it is actually stopped at boot
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0); 
    // ------------------------

    Serial.println("Hardware logic initialized.");
}

void moveWithRamp(uint32_t totalSteps, bool forward) {
    resetPosition();
    Serial.printf("moveWithRamp %u steps, currently %u\n",totalSteps,currentSteps);
    if (forward) {
      Serial.println("set direction forward");
      digitalWrite(DIR_PIN, false);
    } else {
      digitalWrite(DIR_PIN, true);
      Serial.println("set direction reverse");
    }
    //digitalWrite(DIR_PIN, forward ? HIGH : LOW);
    
    if (totalSteps == 0) return;

    float startFreq = 100.0;
    float targetFreq = forward ? (STEPS_PER_MM * FORWARD_VELOCITY_MMS) 
                               : (STEPS_PER_MM * RETURN_VELOCITY_MMS);
    
    // Ramp settings
    int32_t rampRange = min((uint32_t)2000, totalSteps >> 1);
    float currentFreq = startFreq;

    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);

    // THE SYNCED LOOP: Run until currentSteps hits totalSteps
    while (currentSteps < totalSteps) {
        updatePosition(); // Read hardware PCNT
    //Serial.printf(" --- performed %u steps\n",currentSteps);
        
        int32_t progressed = currentSteps;

        // 1. DYNAMIC RAMPING based on ACTUAL position
        if (progressed < rampRange) {
            // Accelerate
            float pct = (float)progressed / rampRange;
            currentFreq = startFreq + (targetFreq - startFreq) * pct;
        } 
        else if (progressed > (totalSteps - rampRange)) {
            // Decelerate
            float pct = (float)(totalSteps - progressed) / rampRange;
            currentFreq = startFreq + (targetFreq - startFreq) * pct;
        } 
        else {
            currentFreq = targetFreq;
        }

        // Safety floor for frequency
        if (currentFreq < startFreq) currentFreq = startFreq;

        mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, (uint32_t)currentFreq);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0);

        // 2. SOLENOID FIRING (Forward only)
        if (forward) {
            for (int t = 0; t < activeTargetCount; t++) {
                if (!targets[t].completed && progressed >= targets[t].fireStep) {
                    targets[t].completed = true;
                    fireSolenoid();
                }
            }
        }

        // 3. BREATHING ROOM
        // This is vital. Without a small delay, this loop runs so fast it 
        // creates massive "bus contention" reading the PCNT.
        delayMicroseconds(100); 
        yield(); 
    }

    stopMovement();
    updatePosition();
    Serial.printf("Target reached: %u steps.\n", currentSteps);
}

void stopMovement() {
    // 1. Force duty to 0 so the pin stays LOW
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
    // 2. Stop the hardware timer
    mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    Serial.println("Movement Stopped.");
}

void handleNewTargets() {
    if (currentState != IDLE) { 
        server.send(400, "text/plain", "Busy"); 
        return; 
    }
    
    String data = server.arg("plain");
    if (data.length() == 0) {
        server.send(400, "text/plain", "Empty payload");
        return;
    }

    activeTargetCount = 0;
    
    // Safety check for math
    if (STEPS_PER_MM <= 0) {
        server.send(500, "text/plain", "Configuration Error: STEPS_PER_MM is 0");
        return;
    }

    float leadSteps = (FORWARD_VELOCITY_MMS * (SOLENOID_LAG_MS / 1000.0)) * STEPS_PER_MM;

    char buf[512];
    data.toCharArray(buf, 512);
    //server.send(200, "text/plain", buf);

    char cmd = toupper(buf[0]);
    if (cmd=='S') {
        int steps = data.substring(1).toInt();
        if (steps > 0) {
          Serial.printf("Forward Steps %d\n", steps);
          totalSteps = steps;
          motorDirection = true; //Forward
          currentState = MOVE;
        }
    } else if (cmd=='B') {
        int steps = data.substring(1).toInt();
        if (steps > 0) {
          Serial.printf("Reverse Steps %d\n", steps);
          totalSteps = steps;
          motorDirection = false; //Reverse
          currentState = MOVE;
        }
    } else if (cmd=='F') {
        int cm = data.substring(1).toInt();
        if (cm > 0) {
          Serial.printf("Forward %dcm\n", cm);
          totalSteps = cm * STEPS_PER_MM * 10;
          motorDirection = true; //Forward
          currentState = MOVE;
        }
    } else if (cmd=='R') {
        int cm = data.substring(1).toInt();
        if (cm > 0) {
          Serial.printf("Reverse %dcm\n", cm);
          totalSteps = cm * STEPS_PER_MM * 10;
          motorDirection = false; //Reverse
          currentState = MOVE;
        }
    } else {
      char* p = strtok(buf, ",");
      while (p != NULL && activeTargetCount < 20) {
          float pos = atof(p);
          Serial.printf("pos = %f\n",pos);
          targets[activeTargetCount].positionMM = pos;
          targets[activeTargetCount].fireStep = (int32_t)(pos * STEPS_PER_MM) - (int32_t)leadSteps;
          targets[activeTargetCount].completed = false;
          activeTargetCount++;
          p = strtok(NULL, ",");
      }
      Serial.printf("activeTargetCount=%d\n",activeTargetCount);

      if (activeTargetCount > 0) {
          server.send(200, "text/plain", "Run Initiated");
          totalSteps = targets[activeTargetCount-1].positionMM * STEPS_PER_MM;
          motorDirection = true; //Forward
          currentState = MOVE_WITH_TARGETS;
      } else {
         server.send(400, "text/plain", "No valid targets found");
      }
    }
}

void initPCNT() {
    pcnt_config_t pcnt_config = {};
    pcnt_config.pulse_gpio_num = PCNT_INPUT_PIN; 
    pcnt_config.ctrl_gpio_num  = DIR_PIN;
    pcnt_config.unit           = PCNT_UNIT_0;
    pcnt_config.channel        = PCNT_CHANNEL_0;
    pcnt_config.pos_mode       = PCNT_COUNT_INC;
    pcnt_config.neg_mode       = PCNT_COUNT_DIS;
    //pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;
    //pcnt_config.lctrl_mode     = PCNT_MODE_REVERSE;
    pcnt_config.lctrl_mode     = PCNT_MODE_KEEP;
    pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;
    
    // Set limits to the absolute 16-bit max
    pcnt_config.counter_h_lim  = 32767;
    pcnt_config.counter_l_lim  = 0;
    //pcnt_config.counter_l_lim  = -32768;
 
    pcnt_unit_config(&pcnt_config);

    // Matrix links
    gpio_matrix_in((gpio_num_t)PCNT_INPUT_PIN, 45, false);
    gpio_matrix_in((gpio_num_t)DIR_PIN, 46, false);
pinMode(DIR_PIN, OUTPUT);
// Force the pin to its maximum current sourcing level
gpio_set_drive_capability((gpio_num_t)DIR_PIN, GPIO_DRIVE_CAP_3); 

// Also do it for the pulse pin
pinMode(STEP_PIN, OUTPUT);
gpio_set_drive_capability((gpio_num_t)STEP_PIN, GPIO_DRIVE_CAP_3);
    // REMOVE: pcnt_isr_register
    // REMOVE: pcnt_intr_enable

    pcnt_counter_clear(PCNT_UNIT_0);
    int16_t pulse_count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &pulse_count);
    Serial.printf("Current Position: %d steps\n", pulse_count);            
    pcnt_counter_resume(PCNT_UNIT_0);
    pcnt_get_counter_value(PCNT_UNIT_0, &pulse_count);
    Serial.printf("Current Position: %d steps\n",pulse_count );            

}
