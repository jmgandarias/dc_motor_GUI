/*

Low-level firmware for encoder reading and PWM control of a motor, with FreeRTOS sampling and
serial communication for experiment control and data logging.

This script uses the 3.X version of the Arduino ESP32 core.

Author: Juan M. Gandarias
web: www.jmgandarias.com
email: jmgandarias@uma.es

*/
#include <M5Unified.h>
#include <M5GFX.h>
#include <ArduinoJson.h>

#define ENCODER_A 19 // pin connected to encoder channel A
#define ENCODER_B 27 // pin connected to encoder channel B

#define PWM_CCW_PIN 25 // PWM pin for clockwise drive
#define PWM_CW_PIN 26  // PWM pin for counter-clockwise drive

#define LED_PIN 13 // activity indicator LED

// Touch details struct for reading touch input
m5::touch_detail_t touchDetail;
static int32_t w;
static int32_t h;

// Button object for reset button on LCD
LGFX_Button reset_button;
const int BUTTON_WIDTH = 80;
const int BUTTON_HEIGHT = 40;
const int BUTTON_OFFSET = 5;

volatile bool update_display = true; // flag to indicate display needs update after config change

// JSON buffer size for configuration parsing
static const size_t MAX_LINE = 1024;
String lineBuffer;
String control_mode = "open-loop";
String input_signal = "step";
float experiment_duration = 10.0;
float sampling_rate = 0.001;

const uint16_t SERIAL_COMM_HZ = 100;                          // fixed serial communication rate
const uint32_t SERIAL_COMM_PERIOD_MS = 1000 / SERIAL_COMM_HZ; // 10 ms
const uint16_t SERIAL_BATCH_MAX_SAMPLES = 32;                 // max samples sent per serial batch
const uint16_t SAMPLE_QUEUE_CAPACITY = 512;                   // buffered samples between serial sends

volatile float Kp = 1.0;
volatile float Ki = 0.0;
volatile float Kd = 0.0;
volatile bool dead_zone_compensation = true;

float ref = 0.0f; // reference value for control (e.g., target position or velocity)

// PWM configuration
const int frequency = 10000;               // PWM frequency in Hz
const int resolution = 11;                 // PWM resolution in bits
const int pwm_max = (1 << resolution) - 1; // maximum duty (2048 for 11-bit)
const int voltage_max = 12;                // maximum voltage corresponding to pwm_max (for reference)
const float zero_band_voltage = 0.05f;     // around-zero band mapped to 0V to avoid chatter
const float dead_zone_voltage = 2.0f;      // motor dead-zone compensation threshold in volts
const float zero_band_pwm = (zero_band_voltage / (float)voltage_max) * (float)pwm_max;
const float dead_zone_pwm = (dead_zone_voltage / (float)voltage_max) * (float)pwm_max;

// Encoder counter
volatile int counter = 0;    // running encoder pulse count (signed)
volatile int last_count = 0; // previous sample's count for delta calculation

// Store the last time
volatile uint32_t last_time = 0; // last sample time in ms (as returned by millis())

// Convert to radians
const float pulses_per_revolution = 4400.0;                     // encoder pulses per motor revolution
const float radians_per_pulse = 2 * PI / pulses_per_revolution; // conversion factor pulses -> radians

// Velocity moving-average filter (FIR)
// Keep an approximately constant time horizon: window_samples ~= horizon / sampling_rate.
// Examples: sampling_rate=0.01 -> 5 samples, sampling_rate=0.001 -> 50 samples.
const float VEL_MA_HORIZON_S = 0.05f; // 50 ms smoothing horizon
const int VEL_MA_WINDOW_MAX = 50;     // upper bound for memory and CPU usage
volatile int VEL_MA_WINDOW = 5;       // effective window used by the filter
float vel_ma_buf[VEL_MA_WINDOW_MAX];
int vel_ma_idx = 0;
int vel_ma_count = 0;
float vel_ma_sum = 0.0f;

// Experiment control
bool experiment_running = false; // true when experiment is running
uint32_t t_ini = 0;

// Task handles for FreeRTOS tasks
TaskHandle_t control_task_handle = NULL; // handle for control task
TaskHandle_t config_task_handle = NULL;  // handle for configuration task
TaskHandle_t display_task_handle = NULL; // handle for display update task

// Global position, velocity, and PWM variables
int pwm = 0;               // current PWM value
float pos = 0.0f;          // current position in radians
float vel = 0.0f;          // current velocity in rad/s
float vel_filtered = 0.0f; // filtered velocity
uint32_t current_time = 0; // current experiment time in ms

typedef struct
{
    float power;
    float pos;
    float vel;
    float vel_filtered;
    float ref;
    uint32_t time_ms;
} SampleFrame;

SampleFrame sample_queue[SAMPLE_QUEUE_CAPACITY];
volatile uint16_t sample_queue_head = 0;
volatile uint16_t sample_queue_tail = 0;
volatile uint16_t sample_queue_count = 0;
volatile uint32_t sample_queue_dropped = 0;
portMUX_TYPE sample_queue_mux = portMUX_INITIALIZER_UNLOCKED;

// Encoder ISRs --------------------------------------------------------------
// Simple quadrature decode: compare A and B to increment/decrement counter.
// Keep ISRs minimal to avoid delays inside interrupt context.
void IRAM_ATTR ISRENCODER_A()
{
    if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B))
    {
        counter++; // same level -> forward step
    }
    else
    {
        counter--; // different -> backward step
    }
}

void IRAM_ATTR ISRENCODER_B()
{
    if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B))
    {
        counter--; // opposite direction relative to channel B change
    }
    else
    {
        counter++;
    }
}

void resetVelocityFilterState()
{
    vel_ma_idx = 0;
    vel_ma_count = 0;
    vel_ma_sum = 0.0f;
    for (int i = 0; i < VEL_MA_WINDOW_MAX; ++i)
    {
        vel_ma_buf[i] = 0.0f;
    }
}

void updateVelocityFilterWindowFromSamplingRate()
{
    float sr = sampling_rate;
    if (sr <= 1e-6f)
    {
        sr = 0.001f;
    }

    int w = (int)((VEL_MA_HORIZON_S / sr) + 0.5f); // rounded to nearest int
    if (w < 1)
    {
        w = 1;
    }
    else if (w > VEL_MA_WINDOW_MAX)
    {
        w = VEL_MA_WINDOW_MAX;
    }

    VEL_MA_WINDOW = w;
    resetVelocityFilterState();
}

uint32_t getControlPeriodMsFromSamplingRate()
{
    float sr = sampling_rate;
    if (sr <= 1e-6f)
    {
        sr = 0.001f;
    }

    // Round to nearest ms instead of truncating so 0.01 s maps to 10 ms, not 9 ms.
    uint32_t period_ms = (uint32_t)((sr * 1000.0f) + 0.5f);
    if (period_ms < 1)
    {
        period_ms = 1;
    }
    return period_ms;
}

void resetSampleQueue()
{
    portENTER_CRITICAL(&sample_queue_mux);
    sample_queue_head = 0;
    sample_queue_tail = 0;
    sample_queue_count = 0;
    sample_queue_dropped = 0;
    portEXIT_CRITICAL(&sample_queue_mux);
}

void enqueueSample(float power, float pos_value, float vel_value, float vel_filtered_value, float ref_value, uint32_t time_ms)
{
    portENTER_CRITICAL(&sample_queue_mux);

    if (sample_queue_count >= SAMPLE_QUEUE_CAPACITY)
    {
        // Drop oldest sample when queue is full to preserve the most recent ones.
        sample_queue_tail = (sample_queue_tail + 1) % SAMPLE_QUEUE_CAPACITY;
        sample_queue_count--;
        sample_queue_dropped++;
    }

    sample_queue[sample_queue_head].power = power;
    sample_queue[sample_queue_head].pos = pos_value;
    sample_queue[sample_queue_head].vel = vel_value;
    sample_queue[sample_queue_head].vel_filtered = vel_filtered_value;
    sample_queue[sample_queue_head].ref = ref_value;
    sample_queue[sample_queue_head].time_ms = time_ms;

    sample_queue_head = (sample_queue_head + 1) % SAMPLE_QUEUE_CAPACITY;
    sample_queue_count++;
    portEXIT_CRITICAL(&sample_queue_mux);
}

int dequeueSampleBatch(SampleFrame *out_frames, int max_frames)
{
    if (max_frames <= 0)
    {
        return 0;
    }

    int count = 0;
    portENTER_CRITICAL(&sample_queue_mux);
    while (sample_queue_count > 0 && count < max_frames)
    {
        out_frames[count] = sample_queue[sample_queue_tail];
        sample_queue_tail = (sample_queue_tail + 1) % SAMPLE_QUEUE_CAPACITY;
        sample_queue_count--;
        count++;
    }
    portEXIT_CRITICAL(&sample_queue_mux);

    return count;
}

uint32_t consumeDroppedSampleCount()
{
    uint32_t dropped = 0;
    portENTER_CRITICAL(&sample_queue_mux);
    dropped = sample_queue_dropped;
    sample_queue_dropped = 0;
    portEXIT_CRITICAL(&sample_queue_mux);
    return dropped;
}

float applyDeadZoneCompensation(float u)
{
    if (!dead_zone_compensation)
    {
        return u;
    }

    if (u >= -zero_band_pwm && u <= zero_band_pwm)
    {
        return 0.0f;
    }
    else if (u > 0.0f && u < dead_zone_pwm)
    {
        return dead_zone_pwm;
    }
    else if (u < 0.0f && u > -dead_zone_pwm)
    {
        return -dead_zone_pwm;
    }
    return u;
}

void finishExperiment()
{
    // End of experiment: stop motor and reset state
    ledcWrite(PWM_CW_PIN, 0);
    ledcWrite(PWM_CCW_PIN, 0);
    Serial.println("END");
    digitalWrite(LED_PIN, LOW);
    experiment_running = false;
    pwm = 0;
    last_count = 0;
    last_time = 0;
    counter = 0;
    resetSampleQueue();

    // Reset moving-average filter state
    resetVelocityFilterState();
}

void setup()
{
    M5.begin();
    Serial.begin(500000); // high baud for fast data logging
    updateVelocityFilterWindowFromSamplingRate();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Configure PWM using ledcAttach (3.X core version)
    ledcAttach(PWM_CW_PIN, frequency, resolution);
    ledcAttach(PWM_CCW_PIN, frequency, resolution);

    // Configure encoder inputs with internal pullups
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    // Attach encoder ISRs: CHANGE to catch both edges
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), ISRENCODER_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), ISRENCODER_B, CHANGE);

    // Create FreeRTOS tasks for control loop, configuration handling, and display updates
    xTaskCreatePinnedToCore(
        ControlLoopTask,      // Task function
        "Control loop task",  // Task description
        8192,                 // Stack size in words
        NULL,                 // Parameters
        1,                    // Priority
        &control_task_handle, // Task handle
        1);                   // Core ID

    xTaskCreatePinnedToCore(
        ConfigAndCommTask,               // Task function
        "Config and communication task", // Task description
        8192,                            // Stack size in words
        NULL,                            // Parameters
        1,                               // Priority
        &config_task_handle,             // Task handle
        0);                              // Core ID

    xTaskCreatePinnedToCore(
        DisplayTask,          // Task function
        "Display Task",       // Task description
        8192,                 // Stack size in words
        NULL,                 // Parameters
        1,                    // Priority
        &display_task_handle, // Task handle
        0);                   // Core ID

    Serial.println("READY"); // <-- announce we are alive and ready
}

void loop()
{
    vTaskDelete(NULL);
}

void ControlLoopTask(void *parameter)
{
    TickType_t xPeriod = pdMS_TO_TICKS(getControlPeriodMsFromSamplingRate());
    TickType_t xLastWakeTime = xTaskGetTickCount();                    // initialize once

    float error = 0.0f;            // error for position control
    float integral_error = 0.0f;   // integral term
    float derivative_error = 0.0f; // derivative term
    float actuation = 0.0f;        // PID control output
    float error_prev = 0.0f;       // initialize previous error

    // This task can be used for more complex control algorithms if needed
    while (true)
    {
        xPeriod = pdMS_TO_TICKS(getControlPeriodMsFromSamplingRate());

        // Start experiment when flag is set by ConfigAndCommTask upon receiving "START" command
        if (experiment_running)
        {
            // Compute increments since last sample (counts)
            long delta_counts = counter - last_count;

            // Get current experiment time in ms
            current_time = millis() - t_ini;

            // elapsed time in seconds since last sample (used for velocity)
            float elapsed_time = (current_time - last_time) / 1000.0f; // convert ms to seconds

            // Calculate position in radians (total)
            pos = counter * radians_per_pulse;

            // Calculate velocity (rad/s) derived from counts:
            // 1) compute counts per second = delta_counts / elapsed_time
            // 2) convert to rad/s by multiplying by radians_per_pulse
            vel = 0.0f;
            vel_filtered = 0.0f;

            if (elapsed_time > 1e-6f)
            {
                float counts_per_sec = (float)delta_counts / elapsed_time;
                vel = counts_per_sec * radians_per_pulse;

                // Moving average filter over last VEL_MA_WINDOW velocity samples
                int window = VEL_MA_WINDOW;
                if (window < 1)
                {
                    window = 1;
                }
                else if (window > VEL_MA_WINDOW_MAX)
                {
                    window = VEL_MA_WINDOW_MAX;
                }
                vel_ma_sum -= vel_ma_buf[vel_ma_idx];
                vel_ma_buf[vel_ma_idx] = vel;
                vel_ma_sum += vel;
                vel_ma_idx = (vel_ma_idx + 1) % window;
                if (vel_ma_count < window)
                {
                    vel_ma_count++;
                }
                vel_filtered = vel_ma_sum / (float)vel_ma_count;
            }

            if (control_mode == "position")
            {
                if (input_signal == "step")
                {
                    if ((current_time < experiment_duration * 1000.0 / 2))
                    {
                        ref = 0.0f; // no power step input
                    }
                    else if ((current_time >= experiment_duration * 1000.0 / 2))
                    {
                        ref = PI; // step to half revolution (pi radians)
                    }
                    else if (current_time > experiment_duration * 1000.0)
                    {
                        finishExperiment();
                        error = 0.0f;            // error for position control
                        integral_error = 0.0f;   // integral term
                        derivative_error = 0.0f; // derivative term
                        actuation = 0.0f;        // PID control output
                        error_prev = 0.0f;       // initialize previous error
                    }
                }
                else if (input_signal == "ramp")
                {
                    // ramp input logic here (e.g., read from serial or buttons)
                    if (current_time <= experiment_duration * 1000.0)
                    {
                        ref = current_time / (experiment_duration * 1000.0) * 2 * PI; // linear ramp from 0 to 2*pi radians over experiment duration
                    }
                    else
                    {
                        finishExperiment();
                        error = 0.0f;            // error for position control
                        integral_error = 0.0f;   // integral term
                        derivative_error = 0.0f; // derivative term
                        actuation = 0.0f;        // PID control output
                        error_prev = 0.0f;       // initialize previous error
                    }
                }
                else if (input_signal == "manual")
                {
                    if (current_time > experiment_duration * 1000.0)
                    {
                        finishExperiment();
                        error = 0.0f;            // error for position control
                        integral_error = 0.0f;   // integral term
                        derivative_error = 0.0f; // derivative term
                        actuation = 0.0f;        // PID control output
                        error_prev = 0.0f;       // initialize previous error
                    }
                }

                // Position control using PID
                error = ref - pos;                                                          // error for position control
                integral_error = integral_error + error;                                    // integral term
                derivative_error = error - error_prev;                                      // derivative term
                actuation = (Kp * error) + (Ki * integral_error) + (Kd * derivative_error); // PID control output
                actuation = applyDeadZoneCompensation(actuation);                           // compensate motor dead-zone (+/-2V)
                pwm = actuation;                                                            // Storage of actuation value for sending via serial
                error_prev = error;                                                         // store error for next iteration

                if (actuation >= 0)
                {                              // CW direction
                    ledcWrite(PWM_CCW_PIN, 0); // CCW PWM to 0
                    if ((actuation > pwm_max))
                    { // Saturate at max PWM value
                        ledcWrite(PWM_CW_PIN, pwm_max);
                    }
                    else
                    {
                        ledcWrite(PWM_CW_PIN, actuation);
                    }
                }
                else
                {                             // CCW direction
                    actuation = -actuation;   // change sign of actuation signal
                    ledcWrite(PWM_CW_PIN, 0); // ensure CW PWM is set to 0
                    if ((actuation > pwm_max))
                    { // Saturate at max PWM value
                        ledcWrite(PWM_CCW_PIN, pwm_max);
                    }
                    else
                    {
                        ledcWrite(PWM_CCW_PIN, actuation);
                    }
                }
                ledcWrite(LED_PIN, actuation); // update LED brightness based on actuation value
            }
            else if (control_mode == "velocity")
            {
                if (input_signal == "step")
                {
                    if ((current_time < experiment_duration * 1000.0 / 2))
                    {
                        ref = 0.0f; // no power step input
                    }
                    else if ((current_time >= experiment_duration * 1000.0 / 2))
                    {
                        ref = PI; // step to half revolution (pi radians)
                    }
                    else if (current_time > experiment_duration * 1000.0)
                    {
                        finishExperiment();
                        error = 0.0f;            // error for velocity control
                        integral_error = 0.0f;   // integral term
                        derivative_error = 0.0f; // derivative term
                        actuation = 0.0f;        // PID control output
                        error_prev = 0.0f;       // initialize previous error
                    }
                }
                else if (input_signal == "ramp")
                {
                    // ramp input logic here (e.g., read from serial or buttons)
                    if (current_time <= experiment_duration * 1000.0)
                    {
                        ref = current_time / (experiment_duration * 1000.0) * 2 * PI; // linear ramp from 0 to 2*pi radians over experiment duration
                    }
                    else
                    {
                        finishExperiment();
                        error = 0.0f;            // error for velocity control
                        integral_error = 0.0f;   // integral term
                        derivative_error = 0.0f; // derivative term
                        actuation = 0.0f;        // PID control output
                        error_prev = 0.0f;       // initialize previous error
                    }
                }
                else if (input_signal == "manual")
                {
                    if (current_time > experiment_duration * 1000.0)
                    {
                        finishExperiment();
                        error = 0.0f;            // error for velocity control
                        integral_error = 0.0f;   // integral term
                        derivative_error = 0.0f; // derivative term
                        actuation = 0.0f;        // PID control output
                        error_prev = 0.0f;       // initialize previous error
                    }
                }

                // Velocity control using PID
                error = ref - vel_filtered;                                                 // error for velocity control
                integral_error = integral_error + error;                                    // integral term
                derivative_error = error - error_prev;                                      // derivative term
                actuation = (Kp * error) + (Ki * integral_error) + (Kd * derivative_error); // PID control output
                actuation = applyDeadZoneCompensation(actuation);                           // compensate motor dead-zone (+/-2V)
                pwm = actuation;                                                            // Storage of actuation value for sending via serial
                error_prev = error;                                                         // store error for next iteration

                if (actuation >= 0)
                {                              // CW direction
                    ledcWrite(PWM_CCW_PIN, 0); // CCW PWM to 0
                    if ((actuation > pwm_max))
                    { // Saturate at max PWM value
                        ledcWrite(PWM_CW_PIN, pwm_max);
                    }
                    else
                    {
                        ledcWrite(PWM_CW_PIN, actuation);
                    }
                }
                else
                {                             // CCW direction
                    actuation = -actuation;   // change sign of actuation signal
                    ledcWrite(PWM_CW_PIN, 0); // ensure CW PWM is set to 0
                    if ((actuation > pwm_max))
                    { // Saturate at max PWM value
                        ledcWrite(PWM_CCW_PIN, pwm_max);
                    }
                    else
                    {
                        ledcWrite(PWM_CCW_PIN, actuation);
                    }
                }
                ledcWrite(LED_PIN, actuation); // update LED brightness based on actuation value
            }
            else
            {
                if (input_signal == "step")
                {
                    if (current_time < experiment_duration * 1000.0 / 2)
                    {
                        pwm = 0;
                        ledcWrite(PWM_CW_PIN, 0);
                        ledcWrite(PWM_CCW_PIN, 0);
                    }
                    else if (current_time <= experiment_duration * 1000.0)
                    {
                        pwm = pwm_max;              // full power step input
                        ledcWrite(PWM_CW_PIN, pwm); // writing to CW (clockwise)
                        ledcWrite(PWM_CCW_PIN, 0);  // stopping CCW (counter-clockwise)
                    }
                    else
                    {
                        finishExperiment();
                    }
                }
                else if (input_signal == "ramp")
                {
                    // ramp input logic here (e.g., read from serial or buttons)
                    if (current_time <= experiment_duration * 1000.0)
                    {
                        pwm = current_time / (experiment_duration * 1000.0) * pwm_max; // linear ramp from 0 to max over experiment duration
                        ledcWrite(PWM_CW_PIN, pwm);                                    // writing to CW (clockwise)
                        ledcWrite(PWM_CCW_PIN, 0);                                     // stopping CCW (counter-clockwise)
                    }
                    else
                    {
                        finishExperiment();
                    }
                }
                else if (input_signal == "manual")
                {
                    if (current_time <= experiment_duration * 1000.0)
                    {
                        pwm = ref / 100 * pwm_max; // convert reference percentage to PWM value
                        if (ref > 0)
                        {
                            ledcWrite(PWM_CW_PIN, pwm); // writing to CW (counter-clockwise)
                            ledcWrite(PWM_CCW_PIN, 0);  // stopping CCW (clockwise)
                        }
                        else
                        {
                            pwm = -pwm;                 // make positive for PWM output
                            ledcWrite(PWM_CCW_PIN, 0);  // writing to CCW (counter-clockwise)
                            ledcWrite(PWM_CW_PIN, pwm); // stopping CW (clockwise)
                        }
                    }
                    else
                    {
                        finishExperiment();
                    }
                }
            }

            // Update last values for next iteration
            if (experiment_running)
            {
                float power = pwm / (float)pwm_max * voltage_max; // convert PWM to volts
                enqueueSample(power, pos, vel, vel_filtered, ref, current_time);
            }
            last_count = counter;
            last_time = current_time;
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod); // Sleep until next cycle
    }
}

void ConfigAndCommTask(void *parameter)
{
    TickType_t xPeriod = pdMS_TO_TICKS(SERIAL_COMM_PERIOD_MS); // fixed serial communication period (100 Hz)
    TickType_t xLastWakeTime = xTaskGetTickCount();            // initialize once

    // This task can be used for more complex configuration handling if needed
    while (true)
    {
        //  Read serial input (non-blocking)
        while (Serial.available())
        {
            char c = Serial.read();

            if (c == '\n')
            {
                // End of line: process the received command/config
                if (lineBuffer.length() > 0)
                {
                    // Remove any trailing whitespace
                    lineBuffer.trim();

                    // Check if it's a START command
                    if (lineBuffer == "START")
                    {
                        if (!experiment_running)
                        {
                            experiment_running = true;
                            t_ini = millis();
                            Serial.println("STARTED"); // <-- confirm reception
                            digitalWrite(LED_PIN, HIGH);

                            // Initialize timing/count baseline to avoid spurious first-sample velocity
                            last_count = counter; // baseline the last_count to current counter
                            last_time = 0;        // keep behavior consistent: current_time will be small after t_ini

                            // Reset moving-average filter state
                            resetSampleQueue();
                            resetVelocityFilterState();
                        }
                    }
                    // Check if it's an END command
                    else if (lineBuffer == "END")
                    {
                        if (experiment_running)
                        {
                            // End of experiment: stop motor and reset state
                            ledcWrite(PWM_CW_PIN, 0);
                            ledcWrite(PWM_CCW_PIN, 0);
                            Serial.println("END");
                            digitalWrite(LED_PIN, LOW);
                            experiment_running = false;
                            last_count = 0;
                            last_time = 0;
                            counter = 0;

                            // Reset moving-average filter state
                            resetSampleQueue();
                            resetVelocityFilterState();
                        }
                    }
                    // Check if a number
                    else if (lineBuffer.length() >= 3 && lineBuffer.length() <= 6)
                    {
                        if (lineBuffer.toFloat() >= -100.0f && lineBuffer.toFloat() <= 100.0f)
                        {
                            ref = lineBuffer.toFloat();
                            Serial.print("Manual reference set to: ");
                            Serial.println(ref);
                        }
                    }
                    // Otherwise, try to parse as JSON configuration
                    else
                    {
                        DynamicJsonDocument doc(512);
                        DeserializationError err = deserializeJson(doc, lineBuffer);

                        if (err)
                        {
                            Serial.print("JSON Error: ");
                            Serial.println(err.c_str());
                        }
                        else
                        {
                            // Extract the fields using const char* first for strings
                            const char *cm = doc["control_mode"];
                            const char *is = doc["input_signal"];

                            if (cm != nullptr)
                            {
                                control_mode = String(cm);
                            }
                            if (is != nullptr)
                            {
                                input_signal = String(is);
                            }

                            // Extract PID gains (floats)
                            if (doc.containsKey("Kp"))
                            {
                                Kp = doc["Kp"].as<float>();
                            }
                            if (doc.containsKey("Ki"))
                            {
                                Ki = doc["Ki"].as<float>();
                            }
                            if (doc.containsKey("Kd"))
                            {
                                Kd = doc["Kd"].as<float>();
                            }

                            // Extract experiment duration and sampling rate
                            if (doc.containsKey("experiment_duration"))
                            {
                                experiment_duration = doc["experiment_duration"].as<float>();
                            }
                            if (doc.containsKey("sampling_rate"))
                            {
                                // FIX: assign sampling_rate (was mistakenly assigning to Kd)
                                sampling_rate = doc["sampling_rate"].as<float>();
                                updateVelocityFilterWindowFromSamplingRate();
                            }
                            if (doc.containsKey("dead_zone_compensation"))
                            {
                                dead_zone_compensation = doc["dead_zone_compensation"].as<bool>();
                            }

                            Serial.println("Config received:");
                            Serial.print("  control_mode: ");
                            Serial.println(control_mode);
                            Serial.print("  input_signal: ");
                            Serial.println(input_signal);
                            Serial.print("  Kp: ");
                            Serial.println(Kp, 2);
                            Serial.print("  Ki: ");
                            Serial.println(Ki, 2);
                            Serial.print("  Kd: ");
                            Serial.println(Kd, 2);
                            Serial.print("  experiment_duration: ");
                            Serial.println(experiment_duration, 2);
                            Serial.print("  sampling_rate: ");
                            Serial.println(sampling_rate, 3);
                            Serial.print("  vel_ma_window: ");
                            Serial.println(VEL_MA_WINDOW);
                            Serial.print("  dead_zone_compensation: ");
                            Serial.println(dead_zone_compensation ? "true" : "false");

                            // Validate values and set defaults if invalid
                            if (control_mode != "open-loop" && control_mode != "position" && control_mode != "velocity")
                            {
                                Serial.println("Invalid control_mode, using default: open-loop");
                                control_mode = "open-loop"; // default
                            }
                            if (input_signal != "step" && input_signal != "ramp" && input_signal != "manual")
                            {
                                Serial.println("Invalid input_signal, using default: step");
                                input_signal = "step"; // default
                            }
                            update_display = true; // flag to update display with new config
                        }
                    }
                    lineBuffer = ""; // clear for next line
                }
            }
            else
            {
                lineBuffer += c; // keep accumulating
            }
        }

        if (experiment_running)
        {
            SampleFrame batch[SERIAL_BATCH_MAX_SAMPLES];
            int batch_size = dequeueSampleBatch(batch, SERIAL_BATCH_MAX_SAMPLES);

            if (batch_size > 0)
            {
                DynamicJsonDocument out_doc(2048 + (batch_size * 80));
                out_doc["type"] = "DATA_BATCH";
                out_doc["n"] = batch_size;

                uint32_t dropped = consumeDroppedSampleCount();
                if (dropped > 0)
                {
                    out_doc["dropped"] = dropped;
                }

                JsonArray power_arr = out_doc.createNestedArray("power");
                JsonArray pos_arr = out_doc.createNestedArray("pos");
                JsonArray vel_arr = out_doc.createNestedArray("vel");
                JsonArray vel_filtered_arr = out_doc.createNestedArray("vel_filtered");
                JsonArray ref_arr = out_doc.createNestedArray("ref");
                JsonArray time_arr = out_doc.createNestedArray("time_ms");

                for (int i = 0; i < batch_size; ++i)
                {
                    power_arr.add(batch[i].power);
                    pos_arr.add(batch[i].pos);
                    vel_arr.add(batch[i].vel);
                    vel_filtered_arr.add(batch[i].vel_filtered);
                    ref_arr.add(batch[i].ref);
                    time_arr.add(batch[i].time_ms);
                }

                serializeJson(out_doc, Serial);
                Serial.println();
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod); // Sleep until next cycle
    }
}

void DisplayTask(void *parameter)
{
    // This task can be used for more complex display updates if needed
    const TickType_t xPeriod = pdMS_TO_TICKS(50);   // 50 ms period
    TickType_t xLastWakeTime = xTaskGetTickCount(); // initialize once

    while (true)
    {
        // Handle M5 button/touch events
        M5.update();

        // Check for touch input in reset button area
        touchDetail = M5.Touch.getDetail();
        if (touchDetail.isReleased())
        {
            if (reset_button.contains(touchDetail.x, touchDetail.y))
            {
                // Perform hardware reset of the microcontroller
                ESP.restart();
            }
        }

        if (update_display)
        {
            M5.Lcd.fillScreen(BLACK);
            M5.Display.setTextSize(2.5);
            M5.Display.setTextColor(WHITE, BLACK);
            M5.Display.setCursor(80, 10);
            M5.Display.println("DC Motor GUI");

            M5.Display.setTextSize(2);
            M5.Display.setCursor(0, 60);
            M5.Display.println("Control mode: " + control_mode);
            M5.Display.setCursor(0, 90);
            M5.Display.println("Input signal: " + input_signal);
            M5.Display.setCursor(0, 130);

            if (control_mode != "open-loop")
            {
                M5.Display.println("Kp: " + String(Kp, 5));
                M5.Display.setCursor(0, 160);
                M5.Display.println("Ki: " + String(Ki, 5));
                M5.Display.setCursor(0, 190);
                M5.Display.println("Kd: " + String(Kd, 5));
            }

            // Initialize reset button in bottom-right corner of the screen
            w = M5.Lcd.width();
            h = M5.Lcd.height();
            reset_button.initButton(&M5.Lcd, w - BUTTON_WIDTH / 2 - BUTTON_OFFSET, h - BUTTON_HEIGHT / 2 - BUTTON_OFFSET, BUTTON_WIDTH, BUTTON_HEIGHT, TFT_WHITE, TFT_YELLOW, TFT_BLACK, "Reset", 2, 2);
            reset_button.drawButton();

            update_display = false; // reset flag
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod); // Sleep until next cycle
    }
}