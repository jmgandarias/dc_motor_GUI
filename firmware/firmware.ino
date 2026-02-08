/*

Low-level firmware for encoder reading and PWM control of a motor, with timer-based sampling and
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
volatile float Kp = 1.0;
volatile float Ki = 0.0;
volatile float Kd = 0.0;

// timer variables
hw_timer_t *timer = NULL;              // hardware timer handle
int timer_frequency = 1e6;             // timer frequency in Hz (1 MHz -> 1 tick = 1 us)
volatile bool timer_activated = false; // flag set by timer ISR each period

// PWM configuration
const int frequency = 10000;               // PWM frequency in Hz
const int resolution = 11;                 // PWM resolution in bits
const int pwm_max = (1 << resolution) - 1; // maximum duty (2048 for 11-bit)

// Encoder counter
volatile int counter = 0;    // running encoder pulse count (signed)
volatile int last_count = 0; // previous sample's count for delta calculation

// Store the last time
volatile uint32_t last_time = 0; // last sample time in ms (as returned by timerReadMillis)

// Convert to radians
const float pulses_per_revolution = 4400.0;                     // encoder pulses per motor revolution
const float radians_per_pulse = 2 * PI / pulses_per_revolution; // conversion factor pulses -> radians

// Velocity moving-average filter (FIR)
const int VEL_MA_WINDOW = 50; // number of samples in the moving average (10 samples ≈ 100 ms at 10 ms period)
float vel_ma_buf[VEL_MA_WINDOW];
int vel_ma_idx = 0;
int vel_ma_count = 0;
float vel_ma_sum = 0.0f;

// Experiment control
int experiment_time = 15000;   // duration of experiment in milliseconds
bool start_experiment = false; // true when experiment is running
uint32_t t_ini = 0;

// Task handles for FreeRTOS tasks
TaskHandle_t control_task_handle = NULL; // handle for control task
TaskHandle_t config_task_handle = NULL;  // handle for configuration task
TaskHandle_t display_task_handle = NULL; // handle for display update task

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

// Timer ISR: set a flag that main loop polls to run 1ms tasks
void IRAM_ATTR timerInterrupt()
{
    timer_activated = true;
}

void setup()
{
    M5.begin();
    Serial.begin(500000); // high baud for fast data logging

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

    // Timer setup
    // Initializes the timer and sets up a 1ms alarm. Timer is stopped initially.
    timer = timerBegin(timer_frequency);          // Initializes the timer at timer_frequency
    timerAttachInterrupt(timer, &timerInterrupt); // Set the ISR associated to the timer
    // Establish the alarm every 10ms
    timerAlarm(timer, 1e4, true, 0);
    timerStop(timer); // The timer will start when the serial command is received

    // Create FreeRTOS tasks for control loop, configuration handling, and display updates
    xTaskCreatePinnedToCore(
        ControlLoopTask,      // Task function
        "Control Loop",       // Task description
        4096,                 // Stack size in words
        NULL,                 // Parameters
        1,                    // Priority
        &control_task_handle, // Task handle
        1);                   // Core ID

    xTaskCreatePinnedToCore(
        ConfigTask,          // Task function
        "Config Task",       // Task description
        4096,                // Stack size in words
        NULL,                // Parameters
        1,                   // Priority
        &config_task_handle, // Task handle
        0);                  // Core ID

    xTaskCreatePinnedToCore(
        DisplayTask,          // Task function
        "Display Task",       // Task description
        4096,                 // Stack size in words
        NULL,                 // Parameters
        1,                    // Priority
        &display_task_handle, // Task handle
        1);                   // Core ID

    Serial.println("READY"); // <-- announce we are alive and ready
}

void loop()
{
}

void ControlLoopTask(void *parameter)
{
    const TickType_t xPeriod = pdMS_TO_TICKS(1);    // 1 ms period
    TickType_t xLastWakeTime = xTaskGetTickCount(); // initialize once

    // This task can be used for more complex control algorithms if needed
    while (true)
    {
        if (control_mode == "position")
        {
            // Position control logic here
        }
        else if (control_mode == "velocity")
        {
            // Velocity control logic here
        }
        else
        {
            // Open-loop control logic here
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod); // Sleep until next cycle
    }
}

void ConfigTask(void *parameter)
{
    const TickType_t xPeriod = pdMS_TO_TICKS(100);  // 100 ms period
    TickType_t xLastWakeTime = xTaskGetTickCount(); // initialize once

    // This task can be used for more complex configuration handling if needed
    while (true)
    {
        // Read serial input for configuration JSON (non-blocking)
        while (Serial.available())
        {
            char c = Serial.read();

            if (c == '\n')
            {
                // End of line: try to parse JSON
                if (lineBuffer.length() > 0)
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

                        // Validate values and set defaults if invalid
                        if (control_mode != "open-loop" && control_mode != "position" && control_mode != "velocity")
                        {
                            Serial.println("Invalid control_mode, using default: open-loop");
                            control_mode = "open-loop"; // default
                        }
                        if (input_signal != "step" && input_signal != "sine" && input_signal != "square" && input_signal != "manual")
                        {
                            Serial.println("Invalid input_signal, using default: step");
                            input_signal = "step"; // default
                        }
                        update_display = true; // flag to update display with new config
                    }
                    lineBuffer = ""; // clear for next line
                }
            }
            else
            {
                lineBuffer += c; // keep accumulating
            }
        }

         // Wait for serial command to start the experiment
        if (!start_experiment)
         {
             if (Serial.available() > 0)
             {
                 // read the incoming byte:
                 int incoming_byte = Serial.read();
                 if (incoming_byte == '1')
                 { // start on character '1'
                     start_experiment = true;
                     t_ini = millis();
                     Serial.println("STARTED"); // <-- confirm reception
                     digitalWrite(LED_PIN, HIGH);
                     timerStart(timer);

                     // Reset moving-average filter state
                     vel_ma_idx = 0;
                     vel_ma_count = 0;
                     vel_ma_sum = 0.0f;
                     for (int i = 0; i < VEL_MA_WINDOW; ++i)
                     {
                         vel_ma_buf[i] = 0.0f;
                     }
                 }
             }
         }

         // Timer-driven sampling: run when ISR sets timer_activated
         if (timer_activated)
         {
             // Compute increments since last sample
             long delta_pos_ppr = counter - last_count;

             // Get current experiment time in ms from timer helper
             uint32_t current_time = millis() - t_ini;

             // elapsed time in seconds since last sample (used for velocity)
             float elapsed_time = (current_time - last_time) / 1000.0; // convert ms to seconds

             // convert pulse delta to radians
             float delta_pos = delta_pos_ppr * radians_per_pulse;

             // Calculate position in radians (total)
             float pos = counter * radians_per_pulse;

             // Calculate velocity (rad/s). Guard against division by zero on first sample.
             float vel = 0.0f;
             float vel_filtered = 0.0f;
             if (elapsed_time > 0.0f)
             {
                 vel = delta_pos / elapsed_time;
                 // Moving average filter over last VEL_MA_WINDOW velocity samples
                 vel_ma_sum -= vel_ma_buf[vel_ma_idx];
                 vel_ma_buf[vel_ma_idx] = vel;
                 vel_ma_sum += vel;
                 vel_ma_idx = (vel_ma_idx + 1) % VEL_MA_WINDOW;
                 if (vel_ma_count < VEL_MA_WINDOW)
                 {
                     vel_ma_count++;
                 }
                 vel_filtered = vel_ma_sum / (float)vel_ma_count;
             }

             if (current_time <= experiment_time)
             {
                 int pwm = current_time / 10;
                 ledcWrite(PWM_CCW_PIN, pwm);                                                        // writing to CCW (counter-clockwise)
                 ledcWrite(PWM_CW_PIN, 0);                                                           // stopping CW (clockwise)
                 Serial.printf("%d;%.4f;%.4f;%.4f;%d\n", pwm, pos, vel, vel_filtered, current_time); // PWM;pos;vel;vel_filtered;time
             }
             else
             {
                 // End of experiment: stop motor, stop timer and reset state
                 ledcWrite(PWM_CW_PIN, 0);
                 ledcWrite(PWM_CCW_PIN, 0);
                 Serial.println("END");
                 timerStop(timer);
                 digitalWrite(LED_PIN, LOW);
                 start_experiment = false;
                 last_count = 0;
                 last_time = 0;
                 counter = 0;

                 // Reset moving-average filter state
                 vel_ma_idx = 0;
                 vel_ma_count = 0;
                 vel_ma_sum = 0.0f;
                 for (int i = 0; i < VEL_MA_WINDOW; ++i)
                 {
                     vel_ma_buf[i] = 0.0f;
                 }
             }

             // clear flag and store last-sample values
             timer_activated = false;

             // Update last values for next iteration
             last_count = counter;
             last_time = current_time;
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