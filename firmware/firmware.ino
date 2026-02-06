/*

Send a PWM ramp to the motor and get the current angular position and velocity of the motor (DC6V210RPM - Reducer 20:1).
The data is send/received through serial port.

This script uses the 3.X version of the Arduino ESP32 core.

Author: Juan M. Gandarias
web: www.jmgandarias.com
email: jmgandarias@uma.es

*/

#define ENCODER_A 26 // pin connected to encoder channel A
#define ENCODER_B 27 // pin connected to encoder channel B

#define PWM_CCW_PIN 32  // PWM pin for clockwise drive
#define PWM_CW_PIN 33 // PWM pin for counter-clockwise drive

#define LED_PIN 13 // activity indicator LED

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
const float pulses_per_revolution = 880.0;                      // encoder pulses per motor revolution
const float radians_per_pulse = 2 * PI / pulses_per_revolution; // conversion factor pulses -> radians

// Velocity moving-average filter (FIR)
const int VEL_MA_WINDOW = 50; // number of samples in the moving average (10 samples ≈ 100 ms at 10 ms period)
float vel_ma_buf[VEL_MA_WINDOW];
int vel_ma_idx = 0;
int vel_ma_count = 0;
float vel_ma_sum = 0.0f;

// Experiment control
int experiment_time = pwm_max * 10; // duration of experiment in milliseconds
bool start_experiment = false;      // true when experiment is running
uint32_t t_ini = 0;

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

    Serial.println("READY"); // <-- announce we are alive and ready
}

void loop()
{
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
        // Experiment phases:
        // - first half: ramp
        // - second half: apply step (max pwm) -> log
        // - after experiment_time: stop and reset
        if (current_time <= experiment_time)
        {
            int pwm = current_time / 10;
            ledcWrite(PWM_CCW_PIN, pwm); // stop drive
            ledcWrite(PWM_CW_PIN, 0);
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
}
