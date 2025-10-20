#include <LowPower.h> // https://github.com/rocketscream/Low-Power
#include <Arduino.h>
#include <VL53L1X.h> // VL53L1X library for laser distance sensor 
#include <Wire.h>    // I2C library

// ——— CONFIGURATION —————————————————————————————————————————————————————
#define DEBUG // comment out to disable Serial debug

// Timing
const unsigned long CHECK_INTERVAL_MS = 1000UL; // 1 hour
const unsigned long SLEEP_CYCLE_MS = 1000UL;    // WDT wake interval

// Bin‑full threshold (cm)
const uint16_t FULL_THRESHOLD_CM = 15;

// Tile Pro on‑time
const unsigned long TILE_ON_MS = 5000UL; // 30 s

// Battery monitor
const uint8_t BATTERY_PIN = A0;     // voltage divider into A7
const float VOLTAGE_DIVIDER = 2.0;  // e.g. 100 kΩ/100 kΩ
const float LOW_BATTERY_VOLT = 5.3; // warn if below 3.3 V

// Pins
const uint8_t PIN_SENSOR_PWR = 7; // MOSFET gate for VL53L0X VCC
const uint8_t PIN_TILE_PWR = 2;   // MOSFET gate for Tile Pro VCC
const uint8_t PIN_LED = 13;       // on‑board LED

// Measurement
const uint8_t DIST_READINGS = 3;        // average N readings
const unsigned long READ_DELAY_MS = 50; // between readings

// ——— GLOBALS —————————————————————————————————————————————————————————————
unsigned long elapsedMs = 0;
VL53L1X sensor; // VL53L1X sensor object

#ifdef DEBUG
#define DBG_START() Serial.begin(9600)
#define DBG_PRINTLN(x) Serial.println(x)
#define DBG_PRINTF(fmt, ...) Serial.print((fmt), __VA_ARGS__)
#else
#define DBG_START()
#define DBG_PRINTLN(x)
#define DBG_PRINTF(fmt, ...)
#endif

// ——— FUNCTION DECLARATIONS —————————————————————————————————————————————
void sleepCycle();
long measureDistance();
float readBatteryVoltage();
void blinkLED(uint8_t times, uint16_t period);

// ——— SETUP —————————————————————————————————————————————————————————————
void setup()
{
    pinMode(PIN_SENSOR_PWR, OUTPUT);
    pinMode(PIN_TILE_PWR, OUTPUT);
    pinMode(PIN_LED, OUTPUT);

    digitalWrite(PIN_SENSOR_PWR, LOW);
    digitalWrite(PIN_TILE_PWR, LOW);
    digitalWrite(PIN_LED, LOW);

    Serial.begin(9600);
    Serial.println(F("=== Bin Sensor System Starting (VLX) ==="));

    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C

    // Power sensor, initialize VL53L1X
    digitalWrite(PIN_SENSOR_PWR, HIGH);
    delay(100); // Wait for sensor to power up

    if (!sensor.init())
    {
        Serial.println("Failed to detect and initialize sensor!");
        while (1);
    }

    // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(50000);

    digitalWrite(PIN_SENSOR_PWR, LOW); // Power down sensor
}

// ——— MAIN LOOP ———————————————————————————————————————————————————————————
void loop()
{
    // 1) Sleep in 8s increments until it's time to check
    while (elapsedMs < CHECK_INTERVAL_MS)
    {
        sleepCycle();
        elapsedMs += SLEEP_CYCLE_MS;
    }
    elapsedMs = 0; // reset timer

    // 2) Check battery
    float battV = readBatteryVoltage();
    Serial.print("Battery: ");
    Serial.print(battV);
    Serial.println(" V");
    if (battV < LOW_BATTERY_VOLT)
    {
        Serial.println(F("! Low battery, skipping check."));
        blinkLED(3, 200);
        return;
    }

    // 3) Power sensor, wait to stabilize
    digitalWrite(PIN_SENSOR_PWR, HIGH);
    delay(100);

    // Start continuous readings at a rate of one measurement every 50 ms
    sensor.startContinuous(50);

    // 4) Measure distance (average of N readings)
    long sum = 0;
    for (uint8_t i = 0; i < DIST_READINGS; i++)
    {
        long d = measureDistance();
        Serial.print("  read ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(d);
        Serial.println(" cm");
        sum += d;
        delay(READ_DELAY_MS);
    }
    long distance = sum / DIST_READINGS;
    Serial.print(" Avg distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    sensor.stopContinuous();
    digitalWrite(PIN_SENSOR_PWR, LOW); // power down sensor

    // 5) If bin full → power Tile for TILE_ON_MS
    if (distance > 0 && distance < FULL_THRESHOLD_CM)
    {
        Serial.println(F("Bin FULL → powering Tile Pro"));
        digitalWrite(PIN_TILE_PWR, HIGH);
        delay(TILE_ON_MS);
        digitalWrite(PIN_TILE_PWR, LOW);
    }
    else
    {
        Serial.println(F("Bin not full."));
        digitalWrite(PIN_TILE_PWR, LOW);
    }
}

// ——— HELPER FUNCTIONS ————————————————————————————————————————————————————

// Perform one WDT sleep cycle
void sleepCycle()
{
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
}

// Measure distance using VL53L1X, return cm (or –1 on timeout)
long measureDistance()
{
    uint16_t distance_mm = sensor.read();
    
    if (sensor.timeoutOccurred())
    {
        return -1;
    }
    return (long)(distance_mm / 10); // Convert mm to cm
}

// Read battery via analog pin + voltage divider
float readBatteryVoltage()
{
    uint16_t raw = analogRead(BATTERY_PIN);
    float volts = raw * (5.0 / 1023.0) * VOLTAGE_DIVIDER;
    return volts;
}

// Blink LED for visual feedback
void blinkLED(uint8_t times, uint16_t period)
{
    while (times--)
    {
        digitalWrite(PIN_LED, HIGH);
        delay(period);
        digitalWrite(PIN_LED, LOW);
        delay(period);
    }
}
