/*
 * Simple Example using the MEMS_NH3 Library
 * for NodeMCU and ESP8266
 *
 * Connections:
 * - Sensor VCC -> 3.3V
 * - Sensor GND -> GND
 * - Sensor OUT -> A0 (analog input pin)
 */

#include "MEMS_NH3.h"

// Define the sensor
// Parameters: (adcPin, RL, Vcc, a, b)
MEMS_NH3 sensor(A0, 4700, 3.3, -1.53, 1.35);

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println(F("\n=== MEMS NH3 Sensor Demo ==="));
    Serial.println(F("SEN0567 - NodeMCU Version"));
    Serial.println();

    // Initialize the sensor
    sensor.begin();

    // Check calibration
    if (!sensor.isCalibrated())
    {
        Serial.println(F("⚠️ Sensor not calibrated!"));
        Serial.println(F("Place the sensor in CLEAN AIR and wait..."));
        Serial.println();

        // Automatic calibration
        // Parameters: (number of samples, delay between samples in ms)
        sensor.calibrateR0(100, 100);
    }
    else
    {
        Serial.print(F("✓ R0 loaded: "));
        Serial.print(sensor.getR0());
        Serial.println(F(" Ω"));
    }

    Serial.println();
}

void loop()
{
    // Check sensor warm-up
    if (!sensor.isWarmedUp())
    {
        unsigned long remaining = sensor.getWarmupRemaining() / 1000;
        Serial.print(F("⏳ Warming up... "));
        Serial.print(remaining);
        Serial.println(F("s remaining"));
        delay(1000);
        return;
    }

    // Read PPM value
    float ppm = sensor.getPPM();

    if (isnan(ppm))
    {
        Serial.println(F("❌ Error reading sensor!"));
    }
    else
    {
        // Display main readings
        Serial.println(F("--- NH3 Reading ---"));
        Serial.print(F("PPM: "));
        Serial.print(ppm, 2);
        Serial.println(F(" ppm"));

        Serial.print(F("Rs: "));
        Serial.print(sensor.getRs(), 2);
        Serial.println(F(" Ω"));

        Serial.print(F("Ratio (Rs/R0): "));
        Serial.println(sensor.getRatio(), 3);

        // Alert level
        AlertLevel alert = sensor.getAlertLevel();
        Serial.print(F("Alert Level: "));
        Serial.print(sensor.getAlertString());

        switch (alert)
        {
        case SAFE:
            Serial.println(F(" ✓"));
            break;
        case CAUTION:
            Serial.println(F(" ⚠️"));
            break;
        case WARNING:
            Serial.println(F(" ⚠️⚠️"));
            break;
        case DANGER:
            Serial.println(F(" 🚨 DANGER!"));
            break;
        }

        // Update TWA (Time Weighted Average)
        sensor.updateTWA();
        Serial.print(F("TWA: "));
        Serial.print(sensor.getTWA(), 2);
        Serial.println(F(" ppm"));

        Serial.println();
    }

    // Check if recalibration is needed
    if (sensor.needsRecalibration())
    {
        Serial.println(F("⚠️ Sensor needs recalibration (30 days passed)"));
    }

    delay(2000);
}
