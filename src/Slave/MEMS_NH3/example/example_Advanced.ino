/*
 * Advanced Example with Temperature/Humidity Compensation and JSON Output
 *
 * Connections:
 * - NH3 Sensor: A0
 * - (Optional) DHT22/SHT31 for temperature and humidity
 */

#include "MEMS_NH3.h"

// Define the sensor
MEMS_NH3 sensor(A0, 4700, 3.3, -1.53, 1.35);

// Temperature and humidity variables (can be read from DHT22)
float temperature = 25.0; // °C
float humidity = 50.0;    // %

// Timers
unsigned long lastRead = 0;
unsigned long lastTWAReset = 0;
const unsigned long READ_INTERVAL = 2000;         // 2 seconds
const unsigned long TWA_RESET_INTERVAL = 3600000; // 1 hour

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println(F("\n=== MEMS NH3 Advanced Demo ==="));
    Serial.println();

    sensor.begin();

    // Enable temperature and humidity compensation
    sensor.enableTempHumidityCorrection(true);
    sensor.setTempCoeff(0.01);      // 1% change per °C
    sensor.setHumidityCoeff(0.005); // 0.5% change per % humidity

    // Enable Kalman filter
    sensor.enableKalmanFilter(true);

    // Calibrate if needed
    if (!sensor.isCalibrated())
    {
        Serial.println(F("Starting calibration..."));
        performCalibration();
    }

    Serial.println(F("Sensor ready!"));
    Serial.println();
}

void loop()
{
    unsigned long now = millis();

    // Periodic reading
    if (now - lastRead >= READ_INTERVAL)
    {
        lastRead = now;

        if (sensor.isWarmedUp())
        {
            readAndDisplay();
        }
        else
        {
            displayWarmupStatus();
        }
    }

    // Reset TWA every hour
    if (now - lastTWAReset >= TWA_RESET_INTERVAL)
    {
        lastTWAReset = now;
        Serial.println(F("\n--- TWA Reset (1 hour period) ---\n"));
        sensor.resetTWA();
    }

    // Serial commands (optional)
    handleSerialCommands();
}

void readAndDisplay()
{
    // Update temperature and humidity
    // In real use, read these values from DHT22/SHT31
    updateEnvironmentalData();
    sensor.setTempHumidityCorrection(temperature, humidity);

    // Read PPM
    float ppm = sensor.getPPM(true); // With Kalman filter

    if (!isnan(ppm))
    {
        // Text output
        displayTextOutput(ppm);

        // JSON output (for sending to server)
        Serial.println(F("\nJSON Output:"));
        Serial.println(sensor.toJSON());
        Serial.println();

        // Update TWA
        sensor.updateTWA();

        // Check alarm
        checkAlarm(ppm);
    }
}

void displayTextOutput(float ppm)
{
    Serial.println(F("==================="));
    Serial.print(F("NH3: "));
    Serial.print(ppm, 2);
    Serial.print(F(" ppm ["));
    Serial.print(sensor.getAlertString());
    Serial.println(F("]"));

    Serial.print(F("Rs: "));
    Serial.print(sensor.getRs(), 2);
    Serial.print(F(" Ω  |  R0: "));
    Serial.print(sensor.getR0(), 2);
    Serial.println(F(" Ω"));

    Serial.print(F("Ratio: "));
    Serial.print(sensor.getRatio(), 3);
    Serial.print(F("  |  TWA: "));
    Serial.print(sensor.getTWA(), 2);
    Serial.println(F(" ppm"));

    Serial.print(F("Temp: "));
    Serial.print(temperature, 1);
    Serial.print(F("°C  |  Humidity: "));
    Serial.print(humidity, 1);
    Serial.println(F("%"));
}

void displayWarmupStatus()
{
    unsigned long remaining = sensor.getWarmupRemaining() / 1000;
    Serial.print(F("⏳ Warming up... "));
    Serial.print(remaining);
    Serial.println(F("s remaining"));
}

void checkAlarm(float ppm)
{
    AlertLevel alert = sensor.getAlertLevel();

    if (alert == DANGER)
    {
        Serial.println(F("\n🚨🚨🚨 DANGER: HIGH NH3 LEVEL! 🚨🚨🚨"));
        Serial.println(F("Evacuate area immediately!"));
        // You can activate a buzzer or warning LED here
    }
    else if (alert == WARNING)
    {
        Serial.println(F("\n⚠️ WARNING: Elevated NH3 detected"));
        Serial.println(F("Ventilate area"));
    }
}

void updateEnvironmentalData()
{
    // Here you should read data from DHT22/SHT31
    // In this demo, constant values are used

    // Example with DHT22:
    // temperature = dht.readTemperature();
    // humidity = dht.readHumidity();

    // Simulate small variations
    temperature = 25.0 + random(-10, 10) / 10.0;
    humidity = 50.0 + random(-50, 50) / 10.0;
}

void performCalibration()
{
    Serial.println(F("\n=== Manual Calibration Mode ==="));
    Serial.println(F("1. Ensure sensor is in CLEAN AIR"));
    Serial.println(F("2. Wait for stable readings"));
    Serial.println(F("3. Press any key to start..."));

    while (!Serial.available())
    {
        delay(100);
    }
    while (Serial.available())
        Serial.read(); // Clear buffer

    sensor.calibrateR0(100, 100);

    Serial.println(F("\n✓ Calibration complete!"));
    delay(2000);
}

void handleSerialCommands()
{
    if (Serial.available())
    {
        char cmd = Serial.read();
        while (Serial.available())
            Serial.read(); // Clear buffer

        switch (cmd)
        {
        case 'c':
        case 'C':
            Serial.println(F("\nStarting recalibration..."));
            performCalibration();
            break;

        case 'r':
        case 'R':
            Serial.println(F("\nResetting TWA..."));
            sensor.resetTWA();
            break;

        case 'k':
        case 'K':
            Serial.println(F("\nResetting Kalman filter..."));
            sensor.resetKalmanFilter();
            break;

        case 'i':
        case 'I':
            printInfo();
            break;

        case 'h':
        case 'H':
            printHelp();
            break;
        }
    }
}

void printInfo()
{
    Serial.println(F("\n=== Sensor Information ==="));
    Serial.print(F("R0: "));
    Serial.print(sensor.getR0(), 2);
    Serial.println(F(" Ω"));

    Serial.print(F("Calibrated: "));
    Serial.println(sensor.isCalibrated() ? F("Yes") : F("No"));

    Serial.print(F("Warmed up: "));
    Serial.println(sensor.isWarmedUp() ? F("Yes") : F("No"));

    Serial.print(F("Needs recalibration: "));
    Serial.println(sensor.needsRecalibration() ? F("Yes") : F("No"));

    Serial.print(F("Temp correction: "));
    Serial.println(temperature, 1);

    Serial.print(F("Humidity correction: "));
    Serial.println(humidity, 1);

    Serial.println();
}

void printHelp()
{
    Serial.println(F("\n=== Available Commands ==="));
    Serial.println(F("C - Start calibration"));
    Serial.println(F("R - Reset TWA"));
    Serial.println(F("K - Reset Kalman filter"));
    Serial.println(F("I - Show sensor info"));
    Serial.println(F("H - Show this help"));
    Serial.println();
}
