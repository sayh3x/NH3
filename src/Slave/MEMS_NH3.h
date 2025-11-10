/*
 * MEMS_NH3.h - High-precision ammonia detection library for SEN0567 sensor
 * Compatible with NodeMCU and ESP8266
 *
 * Features:
 * - Automatic R0 calibration
 * - Kalman filter for high accuracy
 * - Sensor warm-up management
 * - Temperature and humidity compensation
 * - Multi-level alert system
 * - Time Weighted Average (TWA) calculation
 * - EEPROM data storage
 * - JSON output for IoT applications
 */

#ifndef MEMS_NH3_H
#define MEMS_NH3_H

#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>

// EEPROM addresses for data storage
#define EEPROM_R0_ADDR 0
#define EEPROM_CALIB_TIME_ADDR 4
#define EEPROM_MAGIC_ADDR 8
#define EEPROM_MAGIC_NUMBER 0xAB12CD34

// Default values
#define DEFAULT_VCC 3.3
#define DEFAULT_RL 4700.0
#define DEFAULT_A -1.53
#define DEFAULT_B 1.35
#define WARMUP_TIME 300000                  // 5 minutes
#define RECALIBRATION_INTERVAL 2592000000UL // 30 days

// NH3 alert levels (ppm)
#define SAFE_LEVEL 25
#define CAUTION_LEVEL 50
#define WARNING_LEVEL 100

enum AlertLevel
{
    SAFE,    // < 25 ppm
    CAUTION, // 25-50 ppm
    WARNING, // 50-100 ppm
    DANGER   // > 100 ppm
};

// Simple Kalman Filter
class SimpleKalmanFilter
{
private:
    float Q; // Process noise
    float R; // Measurement noise
    float P; // Estimation error
    float K; // Kalman gain
    float X; // Estimated value

public:
    SimpleKalmanFilter(float processNoise = 0.1, float measurementNoise = 5.0);
    float update(float measurement);
    void reset();
};

// Time Weighted Average (TWA) calculator
class TWACalculator
{
private:
    float sumPPM;
    int samples;
    unsigned long startTime;

public:
    TWACalculator();
    void addSample(float ppm);
    float getTWA();
    void reset();
    unsigned long getElapsedTime();
};

// Main sensor class
class MEMS_NH3
{
public:
    // Constructor
    MEMS_NH3(int adcPin, float rl = DEFAULT_RL, float vcc = DEFAULT_VCC,
             float a = DEFAULT_A, float b = DEFAULT_B);

    // Initialization
    void begin();

    // Calibration
    void calibrateR0(int samples = 100, int delayMs = 100);
    bool isCalibrated();
    bool needsRecalibration();

    // Data reading
    float getPPM(bool useFilter = true);
    float getRs();
    float getRatio();
    float getR0();
    float getVnode();

    // R0 management
    void saveR0();
    bool loadR0();
    void setR0(float r0);

    // Sensor status
    bool isWarmedUp();
    bool isValid();
    unsigned long getWarmupRemaining();

    // Environmental correction
    void setTempHumidityCorrection(float temp, float humidity);
    void enableTempHumidityCorrection(bool enable);
    void setTempCoeff(float kT);
    void setHumidityCoeff(float kH);

    // Filter
    void enableKalmanFilter(bool enable);
    void resetKalmanFilter();

    // Alert level
    AlertLevel getAlertLevel();
    const char *getAlertString();

    // TWA
    void updateTWA();
    float getTWA();
    void resetTWA();

    // JSON output
    String toJSON();

    // Configuration
    void setCoefficients(float a, float b);
    void setRL(float rl);
    void setVcc(float vcc);

private:
    // Pins and parameters
    int _adcPin;
    float _rl, _vcc, _r0;
    float _a, _b; // Logarithmic coefficients

    // Timing
    unsigned long _startTime;
    unsigned long _lastCalibrationTime;

    // Environmental correction
    bool _tempHumidityEnabled;
    float _temp, _humidity;
    float _kT, _kH; // Correction coefficients

    // Filter
    bool _kalmanEnabled;
    SimpleKalmanFilter _kalmanFilter;

    // TWA
    TWACalculator _twaCalc;

    // Last reading validity
    bool _lastReadValid;

    // Helper functions
    float readAverageADC(int samples);
    bool validateReading(float vnode);
    bool validateRs(float rs);
    bool validateR0(float r0);
    float calculateRs(float vnode);
    float applyEnvironmentalCorrection(float rs);
    float ratioToPPM(float ratio);
    void saveCalibrationTime();
    bool loadCalibrationTime();
    bool isEEPROMInitialized();
    void initEEPROM();
};

#endif
