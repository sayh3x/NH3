/*
 * MEMS_NH3.cpp - Implementation of the Ammonia Sensor Library
 */

#include "MEMS_NH3.h"

// ==================== SimpleKalmanFilter ====================

SimpleKalmanFilter::SimpleKalmanFilter(float processNoise, float measurementNoise)
    : Q(processNoise), R(measurementNoise), P(1.0), K(0), X(0) {}

float SimpleKalmanFilter::update(float measurement)
{
    // Prediction
    P = P + Q;

    // Update
    K = P / (P + R);
    X = X + K * (measurement - X);
    P = (1 - K) * P;

    return X;
}

void SimpleKalmanFilter::reset()
{
    P = 1.0;
    X = 0;
}

// ==================== TWACalculator ====================

TWACalculator::TWACalculator() : sumPPM(0), samples(0), startTime(0) {}

void TWACalculator::addSample(float ppm)
{
    if (startTime == 0)
    {
        startTime = millis();
    }
    sumPPM += ppm;
    samples++;
}

float TWACalculator::getTWA()
{
    if (samples == 0)
        return 0;
    return sumPPM / samples;
}

void TWACalculator::reset()
{
    sumPPM = 0;
    samples = 0;
    startTime = millis();
}

unsigned long TWACalculator::getElapsedTime()
{
    if (startTime == 0)
        return 0;
    return millis() - startTime;
}

// ==================== MEMS_NH3 ====================

MEMS_NH3::MEMS_NH3(int adcPin, float rl, float vcc, float a, float b)
    : _adcPin(adcPin), _rl(rl), _vcc(vcc), _a(a), _b(b),
      _r0(0), _startTime(0), _lastCalibrationTime(0),
      _tempHumidityEnabled(false), _temp(25), _humidity(50),
      _kT(0.01), _kH(0.005), _kalmanEnabled(true),
      _kalmanFilter(0.1, 5.0), _lastReadValid(false) {}

void MEMS_NH3::begin()
{
    pinMode(_adcPin, INPUT);
    EEPROM.begin(512);

    if (!isEEPROMInitialized())
    {
        initEEPROM();
    }

    loadR0();
    loadCalibrationTime();

    _startTime = millis();

    Serial.println(F("MEMS NH3 Sensor initialized"));
    Serial.print(F("Warmup time: "));
    Serial.print(WARMUP_TIME / 1000);
    Serial.println(F(" seconds"));
}

// ==================== Calibration ====================

void MEMS_NH3::calibrateR0(int samples, int delayMs)
{
    Serial.println(F("=== Starting R0 Calibration ==="));
    Serial.println(F("Ensure sensor is in CLEAN AIR!"));
    Serial.println(F("Waiting 5 seconds..."));

    delay(5000);

    float sum = 0;
    int validSamples = 0;

    Serial.print(F("Collecting "));
    Serial.print(samples);
    Serial.println(F(" samples..."));

    for (int i = 0; i < samples; i++)
    {
        float vnode = readAverageADC(5);

        if (!validateReading(vnode))
        {
            Serial.print(F("."));
            continue;
        }

        float rs = calculateRs(vnode);

        if (!validateRs(rs))
        {
            Serial.print(F("!"));
            continue;
        }

        sum += rs;
        validSamples++;

        if (i % 10 == 0)
        {
            Serial.print(F("#"));
        }

        delay(delayMs);
    }

    Serial.println();

    if (validSamples < samples / 2)
    {
        Serial.println(F("ERROR: Not enough valid samples!"));
        Serial.print(F("Valid: "));
        Serial.print(validSamples);
        Serial.print(F("/"));
        Serial.println(samples);
        return;
    }

    _r0 = sum / validSamples;
    _lastCalibrationTime = millis();

    saveR0();
    saveCalibrationTime();

    Serial.println(F("=== Calibration Complete ==="));
    Serial.print(F("R0 = "));
    Serial.print(_r0);
    Serial.println(F(" Ω"));
    Serial.print(F("Valid samples: "));
    Serial.print(validSamples);
    Serial.print(F("/"));
    Serial.println(samples);
}

bool MEMS_NH3::isCalibrated()
{
    return validateR0(_r0);
}

bool MEMS_NH3::needsRecalibration()
{
    if (_lastCalibrationTime == 0)
        return true;
    return (millis() - _lastCalibrationTime) > RECALIBRATION_INTERVAL;
}

// ==================== Data Reading ====================

float MEMS_NH3::getPPM(bool useFilter)
{
    if (!isWarmedUp())
    {
        _lastReadValid = false;
        return NAN;
    }

    if (!isCalibrated())
    {
        Serial.println(F("ERROR: Sensor not calibrated!"));
        _lastReadValid = false;
        return NAN;
    }

    float vnode = getVnode();

    if (!validateReading(vnode))
    {
        _lastReadValid = false;
        return NAN;
    }

    float rs = calculateRs(vnode);

    if (!validateRs(rs))
    {
        _lastReadValid = false;
        return NAN;
    }

    // Environmental correction
    if (_tempHumidityEnabled)
    {
        rs = applyEnvironmentalCorrection(rs);
    }

    float ratio = rs / _r0;
    float ppm = ratioToPPM(ratio);

    // Kalman filter
    if (useFilter && _kalmanEnabled)
    {
        ppm = _kalmanFilter.update(ppm);
    }

    _lastReadValid = true;
    return ppm;
}

float MEMS_NH3::getRs()
{
    float vnode = getVnode();
    return calculateRs(vnode);
}

float MEMS_NH3::getRatio()
{
    if (_r0 == 0)
        return NAN;
    return getRs() / _r0;
}

float MEMS_NH3::getR0()
{
    return _r0;
}

float MEMS_NH3::getVnode()
{
    return readAverageADC(10);
}

// ==================== R0 Management ====================

void MEMS_NH3::saveR0()
{
    EEPROM.put(EEPROM_R0_ADDR, _r0);
    EEPROM.commit();
    Serial.print(F("R0 saved to EEPROM: "));
    Serial.println(_r0);
}

bool MEMS_NH3::loadR0()
{
    EEPROM.get(EEPROM_R0_ADDR, _r0);

    if (validateR0(_r0))
    {
        Serial.print(F("R0 loaded from EEPROM: "));
        Serial.println(_r0);
        return true;
    }

    Serial.println(F("No valid R0 found in EEPROM"));
    _r0 = 0;
    return false;
}

void MEMS_NH3::setR0(float r0)
{
    if (validateR0(r0))
    {
        _r0 = r0;
        saveR0();
    }
}

// ==================== Sensor Status ====================

bool MEMS_NH3::isWarmedUp()
{
    return (millis() - _startTime) >= WARMUP_TIME;
}

bool MEMS_NH3::isValid()
{
    return _lastReadValid;
}

unsigned long MEMS_NH3::getWarmupRemaining()
{
    unsigned long elapsed = millis() - _startTime;
    if (elapsed >= WARMUP_TIME)
        return 0;
    return WARMUP_TIME - elapsed;
}

// ==================== Environmental Correction ====================

void MEMS_NH3::setTempHumidityCorrection(float temp, float humidity)
{
    _temp = temp;
    _humidity = humidity;
}

void MEMS_NH3::enableTempHumidityCorrection(bool enable)
{
    _tempHumidityEnabled = enable;
}

void MEMS_NH3::setTempCoeff(float kT)
{
    _kT = kT;
}

void MEMS_NH3::setHumidityCoeff(float kH)
{
    _kH = kH;
}

// ==================== Filter ====================

void MEMS_NH3::enableKalmanFilter(bool enable)
{
    _kalmanEnabled = enable;
    if (!enable)
    {
        _kalmanFilter.reset();
    }
}

void MEMS_NH3::resetKalmanFilter()
{
    _kalmanFilter.reset();
}

// ==================== Alert Level ====================

AlertLevel MEMS_NH3::getAlertLevel()
{
    float ppm = getPPM();
    if (isnan(ppm))
        return SAFE;

    if (ppm < SAFE_LEVEL)
        return SAFE;
    if (ppm < CAUTION_LEVEL)
        return CAUTION;
    if (ppm < WARNING_LEVEL)
        return WARNING;
    return DANGER;
}

const char *MEMS_NH3::getAlertString()
{
    switch (getAlertLevel())
    {
    case SAFE:
        return "SAFE";
    case CAUTION:
        return "CAUTION";
    case WARNING:
        return "WARNING";
    case DANGER:
        return "DANGER";
    default:
        return "UNKNOWN";
    }
}

// ==================== TWA ====================

void MEMS_NH3::updateTWA()
{
    float ppm = getPPM();
    if (!isnan(ppm))
    {
        _twaCalc.addSample(ppm);
    }
}

float MEMS_NH3::getTWA()
{
    return _twaCalc.getTWA();
}

void MEMS_NH3::resetTWA()
{
    _twaCalc.reset();
}

// ==================== JSON output ====================

String MEMS_NH3::toJSON()
{
    String json = "{";

    float ppm = getPPM();
    json += "\"ppm\":";
    json += isnan(ppm) ? "null" : String(ppm, 2);

    json += ",\"rs\":";
    json += String(getRs(), 2);

    json += ",\"r0\":";
    json += String(_r0, 2);

    json += ",\"ratio\":";
    json += isnan(ppm) ? "null" : String(getRatio(), 3);

    json += ",\"alert\":\"";
    json += getAlertString();
    json += "\"";

    json += ",\"twa\":";
    json += String(getTWA(), 2);

    json += ",\"warmed_up\":";
    json += isWarmedUp() ? "true" : "false";

    json += ",\"calibrated\":";
    json += isCalibrated() ? "true" : "false";

    json += ",\"timestamp\":";
    json += String(millis());

    json += "}";

    return json;
}

// ==================== Configuration ====================

void MEMS_NH3::setCoefficients(float a, float b)
{
    _a = a;
    _b = b;
}

void MEMS_NH3::setRL(float rl)
{
    _rl = rl;
}

void MEMS_NH3::setVcc(float vcc)
{
    _vcc = vcc;
}

// ==================== Private Functions ====================

float MEMS_NH3::readAverageADC(int samples)
{
    float sum = 0;
    for (int i = 0; i < samples; i++)
    {
        int raw = analogRead(_adcPin);
        float vnode = (raw / 1023.0) * _vcc;
        sum += vnode;
        delay(10);
    }
    return sum / samples;
}

bool MEMS_NH3::validateReading(float vnode)
{
    return (vnode > 0.05 && vnode < (_vcc - 0.05));
}

bool MEMS_NH3::validateRs(float rs)
{
    return (rs > 100 && rs < 1000000);
}

bool MEMS_NH3::validateR0(float r0)
{
    return (r0 > 100 && r0 < 500000);
}

float MEMS_NH3::calculateRs(float vnode)
{
    if (vnode <= 0)
        return NAN;
    return _rl * ((_vcc / vnode) - 1.0);
}

float MEMS_NH3::applyEnvironmentalCorrection(float rs)
{
    float tempFactor = 1.0 + _kT * (_temp - 25.0);
    float humidityFactor = 1.0 + _kH * (_humidity - 50.0);
    return rs * tempFactor * humidityFactor;
}

float MEMS_NH3::ratioToPPM(float ratio)
{
    if (ratio <= 0)
        return NAN;

    // log10(ppm) = a * log10(ratio) + b
    float logPPM = _a * log10(ratio) + _b;
    float ppm = pow(10, logPPM);

    // Limit to a reasonable range
    if (ppm < 0)
        ppm = 0;
    if (ppm > 1000)
        ppm = 1000;

    return ppm;
}

void MEMS_NH3::saveCalibrationTime()
{
    EEPROM.put(EEPROM_CALIB_TIME_ADDR, _lastCalibrationTime);
    EEPROM.commit();
}

bool MEMS_NH3::loadCalibrationTime()
{
    EEPROM.get(EEPROM_CALIB_TIME_ADDR, _lastCalibrationTime);
    return (_lastCalibrationTime != 0xFFFFFFFF && _lastCalibrationTime != 0);
}

bool MEMS_NH3::isEEPROMInitialized()
{
    uint32_t magic;
    EEPROM.get(EEPROM_MAGIC_ADDR, magic);
    return (magic == EEPROM_MAGIC_NUMBER);
}

void MEMS_NH3::initEEPROM()
{
    uint32_t magic = EEPROM_MAGIC_NUMBER;
    EEPROM.put(EEPROM_MAGIC_ADDR, magic);
    EEPROM.commit();
    Serial.println(F("EEPROM initialized"));
}
