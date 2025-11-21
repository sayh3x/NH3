/*
  NH3 Display (aligned labels+values, partial updates)
  NodeMCU (ESP8266) + MEMS_NH3 + GC9A01 240x240
*/

#include <Arduino_GFX_Library.h>
#include "MEMS_NH3.h"
#include <math.h>
#include <stdio.h>

// ---------------- TFT 1.28" GC9A01 ----------------
#define TFT_DC   D1
#define TFT_CS   D8
#define TFT_RST  D2

Arduino_DataBus *bus = new Arduino_ESP8266SPI(TFT_DC, TFT_CS);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, TFT_RST, 0, true);

// ---------------- NH3 Sensor ----------------
MEMS_NH3 sensor(A0, 4700, 3.3, -1.53, 1.35);

// ---------- UI / layout state ----------
int16_t scr_w, scr_h;
int16_t cx, cy, radius;

// color constants (fallbacks in case library doesn't provide)
#ifndef RGB565_BLACK
  #define RGB565_BLACK 0x0000
#endif
#ifndef RGB565_WHITE
  #define RGB565_WHITE 0xFFFF
#endif
#ifndef RGB565_YELLOW
  #define RGB565_YELLOW 0xFFE0
#endif
#ifndef RGB565_CYAN
  #define RGB565_CYAN 0x07FF
#endif
#ifndef RGB565_ORANGE
  #define RGB565_ORANGE 0xFD20
#endif
#ifndef RGB565_GREEN
  #define RGB565_GREEN 0x07E0
#endif
#ifndef RGB565_RED
  #define RGB565_RED 0xF800
#endif

const uint16_t BG_COLOR = RGB565_BLACK;
const uint16_t LABEL_COLOR = RGB565_WHITE;
const uint16_t PPM_COLOR = RGB565_CYAN;
const uint16_t RS_COLOR = RGB565_ORANGE;
const uint16_t RATIO_COLOR = RGB565_YELLOW;
const uint16_t TWA_COLOR = RGB565_GREEN;
const uint16_t ALERT_SAFE = RGB565_GREEN;
const uint16_t ALERT_CAUTION = RGB565_YELLOW;
const uint16_t ALERT_WARNING = RGB565_ORANGE;
const uint16_t ALERT_DANGER = RGB565_RED;

// store previous values to do partial updates
float prev_ppm = NAN;
float prev_rs = NAN;
float prev_ratio = NAN;
float prev_twa = NAN;
int prev_alert = -1;

// areas for partial update (computed in setup)
struct Area { int16_t x, y, w, h; };
Area areaPPM, areaRs, areaRatio, areaTWA, areaAlert;

// label/value layout params (tweak these if you like)
const char *labels[] = { "PPM:", "Rs:", "Ratio:", "TWA:", "Alert:" };
const uint8_t LABEL_TEXTSIZE = 1;   // labels font size
const uint8_t VAL_TEXTSIZE_PPM = 2; // PPM bigger
const uint8_t VAL_TEXTSIZE_OTHER = 1;
const int16_t LEFT_MARGIN = 20;     // distance from left edge of screen to label column
const int16_t LABEL_VAL_PADDING = 6; // px between label and value

// thresholds to decide if we redraw a value
const float TH_PPM = 0.01;   // small threshold for ppm (adjust if noisy)
const float TH_RS  = 1.0;    // ohms
const float TH_RATIO = 0.01;
const float TH_TWA = 0.01;

unsigned long lastReadMillis = 0;
const unsigned long READ_INTERVAL = 800; // ms, update rate (adjust as wanted)

// helper to clear an area (fill with bg)
inline void clearArea(const Area &a) {
  gfx->startWrite();
  gfx->fillRect(a.x, a.y, a.w, a.h, BG_COLOR);
  gfx->endWrite();
}

// helper to draw text into area (we assume area big enough)
void drawValueAt(int16_t x, int16_t y, const char *txt, uint16_t color, uint8_t textSize=2) {
  gfx->startWrite();
  gfx->setTextSize(textSize);
  gfx->setTextColor(color);
  gfx->setCursor(x, y);
  gfx->print(txt);
  gfx->endWrite();
}

// calculate pixel width of a string in default font: approx chars * 6 * textSize
inline int textPixelWidth(const char *s, uint8_t textSize) {
  return strlen(s) * 6 * textSize;
}

void drawStaticBackground()
{
  gfx->fillScreen(BG_COLOR);

  // Draw main circle (so content obviously inside)
  gfx->fillCircle(cx, cy, radius, BG_COLOR);
  gfx->drawCircle(cx, cy, radius - 1, RGB565_WHITE);

  // Title (top)
  gfx->setTextSize(2);
  gfx->setTextColor(LABEL_COLOR);
  gfx->setCursor(cx - 25, cy - radius + 16);
  gfx->print("NH3");

  // static labels column
  gfx->setTextSize(LABEL_TEXTSIZE);
  gfx->setTextColor(LABEL_COLOR);

  int16_t labelX = LEFT_MARGIN;
  int16_t startY = cy - 40;
  int16_t step = 26;

  for (uint8_t i = 0; i < 5; i++) {
    gfx->setCursor(labelX, startY + i * step);
    gfx->print(labels[i]);
  }
}

void computeAreas()
{
  // compute labelX and valueX (value starts after label + padding)
  int16_t labelX = LEFT_MARGIN;
  int16_t startY = cy - 40;
  int16_t step = 26;

  // For each field compute area that will hold the value (we'll clear this area when updating)
  // area width chosen large enough to hold number string; adjust if needed
  int16_t maxValW = 120;
  int16_t valH = 18;

  // PPM (bigger text, so slightly taller)
  int16_t labelW0 = textPixelWidth(labels[0], LABEL_TEXTSIZE);
  int16_t valueX0 = labelX + labelW0 + LABEL_VAL_PADDING;
  areaPPM = { valueX0, startY - 2, maxValW, valH + 8 };

  // Rs
  int16_t labelW1 = textPixelWidth(labels[1], LABEL_TEXTSIZE);
  int16_t valueX1 = labelX + labelW1 + LABEL_VAL_PADDING;
  areaRs = { valueX1, startY + step - 2, maxValW, valH };

  // Ratio
  int16_t labelW2 = textPixelWidth(labels[2], LABEL_TEXTSIZE);
  int16_t valueX2 = labelX + labelW2 + LABEL_VAL_PADDING;
  areaRatio = { valueX2, startY + step*2 - 2, maxValW, valH };

  // TWA
  int16_t labelW3 = textPixelWidth(labels[3], LABEL_TEXTSIZE);
  int16_t valueX3 = labelX + labelW3 + LABEL_VAL_PADDING;
  areaTWA = { valueX3, startY + step*3 - 2, maxValW, valH };

  // Alert
  int16_t labelW4 = textPixelWidth(labels[4], LABEL_TEXTSIZE);
  int16_t valueX4 = labelX + labelW4 + LABEL_VAL_PADDING;
  areaAlert = { valueX4, startY + step*4 - 2, maxValW, valH };
}

void updateAlertDisplay(int alertLevel)
{
  // map alert enum to color & string
  const char *str;
  uint16_t color;
  switch (alertLevel) {
    case SAFE: str = "SAFE"; color = ALERT_SAFE; break;
    case CAUTION: str = "CAUTION"; color = ALERT_CAUTION; break;
    case WARNING: str = "WARNING"; color = ALERT_WARNING; break;
    case DANGER: str = "DANGER"; color = ALERT_DANGER; break;
    default: str = "N/A"; color = LABEL_COLOR; break;
  }
  clearArea(areaAlert);
  // vertically center calc: use areaAlert.y as top; pick small offset
  drawValueAt(areaAlert.x, areaAlert.y + 2, str, color, 2);
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  // init TFT
  gfx->begin();
  scr_w = gfx->width();
  scr_h = gfx->height();
  cx = scr_w / 2;
  cy = scr_h / 2;
  radius = min(scr_w, scr_h) / 2 - 4; // leave tiny margin

  computeAreas();
  drawStaticBackground();

  // helper hint
  gfx->setTextSize(1);
  gfx->setTextColor(RGB565_WHITE);
  gfx->setCursor(6, scr_h - 16);
  gfx->print("Aligned values - partial updates");

  // sensor init
  sensor.begin();
  if (!sensor.isCalibrated()) {
    Serial.println("Sensor not calibrated, calibrating (auto)...");
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565_YELLOW);
    gfx->setCursor(cx - 40, cy);
    gfx->print("Calibrating R0...");
    sensor.calibrateR0(100, 100);
    delay(200);
    drawStaticBackground();
  } else {
    Serial.print("R0: ");
    Serial.println(sensor.getR0());
  }

  lastReadMillis = millis();
}

void loop()
{
  unsigned long now = millis();
  if (now - lastReadMillis < READ_INTERVAL) {
    return;
  }
  lastReadMillis = now;

  // Warm-up handling
  if (!sensor.isWarmedUp())
  {
    unsigned long remaining = sensor.getWarmupRemaining() / 1000;
    char wb[32];
    sprintf(wb, "Warming: %lus", remaining);
    Area warmArea = { cx - 60, cy - 10, 120, 16 };
    clearArea(warmArea);
    drawValueAt(warmArea.x, warmArea.y, wb, RGB565_YELLOW, 1);
    Serial.print("Warming... ");
    Serial.print(remaining);
    Serial.println("s");
    return;
  }

  // read sensor values
  float ppm = sensor.getPPM();
  float Rs = sensor.getRs();
  float ratio = sensor.getRatio();
  sensor.updateTWA();
  float twa = sensor.getTWA();
  AlertLevel alert = sensor.getAlertLevel();

  if (isnan(ppm))
  {
    clearArea(areaPPM);
    drawValueAt(areaPPM.x, areaPPM.y + 2, "Sensor Err", RGB565_RED, 2);
    Serial.println("Sensor read error");
    return;
  }

  // temp buffer
  char buf[32];

  // PPM (bigger)
  if (isnan(prev_ppm) || fabs(ppm - prev_ppm) > TH_PPM) {
    dtostrf(ppm, 5, 2, buf);
    clearArea(areaPPM);
    // place value using computed area
    drawValueAt(areaPPM.x, areaPPM.y + 2, buf, PPM_COLOR, VAL_TEXTSIZE_PPM);
    prev_ppm = ppm;
  }

  // Rs
  if (isnan(prev_rs) || fabs(Rs - prev_rs) > TH_RS) {
    dtostrf(Rs, 6, 2, buf);
    clearArea(areaRs);
    drawValueAt(areaRs.x, areaRs.y + 2, buf, RS_COLOR, VAL_TEXTSIZE_OTHER);
    prev_rs = Rs;
  }

  // Ratio
  if (isnan(prev_ratio) || fabs(ratio - prev_ratio) > TH_RATIO) {
    dtostrf(ratio, 5, 3, buf);
    clearArea(areaRatio);
    drawValueAt(areaRatio.x, areaRatio.y + 2, buf, RATIO_COLOR, VAL_TEXTSIZE_OTHER);
    prev_ratio = ratio;
  }

  // TWA
  if (isnan(prev_twa) || fabs(twa - prev_twa) > TH_TWA) {
    dtostrf(twa, 5, 2, buf);
    clearArea(areaTWA);
    drawValueAt(areaTWA.x, areaTWA.y + 2, buf, TWA_COLOR, VAL_TEXTSIZE_OTHER);
    prev_twa = twa;
  }

  // Alert
  if (alert != prev_alert) {
    updateAlertDisplay(alert);
    prev_alert = alert;
  }

  // Serial full log
  Serial.println(F("--- NH3 Reading ---"));
  Serial.print(F("PPM: ")); Serial.print(ppm, 2); Serial.println(F(" ppm"));
  Serial.print(F("Rs: ")); Serial.print(Rs, 2); Serial.println(F(" Ω"));
  Serial.print(F("Ratio (Rs/R0): ")); Serial.println(ratio, 3);
  Serial.print(F("Alert Level: ")); Serial.println(sensor.getAlertString());
  Serial.print(F("TWA: ")); Serial.print(twa, 2); Serial.println(F(" ppm"));
  Serial.println();
}
