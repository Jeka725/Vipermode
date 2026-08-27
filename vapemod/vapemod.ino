/*
  =====================================================================
  ESP32-C3 Super Mini — Прошивка для самодельного пода (Vape Mod)
  UI в стиле Smoant Pasito 2, неблокирующий код (millis(), без delay())

  v1.1 — ИСПРАВЛЕНИЯ:
    1. Новый LEDC API (ESP32 Arduino core 3.x): ledcAttach(pin, freq, bits)
       вместо ledcSetup()+ledcAttachPin(); ledcWrite(pin, duty) вместо
       ledcWrite(channel, duty).
    2. Правильная физика ШИМ-чоппера: т.к. фильтра (LC) нет, спираль в
       каждый момент видит либо 0, либо полное Vbat. Среднее по времени
       от V² равно D*Vbat², а НЕ (D*Vbat)². Поэтому duty считается из
       отношения желаемой мощности к максимально возможной (Vbat²/R),
       а не из отношения напряжений.
  =====================================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <math.h>

// ============================== ПИНЫ ================================
#define GPIO_MOSFET      4     // ШИМ на затвор MOSFET 60N03P
#define GPIO_BUTTON      5     // Кнопка (PULLDOWN, HIGH = нажата)
#define GPIO_PUFF        6     // Датчик автозатяжки (HIGH = сработал)
#define GPIO_BATTERY     3     // АЦП делителя напряжения АКБ

#define OLED_SDA         8
#define OLED_SCL         9
#define OLED_WIDTH       128
#define OLED_HEIGHT      64
#define OLED_ADDR        0x3C

// ========================= БАЗОВЫЕ ПАРАМЕТРЫ =========================
#define COIL_RESISTANCE     0.6f   // Сопротивление спирали, Ом
#define MAX_PUFF_TIME_SEC   10     // Таймаут одной затяжки, сек
#define MIN_PUFF_TIME_MS    500    // Минимальная длительность для счётчика затяжек
#define MAX_WATTAGE         30     // Максимальная мощность, Вт
#define WATT_STEP           1      // Шаг регулировки мощности, Вт
#define WATT_STEP_INTERVAL  175    // Интервал шага регулировки, мс
#define LONG_PRESS_MS       1000   // Порог долгого нажатия, мс
#define DEBOUNCE_MS         30     // Время антидребезга, мс
#define RESET_CLICKS        5      // Количество кликов для сброса счётчика
#define RESET_CLICK_WINDOW  1500   // Окно времени для серии кликов, мс

// Параметры АКБ (Li-Ion)
#define BAT_V_MIN           3.0f
#define BAT_V_MAX           4.2f
#define ADC_VREF            3.3f
#define ADC_RESOLUTION      4095.0f
#define VOLTAGE_DIVIDER_RATIO 2.0f  // подбери под свою схему (10к/10к -> 2.0)

// PWM (LEDC) параметры — новый API работает напрямую с пином, без каналов
#define PWM_FREQ             20000   // 20 кГц — выше слышимого диапазона
#define PWM_RESOLUTION_BITS  8       // 0-255
#define PWM_MAX_DUTY         255

// ============================ ОБЪЕКТЫ ================================
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Preferences prefs;

// ============================ СОСТОЯНИЯ ===============================
enum FireSource { FIRE_NONE, FIRE_BUTTON, FIRE_PUFF };

bool buttonRawState = LOW;
bool buttonStableState = LOW;
unsigned long buttonLastChangeTime = 0;
unsigned long buttonPressStartTime = 0;
bool longPressActive = false;
bool longPressFired = false;

uint8_t clickCount = 0;
unsigned long firstClickTime = 0;

unsigned long lastWattStepTime = 0;

bool puffRawState = LOW;
bool puffStableState = LOW;
unsigned long puffLastChangeTime = 0;

bool isFiring = false;
FireSource fireSource = FIRE_NONE;
unsigned long fireStartTime = 0;
unsigned long lastPuffDurationMs = 0;

int wattage = 15;
uint32_t puffCount = 0;
float batteryVoltage = 4.2f;

// ============================ ПРОТОТИПЫ ================================
void handleButton();
void handlePuffSensor();
void startFiring(FireSource source);
void stopFiring();
void updatePower();
float readBatteryVoltage();
uint8_t batteryPercent(float v);
void applyPWM(float power, float vBat);
void drawUI();
void loadSettings();
void saveWattage();
void savePuffCount();
void resetPuffCounter();

// ================================================================
//                            SETUP
// ================================================================
void setup() {
  Serial.begin(115200);

  pinMode(GPIO_BUTTON, INPUT);
  pinMode(GPIO_PUFF, INPUT);

  ledcAttach(GPIO_MOSFET, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcWrite(GPIO_MOSFET, 0);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("Ошибка инициализации OLED!");
  }
  display.clearDisplay();
  display.display();

  loadSettings();
}

// ================================================================
//                            LOOP
// ================================================================
void loop() {
  handleButton();
  handlePuffSensor();
  updatePower();
  batteryVoltage = readBatteryVoltage();
  drawUI();
}

// ================================================================
//                    ОБРАБОТКА КНОПКИ (Debounce + Long Press)
// ================================================================
void handleButton() {
  bool reading = digitalRead(GPIO_BUTTON);

  if (reading != buttonRawState) {
    buttonLastChangeTime = millis();
    buttonRawState = reading;
  }

  if ((millis() - buttonLastChangeTime) > DEBOUNCE_MS) {
    if (buttonRawState != buttonStableState) {
      buttonStableState = buttonRawState;

      if (buttonStableState == HIGH) {
        buttonPressStartTime = millis();
        longPressActive = false;
        longPressFired = false;
      } else {
        unsigned long pressDuration = millis() - buttonPressStartTime;

        if (longPressFired) {
          saveWattage();
          longPressActive = false;
        } else if (pressDuration < LONG_PRESS_MS) {
          if (isFiring && fireSource == FIRE_BUTTON) {
            stopFiring();
          } else if (!isFiring) {
            startFiring(FIRE_BUTTON);
          }

          unsigned long now = millis();
          if (clickCount == 0 || (now - firstClickTime) <= RESET_CLICK_WINDOW) {
            if (clickCount == 0) firstClickTime = now;
            clickCount++;
            if (clickCount >= RESET_CLICKS) {
              resetPuffCounter();
              clickCount = 0;
            }
          } else {
            clickCount = 1;
            firstClickTime = now;
          }
        }
      }
    }
  }

  if (buttonStableState == HIGH && !longPressFired) {
    if ((millis() - buttonPressStartTime) >= LONG_PRESS_MS) {
      longPressFired = true;
      longPressActive = true;

      if (isFiring && fireSource == FIRE_BUTTON) {
        stopFiring();
      }
    }
  }

  if (longPressActive) {
    if (millis() - lastWattStepTime >= WATT_STEP_INTERVAL) {
      lastWattStepTime = millis();
      wattage += WATT_STEP;
      if (wattage > MAX_WATTAGE) {
        wattage = 0;
      }
    }
  }
}

// ================================================================
//                 ОБРАБОТКА ДАТЧИКА АВТОЗАТЯЖКИ
// ================================================================
void handlePuffSensor() {
  bool reading = digitalRead(GPIO_PUFF);

  if (reading != puffRawState) {
    puffLastChangeTime = millis();
    puffRawState = reading;
  }

  if ((millis() - puffLastChangeTime) > DEBOUNCE_MS) {
    if (puffRawState != puffStableState) {
      puffStableState = puffRawState;

      if (puffStableState == HIGH) {
        if (!isFiring) {
          startFiring(FIRE_PUFF);
        }
      } else {
        if (isFiring && fireSource == FIRE_PUFF) {
          stopFiring();
        }
      }
    }
  }
}

// ================================================================
//                      СТАРТ / СТОП ЖАРКИ
// ================================================================
void startFiring(FireSource source) {
  isFiring = true;
  fireSource = source;
  fireStartTime = millis();
}

void stopFiring() {
  unsigned long duration = millis() - fireStartTime;
  isFiring = false;
  fireSource = FIRE_NONE;
  lastPuffDurationMs = duration;

  ledcWrite(GPIO_MOSFET, 0);

  if (duration >= MIN_PUFF_TIME_MS) {
    puffCount++;
    savePuffCount();
  }
}

// ================================================================
//         ОБНОВЛЕНИЕ МОЩНОСТИ / ТАЙМАУТ / ПОДАЧА ШИМ
// ================================================================
void updatePower() {
  if (isFiring) {
    unsigned long elapsedMs = millis() - fireStartTime;

    if (elapsedMs >= (unsigned long)MAX_PUFF_TIME_SEC * 1000UL) {
      stopFiring();
      return;
    }

    applyPWM((float)wattage, batteryVoltage);
  }
}

// ================================================================
//        ПРИМЕНЕНИЕ ШИМ НА MOSFET (расчёт по МОЩНОСТИ, не по V)
// ================================================================
void applyPWM(float power, float vBat) {
  if (vBat <= 0.01f) return;

  float maxPower = (vBat * vBat) / COIL_RESISTANCE;

  float dutyF = (power / maxPower) * (float)PWM_MAX_DUTY;

  if (dutyF > PWM_MAX_DUTY) dutyF = PWM_MAX_DUTY; // физический потолок (100% duty)
  if (dutyF < 0) dutyF = 0;

  ledcWrite(GPIO_MOSFET, (uint8_t)dutyF);
}

// ================================================================
//                  ЧТЕНИЕ НАПРЯЖЕНИЯ АКБ (АЦП)
// ================================================================
float readBatteryVoltage() {
  int raw = analogRead(GPIO_BATTERY);
  float vAdc = (raw / ADC_RESOLUTION) * ADC_VREF;
  float vBat = vAdc * VOLTAGE_DIVIDER_RATIO;

  if (vBat < BAT_V_MIN) vBat = BAT_V_MIN;
  if (vBat > BAT_V_MAX) vBat = BAT_V_MAX;

  return vBat;
}

// ================================================================
//                  ПРОЦЕНТ ЗАРЯДА АКБ (линейно)
// ================================================================
uint8_t batteryPercent(float v) {
  float pct = (v - BAT_V_MIN) / (BAT_V_MAX - BAT_V_MIN) * 100.0f;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (uint8_t)pct;
}

// ================================================================
//                    ОТРИСОВКА UI (Smoant Pasito 2 style)
// ================================================================
void drawUI() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  uint8_t pct = batteryPercent(batteryVoltage);

  int battX = 0, battY = 0, battW = 24, battH = 10;
  display.drawRect(battX, battY, battW, battH, SSD1306_WHITE);
  display.fillRect(battX + battW, battY + 3, 2, 4, SSD1306_WHITE);

  int segments = 4;
  int filledSegments = map(pct, 0, 100, 0, segments);
  int segW = (battW - 4) / segments;
  for (int i = 0; i < segments; i++) {
    if (i < filledSegments) {
      display.fillRect(battX + 2 + i * segW, battY + 2, segW - 1, battH - 4, SSD1306_WHITE);
    }
  }

  display.setTextSize(1);
  display.setCursor(battX + battW + 6, battY + 1);
  display.print(pct);
  display.print("%");

  display.setCursor(OLED_WIDTH - 40, 0);
  display.print("POWER");

  display.drawLine(0, 12, OLED_WIDTH - 1, 12, SSD1306_WHITE);

  char wattStr[8];
  snprintf(wattStr, sizeof(wattStr), "%d W", wattage);

  display.setTextSize(3);
  int16_t x1, y1;
  uint16_t wTxt, hTxt;
  display.getTextBounds(wattStr, 0, 0, &x1, &y1, &wTxt, &hTxt);
  display.setCursor((OLED_WIDTH - wTxt) / 2, 18);
  display.print(wattStr);

  float vOutDisplay = sqrtf((float)wattage * COIL_RESISTANCE);
  if (vOutDisplay > batteryVoltage) vOutDisplay = batteryVoltage;

  char vOutStr[10];
  snprintf(vOutStr, sizeof(vOutStr), "%.2f V", vOutDisplay);

  display.setTextSize(1);
  display.getTextBounds(vOutStr, 0, 0, &x1, &y1, &wTxt, &hTxt);
  display.setCursor((OLED_WIDTH - wTxt) / 2, 44);
  display.print(vOutStr);

  display.drawLine(0, 53, OLED_WIDTH - 1, 53, SSD1306_WHITE);

  float puffSeconds;
  if (isFiring) {
    puffSeconds = (millis() - fireStartTime) / 1000.0f;
  } else {
    puffSeconds = lastPuffDurationMs / 1000.0f;
  }

  char timerStr[8];
  snprintf(timerStr, sizeof(timerStr), "%.1fs", puffSeconds);
  display.setCursor(2, 56);
  display.print(timerStr);

  if (!isFiring && lastPuffDurationMs >= (unsigned long)MAX_PUFF_TIME_SEC * 1000UL) {
    display.setCursor(30, 56);
    display.print("TIME OVER");
  } else {
    char coilStr[12];
    snprintf(coilStr, sizeof(coilStr), "%.2f Ohm", COIL_RESISTANCE);
    display.setCursor(46, 56);
    display.print(coilStr);
  }

  char puffStr[14];
  snprintf(puffStr, sizeof(puffStr), "P:%04lu", (unsigned long)puffCount);
  int16_t px1, py1; uint16_t pw, ph;
  display.getTextBounds(puffStr, 0, 0, &px1, &py1, &pw, &ph);
  display.setCursor(OLED_WIDTH - pw - 2, 56);
  display.print(puffStr);

  display.display();
}

// ================================================================
//              РАБОТА С ФЛЕШ-ПАМЯТЬЮ (Preferences.h)
// ================================================================
void loadSettings() {
  prefs.begin("vapemod", false);
  wattage = prefs.getInt("watt", 15);
  puffCount = prefs.getUInt("puffs", 0);
  prefs.end();
}

void saveWattage() {
  prefs.begin("vapemod", false);
  prefs.putInt("watt", wattage);
  prefs.end();
}

void savePuffCount() {
  prefs.begin("vapemod", false);
  prefs.putUInt("puffs", puffCount);
  prefs.end();
}

void resetPuffCounter() {
  puffCount = 0;
  savePuffCount();
}
