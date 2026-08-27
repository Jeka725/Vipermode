/*
  =====================================================================
  ESP32-C3 Super Mini — Прошивка для самодельного пода (Vape Mod)
  v1.2 — U8g2 вместо Adafruit_GFX + автопроверка сопротивления картриджа
  Неблокирующий код (millis()), кроме микро-импульса проверки картриджа
  (250 мкс — намеренное исключение, см. комментарий в checkCoilResistance()).
  =====================================================================
*/

#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <math.h>

// ============================== ПИНЫ ================================
#define GPIO_MOSFET          4     // ШИМ на затвор MOSFET 60N03P
#define GPIO_BUTTON          5     // Кнопка (PULLDOWN, HIGH = нажата)
#define GPIO_PUFF            6     // Датчик автозатяжки (HIGH = сработал)
#define GPIO_BATTERY         3     // АЦП делителя напряжения АКБ
#define GPIO_CURRENT_SENSE   1     // АЦП шунта (верхний вывод шунта в GND-цепи катушки)

#define OLED_SDA             8
#define OLED_SCL             9
#define OLED_WIDTH           128
#define OLED_HEIGHT          64
#define OLED_ADDR            0x3C

// ========================= БАЗОВЫЕ ПАРАМЕТРЫ =========================
#define COIL_RESISTANCE      0.6f   // Сопротивление спирали по умолчанию (пока не измерено), Ом
#define MAX_PUFF_TIME_SEC    10     // Таймаут одной затяжки, сек
#define MIN_PUFF_TIME_MS     500    // Минимальная длительность для счётчика затяжек
#define MAX_WATTAGE          30     // Максимальная мощность, Вт
#define WATT_STEP            1      // Шаг регулировки мощности, Вт
#define WATT_STEP_INTERVAL   175    // Интервал шага регулировки, мс
#define LONG_PRESS_MS        1000   // Порог долгого нажатия, мс
#define DEBOUNCE_MS          30     // Время антидребезга, мс
#define RESET_CLICKS         5      // Количество кликов для сброса счётчика затяжек
#define RESET_CLICK_WINDOW   1500   // Окно времени для серии кликов, мс

// Параметры АКБ (Li-Ion)
#define BAT_V_MIN             3.0f
#define BAT_V_MAX             4.2f
#define ADC_VREF               3.3f
#define ADC_RESOLUTION       4095.0f
#define VOLTAGE_DIVIDER_RATIO   2.0f  // подбери под свою схему (10к/10к -> 2.0)

// PWM (LEDC) — новый API, работает напрямую с пином, без каналов
#define PWM_FREQ              20000
#define PWM_RESOLUTION_BITS   8
#define PWM_MAX_DUTY          255

// ------------------ Параметры проверки картриджа ------------------
#define SHUNT_RESISTANCE      0.1f    // Ом, точный шунт в GND-возврате катушки
#define COIL_CHECK_DUTY        255    // Полная скважность на время теста (максимум сигнал/шум)
#define COIL_CHECK_PULSE_US    250    // Длительность тестового импульса, мкс (намеренно очень коротко)
#define COIL_CHECK_INTERVAL_MS 30000  // Как часто перепроверять картридж в простое, мс
#define COIL_SHORT_OHM         0.15f  // Ниже этого — короткое замыкание
#define COIL_OPEN_OHM          3.5f   // Выше этого — картридж не подключен/обрыв
#define COIL_MIN_CURRENT_A     0.05f  // Ниже этого тока при тесте — обрыв цепи

// ============================ ОБЪЕКТЫ ================================
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
Preferences prefs;

// ============================ СОСТОЯНИЯ ===============================
enum FireSource { FIRE_NONE, FIRE_BUTTON, FIRE_PUFF };
enum CoilState  { COIL_UNKNOWN, COIL_OK, COIL_SHORT, COIL_OPEN };

// --- Кнопка ---
bool buttonRawState = LOW;
bool buttonStableState = LOW;
unsigned long buttonLastChangeTime = 0;
unsigned long buttonPressStartTime = 0;
bool longPressActive = false;
bool longPressFired = false;

// --- Серия кликов для сброса счётчика ---
uint8_t clickCount = 0;
unsigned long firstClickTime = 0;

// --- Регулировка мощности ---
unsigned long lastWattStepTime = 0;

// --- Датчик автозатяжки ---
bool puffRawState = LOW;
bool puffStableState = LOW;
unsigned long puffLastChangeTime = 0;

// --- Общее состояние жарки ---
bool isFiring = false;
FireSource fireSource = FIRE_NONE;
unsigned long fireStartTime = 0;
unsigned long lastPuffDurationMs = 0;

// --- Параметры устройства ---
int wattage = 15;
uint32_t puffCount = 0;
float batteryVoltage = 4.2f;

// --- Картридж ---
CoilState coilState = COIL_UNKNOWN;
float coilResistance = COIL_RESISTANCE; // реальное измеренное сопротивление
unsigned long lastCoilCheckTime = 0;

// ============================ ПРОТОТИПЫ ================================
void handleButton();
void handlePuffSensor();
void startFiring(FireSource source);
void stopFiring();
void updatePower();
float readBatteryVoltage();
uint8_t batteryPercent(float v);
void applyPWM(float power, float vBat);
void checkCoilResistance();
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
  u8g2.begin();
  u8g2.setBusClock(400000);

  loadSettings();

  // Проверяем картридж сразу при включении, до первой возможной затяжки
  batteryVoltage = readBatteryVoltage();
  checkCoilResistance();
}

// ================================================================
//                            LOOP
// ================================================================
void loop() {
  handleButton();
  handlePuffSensor();
  updatePower();
  batteryVoltage = readBatteryVoltage();

  // Периодическая перепроверка картриджа в простое (например, после
  // смены картриджа без выключения устройства)
  if (!isFiring && (millis() - lastCoilCheckTime >= COIL_CHECK_INTERVAL_MS)) {
    checkCoilResistance();
  }

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
  // Защита: не поджигаем, если картридж в коротком замыкании или не подключен
  if (coilState == COIL_SHORT || coilState == COIL_OPEN) {
    return;
  }

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

  // Максимальная мощность при текущем Vbat и измеренном сопротивлении спирали
  float maxPower = (vBat * vBat) / coilResistance;

  // "Сырой" ШИМ-чоппер без LC-фильтра: P_avg = D * Vbat^2 / R
  float dutyF = (power / maxPower) * (float)PWM_MAX_DUTY;

  if (dutyF > PWM_MAX_DUTY) dutyF = PWM_MAX_DUTY;
  if (dutyF < 0) dutyF = 0;

  ledcWrite(GPIO_MOSFET, (uint8_t)dutyF);
}

// ================================================================
//         ПРОВЕРКА КАРТРИДЖА: измерение реального сопротивления
// ================================================================
void checkCoilResistance() {
  float vBat = readBatteryVoltage();

  // Короткий импульс полной мощности — намеренно предельно короткий (250 мкс),
  // чтобы (а) спираль физически не успела нагреться, (б) энергия импульса
  // была пренебрежимо мала даже в худшем случае короткого замыкания.
  ledcWrite(GPIO_MOSFET, COIL_CHECK_DUTY);
  delayMicroseconds(COIL_CHECK_PULSE_US);
  int raw = analogRead(GPIO_CURRENT_SENSE);
  ledcWrite(GPIO_MOSFET, 0);

  float vShunt = (raw / ADC_RESOLUTION) * ADC_VREF;
  float current = vShunt / SHUNT_RESISTANCE;

  if (current < COIL_MIN_CURRENT_A) {
    // Тока почти нет — цепь разомкнута (картридж не вставлен / обрыв спирали)
    coilState = COIL_OPEN;
  } else {
    float vCoil = vBat - vShunt;
    float r = vCoil / current;

    if (r < COIL_SHORT_OHM) {
      coilState = COIL_SHORT;       // Короткое замыкание — жарить нельзя
    } else if (r > COIL_OPEN_OHM) {
      coilState = COIL_OPEN;
    } else {
      coilState = COIL_OK;
      coilResistance = r;           // Сохраняем реальное сопротивление для расчёта мощности
    }
  }

  lastCoilCheckTime = millis();
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
//                    ОТРИСОВКА UI (U8g2, Pasito 2 style)
// ================================================================
void drawUI() {
  char buf[16];

  u8g2.clearBuffer();

  // ---------- Верхний Status Bar ----------
  uint8_t pct = batteryPercent(batteryVoltage);

  int battX = 0, battY = 0, battW = 24, battH = 10;
  u8g2.drawFrame(battX, battY, battW, battH);
  u8g2.drawBox(battX + battW, battY + 3, 2, 4);

  int segments = 4;
  int filledSegments = map(pct, 0, 100, 0, segments);
  int segW = (battW - 4) / segments;
  for (int i = 0; i < segments; i++) {
    if (i < filledSegments) {
      u8g2.drawBox(battX + 2 + i * segW, battY + 2, segW - 1, battH - 4);
    }
  }

  u8g2.setFont(u8g2_font_6x10_tf);
  snprintf(buf, sizeof(buf), "%d%%", pct);
  u8g2.setCursor(battW + 6, 9);
  u8g2.print(buf);

  const char* modeLabel = "POWER";
  int mw = u8g2.getStrWidth(modeLabel);
  u8g2.drawStr(OLED_WIDTH - mw, 9, modeLabel);

  u8g2.drawHLine(0, 12, OLED_WIDTH);

  // ---------- Центральная часть ----------
  if (coilState == COIL_SHORT || coilState == COIL_OPEN) {
    // Предупреждение о картридже перекрывает обычный дисплей мощности
    u8g2.setFont(u8g2_font_logisoso16_tf);
    const char* warn = (coilState == COIL_SHORT) ? "SHORT!" : "NO ATOM";
    int ww = u8g2.getStrWidth(warn);
    u8g2.setCursor((OLED_WIDTH - ww) / 2, 38);
    u8g2.print(warn);
  } else {
    snprintf(buf, sizeof(buf), "%d W", wattage);
    u8g2.setFont(u8g2_font_logisoso24_tf);
    int ww = u8g2.getStrWidth(buf);
    u8g2.setCursor((OLED_WIDTH - ww) / 2, 40);
    u8g2.print(buf);

    float vOutDisplay = sqrtf((float)wattage * coilResistance);
    if (vOutDisplay > batteryVoltage) vOutDisplay = batteryVoltage;

    snprintf(buf, sizeof(buf), "%.2f V", vOutDisplay);
    u8g2.setFont(u8g2_font_6x10_tf);
    ww = u8g2.getStrWidth(buf);
    u8g2.setCursor((OLED_WIDTH - ww) / 2, 50);
    u8g2.print(buf);
  }

  u8g2.drawHLine(0, 53, OLED_WIDTH);

  // ---------- Нижняя часть ----------
  u8g2.setFont(u8g2_font_6x10_tf);

  float puffSeconds = isFiring
                         ? (millis() - fireStartTime) / 1000.0f
                         : lastPuffDurationMs / 1000.0f;
  snprintf(buf, sizeof(buf), "%.1fs", puffSeconds);
  u8g2.setCursor(2, 63);
  u8g2.print(buf);

  if (!isFiring && lastPuffDurationMs >= (unsigned long)MAX_PUFF_TIME_SEC * 1000UL) {
    u8g2.setCursor(30, 63);
    u8g2.print("TIME OVER");
  } else {
    snprintf(buf, sizeof(buf), "%.2f Ohm", coilResistance);
    u8g2.setCursor(46, 63);
    u8g2.print(buf);
  }

  snprintf(buf, sizeof(buf), "P:%04lu", (unsigned long)puffCount);
  int pw = u8g2.getStrWidth(buf);
  u8g2.setCursor(OLED_WIDTH - pw - 2, 63);
  u8g2.print(buf);

  u8g2.sendBuffer();
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
