/*
  =====================================================================
  ESP32-C3 Super Mini — Прошивка для самодельного пода (Vape Mod)
  v1.5 — U8g2, интерфейс в стиле Voopoo Drag X
  БЕЗ измерения напряжения АКБ (нет ADC на батарею), БЕЗ автозатяжки.
  Firing только по кнопке (hold-to-fire).
  Расчёт мощности идёт от ФИКСИРОВАННОГО номинального напряжения АКБ —
  см. BATTERY_NOMINAL_VOLTAGE и комментарий в applyPWM().
  Неблокирующий код (millis())
  =====================================================================
*/

#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <math.h>

// ============================== ПИНЫ ================================
#define GPIO_MOSFET          4     // ШИМ на затвор MOSFET 60N03P
#define GPIO_BUTTON          5     // Кнопка (PULLDOWN, HIGH = нажата)

#define OLED_SDA              8
#define OLED_SCL              9
#define OLED_WIDTH           128
#define OLED_HEIGHT           64
#define OLED_ADDR            0x3C

// ========================= БАЗОВЫЕ ПАРАМЕТРЫ =========================
#define COIL_RESISTANCE      0.6f   // Сопротивление спирали, Ом (фикс.)
#define MAX_PUFF_TIME_SEC     10    // Аварийный таймаут одной затяжки, сек
#define MIN_PUFF_TIME_MS     500    // Мин. длительность для счётчика затяжек
#define MAX_WATTAGE           30    // Максимальная мощность, Вт
#define WATT_STEP              1    // Шаг регулировки мощности, Вт
#define WATT_STEP_INTERVAL   175    // Интервал шага регулировки, мс
#define DEBOUNCE_MS            30   // Время антидребезга, мс

// --- Жест входа в режим настройки мощности: 3 быстрых клика подряд ---
#define CLICK_MAX_MS          350   // Короче этого — считается "клик", не затяжка
#define CLICK_WINDOW_MS       500   // Пауза между кликами, после которой считаем серию
#define WATT_SET_CLICKS         3   // Столько кликов подряд -> вход в настройку Вт
#define PUFF_RESET_CLICKS       5   // Столько кликов подряд -> сброс счётчика затяжек

// Фиксированное "усреднённое" напряжение АКБ, т.к. измерения больше нет.
// Подбери под свой аккум/стиль парения — 3.85В это грубая середина разряда
// Li-Ion между 4.2В (полная) и 3.3-3.5В (подсевшая под нагрузкой).
#define BATTERY_NOMINAL_VOLTAGE  3.85f

// PWM (LEDC) — новый API, работает напрямую с пином, без каналов
#define PWM_FREQ               20000
#define PWM_RESOLUTION_BITS        8
#define PWM_MAX_DUTY              255

// ============================ ОБЪЕКТЫ ================================
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
Preferences prefs;

// ============================ СОСТОЯНИЯ ===============================
enum DeviceMode { MODE_NORMAL, MODE_WATT_SET };

DeviceMode deviceMode = MODE_NORMAL;

// --- Кнопка ---
bool buttonRawState = LOW;
bool buttonStableState = LOW;
unsigned long buttonLastChangeTime = 0;
unsigned long buttonPressStartTime = 0;

// --- Серия кликов (для входа в настройку Вт / сброса счётчика) ---
uint8_t clickCount = 0;
unsigned long lastClickTime = 0;

// --- Регулировка мощности (внутри MODE_WATT_SET) ---
unsigned long lastWattStepTime = 0;

// --- Общее состояние жарки ---
bool isFiring = false;
unsigned long fireStartTime = 0;
unsigned long lastPuffDurationMs = 0;

// --- Параметры устройства ---
int wattage = 15;
uint32_t puffCount = 0;

// ============================ ПРОТОТИПЫ ================================
void handleButton();
void evaluateClickSeries();
void startFiring();
void stopFiring();
void updatePower();
void applyPWM(float power);
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

  ledcAttach(GPIO_MOSFET, PWM_FREQ, PWM_RESOLUTION_BITS);
  ledcWrite(GPIO_MOSFET, 0);

  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();
  u8g2.setBusClock(400000);

  loadSettings();
}

// ================================================================
//                            LOOP
// ================================================================
void loop() {
  handleButton();
  evaluateClickSeries();
  updatePower();
  drawUI();
}

// ================================================================
//                    ОБРАБОТКА КНОПКИ (Debounce)
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

      // --- Кнопка только что нажата ---
      if (buttonStableState == HIGH) {
        buttonPressStartTime = millis();

        if (deviceMode == MODE_NORMAL) {
          // Обычный режим: нажал — сразу греет. Не зажата = не парит.
          startFiring();
        }
      }
      // --- Кнопка только что отпущена ---
      else {
        unsigned long pressDuration = millis() - buttonPressStartTime;

        if (deviceMode == MODE_NORMAL) {
          if (isFiring) {
            stopFiring();
          }
          if (pressDuration < CLICK_MAX_MS) {
            clickCount++;
            lastClickTime = millis();
          }
        } else { // MODE_WATT_SET
          saveWattage();
          deviceMode = MODE_NORMAL;
        }
      }
    }
  }

  // --- Пока кнопка удерживается в MODE_WATT_SET — крутим ватты по кругу ---
  if (deviceMode == MODE_WATT_SET && buttonStableState == HIGH) {
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
//   ОЦЕНКА СЕРИИ КЛИКОВ: 3 клика -> настройка Вт, 5 кликов -> сброс счётчика
// ================================================================
void evaluateClickSeries() {
  if (clickCount == 0) return;

  if (millis() - lastClickTime > CLICK_WINDOW_MS) {
    if (clickCount >= PUFF_RESET_CLICKS) {
      resetPuffCounter();
    } else if (clickCount == WATT_SET_CLICKS && deviceMode == MODE_NORMAL) {
      deviceMode = MODE_WATT_SET;
    }
    clickCount = 0;
  }
}

// ================================================================
//                      СТАРТ / СТОП ЖАРКИ
// ================================================================
void startFiring() {
  isFiring = true;
  fireStartTime = millis();
}

void stopFiring() {
  unsigned long duration = millis() - fireStartTime;
  isFiring = false;
  lastPuffDurationMs = duration;

  ledcWrite(GPIO_MOSFET, 0);

  if (duration >= MIN_PUFF_TIME_MS) {
    puffCount++;
    savePuffCount();
  }
}

// ================================================================
//         ОБНОВЛЕНИЕ МОЩНОСТИ / АВАРИЙНЫЙ ТАЙМАУТ / ПОДАЧА ШИМ
// ================================================================
void updatePower() {
  if (isFiring) {
    unsigned long elapsedMs = millis() - fireStartTime;

    if (elapsedMs >= (unsigned long)MAX_PUFF_TIME_SEC * 1000UL) {
      stopFiring();
      return;
    }

    applyPWM((float)wattage);
  }
}

// ================================================================
//   ПРИМЕНЕНИЕ ШИМ НА MOSFET (расчёт по МОЩНОСТИ от фикс. напряжения)
// ================================================================
void applyPWM(float power) {
  // Т.к. реального Vbat нет, считаем от фиксированной константы —
  // фактическая мощность будет плыть вместе с реальным разрядом банки
  // (выше выставленной на полной банке, ниже — на подсевшей).
  float maxPower = (BATTERY_NOMINAL_VOLTAGE * BATTERY_NOMINAL_VOLTAGE) / COIL_RESISTANCE;

  float dutyF = (power / maxPower) * (float)PWM_MAX_DUTY;

  if (dutyF > PWM_MAX_DUTY) dutyF = PWM_MAX_DUTY;
  if (dutyF < 0) dutyF = 0;

  ledcWrite(GPIO_MOSFET, (uint8_t)dutyF);
}

// ================================================================
//              ОТРИСОВКА UI (U8g2, стиль Voopoo Drag X)
// ================================================================
void drawUI() {
  char buf[16];

  u8g2.clearBuffer();

  // ---------- Верхний Status Bar (без батареи) ----------
  const char* modeLabel = "VW";
  u8g2.setFont(u8g2_font_6x10_tf);
  int mw = u8g2.getStrWidth(modeLabel);
  u8g2.drawStr(OLED_WIDTH - mw, 9, modeLabel);

  u8g2.drawHLine(0, 12, OLED_WIDTH);

  // ---------- Центральная часть: крупная мощность (Drag X style) ----------
  snprintf(buf, sizeof(buf), "%d", wattage);
  u8g2.setFont(u8g2_font_logisoso28_tf);
  int ww = u8g2.getStrWidth(buf);
  int wattX = (OLED_WIDTH - ww) / 2 - 6;
  u8g2.setCursor(wattX, 42);
  u8g2.print(buf);

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(wattX + ww + 4, 42);
  u8g2.print("W");

  // В режиме настройки мощности — рамка вокруг числа, как выделение на Drag X
  if (deviceMode == MODE_WATT_SET) {
    u8g2.drawRFrame(wattX - 6, 15, ww + 24, 30, 3);
  }

  // Сопротивление спирали мелким шрифтом под мощностью
  snprintf(buf, sizeof(buf), "%.2f Ohm", COIL_RESISTANCE);
  u8g2.setFont(u8g2_font_6x10_tf);
  ww = u8g2.getStrWidth(buf);
  u8g2.setCursor((OLED_WIDTH - ww) / 2, 51);
  u8g2.print(buf);

  u8g2.drawHLine(0, 53, OLED_WIDTH);

  // ---------- Нижняя часть: таймер затяжки / счётчик ----------
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
