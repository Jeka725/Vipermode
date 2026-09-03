/*
  =====================================================================
  ESP32-C3 Super Mini — Прошивка для самодельного пода (Vape Mod)
  v1.6 — Adafruit_SSD1306 (откат с U8g2, т.к. не завёлся), Drag X style
  БЕЗ измерения напряжения АКБ, БЕЗ автозатяжки.
  Firing только по кнопке (hold-to-fire).
  Неблокирующий код (millis())
  =====================================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
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

// Фиксированное "усреднённое" напряжение АКБ, т.к. измерения нет
#define BATTERY_NOMINAL_VOLTAGE  3.85f

// PWM (LEDC) — новый API, работает напрямую с пином, без каналов
#define PWM_FREQ               20000
#define PWM_RESOLUTION_BITS        8
#define PWM_MAX_DUTY              255

// ============================ ОБЪЕКТЫ ================================
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Preferences prefs;

// ============================ СОСТОЯНИЯ ===============================
enum DeviceMode { MODE_NORMAL, MODE_WATT_SET };

DeviceMode deviceMode = MODE_NORMAL;

// --- Кнопка ---
bool buttonRawState = LOW;
bool buttonStableState = LOW;
unsigned long buttonLastChangeTime = 0;
unsigned long buttonPressStartTime = 0;

// --- Серия кликов ---
uint8_t clickCount = 0;
unsigned long lastClickTime = 0;

// --- Регулировка мощности ---
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

      if (buttonStableState == HIGH) {
        buttonPressStartTime = millis();

        if (deviceMode == MODE_NORMAL) {
          startFiring();
        }
      } else {
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
  float maxPower = (BATTERY_NOMINAL_VOLTAGE * BATTERY_NOMINAL_VOLTAGE) / COIL_RESISTANCE;

  float dutyF = (power / maxPower) * (float)PWM_MAX_DUTY;

  if (dutyF > PWM_MAX_DUTY) dutyF = PWM_MAX_DUTY;
  if (dutyF < 0) dutyF = 0;

  ledcWrite(GPIO_MOSFET, (uint8_t)dutyF);
}

// ================================================================
//              ОТРИСОВКА UI (Adafruit_GFX, стиль Voopoo Drag X)
// ================================================================
void drawUI() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // ---------- Верхний Status Bar (без батареи) ----------
  display.setTextSize(1);
  display.setCursor(OLED_WIDTH - 18, 0);
  display.print("VW");

  display.drawLine(0, 12, OLED_WIDTH - 1, 12, SSD1306_WHITE);

  // ---------- Центральная часть: крупная мощность (Drag X style) ----------
  char wattStr[8];
  snprintf(wattStr, sizeof(wattStr), "%d", wattage);

  display.setTextSize(4);
  int16_t x1, y1;
  uint16_t wTxt, hTxt;
  display.getTextBounds(wattStr, 0, 0, &x1, &y1, &wTxt, &hTxt);
  int wattX = (OLED_WIDTH - wTxt) / 2 - 8;
  display.setCursor(wattX, 16);
  display.print(wattStr);

  display.setTextSize(1);
  display.setCursor(wattX + wTxt + 6, 16);
  display.print("W");

  // В режиме настройки мощности — рамка вокруг числа
  if (deviceMode == MODE_WATT_SET) {
    display.drawRoundRect(wattX - 6, 12, wTxt + 24, 34, 3, SSD1306_WHITE);
  }

  // Сопротивление спирали мелким шрифтом под мощностью
  char coilStr[12];
  snprintf(coilStr, sizeof(coilStr), "%.2f Ohm", COIL_RESISTANCE);
  display.getTextBounds(coilStr, 0, 0, &x1, &y1, &wTxt, &hTxt);
  display.setCursor((OLED_WIDTH - wTxt) / 2, 50);
  display.print(coilStr);

  display.drawLine(0, 53, OLED_WIDTH - 1, 53, SSD1306_WHITE);

  // ---------- Нижняя часть: таймер затяжки / счётчик ----------
  float puffSeconds = isFiring
                         ? (millis() - fireStartTime) / 1000.0f
                         : lastPuffDurationMs / 1000.0f;

  char timerStr[8];
  snprintf(timerStr, sizeof(timerStr), "%.1fs", puffSeconds);
  display.setCursor(2, 56);
  display.print(timerStr);

  if (!isFiring && lastPuffDurationMs >= (unsigned long)MAX_PUFF_TIME_SEC * 1000UL) {
    display.setCursor(30, 56);
    display.print("TIME OVER");
  }

  char puffStr[14];
  snprintf(puffStr, sizeof(puffStr), "P:%04lu", (unsigned long)puffCount);
  display.getTextBounds(puffStr, 0, 0, &x1, &y1, &wTxt, &hTxt);
  display.setCursor(OLED_WIDTH - wTxt - 2, 56);
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
