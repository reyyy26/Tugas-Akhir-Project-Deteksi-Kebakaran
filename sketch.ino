#include <Wire.h>
#include <DHTesp.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// ===================== PIN CONFIGURATION =====================
#define DHT_PIN         4
#define MQ2_PIN         34
#define FLAME_PIN       35
#define TEST_FLAME_PIN  32
#define BUTTON_PIN      13

#define LED_GREEN       26
#define LED_YELLOW      27
#define LED_RED         14
#define BUZZER_PIN      25

#define LCD_SDA         21
#define LCD_SCL         22

// ===================== MQ-2 CONFIG =====================
#define MQ2_MIN_PPM     0.1f
#define MQ2_MAX_PPM     100000.0f

// Kalibrasi:
#define MQ2_CAL_FACTOR  136.0f

// ===================== FLAME LOGIC =====================
#define FLAME_ACTIVE_LOW 1

// ===================== THRESHOLDS =====================
#define MQ2_WASPADA     350.0f
#define MQ2_BAHAYA      550.0f
#define MQ2_HYST         50.0f

#define TEMP_WASPADA     38.0f
#define TEMP_BAHAYA      48.0f
#define HUM_WASPADA      30.0f
#define HUM_BAHAYA       20.0f
#define TEMP_HYST         2.0f
#define HUM_HYST          3.0f

// ===================== TIMING =====================
#define DHT_INTERVAL             2500UL
#define FAST_SENSOR_INTERVAL      250UL
#define DEBOUNCE_DELAY            200UL
#define BLINK_SLOW                500UL
#define BLINK_FAST                250UL
#define BUZZER_ON_TIME            300UL
#define BUZZER_OFF_TIME           200UL
#define STARTUP_DURATION         1500UL

#define FLAME_TRIGGER_COUNT         3
#define FLAME_CLEAR_COUNT           3

// ===================== STATE MACHINE =====================
enum SystemState { NORMAL, WASPADA, BAHAYA };
SystemState currentState  = NORMAL;
SystemState previousState = NORMAL;

// ===================== OBJECTS =====================
DHTesp dht;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ===================== TIMING VARIABLES =====================
unsigned long lastDhtRead      = 0;
unsigned long lastFastRead     = 0;
unsigned long lastBlink        = 0;
unsigned long lastDebounce     = 0;
unsigned long lastBuzzerToggle = 0;
unsigned long startupStartMs   = 0;

// ===================== SENSOR DATA =====================
float temperature = 25.0f;
float humidity    = 50.0f;

float mq2Ppm      = 0.0f;
int mq2RawLast    = 0;

bool flameDetected   = false;
bool testFlameActive = false;

// Flame filter counters
int flameLowCount  = 0;
int flameHighCount = 0;

// ===================== OUTPUT STATE =====================
bool yellowLedState = false;
bool redLedState    = false;
bool buzzerOn       = false;

// ===================== LCD CONTROL =====================
int currentPage = 0;
const int TOTAL_PAGES = 5;
bool lcdDirty = true;

// ===================== BUTTON =====================
bool lastButtonState = HIGH;

// ===================== STARTUP CONTROL =====================
bool startupDone = false;

// ===================== CUSTOM CHARACTERS =====================
byte charDegree[8] = {
  0b00110, 0b01001, 0b01001, 0b00110, 0b00000, 0b00000, 0b00000, 0b00000
};

byte charBell[8] = {
  0b00100, 0b01110, 0b01110, 0b01110, 0b11111, 0b00000, 0b00100, 0b00000
};

byte charFire[8] = {
  0b00100, 0b01110, 0b11111, 0b11111, 0b01110, 0b00110, 0b00010, 0b00000
};

// ===================== HELPER FUNCTIONS =====================
bool floatChanged(float a, float b, float eps = 0.1f) {
  return fabsf(a - b) >= eps;
}

void printLine(uint8_t row, const char *text) {
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16.16s", text);
  lcd.setCursor(0, row);
  lcd.print(buf);
}

void printPageIndicator(uint8_t page) {
  lcd.setCursor(13, 0);
  lcd.print(page);
  lcd.print("/");
  lcd.print(TOTAL_PAGES);
}

float rawToPpm(int raw) {
  raw = constrain(raw, 0, 4095);

  float norm = (float)raw / 4095.0f;

  // Skala linear ke 0.1..100000
  float ppm = MQ2_MIN_PPM + norm * (MQ2_MAX_PPM - MQ2_MIN_PPM);

  // Kalibrasi agar sesuai dengan slider Wokwi
  ppm = ppm / MQ2_CAL_FACTOR;

  return constrain(ppm, MQ2_MIN_PPM, MQ2_MAX_PPM);
}

bool flameRawActive() {
  int v = digitalRead(FLAME_PIN);
#if FLAME_ACTIVE_LOW
  return (v == LOW);
#else
  return (v == HIGH);
#endif
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(LED_GREEN,  OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED,    OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(FLAME_PIN, INPUT);
  pinMode(TEST_FLAME_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(LED_GREEN,  LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED,    LOW);
  noTone(BUZZER_PIN);

  analogReadResolution(12);
  analogSetPinAttenuation(MQ2_PIN, ADC_11db);

  dht.setup(DHT_PIN, DHTesp::DHT22);

  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init();
  lcd.backlight();

  lcd.createChar(0, charDegree);
  lcd.createChar(1, charBell);
  lcd.createChar(2, charFire);

  startupStartMs = millis();

  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("FIRE GUARD");
  lcd.setCursor(2, 1);
  lcd.print("Initializing...");

  Serial.println("========================================");
  Serial.println("  Sistem Deteksi Kebakaran ESP32 v2     ");
  Serial.println("========================================");
  Serial.println("[OK] DHT22, MQ-2, Flame Sensor aktif");
  Serial.println("[OK] TEST_FLAME tersedia di GPIO32");
}

// ============================================================
// STARTUP SCREEN
// ============================================================
void handleStartupScreen(unsigned long now) {
  if (startupDone) return;

  if (now - startupStartMs < STARTUP_DURATION) {
    return;
  }

  startupDone = true;
  lcd.clear();
  lcdDirty = true;
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  handleStartupScreen(now);
  if (!startupDone) {
    return;
  }

  handleButton(now);

  bool sensorUpdated = readSensors(now);
  bool stateChanged  = updateState();

  updateOutputs(now);

  if (sensorUpdated || stateChanged) {
    printSerialStatus();
  }

  updateLCD();
}

// ============================================================
// BUTTON HANDLER
// ============================================================
void handleButton(unsigned long now) {
  bool buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == LOW && lastButtonState == HIGH) {
    if (now - lastDebounce >= DEBOUNCE_DELAY) {
      currentPage = (currentPage + 1) % TOTAL_PAGES;
      lastDebounce = now;
      lcdDirty = true;
    }
  }

  lastButtonState = buttonState;
}

// ============================================================
// SENSOR READING
// ============================================================
bool readSensors(unsigned long now) {
  bool updated = false;

  if (now - lastFastRead >= FAST_SENSOR_INTERVAL) {
    lastFastRead = now;

    mq2RawLast = analogRead(MQ2_PIN);
    float ppm = rawToPpm(mq2RawLast);

    if (floatChanged(ppm, mq2Ppm, 0.1f)) {
      mq2Ppm = ppm;
      updated = true;
      lcdDirty = true;
    }

    testFlameActive = (digitalRead(TEST_FLAME_PIN) == LOW);
    bool flameRaw = flameRawActive();

    if (testFlameActive) {
      if (!flameDetected) {
        flameDetected = true;
        updated = true;
        lcdDirty = true;
      }
      flameLowCount = 0;
      flameHighCount = 0;
    } else {
      if (flameRaw) {
        flameLowCount++;
        flameHighCount = 0;

        if (!flameDetected && flameLowCount >= FLAME_TRIGGER_COUNT) {
          flameDetected = true;
          updated = true;
          lcdDirty = true;
        }
      } else {
        flameHighCount++;
        flameLowCount = 0;

        if (flameDetected && flameHighCount >= FLAME_CLEAR_COUNT) {
          flameDetected = false;
          updated = true;
          lcdDirty = true;
        }
      }
    }
  }

  if (now - lastDhtRead >= DHT_INTERVAL) {
    lastDhtRead = now;

    TempAndHumidity d = dht.getTempAndHumidity();

    Serial.print("DHT -> T: ");
    if (isnan(d.temperature)) Serial.print("NaN");
    else Serial.print(d.temperature, 1);

    Serial.print(" C | H: ");
    if (isnan(d.humidity)) Serial.print("NaN");
    else Serial.print(d.humidity, 1);

    Serial.println(" %");

    if (!isnan(d.temperature)) {
      temperature = d.temperature;
      updated = true;
      lcdDirty = true;
    }

    if (!isnan(d.humidity)) {
      humidity = d.humidity;
      updated = true;
      lcdDirty = true;
    }
  }

  return updated;
}

// ============================================================
// SERIAL DEBUG
// ============================================================
void printSerialStatus() {
  Serial.println("-----------------------");
  Serial.printf("Temp : %.1f C\n", temperature);
  Serial.printf("Hum  : %.1f %%\n", humidity);
  Serial.printf("MQ2  : %d ppm\n", (int)roundf(mq2Ppm));
  Serial.printf("MQ2 raw: %d\n", mq2RawLast);
  Serial.printf("Flame pin raw: %d\n", digitalRead(FLAME_PIN));
  Serial.printf("Flame detected: %s\n", flameDetected ? "YES" : "NO");
  Serial.printf("TEST_FLAME: %s\n", testFlameActive ? "ON" : "OFF");
  Serial.printf("State: %s\n",
    currentState == NORMAL  ? "NORMAL"  :
    currentState == WASPADA ? "WASPADA" : "BAHAYA");
}

// ============================================================
// STATE MACHINE
// ============================================================
bool updateState() {
  previousState = currentState;

  if (flameDetected ||
      temperature > TEMP_BAHAYA ||
      mq2Ppm > MQ2_BAHAYA ||
      humidity < HUM_BAHAYA) {
    currentState = BAHAYA;
  } else {
    bool hotW, smokeW, dryW;

    if (currentState == WASPADA) {
      hotW   = (temperature >= TEMP_WASPADA - TEMP_HYST);
      smokeW = (mq2Ppm      >= MQ2_WASPADA  - MQ2_HYST);
      dryW   = (humidity    <  HUM_WASPADA  + HUM_HYST);
    } else {
      hotW   = (temperature >= TEMP_WASPADA);
      smokeW = (mq2Ppm      >= MQ2_WASPADA);
      dryW   = (humidity    <  HUM_WASPADA && humidity >= HUM_BAHAYA);
    }

    currentState = (hotW || smokeW || dryW) ? WASPADA : NORMAL;
  }

  if (currentState != previousState) {
    lcdDirty = true;
    return true;
  }

  return false;
}

// ============================================================
// OUTPUT CONTROL
// ============================================================
void updateOutputs(unsigned long now) {
  switch (currentState) {
    case NORMAL:
      digitalWrite(LED_GREEN,  HIGH);
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_RED,    LOW);
      noTone(BUZZER_PIN);
      buzzerOn = false;
      redLedState = false;
      yellowLedState = false;
      break;

    case WASPADA:
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED,   LOW);

      if (now - lastBlink >= BLINK_SLOW) {
        lastBlink = now;
        yellowLedState = !yellowLedState;
        digitalWrite(LED_YELLOW, yellowLedState);
      }

      noTone(BUZZER_PIN);
      buzzerOn = false;
      redLedState = false;
      break;

    case BAHAYA:
      digitalWrite(LED_GREEN,  LOW);
      digitalWrite(LED_YELLOW, LOW);

      if (now - lastBlink >= BLINK_FAST) {
        lastBlink = now;
        redLedState = !redLedState;
        digitalWrite(LED_RED, redLedState);
      }

      if (buzzerOn) {
        if (now - lastBuzzerToggle >= BUZZER_ON_TIME) {
          noTone(BUZZER_PIN);
          buzzerOn = false;
          lastBuzzerToggle = now;
        }
      } else {
        if (now - lastBuzzerToggle >= BUZZER_OFF_TIME) {
          tone(BUZZER_PIN, flameDetected ? 3000 : 2500);
          buzzerOn = true;
          lastBuzzerToggle = now;
        }
      }
      break;
  }
}

// ============================================================
// LCD DISPLAY
// ============================================================
void updateLCD() {
  if (!lcdDirty) return;
  lcdDirty = false;

  char line1[17];
  char line2[17];

  switch (currentPage) {
    case 0:
      if (currentState == NORMAL) {
        snprintf(line1, sizeof(line1), "STATUS:NORMAL");
        snprintf(line2, sizeof(line2), "Semua aman");
      } else if (currentState == WASPADA) {
        snprintf(line1, sizeof(line1), "STS:WASPADA");

        if (temperature >= TEMP_WASPADA) {
          snprintf(line2, sizeof(line2), "Suhu tinggi");
        } else if (mq2Ppm >= MQ2_WASPADA) {
          snprintf(line2, sizeof(line2), "Asap terdetek");
        } else {
          snprintf(line2, sizeof(line2), "RH turun");
        }
      } else {
        snprintf(line1, sizeof(line1), "STATUS:BAHAYA");

        if (testFlameActive) {
          snprintf(line2, sizeof(line2), "TEST FLAME AKTIF");
        } else if (flameDetected) {
          snprintf(line2, sizeof(line2), "API TERDETEKSI");
        } else if (temperature > TEMP_BAHAYA) {
          snprintf(line2, sizeof(line2), "SUHU KRITIS");
        } else if (mq2Ppm > MQ2_BAHAYA) {
          snprintf(line2, sizeof(line2), "ASAP BERBAHAYA");
        } else {
          snprintf(line2, sizeof(line2), "RH KRITIS");
        }
      }

      printLine(0, line1);
      printPageIndicator(1);
      printLine(1, line2);
      break;

    case 1:
      snprintf(line1, sizeof(line1), "Suhu:%.1fC", temperature);
      snprintf(line2, sizeof(line2), "RH:%.1f%%", humidity);

      printLine(0, line1);
      printPageIndicator(2);
      printLine(1, line2);
      break;

    case 2:
      snprintf(line1, sizeof(line1), "MQ2:%d", (int)roundf(mq2Ppm));

      if (mq2Ppm > MQ2_BAHAYA) {
        snprintf(line2, sizeof(line2), "Asap: BAHAYA");
      } else if (mq2Ppm >= MQ2_WASPADA) {
        snprintf(line2, sizeof(line2), "Asap: WASPADA");
      } else {
        snprintf(line2, sizeof(line2), "Asap: NORMAL");
      }

      printLine(0, line1);
      printPageIndicator(3);
      printLine(1, line2);
      break;

    case 3:
      snprintf(line1, sizeof(line1), "Flame Sensor");

      if (testFlameActive) {
        snprintf(line2, sizeof(line2), "TEST FLAME AKTIF");
      } else if (flameDetected) {
        snprintf(line2, sizeof(line2), "API TERDETEKSI");
      } else {
        snprintf(line2, sizeof(line2), "Tidak ada api");
      }

      printLine(0, line1);
      printPageIndicator(4);
      printLine(1, line2);
      break;

    case 4:
      snprintf(line1, sizeof(line1), "Batas Bahaya");
      snprintf(line2, sizeof(line2), "T>48 H<20 M>550");

      printLine(0, line1);
      printPageIndicator(5);
      printLine(1, line2);
      break;
  }
}
