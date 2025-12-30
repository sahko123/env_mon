struct TimeZone;
#include <SPI.h>
#include <Wire.h>
#include "RTClib.h"
#include <SensirionI2CSen5x.h>

// =================== VFD SETTINGS ===================
const uint8_t STB_PIN = 10;           
const uint32_t SPI_HZ = 500000;       

// Brightness via Function Set BR bits:
// 0x38 = 100%, 0x39 = 75%, 0x3A = 50%, 0x3B = 25%
const uint8_t FUNCSET_BRIGHT = 0x38;

// VFD framing bytes (synchronous serial)
const uint8_t START_INSTR = 0xF8; // write instruction (RS=0)
const uint8_t START_DATA  = 0xFA; // write data (RS=1)

const uint8_t PAGE_COUNT = 4;
const uint8_t TZ_PAGE = 3;
const int16_t IST_STD_OFFSET_MIN = 0;
const unsigned long PAGE_INTERVAL_MS = 5000;
const unsigned long CO2_INTERVAL_MS = 1000;
const unsigned long SEN55_INTERVAL_MS = 1000;
const unsigned long KATA_INTERVAL_MS = 120;
const unsigned long EDGE_INTERVAL_MS = 160;
const unsigned long SERIAL_BAUD = 115200;
const unsigned long CAL_HOLD_MS = 3000;
const char CO2_CAL_CMD[] = "G";
const uint8_t POT_PIN = A0;
const unsigned long POT_INTERVAL_MS = 100;
const int16_t POT_DAY_MINUTES = 1440;
const uint8_t POT_DEADZONE = 20;
const int16_t POT_STEP_MINUTES = 10;
const int16_t POT_HYST_MINUTES = 2;

const uint8_t ENC_PIN_A = 4;
const uint8_t ENC_PIN_B = 5;
const uint8_t BUTTON_PIN = 6;
const unsigned long ENC_DEBOUNCE_MS = 1;
const unsigned long BUTTON_DEBOUNCE_MS = 50;
const uint8_t ENC_STEPS_PER_DETENT = 2;

// 20x4 bases in visible order
const uint8_t VFD_COLS = 20;
const uint8_t VFD_ROWS = 4;
const uint8_t rowBase[VFD_ROWS] = {0x00, 0x40, 0x14, 0x54};

// =================== RTC + SEN55 ===================
RTC_DS3231 rtc;
SensirionI2CSen5x sen5x;

// =================== COZIR ===================
float cozirScale = 1.0f;
int   lastCO2ppm = -1;

// ---------- LOW LEVEL VFD ----------
void vfdWrite(uint8_t startByte, uint8_t payload) {
  digitalWrite(STB_PIN, LOW);
  SPI.transfer(startByte);
  delayMicroseconds(1);       
  SPI.transfer(payload);
  digitalWrite(STB_PIN, HIGH);
}
void vfdInstr(uint8_t cmd) { vfdWrite(START_INSTR, cmd); }
void vfdData(uint8_t d)    { vfdWrite(START_DATA,  d);  }

void vfdSetCursor(uint8_t col, uint8_t row) {
  if (row >= VFD_ROWS) row = VFD_ROWS - 1;
  if (col >= VFD_COLS) col = VFD_COLS - 1;
  vfdInstr(0x80 | (rowBase[row] + col));
}
void vfdClear() {
  vfdInstr(0x01);
  delay(2);
}
void vfdPrintLine(uint8_t row, const char* s) {
  vfdSetCursor(0, row);
  uint8_t n = 0;
  while (*s && n < VFD_COLS) {
    vfdData((uint8_t)*s++);
    n++;
  }
  while (n < VFD_COLS) {
    vfdData(' ');
    n++;
  }
}
void vfdInit() {
  delay(120);
  vfdInstr(FUNCSET_BRIGHT);
  vfdInstr(0x0C);           
  vfdClear();
}

// ---------- Float formatting helpers ----------
void fmt1(char* out, float v) { dtostrf(v, 0, 1, out); }
void fmt0(char* out, float v) { dtostrf(v, 0, 0, out); }

void centerText(char* out, size_t outSize, const char* text) {
  if (outSize == 0) return;
  size_t maxLen = VFD_COLS;
  if (maxLen > outSize - 1) maxLen = outSize - 1;
  size_t len = strlen(text);
  if (len > maxLen) len = maxLen;
  size_t pad = (maxLen - len) / 2;
  for (size_t i = 0; i < maxLen; ++i) out[i] = ' ';
  for (size_t i = 0; i < len; ++i) out[pad + i] = text[i];
  out[maxLen] = '\0';
}

void centerTextWithinEdges(char* out, size_t outSize, const char* text) {
  if (outSize == 0) return;
  size_t maxLen = VFD_COLS;
  if (maxLen > outSize - 1) maxLen = outSize - 1;
  memset(out, ' ', maxLen);
  out[maxLen] = '\0';
  if (maxLen < 2) return;
  size_t innerWidth = maxLen - 2;
  size_t len = strlen(text);
  if (len > innerWidth) len = innerWidth;
  size_t pad = (innerWidth - len) / 2;
  size_t start = 1 + pad;
  memcpy(out + start, text, len);
}

void padLine(char* line, size_t lineSize) {
  if (lineSize == 0) return;
  size_t maxLen = VFD_COLS;
  if (maxLen > lineSize - 1) maxLen = lineSize - 1;
  size_t len = strlen(line);
  if (len < maxLen) {
    memset(line + len, ' ', maxLen - len);
  }
  line[maxLen] = '\0';
}

uint8_t randomKatakana();

char randomEdgeChar() {
  return (char)randomKatakana();
}

void initEdgeColumns(char leftCol[VFD_ROWS], char rightCol[VFD_ROWS]) {
  for (uint8_t i = 0; i < VFD_ROWS; ++i) {
    leftCol[i] = randomEdgeChar();
    rightCol[i] = randomEdgeChar();
  }
}

void scrollEdgeColumns(char leftCol[VFD_ROWS], char rightCol[VFD_ROWS]) {
  for (int i = VFD_ROWS - 1; i > 0; --i) {
    leftCol[i] = leftCol[i - 1];
  }
  leftCol[0] = randomEdgeChar();
  for (uint8_t i = 0; i < VFD_ROWS - 1; ++i) {
    rightCol[i] = rightCol[i + 1];
  }
  rightCol[VFD_ROWS - 1] = randomEdgeChar();
}

void applyEdgeToLine(char* line, size_t lineSize, char left, char right) {
  if (lineSize == 0) return;
  size_t maxLen = VFD_COLS;
  if (maxLen > lineSize - 1) maxLen = lineSize - 1;
  padLine(line, lineSize);
  if (maxLen == 0) return;
  line[0] = left;
  if (maxLen > 1) line[maxLen - 1] = right;
}

static const char* const kDayNames[] = {
  "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"
};

enum DstRule {
  DST_NONE,
  DST_EU,
  DST_US
};

struct TimeZone {
  const char* name;
  int16_t offsetMinutes;
  DstRule dstRule;
  const char* dstName;
};

static const TimeZone kTimeZones[] = {
  {"UTC", 0, DST_NONE, nullptr},
  {"GMT", 0, DST_EU, "BST"},
  {"CET", 60, DST_EU, "CEST"},
  {"EET", 120, DST_EU, "EEST"},
  {"MSK", 180, DST_NONE, nullptr},
  {"GST", 240, DST_NONE, nullptr},
  {"ISTI", 330, DST_NONE, nullptr},
  {"HKT", 480, DST_NONE, nullptr},
  {"JST", 540, DST_NONE, nullptr},
  {"AEST", 600, DST_NONE, nullptr},
  {"NZST", 720, DST_NONE, nullptr},
  {"HST", -600, DST_NONE, nullptr},
  {"AKST", -540, DST_US, "AKDT"},
  {"PST", -480, DST_US, "PDT"},
  {"MST", -420, DST_US, "MDT"},
  {"CST", -360, DST_US, "CDT"},
  {"EST", -300, DST_US, "EDT"},
  {"AST", -240, DST_NONE, nullptr}
};
const uint8_t TIMEZONE_COUNT = sizeof(kTimeZones) / sizeof(kTimeZones[0]);

bool isLeapYear(int year) {
  return (year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0));
}

int daysInMonth(int year, int month) {
  static const uint8_t kDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int days = kDays[month - 1];
  if (month == 2 && isLeapYear(year)) days = 29;
  return days;
}

int nthSunday(int year, int month, int nth) {
  DateTime first(year, month, 1, 0, 0, 0);
  int dow = first.dayOfTheWeek();
  int day = 1 + ((7 - dow) % 7) + (nth - 1) * 7;
  return day;
}

int lastSunday(int year, int month) {
  int dim = daysInMonth(year, month);
  DateTime last(year, month, dim, 0, 0, 0);
  int dow = last.dayOfTheWeek();
  return dim - dow;
}

bool isEuDstDate(int year, int month, int day) {
  if (month < 3 || month > 10) return false;
  int startDay = lastSunday(year, 3);
  int endDay = lastSunday(year, 10);
  if (month > 3 && month < 10) return true;
  if (month == 3) return day >= startDay;
  return day <= endDay;
}

uint8_t euDstOffsetMinutes(uint32_t utc) {
  DateTime t(utc);
  int y = t.year();
  int startDay = lastSunday(y, 3);
  int endDay = lastSunday(y, 10);
  uint32_t startUtc = DateTime(y, 3, startDay, 1, 0, 0).unixtime();
  uint32_t endUtc = DateTime(y, 10, endDay, 1, 0, 0).unixtime();
  return (utc >= startUtc && utc < endUtc) ? 60 : 0;
}

uint8_t usDstOffsetMinutes(uint32_t utc, int16_t stdOffsetMin) {
  DateTime t(utc);
  int y = t.year();
  int startDay = nthSunday(y, 3, 2);
  int endDay = nthSunday(y, 11, 1);
  int32_t stdOffsetSec = (int32_t)stdOffsetMin * 60;
  int32_t dstOffsetSec = (int32_t)(stdOffsetMin + 60) * 60;
  uint32_t startUtc = DateTime(y, 3, startDay, 2, 0, 0).unixtime() - stdOffsetSec;
  uint32_t endUtc = DateTime(y, 11, endDay, 2, 0, 0).unixtime() - dstOffsetSec;
  if (startUtc <= endUtc) {
    return (utc >= startUtc && utc < endUtc) ? 60 : 0;
  }
  return (utc >= startUtc || utc < endUtc) ? 60 : 0;
}

uint8_t dstOffsetMinutes(const TimeZone& tz, uint32_t utc) {
  switch (tz.dstRule) {
    case DST_EU:
      return euDstOffsetMinutes(utc);
    case DST_US:
      return usDstOffsetMinutes(utc, tz.offsetMinutes);
    default:
      return 0;
  }
}

static const int8_t kEncTable[16] = {
  0, -1,  1,  0,
  1,  0,  0, -1,
 -1,  0,  0,  1,
  0,  1, -1,  0
};

uint8_t randomKatakana() {
  return (uint8_t)random(0xA1, 0xE0);
}

void initKatakanaLine(char* out, size_t outSize) {
  if (outSize == 0) return;
  size_t maxLen = VFD_COLS;
  if (maxLen > outSize - 1) maxLen = outSize - 1;
  for (size_t i = 0; i < maxLen; ++i) {
    out[i] = (char)randomKatakana();
  }
  out[maxLen] = '\0';
}

void scrollKatakanaLine(char* out, size_t outSize) {
  if (outSize == 0) return;
  size_t maxLen = VFD_COLS;
  if (maxLen > outSize - 1) maxLen = outSize - 1;
  memmove(out, out + 1, maxLen - 1);
  out[maxLen - 1] = (char)randomKatakana();
  out[maxLen] = '\0';
}

void cozirCalibrate400() {
  cozirFlush();
  cozirSend(CO2_CAL_CMD);
}

bool tryReadUnixTime(uint32_t* outSeconds) {
  static char buf[20];
  static uint8_t idx = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[idx] = '\0';
      if (idx == 0) return false;
      uint64_t value = 0;
      uint8_t digits = 0;
      for (uint8_t i = 0; i < idx; ++i) {
        if (buf[i] < '0' || buf[i] > '9') continue;
        value = value * 10 + (uint64_t)(buf[i] - '0');
        digits++;
      }
      idx = 0;
      if (digits == 0) return false;
      if (digits > 10) value /= 1000; // assume milliseconds
      if (value == 0 || value > 0xFFFFFFFFu) return false;
      *outSeconds = (uint32_t)value;
      return true;
    }
    if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }
  return false;
}

// ---------- COZIR UART helpers ----------
void cozirFlush() {
  while (Serial1.available()) Serial1.read();
}

void cozirSend(const char* cmd) {
  Serial1.print(cmd);
  Serial1.print("\r\n");
}

bool cozirReadLine(char* buf, size_t maxLen, uint16_t timeoutMs) {
  size_t idx = 0;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\r') continue;
      if (c == '\n') {
        buf[idx] = '\0';
        return idx > 0;
      }
      if (idx < maxLen - 1) buf[idx++] = c;
    }
  }
  buf[idx] = '\0';
  return idx > 0;
}

float cozirQueryScale() {
  cozirFlush();
  cozirSend(".");
  char buf[24];
  if (!cozirReadLine(buf, sizeof(buf), 250)) return 1.0f;

  char* p = strchr(buf, ' ');
  if (!p) return 1.0f;

  int mult = atoi(p + 1);
  if (mult <= 0) mult = 1;
  return (float)mult;
}

int cozirReadCO2ppm() {
  cozirFlush();
  cozirSend("Z");                 // filtered CO2 request

  char buf[32];
  if (!cozirReadLine(buf, sizeof(buf), 300)) return lastCO2ppm;

  // Trim leading whitespace
  char* p = buf;
  while (*p == ' ' || *p == '\t') p++;

  // Accept either filtered (Z) or raw (z)
  if (*p != 'Z' && *p != 'z') return lastCO2ppm;

  // Find the number after the letter
  char* q = strchr(p, ' ');
  if (!q) return lastCO2ppm;

  int raw = atoi(q + 1);
  if (raw <= 0) return lastCO2ppm;

  int ppm = (int)(raw * cozirScale + 0.5f);
  lastCO2ppm = ppm;
  return ppm;
}

// =================== SETUP ===================
void setup() {
  // VFD SPI
  pinMode(STB_PIN, OUTPUT);
  digitalWrite(STB_PIN, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  vfdInit();
  vfdPrintLine(0, "Booting...");

  // I2C bus
  Wire.begin();

  randomSeed(micros());

  Serial.begin(SERIAL_BAUD);

  pinMode(ENC_PIN_A, INPUT_PULLUP);
  pinMode(ENC_PIN_B, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // DS3231
  if (!rtc.begin()) {
    vfdPrintLine(0, "RTC not found!");
    while (1);
  }
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // SEN55
  sen5x.begin(Wire);
  uint16_t err = sen5x.deviceReset();
  if (err) {
    char line[21];
    snprintf(line, sizeof(line), "SEN55 reset %04X", err);
    vfdPrintLine(0, line);
    while (1);
  }
  err = sen5x.startMeasurement();   // full mode (fan on)
  if (err) {
    char line[21];
    snprintf(line, sizeof(line), "SEN55 start %04X", err);
    vfdPrintLine(0, line);
    while (1);
  }

  // COZIR (UART)
  Serial1.begin(9600);  // fixed baud for CozIR sensors
  delay(500);

  // Switch to polling mode to stop streaming
  cozirFlush();
  cozirSend("K 2");     // Mode 2 = polling
  char dummy[24];
  cozirReadLine(dummy, sizeof(dummy), 300); // eat response if any

  // Read scaling multiplier
  cozirScale = cozirQueryScale();

  vfdClear();
  vfdPrintLine(0, "RTC+SEN55+COZIR");
  delay(100);
}

// =================== LOOP ===================
void loop() {
  static uint8_t page = 0;
  static bool manualMode = false;
  static unsigned long lastPageMs = 0;
  static unsigned long lastCo2Ms = 0;
  static unsigned long lastSenMs = 0;
  static bool timersInit = false;

  static uint8_t lastEncState = 0;
  static int8_t encSteps = 0;
  static unsigned long lastEncMs = 0;
  static uint8_t lastButton = HIGH;
  static unsigned long lastButtonMs = 0;
  static bool buttonPressed = false;
  static bool buttonLongHandled = false;
  static unsigned long buttonPressMs = 0;
  static bool tzEditMode = false;
  static uint8_t tzIndex = 0;
  static uint8_t lastPage = 0;
  static unsigned long lastPotMs = 0;
  static int16_t potIstMinutes = 0;
  static bool potInit = false;

  static int co2ppm = -1;
  static bool senOk = false;
  static uint16_t senErr = 0xFFFF;
  static float pm1 = 0.0f, pm25 = 0.0f, pm4 = 0.0f, pm10 = 0.0f;
  static float humidity = 0.0f, temperature = 0.0f, vocIndex = 0.0f, noxIndex = 0.0f;
  static char kataLine0[21];
  static char kataLine3[21];
  static unsigned long lastKataMs = 0;
  static bool kataInit = false;
  static char edgeLeft[VFD_ROWS];
  static char edgeRight[VFD_ROWS];
  static unsigned long lastEdgeMs = 0;
  static bool edgeInit = false;

  unsigned long nowMs = millis();
  if (!timersInit) {
    lastPageMs = nowMs;
    lastCo2Ms = nowMs - CO2_INTERVAL_MS;
    lastSenMs = nowMs - SEN55_INTERVAL_MS;
    lastEncState = (digitalRead(ENC_PIN_A) << 1) | digitalRead(ENC_PIN_B);
    lastPotMs = nowMs - POT_INTERVAL_MS;
    timersInit = true;
  }
  if (!kataInit) {
    initKatakanaLine(kataLine0, sizeof(kataLine0));
    initKatakanaLine(kataLine3, sizeof(kataLine3));
    lastKataMs = nowMs;
    kataInit = true;
  }
  if (!edgeInit) {
    initEdgeColumns(edgeLeft, edgeRight);
    lastEdgeMs = nowMs;
    edgeInit = true;
  }

  uint8_t encState = (digitalRead(ENC_PIN_A) << 1) | digitalRead(ENC_PIN_B);
  if (encState != lastEncState && nowMs - lastEncMs >= ENC_DEBOUNCE_MS) {
    lastEncMs = nowMs;
    uint8_t idx = (lastEncState << 2) | encState;
    int8_t delta = kEncTable[idx];
    if (delta != 0) {
      encSteps += delta;
      if (encSteps >= (int8_t)ENC_STEPS_PER_DETENT) {
        manualMode = true;
        if (page == TZ_PAGE && tzEditMode) {
          tzIndex = (tzIndex + 1) % TIMEZONE_COUNT;
        } else {
          page = (page + 1) % PAGE_COUNT;
        }
        encSteps = 0;
      } else if (encSteps <= -(int8_t)ENC_STEPS_PER_DETENT) {
        manualMode = true;
        if (page == TZ_PAGE && tzEditMode) {
          tzIndex = (tzIndex + TIMEZONE_COUNT - 1) % TIMEZONE_COUNT;
        } else {
          page = (page + PAGE_COUNT - 1) % PAGE_COUNT;
        }
        encSteps = 0;
      }
    }
    lastEncState = encState;
  }

  uint8_t buttonState = digitalRead(BUTTON_PIN);
  if (buttonState != lastButton && nowMs - lastButtonMs >= BUTTON_DEBOUNCE_MS) {
    lastButtonMs = nowMs;
    lastButton = buttonState;
    if (buttonState == LOW) {
      buttonPressed = true;
      buttonPressMs = nowMs;
      buttonLongHandled = false;
    } else {
      if (buttonPressed && !buttonLongHandled) {
        if (page == TZ_PAGE) {
          tzEditMode = !tzEditMode;
          if (tzEditMode) {
            manualMode = true;
          } else {
            manualMode = false;
            lastPageMs = nowMs;
          }
        } else {
          manualMode = false;
          lastPageMs = nowMs;
        }
      }
      buttonPressed = false;
    }
  }
  if (buttonPressed && !buttonLongHandled && nowMs - buttonPressMs >= CAL_HOLD_MS) {
    buttonLongHandled = true;
    manualMode = false;
    lastPageMs = nowMs;
    cozirCalibrate400();
  }

  if (!manualMode && PAGE_INTERVAL_MS > 0 && nowMs - lastPageMs >= PAGE_INTERVAL_MS) {
    lastPageMs = nowMs;
    page = (page + 1) % PAGE_COUNT;
  }
  if (page != lastPage) {
    tzEditMode = false;
    lastPage = page;
  }

  if (CO2_INTERVAL_MS > 0 && nowMs - lastCo2Ms >= CO2_INTERVAL_MS) {
    lastCo2Ms = nowMs;
    co2ppm = cozirReadCO2ppm();
  }

  if (SEN55_INTERVAL_MS > 0 && nowMs - lastSenMs >= SEN55_INTERVAL_MS) {
    lastSenMs = nowMs;
    senErr = sen5x.readMeasuredValues(pm1, pm25, pm4, pm10,
                                      humidity, temperature,
                                      vocIndex, noxIndex);
    senOk = (senErr == 0);
  }

  if (KATA_INTERVAL_MS > 0 && nowMs - lastKataMs >= KATA_INTERVAL_MS) {
    lastKataMs = nowMs;
    scrollKatakanaLine(kataLine0, sizeof(kataLine0));
    scrollKatakanaLine(kataLine3, sizeof(kataLine3));
  }
  if (EDGE_INTERVAL_MS > 0 && nowMs - lastEdgeMs >= EDGE_INTERVAL_MS) {
    lastEdgeMs = nowMs;
    scrollEdgeColumns(edgeLeft, edgeRight);
  }
  if (POT_INTERVAL_MS > 0 && nowMs - lastPotMs >= POT_INTERVAL_MS) {
    lastPotMs = nowMs;
    int raw = analogRead(POT_PIN);
    int lower = POT_DEADZONE;
    int upper = 1023 - POT_DEADZONE;
    if (raw < lower) raw = lower;
    if (raw > upper) raw = upper;
    long maxMinutes = POT_DAY_MINUTES - (POT_STEP_MINUTES > 0 ? POT_STEP_MINUTES : 1);
    if (maxMinutes < 0) maxMinutes = 0;
    long targetMinutes = map(raw, lower, upper, 0, maxMinutes);
    long newRounded = targetMinutes;
    if (POT_STEP_MINUTES > 0) {
      long step = POT_STEP_MINUTES;
      newRounded = ((newRounded + step / 2) / step) * step;
      if (newRounded > maxMinutes) newRounded = maxMinutes;
      long threshold = (step / 2) + POT_HYST_MINUTES;
      if (!potInit ||
          targetMinutes >= (long)potIstMinutes + threshold ||
          targetMinutes <= (long)potIstMinutes - threshold) {
        potIstMinutes = (int16_t)newRounded;
        potInit = true;
      }
    } else {
      potIstMinutes = (int16_t)targetMinutes;
      potInit = true;
    }
  }

  uint32_t unixSeconds = 0;
  if (tryReadUnixTime(&unixSeconds)) {
    rtc.adjust(DateTime(unixSeconds));
  }

  // ---- Time ----
  DateTime now = rtc.now();
  char line0[21] = "";
  char line1[21] = "";
  char line2[21] = "";
  char line3[21] = "";

  switch (page) {
    case 0: {
      char content[21];
      snprintf(content, sizeof(content), "%02d:%02d:%02d %02d-%02d-%02d",
               now.hour(), now.minute(), now.second(),
               now.year() % 100, now.month(), now.day());
      centerTextWithinEdges(line0, sizeof(line0), content);

      if (co2ppm > 0) {
        const char* co2Status = "OK";
        if (co2ppm < 500) {
          co2Status = "GOOD";
        } else if (co2ppm > 900) {
          co2Status = "POOR";
        }
        snprintf(content, sizeof(content), "CO2:%4d ppm %s", co2ppm, co2Status);
      } else {
        snprintf(content, sizeof(content), "CO2: ---- ppm ----");
      }
      centerTextWithinEdges(line1, sizeof(line1), content);

      if (!senOk) {
        snprintf(content, sizeof(content), "SEN55 read %04X", senErr);
        centerTextWithinEdges(line2, sizeof(line2), content);
        centerTextWithinEdges(line3, sizeof(line3), "");
      } else {
        char ts[8], rhs[8], vocs[8], noxs[8];
        fmt1(ts, temperature);
        fmt1(rhs, humidity);
        fmt0(vocs, vocIndex);
        fmt0(noxs, noxIndex);

        snprintf(content, sizeof(content), "T:%sC H:%s%%", ts, rhs);
        centerTextWithinEdges(line2, sizeof(line2), content);
        snprintf(content, sizeof(content), "VOC:%s NOx:%s", vocs, noxs);
        centerTextWithinEdges(line3, sizeof(line3), content);
      }
      applyEdgeToLine(line0, sizeof(line0), edgeLeft[0], edgeRight[0]);
      applyEdgeToLine(line1, sizeof(line1), edgeLeft[1], edgeRight[1]);
      applyEdgeToLine(line2, sizeof(line2), edgeLeft[2], edgeRight[2]);
      applyEdgeToLine(line3, sizeof(line3), edgeLeft[3], edgeRight[3]);
      break;
    }
    case 1: {
      const char* dayName = kDayNames[now.dayOfTheWeek()];
      char timeLine[21];
      char dateLine[21];
      snprintf(timeLine, sizeof(timeLine), "%02d:%02d:%02d",
               now.hour(), now.minute(), now.second());
      snprintf(dateLine, sizeof(dateLine), "%s %02d/%02d",
               dayName, now.day(), now.month());
      memcpy(line0, kataLine0, VFD_COLS);
      line0[VFD_COLS] = '\0';
      centerText(line1, sizeof(line1), timeLine);
      centerText(line2, sizeof(line2), dateLine);
      memcpy(line3, kataLine3, VFD_COLS);
      line3[VFD_COLS] = '\0';
      break;
    }
    case 2: {
      char content[21];
      if (!senOk) {
        snprintf(content, sizeof(content), "SEN55 read %04X", senErr);
        centerTextWithinEdges(line0, sizeof(line0), content);
        centerTextWithinEdges(line1, sizeof(line1), "");
        centerTextWithinEdges(line2, sizeof(line2), "");
        centerTextWithinEdges(line3, sizeof(line3), "");
      } else {
        char pm1s[8], pm25s[8], pm4s[8], pm10s[8];
        char ts[8], rhs[8], vocs[8], noxs[8];
        fmt0(pm1s, pm1);
        fmt0(pm25s, pm25);
        fmt0(pm4s, pm4);
        fmt0(pm10s, pm10);
        fmt1(ts, temperature);
        fmt1(rhs, humidity);
        fmt0(vocs, vocIndex);
        fmt0(noxs, noxIndex);

        snprintf(content, sizeof(content), "PM1:%s PM2.5:%s", pm1s, pm25s);
        centerTextWithinEdges(line0, sizeof(line0), content);
        snprintf(content, sizeof(content), "PM4:%s PM10:%s", pm4s, pm10s);
        centerTextWithinEdges(line1, sizeof(line1), content);
        snprintf(content, sizeof(content), "T:%sC H:%s%%", ts, rhs);
        centerTextWithinEdges(line2, sizeof(line2), content);
        snprintf(content, sizeof(content), "VOC:%s NOx:%s", vocs, noxs);
        centerTextWithinEdges(line3, sizeof(line3), content);
      }
      applyEdgeToLine(line0, sizeof(line0), edgeLeft[0], edgeRight[0]);
      applyEdgeToLine(line1, sizeof(line1), edgeLeft[1], edgeRight[1]);
      applyEdgeToLine(line2, sizeof(line2), edgeLeft[2], edgeRight[2]);
      applyEdgeToLine(line3, sizeof(line3), edgeLeft[3], edgeRight[3]);
      break;
    }
    case 3: {
      char istLine[21];
      char tzLine[21];
      int32_t istMinutes = potIstMinutes;
      int istHour = istMinutes / 60;
      int istMin = istMinutes % 60;
      bool istDstActive = isEuDstDate(now.year(), now.month(), now.day());
      int16_t istOffsetMin = IST_STD_OFFSET_MIN + (istDstActive ? 60 : 0);
      DateTime istDate(now.year(), now.month(), now.day(), istHour, istMin, 0);
      int64_t baseUtc = (int64_t)istDate.unixtime() - (int64_t)istOffsetMin * 60L;
      if (baseUtc < 0) baseUtc = 0;
      if (baseUtc > 0xFFFFFFFFLL) baseUtc = 0xFFFFFFFFLL;
      uint32_t baseUtc32 = (uint32_t)baseUtc;

      const TimeZone& tz = kTimeZones[tzIndex];
      uint8_t dstMin = dstOffsetMinutes(tz, baseUtc32);
      const char* tzName = tz.name;
      if (dstMin > 0 && tz.dstName != nullptr) {
        tzName = tz.dstName;
      }
      int32_t tzOffsetMin = tz.offsetMinutes + (int32_t)dstMin;
      int64_t tzUnix = (int64_t)baseUtc32 + (int64_t)tzOffsetMin * 60L;
      if (tzUnix < 0) tzUnix = 0;
      if (tzUnix > 0xFFFFFFFFLL) tzUnix = 0xFFFFFFFFLL;
      DateTime tzTime((uint32_t)tzUnix);

      snprintf(istLine, sizeof(istLine), "%s %02d:%02d:00",
               istDstActive ? "IST" : "GMT", istHour, istMin);
      snprintf(tzLine, sizeof(tzLine), "%s %02d:%02d:%02d",
               tzName, tzTime.hour(), tzTime.minute(), tzTime.second());

      memcpy(line0, kataLine0, VFD_COLS);
      line0[VFD_COLS] = '\0';
      centerText(line1, sizeof(line1), istLine);
      centerText(line2, sizeof(line2), tzLine);
      memcpy(line3, kataLine3, VFD_COLS);
      line3[VFD_COLS] = '\0';
      break;
    }
  }

  // ---- Update VFD ----
  vfdPrintLine(0, line0);
  vfdPrintLine(1, line1);
  vfdPrintLine(2, line2);
  vfdPrintLine(3, line3);
}
