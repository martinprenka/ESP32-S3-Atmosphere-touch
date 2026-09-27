/*
  Napomena o razvoju:
  Projekat je osmisljen, sastavljen, testiran i iterativno razvijan kroz praktican rad
  na stvarnom hardveru.

  AI alati su povremeno korisceni kao pomoc pri troubleshootingu, proveri koda,
  optimizaciji pojedinih delova i ubrzavanju razvoja.

  Izbor komponenti, povezivanje hardvera, funkcionalni zahtevi, testiranje,
  kalibracija i finalni projekat radjeni su kroz moj praktican rad.
*/



#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <DHT.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include "secrets.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "sprites.h"

/*
  MARTIN / ATMOSPHERE TOUCH V10.3.5.1 + COMPILE FIX

  ESP32-S3 DevKitC-1 N16R8
  ILI9341 320x240 SPI TFT + XPT2046 resistive touch
  DHT22 3-pin module

  FINAL WIRING USED BY THIS SKETCH
  --------------------------------
  DHT22 DATA   -> GPIO4

  TFT CS       -> GPIO10
  TFT RESET    -> GPIO8
  TFT DC       -> GPIO9
  TFT MOSI     -> GPIO11
  TFT SCK      -> GPIO12
  TFT MISO     -> GPIO13

  TOUCH T_CS   -> GPIO6
  TOUCH T_DIN  -> GPIO11
  TOUCH T_CLK  -> GPIO12
  TOUCH T_DO   -> GPIO13
  TOUCH T_IRQ  -> NOT USED

  TFT VCC       -> 5V IN
  TFT LED       -> 3.3V
  DHT VCC       -> 3.3V
  TFT GND and DHT GND -> GND

 Potrebne biblioteke:
   - Adafruit GFX Library
   - Adafruit ILI9341
   - XPT2046_Touchscreen autora Paul Stoffregen
   - DHT sensor library by Adafruit
   - ArduinoJson autora Benoit Blanchon

      Napomene:

 -- Touch ekran koristi filtrirano afino mapiranje koje je izmereno na ovom konkretnom fizičkom panelu.
 -- Dugmad su namerno velika i postavljena dalje od uglova ekrana.
 -- Detaljni muški i ženski sprite-ovi karaktera ugrađeni su preko fajlova sprites.h i sprites.cpp.
 -- Karakter povremeno prikazuje mali govorni oblačić prilagođen trenutnom kontekstu na Home ekranu.
 -- Wi-Fi i NTP omogućavaju automatsko lokalno vreme za Srbiju.
 -- Open-Meteo obezbeđuje trenutne spoljašnje vremenske podatke i trodnevnu prognozu za Zrenjanin.
 -- DHT22 ostaje senzor za UNUTRAŠNJU temperaturu i vlažnost vazduha.
 -- Karakter i vremenska scena reaguju na SPOLJAŠNJE vremenske uslove kada su internet podaci dostupni.
  */

// =====================================================
// PINOVI
// =====================================================

#define DHT_PIN 4
#define DHT_TYPE DHT22

#define TFT_CS   10
#define TFT_RST   8
#define TFT_DC    9
#define TFT_MOSI 11
#define TFT_SCK  12
#define TFT_MISO 13

#define TOUCH_CS  6

// =====================================================
// FALLBACK SAT - koristi se samo dok NTP nije dostupan
// =====================================================

const int START_YEAR   = 2026;
const int START_MONTH  = 9;
const int START_DAY    = 25;
const int START_HOUR   = 21;
const int START_MINUTE = 15;
const int START_SECOND = 0;

// =====================================================
// KLIMA / RASPORED
// =====================================================

const float COLD_LIMIT = 17.0f;
const float HOT_LIMIT  = 30.0f;

const uint32_t SAMPLE_MS = 5000;
const uint32_t FRAME_MS  = 55;    // partial-refresh pacing; animations keep same motion/speed

const int N = 60;


// =====================================================
// WIFI / NTP / OPEN-METEO - ZRENJANIN
// =====================================================

struct CityPreset {
  const char* name;
  float lat;
  float lon;
};

const CityPreset CITIES[] = {
  {"ZRENJANIN", 45.38361f, 20.38194f},
  {"NOVI SAD", 45.26714f, 19.83355f},
  {"BEOGRAD", 44.78657f, 20.44892f}
};

const uint8_t CITY_COUNT = sizeof(CITIES) / sizeof(CITIES[0]);

const char* TZ_INFO = "CET-1CEST,M3.5.0,M10.5.0/3";

const uint32_t WIFI_RETRY_MS = 30000UL;
const uint32_t WEATHER_RETRY_MS = 60UL * 1000UL;

uint32_t weatherUpdateMs();


// =====================================================
// OBJEKTI
// =====================================================

Adafruit_ILI9341 tft(&SPI, TFT_DC, TFT_CS, TFT_RST);
XPT2046_Touchscreen touch(TOUCH_CS);
DHT dht(DHT_PIN, DHT_TYPE);
Preferences prefs;

GFXcanvas16 frame(320, 240);
GFXcanvas16 portraitScene(240, 150);  // only animated portrait scene; keeps RAM use modest

// =====================================================
// BOJE
// =====================================================

const uint16_t WHITE  = 0xFFFF;
const uint16_t BLACK  = 0x0000;
const uint16_t MUTED  = 0x9D56;
const uint16_t CYAN   = 0x3E9F;
const uint16_t ORANGE = 0xFD68;
const uint16_t GREEN  = 0x5F31;
const uint16_t RED    = 0xFA6A;
const uint16_t BLUE   = 0x3B5F;
const uint16_t PURPLE = 0xB2BA;

const uint16_t BG     = 0x0863;
const uint16_t PANEL  = 0x10E5;
const uint16_t EDGE   = 0x2989;

const float TAU = 6.28318530718f;

// =====================================================
// STANJE
// =====================================================

float temp = NAN;
float rh = NAN;
float lowTemp = NAN;
float highTemp = NAN;
float lowHum = NAN;
float highHum = NAN;

float history[N];
float historyHum[N];

bool sensorOK = false;
bool haveReading = false;
bool memoryOK = false;
bool attemptedReading = false;
bool dataDirty = true;
bool pageDirty = true;

int historyHead = 0;
int historySize = 0;

uint32_t prevTick = 0;
uint32_t sensorTick = 0;
uint32_t frameTick = 0;

uint64_t elapsedMs = 0;

int yearNow = START_YEAR;
int monthNow = START_MONTH;
int dayNow = START_DAY;

uint32_t secondsToday = START_HOUR * 3600UL + START_MINUTE * 60UL + START_SECOND;
uint32_t clockRemainder = 0;


float clouds[4] = {5, 125, 245, 340};

// =====================================================
// PAGES
// =====================================================

enum Page {
  PAGE_HOME,
  PAGE_MENU,
  PAGE_CLIMATE,
  PAGE_WEATHER,
  PAGE_FORECAST,
  PAGE_CLOCK,
  PAGE_WIFI,
  PAGE_HISTORY,
  PAGE_SETTINGS,
  PAGE_SETTINGS_ADVANCED,
  PAGE_AMBIENT
};

enum UiTheme {
  THEME_NATURE = 0,
  THEME_OBSIDIAN = 1,
  THEME_RETRO = 2,
  THEME_INDIE = 3,
  THEME_JAPAN = 4
};

enum CharacterEvent {
  CHAR_IDLE,
  CHAR_DANCE,
  CHAR_BOUNCE,
  CHAR_RUN_AWAY,
  CHAR_BEES,
  CHAR_FLOWER_PICK
};


Page currentPage = PAGE_HOME;

// Settings remembered in NVS
// homeStyle: 0 = animated scene, 1 = dashboard
uint8_t homeStyle = 0;
bool femaleCharacter = false;
bool animationsEnabled = true;

// UI / UX preferences
bool useFahrenheit = false;
uint8_t refreshMinutes = 15;   // 5 / 15 / 30
uint8_t cityIndex = 0;         // Zrenjanin by default
bool quietMode = false;
bool alertsEnabled = true;
bool ambientEnabled = true;
uint8_t uiTheme = THEME_NATURE;
bool portraitMode = false;   // false=320x240 landscape, true=240x320 portrait
bool portraitCalV2 = false;

// Full saved touch calibration.
// screenX = touchA*rawX + touchB*rawY + touchC
// screenY = touchD*rawX + touchE*rawY + touchF
// Defaults are the original measured mapping for this panel.
float touchA = -0.140048f;
float touchB = -0.042030f;
float touchC = 557.79f;
float touchD = -0.036693f;
float touchE = -0.108965f;
float touchF = 431.93f;
bool touchCalibrated = false;

// Portrait gets its own raw->240x320 affine calibration.
// Defaults are mathematically derived from the landscape mapping,
// but the 5-point portrait wizard replaces these with real measurements.
float pTouchA = -0.036693f;
float pTouchB = -0.108965f;
float pTouchC = 431.93f;
float pTouchD =  0.140048f;
float pTouchE =  0.042030f;
float pTouchF = -238.79f; // 319 - 557.79
bool portraitTouchCalibrated = false;

// 3x3 local residual correction grid for portrait touch.
// This corrects non-linearity that a single affine transform cannot remove.
int16_t pCorrX[9] = {0,0,0,0,0,0,0,0,0};
int16_t pCorrY[9] = {0,0,0,0,0,0,0,0,0};
bool portraitLocalCalibrated = false;

uint32_t lastInteractionAt = 0;



float showTemp(float c) {
  return useFahrenheit ? (c * 9.0f / 5.0f + 32.0f) : c;
}

const char* tempUnit() {
  return useFahrenheit ? "F" : "C";
}

const CityPreset& activeCity() {
  if (cityIndex >= CITY_COUNT) cityIndex = 0;
  return CITIES[cityIndex];
}

uint32_t weatherUpdateMs() {
  uint8_t m = refreshMinutes;
  if (m != 5 && m != 15 && m != 30) m = 15;
  return (uint32_t)m * 60UL * 1000UL;
}

const char* comfortLabel() {
  if (!sensorOK) return "NEMA PODATAKA";
  if (rh >= 70.0f) return "VLAGNO";
  if (rh <= 30.0f) return "SUVO";
  if (temp >= 28.0f) return "TOPLO";
  if (temp <= 18.0f) return "HLADNO";
  if (temp >= 20.0f && temp <= 26.0f && rh >= 40.0f && rh <= 60.0f) return "PRIJATNO";
  return "NORMALNO";
}


const char* themeName() {
  if (uiTheme == THEME_OBSIDIAN) return "OBSIDIAN";
  if (uiTheme == THEME_RETRO) return "RETRO";
  if (uiTheme == THEME_INDIE) return "INDIE";
  if (uiTheme == THEME_JAPAN) return "JAPAN";
  return "NATURE";
}

uint16_t themeBg() {
  if (uiTheme == THEME_OBSIDIAN) return rgb(4, 8, 18);
  if (uiTheme == THEME_RETRO) return rgb(4, 22, 22);
  if (uiTheme == THEME_INDIE) return rgb(30, 24, 45);
  if (uiTheme == THEME_JAPAN) return rgb(31, 25, 40);
  return BG;
}

uint16_t themePanel() {
  if (uiTheme == THEME_OBSIDIAN) return rgb(13, 20, 38);
  if (uiTheme == THEME_RETRO) return rgb(7, 39, 38);
  if (uiTheme == THEME_INDIE) return rgb(54, 43, 69);
  if (uiTheme == THEME_JAPAN) return rgb(57, 42, 57);
  return PANEL;
}

uint16_t themeEdge() {
  if (uiTheme == THEME_OBSIDIAN) return rgb(54, 77, 118);
  if (uiTheme == THEME_RETRO) return rgb(43, 154, 126);
  if (uiTheme == THEME_INDIE) return rgb(138, 108, 153);
  if (uiTheme == THEME_JAPAN) return rgb(179, 104, 131);
  return EDGE;
}

uint16_t themeAccent() {
  if (uiTheme == THEME_OBSIDIAN) return rgb(92, 126, 255);
  if (uiTheme == THEME_RETRO) return rgb(77, 255, 178);
  if (uiTheme == THEME_INDIE) return rgb(239, 151, 177);
  if (uiTheme == THEME_JAPAN) return rgb(248, 170, 193);
  return CYAN;
}

uint16_t themeAccent2() {
  if (uiTheme == THEME_OBSIDIAN) return rgb(180, 98, 255);
  if (uiTheme == THEME_RETRO) return rgb(255, 201, 72);
  if (uiTheme == THEME_INDIE) return rgb(245, 195, 118);
  if (uiTheme == THEME_JAPAN) return rgb(225, 70, 82);
  return ORANGE;
}

uint16_t themeMuted() {
  if (uiTheme == THEME_OBSIDIAN) return rgb(125, 141, 170);
  if (uiTheme == THEME_RETRO) return rgb(83, 175, 148);
  if (uiTheme == THEME_INDIE) return rgb(182, 160, 191);
  if (uiTheme == THEME_JAPAN) return rgb(192, 160, 175);
  return MUTED;
}

void drawLofiWindowScene(bool full) {
  // Warm dusk / lo-fi apartment-window mood.
  int topY = full ? 0 : 29;
  int bottomY = full ? 239 : 180;

  for (int y = topY; y <= bottomY; ++y) {
    float f = (y - topY) / (float)max(1, bottomY - topY);
    uint16_t c = blend(rgb(42, 31, 67), rgb(111, 67, 83), f);
    frame.drawFastHLine(0, y, 320, c);
  }

  // Window.
  frame.fillRoundRect(183, 33, 126, 119, 10, rgb(20, 24, 49));
  frame.drawRoundRect(183, 33, 126, 119, 10, rgb(145, 104, 151));
  frame.drawFastVLine(246, 38, 108, rgb(80, 69, 112));
  frame.drawFastHLine(188, 92, 116, rgb(80, 69, 112));

  // City silhouettes.
  for (int i = 0; i < 8; ++i) {
    int x = 190 + i * 14;
    int h = 18 + (i * 9) % 37;
    frame.fillRect(x, 145 - h, 11, h, rgb(25, 31, 53));
    if (i % 2 == 0) {
      frame.fillRect(x + 3, 135 - h / 2, 2, 2, rgb(249, 199, 112));
      frame.fillRect(x + 7, 125 - h / 3, 2, 2, rgb(236, 158, 116));
    }
  }

  // Moon + warm desk lamp.
  frame.fillCircle(282, 55, 12, rgb(246, 222, 180));
  frame.fillCircle(287, 51, 11, rgb(20, 24, 49));

  frame.fillRect(17, 150, 154, 7, rgb(79, 52, 55));
  frame.fillRect(26, 157, 6, 22, rgb(57, 42, 47));
  frame.fillRect(154, 157, 6, 22, rgb(57, 42, 47));
  frame.fillTriangle(45, 119, 30, 145, 60, 145, rgb(229, 159, 91));
  frame.fillRect(43, 145, 4, 8, rgb(83, 57, 52));

  // Tiny plant.
  frame.fillRect(132, 135, 16, 16, rgb(105, 65, 54));
  frame.drawLine(140, 135, 134, 122, rgb(61, 125, 79));
  frame.drawLine(140, 135, 148, 121, rgb(61, 125, 79));
  frame.fillCircle(133, 122, 4, rgb(72, 145, 90));
  frame.fillCircle(149, 121, 4, rgb(72, 145, 90));
}

void drawJapanScenery(bool full) {
  int topY = full ? 0 : 29;
  int bottomY = full ? 239 : 180;

  // Dawn gradient.
  for (int y = topY; y <= bottomY; ++y) {
    float f = (y - topY) / (float)max(1, bottomY - topY);
    uint16_t c = blend(rgb(56, 45, 78), rgb(238, 141, 145), f);
    frame.drawFastHLine(0, y, 320, c);
  }

  // Distant sun + mountains.
  frame.fillCircle(248, 64, 22, rgb(251, 219, 191));
  for (int x = 0; x < 320; ++x) {
    int y1 = 119 + (int)(18.0f * sinf(x * 0.017f));
    int y2 = 139 + (int)(12.0f * sinf(x * 0.028f + 1.3f));
    frame.drawFastVLine(x, y1, max(0, bottomY - y1), rgb(77, 70, 91));
    frame.drawFastVLine(x, y2, max(0, bottomY - y2), rgb(61, 64, 76));
  }

  // Torii silhouette.
  frame.fillRect(29, 101, 7, 57, rgb(128, 41, 51));
  frame.fillRect(72, 101, 7, 57, rgb(128, 41, 51));
  frame.fillRect(20, 96, 68, 6, rgb(168, 49, 57));
  frame.fillRect(25, 107, 58, 4, rgb(143, 42, 52));

  // Sakura trunk and branches.
  frame.drawLine(302, 49, 267, 110, rgb(80, 50, 56));
  frame.drawLine(289, 67, 247, 47, rgb(80, 50, 56));
  frame.drawLine(280, 80, 243, 88, rgb(80, 50, 56));
  frame.drawLine(269, 102, 238, 121, rgb(80, 50, 56));

  // Blossoms.
  for (int i = 0; i < 25; ++i) {
    int bx = 241 + (i * 19) % 72;
    int by = 38 + (i * 31) % 91;
    uint16_t p = (i % 3 == 0) ? rgb(255, 196, 211) : rgb(242, 153, 181);
    frame.fillCircle(bx, by, (i % 5 == 0) ? 3 : 2, p);
  }
}

void drawThemeBackdrop(bool dense = false) {
  if (uiTheme == THEME_NATURE) {
    for (int y = 0; y < 240; ++y) {
      uint16_t c = blend(themeBg(), rgb(11, 38, 58), y / 239.0f * 0.35f);
      frame.drawFastHLine(0, y, 320, c);
    }

    for (int i = 0; i < (dense ? 18 : 8); ++i) {
      int x = (i * 61 + 23) % 320;
      int y = (i * 43 + 37) % 220;
      frame.fillCircle(x, y, 1, rgb(35, 75, 92));
    }
  }
  else if (uiTheme == THEME_OBSIDIAN) {
    for (int y = 0; y < 240; ++y) {
      float f = y / 239.0f;
      uint16_t c = blend(rgb(3, 7, 17), rgb(17, 11, 38), f);
      frame.drawFastHLine(0, y, 320, c);
    }

    frame.drawCircle(286, 47, 42, rgb(20, 33, 66));
    frame.drawCircle(286, 47, 58, rgb(11, 22, 48));
    frame.drawCircle(36, 210, 54, rgb(20, 13, 45));

    for (int i = 0; i < (dense ? 26 : 12); ++i) {
      int x = (i * 83 + 17) % 320;
      int y = (i * 31 + 19) % 240;
      frame.drawPixel(x, y, (i % 3 == 0) ? rgb(85, 102, 160) : rgb(29, 40, 69));
    }
  }
  else if (uiTheme == THEME_RETRO) {
    frame.fillScreen(themeBg());

    for (int x = 0; x < 320; x += 16) {
      frame.drawFastVLine(x, 0, 240, rgb(6, 48, 43));
    }
    for (int y = 0; y < 240; y += 16) {
      frame.drawFastHLine(0, y, 320, rgb(6, 48, 43));
    }
    for (int y = 2; y < 240; y += 4) {
      frame.drawFastHLine(0, y, 320, rgb(3, 27, 27));
    }

    frame.drawFastHLine(3, 3, 24, themeAccent());
    frame.drawFastVLine(3, 3, 16, themeAccent());
    frame.drawFastHLine(293, 236, 24, themeAccent());
    frame.drawFastVLine(316, 221, 16, themeAccent());
  }
  else if (uiTheme == THEME_INDIE) {
    drawLofiWindowScene(true);
    // Vinyl-like floating rings.
    frame.drawCircle(86, 58, 19, rgb(74, 56, 91));
    frame.drawCircle(86, 58, 12, rgb(99, 71, 105));
    frame.fillCircle(86, 58, 3, rgb(239, 151, 177));
  }
  else {
    drawJapanScenery(true);
  }
}

// =====================================================
// ONLINE STANJE
// =====================================================

bool wifiConnected = false;
bool ntpConfigured = false;
bool timeSynced = false;
bool weatherOK = false;

uint32_t lastWiFiAttempt = 0;
uint32_t lastWeatherFetch = 0;
uint32_t lastWeatherAttempt = 0;
uint32_t lastClockSync = 0;

float outdoorTemp = NAN;
float outdoorFeels = NAN;
float outdoorHumidity = NAN;
float outdoorWind = NAN;
int outdoorWeatherCode = -1;
bool outdoorIsDay = true;

float forecastMax[3] = {NAN, NAN, NAN};
float forecastMin[3] = {NAN, NAN, NAN};
int forecastCode[3] = {-1, -1, -1};
int forecastRainChance[3] = {0, 0, 0};
char forecastDate[3][11] = {{0}};

char weatherLastError[48] = "";

bool isNightUiActive() {
  return false;  // Night UI removed in V9.4: always render full UI colors.
}

// =====================================================
// POVREMENI OBLACIC / PORUKE LIKA
// =====================================================

bool speechVisible = false;
uint32_t speechStartedAt = 0;
uint32_t nextSpeechAt = 0;

const uint32_t SPEECH_DURATION_MS = 4800;

char speechLine1[22] = "";
char speechLine2[22] = "";

void setSpeech(const char* line1, const char* line2) {
  snprintf(speechLine1, sizeof(speechLine1), "%s", line1);
  snprintf(speechLine2, sizeof(speechLine2), "%s", line2);

  speechVisible = true;
  speechStartedAt = millis();
  pageDirty = true;
}

void chooseSpeechMessage() {
  // Prvo koristi stvarno spoljno vreme ako ga imamo.
  if (weatherOK && alertsEnabled) {
    int mode = sceneMode();

    if (mode == 3) {
      switch (random(3)) {
        case 0: setSpeech("Ponesi", "kisobran."); break;
        case 1: setSpeech("Kisa je", "napolju."); break;
        default:setSpeech("Napolju je", "mokro vreme."); break;
      }
      return;
    }

    if (mode == 1) {
      switch (random(3)) {
        case 0: setSpeech("Brrr...", "Hladno napolju."); break;
        case 1: setSpeech("Toplija jakna", "danas."); break;
        default:setSpeech("Sneg mode", "aktivan."); break;
      }
      return;
    }

    if (outdoorTemp >= 30.0f) {
      switch (random(3)) {
        case 0: setSpeech("Bas je vruce", "napolju!"); break;
        case 1: setSpeech("Voda + hlad", "dobra ideja."); break;
        default:setSpeech("Letnji mode", "aktivan!"); break;
      }
      return;
    }

    if (forecastRainChance[0] >= 60 && mode != 3) {
      setSpeech("Moguca kisa", "danas.");
      return;
    }

    if (isfinite(forecastMax[1]) && random(4) == 0) {
      char l1[22];
      char l2[22];
      snprintf(l1, sizeof(l1), "Sutra %.0f/%.0f %s", showTemp(forecastMax[1]), showTemp(forecastMin[1]), tempUnit());
      snprintf(l2, sizeof(l2), "%s", weatherLabel(forecastCode[1]));
      setSpeech(l1, l2);
      return;
    }
  }

  // Zatim komentarisi unutrasnju klimu sa DHT22.
  if (!sensorOK) {
    setSpeech("Cekam DHT22...", "Jos malo.");
    return;
  }

  if (rh >= 70.0f) {
    setSpeech("Vlaga u sobi", "je visoka.");
    return;
  }

  if (rh <= 30.0f) {
    setSpeech("Vazduh u sobi", "je malo suv.");
    return;
  }

  if (temp >= 28.0f) {
    setSpeech("U sobi je", "prilicno toplo.");
    return;
  }

  if (temp <= 18.0f) {
    setSpeech("U sobi je", "malo hladno.");
    return;
  }

  switch (random(4)) {
    case 0: setSpeech("Bas je", "prijatno!"); break;
    case 1: setSpeech("Hej!", "Kako ide?"); break;
    case 2: setSpeech("Atmosfera:", "odlicna."); break;
    default:setSpeech("Sve izgleda", "sasvim okej."); break;
  }
}

CharacterEvent characterEvent = CHAR_IDLE;
uint32_t characterEventStartedAt = 0;
uint32_t characterEventDuration = 0;
uint32_t nextCharacterEventAt = 0;

// Small hidden scene detail.
uint32_t nextSkySparkAt = 0;
uint32_t skySparkStartedAt = 0;
bool skySparkActive = false;

void scheduleNextCharacterEvent(uint32_t now) {
  if (quietMode) {
    nextCharacterEventAt = now + (uint32_t)random(30000, 60001);
  } else {
    nextCharacterEventAt = now + (uint32_t)random(10000, 15001);
  }
}

void startCharacterMotion(CharacterEvent e, uint32_t durationMs) {
  characterEvent = e;
  characterEventStartedAt = millis();
  characterEventDuration = durationMs;
  pageDirty = true;
}

void updateCharacterActivity(uint32_t now) {
  // Personality events only live on the animated Home scene.
  if (currentPage != PAGE_HOME || homeStyle != 0) {
    return;
  }

  // Speech is one event type. Do not overlap it with motion.
  if (speechVisible) {
    if ((uint32_t)(now - speechStartedAt) >= SPEECH_DURATION_MS) {
      speechVisible = false;
      scheduleNextCharacterEvent(now);
      pageDirty = true;
    }
    return;
  }

  // Finish a motion event before scheduling the next one.
  if (characterEvent != CHAR_IDLE) {
    if ((uint32_t)(now - characterEventStartedAt) >= characterEventDuration) {
      characterEvent = CHAR_IDLE;
      scheduleNextCharacterEvent(now);
      pageDirty = true;
    }
    return;
  }

  if (nextCharacterEventAt == 0) {
    // First event comes a little sooner after boot.
    nextCharacterEventAt = now + (uint32_t)(quietMode ? random(20000, 35001) : random(7000, 11001));
  }

  if ((int32_t)(now - nextCharacterEventAt) < 0) {
    return;
  }

  int roll = random(100);

  if (quietMode) {
    if (roll < 70) chooseSpeechMessage();
    else startCharacterMotion(CHAR_BOUNCE, 1800);
    return;
  }

  // Mix speech with short and larger scene interactions.
  if (roll < 32) {
    chooseSpeechMessage();
  }
  else if (roll < 48) {
    startCharacterMotion(CHAR_DANCE, 3200);
  }
  else if (roll < 60) {
    startCharacterMotion(CHAR_BOUNCE, 2200);
  }
  else if (roll < 72) {
    startCharacterMotion(CHAR_RUN_AWAY, 5200);
  }
  else if (roll < 88) {
    startCharacterMotion(CHAR_BEES, 7000);
  }
  else {
    startCharacterMotion(CHAR_FLOWER_PICK, 6500);
  }
}

void getCharacterPose(int &x, int &y, bool &visible) {
  x = 230;
  y = 72;
  visible = true;

  if (!animationsEnabled || characterEvent == CHAR_IDLE) {
    return;
  }

  float t = (millis() - characterEventStartedAt) / 1000.0f;

  if (characterEvent == CHAR_DANCE) {
    x += (int)(sinf(t * 8.0f) * 15.0f);
    y -= (int)(fabsf(sinf(t * 8.0f)) * 5.0f);
  }
  else if (characterEvent == CHAR_BOUNCE) {
    y -= (int)(fabsf(sinf(t * 5.8f)) * 13.0f);
  }
  else if (characterEvent == CHAR_RUN_AWAY) {
    const float runOut = 1.75f;
    const float hiddenUntil = 2.55f;
    const float returnEnd = 5.20f;

    if (t < runOut) {
      float p = t / runOut;
      x = 230 + (int)(p * 125.0f);
      y -= (int)(fabsf(sinf(t * 11.0f)) * 5.0f);
    }
    else if (t < hiddenUntil) {
      visible = false;
    }
    else {
      float p = (t - hiddenUntil) / (returnEnd - hiddenUntil);
      p = constrain(p, 0.0f, 1.0f);
      x = 355 - (int)(p * 125.0f);
      y -= (int)(fabsf(sinf(t * 6.5f)) * 4.0f);
    }
  }
  else if (characterEvent == CHAR_BEES) {
    // Panic -> visible sprint -> short off-screen pause -> cautious return.
    if (t < 1.8f) {
      x += (int)(sinf(t * 15.0f) * 9.0f);
      y -= (int)(fabsf(sinf(t * 10.0f)) * 4.0f);
    }
    else if (t < 3.25f) {
      float p = (t - 1.8f) / 1.45f;
      x = 230 + (int)(p * 128.0f);
      y -= (int)(fabsf(sinf(t * 12.0f)) * 6.0f);
    }
    else if (t < 3.85f) {
      visible = false;
    }
    else {
      float p = (t - 3.85f) / 3.15f;
      p = constrain(p, 0.0f, 1.0f);
      x = 358 - (int)(p * 128.0f);
      y -= (int)(fabsf(sinf(t * 5.2f)) * 3.0f);
    }
  }
  else if (characterEvent == CHAR_FLOWER_PICK) {
    // Walk to the flower, crouch, pick it up, then walk home.
    if (t < 1.5f) {
      float p = t / 1.5f;
      x = 230 - (int)(p * 42.0f);
      y -= (int)(fabsf(sinf(t * 8.0f)) * 3.0f);
    }
    else if (t < 2.8f) {
      x = 188;
      float p = (t - 1.5f) / 1.3f;
      y = 72 + (int)(sinf(p * PI) * 11.0f);
    }
    else if (t < 4.2f) {
      x = 188;
      y = 72 - (int)(fabsf(sinf(t * 3.0f)) * 2.0f);
    }
    else {
      float p = (t - 4.2f) / 2.3f;
      p = constrain(p, 0.0f, 1.0f);
      x = 188 + (int)(p * 42.0f);
      y -= (int)(fabsf(sinf(t * 7.0f)) * 3.0f);
    }
  }
}


void drawTinyBee(int x, int y, bool wingUp) {
  uint16_t yellow = rgb(255, 205, 46);
  uint16_t dark = rgb(38, 33, 27);
  uint16_t wing = rgb(221, 241, 246);

  frame.fillCircle(x, y, 3, yellow);
  frame.drawFastVLine(x - 1, y - 3, 7, dark);
  frame.drawFastVLine(x + 1, y - 3, 7, dark);

  int wingY = wingUp ? y - 5 : y - 3;
  frame.drawCircle(x - 4, wingY, 2, wing);
  frame.drawCircle(x + 4, wingY, 2, wing);
}

void drawLittleFlower(int x, int y, bool picked) {
  if (picked) return;

  uint16_t stem = rgb(44, 121, 55);
  uint16_t petal = rgb(255, 224, 237);
  uint16_t center = rgb(255, 198, 43);

  frame.drawFastVLine(x, y - 10, 11, stem);
  frame.drawLine(x, y - 5, x - 4, y - 8, stem);
  frame.fillCircle(x, y - 12, 2, center);
  frame.fillCircle(x - 3, y - 12, 2, petal);
  frame.fillCircle(x + 3, y - 12, 2, petal);
  frame.fillCircle(x, y - 15, 2, petal);
  frame.fillCircle(x, y - 9, 2, petal);
}

void drawHeldFlower(int characterX, int characterY) {
  uint16_t stem = rgb(44, 121, 55);
  uint16_t petal = rgb(255, 224, 237);
  uint16_t center = rgb(255, 198, 43);

  int x = characterX + 10;
  int y = characterY + 50;

  frame.drawLine(x, y, x + 5, y - 10, stem);
  frame.fillCircle(x + 5, y - 12, 2, center);
  frame.fillCircle(x + 2, y - 12, 2, petal);
  frame.fillCircle(x + 8, y - 12, 2, petal);
  frame.fillCircle(x + 5, y - 15, 2, petal);
}

void drawCharacterEventProps(int characterX, int characterY, bool characterVisible) {
  if (!animationsEnabled) return;

  float t = (millis() - characterEventStartedAt) / 1000.0f;

  if (characterEvent == CHAR_BEES) {
    // Three bees orbit and chase the character before flying away.
    if (t < 3.65f) {
      int baseX = characterVisible ? characterX + 34 : 286;
      int baseY = characterVisible ? characterY + 31 : 96;

      for (int i = 0; i < 3; ++i) {
        float a = t * (4.5f + i * 0.35f) + i * 2.1f;
        int bx = baseX + (int)(cosf(a) * (18 + i * 6));
        int by = baseY + (int)(sinf(a * 1.25f) * (13 + i * 4));
        drawTinyBee(bx, by, ((int)(t * 12.0f + i) & 1) == 0);
      }
    }
  }
  else if (characterEvent == CHAR_FLOWER_PICK) {
    bool picked = t >= 2.55f;
    drawLittleFlower(188, 169, picked);

    if (picked && t < 4.4f && characterVisible) {
      drawHeldFlower(characterX, characterY);
    }
  }
}

void drawSpeechBubble() {
  if (!speechVisible) return;

  // Oblacic stoji iznad i malo levo od lika, bez prekrivanja info kartica.
  const int x = 126;
  const int y = 36;
  const int w = 108;
  const int h = 47;

  uint16_t bubble = rgb(248, 249, 244);
  uint16_t outline = rgb(38, 48, 58);
  uint16_t text = rgb(24, 30, 36);

  frame.fillRoundRect(x, y, w, h, 10, bubble);
  frame.drawRoundRect(x, y, w, h, 10, outline);

  // Rep oblacica prema glavi lika.
  frame.fillTriangle(
    x + w - 15, y + h - 1,
    x + w + 4,  y + h + 13,
    x + w - 31, y + h - 1,
    bubble
  );

  frame.drawLine(x + w - 15, y + h, x + w + 4, y + h + 13, outline);
  frame.drawLine(x + w + 4, y + h + 13, x + w - 31, y + h, outline);

  frame.setTextWrap(false);
  frame.setTextColor(text);
  frame.setTextSize(1);

  // Centriraj obe kratke linije.
  int w1 = (int)strlen(speechLine1) * 6;
  int w2 = (int)strlen(speechLine2) * 6;

  frame.setCursor(x + (w - w1) / 2, y + 12);
  frame.print(speechLine1);

  frame.setCursor(x + (w - w2) / 2, y + 27);
  frame.print(speechLine2);
}

// =====================================================
// HELPERS
// =====================================================

float clampF(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

bool leap(int y) {
  return y % 4 == 0 && (y % 100 != 0 || y % 400 == 0);
}

int monthDays(int y, int m) {
  const int days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  return days[m - 1] + ((m == 2 && leap(y)) ? 1 : 0);
}

void tickClock(uint32_t dt) {
  uint64_t ms = (uint64_t)clockRemainder + dt;
  uint64_t total = (uint64_t)secondsToday + ms / 1000;

  clockRemainder = ms % 1000;
  secondsToday = total % 86400;

  for (uint32_t d = total / 86400; d > 0; --d) {
    if (++dayNow > monthDays(yearNow, monthNow)) {
      dayNow = 1;
      if (++monthNow > 12) {
        monthNow = 1;
        ++yearNow;
      }
    }
  }
}

void label(int x, int y, const char* str, uint16_t col = WHITE, int size = 1) {
  frame.setTextSize(size);
  frame.setTextColor(col);
  frame.setCursor(x, y);
  frame.print(str);
}

void centerText(int y, const char* str, uint16_t col = WHITE, int size = 1) {
  int w = (int)strlen(str) * 6 * size;
  label((320 - w) / 2, y, str, col, size);
}

uint16_t rgb(int r, int g, int b) {
  return ((r & 248) << 8) | ((g & 252) << 3) | (b >> 3);
}

uint16_t blend(uint16_t a, uint16_t b, float f) {
  f = clampF(f, 0, 1);

  int r  = (a >> 11) & 31;
  int g  = (a >> 5) & 63;
  int bl = a & 31;

  return ((int)(r + (((b >> 11) & 31) - r) * f) << 11) |
         ((int)(g + (((b >> 5) & 63) - g) * f) << 5) |
         (int)(bl + ((b & 31) - bl) * f);
}

void panel(int x, int y, int w, int h) {
  if (uiTheme == THEME_RETRO) {
    frame.fillRect(x, y, w, h, themePanel());
    frame.drawRect(x, y, w, h, themeEdge());
    frame.drawFastHLine(x + 3, y + 3, min(w - 6, 22), themeAccent());
  } else {
    int radius = uiTheme == THEME_OBSIDIAN ? 11 : 8;
    frame.fillRoundRect(x, y, w, h, radius, themePanel());
    frame.drawRoundRect(x, y, w, h, radius, themeEdge());

    if (uiTheme == THEME_OBSIDIAN) {
      frame.drawFastHLine(x + 10, y + 5, max(0, w - 20), rgb(35, 48, 82));
    }
  }
}

void buttonCanvas(int x, int y, int w, int h,
                  const char* text, uint16_t fill,
                  uint16_t textCol = WHITE, int size = 2) {

  if (uiTheme == THEME_RETRO) {
    frame.fillRect(x, y, w, h, fill);
    frame.drawRect(x, y, w, h, themeEdge());
    frame.drawFastHLine(x + 3, y + 3, max(0, w - 6), blend(fill, WHITE, 0.18f));
  } else {
    int r = uiTheme == THEME_OBSIDIAN ? 11 : 8;
    frame.fillRoundRect(x, y, w, h, r, fill);
    frame.drawRoundRect(x, y, w, h, r, themeEdge());
    if (uiTheme == THEME_OBSIDIAN) {
      frame.drawFastHLine(x + 10, y + 4, max(0, w - 20), blend(fill, WHITE, 0.18f));
    }
  }

  int tw = (int)strlen(text) * 6 * size;
  int tx = x + (w - tw) / 2;
  int ty = y + (h - 8 * size) / 2;

  label(tx, ty, text, textCol, size);
}

// =====================================================
// SETTINGS STORAGE
// =====================================================

void loadSettings() {
  homeStyle = prefs.getUChar("home", 0);
  if (homeStyle > 1) homeStyle = 0;

  femaleCharacter = prefs.getBool("female", false);
  animationsEnabled = prefs.getBool("anim", true);
  useFahrenheit = prefs.getBool("unitF", false);

  refreshMinutes = prefs.getUChar("refresh", 15);
  if (refreshMinutes != 5 && refreshMinutes != 15 && refreshMinutes != 30) refreshMinutes = 15;

  cityIndex = prefs.getUChar("city", 0);
  if (cityIndex >= CITY_COUNT) cityIndex = 0;

  quietMode = prefs.getBool("quiet", false);
  alertsEnabled = prefs.getBool("alerts", true);
  ambientEnabled = prefs.getBool("ambient", true);

  uiTheme = prefs.getUChar("theme", THEME_NATURE);
  if (uiTheme > THEME_JAPAN) uiTheme = THEME_NATURE;
  portraitMode = prefs.getBool("portrait", false);
  portraitCalV2 = prefs.getBool("pCalV2", false);

  touchCalibrated = prefs.getBool("touchCal", false);
  if (touchCalibrated) {
    touchA = prefs.getFloat("touchA", touchA);
    touchB = prefs.getFloat("touchB", touchB);
    touchC = prefs.getFloat("touchC", touchC);
    touchD = prefs.getFloat("touchD", touchD);
    touchE = prefs.getFloat("touchE", touchE);
    touchF = prefs.getFloat("touchF", touchF);
  }

  portraitTouchCalibrated = prefs.getBool("pTouchCal", false);
  if (portraitTouchCalibrated) {
    pTouchA = prefs.getFloat("pTouchA", pTouchA);
    pTouchB = prefs.getFloat("pTouchB", pTouchB);
    pTouchC = prefs.getFloat("pTouchC", pTouchC);
    pTouchD = prefs.getFloat("pTouchD", pTouchD);
    pTouchE = prefs.getFloat("pTouchE", pTouchE);
    pTouchF = prefs.getFloat("pTouchF", pTouchF);
  }

  portraitLocalCalibrated = prefs.getBool("pLocalCal", false);
  if (portraitLocalCalibrated) {
    if (prefs.getBytesLength("pCorrX") == sizeof(pCorrX)) {
      prefs.getBytes("pCorrX", pCorrX, sizeof(pCorrX));
      prefs.getBytes("pCorrY", pCorrY, sizeof(pCorrY));
    } else {
      portraitLocalCalibrated = false;
    }
  }
}

void saveSettings() {
  prefs.putUChar("home", homeStyle);
  prefs.putBool("female", femaleCharacter);
  prefs.putBool("anim", animationsEnabled);
  prefs.putBool("unitF", useFahrenheit);
  prefs.putUChar("refresh", refreshMinutes);
  prefs.putUChar("city", cityIndex);
  prefs.putBool("quiet", quietMode);
  prefs.putBool("alerts", alertsEnabled);
  prefs.putBool("ambient", ambientEnabled);
  prefs.putUChar("theme", uiTheme);
  prefs.putBool("portrait", portraitMode);
  prefs.putBool("pCalV2", portraitCalV2);

  prefs.putFloat("touchA", touchA);
  prefs.putFloat("touchB", touchB);
  prefs.putFloat("touchC", touchC);
  prefs.putFloat("touchD", touchD);
  prefs.putFloat("touchE", touchE);
  prefs.putFloat("touchF", touchF);
  prefs.putBool("touchCal", touchCalibrated);

  prefs.putFloat("pTouchA", pTouchA);
  prefs.putFloat("pTouchB", pTouchB);
  prefs.putFloat("pTouchC", pTouchC);
  prefs.putFloat("pTouchD", pTouchD);
  prefs.putFloat("pTouchE", pTouchE);
  prefs.putFloat("pTouchF", pTouchF);
  prefs.putBool("pTouchCal", portraitTouchCalibrated);

  prefs.putBytes("pCorrX", pCorrX, sizeof(pCorrX));
  prefs.putBytes("pCorrY", pCorrY, sizeof(pCorrY));
  prefs.putBool("pLocalCal", portraitLocalCalibrated);
}

// =====================================================
// TOUCH FILTER + FULL AFFINE CALIBRATION
// =====================================================

int medianValue(int values[], int count) {
  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (values[j] < values[i]) {
        int t = values[i];
        values[i] = values[j];
        values[j] = t;
      }
    }
  }
  return values[count / 2];
}

bool readStableTouchRaw(int &rawX, int &rawY) {
  TS_Point p = touch.getPoint();
  if (p.z < 250) return false;

  delay(30);

  const int SAMPLES = 9;
  int xs[SAMPLES], ys[SAMPLES];
  int count = 0;

  for (int i = 0; i < SAMPLES; ++i) {
    p = touch.getPoint();
    if (p.z > 200) {
      xs[count] = p.x;
      ys[count] = p.y;
      ++count;
    }
    delay(4);
  }

  if (count < 5) return false;

  rawX = medianValue(xs, count);
  rawY = medianValue(ys, count);
  return true;
}

void mapTouchRaw(int rawX, int rawY, float &screenX, float &screenY) {
  screenX = touchA * rawX + touchB * rawY + touchC;
  screenY = touchD * rawX + touchE * rawY + touchF;
}


float pBilerp(float q00, float q10, float q01, float q11, float tx, float ty) {
  float a = q00 + (q10 - q00) * tx;
  float b = q01 + (q11 - q01) * tx;
  return a + (b - a) * ty;
}

void applyPortraitLocalCorrection(float sx, float sy, float &dx, float &dy) {
  dx = 0.0f;
  dy = 0.0f;
  if (!portraitLocalCalibrated) return;

  const float gx[3] = {25.0f, 120.0f, 215.0f};
  const float gy[3] = {35.0f, 160.0f, 285.0f};

  int c = sx < gx[1] ? 0 : 1;
  int r = sy < gy[1] ? 0 : 1;

  float tx = (sx - gx[c]) / (gx[c+1] - gx[c]);
  float ty = (sy - gy[r]) / (gy[r+1] - gy[r]);
  tx = constrain(tx, 0.0f, 1.0f);
  ty = constrain(ty, 0.0f, 1.0f);

  int i00 = r * 3 + c;
  int i10 = r * 3 + c + 1;
  int i01 = (r + 1) * 3 + c;
  int i11 = (r + 1) * 3 + c + 1;

  dx = pBilerp(pCorrX[i00], pCorrX[i10], pCorrX[i01], pCorrX[i11], tx, ty);
  dy = pBilerp(pCorrY[i00], pCorrY[i10], pCorrY[i01], pCorrY[i11], tx, ty);
}

bool getTouchPixel(int &x, int &y) {
  int rawX, rawY;
  if (!readStableTouchRaw(rawX, rawY)) return false;

  float sx, sy;

  if (portraitMode) {
    sx = pTouchA * rawX + pTouchB * rawY + pTouchC;
    sy = pTouchD * rawX + pTouchE * rawY + pTouchF;

    float dx, dy;
    applyPortraitLocalCorrection(sx, sy, dx, dy);
    sx += dx;
    sy += dy;

    x = constrain((int)roundf(sx), 0, 239);
    y = constrain((int)roundf(sy), 0, 319);
  } else {
    mapTouchRaw(rawX, rawY, sx, sy);
    x = constrain((int)roundf(sx), 0, 319);
    y = constrain((int)roundf(sy), 0, 239);
  }

  Serial.print("TOUCH RAW ");
  Serial.print(rawX);
  Serial.print(",");
  Serial.print(rawY);
  Serial.print(" -> PIXEL ");
  Serial.print(x);
  Serial.print(",");
  Serial.println(y);

  return true;
}

void waitTouchRelease() {
  delay(20);
  while (true) {
    TS_Point p = touch.getPoint();
    if (p.z < 180) break;
    delay(8);
  }
  delay(30);
}

void drawCalibrationTarget(int x, int y, int step) {
  frame.fillScreen(rgb(4, 10, 18));
  centerText(16, "TOUCH KALIBRACIJA", CYAN, 2);

  char b[40];
  snprintf(b, sizeof(b), "TACKA %d / 9", step + 1);
  centerText(43, b, MUTED, 1);
  centerText(59, "DODIRNI CENTAR KRSTICA", WHITE, 1);

  frame.drawCircle(x, y, 13, rgb(79, 166, 222));
  frame.drawCircle(x, y, 6, WHITE);
  frame.drawFastHLine(x - 20, y, 41, WHITE);
  frame.drawFastVLine(x, y - 20, 41, WHITE);
  frame.fillCircle(x, y, 2, ORANGE);

  centerText(220, "DRZI KRATKO I PUSTI", MUTED, 1);
  tft.drawRGBBitmap(0, 0, frame.getBuffer(), 320, 240);
}

bool captureCalibrationRaw(int targetX, int targetY, int &rawX, int &rawY, int step) {
  drawCalibrationTarget(targetX, targetY, step);

  while (touch.getPoint().z >= 180) delay(8);

  uint32_t started = millis();
  while ((uint32_t)(millis() - started) < 12000UL) {
    if (readStableTouchRaw(rawX, rawY)) {
      frame.fillCircle(targetX, targetY, 7, GREEN);
      tft.drawRGBBitmap(0, 0, frame.getBuffer(), 320, 240);
      waitTouchRelease();
      delay(150);
      return true;
    }
    delay(10);
  }
  return false;
}

bool solve3x3(float m[3][4], float &x0, float &x1, float &x2) {
  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    float best = fabsf(m[col][col]);

    for (int r = col + 1; r < 3; ++r) {
      float v = fabsf(m[r][col]);
      if (v > best) {
        best = v;
        pivot = r;
      }
    }

    if (best < 0.000001f) return false;

    if (pivot != col) {
      for (int c = col; c < 4; ++c) {
        float t = m[col][c];
        m[col][c] = m[pivot][c];
        m[pivot][c] = t;
      }
    }

    float div = m[col][col];
    for (int c = col; c < 4; ++c) m[col][c] /= div;

    for (int r = 0; r < 3; ++r) {
      if (r == col) continue;
      float f = m[r][col];
      for (int c = col; c < 4; ++c) m[r][c] -= f * m[col][c];
    }
  }

  x0 = m[0][3];
  x1 = m[1][3];
  x2 = m[2][3];
  return true;
}


bool fitAffineN(const int *rx, const int *ry,
                const int *sx, const int *sy, int n,
                float &a, float &b, float &c,
                float &d, float &e, float &f) {
  float sxx = 0, sxy = 0, sx1 = 0;
  float syy = 0, sy1 = 0, s11 = (float)n;
  float bx0 = 0, bx1 = 0, bx2 = 0;
  float by0 = 0, by1 = 0, by2 = 0;

  for (int i = 0; i < n; ++i) {
    float X = rx[i], Y = ry[i];
    sxx += X * X; sxy += X * Y; sx1 += X;
    syy += Y * Y; sy1 += Y;
    bx0 += X * sx[i]; bx1 += Y * sx[i]; bx2 += sx[i];
    by0 += X * sy[i]; by1 += Y * sy[i]; by2 += sy[i];
  }

  float mx[3][4] = {
    {sxx, sxy, sx1, bx0},
    {sxy, syy, sy1, bx1},
    {sx1, sy1, s11, bx2}
  };
  float my[3][4] = {
    {sxx, sxy, sx1, by0},
    {sxy, syy, sy1, by1},
    {sx1, sy1, s11, by2}
  };

  if (!solve3x3(mx, a, b, c)) return false;
  if (!solve3x3(my, d, e, f)) return false;
  return true;
}

bool fitAffine5(const int rx[5], const int ry[5],
                const int sx[5], const int sy[5],
                float &a, float &b, float &c,
                float &d, float &e, float &f) {
  float sxx = 0, sxy = 0, sx1 = 0;
  float syy = 0, sy1 = 0, s11 = 5.0f;

  float bx0 = 0, bx1 = 0, bx2 = 0;
  float by0 = 0, by1 = 0, by2 = 0;

  for (int i = 0; i < 5; ++i) {
    float X = rx[i];
    float Y = ry[i];

    sxx += X * X;
    sxy += X * Y;
    sx1 += X;
    syy += Y * Y;
    sy1 += Y;

    bx0 += X * sx[i];
    bx1 += Y * sx[i];
    bx2 += sx[i];

    by0 += X * sy[i];
    by1 += Y * sy[i];
    by2 += sy[i];
  }

  float mx[3][4] = {
    {sxx, sxy, sx1, bx0},
    {sxy, syy, sy1, bx1},
    {sx1, sy1, s11, bx2}
  };

  float my[3][4] = {
    {sxx, sxy, sx1, by0},
    {sxy, syy, sy1, by1},
    {sx1, sy1, s11, by2}
  };

  if (!solve3x3(mx, a, b, c)) return false;
  if (!solve3x3(my, d, e, f)) return false;

  return true;
}

void runTouchCalibration() {
  const int sx[5] = {28, 292, 292, 28, 160};
  const int sy[5] = {30, 30, 210, 210, 120};

  int rx[5], ry[5];

  // Keep previous calibration in case calibration is aborted/fails.
  float oldA = touchA, oldB = touchB, oldC = touchC;
  float oldD = touchD, oldE = touchE, oldF = touchF;
  bool oldCal = touchCalibrated;

  for (int i = 0; i < 5; ++i) {
    if (!captureCalibrationRaw(sx[i], sy[i], rx[i], ry[i], i)) {
      touchA = oldA; touchB = oldB; touchC = oldC;
      touchD = oldD; touchE = oldE; touchF = oldF;
      touchCalibrated = oldCal;

      frame.fillScreen(rgb(4, 10, 18));
      centerText(70, "KALIBRACIJA PREKINUTA", RED, 1);
      centerText(104, "STARE VREDNOSTI OSTAJU", WHITE, 1);
      tft.drawRGBBitmap(0, 0, frame.getBuffer(), 320, 240);
      delay(1400);
      pageDirty = true;
      return;
    }
  }

  float a,b,c,d,e,f;
  bool ok = fitAffine5(rx, ry, sx, sy, a,b,c,d,e,f);

  // Validate result against all 5 points.
  float maxErr = 999.0f;
  if (ok) {
    maxErr = 0.0f;
    for (int i = 0; i < 5; ++i) {
      float px = a * rx[i] + b * ry[i] + c;
      float py = d * rx[i] + e * ry[i] + f;
      float dx = px - sx[i];
      float dy = py - sy[i];
      float err = sqrtf(dx*dx + dy*dy);
      if (err > maxErr) maxErr = err;
    }

    // Reject clearly broken solutions.
    if (!isfinite(a) || !isfinite(b) || !isfinite(c) ||
        !isfinite(d) || !isfinite(e) || !isfinite(f) ||
        maxErr > 18.0f) {
      ok = false;
    }
  }

  frame.fillScreen(rgb(4, 10, 18));

  if (ok) {
    touchA = a; touchB = b; touchC = c;
    touchD = d; touchE = e; touchF = f;
    touchCalibrated = true;
    saveSettings();

    centerText(53, "FULL TOUCH CAL SACUVAN", GREEN, 2);

    char b1[48];
    snprintf(b1, sizeof(b1), "MAX GRESKA %.1f px", maxErr);
    centerText(93, b1, WHITE, 1);
    centerText(121, "OFFSET + SCALE + SKEW", CYAN, 1);
    centerText(147, "OSTAJE POSLE GASENJA", CYAN, 1);
  } else {
    touchA = oldA; touchB = oldB; touchC = oldC;
    touchD = oldD; touchE = oldE; touchF = oldF;
    touchCalibrated = oldCal;

    centerText(64, "KALIBRACIJA NIJE USPELA", RED, 1);
    centerText(96, "PONOVI PRECIZNIJE", WHITE, 2);
    centerText(128, "STARE VREDNOSTI OSTAJU", MUTED, 1);
  }

  centerText(188, "DODIRNI ZA POVRATAK", MUTED, 1);
  tft.drawRGBBitmap(0, 0, frame.getBuffer(), 320, 240);

  waitTouchRelease();

  uint32_t waitStart = millis();
  while ((uint32_t)(millis() - waitStart) < 10000UL) {
    TS_Point p = touch.getPoint();
    if (p.z > 250) {
      waitTouchRelease();
      break;
    }
    delay(10);
  }

  pageDirty = true;
  dataDirty = true;
  lastInteractionAt = millis();
}

void goTo(Page p) {
  currentPage = p;
  pageDirty = true;
}


// =====================================================
// WIFI / NTP / WEATHER HELPERS
// =====================================================

bool weatherIsRainCode(int code) {
  return (code >= 51 && code <= 67) ||
         (code >= 80 && code <= 82) ||
         (code >= 95 && code <= 99);
}

bool weatherIsSnowCode(int code) {
  return (code >= 71 && code <= 77) ||
         (code >= 85 && code <= 86);
}

bool weatherIsCloudCode(int code) {
  return code >= 1 && code <= 3;
}

const char* weatherLabel(int code) {
  if (code == 0) return "VEDRO";
  if (code == 1) return "PRETEZNO VEDRO";
  if (code == 2) return "DEL. OBLACNO";
  if (code == 3) return "OBLACNO";
  if (code == 45 || code == 48) return "MAGLA";
  if (code >= 51 && code <= 57) return "ROSULJA";
  if (code >= 61 && code <= 67) return "KISA";
  if (code >= 71 && code <= 77) return "SNEG";
  if (code >= 80 && code <= 82) return "PLJUSAK";
  if (code >= 85 && code <= 86) return "SNEG / PLJUSAK";
  if (code >= 95 && code <= 99) return "OLUJA";
  return "NEPOZNATO";
}

void drawWiFiIcon(int x, int y) {
  bool connected = WiFi.status() == WL_CONNECTED;
  uint16_t c = connected ? WHITE : RED;

  // Background-free Wi-Fi icon.
  // Draw only the visible upper arc segments, so there is no masking rectangle.
  if (connected) {
    // Outer arc.
    frame.drawLine(x - 10, y - 4, x - 7, y - 7, c);
    frame.drawLine(x - 7,  y - 7, x - 3, y - 9, c);
    frame.drawLine(x - 3,  y - 9, x,     y - 10, c);
    frame.drawLine(x,      y - 10, x + 3, y - 9, c);
    frame.drawLine(x + 3,  y - 9, x + 7, y - 7, c);
    frame.drawLine(x + 7,  y - 7, x + 10, y - 4, c);

    // Middle arc.
    frame.drawLine(x - 6, y - 2, x - 3, y - 5, c);
    frame.drawLine(x - 3, y - 5, x,     y - 6, c);
    frame.drawLine(x,     y - 6, x + 3, y - 5, c);
    frame.drawLine(x + 3, y - 5, x + 6, y - 2, c);

    // Inner signal + dot.
    frame.drawLine(x - 3, y, x, y - 2, c);
    frame.drawLine(x, y - 2, x + 3, y, c);
    frame.fillCircle(x, y + 2, 1, c);
  } else {
    frame.drawLine(x - 8, y - 8, x + 8, y + 8, c);
    frame.drawLine(x + 8, y - 8, x - 8, y + 8, c);
  }
}

void drawWeatherIcon(int cx, int cy, int code, int scale = 1) {
  if (scale < 1) scale = 1;

  // Sun / clear.
  if (code == 0 || code == 1) {
    int r = 9 * scale;
    frame.fillCircle(cx, cy, r, rgb(255, 215, 76));
    for (int i = 0; i < 8; ++i) {
      float a = i * TAU / 8.0f;
      frame.drawLine(
        cx + cosf(a) * (r + 4), cy + sinf(a) * (r + 4),
        cx + cosf(a) * (r + 10), cy + sinf(a) * (r + 10),
        rgb(255, 224, 95)
      );
    }
    return;
  }

  uint16_t cc = rgb(200, 215, 225);
  if (code == 3 || weatherIsRainCode(code) || weatherIsSnowCode(code)) {
    cc = rgb(115, 130, 148);
  }

  int s = 5 * scale;
  cloud(cx - 2 * s, cy - s, s, cc);

  if (weatherIsRainCode(code)) {
    for (int i = -2; i <= 2; ++i) {
      int xx = cx + i * 7 * scale;
      frame.drawLine(xx, cy + 10 * scale, xx - 2 * scale, cy + 16 * scale, CYAN);
    }
  }

  if (weatherIsSnowCode(code)) {
    for (int i = -2; i <= 2; ++i) {
      int xx = cx + i * 7 * scale;
      frame.fillCircle(xx, cy + 14 * scale, scale, WHITE);
    }
  }

  if (code == 45 || code == 48) {
    for (int i = 0; i < 3; ++i) {
      frame.drawFastHLine(cx - 20 * scale, cy + (10 + i * 5) * scale, 40 * scale, MUTED);
    }
  }
}

void syncClockFromNTP() {
  if (!ntpConfigured) return;

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 20)) return;

  yearNow = timeinfo.tm_year + 1900;
  monthNow = timeinfo.tm_mon + 1;
  dayNow = timeinfo.tm_mday;
  secondsToday = timeinfo.tm_hour * 3600UL + timeinfo.tm_min * 60UL + timeinfo.tm_sec;
  clockRemainder = 0;
  timeSynced = true;
}

void configureNTP() {
  if (ntpConfigured) return;
  configTzTime(TZ_INFO, "pool.ntp.org", "time.nist.gov");
  ntpConfigured = true;
}

bool connectWiFi(uint32_t timeoutMs) {
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    return true;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  Serial.print("WiFi: connecting");

  while (WiFi.status() != WL_CONNECTED && (uint32_t)(millis() - start) < timeoutMs) {
    delay(250);
    Serial.print(".");
  }

  wifiConnected = WiFi.status() == WL_CONNECTED;
  Serial.println(wifiConnected ? " connected" : " timeout");

  if (wifiConnected) {
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    configureNTP();
    syncClockFromNTP();
  }

  return wifiConnected;
}


void printNetworkDiagnostics() {
  Serial.println("===== NETWORK DIAGNOSTICS =====");
  Serial.print("WiFi connected: ");
  Serial.println(WiFi.status() == WL_CONNECTED ? "YES" : "NO");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("DNS: ");
  Serial.println(WiFi.dnsIP());

  IPAddress resolved;
  bool dnsOK = WiFi.hostByName("api.open-meteo.com", resolved);
  Serial.print("DNS api.open-meteo.com: ");
  if (dnsOK) {
    Serial.print("OK -> ");
    Serial.println(resolved);
  } else {
    Serial.println("FAILED");
  }
  Serial.println("===============================");
}

bool fetchWeather() {
  lastWeatherAttempt = millis();

  if (WiFi.status() != WL_CONNECTED) {
    snprintf(weatherLastError, sizeof(weatherLastError), "WiFi nije povezan");
    weatherOK = false;
    return false;
  }

  const String httpsUrl =
    String("https://api.open-meteo.com/v1/forecast?") +
    "latitude=" + String(activeCity().lat, 5) +
    "&longitude=" + String(activeCity().lon, 5) +
    "&current=temperature_2m,relative_humidity_2m,apparent_temperature,weather_code,wind_speed_10m,is_day" +
    "&daily=weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max" +
    "&timezone=Europe%2FBelgrade&forecast_days=3";

  String payload;
  int status = -1;
  bool gotPayload = false;

  Serial.println("----- WEATHER REQUEST -----");
  Serial.print("WiFi status: ");
  Serial.println((int)WiFi.status());
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("DNS: ");
  Serial.println(WiFi.dnsIP());

  // 1) Primary: HTTPS / TLS.
  {
    WiFiClientSecure secureClient;
    secureClient.setInsecure();

    HTTPClient https;
    https.setTimeout(10000);

    Serial.print("HTTPS URL: ");
    Serial.println(httpsUrl);

    if (https.begin(secureClient, httpsUrl)) {
      status = https.GET();

      Serial.print("Open-Meteo HTTPS code: ");
      Serial.println(status);

      if (status < 0) {
        Serial.print("HTTPS error: ");
        Serial.println(HTTPClient::errorToString(status).c_str());
      }

      if (status == HTTP_CODE_OK) {
        payload = https.getString();
        gotPayload = payload.length() > 0;
        if (gotPayload) Serial.println("WEATHER SOURCE: HTTPS");
      }

      https.end();
    } else {
      Serial.println("HTTPS begin() FAILED");
    }
  }

  // 2) Fallback: plain HTTP.
  // This is intentionally attempted only if HTTPS did not deliver JSON.
  // If Open-Meteo redirects/refuses HTTP, the exact status is printed.
  if (!gotPayload) {
    delay(120);

    String httpUrl = httpsUrl;
    httpUrl.replace("https://", "http://");

    WiFiClient plainClient;
    HTTPClient plain;
    plain.setTimeout(10000);

    Serial.println("HTTPS nije uspeo -> probam HTTP fallback");
    Serial.print("HTTP URL: ");
    Serial.println(httpUrl);

    if (plain.begin(plainClient, httpUrl)) {
      int httpStatus = plain.GET();

      Serial.print("Open-Meteo HTTP fallback code: ");
      Serial.println(httpStatus);

      if (httpStatus < 0) {
        Serial.print("HTTP fallback error: ");
        Serial.println(HTTPClient::errorToString(httpStatus).c_str());
      }

      if (httpStatus == HTTP_CODE_OK) {
        payload = plain.getString();
        gotPayload = payload.length() > 0;
        status = httpStatus;
        if (gotPayload) Serial.println("WEATHER SOURCE: HTTP FALLBACK");
      } else {
        status = httpStatus;
        String location = plain.header("Location");
        if (location.length()) {
          Serial.print("HTTP redirect Location: ");
          Serial.println(location);
        }
      }

      plain.end();
    } else {
      Serial.println("HTTP fallback begin() FAILED");
    }
  }

  if (!gotPayload) {
    snprintf(weatherLastError, sizeof(weatherLastError), "HTTP %d", status);
    weatherOK = false;
    Serial.print("Weather OFFLINE: ");
    Serial.println(weatherLastError);
    return false;
  }

  #if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
  #else
    DynamicJsonDocument doc(8192);
  #endif

  DeserializationError err = deserializeJson(doc, payload);

  if (err) {
    snprintf(weatherLastError, sizeof(weatherLastError), "JSON: %s", err.c_str());
    weatherOK = false;
    Serial.print("Weather JSON error: ");
    Serial.println(err.c_str());
    return false;
  }

  JsonObject current = doc["current"];
  JsonObject daily = doc["daily"];

  outdoorTemp = current["temperature_2m"] | NAN;
  outdoorHumidity = current["relative_humidity_2m"] | NAN;
  outdoorFeels = current["apparent_temperature"] | NAN;
  outdoorWeatherCode = current["weather_code"] | -1;
  outdoorWind = current["wind_speed_10m"] | NAN;
  outdoorIsDay = (current["is_day"] | 1) == 1;

  JsonArray times = daily["time"].as<JsonArray>();
  JsonArray codes = daily["weather_code"].as<JsonArray>();
  JsonArray maxs = daily["temperature_2m_max"].as<JsonArray>();
  JsonArray mins = daily["temperature_2m_min"].as<JsonArray>();
  JsonArray rains = daily["precipitation_probability_max"].as<JsonArray>();

  for (int i = 0; i < 3; ++i) {
    forecastCode[i] = i < (int)codes.size() ? (codes[i] | -1) : -1;
    forecastMax[i] = i < (int)maxs.size() ? (maxs[i] | NAN) : NAN;
    forecastMin[i] = i < (int)mins.size() ? (mins[i] | NAN) : NAN;
    forecastRainChance[i] = i < (int)rains.size() ? (rains[i] | 0) : 0;

    const char* ds = i < (int)times.size() ? (times[i] | "") : "";
    snprintf(forecastDate[i], sizeof(forecastDate[i]), "%s", ds);
  }

  weatherOK = isfinite(outdoorTemp) && outdoorWeatherCode >= 0;
  lastWeatherFetch = millis();

  if (weatherOK) {
    weatherLastError[0] = '\0';
    pageDirty = true;

    Serial.print("Weather: ");
    Serial.print(outdoorTemp);
    Serial.print(" C / code ");
    Serial.print(outdoorWeatherCode);
    Serial.print(" / wind ");
    Serial.println(outdoorWind);
  } else {
    snprintf(weatherLastError, sizeof(weatherLastError), "Podaci nisu validni");
  }

  return weatherOK;
}

void maintainOnlineServices(uint32_t now) {
  wifiConnected = WiFi.status() == WL_CONNECTED;

  if (!wifiConnected) {
    if ((uint32_t)(now - lastWiFiAttempt) >= WIFI_RETRY_MS) {
      lastWiFiAttempt = now;
      connectWiFi(5000);
    }
    return;
  }

  if (!ntpConfigured) configureNTP();

  if ((uint32_t)(now - lastClockSync) >= 1000UL) {
    lastClockSync = now;
    syncClockFromNTP();
  }

  if (weatherOK) {
    if ((uint32_t)(now - lastWeatherFetch) >= weatherUpdateMs()) {
      fetchWeather();
    }
  } else {
    if ((uint32_t)(now - lastWeatherAttempt) >= WEATHER_RETRY_MS) {
      fetchWeather();
    }
  }
}

// =====================================================
// SCENE / SENSOR LOGIC
// =====================================================

// 0 calm, 1 snow, 2 heat, 3 rain, 4 unavailable
int sceneMode() {
  // Ako imamo internet prognozu, scena prati pravo vreme NAPOLJU.
  if (weatherOK) {
    if (weatherIsSnowCode(outdoorWeatherCode)) return 1;
    if (weatherIsRainCode(outdoorWeatherCode)) return 3;

    if (outdoorTemp >= 27.0f && outdoorWeatherCode <= 2) {
      return 2;
    }

    return 0;
  }

  // Offline fallback: stara DHT22 metafora.
  if (!sensorOK) return 4;
  if (temp <= COLD_LIMIT) return 1;
  if (temp >= HOT_LIMIT) return 2;
  return rh >= 70 ? 3 : 0;
}

const char* statusText() {
  if (!haveReading) return "WAITING FOR DHT22";
  if (!sensorOK) return "SENSOR ERROR";
  if (temp <= COLD_LIMIT) return "COLD / SNOW MODE";
  if (temp >= HOT_LIMIT) return "HOT / HEATWAVE";
  if (rh >= 70) return "HIGH HUMIDITY";
  if (rh <= 30) return "DRY AIR";
  if (temp >= 20 && temp <= 26 && rh >= 40 && rh <= 60) return "PERFECT COMFORT";
  return "ROOM MONITOR";
}

void cloud(int x, int y, int s, uint16_t col) {
  frame.fillCircle(x + s, y + s, s, col);
  frame.fillCircle(x + 2 * s, y, s + 2, col);
  frame.fillCircle(x + 3 * s, y + s, s, col);
  frame.fillRoundRect(x, y + s, 4 * s, s + 4, 5, col);
}

const SpriteAsset& characterSpriteForMode(int mode) {
  // Two-frame idle animation. Weather variants:
  // mode 0 calm  -> normal
  // mode 1 cold  -> normal for now (snow outfit can be added later)
  // mode 2 heat  -> sunny
  // mode 3 rain  -> rain
  // mode 4 error -> normal
  int animFrame = 0;

  if (animationsEnabled) {
    animFrame = (elapsedMs / 650ULL) & 1;
  }

  if (femaleCharacter) {
    if (mode == 2) return animFrame ? female_sunny_1 : female_sunny_0;
    if (mode == 3) return animFrame ? female_rain_1  : female_rain_0;
    return animFrame ? female_normal_1 : female_normal_0;
  }

  if (mode == 2) return animFrame ? male_sunny_1 : male_sunny_0;
  if (mode == 3) return animFrame ? male_rain_1  : male_rain_0;
  return animFrame ? male_normal_1 : male_normal_0;
}

void drawCharacterSprite(int mode, int x, int y) {
  drawSpriteAlpha(frame, x, y, characterSpriteForMode(mode));
}

void drawSky(float sec, int mode) {
  uint16_t top = rgb(12, 46, 86);
  uint16_t bottom = rgb(59, 139, 169);

  if (mode == 1 || mode == 3) {
    top = rgb(21, 27, 43);
    bottom = rgb(76, 94, 110);
  }

  if (mode == 2) {
    top = rgb(86, 21, 34);
    bottom = rgb(220, 94, 29);
  }

  if (mode == 4) {
    top = BG;
    bottom = EDGE;
  }

  for (int yy = 0; yy < 86; yy++) {
    frame.drawFastHLine(0, yy, 320, blend(top, bottom, yy / 85.0f));
  }

  if (mode == 0 || mode == 2) {
    int cx = 275;
    int cy = 28;

    for (int r = 28; r > 16; --r) {
      frame.drawCircle(cx, cy, r, blend(bottom, ORANGE, (29 - r) / 20.0f));
    }

    for (int i = 0; i < 12; i++) {
      float a = i * TAU / 12 + sec * 0.12f;

      frame.drawLine(
        cx + cosf(a) * 22,
        cy + sinf(a) * 22,
        cx + cosf(a) * 28,
        cy + sinf(a) * 28,
        ORANGE
      );
    }

    frame.fillCircle(cx, cy, 16, rgb(255, 218, 106));
    frame.fillCircle(cx - 4, cy - 4, 7, rgb(255, 237, 169));
  }

  uint16_t cc = (mode == 1 || mode == 3)
                  ? rgb(98, 112, 132)
                  : rgb(204, 225, 235);

  for (int i = 0; i < 4; i++) {
    if (mode == 2 && i > 0) break;
    cloud((int)clouds[i], 28 + (i % 2) * 17, 7 + i % 3, cc);
  }

  // mountain silhouette
  for (int xx = 0; xx < 320; xx++) {
    int yy = 76 + 4 * sinf(xx * 0.022f) + 3 * sinf(xx * 0.065f);
    frame.drawFastVLine(xx, yy, 86 - yy, blend(bottom, BG, 0.55f));
  }

  if (mode == 1 || mode == 3) {
    for (int i = 0; i < 30; i++) {
      float speed = mode == 1 ? 11 + (i % 4) * 4 : 70 + (i % 4) * 9;
      int yy = (int)fmodf(i * 19.0f + sec * speed, 86.0f);
      int xx = (int)(i * 37 + 12 * sinf(sec * 0.7f + i)) % 320;

      if (xx < 0) xx += 320;

      if (mode == 1) {
        frame.fillCircle(xx, yy, i % 3 == 0 ? 2 : 1, WHITE);

        if (i % 5 == 0) {
          frame.drawFastHLine(xx - 3, yy, 7, WHITE);
          frame.drawFastVLine(xx, yy - 3, 7, WHITE);
        }
      } else {
        frame.drawLine(xx, yy, xx - 3, yy + 6, CYAN);
      }
    }
  }

  if (mode == 2) {
    for (int xx = -8; xx < 328; xx += 8) {
      int h = 12 + 9 * (1 + sinf(xx * 0.18f + sec * 5)) + 5 * sinf(xx + sec * 8);
      int sway = 4 * sinf(sec * 4 + xx);

      frame.fillTriangle(xx - 8, 85, xx + sway, 85 - h, xx + 10, 85, RED);
      frame.fillTriangle(xx - 5, 85, xx + sway, 89 - h, xx + 7, 85, ORANGE);
      frame.fillTriangle(xx - 2, 85, xx, 94 - h, xx + 4, 85, rgb(255, 234, 130));
    }
  }

  // readable labels above animation
  frame.fillRoundRect(7, 6, 185, 19, 5, BG);
  label(14, 12, "MARTIN / ATMOSPHERE", WHITE);

  frame.fillRoundRect(7, 62, 180, 18, 5, BG);
  label(14, 68, statusText(), mode == 2 ? ORANGE : (mode == 1 ? CYAN : WHITE));
}

// =====================================================
// DHT22
// =====================================================

void sampleSensor() {
  attemptedReading = true;
  dataDirty = true;

  float t = dht.readTemperature();
  float h = dht.readHumidity();

  Serial.print("T: ");
  Serial.print(t);
  Serial.print(" | H: ");
  Serial.println(h);

  sensorOK = isfinite(t) && isfinite(h) && t >= -40 && t <= 80 && h >= 0 && h <= 100;

  history[historyHead] = sensorOK ? t : NAN;
  historyHum[historyHead] = sensorOK ? h : NAN;
  historyHead = (historyHead + 1) % N;

  if (historySize < N) {
    ++historySize;
  }

  if (sensorOK) {
    temp = t;
    rh = h;

    if (!haveReading) {
      lowTemp = t;
      highTemp = t;
      lowHum = h;
      highHum = h;
    }

    lowTemp = fminf(lowTemp, t);
    highTemp = fmaxf(highTemp, t);
    lowHum = fminf(lowHum, h);
    highHum = fmaxf(highHum, h);
    haveReading = true;
  }
}

// =====================================================
// GRAPH
// =====================================================

void drawGraph() {
  panel(7, 149, 306, 57);

  label(15, 155, "TEMP / C", ORANGE);
  label(205, 155, "60 SLOTS", MUTED);

  float lo = 100;
  float hi = -100;

  for (int i = 0; i < historySize; i++) {
    float v = history[(historyHead - historySize + i + N) % N];

    if (isfinite(v)) {
      lo = fminf(lo, v);
      hi = fmaxf(hi, v);
    }
  }

  if (lo > hi) {
    lo = 18;
    hi = 28;
  }

  float mid = (lo + hi) / 2;
  float span = fmaxf(hi - lo + 2, 4);

  lo = mid - span / 2;
  hi = mid + span / 2;

  const int x0 = 47;
  const int y0 = 170;
  const int w = 252;
  const int h = 22;

  for (int j = 0; j < 3; j++) {
    frame.drawFastHLine(x0, y0 + j * h / 2, w, EDGE);
  }

  char b[28];

  snprintf(b, sizeof(b), "%.0f", hi);
  label(15, y0, b, MUTED);

  snprintf(b, sizeof(b), "%.0f", lo);
  label(15, y0 + h - 7, b, MUTED);

  bool previous = false;
  int px = 0;
  int py = 0;

  for (int i = 0; i < historySize; i++) {
    float v = history[(historyHead - historySize + i + N) % N];

    if (!isfinite(v)) {
      previous = false;
      continue;
    }

    int xx = x0 + (N - historySize + i) * (w - 1) / (N - 1);
    int yy = y0 + h - (int)((v - lo) / span * h);

    if (previous) {
      frame.drawLine(px, py, xx, yy, ORANGE);
    }

    frame.fillCircle(xx, yy, 1, ORANGE);

    px = xx;
    py = yy;
    previous = true;
  }

  if (haveReading) {
    snprintf(b, sizeof(b), "MIN %.1f  MAX %.1f", lowTemp, highTemp);
  } else {
    snprintf(b, sizeof(b), "WAITING FOR DATA");
  }

  label(47, 195, b, MUTED);
}


// =====================================================
// LARGE SPRITE HOME SCENE
// =====================================================


void drawAmbientSkyDetail() {
  if (!weatherOK || outdoorIsDay || !animationsEnabled) return;

  uint32_t now = millis();

  if (nextSkySparkAt == 0) {
    nextSkySparkAt = now + (uint32_t)random(18000, 36001);
  }

  if (!skySparkActive && (int32_t)(now - nextSkySparkAt) >= 0) {
    skySparkActive = true;
    skySparkStartedAt = now;
  }

  if (skySparkActive) {
    uint32_t age = now - skySparkStartedAt;

    if (age < 950) {
      float p = age / 950.0f;
      int x = 25 + (int)(p * 105.0f);
      int y = 28 + (int)(p * 45.0f);

      uint16_t c = rgb(245, 246, 220);
      frame.drawLine(x, y, x - 14, y - 6, c);
      frame.drawLine(x - 4, y - 2, x - 20, y - 8, MUTED);
      frame.fillCircle(x, y, 1, WHITE);
    } else {
      skySparkActive = false;
      nextSkySparkAt = now + (uint32_t)random(18000, 36001);
    }
  }
}

void drawHomeNature(float sec, int mode) {
  const int sceneBottom = 180;

  uint16_t top = rgb(15, 58, 105);
  uint16_t bottom = rgb(88, 174, 201);

  if (mode == 1 || mode == 3) {
    top = rgb(25, 34, 54);
    bottom = rgb(92, 112, 130);
  }

  if (mode == 2) {
    top = rgb(38, 112, 181);
    bottom = rgb(122, 201, 222);
  }

  if (mode == 4) {
    top = BG;
    bottom = EDGE;
  }

  if (weatherOK && !outdoorIsDay) {
    top = rgb(5, 15, 38);
    bottom = rgb(18, 42, 72);
  }

  // Sky gradient.
  for (int yy = 0; yy < 151; ++yy) {
    frame.drawFastHLine(0, yy, 320, blend(top, bottom, yy / 150.0f));
  }

  // Distant hills.
  uint16_t hillFar = blend(bottom, rgb(52, 104, 93), 0.58f);
  uint16_t hillNear = rgb(48, 112, 72);

  for (int xx = 0; xx < 320; ++xx) {
    int y1 = 125 + 8 * sinf(xx * 0.020f) + 4 * sinf(xx * 0.055f);
    int y2 = 142 + 6 * sinf(xx * 0.030f + 1.2f);

    frame.drawFastVLine(xx, y1, 151 - y1, hillFar);
    frame.drawFastVLine(xx, y2, 151 - y2, hillNear);
  }

  // Ground.
  frame.fillRect(0, 151, 320, sceneBottom - 151, rgb(70, 146, 70));
  frame.fillRect(0, 174, 320, 6, rgb(49, 113, 61));

  // Sun / moon.
  if ((mode == 0 || mode == 2) && (!weatherOK || outdoorIsDay)) {
    const int sx = 270;
    const int sy = 35;

    frame.fillCircle(sx, sy, 17, rgb(255, 216, 85));
    frame.fillCircle(sx - 5, sy - 5, 6, rgb(255, 239, 164));

    for (int i = 0; i < 10; ++i) {
      float a = i * TAU / 10.0f + (animationsEnabled ? sec * 0.20f : 0.0f);

      frame.drawLine(
        sx + cosf(a) * 23,
        sy + sinf(a) * 23,
        sx + cosf(a) * 30,
        sy + sinf(a) * 30,
        rgb(255, 221, 92)
      );
    }
  }
  else if (weatherOK && !outdoorIsDay) {
    frame.fillCircle(270, 35, 16, rgb(230, 235, 205));
    frame.fillCircle(277, 29, 15, top);

    for (int i = 0; i < 16; ++i) {
      int sx = (i * 37 + 15) % 320;
      int sy = (i * 23 + 8) % 90;
      frame.drawPixel(sx, sy, (i % 3 == 0) ? WHITE : MUTED);
    }
  }

  drawAmbientSkyDetail();

  // Moving clouds.
  uint16_t cloudCol = (mode == 1 || mode == 3)
                        ? rgb(105, 119, 139)
                        : rgb(219, 235, 241);

  for (int i = 0; i < 3; ++i) {
    int cx = (int)clouds[i];

    if (mode == 2 && i > 0) break;

    cloud(cx, 32 + i * 20, 7 + (i % 2) * 2, cloudCol);
  }

  // Small procedural tree on the right side of the scene.
  const int treeX = 184;
  const int treeY = 104;

  frame.fillRoundRect(treeX - 4, treeY + 23, 8, 42, 3, rgb(107, 70, 38));
  frame.fillCircle(treeX, treeY + 18, 19, rgb(48, 126, 61));
  frame.fillCircle(treeX - 13, treeY + 25, 14, rgb(59, 145, 68));
  frame.fillCircle(treeX + 14, treeY + 26, 15, rgb(43, 117, 55));

  // Weather particles.
  if (mode == 3) {
    for (int i = 0; i < 30; ++i) {
      int xx = (i * 43 + (int)(sec * (42 + (i % 3) * 7))) % 320;
      int yy = (i * 31 + (int)(sec * (74 + (i % 4) * 9))) % 150;

      frame.drawLine(xx, yy, xx - 2, yy + 7, CYAN);
    }
  }

  if (mode == 1) {
    for (int i = 0; i < 28; ++i) {
      int xx = (i * 47 + (int)(sec * (6 + i % 4))) % 320;
      int yy = (i * 29 + (int)(sec * (12 + i % 5))) % 150;

      frame.fillCircle(xx, yy, (i % 4 == 0) ? 2 : 1, WHITE);
    }

    frame.fillRect(0, 166, 320, 14, rgb(220, 233, 239));
  }

  // Character: base sprite + lightweight personality movement.
  int characterX;
  int characterY;
  bool characterVisible;
  getCharacterPose(characterX, characterY, characterVisible);

  if (characterVisible) {
    drawCharacterSprite(mode, characterX, characterY);

    // Shadow follows the character horizontally.
    int shadowY = 171;
    frame.fillEllipse(characterX + 36, shadowY, 28, 4, rgb(42, 93, 53));
  }

  drawCharacterEventProps(characterX, characterY, characterVisible);
}


void drawStatusBar(const char* pageName, const char* sourceTag) {
  if (uiTheme == THEME_NATURE) {
    frame.fillRoundRect(5, 4, 310, 24, 7, rgb(7, 31, 54));
    frame.drawRoundRect(5, 4, 310, 24, 7, blend(themeEdge(), WHITE, 0.18f));
    label(12, 12, pageName, WHITE, 1);

    if (sourceTag && sourceTag[0]) {
      int sw = strlen(sourceTag) * 6 + 12;
      int sx = 160 - sw / 2;
      frame.fillRoundRect(sx, 8, sw, 15, 5, rgb(21, 67, 91));
      label(sx + 6, 12, sourceTag, themeAccent(), 1);
    }
  }
  else if (uiTheme == THEME_OBSIDIAN) {
    frame.fillRoundRect(7, 5, 306, 23, 11, rgb(9, 14, 29));
    frame.drawRoundRect(7, 5, 306, 23, 11, rgb(45, 61, 101));
    frame.drawFastHLine(18, 8, 70, rgb(72, 98, 185));
    label(16, 12, pageName, WHITE, 1);

    if (sourceTag && sourceTag[0]) {
      int sw = strlen(sourceTag) * 6 + 14;
      int sx = 162 - sw / 2;
      frame.fillRoundRect(sx, 8, sw, 15, 7, rgb(25, 26, 54));
      label(sx + 7, 12, sourceTag, themeAccent2(), 1);
    }
  }
  else {
    frame.fillRect(4, 4, 312, 24, rgb(4, 34, 32));
    frame.drawRect(4, 4, 312, 24, themeAccent());
    label(10, 12, pageName, themeAccent(), 1);

    if (sourceTag && sourceTag[0]) {
      int sw = strlen(sourceTag) * 6 + 8;
      int sx = 160 - sw / 2;
      frame.fillRect(sx, 8, sw, 15, rgb(8, 50, 44));
      label(sx + 4, 12, sourceTag, themeAccent2(), 1);
    }
  }

  int hr = secondsToday / 3600;
  int mn = (secondsToday / 60) % 60;
  char tb[8];
  snprintf(tb, sizeof(tb), "%02d:%02d", hr, mn);
  label(250, 12, tb, themeMuted(), 1);
  drawWiFiIcon(304, 19);
}

void drawSourcePill(int x, int y, const char* text, uint16_t c) {
  int w = strlen(text) * 6 + 14;
  frame.fillRoundRect(x, y, w, 16, 5, blend(c, BG, 0.45f));
  frame.drawRoundRect(x, y, w, 16, 5, c);
  label(x + 7, y + 5, text, WHITE, 1);
}

void applySoftwareNightDim(int first, int count) {
  // Night UI intentionally removed. Keep function as a no-op for transfer code.
  (void)first;
  (void)count;
}

// =====================================================
// HOME PAGES
// =====================================================

void drawHomeNatureTheme(float sec) {
  int mode = sceneMode();
  drawHomeNature(sec, mode);

  frame.fillRoundRect(8, 38, 112, 49, 9, rgb(9, 39, 66));
  frame.drawRoundRect(8, 38, 112, 49, 9, blend(themeEdge(), WHITE, 0.25f));
  frame.fillRoundRect(8, 94, 112, 49, 9, rgb(9, 39, 66));
  frame.drawRoundRect(8, 94, 112, 49, 9, blend(themeEdge(), WHITE, 0.25f));

  label(15, 45, "UNUTRA / TEMP", themeMuted());
  label(15, 101, "UNUTRA / VLAGA", themeMuted());

  char b[40];
  if (sensorOK) snprintf(b, sizeof(b), "%.1f %s", showTemp(temp), tempUnit());
  else snprintf(b, sizeof(b), "--.- %s", tempUnit());
  label(15, 60, b, ORANGE, 2);

  if (sensorOK) snprintf(b, sizeof(b), "%.0f %%", rh);
  else snprintf(b, sizeof(b), "-- %%");
  label(15, 116, b, CYAN, 2);

  frame.fillRoundRect(7, 6, 305, 22, 6, rgb(7, 31, 54));
  label(14, 13, "ATMOSPHERE", WHITE);
  label(98, 13, activeCity().name, CYAN);

  frame.fillRoundRect(8, 149, 216, 23, 6, rgb(7, 31, 54));
  if (weatherOK) {
    snprintf(b, sizeof(b), "NAPOLJU %.0f %s / %s",
             showTemp(outdoorTemp), tempUnit(), weatherLabel(outdoorWeatherCode));
  } else {
    snprintf(b, sizeof(b), "NAPOLJU: OFFLINE");
  }
  label(15, 156, b, weatherOK ? WHITE : themeMuted());
  label(226, 156, comfortLabel(), GREEN, 1);
  drawWiFiIcon(304, 19);

  frame.fillRect(0, 180, 320, 60, themeBg());
  buttonCanvas(82, 180, 156, 43, "MENI", CYAN, WHITE, 2);

  char dateBuf[16], timeBuf[8];
  int hr = secondsToday / 3600;
  int mn = (secondsToday / 60) % 60;
  snprintf(dateBuf, sizeof(dateBuf), "%02d.%02d.%04d", dayNow, monthNow, yearNow);
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", hr, mn);

  label(7, 226, dateBuf, themeMuted(), 1);
  int timeX = 320 - ((int)strlen(timeBuf) * 6) - 8;
  label(timeX, 226, timeBuf, WHITE, 1);
}

void drawHomeObsidianTheme(float sec) {
  drawThemeBackdrop(true);

  int mode = sceneMode();

  // Premium atmospheric glow instead of landscape.
  for (int r = 58; r > 8; r -= 5) {
    uint16_t c = blend(rgb(6, 10, 22), themeAccent(), (58 - r) / 95.0f);
    frame.drawCircle(267, 90, r, c);
  }

  // Character and event props.
  int characterX, characterY;
  bool characterVisible;
  getCharacterPose(characterX, characterY, characterVisible);
  characterX -= 2;
  characterY += 2;

  // Hero outdoor card.
  frame.fillRoundRect(12, 39, 188, 72, 14, rgb(11, 18, 34));
  frame.drawRoundRect(12, 39, 188, 72, 14, rgb(44, 58, 97));
  frame.drawFastHLine(28, 45, 75, themeAccent());

  label(24, 51, "NAPOLJU", themeMuted(), 1);

  char b[48];
  if (weatherOK) {
    snprintf(b, sizeof(b), "%.0f %s", showTemp(outdoorTemp), tempUnit());
  } else {
    snprintf(b, sizeof(b), "-- %s", tempUnit());
  }
  label(23, 67, b, WHITE, 3);

  if (weatherOK) label(24, 96, weatherLabel(outdoorWeatherCode), themeAccent(), 1);
  else label(24, 96, "OFFLINE", themeMuted(), 1);

  // Indoor compact cards.
  frame.fillRoundRect(12, 120, 91, 45, 12, rgb(13, 21, 39));
  frame.drawRoundRect(12, 120, 91, 45, 12, rgb(37, 53, 83));
  frame.fillRoundRect(109, 120, 91, 45, 12, rgb(13, 21, 39));
  frame.drawRoundRect(109, 120, 91, 45, 12, rgb(37, 53, 83));

  label(20, 127, "UNUTRA", themeMuted(), 1);
  label(117, 127, "VLAGA", themeMuted(), 1);

  if (sensorOK) snprintf(b, sizeof(b), "%.1f %s", showTemp(temp), tempUnit());
  else snprintf(b, sizeof(b), "--.-");
  label(20, 143, b, themeAccent2(), 2);

  if (sensorOK) snprintf(b, sizeof(b), "%.0f%%", rh);
  else snprintf(b, sizeof(b), "--%%");
  label(117, 143, b, themeAccent(), 2);

  // Header.
  frame.fillRoundRect(10, 6, 300, 24, 12, rgb(8, 13, 27));
  frame.drawRoundRect(10, 6, 300, 24, 12, rgb(38, 50, 82));
  label(18, 13, "ATMOSPHERE / OBSIDIAN", WHITE);
  drawWiFiIcon(301, 19);

  // Bottom dock.
  frame.fillRoundRect(10, 181, 300, 50, 15, rgb(8, 13, 27));
  frame.drawRoundRect(10, 181, 300, 50, 15, rgb(38, 50, 82));

  char dateBuf[16], timeBuf[8];
  int hr = secondsToday / 3600;
  int mn = (secondsToday / 60) % 60;
  snprintf(dateBuf, sizeof(dateBuf), "%02d.%02d.%04d", dayNow, monthNow, yearNow);
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", hr, mn);

  label(20, 199, dateBuf, themeMuted(), 1);
  label(264, 199, timeBuf, WHITE, 1);
  buttonCanvas(86, 185, 148, 40, "MENI", rgb(45, 75, 170), WHITE, 2);
  // Draw character last so cards cannot hide the movement.
  if (characterVisible) {
    drawCharacterSprite(mode, characterX, characterY);
    frame.fillEllipse(characterX + 36, 173, 27, 4, rgb(12, 17, 31));
  }
  drawCharacterEventProps(characterX, characterY, characterVisible);
}

void drawHomeRetroTheme(float sec) {
  drawThemeBackdrop(true);

  int mode = sceneMode();

  // Boxy CRT header.
  frame.fillRect(7, 6, 306, 24, rgb(5, 38, 35));
  frame.drawRect(7, 6, 306, 24, themeAccent());
  label(14, 13, "ATMOS//RETRO", themeAccent());
  label(111, 13, activeCity().name, themeAccent2());
  drawWiFiIcon(304, 19);

  // Character stage.
  frame.fillRect(190, 42, 122, 133, rgb(3, 30, 29));
  frame.drawRect(190, 42, 122, 133, themeAccent());
  frame.drawFastHLine(194, 46, 46, themeAccent2());

  int characterX, characterY;
  bool characterVisible;
  getCharacterPose(characterX, characterY, characterVisible);
  characterX -= 28;

  // Data terminal blocks.
  frame.fillRect(8, 42, 174, 55, rgb(5, 42, 39));
  frame.drawRect(8, 42, 174, 55, themeAccent());
  label(15, 49, "[OUTDOOR]", themeAccent(), 1);

  char b[48];
  if (weatherOK) {
    snprintf(b, sizeof(b), "%.0f%s  %s",
             showTemp(outdoorTemp), tempUnit(), weatherLabel(outdoorWeatherCode));
  } else {
    snprintf(b, sizeof(b), "OFFLINE");
  }
  label(15, 66, b, WHITE, 2);

  frame.fillRect(8, 104, 82, 63, rgb(5, 42, 39));
  frame.drawRect(8, 104, 82, 63, themeAccent());
  frame.fillRect(99, 104, 83, 63, rgb(5, 42, 39));
  frame.drawRect(99, 104, 83, 63, themeAccent());

  label(14, 111, "IN/TEMP", themeMuted());
  label(105, 111, "IN/HUM", themeMuted());

  if (sensorOK) snprintf(b, sizeof(b), "%.1f%s", showTemp(temp), tempUnit());
  else snprintf(b, sizeof(b), "--");
  label(14, 132, b, themeAccent2(), 2);

  if (sensorOK) snprintf(b, sizeof(b), "%.0f%%", rh);
  else snprintf(b, sizeof(b), "--");
  label(105, 132, b, themeAccent(), 2);

  // Bottom command bar.
  frame.fillRect(7, 183, 306, 49, rgb(4, 34, 32));
  frame.drawRect(7, 183, 306, 49, themeAccent());

  char dateBuf[16], timeBuf[8];
  int hr = secondsToday / 3600;
  int mn = (secondsToday / 60) % 60;
  snprintf(dateBuf, sizeof(dateBuf), "%02d.%02d.%04d", dayNow, monthNow, yearNow);
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", hr, mn);

  label(14, 190, dateBuf, themeMuted(), 1);
  label(266, 190, timeBuf, WHITE, 1);
  buttonCanvas(82, 188, 156, 39, ">> MENI <<", rgb(7, 67, 58), themeAccent(), 1);
  // Draw character last so terminal panels cannot hide the run.
  if (characterVisible) {
    drawCharacterSprite(mode, characterX, characterY);
    frame.drawFastHLine(characterX + 8, 173, 58, rgb(34, 116, 94));
  }
  drawCharacterEventProps(characterX, characterY, characterVisible);
}


void drawHomeIndieTheme(float sec) {
  drawLofiWindowScene(true);

  int mode = sceneMode();
  int characterX, characterY;
  bool characterVisible;
  getCharacterPose(characterX, characterY, characterVisible);

  characterX -= 6;
  characterY -= 2;

  // Warm translucent cards.
  frame.fillRoundRect(10, 38, 155, 61, 12, rgb(48, 38, 62));
  frame.drawRoundRect(10, 38, 155, 61, 12, rgb(125, 91, 137));

  char b[48];
  label(20, 47, "NAPOLJU / LOFI", themeMuted(), 1);
  if (weatherOK) snprintf(b, sizeof(b), "%.0f %s  %s", showTemp(outdoorTemp), tempUnit(), weatherLabel(outdoorWeatherCode));
  else snprintf(b, sizeof(b), "OFFLINE");
  label(20, 66, b, WHITE, 2);

  frame.fillRoundRect(10, 107, 74, 53, 12, rgb(53, 41, 65));
  frame.fillRoundRect(91, 107, 74, 53, 12, rgb(53, 41, 65));
  frame.drawRoundRect(10, 107, 74, 53, 12, rgb(126, 94, 135));
  frame.drawRoundRect(91, 107, 74, 53, 12, rgb(126, 94, 135));

  label(18, 115, "UNUTRA", themeMuted());
  label(99, 115, "VLAGA", themeMuted());

  if (sensorOK) snprintf(b, sizeof(b), "%.1f%s", showTemp(temp), tempUnit());
  else snprintf(b, sizeof(b), "--");
  label(18, 134, b, themeAccent2(), 2);

  if (sensorOK) snprintf(b, sizeof(b), "%.0f%%", rh);
  else snprintf(b, sizeof(b), "--");
  label(99, 134, b, themeAccent(), 2);

  frame.fillRoundRect(8, 6, 304, 24, 10, rgb(39, 31, 54));
  label(16, 13, "MARTIN / LOFI ROOM", WHITE);
  drawWiFiIcon(304, 19);

  frame.fillRoundRect(8, 181, 304, 51, 14, rgb(42, 33, 55));
  char dateBuf[16], timeBuf[8];
  int hr = secondsToday / 3600;
  int mn = (secondsToday / 60) % 60;
  snprintf(dateBuf, sizeof(dateBuf), "%02d.%02d.%04d", dayNow, monthNow, yearNow);
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", hr, mn);

  label(17, 199, dateBuf, themeMuted(), 1);
  label(266, 199, timeBuf, WHITE, 1);
  buttonCanvas(90, 187, 140, 39, "MENI", rgb(121, 72, 112), WHITE, 2);
  // Draw character last so the lo-fi cards never mask the animation.
  if (characterVisible) {
    drawCharacterSprite(mode, characterX, characterY);
    frame.fillEllipse(characterX + 36, 172, 27, 4, rgb(45, 33, 48));
  }
  drawCharacterEventProps(characterX, characterY, characterVisible);
}

void drawHomeJapanTheme(float sec) {
  drawJapanScenery(true);

  int mode = sceneMode();
  int characterX, characterY;
  bool characterVisible;
  getCharacterPose(characterX, characterY, characterVisible);

  characterX -= 15;

  // Shoji-inspired info panels.
  frame.fillRoundRect(9, 40, 169, 55, 5, rgb(62, 43, 57));
  frame.drawRoundRect(9, 40, 169, 55, 5, rgb(220, 136, 159));

  char b[48];
  label(18, 48, "NAPOLJU", rgb(245, 181, 198), 1);
  if (weatherOK) snprintf(b, sizeof(b), "%.0f %s / %s", showTemp(outdoorTemp), tempUnit(), weatherLabel(outdoorWeatherCode));
  else snprintf(b, sizeof(b), "OFFLINE");
  label(18, 67, b, WHITE, 2);

  frame.fillRoundRect(9, 104, 82, 57, 5, rgb(62, 43, 57));
  frame.fillRoundRect(97, 104, 81, 57, 5, rgb(62, 43, 57));
  frame.drawRoundRect(9, 104, 82, 57, 5, rgb(192, 91, 124));
  frame.drawRoundRect(97, 104, 81, 57, 5, rgb(192, 91, 124));

  label(17, 112, "UNUTRA", themeMuted());
  label(105, 112, "VLAGA", themeMuted());

  if (sensorOK) snprintf(b, sizeof(b), "%.1f%s", showTemp(temp), tempUnit());
  else snprintf(b, sizeof(b), "--");
  label(17, 132, b, rgb(255, 203, 179), 2);

  if (sensorOK) snprintf(b, sizeof(b), "%.0f%%", rh);
  else snprintf(b, sizeof(b), "--");
  label(105, 132, b, themeAccent(), 2);

  // Header inspired by lacquer + paper.
  frame.fillRoundRect(7, 6, 306, 24, 5, rgb(74, 35, 49));
  frame.drawRoundRect(7, 6, 306, 24, 5, rgb(232, 139, 160));
  label(15, 13, "MARTIN / SAKURA", WHITE);
  drawWiFiIcon(304, 19);

  // Bottom bar.
  frame.fillRoundRect(7, 181, 306, 51, 6, rgb(61, 36, 49));
  char dateBuf[16], timeBuf[8];
  int hr = secondsToday / 3600;
  int mn = (secondsToday / 60) % 60;
  snprintf(dateBuf, sizeof(dateBuf), "%02d.%02d.%04d", dayNow, monthNow, yearNow);
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", hr, mn);

  label(16, 199, dateBuf, themeMuted(), 1);
  label(266, 199, timeBuf, WHITE, 1);
  buttonCanvas(90, 187, 140, 39, "MENI", rgb(150, 54, 76), WHITE, 2);
  // Draw character last so shoji cards never mask the animation.
  if (characterVisible) {
    drawCharacterSprite(mode, characterX, characterY);
    frame.fillEllipse(characterX + 36, 173, 28, 4, rgb(63, 48, 56));
  }
  drawCharacterEventProps(characterX, characterY, characterVisible);
}

void homeScenePage(float sec) {
  if (uiTheme == THEME_OBSIDIAN) {
    drawHomeObsidianTheme(sec);
  }
  else if (uiTheme == THEME_RETRO) {
    drawHomeRetroTheme(sec);
  }
  else if (uiTheme == THEME_INDIE) {
    drawHomeIndieTheme(sec);
  }
  else if (uiTheme == THEME_JAPAN) {
    drawHomeJapanTheme(sec);
  }
  else {
    drawHomeNatureTheme(sec);
  }

  // Speech bubble is intentionally drawn LAST, over every UI panel/theme.
  drawSpeechBubble();
}

void homeDashboardPage() {
  drawThemeBackdrop(true);

  centerText(15, "MARTIN / ROOM", CYAN, 2);

  panel(18, 55, 132, 102);
  panel(170, 55, 132, 102);

  label(38, 69, "TEMPERATURE", MUTED);
  label(199, 69, "HUMIDITY", MUTED);

  char b[24];

  if (sensorOK) snprintf(b, sizeof(b), "%.1f", showTemp(temp));
  else snprintf(b, sizeof(b), "--.-");

  label(33, 98, b, ORANGE, 4);
  label(119, 135, "C", ORANGE, 2);

  if (sensorOK) snprintf(b, sizeof(b), "%.1f", rh);
  else snprintf(b, sizeof(b), "--.-");

  label(184, 98, b, CYAN, 4);
  label(277, 135, "%", CYAN, 2);

  centerText(168, statusText(), sensorOK ? WHITE : RED);

  buttonCanvas(90, 193, 140, 38, "MENI", CYAN, WHITE, 2);
}

// =====================================================
// CLIMATE PAGE
// =====================================================


void drawClimateThemeScene(float sec, int mode) {
  // Top 0..86 px of KLIMA page. No character here by design.
  if (uiTheme == THEME_NATURE) {
    drawSky(sec, mode);
    return;
  }

  if (uiTheme == THEME_OBSIDIAN) {
    // Dark glass / aurora header.
    for (int y = 0; y < 86; ++y) {
      float f = y / 85.0f;
      uint16_t c = blend(rgb(3, 7, 18), rgb(18, 15, 42), f);
      frame.drawFastHLine(0, y, 320, c);
    }

    for (int r = 34; r > 8; r -= 5) {
      uint16_t c = blend(rgb(10, 15, 31), themeAccent(), (34 - r) / 70.0f);
      frame.drawCircle(267, 42, r, c);
    }

    // Sleek cloud
    uint16_t cc = rgb(104, 119, 153);
    frame.fillCircle(72, 47, 9, cc);
    frame.fillCircle(84, 42, 12, cc);
    frame.fillCircle(98, 48, 8, cc);
    frame.fillRoundRect(62, 49, 48, 10, 5, cc);

    frame.drawFastHLine(15, 76, 290, rgb(42, 55, 89));
    return;
  }

  if (uiTheme == THEME_RETRO) {
    // CRT terminal weather header.
    frame.fillRect(0, 0, 320, 86, rgb(3, 25, 24));

    for (int x = 0; x < 320; x += 16) {
      frame.drawFastVLine(x, 0, 86, rgb(6, 48, 43));
    }
    for (int y = 0; y < 86; y += 8) {
      frame.drawFastHLine(0, y, 320, rgb(6, 48, 43));
    }
    for (int y = 2; y < 86; y += 4) {
      frame.drawFastHLine(0, y, 320, rgb(3, 30, 28));
    }

    frame.drawRect(14, 14, 292, 58, themeAccent());
    frame.drawFastHLine(24, 22, 54, themeAccent2());

    uint16_t c = themeAccent();
    frame.drawCircle(235, 42, 10, c);
    frame.drawCircle(249, 38, 13, c);
    frame.drawCircle(264, 44, 9, c);
    frame.drawFastHLine(225, 51, 50, c);

    label(25, 35, "LOCAL CLIMATE", c, 1);
    label(25, 52, "SENSOR / LIVE", themeAccent2(), 1);
    return;
  }

  if (uiTheme == THEME_INDIE) {
    // Lo-fi mini scene: dusk window + city + lamp glow.
    for (int y = 0; y < 86; ++y) {
      float f = y / 85.0f;
      uint16_t c = blend(rgb(43, 31, 68), rgb(109, 65, 82), f);
      frame.drawFastHLine(0, y, 320, c);
    }

    frame.fillRoundRect(186, 10, 118, 67, 8, rgb(19, 24, 48));
    frame.drawRoundRect(186, 10, 118, 67, 8, rgb(141, 102, 148));
    frame.drawFastVLine(245, 14, 59, rgb(78, 67, 109));

    frame.fillCircle(277, 29, 10, rgb(245, 220, 180));
    frame.fillCircle(281, 26, 9, rgb(19, 24, 48));

    for (int i = 0; i < 7; ++i) {
      int x = 194 + i * 15;
      int h = 8 + (i * 7) % 24;
      frame.fillRect(x, 69 - h, 11, h, rgb(28, 32, 54));
      if (i % 2 == 0) frame.fillRect(x + 4, 61 - h / 2, 2, 2, rgb(244, 191, 112));
    }

    // Small desk lamp shape
    frame.fillTriangle(38, 25, 24, 49, 52, 49, rgb(230, 157, 92));
    frame.fillRect(36, 49, 4, 18, rgb(86, 58, 54));
    frame.drawFastHLine(16, 68, 72, rgb(102, 66, 67));
    return;
  }

  // JAPAN
  for (int y = 0; y < 86; ++y) {
    float f = y / 85.0f;
    uint16_t c = blend(rgb(59, 45, 79), rgb(229, 129, 140), f);
    frame.drawFastHLine(0, y, 320, c);
  }

  frame.fillCircle(246, 39, 17, rgb(250, 216, 190));

  // Distant ridge
  for (int x = 0; x < 320; ++x) {
    int y1 = 61 + (int)(7.0f * sinf(x * 0.025f));
    frame.drawFastVLine(x, y1, 86 - y1, rgb(74, 65, 84));
  }

  // Small torii
  frame.fillRect(28, 41, 5, 31, rgb(151, 45, 55));
  frame.fillRect(62, 41, 5, 31, rgb(151, 45, 55));
  frame.fillRect(21, 37, 53, 5, rgb(183, 52, 62));
  frame.fillRect(25, 46, 45, 3, rgb(162, 46, 56));

  // Sakura branch
  frame.drawLine(309, 11, 276, 60, rgb(82, 50, 58));
  frame.drawLine(296, 28, 266, 19, rgb(82, 50, 58));
  frame.drawLine(286, 42, 262, 53, rgb(82, 50, 58));

  for (int i = 0; i < 14; ++i) {
    int bx = 264 + (i * 17) % 49;
    int by = 11 + (i * 23) % 50;
    frame.fillCircle(bx, by, (i % 4 == 0) ? 3 : 2,
                     (i % 3 == 0) ? rgb(255, 198, 213) : rgb(242, 154, 182));
  }
}

void climatePage(float sec) {
  drawThemeBackdrop();

  drawClimateThemeScene(sec, sceneMode());
  drawStatusBar("KLIMA", "UNUTRA");

  panel(7, 91, 150, 52);
  panel(163, 91, 150, 52);

  label(15, 98, "UNUTRA / TEMP", MUTED);
  label(171, 98, "UNUTRA / VLAGA", MUTED);

  char b[24];

  if (sensorOK) snprintf(b, sizeof(b), "%.1f", temp);
  else snprintf(b, sizeof(b), "--.-");

  label(15, 111, b, ORANGE, 3);
  label(139, 130, tempUnit(), ORANGE);

  if (sensorOK) snprintf(b, sizeof(b), "%.1f", rh);
  else snprintf(b, sizeof(b), "--.-");

  label(171, 111, b, CYAN, 3);
  label(296, 130, "%", CYAN);

  drawGraph();

  buttonCanvas(100, 210, 120, 27, "NAZAD", RED, WHITE, 1);
}



void drawWeatherAlertBadge() {
  if (!alertsEnabled || !weatherOK) return;

  bool rainAlert = forecastRainChance[0] >= 60;
  bool hotAlert = outdoorTemp >= 32.0f;
  bool coldAlert = outdoorTemp <= 0.0f;

  if (!(rainAlert || hotAlert || coldAlert)) return;

  const char* txt = rainAlert ? "KISA" : (hotAlert ? "VRUCINA" : "MRAZ");
  uint16_t c = rainAlert ? CYAN : (hotAlert ? ORANGE : WHITE);

  frame.fillRoundRect(236, 34, 72, 18, 6, rgb(61, 42, 46));
  frame.drawRoundRect(236, 34, 72, 18, 6, c);
  int tw = strlen(txt) * 6;
  label(272 - tw / 2, 40, txt, c, 1);
}

// =====================================================
// OUTDOOR WEATHER PAGE
// =====================================================

void weatherPage() {
  drawThemeBackdrop();

  drawStatusBar("VREME", "NAPOLJU");

  // Theme-specific atmosphere behind weather data.
  if (uiTheme == THEME_INDIE) {
    frame.drawCircle(289, 47, 13, rgb(245, 217, 176));
    frame.drawCircle(294, 43, 12, themeBg());
    frame.drawFastHLine(11, 109, 298, rgb(91, 63, 91));
    for (int i = 0; i < 5; ++i) {
      int bx = 214 + i * 18;
      int bh = 8 + (i * 9) % 24;
      frame.fillRect(bx, 108 - bh, 12, bh, rgb(33, 34, 55));
    }
  }
  else if (uiTheme == THEME_JAPAN) {
    frame.drawLine(310, 32, 266, 90, rgb(88, 52, 60));
    for (int i = 0; i < 10; ++i) {
      int bx = 263 + (i * 19) % 50;
      int by = 37 + (i * 23) % 59;
      frame.fillCircle(bx, by, 2, rgb(245, 166, 192));
    }
    frame.fillCircle(223, 65, 15, rgb(249, 211, 190));
  }
  else if (uiTheme == THEME_OBSIDIAN) {
    frame.drawCircle(278, 72, 34, rgb(31, 42, 77));
    frame.drawCircle(278, 72, 48, rgb(18, 27, 54));
  }
  else if (uiTheme == THEME_RETRO) {
    for (int x = 205; x < 313; x += 12) frame.drawFastVLine(x, 34, 72, rgb(8, 59, 50));
  }

  if (uiTheme == THEME_OBSIDIAN) frame.drawFastHLine(15, 31, 130, themeAccent());
  if (uiTheme == THEME_RETRO) frame.drawFastHLine(12, 31, 296, themeAccent());

  char titleCity[28];
  snprintf(titleCity, sizeof(titleCity), "%s", activeCity().name);
  centerText(34, titleCity, CYAN, 1);
  drawWeatherAlertBadge();

  if (!weatherOK) {
    centerText(72, "NEMA VREMENSKIH PODATAKA", RED, 1);
    centerText(94, WiFi.status() == WL_CONNECTED ? "POKUSAVAM PONOVO..." : "WIFI NIJE POVEZAN", MUTED, 1);

    if (weatherLastError[0]) {
      centerText(118, weatherLastError, MUTED, 1);
    }

    buttonCanvas(100, 200, 120, 32, "NAZAD", RED, WHITE, 1);
    return;
  }

  drawWeatherIcon(58, 78, outdoorWeatherCode, 1);

  char b[40];

  snprintf(b, sizeof(b), "%.1f %s", showTemp(outdoorTemp), tempUnit());
  label(105, 48, b, WHITE, 4);

  label(105, 85, weatherLabel(outdoorWeatherCode), CYAN, 1);

  panel(13, 116, 294, 62);

  snprintf(b, sizeof(b), "OSECAJ %.1f %s", showTemp(outdoorFeels), tempUnit());
  label(24, 128, b, ORANGE, 1);

  snprintf(b, sizeof(b), "VLAGA %.0f %%", outdoorHumidity);
  label(24, 147, b, CYAN, 1);

  snprintf(b, sizeof(b), "VETAR %.0f km/h", outdoorWind);
  label(170, 128, b, WHITE, 1);

  snprintf(b, sizeof(b), "KISA %d %%", forecastRainChance[0]);
  label(170, 147, b, MUTED, 1);

  if (isfinite(forecastMax[0]) && isfinite(forecastMin[0])) {
    snprintf(b, sizeof(b), "MIN %.0f / MAX %.0f %s",
             showTemp(forecastMin[0]), showTemp(forecastMax[0]), tempUnit());
    centerText(169, b, MUTED, 1);
  }

  buttonCanvas(18, 195, 135, 38, "3 DANA", BLUE, WHITE, 1);
  buttonCanvas(167, 195, 135, 38, "NAZAD", RED, WHITE, 1);
}

// =====================================================
// 3-DAY FORECAST PAGE
// =====================================================

void forecastPage() {
  drawThemeBackdrop();
  drawStatusBar("PROGNOZA", "NAPOLJU");
  centerText(34, "3 DANA", CYAN, 1);

  if (!weatherOK) {
    centerText(90, "NEMA PROGNOZE", RED, 2);
    buttonCanvas(100, 200, 120, 32, "NAZAD", RED, WHITE, 1);
    return;
  }

  const char* names[3] = {"DANAS", "SUTRA", "PREKOSUTRA"};
  const int xs[3] = {8, 109, 210};

  for (int i = 0; i < 3; ++i) {
    panel(xs[i], 42, 94, 146);
    int tw = strlen(names[i]) * 6;
    label(xs[i] + (94 - tw) / 2, 51, names[i], WHITE, 1);

    drawWeatherIcon(xs[i] + 47, 88, forecastCode[i], 1);

    char b[24];
    snprintf(b, sizeof(b), "%.0f / %.0f %s", showTemp(forecastMax[i]), showTemp(forecastMin[i]), tempUnit());
    int bw = strlen(b) * 6;
    label(xs[i] + (94 - bw) / 2, 119, b, ORANGE, 1);

    snprintf(b, sizeof(b), "KISA %d%%", forecastRainChance[i]);
    bw = strlen(b) * 6;
    label(xs[i] + (94 - bw) / 2, 141, b, CYAN, 1);

    const char* wl = weatherLabel(forecastCode[i]);
    char shortLabel[15];
    snprintf(shortLabel, sizeof(shortLabel), "%.14s", wl);
    bw = strlen(shortLabel) * 6;
    label(xs[i] + (94 - bw) / 2, 162, shortLabel, MUTED, 1);
  }

  buttonCanvas(100, 200, 120, 32, "NAZAD", RED, WHITE, 1);
}

// =====================================================
// CLOCK
// =====================================================

void digit(int x, int y, int number, uint16_t color) {
  const uint8_t segments[] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,
    0x6D, 0x7D, 0x07, 0x7F, 0x6F
  };

  uint8_t bits = segments[number];

  for (int k = 0; k < 7; k++) {
    uint16_t c = (bits & (1 << k)) ? color : rgb(21, 36, 52);

    if (k == 0 || k == 3 || k == 6) {
      int dy = k == 0 ? 0 : (k == 3 ? 56 : 28);
      frame.fillRoundRect(x + 5, y + dy, 27, 5, 2, c);
    } else {
      int dx = (k == 1 || k == 2) ? 32 : 0;
      int dy = (k == 1 || k == 5) ? 5 : 33;
      frame.fillRoundRect(x + dx, y + dy, 5, 23, 2, c);
    }
  }
}

void clockPage(float sec) {
  if (uiTheme == THEME_NATURE) {
    for (int yy = 0; yy < 190; yy++) {
      float glow = (1 + sinf(yy * 0.024f + sec * 0.15f)) * 0.5f;
      frame.drawFastHLine(0, yy, 320, blend(themeBg(), rgb(18, 45, 63), glow));
    }

    for (int i = 0; i < 48; i++) {
      int xx = (i * 73 + 17) % 320;
      int yy = (i * 47 + 9) % 186;
      float tw = (1 + sinf(sec * 1.3f + i * 2.1f)) * 0.5f;
      frame.drawPixel(xx, yy, blend(themeEdge(), WHITE, tw));
    }
  }
  else if (uiTheme == THEME_OBSIDIAN) {
    drawThemeBackdrop(true);
    frame.fillRoundRect(27, 52, 266, 112, 18, rgb(8, 13, 27));
    frame.drawRoundRect(27, 52, 266, 112, 18, rgb(39, 52, 86));
  }
  else {
    drawThemeBackdrop(true);
    frame.drawRect(27, 52, 266, 112, themeAccent());
    frame.drawRect(31, 56, 258, 104, rgb(18, 102, 82));
  }

  drawStatusBar("SAT", "LOKALNO");

  int hr = secondsToday / 3600;
  int mn = (secondsToday / 60) % 60;
  int ss = secondsToday % 60;

  char b[40];

  snprintf(b, sizeof(b), "%02d.%02d.%04d.", dayNow, monthNow, yearNow);
  centerText(32, b, WHITE, 2);

  panel(34, 60, 252, 92);

  digit(48, 70, hr / 10, WHITE);
  digit(91, 70, hr % 10, WHITE);

  digit(170, 70, mn / 10, CYAN);
  digit(213, 70, mn % 10, CYAN);

  if (ss % 2 == 0) {
    frame.fillCircle(151, 87, 3, CYAN);
    frame.fillCircle(151, 109, 3, CYAN);
  }

  snprintf(b, sizeof(b), "%02d SECONDS", ss);
  centerText(140, b, MUTED);

  if (sensorOK) {
    snprintf(b, sizeof(b), "UNUTRA %.1f %s  VLAGA %.0f%%", showTemp(temp), tempUnit(), rh);
  } else {
    snprintf(b, sizeof(b), "DHT22: NO VALID READING");
  }

  centerText(169, b, sensorOK ? WHITE : RED);

  buttonCanvas(100, 202, 120, 32, "NAZAD", RED, WHITE, 1);
}

// =====================================================
// MENU
// =====================================================

void menuCardBase(int x, int y, int w, int h, uint16_t bg, const char* title) {
  frame.fillRoundRect(x, y, w, h, 8, bg);
  frame.drawRoundRect(x, y, w, h, 8, blend(bg, WHITE, 0.28f));
}

void menuCardTitleOverlay(int x, int y, int w, int h, uint16_t bg, const char* title) {
  // Always drawn after the artwork so icons/background can never cover text.
  uint16_t strip = blend(bg, rgb(4, 12, 24), 0.52f);
  frame.fillRoundRect(x + 4, y + h - 18, w - 8, 15, 4, strip);
  label(x + 9, y + h - 15, title, WHITE, 1);
}

void drawMenuClimateCard(int x, int y, int w, int h) {
  uint16_t sky = rgb(42, 128, 190);
  menuCardBase(x, y, w, h, sky, "KLIMA");

  // grass
  frame.fillRect(x + 2, y + h - 11, w - 4, 9, rgb(57, 139, 72));
  // cloud
  uint16_t cc = rgb(225, 237, 242);
  frame.fillCircle(x + w - 35, y + 14, 7, cc);
  frame.fillCircle(x + w - 27, y + 12, 9, cc);
  frame.fillCircle(x + w - 18, y + 15, 6, cc);
  frame.fillRoundRect(x + w - 42, y + 15, 30, 8, 4, cc);
  // small thermometer
  frame.drawRoundRect(x + 15, y + 8, 7, 20, 3, WHITE);
  frame.fillCircle(x + 18, y + 29, 5, ORANGE);
  frame.drawFastVLine(x + 18, y + 13, 14, ORANGE);
}

void drawMenuWeatherCard(int x, int y, int w, int h) {
  uint16_t bg = rgb(52, 75, 146);
  menuCardBase(x, y, w, h, bg, "VREME");

  // moon
  frame.fillCircle(x + 21, y + 15, 8, rgb(239, 236, 190));
  frame.fillCircle(x + 25, y + 12, 7, bg);
  // sun
  frame.fillCircle(x + 55, y + 15, 7, rgb(255, 211, 68));
  for (int i = 0; i < 8; ++i) {
    float a = i * TAU / 8.0f;
    frame.drawLine(
      x + 55 + cosf(a) * 10, y + 15 + sinf(a) * 10,
      x + 55 + cosf(a) * 13, y + 15 + sinf(a) * 13,
      rgb(255, 224, 105)
    );
  }
  // cloud
  uint16_t cc = rgb(218, 227, 237);
  frame.fillCircle(x + 94, y + 16, 7, cc);
  frame.fillCircle(x + 104, y + 13, 9, cc);
  frame.fillCircle(x + 114, y + 17, 6, cc);
  frame.fillRoundRect(x + 89, y + 17, 31, 7, 3, cc);
}

void drawMenuForecastCard(int x, int y, int w, int h) {
  uint16_t bg = rgb(33, 124, 128);
  menuCardBase(x, y, w, h, bg, "3 DANA");

  // Three tiny forecast tiles.
  for (int i = 0; i < 3; ++i) {
    int bx = x + 12 + i * 36;
    frame.fillRoundRect(bx, y + 7, 28, 23, 4, blend(bg, WHITE, 0.12f));

    if (i == 0) {
      frame.fillCircle(bx + 14, y + 17, 5, rgb(255, 215, 75));
    } else if (i == 1) {
      frame.fillCircle(bx + 11, y + 18, 5, rgb(220, 231, 236));
      frame.fillCircle(bx + 17, y + 16, 6, rgb(220, 231, 236));
    } else {
      frame.drawLine(bx + 8, y + 12, bx + 5, y + 23, CYAN);
      frame.drawLine(bx + 16, y + 12, bx + 13, y + 23, CYAN);
      frame.drawLine(bx + 24, y + 12, bx + 21, y + 23, CYAN);
    }
  }
}

void drawMenuClockCard(int x, int y, int w, int h) {
  uint16_t bg = rgb(101, 67, 153);
  menuCardBase(x, y, w, h, bg, "SAT");

  int cx = x + w - 31;
  int cy = y + 18;
  frame.fillCircle(cx, cy, 14, rgb(239, 236, 245));
  frame.drawCircle(cx, cy, 14, WHITE);
  frame.drawLine(cx, cy, cx, cy - 8, rgb(55, 49, 68));
  frame.drawLine(cx, cy, cx + 7, cy + 4, rgb(55, 49, 68));
  frame.fillCircle(cx, cy, 2, rgb(55, 49, 68));
}

void drawMenuWiFiCard(int x, int y, int w, int h) {
  uint16_t bg = rgb(25, 111, 151);
  menuCardBase(x, y, w, h, bg, "WIFI");

  int cx = x + w - 31;
  int cy = y + 24;
  uint16_t c = WiFi.status() == WL_CONNECTED ? WHITE : rgb(235, 170, 170);

  // Wi-Fi fan icon.
  frame.drawCircle(cx, cy, 5, c);
  frame.drawCircle(cx, cy, 10, c);
  frame.drawCircle(cx, cy, 15, c);
  frame.fillRect(cx - 18, cy + 1, 37, 19, bg);
  frame.fillCircle(cx, cy, 2, c);
}

void drawMenuSettingsCard(int x, int y, int w, int h) {
  uint16_t bg = rgb(86, 93, 108);
  menuCardBase(x, y, w, h, bg, "PODESAVANJA");

  int cx = x + w - 31;
  int cy = y + 18;

  // Simple gear-like icon.
  frame.drawCircle(cx, cy, 10, WHITE);
  frame.drawCircle(cx, cy, 4, WHITE);
  for (int i = 0; i < 8; ++i) {
    float a = i * TAU / 8.0f;
    int x1 = cx + cosf(a) * 11;
    int y1 = cy + sinf(a) * 11;
    int x2 = cx + cosf(a) * 16;
    int y2 = cy + sinf(a) * 16;
    frame.drawLine(x1, y1, x2, y2, WHITE);
  }
}



void drawPremiumTileScene(int x, int y, int w, int h, uint16_t accent, int iconType) {
  // Decorative mini-background inside non-Nature menu cards.
  // Everything is intentionally subtle so labels remain readable.
  int l = x + 3, t = y + 3, r = x + w - 4, b = y + h - 4;

  if (uiTheme == THEME_OBSIDIAN) {
    // Glass-tech grid + glow/orbit.
    for (int gx = l + 8; gx < r; gx += 18)
      frame.drawFastVLine(gx, t + 4, max(0, h - 8), rgb(18, 29, 55));
    for (int gy = t + 9; gy < b; gy += 14)
      frame.drawFastHLine(l + 5, gy, max(0, w - 10), rgb(18, 29, 55));

    int cx = r - 23, cy = t + h / 2;
    frame.drawCircle(cx, cy, min(15, h/3), blend(accent, rgb(11,18,34), 0.40f));
    frame.drawCircle(cx, cy, min(10, h/4), blend(accent, WHITE, 0.05f));
  }
  else if (uiTheme == THEME_RETRO) {
    // CRT scanlines + tiny terminal blocks.
    for (int gy = t + 5; gy < b; gy += 6)
      frame.drawFastHLine(l + 4, gy, max(0, w - 8), rgb(7, 58, 50));

    for (int i = 0; i < 4; ++i) {
      int px = r - 38 + i * 7;
      int ph = 5 + (i * 5 + iconType * 3) % max(6, h - 12);
      frame.fillRect(px, b - ph, 4, ph, blend(accent, rgb(5,42,39), 0.30f));
    }
  }
  else if (uiTheme == THEME_INDIE) {
    // Lo-fi dusk/window/cassette mood.
    frame.fillRoundRect(r - 54, t + 5, 46, max(16, h - 10), 5, rgb(34, 30, 57));
    frame.drawRoundRect(r - 54, t + 5, 46, max(16, h - 10), 5, rgb(113, 83, 128));
    frame.fillCircle(r - 18, t + 13, 5, rgb(245, 205, 157));
    frame.fillCircle(r - 16, t + 11, 5, rgb(34, 30, 57));
    frame.drawFastHLine(l + 8, b - 7, min(40, w - 20), rgb(116, 75, 78));
  }
  else if (uiTheme == THEME_JAPAN) {
    // Sakura + sun + tiny torii strokes.
    frame.fillCircle(r - 25, t + 13, 7, rgb(247, 199, 184));
    frame.drawLine(r - 6, t + 3, r - 29, b - 4, rgb(91, 54, 62));
    for (int i = 0; i < 5; ++i) {
      int bx = r - 10 - (i * 11) % max(12, w/3);
      int by = t + 7 + (i * 13) % max(10, h - 12);
      frame.fillCircle(bx, by, 1 + (i==0), rgb(247, 165, 190));
    }
    if (w >= 90 && h >= 38) {
      frame.fillRect(r - 58, b - 19, 3, 14, rgb(154, 47, 57));
      frame.fillRect(r - 40, b - 19, 3, 14, rgb(154, 47, 57));
      frame.fillRect(r - 62, b - 22, 29, 3, rgb(181, 54, 63));
    }
  }

  // Function-specific mini detail, kept on the right half.
  int ix = r - 19;
  int iy = t + h / 2;

  if (iconType == 0) { // climate
    frame.drawRoundRect(ix - 2, iy - 8, 5, 13, 2, WHITE);
    frame.fillCircle(ix, iy + 7, 4, accent);
  }
  else if (iconType == 1) { // weather
    frame.fillCircle(ix - 7, iy - 4, 5, rgb(255, 212, 70));
    frame.fillCircle(ix + 3, iy + 2, 6, rgb(218, 230, 238));
  }
  else if (iconType == 2) { // forecast
    for (int i=0;i<3;++i)
      frame.fillRoundRect(ix - 15 + i*9, iy - 6, 7, 12, 2, blend(accent, WHITE, 0.15f));
  }
  else if (iconType == 3) { // clock
    frame.drawCircle(ix, iy, 8, WHITE);
    frame.drawLine(ix, iy, ix, iy - 5, accent);
    frame.drawLine(ix, iy, ix + 4, iy + 2, accent);
  }
  else if (iconType == 4) { // wifi, background-free
    frame.drawLine(ix-8,iy-4,ix,iy-10,WHITE);
    frame.drawLine(ix,iy-10,ix+8,iy-4,WHITE);
    frame.drawLine(ix-5,iy,ix,iy-4,WHITE);
    frame.drawLine(ix,iy-4,ix+5,iy,WHITE);
    frame.fillCircle(ix,iy+4,1,accent);
  }
  else { // settings
    frame.drawCircle(ix, iy, 7, WHITE);
    frame.drawCircle(ix, iy, 2, accent);
  }
}

void drawPremiumMenuTile(int x, int y, int w, int h,
                         const char* title, const char* sub,
                         uint16_t accent, int iconType) {
  if (uiTheme == THEME_OBSIDIAN) {
    frame.fillRoundRect(x, y, w, h, 12, rgb(11, 18, 34));
    frame.drawRoundRect(x, y, w, h, 12, rgb(43, 57, 92));
    frame.drawFastHLine(x + 10, y + 5, min(54, w - 20), accent);
  } else if (uiTheme == THEME_RETRO) {
    frame.fillRect(x, y, w, h, rgb(5, 42, 39));
    frame.drawRect(x, y, w, h, accent);
    frame.drawFastHLine(x + 3, y + 3, min(30, w - 6), themeAccent2());
  } else {
    frame.fillRoundRect(x, y, w, h, 9, blend(accent, themeBg(), 0.48f));
    frame.drawRoundRect(x, y, w, h, 9, blend(accent, WHITE, 0.18f));
  }

  drawPremiumTileScene(x, y, w, h, accent, iconType);

  // Dark translucent-ish label plate (RGB565 solid approximation) keeps text readable.
  uint16_t plate = (uiTheme == THEME_RETRO) ? rgb(5, 42, 39) :
                   (uiTheme == THEME_OBSIDIAN ? rgb(11,18,34) :
                    blend(themeBg(), BLACK, 0.15f));
  int plateW = min(w - 12, max(52, (int)strlen(title) * 6 + 16));
  frame.fillRoundRect(x + 6, y + 7, plateW, 17, 5, plate);

  label(x + 10, y + 11, title, WHITE, 1);
  if (sub && sub[0] && h >= 40) {
    int sw = min(w - 12, max(52, (int)strlen(sub) * 6 + 12));
    frame.fillRoundRect(x + 6, y + h - 18, sw, 14, 4, plate);
    label(x + 10, y + h - 15, sub, themeMuted(), 1);
  }
}


void menuNatureLayout() {
  drawStatusBar("GLAVNI MENI", "NATURE");

  const int leftX = 10, rightX = 165, w = 145, h = 43;

  drawMenuClimateCard(leftX, 35, w, h);
  drawMenuWeatherCard(rightX, 35, w, h);
  drawMenuForecastCard(leftX, 86, w, h);
  drawMenuClockCard(rightX, 86, w, h);
  drawMenuWiFiCard(leftX, 137, w, h);
  drawMenuSettingsCard(rightX, 137, w, h);

  menuCardTitleOverlay(leftX, 35, w, h, rgb(42, 128, 190), "KLIMA");
  menuCardTitleOverlay(rightX, 35, w, h, rgb(52, 75, 146), "VREME");
  menuCardTitleOverlay(leftX, 86, w, h, rgb(33, 124, 128), "3 DANA");
  menuCardTitleOverlay(rightX, 86, w, h, rgb(101, 67, 153), "SAT");
  menuCardTitleOverlay(leftX, 137, w, h, rgb(25, 111, 151), "WIFI");
  menuCardTitleOverlay(rightX, 137, w, h, rgb(86, 93, 108), "PODESAVANJA");

  buttonCanvas(105, 200, 110, 32, "POCETNA", rgb(14, 54, 92), WHITE, 1);
}

void menuObsidianLayout() {
  drawStatusBar("CONTROL CENTER", "OBSIDIAN");

  // One hero card.
  drawPremiumMenuTile(10, 37, 300, 47, "VREME", "trenutno stanje napolju", themeAccent(), 1);

  // Two medium cards.
  drawPremiumMenuTile(10, 92, 145, 43, "KLIMA", "unutrasnji senzor", rgb(58, 153, 221), 0);
  drawPremiumMenuTile(165, 92, 145, 43, "3 DANA", "prognoza", rgb(75, 174, 170), 2);

  // Three compact cards.
  drawPremiumMenuTile(10, 143, 94, 39, "SAT", "", rgb(155, 94, 232), 3);
  drawPremiumMenuTile(113, 143, 94, 39, "WIFI", "", rgb(61, 151, 197), 4);
  drawPremiumMenuTile(216, 143, 94, 39, "SET", "", rgb(110, 116, 137), 5);

  buttonCanvas(108, 199, 104, 32, "HOME", rgb(32, 49, 99), WHITE, 1);
}

void menuRetroLayout() {
  drawStatusBar("MAIN//MENU", "RETRO");

  const int xs[3] = {8, 111, 214};

  drawPremiumMenuTile(xs[0], 43, 98, 58, "KLIMA", "IN", themeAccent(), 0);
  drawPremiumMenuTile(xs[1], 43, 98, 58, "VREME", "OUT", themeAccent2(), 1);
  drawPremiumMenuTile(xs[2], 43, 98, 58, "3 DANA", "FCST", rgb(95, 204, 255), 2);

  drawPremiumMenuTile(xs[0], 112, 98, 58, "SAT", "TIME", rgb(202, 132, 255), 3);
  drawPremiumMenuTile(xs[1], 112, 98, 58, "WIFI", "NET", rgb(61, 219, 186), 4);
  drawPremiumMenuTile(xs[2], 112, 98, 58, "SET", "CFG", rgb(255, 198, 72), 5);

  frame.fillRect(65, 192, 190, 34, rgb(5, 42, 39));
  frame.drawRect(65, 192, 190, 34, themeAccent());
  centerText(204, "[ RETURN HOME ]", themeAccent(), 1);
}


void menuIndieLayout() {
  drawStatusBar("LOFI MENU", "INDIE");

  // Cassette / album-card inspired 2-column layout.
  drawPremiumMenuTile(10, 39, 145, 54, "KLIMA", "room vibe", rgb(213, 126, 151), 0);
  drawPremiumMenuTile(165, 39, 145, 54, "VREME", "outside mood", rgb(237, 170, 112), 1);
  drawPremiumMenuTile(10, 101, 145, 54, "3 DANA", "next tracks", rgb(109, 146, 178), 2);
  drawPremiumMenuTile(165, 101, 145, 54, "SAT", "late night", rgb(153, 111, 177), 3);
  drawPremiumMenuTile(10, 163, 145, 44, "WIFI", "connected", rgb(93, 154, 143), 4);
  drawPremiumMenuTile(165, 163, 145, 44, "SET", "preferences", rgb(161, 117, 136), 5);

  frame.fillRoundRect(83, 213, 154, 22, 10, rgb(65, 48, 74));
  centerText(219, "HOME", themeAccent2(), 1);
}

void menuJapanLayout() {
  drawStatusBar("MENU", "SAKURA");

  // Vertical paper-card feeling with red seal accents.
  drawPremiumMenuTile(10, 39, 145, 50, "KLIMA", "uchi / indoor", rgb(206, 98, 132), 0);
  drawPremiumMenuTile(165, 39, 145, 50, "VREME", "sora / sky", rgb(231, 139, 111), 1);
  drawPremiumMenuTile(10, 97, 145, 50, "3 DANA", "forecast", rgb(174, 112, 155), 2);
  drawPremiumMenuTile(165, 97, 145, 50, "SAT", "time", rgb(131, 104, 153), 3);
  drawPremiumMenuTile(10, 155, 145, 50, "WIFI", "network", rgb(92, 132, 133), 4);
  drawPremiumMenuTile(165, 155, 145, 50, "SET", "settings", rgb(163, 77, 92), 5);

  frame.fillRoundRect(92, 212, 136, 23, 5, rgb(108, 38, 55));
  centerText(219, "POCETNA", rgb(255, 205, 216), 1);
}

void menuPage() {
  drawThemeBackdrop(true);

  if (uiTheme == THEME_OBSIDIAN) menuObsidianLayout();
  else if (uiTheme == THEME_RETRO) menuRetroLayout();
  else if (uiTheme == THEME_INDIE) menuIndieLayout();
  else if (uiTheme == THEME_JAPAN) menuJapanLayout();
  else menuNatureLayout();
}


// =====================================================
// WIFI INFO
// =====================================================

const char* wifiQualityLabel(int rssi) {
  if (rssi >= -55) return "ODLICAN";
  if (rssi >= -67) return "DOBAR";
  if (rssi >= -75) return "SOLIDAN";
  return "SLAB";
}

void wifiPage() {
  drawThemeBackdrop();

  drawStatusBar("WIFI", "SISTEM");

  panel(15, 40, 290, 138);

  bool connected = WiFi.status() == WL_CONNECTED;

  label(28, 54, "STATUS", MUTED);
  label(115, 54, connected ? "POVEZAN" : "NIJE POVEZAN", connected ? GREEN : RED, 1);

  label(28, 78, "MREZA", MUTED);
  if (connected) {
    String ssid = WiFi.SSID();
    char ssidBuf[27];
    snprintf(ssidBuf, sizeof(ssidBuf), "%.26s", ssid.c_str());
    label(115, 78, ssidBuf, WHITE, 1);
  } else {
    label(115, 78, "-", MUTED, 1);
  }

  label(28, 102, "IP", MUTED);
  if (connected) {
    String ip = WiFi.localIP().toString();
    label(115, 102, ip.c_str(), WHITE, 1);
  } else {
    label(115, 102, "-", MUTED, 1);
  }

  label(28, 126, "SIGNAL", MUTED);
  if (connected) {
    int rssi = WiFi.RSSI();
    char signalBuf[28];
    snprintf(signalBuf, sizeof(signalBuf), "%d dBm / %s", rssi, wifiQualityLabel(rssi));
    label(115, 126, signalBuf, WHITE, 1);
  } else {
    label(115, 126, "-", MUTED, 1);
  }

  label(28, 150, activeCity().name, MUTED);
  label(115, 150, weatherOK ? "ONLINE" : "OFFLINE", weatherOK ? GREEN : ORANGE, 1);

  drawWiFiIcon(282, 61);

  buttonCanvas(100, 197, 120, 35, "NAZAD", rgb(35, 86, 132), WHITE, 1);
}

// =====================================================
// SETTINGS
// =====================================================

void settingsPage() {
  drawThemeBackdrop();
  drawStatusBar("PODESAVANJA", "1/2");

  label(14, 38, "JEDINICA TEMPERATURE", MUTED);
  buttonCanvas(14, 50, 140, 31, "CELZIJUS / C", !useFahrenheit ? GREEN : PANEL, WHITE, 1);
  buttonCanvas(166, 50, 140, 31, "FARENHAJT / F", useFahrenheit ? GREEN : PANEL, WHITE, 1);

  label(14, 87, "OSVEZAVANJE VREMENA", MUTED);
  buttonCanvas(14, 99, 88, 31, "5 MIN", refreshMinutes == 5 ? BLUE : PANEL, WHITE, 1);
  buttonCanvas(116, 99, 88, 31, "15 MIN", refreshMinutes == 15 ? BLUE : PANEL, WHITE, 1);
  buttonCanvas(218, 99, 88, 31, "30 MIN", refreshMinutes == 30 ? BLUE : PANEL, WHITE, 1);

  label(14, 136, "GRAD ZA PROGNOZU", MUTED);
  buttonCanvas(14, 148, 92, 31, "ZRENJANIN", cityIndex == 0 ? CYAN : PANEL, WHITE, 1);
  buttonCanvas(114, 148, 92, 31, "NOVI SAD", cityIndex == 1 ? CYAN : PANEL, WHITE, 1);
  buttonCanvas(214, 148, 92, 31, "BEOGRAD", cityIndex == 2 ? CYAN : PANEL, WHITE, 1);

  buttonCanvas(14, 196, 90, 36, "NAZAD", rgb(45, 61, 78), WHITE, 1);
  buttonCanvas(114, 196, 92, 36, "ISTORIJA", rgb(39, 102, 126), WHITE, 1);
  buttonCanvas(216, 196, 90, 36, "VISE >", rgb(54, 92, 151), WHITE, 1);
}

void settingsAdvancedPage() {
  drawThemeBackdrop();
  drawStatusBar("PODESAVANJA", "2/2");

  label(10, 34, "TEMA", themeMuted(), 1);

  buttonCanvas(8, 44, 96, 27, "NATURE",
               uiTheme == THEME_NATURE ? rgb(37, 122, 104) : themePanel(), WHITE, 1);
  buttonCanvas(112, 44, 96, 27, "OBSIDIAN",
               uiTheme == THEME_OBSIDIAN ? rgb(63, 82, 178) : themePanel(), WHITE, 1);
  buttonCanvas(216, 44, 96, 27, "RETRO",
               uiTheme == THEME_RETRO ? rgb(18, 112, 91) : themePanel(), WHITE, 1);

  buttonCanvas(56, 76, 96, 27, "INDIE",
               uiTheme == THEME_INDIE ? rgb(135, 76, 118) : themePanel(), WHITE, 1);
  buttonCanvas(168, 76, 96, 27, "JAPAN",
               uiTheme == THEME_JAPAN ? rgb(151, 55, 78) : themePanel(), WHITE, 1);

  label(10, 108, "PONASANJE", themeMuted(), 1);
  buttonCanvas(8, 118, 96, 27, animationsEnabled ? "ANIM: ON" : "ANIM: OFF",
               animationsEnabled ? GREEN : themePanel(), WHITE, 1);
  buttonCanvas(112, 118, 96, 27, quietMode ? "QUIET: ON" : "QUIET: OFF",
               quietMode ? themeAccent() : themePanel(), WHITE, 1);
  buttonCanvas(216, 118, 96, 27, alertsEnabled ? "ALERT: ON" : "ALERT: OFF",
               alertsEnabled ? ORANGE : themePanel(), WHITE, 1);

  label(10, 150, "EKRAN / LIK", themeMuted(), 1);
  buttonCanvas(8, 160, 96, 27, portraitMode ? "LANDSCAPE" : "PORTRAIT",
               portraitMode ? themeAccent() : themePanel(), WHITE, 1);
  buttonCanvas(112, 160, 96, 27, ambientEnabled ? "AMBIENT" : "AMB: OFF",
               ambientEnabled ? BLUE : themePanel(), WHITE, 1);
  buttonCanvas(216, 160, 96, 27, femaleCharacter ? "ZENSKI" : "MUSKI",
               themeAccent(), WHITE, 1);

  buttonCanvas(8, 194, 122, 27, homeStyle == 0 ? "HOME: SCENA" : "HOME: DASH",
               homeStyle == 0 ? GREEN : themePanel(), WHITE, 1);
  buttonCanvas(190, 194, 122, 27,
               touchCalibrated ? "TOUCH: OK" : "TOUCH KALIB.",
               touchCalibrated ? rgb(39, 116, 92) : rgb(106, 72, 46),
               WHITE, 1);

  buttonCanvas(8, 226, 92, 12, "< PRETH", rgb(54, 92, 151), WHITE, 1);
  buttonCanvas(220, 226, 92, 12, "MENI", rgb(45, 61, 78), WHITE, 1);
}

void historyPage() {
  drawThemeBackdrop();
  drawStatusBar("ISTORIJA", "UNUTRA");

  panel(8, 35, 304, 72);
  label(16, 42, "TEMPERATURA", ORANGE, 1);

  float tLo = 100, tHi = -100;
  for (int i = 0; i < historySize; ++i) {
    float v = history[(historyHead - historySize + i + N) % N];
    if (isfinite(v)) { tLo = fminf(tLo, v); tHi = fmaxf(tHi, v); }
  }
  if (tLo > tHi) { tLo = 18; tHi = 28; }
  float tSpan = fmaxf(tHi - tLo, 2.0f);

  int x0 = 34, w = 266, y0 = 60, h = 36;
  frame.drawFastHLine(x0, y0 + h, w, EDGE);

  bool prev = false; int px = 0, py = 0;
  for (int i = 0; i < historySize; ++i) {
    float v = history[(historyHead - historySize + i + N) % N];
    if (!isfinite(v)) { prev = false; continue; }
    int xx = x0 + (N - historySize + i) * (w - 1) / (N - 1);
    int yy = y0 + h - (int)((v - tLo) / tSpan * h);
    if (prev) frame.drawLine(px, py, xx, yy, ORANGE);
    px = xx; py = yy; prev = true;
  }

  panel(8, 113, 304, 72);
  label(16, 120, "VLAGA", CYAN, 1);

  int hy0 = 138, hh = 36;
  prev = false;
  for (int i = 0; i < historySize; ++i) {
    float v = historyHum[(historyHead - historySize + i + N) % N];
    if (!isfinite(v)) { prev = false; continue; }
    int xx = x0 + (N - historySize + i) * (w - 1) / (N - 1);
    int yy = hy0 + hh - (int)(constrain(v, 0.0f, 100.0f) / 100.0f * hh);
    if (prev) frame.drawLine(px, py, xx, yy, CYAN);
    px = xx; py = yy; prev = true;
  }

  char b[50];
  if (haveReading) {
    snprintf(b, sizeof(b), "TEMP %.1f-%.1f %s | VLAGA %.0f-%.0f%%",
             showTemp(lowTemp), showTemp(highTemp), tempUnit(), lowHum, highHum);
  } else {
    snprintf(b, sizeof(b), "CEKAM DHT22 PODATKE");
  }
  centerText(190, b, MUTED, 1);

  buttonCanvas(100, 205, 120, 30, "NAZAD", rgb(45, 61, 78), WHITE, 1);
}

void ambientPage(float sec) {
  char b[50];
  int hr = secondsToday / 3600;
  int mn = (secondsToday / 60) % 60;

  if (uiTheme == THEME_NATURE) {
    drawThemeBackdrop(true);

    // Quiet nature horizon.
    frame.fillCircle(270, 54, 18, weatherOK && !outdoorIsDay ? rgb(231, 235, 211) : rgb(255, 218, 89));
    frame.fillRect(0, 181, 320, 59, rgb(27, 66, 54));

    snprintf(b, sizeof(b), "%02d:%02d", hr, mn);
    centerText(45, b, WHITE, 4);

    snprintf(b, sizeof(b), "%02d.%02d.%04d", dayNow, monthNow, yearNow);
    centerText(84, b, themeMuted(), 1);

    if (weatherOK) {
      snprintf(b, sizeof(b), "%.0f %s / %s", showTemp(outdoorTemp), tempUnit(), weatherLabel(outdoorWeatherCode));
      centerText(126, b, CYAN, 1);
      centerText(145, activeCity().name, themeMuted(), 1);
    }

    if (sensorOK) {
      snprintf(b, sizeof(b), "UNUTRA %.1f %s  %.0f%%  %s",
               showTemp(temp), tempUnit(), rh, comfortLabel());
      centerText(194, b, WHITE, 1);
    }
  }
  else if (uiTheme == THEME_OBSIDIAN) {
    drawThemeBackdrop(true);

    frame.fillRoundRect(18, 30, 284, 145, 18, rgb(8, 13, 27));
    frame.drawRoundRect(18, 30, 284, 145, 18, rgb(39, 52, 86));
    frame.drawFastHLine(37, 37, 90, themeAccent());

    snprintf(b, sizeof(b), "%02d:%02d", hr, mn);
    centerText(57, b, WHITE, 5);

    snprintf(b, sizeof(b), "%02d.%02d.%04d", dayNow, monthNow, yearNow);
    centerText(102, b, themeMuted(), 1);

    if (weatherOK) {
      snprintf(b, sizeof(b), "%s  %.0f %s", weatherLabel(outdoorWeatherCode), showTemp(outdoorTemp), tempUnit());
      centerText(128, b, themeAccent(), 1);
    }

    if (sensorOK) {
      snprintf(b, sizeof(b), "INDOOR %.1f %s / %.0f%%", showTemp(temp), tempUnit(), rh);
      centerText(151, b, themeAccent2(), 1);
    }

    frame.fillRoundRect(91, 190, 138, 27, 13, rgb(14, 20, 39));
    centerText(199, "TAP TO WAKE", themeMuted(), 1);
  }
  else if (uiTheme == THEME_RETRO) {
    drawThemeBackdrop(true);

    frame.drawRect(21, 31, 278, 154, themeAccent());
    frame.drawRect(26, 36, 268, 144, rgb(20, 102, 83));

    label(35, 45, "SYSTEM.IDLE", themeAccent2(), 1);

    snprintf(b, sizeof(b), "%02d:%02d", hr, mn);
    centerText(69, b, themeAccent(), 4);

    snprintf(b, sizeof(b), "DATE %02d.%02d.%04d", dayNow, monthNow, yearNow);
    centerText(109, b, themeMuted(), 1);

    if (weatherOK) {
      snprintf(b, sizeof(b), "OUT %.0f%s // %s", showTemp(outdoorTemp), tempUnit(), weatherLabel(outdoorWeatherCode));
      centerText(132, b, WHITE, 1);
    }

    if (sensorOK) {
      snprintf(b, sizeof(b), "IN %.1f%s // RH %.0f%%", showTemp(temp), tempUnit(), rh);
      centerText(151, b, WHITE, 1);
    }

    frame.fillRect(72, 201, 176, 24, rgb(5, 42, 39));
    frame.drawRect(72, 201, 176, 24, themeAccent());
    centerText(209, "[ TOUCH TO RETURN ]", themeAccent(), 1);
  }

  else if (uiTheme == THEME_INDIE) {
    drawLofiWindowScene(true);
    frame.fillRoundRect(30, 37, 260, 142, 18, rgb(42, 33, 55));
    frame.drawRoundRect(30, 37, 260, 142, 18, rgb(124, 91, 134));

    snprintf(b, sizeof(b), "%02d:%02d", hr, mn);
    centerText(58, b, WHITE, 5);

    snprintf(b, sizeof(b), "%02d.%02d.%04d", dayNow, monthNow, yearNow);
    centerText(103, b, themeMuted(), 1);

    if (weatherOK) {
      snprintf(b, sizeof(b), "%.0f%s  %s", showTemp(outdoorTemp), tempUnit(), weatherLabel(outdoorWeatherCode));
      centerText(130, b, themeAccent2(), 1);
    }
    centerText(208, "late night / tap to wake", themeMuted(), 1);
  }
  else {
    drawJapanScenery(true);
    frame.fillRoundRect(27, 35, 266, 142, 7, rgb(66, 39, 53));
    frame.drawRoundRect(27, 35, 266, 142, 7, rgb(220, 128, 154));

    snprintf(b, sizeof(b), "%02d:%02d", hr, mn);
    centerText(58, b, WHITE, 5);

    snprintf(b, sizeof(b), "%02d.%02d.%04d", dayNow, monthNow, yearNow);
    centerText(103, b, themeMuted(), 1);

    if (weatherOK) {
      snprintf(b, sizeof(b), "%s  %.0f%s", weatherLabel(outdoorWeatherCode), showTemp(outdoorTemp), tempUnit());
      centerText(132, b, rgb(255, 190, 205), 1);
    }
    centerText(207, "SAKURA / TOUCH", rgb(236, 156, 179), 1);
  }

  drawWiFiIcon(304, 19);
}


// =====================================================
// TFT TRANSFER
// =====================================================


// =====================================================
// PORTRAIT UI (240x320)
// Keeps the same data/features/themes; only layout changes.
// =====================================================

void pLabel(int x, int y, const char* s, uint16_t c = WHITE, int size = 1) {
  tft.setTextWrap(false);
  tft.setTextSize(size);
  tft.setTextColor(c);
  tft.setCursor(x, y);
  tft.print(s);
}

void pCenter(int y, const char* s, uint16_t c = WHITE, int size = 1) {
  int w = (int)strlen(s) * 6 * size;
  pLabel((240 - w) / 2, y, s, c, size);
}

void pPanel(int x, int y, int w, int h) {
  if (uiTheme == THEME_RETRO) {
    tft.fillRect(x, y, w, h, themePanel());
    tft.drawRect(x, y, w, h, themeEdge());
  } else {
    tft.fillRoundRect(x, y, w, h, 9, themePanel());
    tft.drawRoundRect(x, y, w, h, 9, themeEdge());
  }
}

void pButton(int x, int y, int w, int h, const char* s, uint16_t fill, int size = 1) {
  if (uiTheme == THEME_RETRO) {
    tft.fillRect(x, y, w, h, fill);
    tft.drawRect(x, y, w, h, themeEdge());
  } else {
    tft.fillRoundRect(x, y, w, h, 8, fill);
    tft.drawRoundRect(x, y, w, h, 8, themeEdge());
  }
  int tw = (int)strlen(s) * 6 * size;
  int tx = x + (w - tw) / 2;
  int ty = y + (h - 8 * size) / 2;
  pLabel(tx, ty, s, WHITE, size);
}






void pBackdrop() {
  // Portrait is intentionally instant: no transition animation.
  tft.fillScreen(themeBg());

  if (uiTheme == THEME_OBSIDIAN) {
    tft.drawCircle(207, 47, 28, rgb(27, 39, 78));
    tft.drawCircle(207, 47, 40, rgb(18, 27, 58));
  }
  else if (uiTheme == THEME_RETRO) {
    for (int x = 0; x < 240; x += 20) tft.drawFastVLine(x, 0, 320, rgb(7, 45, 42));
    for (int y = 0; y < 320; y += 16) tft.drawFastHLine(0, y, 240, rgb(7, 45, 42));
  }
  else if (uiTheme == THEME_INDIE) {
    tft.fillCircle(210, 43, 18, rgb(89, 61, 102));
    tft.fillCircle(215, 38, 16, themeBg());
  }
  else if (uiTheme == THEME_JAPAN) {
    tft.fillCircle(200, 45, 18, rgb(244, 188, 177));
    tft.drawLine(235, 15, 207, 72, rgb(92, 57, 65));
    for (int i = 0; i < 8; ++i) {
      int bx = 204 + (i * 17) % 34;
      int by = 18 + (i * 23) % 48;
      tft.fillCircle(bx, by, 2, rgb(246, 166, 190));
    }
  }
}

void pStatus(const char* title, const char* tag) {
  uint16_t bar = uiTheme == THEME_RETRO ? rgb(5, 34, 33) : themePanel();
  tft.fillRect(0, 0, 240, 30, bar);
  tft.drawFastHLine(0, 29, 240, themeEdge());
  pLabel(8, 10, title, WHITE, 1);

  int tw = (int)strlen(tag) * 6;
  pLabel(232 - tw, 10, tag, themeAccent(), 1);
}

void pSceneLabel(int x, int y, const char* s, uint16_t c, int size=1) {
  portraitScene.setTextWrap(false);
  portraitScene.setTextColor(c);
  portraitScene.setTextSize(size);
  portraitScene.setCursor(x, y);
  portraitScene.print(s);
}

void pSceneCloud(int x, int y, int s, uint16_t c) {
  portraitScene.fillCircle(x, y, s, c);
  portraitScene.fillCircle(x + s, y - s/2, s + 2, c);
  portraitScene.fillCircle(x + 2*s, y, s, c);
  portraitScene.fillRoundRect(x - s, y, 4*s, s + 4, s/2 + 1, c);
}

void pBuildHomeScene(float sec) {
  int mode = sceneMode();

  uint16_t top = themeBg();
  uint16_t bottom = blend(themeBg(), themeAccent(), 0.25f);

  if (uiTheme == THEME_NATURE) {
    top = (weatherOK && !outdoorIsDay) ? rgb(5,15,38) : rgb(17,63,111);
    bottom = (mode == 1 || mode == 3) ? rgb(82,101,120) : rgb(90,168,194);
  } else if (uiTheme == THEME_OBSIDIAN) {
    top = rgb(3,7,17); bottom = rgb(21,13,44);
  } else if (uiTheme == THEME_RETRO) {
    top = rgb(3,22,22); bottom = rgb(8,45,41);
  } else if (uiTheme == THEME_INDIE) {
    top = rgb(43,31,68); bottom = rgb(111,67,83);
  } else if (uiTheme == THEME_JAPAN) {
    top = rgb(58,46,79); bottom = rgb(226,129,141);
  }

  for (int y = 0; y < 150; ++y) {
    portraitScene.drawFastHLine(0, y, 240, blend(top, bottom, y / 149.0f));
  }

  // Theme atmosphere.
  if (uiTheme == THEME_RETRO) {
    for (int x = 0; x < 240; x += 20) portraitScene.drawFastVLine(x, 0, 150, rgb(9,55,48));
    for (int y = 0; y < 150; y += 15) portraitScene.drawFastHLine(0, y, 240, rgb(9,55,48));
  }
  else if (uiTheme == THEME_INDIE) {
    portraitScene.fillRoundRect(10, 36, 92, 75, 7, rgb(20,24,49));
    portraitScene.drawRoundRect(10, 36, 92, 75, 7, rgb(145,104,151));
    portraitScene.fillCircle(79, 54, 9, rgb(246,222,180));
    portraitScene.fillCircle(83, 51, 8, rgb(20,24,49));
    for (int i = 0; i < 6; ++i) {
      int xx = 16 + i * 14;
      int h = 10 + (i * 9) % 26;
      portraitScene.fillRect(xx, 106-h, 10, h, rgb(28,32,54));
    }
  }
  else if (uiTheme == THEME_JAPAN) {
    portraitScene.fillCircle(63, 50, 17, rgb(250,216,190));
    portraitScene.fillRect(20, 82, 5, 45, rgb(156,44,55));
    portraitScene.fillRect(61, 82, 5, 45, rgb(156,44,55));
    portraitScene.fillRect(13, 78, 60, 5, rgb(183,52,62));
    portraitScene.drawLine(233, 15, 204, 84, rgb(82,50,58));
    for (int i = 0; i < 12; ++i) {
      int bx = 198 + (i * 17) % 38;
      int by = 17 + (i * 23) % 69;
      portraitScene.fillCircle(bx, by, 2, rgb(247,169,193));
    }
  }
  else if (uiTheme == THEME_OBSIDIAN) {
    for (int r = 38; r > 8; r -= 5) {
      portraitScene.drawCircle(54, 63, r, blend(top, themeAccent(), (38-r)/65.0f));
    }
  }

  if (uiTheme == THEME_NATURE) {
    if ((mode == 0 || mode == 2) && (!weatherOK || outdoorIsDay)) {
      portraitScene.fillCircle(44, 46, 15, rgb(255,218,106));
    }
    uint16_t cc = (mode == 1 || mode == 3) ? rgb(105,119,139) : rgb(219,235,241);
    for (int i = 0; i < 3; ++i) {
      int cx = ((int)clouds[i]) % 300;
      if (cx < -40) cx += 300;
      pSceneCloud(cx, 34 + i*18, 6 + (i%2)*2, cc);
    }

    if (mode == 3) {
      for (int i = 0; i < 22; ++i) {
        int xx = (i * 37 + (int)(sec * (43 + i%3*7))) % 240;
        int yy = (i * 29 + (int)(sec * (70 + i%4*8))) % 135;
        portraitScene.drawLine(xx, yy, xx-2, yy+6, CYAN);
      }
    } else if (mode == 1) {
      for (int i = 0; i < 22; ++i) {
        int xx = (i * 41 + (int)(sec * (7 + i%4))) % 240;
        int yy = (i * 31 + (int)(sec * (12 + i%5))) % 135;
        portraitScene.fillCircle(xx, yy, i%4==0 ? 2 : 1, WHITE);
      }
    }
  }

  // Header inside scene.
  portraitScene.fillRoundRect(7, 6, 150, 20, 6, blend(top, BLACK, 0.30f));
  pSceneLabel(13, 12, "ATMOSPHERE", WHITE, 1);

  // Character uses the same movement engine; only base position changes.
  int cx, cy; bool visible;
  getCharacterPose(cx, cy, visible);
  int px = 158 + (cx - 230);
  int py = 42 + (cy - 72);

  // In portrait keep the character visibly alive between larger events.
  // Action animations still come entirely from getCharacterPose().
  if (visible && characterEvent == CHAR_IDLE && animationsEnabled) {
    py -= (int)roundf(sinf(sec * 2.4f) * 2.0f);
    px += (int)roundf(sinf(sec * 1.15f) * 1.0f);
  }

  if (visible) drawSpriteAlpha(portraitScene, px, py, characterSpriteForMode(mode));

  // Lightweight event props in portrait.
  if (animationsEnabled && characterEvent == CHAR_BEES) {
    float t = (millis() - characterEventStartedAt) / 1000.0f;
    if (t < 3.65f) {
      int bx0 = visible ? px + 34 : 210;
      int by0 = visible ? py + 30 : 75;
      for (int i = 0; i < 3; ++i) {
        float a = t * (4.5f + i*0.35f) + i*2.1f;
        int bx = bx0 + (int)(cosf(a) * (15+i*4));
        int by = by0 + (int)(sinf(a*1.25f) * (10+i*3));
        portraitScene.fillCircle(bx, by, 2, rgb(255,205,46));
        portraitScene.drawFastVLine(bx, by-2, 5, rgb(38,33,27));
      }
    }
  }

  // Speech bubble remains available in portrait.
  if (speechVisible) {
    int bx = 78, by = 31, bw = 91, bh = 43;
    portraitScene.fillRoundRect(bx, by, bw, bh, 8, rgb(248,249,244));
    portraitScene.drawRoundRect(bx, by, bw, bh, 8, rgb(38,48,58));
    int w1 = strlen(speechLine1) * 6;
    int w2 = strlen(speechLine2) * 6;
    pSceneLabel(bx + (bw-w1)/2, by+10, speechLine1, rgb(24,30,36), 1);
    pSceneLabel(bx + (bw-w2)/2, by+25, speechLine2, rgb(24,30,36), 1);
  }
}

void pTransferSceneRect(int x, int y, int w, int h) {
  if (w<=0 || h<=0) return;
  if (x<0) { w+=x; x=0; }
  if (y<0) { h+=y; y=0; }
  if (x>=240 || y>=150) return;
  if (x+w>240) w=240-x;
  if (y+h>150) h=150-y;

  uint16_t *buf = portraitScene.getBuffer();

  tft.startWrite();
  tft.setAddrWindow(x, y+30, w, h);
  for (int row=0; row<h; ++row) {
    tft.writePixels(buf + (y+row)*240 + x, w, true);
  }
  tft.endWrite();
}

void pTransferSceneFull() {
  tft.drawRGBBitmap(0, 30, portraitScene.getBuffer(), 240, 150);
}

void pTransferSceneAnimated() {
  int mode = sceneMode();

  if (uiTheme == THEME_NATURE) {
    // Moving weather/clouds mostly live in the upper part.
    pTransferSceneRect(0, 0, 240, (mode==1 || mode==3) ? 112 : 88);
  }

  // Character/action/speech region only.
  pTransferSceneRect(70, 24, 170, 126);
}


void pUpdateHomeDataCards() {
  char b[42];

  // Repaint ONLY the two data cards, never the whole bottom/menu region.
  tft.fillRect(6, 184, 228, 62, themeBg());

  pPanel(8, 186, 108, 58);
  pLabel(16, 194, "UNUTRA", themeMuted(), 1);
  if (sensorOK) snprintf(b, sizeof(b), "%.1f %s", showTemp(temp), tempUnit());
  else snprintf(b, sizeof(b), "--.- %s", tempUnit());
  pLabel(16, 211, b, themeAccent2(), 2);

  pPanel(124, 186, 108, 58);
  pLabel(132, 194, "VLAGA", themeMuted(), 1);
  if (sensorOK) snprintf(b, sizeof(b), "%.0f %%", rh);
  else snprintf(b, sizeof(b), "-- %%");
  pLabel(132, 211, b, themeAccent(), 2);
}

void pUpdateHomeClock() {
  // Only this tiny strip changes once per second; the MENI button is untouched.
  tft.fillRect(0, 301, 240, 19, themeBg());

  int hr = secondsToday / 3600;
  int mn = (secondsToday / 60) % 60;
  char timeBuf[8], dateBuf[16];
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", hr, mn);
  snprintf(dateBuf, sizeof(dateBuf), "%02d.%02d.%04d", dayNow, monthNow, yearNow);

  pLabel(8, 307, dateBuf, themeMuted(), 1);
  pLabel(198, 307, timeBuf, WHITE, 1);
}

void pHomeStaticBottom() {
  // Full bottom is drawn only when the page truly opens/rebuilds.
  tft.fillRect(0, 180, 240, 140, themeBg());
  pUpdateHomeDataCards();
  pButton(38, 253, 164, 42, "MENI", themeAccent(), 2);
  pUpdateHomeClock();
}

void portraitHomePage(float sec, bool full) {
  if (full) {
    pBackdrop();
    pStatus("ATMOSPHERE", themeName());
    pHomeStaticBottom();
  }

  pBuildHomeScene(sec);

  if (full) pTransferSceneFull();
  else pTransferSceneAnimated();
}


void pMenuTile(int x, int y, int w, int h,
               const char* title, const char* sub,
               uint16_t accent, int iconType) {
  // Base card.
  if (uiTheme == THEME_RETRO) {
    tft.fillRect(x,y,w,h,rgb(5,42,39));
    tft.drawRect(x,y,w,h,accent);
  } else if (uiTheme == THEME_OBSIDIAN) {
    tft.fillRoundRect(x,y,w,h,10,rgb(11,18,34));
    tft.drawRoundRect(x,y,w,h,10,rgb(43,57,92));
  } else {
    tft.fillRoundRect(x,y,w,h,9,blend(accent,themeBg(),0.52f));
    tft.drawRoundRect(x,y,w,h,9,blend(accent,WHITE,0.18f));
  }

  int r=x+w-4, b=y+h-4;

  if (uiTheme == THEME_OBSIDIAN) {
    for(int gx=x+12; gx<r; gx+=20) tft.drawFastVLine(gx,y+4,h-8,rgb(18,29,55));
    for(int gy=y+9; gy<b; gy+=13) tft.drawFastHLine(x+5,gy,w-10,rgb(18,29,55));
    tft.drawCircle(r-20,y+h/2,12,blend(accent,rgb(11,18,34),0.45f));
  }
  else if (uiTheme == THEME_RETRO) {
    for(int gy=y+5; gy<b; gy+=6) tft.drawFastHLine(x+4,gy,w-8,rgb(7,58,50));
    for(int i=0;i<5;++i) {
      int ph=4+(i*4+iconType*3)%max(5,h-12);
      tft.fillRect(r-42+i*7,b-ph,4,ph,blend(accent,rgb(5,42,39),0.28f));
    }
  }
  else if (uiTheme == THEME_INDIE) {
    tft.fillRoundRect(r-52,y+5,44,h-10,5,rgb(34,30,57));
    tft.drawRoundRect(r-52,y+5,44,h-10,5,rgb(113,83,128));
    tft.fillCircle(r-17,y+13,5,rgb(245,205,157));
    tft.fillCircle(r-15,y+11,5,rgb(34,30,57));
    tft.drawFastHLine(x+10,b-7,min(42,w-20),rgb(116,75,78));
  }
  else if (uiTheme == THEME_JAPAN) {
    tft.fillCircle(r-22,y+13,7,rgb(247,199,184));
    tft.drawLine(r-5,y+3,r-29,b-4,rgb(91,54,62));
    for(int i=0;i<5;++i) {
      int bx=r-10-(i*11)%max(12,w/3);
      int by=y+7+(i*13)%max(10,h-12);
      tft.fillCircle(bx,by,1+(i==0),rgb(247,165,190));
    }
    tft.fillRect(r-54,b-18,3,13,rgb(154,47,57));
    tft.fillRect(r-38,b-18,3,13,rgb(154,47,57));
    tft.fillRect(r-58,b-21,27,3,rgb(181,54,63));
  }
  else { // Nature
    // Small natural/weather landscape so portrait Nature matches landscape richness.
    uint16_t sky=blend(accent,rgb(60,145,184),0.42f);
    tft.fillRoundRect(x+3,y+3,w-6,h-6,7,sky);
    tft.fillRect(x+3,b-10,w-6,9,rgb(56,132,70));
    tft.fillCircle(r-24,y+13,6,rgb(255,218,85));
    uint16_t cc=rgb(222,235,241);
    tft.fillCircle(r-51,y+17,5,cc);
    tft.fillCircle(r-44,y+14,7,cc);
    tft.fillRoundRect(r-56,y+18,20,5,2,cc);
  }

  // Function icon.
  int ix=r-18, iy=y+h/2;
  if(iconType==0) {
    tft.drawRoundRect(ix-2,iy-8,5,13,2,WHITE);
    tft.fillCircle(ix,iy+7,4,accent);
  } else if(iconType==1) {
    tft.fillCircle(ix-7,iy-4,5,rgb(255,212,70));
    tft.fillCircle(ix+3,iy+2,6,rgb(218,230,238));
  } else if(iconType==2) {
    for(int i=0;i<3;++i) tft.fillRoundRect(ix-15+i*9,iy-6,7,12,2,blend(accent,WHITE,0.15f));
  } else if(iconType==3) {
    tft.drawCircle(ix,iy,8,WHITE);
    tft.drawLine(ix,iy,ix,iy-5,accent);
    tft.drawLine(ix,iy,ix+4,iy+2,accent);
  } else if(iconType==4) {
    tft.drawLine(ix-8,iy-4,ix,iy-10,WHITE);
    tft.drawLine(ix,iy-10,ix+8,iy-4,WHITE);
    tft.drawLine(ix-5,iy,ix,iy-4,WHITE);
    tft.drawLine(ix,iy-4,ix+5,iy,WHITE);
    tft.fillCircle(ix,iy+4,1,accent);
  } else {
    tft.drawCircle(ix,iy,7,WHITE);
    tft.drawCircle(ix,iy,2,accent);
  }

  uint16_t plate=(uiTheme==THEME_RETRO)?rgb(5,42,39):
                 (uiTheme==THEME_OBSIDIAN?rgb(11,18,34):blend(themeBg(),BLACK,0.12f));
  int pw=min(w-14,max(68,(int)strlen(title)*6+16));
  tft.fillRoundRect(x+7,y+7,pw,17,5,plate);
  pLabel(x+11,y+11,title,WHITE,1);

  if(sub && sub[0]) {
    int sw=min(w-14,max(68,(int)strlen(sub)*6+12));
    tft.fillRoundRect(x+7,y+h-18,sw,14,4,plate);
    pLabel(x+11,y+h-15,sub,themeMuted(),1);
  }
}

void portraitMenuPage() {
  pBackdrop();
  pStatus("MENI", themeName());

  pMenuTile(12, 40, 216, 38, "KLIMA", "unutrasnji senzor", rgb(58,153,221), 0);
  pMenuTile(12, 84, 216, 38, "VREME", "stanje napolju", rgb(237,170,112), 1);
  pMenuTile(12, 128, 216, 38, "3 DANA", "prognoza", rgb(75,174,170), 2);
  pMenuTile(12, 172, 216, 38, "SAT", "lokalno vreme", rgb(155,94,232), 3);
  pMenuTile(12, 216, 216, 38, "WIFI", "mreza / status", rgb(61,151,197), 4);
  pMenuTile(12, 260, 216, 38, "PODESAVANJA", "sistem / teme", rgb(110,116,137), 5);

  pButton(70, 302, 100, 16, "HOME", themePanel(), 1);
}


void pSimpleWeatherIcon(int cx, int cy, int code) {
  if (code == 0 || code == 1) {
    tft.fillCircle(cx, cy, 15, rgb(255,215,76));
    return;
  }
  uint16_t cc = weatherIsRainCode(code) ? rgb(120,132,150) : rgb(205,220,230);
  tft.fillCircle(cx-12, cy, 10, cc);
  tft.fillCircle(cx, cy-7, 14, cc);
  tft.fillCircle(cx+15, cy, 10, cc);
  tft.fillRoundRect(cx-22, cy, 47, 12, 6, cc);
  if (weatherIsRainCode(code)) {
    for (int i=-1;i<=1;++i) tft.drawLine(cx+i*12,cy+17,cx+i*12-3,cy+25,CYAN);
  }
}

void portraitClimatePage(float sec, bool full) {
  if (full) {
    pBackdrop();
    pStatus("KLIMA", "UNUTRA");

    char b[32];
    pPanel(12, 140, 216, 66);
    pLabel(22, 150, "TEMPERATURA", themeMuted(), 1);
    if (sensorOK) snprintf(b,sizeof(b),"%.1f %s",showTemp(temp),tempUnit());
    else snprintf(b,sizeof(b),"--.- %s",tempUnit());
    pLabel(22, 170, b, themeAccent2(), 3);

    pPanel(12, 214, 216, 58);
    pLabel(22, 224, "VLAGA", themeMuted(), 1);
    if (sensorOK) snprintf(b,sizeof(b),"%.0f %%",rh); else snprintf(b,sizeof(b),"-- %%");
    pLabel(22, 242, b, themeAccent(), 2);

    pButton(58, 281, 124, 32, "NAZAD", themePanel(), 1);
  }

  // Reuse the portrait animated scene buffer as a compact climate header.
  portraitScene.fillScreen(themeBg());
  uint16_t a = themeAccent();
  if (uiTheme == THEME_NATURE) {
    uint16_t top = rgb(20,55,91), bot = rgb(81,146,170);
    for (int y=0;y<100;++y) portraitScene.drawFastHLine(0,y,240,blend(top,bot,y/99.0f));
    for (int i=0;i<3;++i) pSceneCloud(((int)clouds[i])%260,35+i*18,6, rgb(211,226,234));
  } else if (uiTheme == THEME_OBSIDIAN) {
    for(int y=0;y<100;++y) portraitScene.drawFastHLine(0,y,240,blend(rgb(3,7,18),rgb(20,14,43),y/99.0f));
    for(int r=35;r>8;r-=5) portraitScene.drawCircle(188,52,r,blend(rgb(10,15,31),a,(35-r)/65.0f));
  } else if (uiTheme == THEME_RETRO) {
    portraitScene.fillRect(0,0,240,100,rgb(3,25,24));
    for(int x=0;x<240;x+=16) portraitScene.drawFastVLine(x,0,100,rgb(6,48,43));
    for(int y=0;y<100;y+=10) portraitScene.drawFastHLine(0,y,240,rgb(6,48,43));
  } else if (uiTheme == THEME_INDIE) {
    for(int y=0;y<100;++y) portraitScene.drawFastHLine(0,y,240,blend(rgb(43,31,68),rgb(109,65,82),y/99.0f));
    portraitScene.fillRoundRect(130,14,92,72,7,rgb(19,24,48));
    portraitScene.fillCircle(195,34,10,rgb(245,220,180));
  } else {
    for(int y=0;y<100;++y) portraitScene.drawFastHLine(0,y,240,blend(rgb(59,45,79),rgb(229,129,140),y/99.0f));
    portraitScene.fillCircle(185,40,16,rgb(250,216,190));
    portraitScene.fillRect(24,55,5,34,rgb(151,45,55));
    portraitScene.fillRect(58,55,5,34,rgb(151,45,55));
    portraitScene.fillRect(18,51,52,5,rgb(183,52,62));
  }
  pSceneLabel(12, 12, "LOKALNA KLIMA", WHITE, 1);
  tft.drawRGBBitmap(0, 30, portraitScene.getBuffer(), 240, 100);
}

void portraitWeatherPage() {
  pBackdrop();
  pStatus("VREME", "NAPOLJU");

  char b[48];
  pPanel(12, 44, 216, 116);
  pLabel(22, 55, activeCity().name, themeMuted(), 1);
  if (weatherOK) {
    snprintf(b,sizeof(b),"%.0f %s",showTemp(outdoorTemp),tempUnit());
    pLabel(22,80,b,WHITE,4);
    pLabel(22,121,weatherLabel(outdoorWeatherCode),themeAccent(),1);
    pSimpleWeatherIcon(187, 94, outdoorWeatherCode);
  } else {
    pCenter(89, "OFFLINE", ORANGE, 2);
  }

  pPanel(12, 171, 216, 73);
  if (weatherOK) {
    snprintf(b,sizeof(b),"VLAGA %.0f%%",outdoorHumidity); pLabel(22,184,b,CYAN,1);
    snprintf(b,sizeof(b),"VETAR %.1f",outdoorWind); pLabel(22,203,b,WHITE,1);
    snprintf(b,sizeof(b),"KISA %d%%",forecastRainChance[0]); pLabel(22,222,b,themeAccent2(),1);
  } else pCenter(201,"CEKAM PODATKE",themeMuted(),1);

  pButton(58, 270, 124, 34, "NAZAD", themePanel(), 1);
}

void portraitForecastPage() {
  pBackdrop();
  pStatus("3 DANA", "PROGNOZA");

  for (int i=0;i<3;++i) {
    int y = 43 + i*76;
    pPanel(10,y,220,66);
    char b[50];
    if (weatherOK && isfinite(forecastMax[i])) {
      snprintf(b,sizeof(b),"DAN %d   %.0f/%.0f %s",i+1,showTemp(forecastMax[i]),showTemp(forecastMin[i]),tempUnit());
      pLabel(20,y+12,b,WHITE,1);
      pLabel(20,y+32,weatherLabel(forecastCode[i]),themeAccent(),1);
      snprintf(b,sizeof(b),"KISA %d%%",forecastRainChance[i]);
      pLabel(150,y+32,b,themeAccent2(),1);
    } else {
      snprintf(b,sizeof(b),"DAN %d  --",i+1);
      pLabel(20,y+24,b,themeMuted(),1);
    }
  }
  pButton(58, 284, 124, 30, "NAZAD", themePanel(), 1);
}


void pUpdateClockTimeOnly() {
  int hr=secondsToday/3600, mn=(secondsToday/60)%60, sc=secondsToday%60;
  char b[32];

  tft.fillRect(12,72,216,96,themeBg());

  snprintf(b,sizeof(b),"%02d:%02d",hr,mn);
  pCenter(86,b,WHITE,5);
  snprintf(b,sizeof(b),":%02d",sc);
  pCenter(135,b,themeAccent(),2);
}

void portraitClockPage() {
  pBackdrop();
  pStatus("SAT", "LOKALNO");

  int hr=secondsToday/3600, mn=(secondsToday/60)%60, sc=secondsToday%60;
  char b[32];
  snprintf(b,sizeof(b),"%02d:%02d",hr,mn);
  pCenter(86,b,WHITE,5);
  snprintf(b,sizeof(b),":%02d",sc);
  pCenter(135,b,themeAccent(),2);
  snprintf(b,sizeof(b),"%02d.%02d.%04d",dayNow,monthNow,yearNow);
  pCenter(180,b,themeMuted(),2);

  pPanel(28,215,184,45);
  pCenter(228,timeSynced?"NTP SINHRONIZOVAN":"LOKALNI SAT",timeSynced?GREEN:ORANGE,1);
  pButton(58,280,124,32,"NAZAD",themePanel(),1);
}

void portraitWifiPage() {
  pBackdrop();
  pStatus("WIFI","SISTEM");
  bool connected = WiFi.status()==WL_CONNECTED;

  pPanel(12,45,216,180);
  pLabel(22,58,"STATUS",themeMuted(),1);
  pLabel(105,58,connected?"POVEZAN":"NIJE POVEZAN",connected?GREEN:RED,1);

  pLabel(22,90,"MREZA",themeMuted(),1);
  if (connected) {
    String s=WiFi.SSID(); char bb[20]; snprintf(bb,sizeof(bb),"%.18s",s.c_str()); pLabel(22,108,bb,WHITE,1);
  } else pLabel(22,108,"-",themeMuted(),1);

  pLabel(22,140,"IP",themeMuted(),1);
  if (connected) { String ip=WiFi.localIP().toString(); pLabel(22,158,ip.c_str(),WHITE,1); }

  pLabel(22,190,"SIGNAL",themeMuted(),1);
  if (connected) {
    char b[36]; int r=WiFi.RSSI(); snprintf(b,sizeof(b),"%d dBm / %s",r,wifiQualityLabel(r)); pLabel(22,208,b,WHITE,1);
  }

  pButton(58,270,124,34,"NAZAD",themePanel(),1);
}

void portraitHistoryPage() {
  pBackdrop();
  pStatus("ISTORIJA","UNUTRA");

  pPanel(12,45,216,92);
  pLabel(22,56,"TEMPERATURA MIN / MAX",themeMuted(),1);
  char b[40];
  if (haveReading) snprintf(b,sizeof(b),"%.1f / %.1f %s",showTemp(lowTemp),showTemp(highTemp),tempUnit());
  else snprintf(b,sizeof(b),"-- / --");
  pCenter(89,b,themeAccent2(),2);

  pPanel(12,147,216,92);
  pLabel(22,158,"VLAGA MIN / MAX",themeMuted(),1);
  if (haveReading) snprintf(b,sizeof(b),"%.0f / %.0f %%",lowHum,highHum); else snprintf(b,sizeof(b),"-- / --");
  pCenter(191,b,themeAccent(),2);

  pButton(58,270,124,34,"NAZAD",themePanel(),1);
}

void portraitSettingsPage() {
  pBackdrop();
  pStatus("PODESAVANJA","1/2");

  pLabel(12,40,"TEMPERATURA",themeMuted(),1);
  pButton(12,53,103,34,"CELZIJUS",!useFahrenheit?GREEN:themePanel(),1);
  pButton(125,53,103,34,"FARENHAJT",useFahrenheit?GREEN:themePanel(),1);

  pLabel(12,99,"OSVEZAVANJE VREMENA",themeMuted(),1);
  pButton(12,112,64,32,"5 MIN",refreshMinutes==5?BLUE:themePanel(),1);
  pButton(88,112,64,32,"15 MIN",refreshMinutes==15?BLUE:themePanel(),1);
  pButton(164,112,64,32,"30 MIN",refreshMinutes==30?BLUE:themePanel(),1);

  pLabel(12,157,"GRAD",themeMuted(),1);
  pButton(12,170,216,31,activeCity().name,CYAN,1);

  pButton(12,221,68,37,"NAZAD",themePanel(),1);
  pButton(86,221,68,37,"ISTORIJA",rgb(39,102,126),1);
  pButton(160,221,68,37,"VISE >",rgb(54,92,151),1);

  pLabel(12,277,"GRAD MENJAS DODIROM DUGMETA",themeMuted(),1);
}

void portraitSettingsAdvancedPage() {
  pBackdrop();
  pStatus("PODESAVANJA","2/2");

  pLabel(10,38,"TEMA",themeMuted(),1);
  pButton(8,50,72,30,"NATURE",uiTheme==THEME_NATURE?themeAccent():themePanel(),1);
  pButton(84,50,72,30,"OBSID.",uiTheme==THEME_OBSIDIAN?themeAccent():themePanel(),1);
  pButton(160,50,72,30,"RETRO",uiTheme==THEME_RETRO?themeAccent():themePanel(),1);
  pButton(45,86,72,30,"INDIE",uiTheme==THEME_INDIE?themeAccent():themePanel(),1);
  pButton(123,86,72,30,"JAPAN",uiTheme==THEME_JAPAN?themeAccent():themePanel(),1);

  pLabel(10,126,"PONASANJE",themeMuted(),1);
  pButton(8,138,72,30,animationsEnabled?"ANIM ON":"ANIM OFF",animationsEnabled?GREEN:themePanel(),1);
  pButton(84,138,72,30,quietMode?"QUIET ON":"QUIET OFF",quietMode?themeAccent():themePanel(),1);
  pButton(160,138,72,30,alertsEnabled?"ALERT ON":"ALERT OFF",alertsEnabled?ORANGE:themePanel(),1);

  pLabel(10,178,"EKRAN / LIK",themeMuted(),1);
  pButton(8,190,72,34,"LANDSCAPE",themeAccent(),1);
  pButton(84,190,72,34,ambientEnabled?"AMBIENT":"AMB OFF",ambientEnabled?BLUE:themePanel(),1);
  pButton(160,190,72,34,femaleCharacter?"ZENSKI":"MUSKI",themePanel(),1);

  pButton(8,236,108,34,homeStyle==0?"HOME SCENA":"HOME DASH",homeStyle==0?GREEN:themePanel(),1);
  pButton(124,236,108,34,portraitTouchCalibrated?"TOUCH OK":"TOUCH CAL",themePanel(),1);

  pButton(8,286,92,27,"< PRETH",rgb(54,92,151),1);
  pButton(140,286,92,27,"MENI",themePanel(),1);
}

void portraitAmbientPage() {
  pBackdrop();
  int hr=secondsToday/3600, mn=(secondsToday/60)%60;
  char b[36];
  snprintf(b,sizeof(b),"%02d:%02d",hr,mn);
  pCenter(82,b,WHITE,5);
  snprintf(b,sizeof(b),"%02d.%02d.%04d",dayNow,monthNow,yearNow);
  pCenter(133,b,themeMuted(),1);
  if (weatherOK) {
    snprintf(b,sizeof(b),"NAPOLJU %.0f %s",showTemp(outdoorTemp),tempUnit());
    pCenter(179,b,themeAccent(),1);
  }
  if (sensorOK) {
    snprintf(b,sizeof(b),"UNUTRA %.1f %s / %.0f%%",showTemp(temp),tempUnit(),rh);
    pCenter(206,b,WHITE,1);
  }
  pCenter(290,"DODIRNI ZA POVRATAK",themeMuted(),1);
}


void drawPortraitCalibrationTarget(int x, int y, int step) {
  tft.fillScreen(rgb(4, 10, 18));
  pCenter(18, "TOUCH KALIBRACIJA", CYAN, 2);

  char b[32];
  snprintf(b, sizeof(b), "TACKA %d / 5", step + 1);
  pCenter(49, b, MUTED, 1);
  pCenter(66, "DODIRNI CENTAR KRSTICA", WHITE, 1);

  tft.drawCircle(x, y, 13, rgb(79,166,222));
  tft.drawCircle(x, y, 6, WHITE);
  tft.drawFastHLine(x - 20, y, 41, WHITE);
  tft.drawFastVLine(x, y - 20, 41, WHITE);
  tft.fillCircle(x, y, 2, ORANGE);

  pCenter(296, "DRZI KRATKO I PUSTI", MUTED, 1);
}

bool capturePortraitCalibrationRaw(int targetX, int targetY,
                                   int &rawX, int &rawY, int step) {
  drawPortraitCalibrationTarget(targetX, targetY, step);

  while (touch.getPoint().z >= 180) delay(8);

  uint32_t started = millis();
  while ((uint32_t)(millis() - started) < 12000UL) {
    if (readStableTouchRaw(rawX, rawY)) {
      tft.fillCircle(targetX, targetY, 7, GREEN);
      waitTouchRelease();
      delay(160);
      return true;
    }
    delay(10);
  }
  return false;
}

void runPortraitTouchCalibration() {
  // 3x3 portrait grid: affine fit + local residual correction.
  const int sx[9] = {25,120,215, 25,120,215, 25,120,215};
  const int sy[9] = {35,35,35, 160,160,160, 285,285,285};
  int rx[9], ry[9];

  float oldA=pTouchA, oldB=pTouchB, oldC=pTouchC;
  float oldD=pTouchD, oldE=pTouchE, oldF=pTouchF;
  int16_t oldCX[9], oldCY[9];
  memcpy(oldCX, pCorrX, sizeof(oldCX));
  memcpy(oldCY, pCorrY, sizeof(oldCY));
  bool oldCal=portraitTouchCalibrated;
  bool oldLocal=portraitLocalCalibrated;

  tft.setRotation(0);
  tft.fillScreen(BLACK);

  for (int i=0; i<9; ++i) {
    if (!capturePortraitCalibrationRaw(sx[i], sy[i], rx[i], ry[i], i)) {
      pTouchA=oldA; pTouchB=oldB; pTouchC=oldC;
      pTouchD=oldD; pTouchE=oldE; pTouchF=oldF;
      memcpy(pCorrX, oldCX, sizeof(oldCX));
      memcpy(pCorrY, oldCY, sizeof(oldCY));
      portraitTouchCalibrated=oldCal;
      portraitLocalCalibrated=oldLocal;

      tft.fillScreen(rgb(4,10,18));
      pCenter(118, "KALIBRACIJA PREKINUTA", RED, 1);
      pCenter(145, "STARE VREDNOSTI OSTAJU", WHITE, 1);
      delay(1300);
      pageDirty=true;
      return;
    }
  }

  float a,b,c,d,e,f;
  bool ok = fitAffineN(rx, ry, sx, sy, 9, a,b,c,d,e,f);

  float maxAffineErr = 999.0f;
  if (ok) {
    maxAffineErr = 0.0f;
    for (int i=0;i<9;++i) {
      float px=a*rx[i]+b*ry[i]+c;
      float py=d*rx[i]+e*ry[i]+f;
      float ex=sx[i]-px;
      float ey=sy[i]-py;

      if (ex > 60) ex = 60;
      if (ex < -60) ex = -60;
      if (ey > 60) ey = 60;
      if (ey < -60) ey = -60;

      pCorrX[i] = (int16_t)roundf(ex);
      pCorrY[i] = (int16_t)roundf(ey);

      float err=sqrtf(ex*ex+ey*ey);
      if(err>maxAffineErr) maxAffineErr=err;
    }

    if (!isfinite(a)||!isfinite(b)||!isfinite(c)||
        !isfinite(d)||!isfinite(e)||!isfinite(f)||
        maxAffineErr>65.0f) ok=false;
  }

  tft.fillScreen(rgb(4,10,18));

  if (ok) {
    pTouchA=a; pTouchB=b; pTouchC=c;
    pTouchD=d; pTouchE=e; pTouchF=f;
    portraitTouchCalibrated=true;
    portraitLocalCalibrated=true;
    portraitCalV2=true;
    saveSettings();

    pCenter(92, "PORTRAIT TOUCH SACUVAN", GREEN, 1);
    pCenter(120, "9 TACKI + LOKALNA KOREKCIJA", CYAN, 1);
    char b1[40];
    snprintf(b1,sizeof(b1),"AFFINE RESIDUAL %.1f px",maxAffineErr);
    pCenter(146,b1,WHITE,1);
    pCenter(174,"OSTAJE POSLE GASENJA",CYAN,1);
  } else {
    pTouchA=oldA; pTouchB=oldB; pTouchC=oldC;
    pTouchD=oldD; pTouchE=oldE; pTouchF=oldF;
    memcpy(pCorrX, oldCX, sizeof(oldCX));
    memcpy(pCorrY, oldCY, sizeof(oldCY));
    portraitTouchCalibrated=oldCal;
    portraitLocalCalibrated=oldLocal;

    pCenter(108,"KALIBRACIJA NIJE USPELA",RED,1);
    pCenter(137,"PONOVI PRECIZNIJE",WHITE,1);
  }

  pCenter(220,"DODIRNI ZA POVRATAK",MUTED,1);

  waitTouchRelease();
  uint32_t ws=millis();
  while ((uint32_t)(millis()-ws)<10000UL) {
    TS_Point p=touch.getPoint();
    if (p.z>250) { waitTouchRelease(); break; }
    delay(10);
  }

  tft.fillScreen(BLACK);
  pageDirty=true;
  dataDirty=true;
  lastInteractionAt=millis();
}


void applyScreenOrientation() {
  tft.setRotation(portraitMode ? 0 : 1);
  tft.fillScreen(BLACK);
pageDirty = true;
  dataDirty = true;
}

void runCalibrationKeepingOrientation() {
  if (portraitMode) {
    runPortraitTouchCalibration();
  } else {
    runTouchCalibration();
  }

  pageDirty = true;
  dataDirty = true;
}






// ===== Portrait dirty redraw helpers =====
void pRedrawSettingsUnitButtons() {
  pButton(12,53,103,34,"CELZIJUS",!useFahrenheit?GREEN:themePanel(),1);
  pButton(125,53,103,34,"FARENHAJT",useFahrenheit?GREEN:themePanel(),1);
}

void pRedrawSettingsRefreshButtons() {
  pButton(12,112,64,32,"5 MIN",refreshMinutes==5?BLUE:themePanel(),1);
  pButton(88,112,64,32,"15 MIN",refreshMinutes==15?BLUE:themePanel(),1);
  pButton(164,112,64,32,"30 MIN",refreshMinutes==30?BLUE:themePanel(),1);
}

void pRedrawSettingsCityButton() {
  pButton(12,170,216,31,activeCity().name,CYAN,1);
}

void pRedrawAdvancedBehaviorButtons() {
  pButton(8,138,72,30,animationsEnabled?"ANIM ON":"ANIM OFF",animationsEnabled?GREEN:themePanel(),1);
  pButton(84,138,72,30,quietMode?"QUIET ON":"QUIET OFF",quietMode?themeAccent():themePanel(),1);
  pButton(160,138,72,30,alertsEnabled?"ALERT ON":"ALERT OFF",alertsEnabled?ORANGE:themePanel(),1);
}

void pRedrawAdvancedScreenButtons() {
  pButton(8,190,72,34,"LANDSCAPE",themeAccent(),1);
  pButton(84,190,72,34,ambientEnabled?"AMBIENT":"AMB OFF",ambientEnabled?BLUE:themePanel(),1);
  pButton(160,190,72,34,femaleCharacter?"ZENSKI":"MUSKI",themePanel(),1);
}

void pRedrawAdvancedHomeTouchButtons() {
  pButton(8,236,108,34,homeStyle==0?"HOME SCENA":"HOME DASH",homeStyle==0?GREEN:themePanel(),1);
  pButton(124,236,108,34,portraitTouchCalibrated?"TOUCH OK":"TOUCH CAL",themePanel(),1);
}

void pUpdateAmbientDynamicOnly() {
  // No backdrop/full-page redraw. Only the dynamic lines are erased/repainted.
  int hr=secondsToday/3600, mn=(secondsToday/60)%60;
  char b[40];

  tft.fillRect(22,72,196,78,themeBg());
  snprintf(b,sizeof(b),"%02d:%02d",hr,mn);
  pCenter(82,b,WHITE,5);
  snprintf(b,sizeof(b),"%02d.%02d.%04d",dayNow,monthNow,yearNow);
  pCenter(133,b,themeMuted(),1);

  tft.fillRect(10,170,220,62,themeBg());
  if (weatherOK) {
    snprintf(b,sizeof(b),"NAPOLJU %.0f %s",showTemp(outdoorTemp),tempUnit());
    pCenter(179,b,themeAccent(),1);
  }
  if (sensorOK) {
    snprintf(b,sizeof(b),"UNUTRA %.1f %s / %.0f%%",showTemp(temp),tempUnit(),rh);
    pCenter(206,b,WHITE,1);
  }
}

void drawPortraitFrame(float sec, bool secondChanged) {
  static Page lastPortraitPage = (Page)-1;
  static uint8_t lastPortraitTheme = 255;
  static uint8_t lastPortraitHomeStyle = 255;

  bool pageChanged = currentPage != lastPortraitPage;
  bool themeChanged = uiTheme != lastPortraitTheme;
  bool homeStyleChanged = homeStyle != lastPortraitHomeStyle;

  if (currentPage == PAGE_HOME) {
    if (homeStyle == 0) {
      bool hardFull = pageChanged || themeChanged || homeStyleChanged;

      if (hardFull) {
        portraitHomePage(animationsEnabled ? sec : 0.0f, true);
      } else {
        pBuildHomeScene(animationsEnabled ? sec : 0.0f);

        if (dataDirty) {
          pTransferSceneFull();
          pUpdateHomeDataCards();
        } else if (animationsEnabled) {
          pTransferSceneAnimated();
        }

        if (secondChanged) pUpdateHomeClock();
      }
    } else {
      bool hardFull = pageChanged || themeChanged || homeStyleChanged;
      if (hardFull) {
        pBackdrop();
        pStatus("ATMOSPHERE","DASH");
      }

      if (hardFull || dataDirty || secondChanged) {
        char b[40];

        pPanel(16,60,208,80);
        pCenter(73,"TEMPERATURA",themeMuted(),1);
        if(sensorOK) snprintf(b,sizeof(b),"%.1f %s",showTemp(temp),tempUnit());
        else snprintf(b,sizeof(b),"--.-");
        pCenter(96,b,themeAccent2(),3);

        pPanel(16,151,208,80);
        pCenter(164,"VLAGA",themeMuted(),1);
        if(sensorOK) snprintf(b,sizeof(b),"%.0f %%",rh);
        else snprintf(b,sizeof(b),"--");
        pCenter(187,b,themeAccent(),3);

        pButton(48,257,144,40,"MENI",themeAccent(),2);
      }
    }
  }
  else if (currentPage == PAGE_MENU) {
    if (pageChanged || themeChanged) portraitMenuPage();
  }
  else if (currentPage == PAGE_CLIMATE) {
    if (pageChanged || themeChanged || dataDirty ||
        (animationsEnabled && uiTheme == THEME_NATURE))
      portraitClimatePage(animationsEnabled ? sec : 0.0f,
                          pageChanged || themeChanged);
  }
  else if (currentPage == PAGE_WEATHER) {
    if (pageChanged || themeChanged || dataDirty) portraitWeatherPage();
  }
  else if (currentPage == PAGE_FORECAST) {
    if (pageChanged || themeChanged || dataDirty) portraitForecastPage();
  }
  else if (currentPage == PAGE_CLOCK) {
    if (pageChanged || themeChanged || dataDirty) {
      portraitClockPage();
    } else if (secondChanged) {
      pUpdateClockTimeOnly();
    }
  }
  else if (currentPage == PAGE_WIFI) {
    if (pageChanged || themeChanged || dataDirty) portraitWifiPage();
  }
  else if (currentPage == PAGE_HISTORY) {
    if (pageChanged || themeChanged || dataDirty) portraitHistoryPage();
  }
  else if (currentPage == PAGE_SETTINGS) {
    if (pageChanged || themeChanged) portraitSettingsPage();
  }
  else if (currentPage == PAGE_SETTINGS_ADVANCED) {
    if (pageChanged || themeChanged) portraitSettingsAdvancedPage();
  }
  else if (currentPage == PAGE_AMBIENT) {
    if (pageChanged || themeChanged) {
      portraitAmbientPage();
    } else if (secondChanged || dataDirty) {
      pUpdateAmbientDynamicOnly();
    }
  }

  lastPortraitPage = currentPage;
  lastPortraitTheme = uiTheme;
  lastPortraitHomeStyle = homeStyle;
}

void handlePortraitTouch(int x, int y) {
  auto hit = [&](int bx, int by, int bw, int bh, int pad=5) {
    return x >= bx-pad && x <= bx+bw+pad &&
           y >= by-pad && y <= by+bh+pad;
  };

  if (currentPage == PAGE_HOME) {
    if (hit(38,253,164,42,10)) {
      waitTouchRelease(); goTo(PAGE_MENU); return;
    }

    if (homeStyle==0 && hit(145,55,95,115,8)) {
      waitTouchRelease();
      if (!speechVisible && characterEvent==CHAR_IDLE) {
        int surprise=random(4);
        if(surprise==0) startCharacterMotion(CHAR_DANCE,3000);
        else if(surprise==1) startCharacterMotion(CHAR_BOUNCE,2200);
        else if(surprise==2) startCharacterMotion(CHAR_FLOWER_PICK,6500);
        else startCharacterMotion(CHAR_BEES,7000);
      }
      return;
    }
  }

  else if (currentPage == PAGE_MENU) {
    if (hit(12,40,216,38,3)) { waitTouchRelease(); goTo(PAGE_CLIMATE); return; }
    if (hit(12,84,216,38,3)) { waitTouchRelease(); goTo(PAGE_WEATHER); return; }
    if (hit(12,128,216,38,3)) { waitTouchRelease(); goTo(PAGE_FORECAST); return; }
    if (hit(12,172,216,38,3)) { waitTouchRelease(); goTo(PAGE_CLOCK); return; }
    if (hit(12,216,216,38,3)) { waitTouchRelease(); goTo(PAGE_WIFI); return; }
    if (hit(12,260,216,38,3)) { waitTouchRelease(); goTo(PAGE_SETTINGS); return; }
    if (hit(70,302,100,16,2)) { waitTouchRelease(); goTo(PAGE_HOME); return; }
  }

  else if (currentPage == PAGE_CLIMATE) {
    if (hit(58,281,124,32,8)) { waitTouchRelease(); goTo(PAGE_MENU); return; }
  }
  else if (currentPage == PAGE_WEATHER) {
    if (hit(58,270,124,34,8)) { waitTouchRelease(); goTo(PAGE_MENU); return; }
  }
  else if (currentPage == PAGE_FORECAST) {
    if (hit(58,284,124,30,8)) { waitTouchRelease(); goTo(PAGE_MENU); return; }
  }
  else if (currentPage == PAGE_CLOCK) {
    if (hit(58,280,124,32,8)) { waitTouchRelease(); goTo(PAGE_MENU); return; }
  }
  else if (currentPage == PAGE_WIFI) {
    if (hit(58,270,124,34,8)) { waitTouchRelease(); goTo(PAGE_MENU); return; }
  }
  else if (currentPage == PAGE_HISTORY) {
    if (hit(58,270,124,34,8)) { waitTouchRelease(); goTo(PAGE_SETTINGS); return; }
  }

  else if (currentPage == PAGE_SETTINGS) {
    if (hit(12,53,103,34,4)) {
      useFahrenheit=false;
      saveSettings();
      waitTouchRelease();
      pRedrawSettingsUnitButtons();
      return;
    }
    if (hit(125,53,103,34,4)) {
      useFahrenheit=true;
      saveSettings();
      waitTouchRelease();
      pRedrawSettingsUnitButtons();
      return;
    }

    if (hit(12,112,64,32,4)) {
      refreshMinutes=5;
      saveSettings(); weatherOK=false; lastWeatherAttempt=0;
      waitTouchRelease();
      pRedrawSettingsRefreshButtons();
      return;
    }
    if (hit(88,112,64,32,4)) {
      refreshMinutes=15;
      saveSettings(); weatherOK=false; lastWeatherAttempt=0;
      waitTouchRelease();
      pRedrawSettingsRefreshButtons();
      return;
    }
    if (hit(164,112,64,32,4)) {
      refreshMinutes=30;
      saveSettings(); weatherOK=false; lastWeatherAttempt=0;
      waitTouchRelease();
      pRedrawSettingsRefreshButtons();
      return;
    }
    if (hit(12,170,216,31,5)) {
      cityIndex=(cityIndex+1)%CITY_COUNT;
      saveSettings(); weatherOK=false; lastWeatherAttempt=0;
      waitTouchRelease();
      pRedrawSettingsCityButton();
      return;
    }

    if (hit(12,221,68,37,5)) { waitTouchRelease(); goTo(PAGE_MENU); return; }
    if (hit(86,221,68,37,5)) { waitTouchRelease(); goTo(PAGE_HISTORY); return; }
    if (hit(160,221,68,37,5)) { waitTouchRelease(); goTo(PAGE_SETTINGS_ADVANCED); return; }

    waitTouchRelease();
    return;
  }

  else if (currentPage == PAGE_SETTINGS_ADVANCED) {
    if (hit(8,50,72,30,4)) {
      uiTheme=THEME_NATURE; saveSettings(); waitTouchRelease();
      pageDirty=true; return; // theme change genuinely changes whole page
    }
    if (hit(84,50,72,30,4)) {
      uiTheme=THEME_OBSIDIAN; saveSettings(); waitTouchRelease();
      pageDirty=true; return;
    }
    if (hit(160,50,72,30,4)) {
      uiTheme=THEME_RETRO; saveSettings(); waitTouchRelease();
      pageDirty=true; return;
    }
    if (hit(45,86,72,30,4)) {
      uiTheme=THEME_INDIE; saveSettings(); waitTouchRelease();
      pageDirty=true; return;
    }
    if (hit(123,86,72,30,4)) {
      uiTheme=THEME_JAPAN; saveSettings(); waitTouchRelease();
      pageDirty=true; return;
    }

    if (hit(8,138,72,30,4)) {
      animationsEnabled=!animationsEnabled; saveSettings(); waitTouchRelease();
      pRedrawAdvancedBehaviorButtons(); return;
    }
    if (hit(84,138,72,30,4)) {
      quietMode=!quietMode; saveSettings(); waitTouchRelease();
      pRedrawAdvancedBehaviorButtons(); return;
    }
    if (hit(160,138,72,30,4)) {
      alertsEnabled=!alertsEnabled; saveSettings(); waitTouchRelease();
      pRedrawAdvancedBehaviorButtons(); return;
    }

    if (hit(8,190,72,34,4)) {
      waitTouchRelease();
      portraitMode=false;
      saveSettings();
      applyScreenOrientation();
      return;
    }
    if (hit(84,190,72,34,4)) {
      ambientEnabled=!ambientEnabled; saveSettings(); waitTouchRelease();
      pRedrawAdvancedScreenButtons(); return;
    }
    if (hit(160,190,72,34,4)) {
      femaleCharacter=!femaleCharacter; saveSettings(); waitTouchRelease();
      pRedrawAdvancedScreenButtons(); return;
    }

    if (hit(8,236,108,34,4)) {
      homeStyle=homeStyle==0?1:0; saveSettings(); waitTouchRelease();
      pRedrawAdvancedHomeTouchButtons(); return;
    }
    if (hit(124,236,108,34,4)) {
      waitTouchRelease();
      runPortraitTouchCalibration();
      return;
    }

    if (hit(8,286,92,27,4)) { waitTouchRelease(); goTo(PAGE_SETTINGS); return; }
    if (hit(140,286,92,27,4)) { waitTouchRelease(); goTo(PAGE_MENU); return; }

    waitTouchRelease();
    return;
  }

  else if (currentPage == PAGE_AMBIENT) {
    waitTouchRelease(); lastInteractionAt=millis(); goTo(PAGE_HOME); return;
  }

  waitTouchRelease();
}


void transferRows(int first, int count) {
  // Posalji trazeni deo frame-a u jednom SPI transferu.
  // Ovo uklanja vidljivo "slajdovanje" / crtanje odozgo nadole.
  if (count <= 0) return;

  if (first < 0) {
    count += first;
    first = 0;
  }

  if (first >= 240) return;
  if (first + count > 240) count = 240 - first;

  tft.drawRGBBitmap(
    0,
    first,
    frame.getBuffer() + first * 320,
    320,
    count
  );
}

void transferRect(int x, int y, int w, int h) {
  // Send only the rectangle that actually changed.
  // The canvas has a 320-pixel stride, so rows are streamed while one SPI
  // transaction stays open. This is much cheaper than repainting 320x180/240.
  if (w <= 0 || h <= 0) return;

  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x >= 320 || y >= 240) return;
  if (x + w > 320) w = 320 - x;
  if (y + h > 240) h = 240 - y;
  if (w <= 0 || h <= 0) return;

  uint16_t* buf = frame.getBuffer();

  tft.startWrite();
  tft.setAddrWindow(x, y, w, h);

  for (int row = 0; row < h; ++row) {
    tft.writePixels(buf + (y + row) * 320 + x, w, true);
  }

  tft.endWrite();
}

void transferHomeAnimatedRegions(bool secondChanged) {
  // homeScenePage() still draws the COMPLETE next frame into RAM.
  // Only selected dirty regions are transferred to the physical TFT.

  int mode = sceneMode();

  if (uiTheme == THEME_NATURE) {
    // Clear/sunny/hot: clouds/sun live mainly in the upper sky.
    // Rain/snow particles use almost the whole upper landscape.
    if (mode == 1 || mode == 3) {
      transferRect(0, 0, 320, 151);
    } else {
      transferRect(0, 0, 320, 92);
    }

    // Character + bees + flower + old/new character positions.
    transferRect(174, 48, 146, 132);
  }
  else {
    // Obsidian/Retro/Indie/Japan backdrops are static.
    // During an action refresh continuously; while idle refresh only when the
    // character's normal 2-frame sprite changes (every ~650 ms).
    static int lastIdleFrame = -1;
    int idleFrame = (int)((elapsedMs / 650ULL) & 1ULL);

    if (characterEvent != CHAR_IDLE) {
      transferRect(174, 44, 146, 136);
      lastIdleFrame = idleFrame;
    }
    else if (idleFrame != lastIdleFrame) {
      transferRect(174, 44, 146, 136);
      lastIdleFrame = idleFrame;
    }
  }

  // Clock/date footer only needs one update per second.
  if (secondChanged) {
    transferRect(0, 180, 320, 60);
  }
}

// =====================================================
// RENDER SCHEDULER
// =====================================================

void drawFrame() {
  static uint64_t previousSecond = UINT64_MAX;

  float sec = (elapsedMs % 3600000ULL) / 1000.0f;
  bool secondChanged = previousSecond != elapsedMs / 1000ULL;

  if (portraitMode) {
    drawPortraitFrame(sec, secondChanged);
    pageDirty = false;
    dataDirty = false;
    previousSecond = elapsedMs / 1000ULL;
    return;
  }

  if (currentPage == PAGE_HOME) {
    if (homeStyle == 0) {
      if (pageDirty || dataDirty) {
        // Page changes, sensor/weather changes and event start/end get one
        // complete clean frame so no stale pixels can remain.
        homeScenePage(animationsEnabled ? sec : 0.0f);
        transferRows(0, 240);
      }
      else if (animationsEnabled) {
        // Build the next complete frame off-screen, but transmit only moving areas.
        homeScenePage(sec);
        transferHomeAnimatedRegions(secondChanged);
      }
      else if (secondChanged) {
        homeScenePage(0.0f);
        transferRect(0, 180, 320, 60);
      }
    }
    else {
      if (pageDirty || dataDirty) {
        homeDashboardPage();
        transferRows(0, 240);
      }
    }
  }

  else if (currentPage == PAGE_MENU) {
    if (pageDirty) {
      menuPage();
      transferRows(0, 240);
    }
  }

  else if (currentPage == PAGE_CLIMATE) {
    if (pageDirty || dataDirty) {
      climatePage(animationsEnabled ? sec : 0.0f);
      transferRows(0, 240);
    }
    else if (animationsEnabled && uiTheme == THEME_NATURE) {
      // Only Nature KLIMA has continuously moving clouds/particles.
      drawClimateThemeScene(sec, sceneMode());
      drawStatusBar("KLIMA", "UNUTRA");
      transferRect(0, 0, 320, 86);
    }
  }

  else if (currentPage == PAGE_WEATHER) {
    if (pageDirty || dataDirty) {
      weatherPage();
      transferRows(0, 240);
    }
  }

  else if (currentPage == PAGE_FORECAST) {
    if (pageDirty || dataDirty) {
      forecastPage();
      transferRows(0, 240);
    }
  }

  else if (currentPage == PAGE_CLOCK) {
    if (pageDirty || secondChanged || dataDirty) {
      clockPage(animationsEnabled ? sec : 0.0f);
      transferRows(0, 240);
    }
  }

  else if (currentPage == PAGE_WIFI) {
    if (pageDirty || dataDirty || secondChanged) {
      wifiPage();
      transferRows(0, 240);
    }
  }

  else if (currentPage == PAGE_HISTORY) {
    if (pageDirty || dataDirty) {
      historyPage();
      transferRows(0, 240);
    }
  }

  else if (currentPage == PAGE_SETTINGS) {
    if (pageDirty) {
      settingsPage();
      transferRows(0, 240);
    }
  }

  else if (currentPage == PAGE_SETTINGS_ADVANCED) {
    if (pageDirty) {
      settingsAdvancedPage();
      transferRows(0, 240);
    }
  }

  else if (currentPage == PAGE_AMBIENT) {
    if (pageDirty || secondChanged || dataDirty) {
      ambientPage(animationsEnabled ? sec : 0.0f);
      transferRows(0, 240);
    }
  }

  pageDirty = false;
  dataDirty = false;
  previousSecond = elapsedMs / 1000ULL;
}

// =====================================================
// TOUCH NAVIGATION
// =====================================================

void handleTouch() {
  int x;
  int y;

  if (!getTouchPixel(x, y)) {
    return;
  }

  lastInteractionAt = millis();

  if (portraitMode) {
    handlePortraitTouch(x, y);
    return;
  }

  // HOME ------------------------------------------------
  if (currentPage == PAGE_HOME) {
    // MENI gets priority so the character interaction zone can never steal the tap.
    if (x >= 45 && x <= 275 && y >= 170 && y <= 235) {
      waitTouchRelease();
      goTo(PAGE_MENU);
      return;
    }

    // Character interaction zone, kept clear of the menu area.
    if (homeStyle == 0 && x >= 220 && x <= 319 && y >= 55 && y <= 165) {
      waitTouchRelease();

      if (!speechVisible && characterEvent == CHAR_IDLE) {
        int surprise = random(4);
        if (surprise == 0) startCharacterMotion(CHAR_DANCE, 3000);
        else if (surprise == 1) startCharacterMotion(CHAR_BOUNCE, 2200);
        else if (surprise == 2) startCharacterMotion(CHAR_FLOWER_PICK, 6500);
        else startCharacterMotion(CHAR_BEES, 7000);
      }

      return;
    }
  }

  // MENU ------------------------------------------------
  else if (currentPage == PAGE_MENU) {
    if (uiTheme == THEME_OBSIDIAN) {
      // Hero VREME
      if (y >= 36 && y <= 82) {
        waitTouchRelease();
        goTo(PAGE_WEATHER);
        return;
      }

      // KLIMA / 3 DANA
      if (y >= 92 && y <= 134) {
        waitTouchRelease();
        goTo(x < 160 ? PAGE_CLIMATE : PAGE_FORECAST);
        return;
      }

      // SAT / WIFI / SETTINGS
      if (y >= 144 && y <= 184) {
        waitTouchRelease();
        if (x < 108) goTo(PAGE_CLOCK);
        else if (x < 212) goTo(PAGE_WIFI);
        else goTo(PAGE_SETTINGS);
        return;
      }

      if (y >= 194) {
        waitTouchRelease();
        goTo(PAGE_HOME);
        return;
      }
    }
    else if (uiTheme == THEME_RETRO) {
      if (y >= 40 && y <= 100) {
        waitTouchRelease();
        if (x < 108) goTo(PAGE_CLIMATE);
        else if (x < 212) goTo(PAGE_WEATHER);
        else goTo(PAGE_FORECAST);
        return;
      }

      if (y >= 112 && y <= 172) {
        waitTouchRelease();
        if (x < 108) goTo(PAGE_CLOCK);
        else if (x < 212) goTo(PAGE_WIFI);
        else goTo(PAGE_SETTINGS);
        return;
      }

      if (y >= 190) {
        waitTouchRelease();
        goTo(PAGE_HOME);
        return;
      }
    }
    else if (uiTheme == THEME_INDIE || uiTheme == THEME_JAPAN) {
      if (y >= 38 && y <= 88) {
        waitTouchRelease();
        goTo(x < 160 ? PAGE_CLIMATE : PAGE_WEATHER);
        return;
      }

      if (y >= 98 && y <= 150) {
        waitTouchRelease();
        goTo(x < 160 ? PAGE_FORECAST : PAGE_CLOCK);
        return;
      }

      if (y >= 160 && y <= 204) {
        waitTouchRelease();
        goTo(x < 160 ? PAGE_WIFI : PAGE_SETTINGS);
        return;
      }

      if (y >= 212) {
        waitTouchRelease();
        goTo(PAGE_HOME);
        return;
      }
    }
    else {
      // Nature: classic two-column grid.
      if (x >= 8 && x <= 154 && y >= 36 && y <= 78) {
        waitTouchRelease();
        goTo(PAGE_CLIMATE);
        return;
      }

      if (x >= 166 && x <= 312 && y >= 36 && y <= 78) {
        waitTouchRelease();
        goTo(PAGE_WEATHER);
        return;
      }

      if (x >= 8 && x <= 154 && y >= 89 && y <= 131) {
        waitTouchRelease();
        goTo(PAGE_FORECAST);
        return;
      }

      if (x >= 166 && x <= 312 && y >= 89 && y <= 131) {
        waitTouchRelease();
        goTo(PAGE_CLOCK);
        return;
      }

      if (x >= 8 && x <= 154 && y >= 142 && y <= 184) {
        waitTouchRelease();
        goTo(PAGE_WIFI);
        return;
      }

      if (x >= 166 && x <= 312 && y >= 142 && y <= 184) {
        waitTouchRelease();
        goTo(PAGE_SETTINGS);
        return;
      }

      if (x >= 70 && x <= 250 && y >= 190 && y <= 239) {
        waitTouchRelease();
        goTo(PAGE_HOME);
        return;
      }
    }
  }

  // CLIMATE ---------------------------------------------
  else if (currentPage == PAGE_CLIMATE) {
    if (x >= 70 && x <= 250 && y >= 195 && y <= 239) {
      waitTouchRelease();
      goTo(PAGE_MENU);
      return;
    }
  }

  // WEATHER ---------------------------------------------
  else if (currentPage == PAGE_WEATHER) {
    if (x >= 0 && x <= 160 && y >= 185 && y <= 239) {
      waitTouchRelease();
      goTo(PAGE_FORECAST);
      return;
    }

    if (x >= 160 && x <= 319 && y >= 185 && y <= 239) {
      waitTouchRelease();
      goTo(PAGE_MENU);
      return;
    }
  }

  // FORECAST --------------------------------------------
  else if (currentPage == PAGE_FORECAST) {
    if (x >= 65 && x <= 255 && y >= 185 && y <= 239) {
      waitTouchRelease();
      goTo(PAGE_MENU);
      return;
    }
  }

  // CLOCK -----------------------------------------------
  else if (currentPage == PAGE_CLOCK) {
    if (x >= 70 && x <= 250 && y >= 190 && y <= 239) {
      waitTouchRelease();
      goTo(PAGE_MENU);
      return;
    }
  }

  // WIFI ------------------------------------------------
  else if (currentPage == PAGE_WIFI) {
    if (x >= 70 && x <= 250 && y >= 185 && y <= 239) {
      waitTouchRelease();
      goTo(PAGE_MENU);
      return;
    }
  }

  // HISTORY ---------------------------------------------
  else if (currentPage == PAGE_HISTORY) {
    if (x >= 60 && x <= 260 && y >= 195 && y <= 239) {
      waitTouchRelease();
      goTo(PAGE_SETTINGS);
      return;
    }
  }

  // SETTINGS / PAGE 1 -----------------------------------
  else if (currentPage == PAGE_SETTINGS) {
    // Unit
    if (y >= 45 && y <= 86) {
      if (x < 160) useFahrenheit = false;
      else useFahrenheit = true;
      saveSettings();
      waitTouchRelease();
      dataDirty = true;
      pageDirty = true;
      return;
    }

    // Refresh interval
    if (y >= 94 && y <= 135) {
      if (x < 108) refreshMinutes = 5;
      else if (x < 212) refreshMinutes = 15;
      else refreshMinutes = 30;
      saveSettings();
      lastWeatherFetch = 0;
      waitTouchRelease();
      pageDirty = true;
      return;
    }

    // City
    if (y >= 143 && y <= 185) {
      if (x < 110) cityIndex = 0;
      else if (x < 210) cityIndex = 1;
      else cityIndex = 2;

      saveSettings();
      weatherOK = false;
      lastWeatherAttempt = 0;
      lastWeatherFetch = 0;
      waitTouchRelease();
      pageDirty = true;
      return;
    }

    // Bottom nav
    if (y >= 190 && y <= 239) {
      if (x < 110) {
        waitTouchRelease();
        goTo(PAGE_MENU);
      } else if (x < 212) {
        waitTouchRelease();
        goTo(PAGE_HISTORY);
      } else {
        waitTouchRelease();
        goTo(PAGE_SETTINGS_ADVANCED);
      }
      return;
    }
  }

  // SETTINGS / PAGE 2 -----------------------------------
  else if (currentPage == PAGE_SETTINGS_ADVANCED) {
    // Theme row 1
    if (y >= 39 && y <= 73) {
      if (x < 108) uiTheme = THEME_NATURE;
      else if (x < 212) uiTheme = THEME_OBSIDIAN;
      else uiTheme = THEME_RETRO;

      saveSettings();
      waitTouchRelease();
      pageDirty = true;
      return;
    }

    // Theme row 2
    if (y >= 73 && y <= 106) {
      if (x < 160) uiTheme = THEME_INDIE;
      else uiTheme = THEME_JAPAN;

      saveSettings();
      waitTouchRelease();
      pageDirty = true;
      return;
    }

    // Behavior
    if (y >= 112 && y <= 149) {
      if (x < 108) animationsEnabled = !animationsEnabled;
      else if (x < 212) quietMode = !quietMode;
      else alertsEnabled = !alertsEnabled;

      saveSettings();
      waitTouchRelease();
      pageDirty = true;
      return;
    }

    // Orientation / ambient / character
    if (y >= 154 && y <= 191) {
      if (x < 108) {
        waitTouchRelease();
        portraitMode = !portraitMode;
        saveSettings();
        tft.setRotation(portraitMode ? 0 : 1);
        tft.fillScreen(BLACK);

        // First ever portrait entry immediately gets a real portrait calibration,
        // before the user has to navigate a potentially mis-mapped UI.
        if (portraitMode && (!portraitTouchCalibrated || !portraitCalV2)) {
          runPortraitTouchCalibration();
        }

        pageDirty = true;
        dataDirty = true;
        return;
      }
      else if (x < 212) ambientEnabled = !ambientEnabled;
      else femaleCharacter = !femaleCharacter;

      saveSettings();
      waitTouchRelease();
      pageDirty = true;
      return;
    }

    // Home style / touch calibration
    if (y >= 190 && y <= 224) {
      if (x < 160) {
        homeStyle = homeStyle == 0 ? 1 : 0;
        saveSettings();
        waitTouchRelease();
        pageDirty = true;
      } else {
        waitTouchRelease();
        runTouchCalibration();
      }
      return;
    }

    if (y >= 223 && y <= 239) {
      if (x < 160) {
        waitTouchRelease();
        goTo(PAGE_SETTINGS);
      } else {
        waitTouchRelease();
        goTo(PAGE_MENU);
      }
      return;
    }
  }

  // AMBIENT ---------------------------------------------
  else if (currentPage == PAGE_AMBIENT) {
    waitTouchRelease();
    lastInteractionAt = millis();
    goTo(PAGE_HOME);
    return;
  }

  // Touch outside any active button: just release it.
  waitTouchRelease();
}

// =====================================================
// SETUP
// =====================================================



void drawMartinOSBootPortrait(int stage) {
  tft.fillScreen(rgb(3,7,14));

  tft.drawRoundRect(7,7,226,306,12,rgb(35,61,91));
  tft.drawFastHLine(18,20,80,rgb(78,151,219));
  tft.drawFastHLine(18,23,46,rgb(162,88,219));

  tft.setTextWrap(false);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.setCursor(18,38);
  tft.print("MartinOS");

  pLabel(19,73,"ATMOSPHERE CORE",rgb(95,181,228),1);
  pLabel(19,90,"ESP32-S3 / N16R8",rgb(118,136,159),1);

  // Clean system mark - no character/sprite on boot.
  tft.drawCircle(120,137,28,rgb(46,82,125));
  tft.drawCircle(120,137,18,rgb(79,151,219));
  tft.fillCircle(120,137,5,rgb(162,88,219));
  tft.drawFastHLine(82,137,22,rgb(46,82,125));
  tft.drawFastHLine(136,137,22,rgb(46,82,125));
  pCenter(174,"SYSTEM CORE",rgb(91,111,134),1);

  const char* l1 = stage >= 0 ? "[OK] DISPLAY" : "[..] DISPLAY";
  const char* l2 = stage >= 1 ? "[OK] UI CORE" : "[..] UI CORE";
  const char* l3 = stage >= 2 ? "[OK] SENSORS" : "[..] SENSORS";
  const char* l4 = stage >= 3 ? "[OK] NETWORK" : "[..] NETWORK";

  pLabel(28,201,l1,stage>=0?rgb(89,213,151):rgb(109,124,143),1);
  pLabel(28,221,l2,stage>=1?rgb(89,213,151):rgb(109,124,143),1);
  pLabel(28,241,l3,stage>=2?rgb(89,213,151):rgb(109,124,143),1);
  pLabel(28,261,l4,stage>=3?rgb(89,213,151):rgb(109,124,143),1);

  tft.drawRoundRect(18,282,204,13,6,rgb(45,62,82));
  int progress=47+stage*47;
  if(progress>196) progress=196;
  tft.fillRoundRect(22,286,progress,5,2,rgb(76,145,222));

  if(stage>=3) pCenter(303,"HOLD TOUCH = CAL",rgb(91,111,134),1);
}

void drawMartinOSBootScreen(int stage) {
  frame.fillScreen(rgb(3, 7, 14));

  frame.drawRoundRect(7, 7, 306, 226, 12, rgb(35, 61, 91));
  frame.drawFastHLine(20, 20, 94, rgb(78, 151, 219));
  frame.drawFastHLine(20, 23, 54, rgb(162, 88, 219));

  label(20, 37, "MartinOS", WHITE, 3);
  label(21, 68, "ATMOSPHERE CORE", rgb(95, 181, 228), 1);
  label(21, 83, "ESP32-S3 / N16R8", rgb(118, 136, 159), 1);

  // Clean system mark - no character/sprite on boot.
  frame.drawCircle(255, 70, 28, rgb(46, 82, 125));
  frame.drawCircle(255, 70, 18, rgb(79, 151, 219));
  frame.fillCircle(255, 70, 5, rgb(162, 88, 219));
  frame.drawFastHLine(217, 70, 22, rgb(46, 82, 125));
  frame.drawFastHLine(271, 70, 22, rgb(46, 82, 125));
  label(226, 104, "SYSTEM CORE", rgb(91, 111, 134), 1);

  const char* l1 = stage >= 0 ? "[OK] DISPLAY" : "[..] DISPLAY";
  const char* l2 = stage >= 1 ? "[OK] UI CORE" : "[..] UI CORE";
  const char* l3 = stage >= 2 ? "[OK] SENSORS" : "[..] SENSORS";
  const char* l4 = stage >= 3 ? "[OK] NETWORK" : "[..] NETWORK";

  label(21, 120, l1, stage >= 0 ? rgb(89, 213, 151) : rgb(109, 124, 143), 1);
  label(21, 139, l2, stage >= 1 ? rgb(89, 213, 151) : rgb(109, 124, 143), 1);
  label(21, 158, l3, stage >= 2 ? rgb(89, 213, 151) : rgb(109, 124, 143), 1);
  label(21, 177, l4, stage >= 3 ? rgb(89, 213, 151) : rgb(109, 124, 143), 1);

  frame.drawRoundRect(20, 204, 280, 13, 6, rgb(45, 62, 82));
  int progress = 65 + stage * 65;
  if (progress > 272) progress = 272;
  frame.fillRoundRect(24, 208, progress, 5, 2, rgb(76, 145, 222));

  label(232, 221, "BOOT", rgb(91, 111, 134), 1);

  if (stage >= 3) {
    label(21, 221, "HOLD TOUCH = FULL CAL", rgb(91, 111, 134), 1);
  }

  tft.drawRGBBitmap(0, 0, frame.getBuffer(), 320, 240);
}

void drawMartinOSBootStage(int stage) {
  if (portraitMode) drawMartinOSBootPortrait(stage);
  else drawMartinOSBootScreen(stage);
}

void showMartinOSBoot() {
  // Settings are loaded before this call, so boot follows the saved orientation.
  tft.setRotation(portraitMode ? 0 : 1);
  tft.fillScreen(BLACK);

  drawMartinOSBootStage(0);
  delay(220);
  drawMartinOSBootStage(1);
  delay(220);
  drawMartinOSBootStage(2);
  delay(220);
}

void setup() {
  Serial.begin(115200);
  randomSeed((uint32_t)micros());

  delay(80);

  pinMode(TFT_CS, OUTPUT);
  pinMode(TOUCH_CS, OUTPUT);

  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TOUCH_CS, HIGH);

  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);

  // 32 MHz skracuje vreme prenosa frejma i smanjuje vidljivo tearing/treperenje.
  // Ako breadboard zice izazovu artefakte, vrati samo ovu vrednost na 20000000.
  tft.begin(32000000);
  tft.setRotation(1);
  tft.setTextWrap(false);
  tft.fillScreen(BLACK);

  touch.begin();
  // IMPORTANT: do not call touch.setRotation() with the hard-coded affine mapping.

  frame.setTextWrap(false);

  memoryOK = frame.getBuffer() != nullptr && portraitScene.getBuffer() != nullptr;

  if (!memoryOK) {
    tft.fillScreen(BG);
    tft.setCursor(10, 20);
    tft.setTextColor(WHITE);
    tft.print("Not enough RAM for canvas");
    return;
  }

  bool valid = START_YEAR >= 2000 && START_MONTH >= 1 && START_MONTH <= 12;

  if (valid) {
    valid = START_DAY >= 1 && START_DAY <= monthDays(START_YEAR, START_MONTH);
  }

  valid = valid &&
          START_HOUR >= 0 && START_HOUR < 24 &&
          START_MINUTE >= 0 && START_MINUTE < 60 &&
          START_SECOND >= 0 && START_SECOND < 60;

  if (!valid) {
    memoryOK = false;

    tft.fillScreen(BG);
    tft.setCursor(10, 20);
    tft.setTextColor(WHITE);
    tft.print("Invalid START date/time");
    return;
  }

  for (int i = 0; i < N; i++) {
    history[i] = NAN;
    historyHum[i] = NAN;
  }

  dht.begin();

  prefs.begin("atmosUI", false);
  loadSettings();

  // Custom MartinOS startup instead of a blank/white wait.
  showMartinOSBoot();

  // Pokusaj internet konekciju, ali ako ne uspe uredjaj nastavlja offline.
  connectWiFi(12000);
  if (WiFi.status() == WL_CONNECTED) {
    printNetworkDiagnostics();
    fetchWeather();
    syncClockFromNTP();
  }

  drawMartinOSBootStage(3);

  // Emergency calibration shortcut:
  // keep a finger/stylus pressed during the final MartinOS boot screen.
  uint32_t bootTouchStart = millis();
  bool forceCalibration = false;

  while ((uint32_t)(millis() - bootTouchStart) < 900UL) {
    TS_Point bp = touch.getPoint();

    if (bp.z > 250) {
      delay(350);
      bp = touch.getPoint();

      if (bp.z > 250) {
        forceCalibration = true;
        break;
      }
    }

    delay(15);
  }

  if (forceCalibration) {
    waitTouchRelease();
    if (portraitMode) runPortraitTouchCalibration();
    else runTouchCalibration();
  }

  // Boot already follows the saved orientation; re-apply it cleanly for the UI.
  applyScreenOrientation();

  // Portrait keeps its independent calibration. If none exists yet, run it once.
  if (portraitMode && (!portraitTouchCalibrated || !portraitCalV2)) {
    runPortraitTouchCalibration();
  }

  prevTick = millis();
  sensorTick = prevTick;
  frameTick = prevTick;
  lastInteractionAt = prevTick;

  Serial.println("ATMOSPHERE TOUCH V10.3.5.1 + COMPILE FIX START");
  Serial.println(portraitMode ? "TFT PORTRAIT 240x320 / DHT GPIO4 / TOUCH CS GPIO6" : "TFT LANDSCAPE 320x240 / DHT GPIO4 / TOUCH CS GPIO6");

  // First DHT read immediately so the home screen has real values sooner.
  sampleSensor();

  drawFrame();
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  if (!memoryOK) {
    delay(100);
    return;
  }

  uint32_t now = millis();

  uint32_t dt = now - prevTick;
  prevTick = now;

  elapsedMs += dt;

  // NTP vodi sat kada je dostupan; u suprotnom koristi lokalni fallback clock.
  if (!timeSynced) {
    tickClock(dt);
  }

  maintainOnlineServices(now);

  if (animationsEnabled) {
    for (int i = 0; i < 4; i++) {
      clouds[i] -= (3.0f + i * 1.3f) * dt / 1000.0f;

      while (clouds[i] < -65) {
        clouds[i] += 450;
      }
    }
  }

  if ((uint32_t)(now - sensorTick) >= SAMPLE_MS) {
    sensorTick = now;
    sampleSensor();
  }

  // Enter calm ambient screen after 60 seconds without touch.
  if (ambientEnabled && currentPage != PAGE_AMBIENT &&
      (uint32_t)(now - lastInteractionAt) >= 60000UL) {
    goTo(PAGE_AMBIENT);
  }

  // Personality system: speech or lightweight random movement every ~10-15 s.
  updateCharacterActivity(now);

  // Touch is checked continuously, independent of frame rate.
  handleTouch();

  if ((uint32_t)(now - frameTick) >= FRAME_MS || pageDirty) {
    frameTick = now;
    drawFrame();
    frameTick = millis();
  }

  delay(1);
}