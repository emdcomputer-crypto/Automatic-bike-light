/*
  SMART BIKE - FIRMWARE v4.0 (Code_3, self-contained)

  ESP32 + MPU6050 automatic bike lighting, with a phone-controlled
  Wi-Fi web page for the manual modes. No Bluetooth, no app, no
  internet needed - a phone connects straight to the bike's own
  Wi-Fi hotspot and loads the page from the ESP32 itself.

  WHY THIS VERSION IS A REWRITE, NOT ANOTHER PATCH
  -------------------------------------------------
  Every earlier Code_3 build worked by textually gluing Code_2.ino
  into this file with preprocessor macros, then deciding, loop by
  loop, whether to run Code_2's whole sensor engine at all:

    if (mode == AUTO || mode == HAZARD) baseLoop();   // runs Code_2
    else                                yield();      // skips it

  That split is what caused "only Auto and Hazard do anything":
  Code_2's crash-SOS latch can only clear itself once the bike is
  ACTUALLY ridden again, which can never happen while it is sitting
  on a desk being tested from a phone. One bump while wiring it up,
  and the latch stuck for the rest of that session - and because the
  sensor engine was skipped in every other mode, nothing could ever
  clear it or even tell you it had happened.

  This version is one plain file, one loop, no included sketch, and
  no code path that silently stops updating the sensors:
    - The sensor engine (MPU6050 read, filtering, orientation, brake
      and turn detection) runs every loop, in every mode, so the
      phone's live readout never freezes on you mid-test.
    - Automatic crash detection itself only runs while actually
      riding (AUTO or HAZARD) - tapping any phone mode is by
      definition proof someone is there and fine, so a bench-test
      bump can never again block Manual, Flash, Chase, or Off.
    - Exactly one function (renderLights) ever writes to the LED
      pins, so nothing can race with the phone's commands.
    - Every function used anywhere in the file has a plain forward
      declaration up top, so the Arduino build never has to guess a
      prototype from code - this sketch embeds a large HTML page as
      a raw string, which has tripped up that guesswork before.

  Everything else keeps working exactly as before: MPU6050
  calibration, tilt-compensated braking, turn signals, hazard
  flashing, the emergency-stop flash, crash SOS, and the Serial
  tuning console. Code_2/Code_2.ino is no longer needed to build
  this file - it is left in place only as a reference copy.
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>

// ===================== FEATURE TOGGLES =====================
// Optional features that need extra hardware wired up:
#define ENABLE_HAZARD_BUTTON      0   // momentary button: GPIO33 -> GND
#define ENABLE_BATTERY_MONITOR    0   // voltage divider into GPIO34
#define ENABLE_AMBIENT_BRIGHTNESS 0   // LDR voltage divider into GPIO35
#define ENABLE_BLE_TELEMETRY      0   // no extra hardware, just flash/RAM cost

// Software-only, work with the original hardware, no wiring changes:
#define ENABLE_TILT_COMPENSATION  1
#define ENABLE_CRASH_DETECTION    1
#define ENABLE_ESS_FLASH          1
#define ENABLE_DRL                0
#define ENABLE_SERIAL_TUNING      1

#if ENABLE_BLE_TELEMETRY
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29578f
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292f
#endif

Adafruit_MPU6050 mpu;
Preferences prefs;

// ===================== LED PINS (unchanged from Code#1/#2) =====================
const int LEFT1 = 13, LEFT2 = 14, LEFT3 = 16;
const int BRAKE1 = 18, BRAKE2 = 19, BRAKE3 = 23, BRAKE4 = 25;
const int RIGHT1 = 26, RIGHT2 = 27, RIGHT3 = 32;

const int brakePins[4] = { BRAKE1, BRAKE2, BRAKE3, BRAKE4 };
const int leftPins[3]  = { LEFT3,  LEFT2,  LEFT1  };
const int rightPins[3] = { RIGHT1, RIGHT2, RIGHT3 };

// ===================== OPTIONAL HARDWARE PINS =====================
#if ENABLE_HAZARD_BUTTON
const int HAZARD_BUTTON_PIN = 33;   // INPUT_PULLUP, active LOW
#endif
#if ENABLE_BATTERY_MONITOR
const int BATTERY_ADC_PIN = 34;
const float BATTERY_DIVIDER_RATIO = 2.0f;
const float ADC_REF_VOLTAGE = 3.3f;
#endif
#if ENABLE_AMBIENT_BRIGHTNESS
const int LDR_ADC_PIN = 35;
#endif

// ===================== PWM =====================
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;

// ===================== AXIS / SIGN CONFIG =====================
#define BRAKE_AXIS_X
#define TURN_AXIS_Z
const bool BRAKING_IS_NEGATIVE = true;
const float GRAVITY_MPS2 = 9.80665f;

// ===================== LIVE-TUNABLE PARAMETERS =====================
struct Tunables {
  float brakeTrigger      = 1.5f;
  float hardBrake         = 5.0f;
  float minBrakeBright    = 90.0f;
  float brakeFlashMs      = 100.0f;
  float brakeSustainMs    = 450.0f;
  float brakeMaxMs        = 3000.0f;
  float brightnessSmooth  = 0.30f;
  float stillJitter       = 0.60f;
  float stillGyroMax      = 0.20f;
  float stillConfirmMs    = 350.0f;
  float turnThreshold     = 0.5f;
  float turnConfirmMs     = 90.0f;
  float turnHoldMs        = 1200.0f;
  float turnAnimSlowMs    = 180.0f;
  float turnAnimFastMs    = 60.0f;
  float accelFilter       = 0.18f;
  float gyroFilter        = 0.30f;
  float baselineAdapt     = 0.006f;
  float spikeClamp        = 6.0f;
  float essThreshold      = 5.0f;
  float essFlashHz        = 10.0f;
  float essDurationMs     = 500.0f;
  float impactThresholdG  = 3.0f;
  float crashStillMs      = 3000.0f;
  float fallenAngleDeg    = 65.0f;
  float fallenConfirmMs   = 1500.0f;
  float drlBrightness     = 12.0f;
  float compAlpha         = 0.98f;
  float lowBatteryVolts   = 3.3f;
};
Tunables cfg;

struct TunableEntry { const char* name; float* ptr; };
TunableEntry tunableTable[] = {
  {"BRAKE_TRIGGER",       &cfg.brakeTrigger},
  {"HARD_BRAKE",          &cfg.hardBrake},
  {"MIN_BRAKE_BRIGHT",    &cfg.minBrakeBright},
  {"BRAKE_FLASH_MS",      &cfg.brakeFlashMs},
  {"BRAKE_SUSTAIN_MS",    &cfg.brakeSustainMs},
  {"BRAKE_MAX_MS",        &cfg.brakeMaxMs},
  {"BRIGHTNESS_SMOOTH",   &cfg.brightnessSmooth},
  {"STILL_JITTER",        &cfg.stillJitter},
  {"STILL_GYRO_MAX",      &cfg.stillGyroMax},
  {"STILL_CONFIRM_MS",    &cfg.stillConfirmMs},
  {"TURN_THRESHOLD",      &cfg.turnThreshold},
  {"TURN_CONFIRM_MS",     &cfg.turnConfirmMs},
  {"TURN_HOLD_MS",        &cfg.turnHoldMs},
  {"TURN_ANIM_SLOW_MS",   &cfg.turnAnimSlowMs},
  {"TURN_ANIM_FAST_MS",   &cfg.turnAnimFastMs},
  {"ACCEL_FILTER",        &cfg.accelFilter},
  {"GYRO_FILTER",         &cfg.gyroFilter},
  {"BASELINE_ADAPT",      &cfg.baselineAdapt},
  {"SPIKE_CLAMP",         &cfg.spikeClamp},
  {"ESS_THRESHOLD",       &cfg.essThreshold},
  {"ESS_FLASH_HZ",        &cfg.essFlashHz},
  {"ESS_DURATION_MS",     &cfg.essDurationMs},
  {"IMPACT_THRESHOLD_G",  &cfg.impactThresholdG},
  {"CRASH_STILL_MS",      &cfg.crashStillMs},
  {"FALLEN_ANGLE_DEG",    &cfg.fallenAngleDeg},
  {"FALLEN_CONFIRM_MS",   &cfg.fallenConfirmMs},
  {"DRL_BRIGHTNESS",      &cfg.drlBrightness},
  {"COMP_ALPHA",          &cfg.compAlpha},
  {"LOW_BATTERY_VOLTS",   &cfg.lowBatteryVolts},
};
const int NUM_TUNABLES = sizeof(tunableTable) / sizeof(tunableTable[0]);

// ===================== CORE SENSOR STATE =====================
float accelBaseline = 0, gyroZBias = 0;
float pitchBaseline = 0, rollBaseline = 0;
float pitchEstimate = 0, rollEstimate = 0;
float filtAccel = 0, filtGyroZ = 0;
float deceleration = 0;
float lastTurnRate = 0;

float accelMean = 0, accelVar = 0, accelStd = 0, gyroMag = 0;
bool  isStationary = false;
unsigned long stillSince = 0;

bool  brakeActive = false;
unsigned long brakeStart = 0, brakeLastDecel = 0;
float currentBrightness = 0;

#if ENABLE_ESS_FLASH
bool essActive = false;
unsigned long essStart = 0;
#endif

bool leftTurning = false, rightTurning = false;
unsigned long leftTurnTime = 0, rightTurnTime = 0;
int  leftStep = 0, rightStep = 0;
unsigned long leftAnimAt = 0, rightAnimAt = 0;
unsigned long leftCandidateSince = 0, rightCandidateSince = 0;

bool hazardActive = false;

unsigned long lastOrientationMicros = 0;
unsigned long bootMillis = 0;

#if ENABLE_CRASH_DETECTION
bool crashDetected = false;
bool impactFlag = false;
unsigned long impactAt = 0;
unsigned long fallenSince = 0;
unsigned long crashClearMovingSince = 0;
#endif

#if ENABLE_AMBIENT_BRIGHTNESS
float brightnessScale = 1.0f;
unsigned long lastLdrRead = 0;
#endif

#if ENABLE_BATTERY_MONITOR
float batteryVoltage = 0;
bool lowBatteryFlag = false;
unsigned long lastBattRead = 0;
#endif

#if ENABLE_SERIAL_TUNING
String serialBuffer = "";
#endif

#if ENABLE_BLE_TELEMETRY
#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLECharacteristic *bleChar = nullptr;
unsigned long lastBleUpdate = 0;
#endif

// Blob-saved structs for NVS, kept separate from cfg/live state so a
// firmware update that adds fields doesn't corrupt old saved data.
struct CalData {
  float accelBaseline = 0, gyroZBias = 0, pitchBaseline = 0, rollBaseline = 0;
  bool  valid = false;
};
CalData calData;

// ===================== PHONE CONTROLLER STATE =====================
const char *CONTROL_AP_NAME = "SmartBike";
const char *CONTROL_AP_PASSWORD = "Bike123";

WebServer web(80);

enum ControlMode {
  MODE_AUTO,
  MODE_HAZARD,
  MODE_MANUAL,
  MODE_FLASH,
  MODE_CHASE,
  MODE_OFF
};

ControlMode controlMode = MODE_AUTO;

// Physical bit order:
// 0 = L1, 1 = L2, 2 = L3,
// 3 = B1, 4 = B2, 5 = B3, 6 = B4,
// 7 = R1, 8 = R2, 9 = R3.
uint16_t manualPhysicalMask = 0;

uint8_t redBrightness = 255;
unsigned long modeStartedAt = 0;
bool ecoRadioMode = false;

const unsigned long TEST_MODE_TIMEOUT_MS = 120000UL;

// Browser rear-view order:
// L3 L2 L1 | B1 B2 B3 B4 | R3 R2 R1
const uint8_t UI_TO_PHYSICAL[10] = {
  9, 8, 7,
  3, 4, 5, 6,
  2, 1, 0
};

// ===================== FORWARD DECLARATIONS =====================
// Every function used anywhere in this file is declared here so the
// Arduino build never has to guess a prototype from the code below.
void updateOrientation(const sensors_event_t &a, const sensors_event_t &g, float dt);
void runCalibration(int samples, float blendExisting);
void animateTurn(const int pins[3], int &step, unsigned long &animAt, float turnRateValue);
void startupLEDTest();
void sensorError();
void readyAnimation();
void leftLights(); void leftOff();
void rightLights(); void rightOff();
void brakeLights(); void brakeOff();
void setBrakeBrightness(int b);
void allLightsOn(); void allLightsOff();
void saveTunables(); void loadTunables();
void saveCalibration(); bool loadCalibration();

#if ENABLE_CRASH_DETECTION
void updateCrashDetection(const sensors_event_t &a, unsigned long now);
void renderCrashBeacon(unsigned long now);
#endif
#if ENABLE_AMBIENT_BRIGHTNESS
void updateAmbientBrightness(unsigned long now);
#endif
#if ENABLE_BATTERY_MONITOR
void updateBatteryMonitor(unsigned long now);
#endif
#if ENABLE_SERIAL_TUNING
void handleSerialInput();
void processCommand(String line);
void printHelp();
void printStatus();
#endif
#if ENABLE_BLE_TELEMETRY
void setupBLE();
void updateBLE(unsigned long now);
#endif
#if ENABLE_HAZARD_BUTTON
void checkHazardButton(unsigned long now);
#endif

void updateSensors(unsigned long now, bool doCrashCheck);
void renderLights(unsigned long now);
void renderAutoAndHazard(unsigned long now);

void startController();
void handleRoot();
void handleStatus();
void handleMode();
void handleLed();
void handleAll();
void handleBrightness();
void handleEco();
void handleSelfTest();
void handleCalibration();
void handleLegacyCommand();

void sendOk();
void selectMode(ControlMode mode);

uint16_t currentPhysicalMask(unsigned long now);
uint16_t physicalToUiMask(uint16_t raw);
uint16_t activePhysicalMask(unsigned long now);

void writeManualPhysicalMask(uint16_t raw);
void setEcoRadio(bool enabled);
const char *modeName();

const char CONTROL_PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta charset="utf-8">
<title>Smart Bike</title>

<style>
* {
  box-sizing: border-box;
}

body {
  margin: 0;
  background: #070a10;
  color: #f5f7fb;
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial, sans-serif;
}

main {
  max-width: 530px;
  margin: auto;
  padding: 16px 12px 30px;
}

.top {
  display: flex;
  align-items: center;
  justify-content: space-between;
  margin: 4px 4px 18px;
}

.brand {
  font-size: 22px;
  font-weight: 800;
  letter-spacing: 2px;
}

.brand small {
  display: block;
  font-size: 10px;
  letter-spacing: 1px;
  color: #91a0b9;
  margin-top: 4px;
}

.connection {
  font-size: 11px;
  font-weight: bold;
  letter-spacing: 1px;
  border-radius: 20px;
  padding: 8px 10px;
  background: #1c3144;
  color: #8bd4ff;
}

.panel {
  background: #121926;
  border: 1px solid #293750;
  border-radius: 16px;
  padding: 15px;
  margin: 12px 0;
}

.panel.safe {
  border-color: #28765f;
}

.panel.warn {
  border-color: #996920;
}

.panel.alert {
  border-color: #a33a4e;
}

.kicker {
  font-size: 11px;
  font-weight: 600;
  color: #9baac2;
}

.mode {
  font-size: 27px;
  font-weight: bold;
  margin: 5px 0;
}

.detail {
  min-height: 18px;
  color: #bac6d9;
  font-size: 13px;
  line-height: 1.35;
}

.buttons {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 9px;
  margin-top: 14px;
}

button {
  min-height: 46px;
  border: 1px solid #3b4c68;
  border-radius: 11px;
  background: #202c40;
  color: #f4f7ff;
  font-size: 14px;
  font-weight: bold;
  padding: 10px 8px;
}

button.active {
  background: #1477b8;
  border-color: #67c4f6;
}

button.hazard {
  background: #4d3518;
  border-color: #976a20;
}

button.off {
  background: #4a2330;
  border-color: #914254;
}

.titleline {
  display: flex;
  justify-content: space-between;
  align-items: end;
  gap: 10px;
}

.titleline h2 {
  font-size: 17px;
  margin: 4px 0 0;
}

.small {
  font-size: 11px;
  color: #9caac2;
}

.mapnote {
  text-align: center;
  color: #aab7ca;
  font-size: 11px;
  margin: 14px 0 8px;
}

.leds {
  display: grid;
  grid-template-columns: repeat(10, minmax(0, 1fr));
  gap: 4px;
}

.led {
  height: 49px;
  min-height: 49px;
  padding: 3px 0 0;
  background: #1d2636;
  border: 1px solid #40506b;
  border-radius: 11px;
  color: #c0ccde;
  font-size: 10px;
  font-weight: bold;
  display: flex;
  align-items: center;
  justify-content: center;
}

button.led {
  cursor: pointer;
}

.led.gap1 {
  margin-left: 7px;
}

.led.gap2 {
  margin-left: 7px;
}

.led.yellow.on {
  background: #a87508;
  border-color: #ffd052;
  color: #fff;
  box-shadow: 0 0 12px #c98b08;
}

.led.red.on {
  background: #ae2943;
  border-color: #ff7890;
  color: #fff;
  box-shadow: 0 0 12px #d82e4e;
}

.twobuttons {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 9px;
  margin-top: 13px;
}

.slider {
  margin-top: 15px;
}

.slider label {
  display: block;
  font-size: 13px;
  color: #c6d0e0;
  margin-bottom: 8px;
}

.slider input {
  width: 100%;
  accent-color: #ee4f68;
}

.metrics {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 9px;
  margin-top: 12px;
}

.metric {
  background: #1a2333;
  border-radius: 11px;
  padding: 11px;
}

.metric span {
  display: block;
  color: #9baac2;
  font-size: 10px;
  margin-bottom: 4px;
}

.metric b {
  font-size: 20px;
}

.metric i {
  font-style: normal;
  font-size: 11px;
  color: #b7c2d6;
}

.flags {
  margin-top: 12px;
  color: #c6d1e2;
  font-size: 13px;
  line-height: 1.65;
}

.good {
  color: #78dfab;
  font-weight: bold;
}

.bad {
  color: #ff8b9b;
  font-weight: bold;
}

.eco {
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 12px;
}

.eco b {
  font-size: 14px;
}

.eco span {
  display: block;
  font-size: 12px;
  color: #a9b6c9;
  line-height: 1.35;
  margin-top: 4px;
}

.eco button {
  min-width: 90px;
  min-height: 40px;
}

.footer {
  text-align: center;
  color: #98a6bd;
  font-size: 11px;
  line-height: 1.6;
  margin: 14px 8px;
}

@media (max-width: 370px) {
  main {
    padding-left: 8px;
    padding-right: 8px;
  }

  .leds {
    gap: 2px;
  }

  .led {
    height: 43px;
    min-height: 43px;
    font-size: 9px;
  }

  .led.gap1,
  .led.gap2 {
    margin-left: 3px;
  }
}
</style>
</head>

<body>
<main>

<header class="top">
  <div class="brand">
    SMART BIKE
    <small>PHONE CONTROL</small>
  </div>
  <div class="connection" id="connection">CONNECTING</div>
</header>

<section class="panel safe" id="modepanel">
  <div class="kicker">Active mode</div>
  <div class="mode" id="mode">LOADING</div>
  <div class="detail" id="detail">Checking controller status.</div>

  <div class="buttons">
    <button id="auto" onclick="setMode('auto')">RIDE AUTO</button>
    <button id="hazard" class="hazard" onclick="setMode('hazard')">
      HAZARD
    </button>
    <button id="manual" onclick="setMode('manual')">MANUAL TEST</button>
    <button id="flash" onclick="setMode('flash')">FLASH ALL</button>
    <button id="chase" onclick="setMode('chase')">CHASE</button>
    <button id="off" class="off" onclick="setMode('off')">LIGHTS OFF</button>
  </div>
</section>

<section class="panel">
  <div class="titleline">
    <div>
      <div class="kicker">Live LED output</div>
      <h2>What the bike is showing</h2>
    </div>
    <div class="small">REAR VIEW</div>
  </div>

  <div class="mapnote">
    L3 L2 L1 | B1 B2 B3 B4 | R3 R2 R1
  </div>

  <div class="leds" id="preview"></div>
</section>

<section class="panel">
  <div class="titleline">
    <div>
      <div class="kicker">Manual LED test</div>
      <h2>Individual LEDs</h2>
    </div>
    <div class="small">REAR VIEW</div>
  </div>

  <div class="mapnote">
    L3 L2 L1 | B1 B2 B3 B4 | R3 R2 R1
  </div>

  <div class="leds" id="leds"></div>

  <div class="twobuttons">
    <button onclick="setAll(1)">ALL ON</button>
    <button onclick="setAll(0)">ALL OFF</button>
    <button onclick="runSelfTest()">LED SELF TEST</button>
    <button onclick="runCalibration()">RECALIBRATE</button>
  </div>

  <div class="slider">
    <label for="brightness">
      RED LED BRIGHTNESS:
      <b id="brightvalue">255</b> / 255
    </label>
    <input
      id="brightness"
      type="range"
      min="0"
      max="255"
      value="255"
      oninput="setBrightness(this.value)"
    >
  </div>
</section>

<section class="panel">
  <div class="titleline">
    <div>
      <div class="kicker">Live sensor data</div>
      <h2>Ride status</h2>
    </div>
    <div class="small" id="movement">--</div>
  </div>

  <div class="metrics">
    <div class="metric">
      <span>DECELERATION</span>
      <b id="decel">--</b>
      <i>m/s2</i>
    </div>

    <div class="metric">
      <span>PITCH</span>
      <b id="pitch">--</b>
      <i>deg</i>
    </div>

    <div class="metric">
      <span>ROLL</span>
      <b id="roll">--</b>
      <i>deg</i>
    </div>

    <div class="metric">
      <span>BRAKE</span>
      <b id="brake">--</b>
    </div>
  </div>

  <div class="flags" id="flags">Reading sensor state...</div>
</section>

<section class="panel eco">
  <div>
    <b>LOW POWER RADIO</b>
    <span>
      Reduces Wi-Fi range and radio power. It never dims automatic safety
      lights.
    </span>
  </div>
  <button id="eco" onclick="toggleEco()">OFF</button>
</section>

<div class="footer">
  AUTO is the riding mode. Manual, Flash, Chase, and Lights Off return to
  AUTO after 2 minutes. Crash SOS always wins.
  <br>
  Controller uptime: <span id="uptime">--</span>
</div>

</main>

<script>
var uiMask = 0;
var previewMask = 0;
var eco = false;

var labels = [
  'L3', 'L2', 'L1',
  'B1', 'B2', 'B3', 'B4',
  'R3', 'R2', 'R1'
];

var red = [
  false, false, false,
  true, true, true, true,
  false, false, false
];

var box = document.getElementById('leds');
var preview = document.getElementById('preview');

for (var i = 0; i < 10; i++) {
  var b = document.createElement('button');

  b.className =
    'led ' +
    (red[i] ? 'red' : 'yellow') +
    (i === 3 ? ' gap1' : '') +
    (i === 7 ? ' gap2' : '');

  b.textContent = labels[i];
  b.setAttribute('aria-label', 'Toggle ' + labels[i]);
  b.setAttribute('data-pos', i);

  b.onclick = function() {
    toggleLed(parseInt(this.getAttribute('data-pos')));
  };

  box.appendChild(b);

  var p = document.createElement('div');

  p.className =
    'led ' +
    (red[i] ? 'red' : 'yellow') +
    (i === 3 ? ' gap1' : '') +
    (i === 7 ? ' gap2' : '');

  p.textContent = labels[i];
  preview.appendChild(p);
}

function get(path, done) {
  var x = new XMLHttpRequest();

  x.open('GET', path, true);

  x.onreadystatechange = function() {
    if (x.readyState === 4 && done) {
      done(x);
    }
  };

  x.onerror = function() {
    document.getElementById('connection').textContent = 'OFFLINE';
    document.getElementById('detail').textContent =
      'The bike controller did not respond.';
  };

  x.send();
}

function command(path) {
  get(path, function(x) {
    if (x.status !== 200) {
      document.getElementById('connection').textContent = 'COMMAND ERROR';
      document.getElementById('detail').textContent =
        'Controller rejected the command: HTTP ' + x.status;
      return;
    }

    setTimeout(poll, 80);
  });
}

function setMode(value) {
  // Immediate feedback while the request round-trips, so a tap never
  // feels like it did nothing even on a slow connection.
  document.getElementById('detail').textContent =
    'Sending ' + value.toUpperCase() + '...';

  command('/api/mode?m=' + encodeURIComponent(value));
}

function setAll(value) {
  command('/api/all?v=' + value);
}

function toggleLed(position) {
  var on = (uiMask & (1 << position)) ? 0 : 1;

  command(
    '/api/led?p=' +
    encodeURIComponent(position) +
    '&v=' +
    encodeURIComponent(on)
  );
}

var brightnessTimer = null;

function setBrightness(value) {
  document.getElementById('brightvalue').textContent = value;

  clearTimeout(brightnessTimer);

  brightnessTimer = setTimeout(function() {
    command('/api/brightness?v=' + encodeURIComponent(value));
  }, 80);
}

function toggleEco() {
  command('/api/eco?v=' + (eco ? 0 : 1));
}

function runSelfTest() {
  document.getElementById('detail').textContent =
    'Running LED self test. Please wait.';

  command('/api/selftest');
}

function runCalibration() {
  if (confirm('Keep the bike completely still. Start calibration?')) {
    document.getElementById('detail').textContent =
      'Calibrating. Keep the bike still.';

    command('/api/calibrate');
  }
}

function paintGrid(kids, mask) {
  for (var position = 0; position < 10; position++) {
    kids[position].classList.toggle(
      'on',
      (mask & (1 << position)) !== 0
    );
  }
}

function modeText(mode) {
  var data = {
    AUTO:
      'Automatic brake, turn, and crash detection control the lights.',
    HAZARD:
      'Both yellow sides are flashing for roadside visibility.',
    MANUAL:
      'Individual LED test override. Returns to AUTO after 2 minutes.',
    FLASH:
      'All LEDs are flashing. Returns to AUTO after 2 minutes.',
    CHASE:
      'Chase pattern running. Returns to AUTO after 2 minutes.',
    LIGHTS_OFF:
      'All LEDs are off. Returns to AUTO after 2 minutes.'
  };

  return data[mode] || '';
}

function formatUptime(totalSeconds) {
  var total = Math.max(0, Math.floor(totalSeconds || 0));
  var h = Math.floor(total / 3600);
  var m = Math.floor((total % 3600) / 60);
  var s = total % 60;

  if (h > 0) return h + 'h ' + m + 'm';
  if (m > 0) return m + 'm ' + s + 's';
  return s + 's';
}

function updateMode(data) {
  var mode = data.mode;
  var crash = data.crash === 1;

  document.getElementById('mode').textContent =
    crash ? 'CRASH SOS' : mode.replace('_', ' ');

  document.getElementById('detail').textContent =
    crash
      ? 'A possible impact was detected, so all lights are flashing SOS. ' +
        'Tap any mode button below to confirm the bike is fine and clear it.'
      : modeText(mode);

  var panel = document.getElementById('modepanel');

  panel.className =
    'panel ' +
    (
      crash
        ? 'alert'
        : mode === 'AUTO'
          ? 'safe'
          : mode === 'HAZARD'
            ? 'alert'
            : 'warn'
    );

  var ids = [
    'auto',
    'hazard',
    'manual',
    'flash',
    'chase',
    'off'
  ];

  for (var i = 0; i < ids.length; i++) {
    document.getElementById(ids[i]).classList.remove('active');
  }

  var active =
    mode === 'LIGHTS_OFF'
      ? 'off'
      : mode.toLowerCase();

  var element = document.getElementById(active);

  if (element) {
    element.classList.add('active');
  }
}

function poll() {
  get('/api/status?t=' + Date.now(), function(x) {
    if (x.status !== 200) {
      document.getElementById('connection').textContent = 'OFFLINE';
      return;
    }

    try {
      var data = JSON.parse(x.responseText);

      uiMask = data.test;
      previewMask = data.preview;
      eco = data.eco === 1;

      paintGrid(box.children, uiMask);
      paintGrid(preview.children, previewMask);
      updateMode(data);

      document.getElementById('connection').textContent = 'ONLINE';

      document.getElementById('decel').textContent =
        Number(data.decel).toFixed(2);

      document.getElementById('pitch').textContent =
        Number(data.pitch).toFixed(1);

      document.getElementById('roll').textContent =
        Number(data.roll).toFixed(1);

      document.getElementById('brake').textContent =
        data.brake ? 'ON' : 'READY';

      document.getElementById('movement').textContent =
        data.moving ? 'MOVING' : 'STILL';

      document.getElementById('uptime').textContent =
        formatUptime(data.uptime);

      var state = [];

      state.push(data.brake ? 'BRAKE ACTIVE' : 'BRAKE READY');

      if (data.left) {
        state.push('LEFT TURN');
      }

      if (data.right) {
        state.push('RIGHT TURN');
      }

      if (data.hazard) {
        state.push('HAZARD ACTIVE');
      }

      if (data.crash) {
        state.push('CRASH SOS');
      }

      var flags = document.getElementById('flags');

      flags.textContent = state.join(' | ');
      flags.className = data.crash ? 'flags bad' : 'flags good';

      var ecoButton = document.getElementById('eco');

      ecoButton.textContent = eco ? 'ON' : 'OFF';
      ecoButton.className = eco ? 'active' : '';
    } catch (error) {
      document.getElementById('connection').textContent = 'DATA ERROR';
      document.getElementById('detail').textContent =
        'Invalid status data received from the controller.';
    }
  });
}

poll();
setInterval(poll, 500);
</script>

</body>
</html>
)HTML";

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  for (int p : leftPins)  pinMode(p, OUTPUT);
  for (int p : rightPins) pinMode(p, OUTPUT);

#if ENABLE_HAZARD_BUTTON
  pinMode(HAZARD_BUTTON_PIN, INPUT_PULLUP);
#endif

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  for (int i = 0; i < 4; i++) ledcAttach(brakePins[i], PWM_FREQ, PWM_RES);
#else
  for (int i = 0; i < 4; i++) { ledcSetup(i, PWM_FREQ, PWM_RES); ledcAttachPin(brakePins[i], i); }
#endif

  allLightsOff();
  startupLEDTest();

  Wire.begin(21, 22);
  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 NOT DETECTED");
    sensorError();
    while (true) delay(100);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(300);

  loadTunables();

  bool haveSavedCal = loadCalibration();
  bool forceRecal = false;
#if ENABLE_HAZARD_BUTTON
  forceRecal = (digitalRead(HAZARD_BUTTON_PIN) == LOW);
#endif

  if (haveSavedCal && !forceRecal) {
    Serial.println("Using saved calibration (quick re-check)...");
    pitchEstimate = pitchBaseline;
    rollEstimate  = rollBaseline;
    runCalibration(30, 0.7f);
  } else {
    Serial.println("No valid saved calibration (or recal forced) - running full calibration.");
    pitchEstimate = 0; rollEstimate = 0;
    runCalibration(120, 0.0f);
    saveCalibration();
  }

  lastOrientationMicros = micros();
  bootMillis = millis();

#if ENABLE_BLE_TELEMETRY
  setupBLE();
#endif

  readyAnimation();

  controlMode = MODE_AUTO;
  modeStartedAt = millis();
  manualPhysicalMask = 0;
  hazardActive = false;

  startController();

  Serial.println("=== SMART BIKE READY (firmware v4.0) ===");
#if ENABLE_SERIAL_TUNING
  Serial.println("Type HELP for the live tuning console.");
#endif
}

// ===================== MAIN LOOP =====================
void loop() {
  unsigned long now = millis();

  web.handleClient();

  // handleClient() may have just called selectMode() (a phone tap),
  // which stamps modeStartedAt with a fresher millis() than the "now"
  // captured above. Re-read it here - otherwise the timeout check below
  // computes now - modeStartedAt on unsigned values where modeStartedAt
  // is now the LARGER number, underflows to a huge value, and instantly
  // (and wrongly) reverts the mode back to AUTO on the very same loop.
  now = millis();

  // Riding modes (AUTO/HAZARD) are the only ones where automatic crash
  // detection should be armed - see the header note on why. The sensor
  // read/filter/orientation/brake/turn pipeline itself always runs, so
  // the phone's live readout never freezes in any mode.
  bool ridingMode = (controlMode == MODE_AUTO || controlMode == MODE_HAZARD);
  updateSensors(now, ridingMode);

#if ENABLE_HAZARD_BUTTON
  checkHazardButton(now);
#endif

  if ((controlMode == MODE_MANUAL ||
       controlMode == MODE_FLASH ||
       controlMode == MODE_CHASE ||
       controlMode == MODE_OFF) &&
      now - modeStartedAt >= TEST_MODE_TIMEOUT_MS) {
    selectMode(MODE_AUTO);
  }

  renderLights(now);

#if ENABLE_SERIAL_TUNING
  handleSerialInput();
#endif
#if ENABLE_BLE_TELEMETRY
  updateBLE(now);
#endif

  static unsigned long logAt = 0;
  if (now - logAt > 250) {
    logAt = now;
    Serial.print("MODE:"); Serial.print(modeName());
    Serial.print(" ACC:"); Serial.print(filtAccel, 2);
    Serial.print(" DEC:"); Serial.print(deceleration, 2);
    Serial.print(" PITCH:"); Serial.print(pitchEstimate * RAD_TO_DEG, 1);
    Serial.print(" ROLL:");  Serial.print(rollEstimate * RAD_TO_DEG, 1);
    Serial.print(isStationary ? " STILL" : " MOVING");
    Serial.print(brakeActive ? " BRAKE" : "");
#if ENABLE_ESS_FLASH
    Serial.print(essActive ? " ESS" : "");
#endif
    Serial.print(leftTurning ? " L" : "");
    Serial.print(rightTurning ? " R" : "");
    Serial.print(hazardActive ? " HAZARD" : "");
#if ENABLE_CRASH_DETECTION
    Serial.print(crashDetected ? " CRASH!" : "");
#endif
#if ENABLE_BATTERY_MONITOR
    Serial.print(" BATT:"); Serial.print(batteryVoltage, 2);
#endif
    Serial.println();
  }
}

// ===================== SENSOR ENGINE =====================
// Reads the MPU6050 once, updates every filter/estimate, and (only when
// doCrashCheck is true) runs crash detection. Never touches an LED pin -
// see renderLights()/renderAutoAndHazard() for all physical output.
void updateSensors(unsigned long now, bool doCrashCheck) {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastOrientationMicros) / 1000000.0f;
  lastOrientationMicros = nowMicros;
  if (dt <= 0 || dt > 0.5f) dt = 0.01f;

#ifdef BRAKE_AXIS_X
  float accRaw = a.acceleration.x;
#else
  float accRaw = a.acceleration.y;
#endif
#ifdef TURN_AXIS_Z
  float gyrRaw = g.gyro.z;
#else
  float gyrRaw = g.gyro.y;
#endif

  // Spike (bump/pothole) rejection before filtering.
  float rawDiff = accRaw - filtAccel;
  if (fabsf(rawDiff) > cfg.spikeClamp) {
    accRaw = filtAccel + copysignf(cfg.spikeClamp, rawDiff);
  }

  filtAccel = filtAccel * (1 - cfg.accelFilter) + accRaw * cfg.accelFilter;
  filtGyroZ = filtGyroZ * (1 - cfg.gyroFilter)  + gyrRaw * cfg.gyroFilter;

  updateOrientation(a, g, dt);

  // Tilt-compensated deceleration (removes hill-slope false triggers).
  float pitchDelta = pitchEstimate - pitchBaseline;
  float gravityCompX = 0;
#if ENABLE_TILT_COMPENSATION
  gravityCompX = GRAVITY_MPS2 * sinf(pitchDelta);
#endif
  float accelFromRest = (filtAccel - accelBaseline) - gravityCompX;
  float turnRate       = filtGyroZ - gyroZBias;
  lastTurnRate = turnRate;

  // Running variance for stillness.
  accelMean = accelMean * 0.95f + accRaw * 0.05f;
  float d = accRaw - accelMean;
  accelVar = accelVar * 0.95f + (d * d) * 0.05f;
  accelStd = sqrtf(accelVar);

  gyroMag = fabsf(g.gyro.x) + fabsf(g.gyro.y) + fabsf(g.gyro.z);

  bool stillNow = (accelStd < cfg.stillJitter) && (gyroMag < cfg.stillGyroMax);
  if (stillNow) {
    if (stillSince == 0) stillSince = now;
    if (now - stillSince > (unsigned long)cfg.stillConfirmMs) isStationary = true;
  } else {
    stillSince = 0;
    isStationary = false;
  }

  if (isStationary) {
    accelBaseline = accelBaseline * 0.90f + filtAccel * 0.10f;
    gyroZBias     = gyroZBias * 0.99f + filtGyroZ * 0.01f;
    pitchBaseline = pitchBaseline * 0.98f + pitchEstimate * 0.02f;
    rollBaseline  = rollBaseline  * 0.98f + rollEstimate  * 0.02f;
  }

  float decelInstant = 0;
  if (BRAKING_IS_NEGATIVE) { if (accelFromRest < 0) decelInstant = -accelFromRest; }
  else                     { if (accelFromRest > 0) decelInstant =  accelFromRest; }
  deceleration = decelInstant;

#if ENABLE_CRASH_DETECTION
  if (doCrashCheck) {
    updateCrashDetection(a, now);
  } else {
    // Not a riding mode right now: never leave a stale/bench-test flag
    // armed where nothing will run to clear it again.
    crashDetected = false;
    impactFlag = false;
    fallenSince = 0;
    crashClearMovingSince = 0;
  }
#endif

  bool braking = (deceleration >= cfg.brakeTrigger) && !isStationary;
  if (braking) {
    if (!brakeActive) {
      brakeActive = true;
      brakeStart = now;
#if ENABLE_ESS_FLASH
      essActive = false;
#endif
    }
    brakeLastDecel = now;
#if ENABLE_ESS_FLASH
    if (!essActive && deceleration >= cfg.essThreshold) { essActive = true; essStart = now; }
#endif
  }
  if (brakeActive) {
    bool expired = (now - brakeLastDecel > (unsigned long)cfg.brakeSustainMs);
    bool ceiling = (now - brakeStart > (unsigned long)cfg.brakeMaxMs);
    if (expired || ceiling || isStationary) {
      brakeActive = false;
#if ENABLE_ESS_FLASH
      essActive = false;
#endif
    }
  }
#if ENABLE_ESS_FLASH
  if (essActive && now - essStart > (unsigned long)cfg.essDurationMs) essActive = false;
#endif

  int target =
#if ENABLE_DRL
    (int)cfg.drlBrightness;
#else
    0;
#endif

  if (brakeActive) {
#if ENABLE_ESS_FLASH
    if (essActive) {
      float period = 1000.0f / fmaxf(cfg.essFlashHz, 1.0f);
      float phase = fmodf((float)(now - essStart), period) / period;
      target = (phase < 0.5f) ? 255 : (int)cfg.minBrakeBright;
    } else
#endif
    {
      if (now - brakeStart < (unsigned long)cfg.brakeFlashMs) {
        target = 255;
      } else {
        float span = fmaxf(cfg.hardBrake - cfg.brakeTrigger, 0.01f);
        float s = (deceleration - cfg.brakeTrigger) / span;
        s = constrain(s, 0.0f, 1.0f);
        target = (int)cfg.minBrakeBright + (int)(s * (255 - (int)cfg.minBrakeBright));
      }
    }
  }

#if ENABLE_AMBIENT_BRIGHTNESS
  updateAmbientBrightness(now);
  target = (int)constrain((float)target * brightnessScale, 0.0f, 255.0f);
#endif

  currentBrightness += (target - currentBrightness) * cfg.brightnessSmooth;
  currentBrightness = constrain(currentBrightness, 0, 255);

  if (turnRate < -cfg.turnThreshold) {
    if (leftCandidateSince == 0) leftCandidateSince = now;
    if (!leftTurning && now - leftCandidateSince >= (unsigned long)cfg.turnConfirmMs) {
      leftTurning = true; rightTurning = false; leftStep = 0; leftAnimAt = now;
    }
    if (leftTurning) leftTurnTime = now;
  } else {
    leftCandidateSince = 0;
  }
  if (turnRate > cfg.turnThreshold) {
    if (rightCandidateSince == 0) rightCandidateSince = now;
    if (!rightTurning && now - rightCandidateSince >= (unsigned long)cfg.turnConfirmMs) {
      rightTurning = true; leftTurning = false; rightStep = 0; rightAnimAt = now;
    }
    if (rightTurning) rightTurnTime = now;
  } else {
    rightCandidateSince = 0;
  }
  if (leftTurning  && now - leftTurnTime  > (unsigned long)cfg.turnHoldMs) leftTurning  = false;
  if (rightTurning && now - rightTurnTime > (unsigned long)cfg.turnHoldMs) rightTurning = false;

  if (!brakeActive && !isStationary) {
    accelBaseline = accelBaseline * (1 - cfg.baselineAdapt) + filtAccel * cfg.baselineAdapt;
  }

#if ENABLE_BATTERY_MONITOR
  updateBatteryMonitor(now);
#endif
}

// ===================== OUTPUT (single writer for every LED) =====================
// This is the only place in the whole sketch that decides what the
// physical LEDs show. Crash SOS beats everything; otherwise the current
// mode owns the outputs completely, with no other code path able to
// overwrite it later in the same loop.
void renderLights(unsigned long now) {
#if ENABLE_CRASH_DETECTION
  if (crashDetected) {
    renderCrashBeacon(now);
    return;
  }
#endif

  switch (controlMode) {
    case MODE_AUTO:
    case MODE_HAZARD:
      renderAutoAndHazard(now);
      break;

    case MODE_MANUAL:
    case MODE_FLASH:
    case MODE_CHASE:
    case MODE_OFF:
    default:
      writeManualPhysicalMask(currentPhysicalMask(now));
      break;
  }
}

// Automatic brake/turn/hazard output, used only while controlMode is
// AUTO or HAZARD.
void renderAutoAndHazard(unsigned long now) {
  int outputBrightness = (int)currentBrightness;

#if ENABLE_BATTERY_MONITOR
  if (lowBatteryFlag && !brakeActive) {
    unsigned long cyclePos = now % 10000UL;
    if ((cyclePos < 120) || (cyclePos > 250 && cyclePos < 370)) outputBrightness = 255;
  }
#endif

  if (controlMode == MODE_HAZARD) {
    unsigned long hazPeriod = 500;
    bool onPhase = ((now / hazPeriod) % 2) == 0;
    if (onPhase) { leftLights(); rightLights(); } else { leftOff(); rightOff(); }
  } else {
    if (leftTurning)  animateTurn(leftPins,  leftStep,  leftAnimAt,  lastTurnRate);
    else              leftOff();
    if (rightTurning) animateTurn(rightPins, rightStep, rightAnimAt, lastTurnRate);
    else              rightOff();
  }

  setBrakeBrightness(outputBrightness);
}

// ===================== ORIENTATION =====================
// Complementary filter: fuses the accelerometer's gravity-vector angle
// (accurate long-term, noisy short-term) with the gyro's integrated rate
// (accurate short-term, drifts long-term) into a stable pitch/roll estimate.
void updateOrientation(const sensors_event_t &a, const sensors_event_t &g, float dt) {
  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az));
  float rollAcc  = atan2f(ay, az);

  float gyroPitchRate = g.gyro.y;
  float gyroRollRate  = g.gyro.x;

  pitchEstimate = cfg.compAlpha * (pitchEstimate + gyroPitchRate * dt) + (1.0f - cfg.compAlpha) * pitchAcc;
  rollEstimate  = cfg.compAlpha * (rollEstimate  + gyroRollRate  * dt) + (1.0f - cfg.compAlpha) * rollAcc;
}

// ===================== CALIBRATION =====================
void runCalibration(int samples, float blendExisting) {
  Serial.println("CALIBRATING - keep still");
  sensors_event_t a, g, t;
  for (int i = 0; i < 6; i++) {
    if (i % 2 == 0) { leftLights(); rightLights(); } else { brakeLights(); }
    delay(250); allLightsOff();
  }

  float sa = 0, sg = 0;
  unsigned long prevMicros = micros();
  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&a, &g, &t);
    unsigned long nowMicros = micros();
    float dt = (nowMicros - prevMicros) / 1000000.0f;
    prevMicros = nowMicros;
    if (dt <= 0 || dt > 0.5f) dt = 0.01f;
    updateOrientation(a, g, dt);
#ifdef BRAKE_AXIS_X
    sa += a.acceleration.x;
#else
    sa += a.acceleration.y;
#endif
#ifdef TURN_AXIS_Z
    sg += g.gyro.z;
#else
    sg += g.gyro.y;
#endif
    delay(8);
  }
  float newAccelBase = sa / samples;
  float newGyroBias  = sg / samples;

  accelBaseline = accelBaseline * blendExisting + newAccelBase * (1.0f - blendExisting);
  gyroZBias     = gyroZBias     * blendExisting + newGyroBias  * (1.0f - blendExisting);
  pitchBaseline = pitchBaseline * blendExisting + pitchEstimate * (1.0f - blendExisting);
  rollBaseline  = rollBaseline  * blendExisting + rollEstimate  * (1.0f - blendExisting);

  filtAccel = accelBaseline; filtGyroZ = gyroZBias; accelMean = accelBaseline;
  Serial.print("Accel base: "); Serial.println(accelBaseline, 3);
}

// ===================== PERSISTENT STORAGE =====================
void saveTunables() {
  prefs.begin("bikecfg", false);
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
}
void loadTunables() {
  prefs.begin("bikecfg", true);
  if (prefs.isKey("cfg") && prefs.getBytesLength("cfg") == sizeof(cfg)) {
    prefs.getBytes("cfg", &cfg, sizeof(cfg));
  }
  prefs.end();
}
void saveCalibration() {
  calData.accelBaseline = accelBaseline;
  calData.gyroZBias     = gyroZBias;
  calData.pitchBaseline = pitchBaseline;
  calData.rollBaseline  = rollBaseline;
  calData.valid = true;
  prefs.begin("bikecfg", false);
  prefs.putBytes("cal", &calData, sizeof(calData));
  prefs.end();
}
bool loadCalibration() {
  bool ok = false;
  prefs.begin("bikecfg", true);
  if (prefs.isKey("cal") && prefs.getBytesLength("cal") == sizeof(calData)) {
    prefs.getBytes("cal", &calData, sizeof(calData));
    if (calData.valid) {
      accelBaseline = calData.accelBaseline;
      gyroZBias     = calData.gyroZBias;
      pitchBaseline = calData.pitchBaseline;
      rollBaseline  = calData.rollBaseline;
      ok = true;
    }
  }
  prefs.end();
  return ok;
}

// ===================== CRASH / IMPACT DETECTION =====================
#if ENABLE_CRASH_DETECTION
void updateCrashDetection(const sensors_event_t &a, unsigned long now) {
  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  float excessG = fabsf(mag - GRAVITY_MPS2) / GRAVITY_MPS2;

  if (!crashDetected && excessG > cfg.impactThresholdG) {
    impactFlag = true;
    impactAt = now;
  }

  bool fallen = fabsf(rollEstimate) > (cfg.fallenAngleDeg * DEG_TO_RAD);
  if (fallen) { if (fallenSince == 0) fallenSince = now; }
  else fallenSince = 0;
  bool fallenConfirmed = fallen && fallenSince != 0 && (now - fallenSince > (unsigned long)cfg.fallenConfirmMs);

  if (!crashDetected) {
    bool impactThenStill = impactFlag && (now - impactAt < 8000UL) &&
                            isStationary && stillSince != 0 &&
                            (now - stillSince > (unsigned long)cfg.crashStillMs);
    if (impactThenStill || fallenConfirmed) {
      crashDetected = true;
      crashClearMovingSince = 0;
    }
    if (impactFlag && now - impactAt > 8000UL) impactFlag = false;
  } else {
    bool activelyMoving = !isStationary &&
                           (accelStd > cfg.stillJitter * 2.0f || gyroMag > cfg.stillGyroMax * 3.0f);
    if (activelyMoving) { if (crashClearMovingSince == 0) crashClearMovingSince = now; }
    else crashClearMovingSince = 0;
    if (crashClearMovingSince != 0 && now - crashClearMovingSince > 1500UL) {
      crashDetected = false;
      impactFlag = false;
    }
  }
}

// Non-blocking SOS pattern (3 short / 3 long / 3 short) across ALL lights.
void renderCrashBeacon(unsigned long now) {
  const int PATTERN_SHORT = 150, PATTERN_LONG = 400, PATTERN_GAP = 150, WORD_GAP = 900;
  const int SEQ_LEN = 9;
  const int seq[SEQ_LEN] = {
    PATTERN_SHORT, PATTERN_SHORT, PATTERN_SHORT,
    PATTERN_LONG,  PATTERN_LONG,  PATTERN_LONG,
    PATTERN_SHORT, PATTERN_SHORT, PATTERN_SHORT
  };
  static int idx = 0;
  static unsigned long stepStart = 0;
  static bool on = false;

  if (stepStart == 0) { stepStart = now; on = true; allLightsOn(); }

  unsigned long elapsed = now - stepStart;
  if (on) {
    if (elapsed >= (unsigned long)seq[idx]) { on = false; stepStart = now; allLightsOff(); }
  } else {
    unsigned long gap = (idx == SEQ_LEN - 1) ? WORD_GAP : PATTERN_GAP;
    if (elapsed >= gap) {
      idx = (idx + 1) % SEQ_LEN;
      on = true; stepStart = now; allLightsOn();
    }
  }
}
#endif

// ===================== AMBIENT BRIGHTNESS (optional) =====================
#if ENABLE_AMBIENT_BRIGHTNESS
void updateAmbientBrightness(unsigned long now) {
  if (now - lastLdrRead < 250) return;
  lastLdrRead = now;
  int raw = analogRead(LDR_ADC_PIN);
  float ambient = raw / 4095.0f;
  brightnessScale = 0.35f + ambient * 0.65f;
}
#endif

// ===================== BATTERY MONITOR (optional) =====================
#if ENABLE_BATTERY_MONITOR
void updateBatteryMonitor(unsigned long now) {
  if (now - lastBattRead < 5000) return;
  lastBattRead = now;
  int raw = analogRead(BATTERY_ADC_PIN);
  batteryVoltage = (raw / 4095.0f) * ADC_REF_VOLTAGE * BATTERY_DIVIDER_RATIO;
  lowBatteryFlag = batteryVoltage < cfg.lowBatteryVolts;
}
#endif

// ===================== HAZARD BUTTON (optional) =====================
#if ENABLE_HAZARD_BUTTON
void checkHazardButton(unsigned long now) {
  static int lastButtonState = HIGH;
  static unsigned long lastToggleAt = 0;
  int btn = digitalRead(HAZARD_BUTTON_PIN);
  if (lastButtonState == HIGH && btn == LOW && now - lastToggleAt > 250) {
    selectMode(controlMode == MODE_HAZARD ? MODE_AUTO : MODE_HAZARD);
    lastToggleAt = now;
  }
  lastButtonState = btn;
}
#endif

// ===================== SERIAL TUNING CONSOLE =====================
#if ENABLE_SERIAL_TUNING
void handleSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) { processCommand(serialBuffer); serialBuffer = ""; }
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 64) serialBuffer = "";
    }
  }
}

void processCommand(String line) {
  line.trim();
  line.toUpperCase();
  if (line == "HELP")     { printHelp(); return; }
  if (line == "STATUS")   { printStatus(); return; }
  if (line == "SAVE")     { saveTunables(); Serial.println("OK SAVED"); return; }
  if (line == "LOAD")     { loadTunables(); Serial.println("OK LOADED"); return; }
  if (line == "DEFAULTS") { cfg = Tunables(); Serial.println("OK DEFAULTS RESTORED"); return; }
  if (line == "CAL")      { runCalibration(120, 0.0f); saveCalibration(); Serial.println("OK RECALIBRATED"); return; }
  if (line == "HAZARD")   {
    selectMode(controlMode == MODE_HAZARD ? MODE_AUTO : MODE_HAZARD);
    Serial.println(controlMode == MODE_HAZARD ? "HAZARD ON" : "HAZARD OFF");
    return;
  }

  if (line.startsWith("MODE ")) {
    String m = line.substring(5);
    m.trim();
    if      (m == "AUTO")    { selectMode(MODE_AUTO);    Serial.println("OK MODE AUTO"); }
    else if (m == "HAZARD")  { selectMode(MODE_HAZARD);  Serial.println("OK MODE HAZARD"); }
    else if (m == "MANUAL")  { selectMode(MODE_MANUAL);  Serial.println("OK MODE MANUAL"); }
    else if (m == "FLASH")   { selectMode(MODE_FLASH);   Serial.println("OK MODE FLASH"); }
    else if (m == "CHASE")   { selectMode(MODE_CHASE);   Serial.println("OK MODE CHASE"); }
    else if (m == "OFF")     { selectMode(MODE_OFF);     Serial.println("OK MODE OFF"); }
    else Serial.println("ERR UNKNOWN MODE (AUTO/HAZARD/MANUAL/FLASH/CHASE/OFF)");
    return;
  }

  if (line.startsWith("SET ")) {
    String rest = line.substring(4);
    int sp2 = rest.indexOf(' ');
    if (sp2 > 0) {
      String name = rest.substring(0, sp2);
      float val = rest.substring(sp2 + 1).toFloat();
      bool found = false;
      for (int i = 0; i < NUM_TUNABLES; i++) {
        if (name == tunableTable[i].name) { *tunableTable[i].ptr = val; found = true; break; }
      }
      Serial.println(found ? "OK SET" : "ERR UNKNOWN PARAM (see STATUS for names)");
    } else {
      Serial.println("ERR USAGE: SET <PARAM> <VALUE>");
    }
    return;
  }
  Serial.println("ERR UNKNOWN COMMAND (type HELP)");
}

void printHelp() {
  Serial.println("--- Smart Bike Tuning Console ---");
  Serial.println("STATUS            - show live sensor + config snapshot");
  Serial.println("MODE <M>          - AUTO/HAZARD/MANUAL/FLASH/CHASE/OFF, same as the phone");
  Serial.println("SET <PARAM> <VAL> - change a tunable (see STATUS for names)");
  Serial.println("SAVE / LOAD       - persist / reload tunables from flash");
  Serial.println("DEFAULTS          - reset tunables to firmware defaults");
  Serial.println("CAL               - force a fresh calibration (keep still)");
  Serial.println("HAZARD            - toggle hazard lights");
}

void printStatus() {
  Serial.println("--- CONFIG ---");
  for (int i = 0; i < NUM_TUNABLES; i++) {
    Serial.print(tunableTable[i].name); Serial.print(" = "); Serial.println(*tunableTable[i].ptr, 4);
  }
  Serial.println("--- LIVE ---");
  Serial.print("MODE="); Serial.print(modeName());
  Serial.print(" DEC="); Serial.print(deceleration, 2);
  Serial.print(" PITCH="); Serial.print(pitchEstimate * RAD_TO_DEG, 1);
  Serial.print(" ROLL="); Serial.print(rollEstimate * RAD_TO_DEG, 1);
  Serial.print(isStationary ? " STILL" : " MOVING");
#if ENABLE_CRASH_DETECTION
  Serial.print(crashDetected ? " CRASH!" : "");
#endif
  Serial.println();
}
#endif

// ===================== BLE TELEMETRY (optional) =====================
#if ENABLE_BLE_TELEMETRY
void setupBLE() {
  BLEDevice::init("SmartBike");
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(BLE_SERVICE_UUID);
  bleChar = service->createCharacteristic(
    BLE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  bleChar->addDescriptor(new BLE2902());
  service->start();
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->start();
  Serial.println("BLE telemetry advertising as 'SmartBike'");
}

void updateBLE(unsigned long now) {
  if (!bleChar) return;
  if (now - lastBleUpdate < 1000) return;
  lastBleUpdate = now;
  char buf[96];
  int crashFlagInt = 0;
#if ENABLE_CRASH_DETECTION
  crashFlagInt = crashDetected ? 1 : 0;
#endif
  snprintf(buf, sizeof(buf), "D:%.2f P:%.1f R:%.1f M:%d H:%d C:%d",
           deceleration, pitchEstimate * RAD_TO_DEG, rollEstimate * RAD_TO_DEG,
           isStationary ? 0 : 1, hazardActive ? 1 : 0, crashFlagInt);
  bleChar->setValue((uint8_t*)buf, strlen(buf));
  bleChar->notify();
}
#endif

// ===================== TURN ANIMATION =====================
void animateTurn(const int pins[3], int &step, unsigned long &animAt, float turnRateValue) {
  float mag = constrain(fabsf(turnRateValue), 1.0f, 3.0f);
  float f = (mag - 1.0f) / 2.0f;
  unsigned long speed = (unsigned long)cfg.turnAnimSlowMs -
                        (unsigned long)(f * (cfg.turnAnimSlowMs - cfg.turnAnimFastMs));

  if (millis() - animAt < speed) return;
  animAt = millis();

  for (int i = 0; i < 3; i++) digitalWrite(pins[i], LOW);
  for (int i = 0; i <= step && i < 3; i++) digitalWrite(pins[i], HIGH);

  step++;
  if (step > 2) step = 0;
}

// ===================== ANIMATIONS / HELPERS =====================
void startupLEDTest() {
  leftLights();  delay(300); leftOff();
  brakeLights(); delay(300); brakeOff();
  rightLights(); delay(300); rightOff();
  allLightsOn(); delay(250); allLightsOff(); delay(150);
  allLightsOn(); delay(250); allLightsOff(); delay(300);
}

void sensorError() {
  while (true) {
    for (int i = 0; i < 3; i++) {
      leftLights(); rightLights(); delay(200);
      leftOff(); rightOff(); delay(200);
    }
    delay(700);
  }
}

void readyAnimation() {
  for (int p : leftPins)  { digitalWrite(p, HIGH); delay(80); }
  setBrakeBrightness(255); delay(320);
  for (int p : rightPins) { digitalWrite(p, HIGH); delay(80); }
  allLightsOn(); delay(300); allLightsOff();
}

void leftLights()  { for (int p : leftPins)  digitalWrite(p, HIGH); }
void leftOff()     { for (int p : leftPins)  digitalWrite(p, LOW); }
void rightLights() { for (int p : rightPins) digitalWrite(p, HIGH); }
void rightOff()    { for (int p : rightPins) digitalWrite(p, LOW); }

void brakeLights() { setBrakeBrightness(255); }
void brakeOff()    { currentBrightness = 0; setBrakeBrightness(0); }

void setBrakeBrightness(int b) {
  b = constrain(b, 0, 255);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  for (int i = 0; i < 4; i++) ledcWrite(brakePins[i], b);
#else
  for (int i = 0; i < 4; i++) ledcWrite(i, b);
#endif
}

void allLightsOn()  { leftLights(); brakeLights(); rightLights(); }
void allLightsOff() { leftOff(); brakeOff(); rightOff(); }

// ===================== PHONE CONTROLLER =====================
void startController() {
  WiFi.mode(WIFI_AP);

  bool started = WiFi.softAP(
    CONTROL_AP_NAME,
    CONTROL_AP_PASSWORD
  );

  web.on("/", HTTP_GET, handleRoot);
  web.on("/api/status", HTTP_GET, handleStatus);
  web.on("/api/mode", HTTP_GET, handleMode);
  web.on("/api/led", HTTP_GET, handleLed);
  web.on("/api/all", HTTP_GET, handleAll);
  web.on("/api/brightness", HTTP_GET, handleBrightness);
  web.on("/api/eco", HTTP_GET, handleEco);
  web.on("/api/selftest", HTTP_GET, handleSelfTest);
  web.on("/api/calibrate", HTTP_GET, handleCalibration);
  web.on("/api/command", HTTP_GET, handleLegacyCommand);

  web.onNotFound([]() {
    web.sendHeader("Cache-Control", "no-store");
    web.send(404, "text/plain", "Not found");
  });

  web.begin();

  Serial.println(
    started
      ? "Phone controller ready"
      : "ERROR: Wi-Fi hotspot failed"
  );

  Serial.print("Network: ");
  Serial.println(CONTROL_AP_NAME);

  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());
}

void handleRoot() {
  web.sendHeader(
    "Cache-Control",
    "no-store, no-cache, must-revalidate, max-age=0"
  );
  web.sendHeader("Pragma", "no-cache");
  web.sendHeader("Expires", "0");
  web.send_P(200, "text/html", CONTROL_PAGE);
}

void handleStatus() {
  char json[480];
  int crash = 0;

#if ENABLE_CRASH_DETECTION
  crash = crashDetected ? 1 : 0;
#endif

  unsigned long now = millis();

  uint16_t testMask =
    physicalToUiMask(manualPhysicalMask);

  uint16_t previewMask =
    physicalToUiMask(activePhysicalMask(now));

  snprintf(
    json,
    sizeof(json),
    "{"
      "\"mode\":\"%s\","
      "\"mask\":%u,"
      "\"test\":%u,"
      "\"preview\":%u,"
      "\"decel\":%.2f,"
      "\"pitch\":%.1f,"
      "\"roll\":%.1f,"
      "\"brake\":%d,"
      "\"left\":%d,"
      "\"right\":%d,"
      "\"hazard\":%d,"
      "\"crash\":%d,"
      "\"moving\":%d,"
      "\"eco\":%d,"
      "\"uptime\":%lu"
    "}",
    modeName(),
    testMask,
    testMask,
    previewMask,
    deceleration,
    pitchEstimate * RAD_TO_DEG,
    rollEstimate * RAD_TO_DEG,
    brakeActive ? 1 : 0,
    leftTurning ? 1 : 0,
    rightTurning ? 1 : 0,
    hazardActive ? 1 : 0,
    crash,
    isStationary ? 0 : 1,
    ecoRadioMode ? 1 : 0,
    (unsigned long)((now - bootMillis) / 1000UL)
  );

  web.sendHeader("Cache-Control", "no-store");
  web.send(200, "application/json", json);
}

void handleMode() {
  if (!web.hasArg("m")) {
    web.send(400, "text/plain", "missing mode");
    return;
  }

  String mode = web.arg("m");
  mode.toUpperCase();

  if (mode == "AUTO") {
    selectMode(MODE_AUTO);
  } else if (mode == "HAZARD") {
    selectMode(MODE_HAZARD);
  } else if (mode == "MANUAL") {
    selectMode(MODE_MANUAL);
  } else if (mode == "FLASH") {
    selectMode(MODE_FLASH);
  } else if (mode == "CHASE") {
    selectMode(MODE_CHASE);
  } else if (mode == "OFF") {
    selectMode(MODE_OFF);
  } else {
    web.send(400, "text/plain", "unknown mode");
    return;
  }

  sendOk();
}

void handleLed() {
  if (!web.hasArg("p") || !web.hasArg("v")) {
    web.send(400, "text/plain", "missing LED value");
    return;
  }

  int position = web.arg("p").toInt();
  int on = web.arg("v").toInt();

  if (position < 0 ||
      position > 9 ||
      (on != 0 && on != 1)) {
    web.send(400, "text/plain", "invalid LED value");
    return;
  }

  uint16_t bit =
    1U << UI_TO_PHYSICAL[position];

  if (on) {
    manualPhysicalMask |= bit;
  } else {
    manualPhysicalMask &= ~bit;
  }

  selectMode(MODE_MANUAL);
  sendOk();
}

void handleAll() {
  if (!web.hasArg("v")) {
    web.send(400, "text/plain", "missing all value");
    return;
  }

  int value = web.arg("v").toInt();

  if (value != 0 && value != 1) {
    web.send(400, "text/plain", "invalid all value");
    return;
  }

  manualPhysicalMask = value ? 0x03FF : 0;

  selectMode(MODE_MANUAL);
  sendOk();
}

void handleBrightness() {
  if (!web.hasArg("v")) {
    web.send(400, "text/plain", "missing brightness");
    return;
  }

  int value = web.arg("v").toInt();

  if (value < 0) {
    value = 0;
  }

  if (value > 255) {
    value = 255;
  }

  redBrightness = static_cast<uint8_t>(value);

  if (controlMode == MODE_MANUAL ||
      controlMode == MODE_FLASH ||
      controlMode == MODE_CHASE ||
      controlMode == MODE_OFF) {
    writeManualPhysicalMask(currentPhysicalMask(millis()));
  }

  sendOk();
}

void handleEco() {
  if (!web.hasArg("v")) {
    web.send(400, "text/plain", "missing eco value");
    return;
  }

  int value = web.arg("v").toInt();

  if (value != 0 && value != 1) {
    web.send(400, "text/plain", "invalid eco value");
    return;
  }

  setEcoRadio(value != 0);
  sendOk();
}

void handleSelfTest() {
  web.sendHeader("Cache-Control", "no-store");
  web.send(200, "text/plain", "OK");

  selectMode(MODE_OFF);
  allLightsOff();

  delay(100);

  startupLEDTest();
  allLightsOff();

  selectMode(MODE_AUTO);
}

void handleCalibration() {
  web.sendHeader("Cache-Control", "no-store");
  web.send(200, "text/plain", "OK");

  selectMode(MODE_OFF);
  allLightsOff();

  runCalibration(120, 0.0f);
  saveCalibration();

  selectMode(MODE_AUTO);
}

// Compatibility with older cached Code 3 pages, and a handy way to test
// any mode straight from a browser address bar or curl, e.g.:
//   http://192.168.4.1/api/command?value=MANUAL
void handleLegacyCommand() {
  if (!web.hasArg("value")) {
    web.send(400, "text/plain", "missing command");
    return;
  }

  String command = web.arg("value");
  command.toUpperCase();

  if (command == "AUTO") {
    selectMode(MODE_AUTO);
  } else if (command == "HAZARD") {
    selectMode(MODE_HAZARD);
  } else if (command == "MANUAL") {
    selectMode(MODE_MANUAL);
  } else if (command == "FLASH") {
    selectMode(MODE_FLASH);
  } else if (command == "CHASE") {
    selectMode(MODE_CHASE);
  } else if (command == "OFF") {
    selectMode(MODE_OFF);
  } else if (command == "ALL") {
    manualPhysicalMask = 0x03FF;
    selectMode(MODE_MANUAL);
  } else if (command == "NONE") {
    manualPhysicalMask = 0;
    selectMode(MODE_MANUAL);
  } else if (command.startsWith("BRI=")) {
    redBrightness = static_cast<uint8_t>(
      constrain(command.substring(4).toInt(), 0, 255)
    );
  } else if (command.startsWith("LED")) {
    int equal = command.indexOf('=');

    if (equal <= 3) {
      web.send(400, "text/plain", "invalid LED");
      return;
    }

    int position =
      command.substring(3, equal).toInt();

    int on =
      command.substring(equal + 1).toInt();

    if (position < 0 ||
        position > 9 ||
        (on != 0 && on != 1)) {
      web.send(400, "text/plain", "invalid LED");
      return;
    }

    uint16_t bit =
      1U << UI_TO_PHYSICAL[position];

    if (on) {
      manualPhysicalMask |= bit;
    } else {
      manualPhysicalMask &= ~bit;
    }

    selectMode(MODE_MANUAL);
  } else {
    web.send(400, "text/plain", "unknown command");
    return;
  }

  sendOk();
}

void sendOk() {
  web.sendHeader("Cache-Control", "no-store");
  web.send(200, "text/plain", "OK");
}

// Switches modes. Every phone/serial/button mode change funnels through
// here, so this is the one place that has to keep the mode, the hazard
// flag, and the crash latch consistent with each other.
void selectMode(ControlMode mode) {
  controlMode = mode;
  modeStartedAt = millis();
  hazardActive = (mode == MODE_HAZARD);

#if ENABLE_CRASH_DETECTION
  // A deliberate mode choice - from the phone, Serial, or the optional
  // hazard button - is proof the rider is conscious and in control, the
  // same way a car lets you cancel a false SOS call with a button press.
  // Real crash detection is untouched: it only runs in AUTO/HAZARD (see
  // loop()) and will re-arm and re-trigger immediately on a genuine
  // impact next time the bike is actually ridden.
  crashDetected = false;
  impactFlag = false;
  fallenSince = 0;
  crashClearMovingSince = 0;
#endif

  // Clear whatever the previous mode was showing; renderLights() redraws
  // the correct pattern for the new mode on the very next loop.
  allLightsOff();
}

uint16_t currentPhysicalMask(unsigned long now) {
  switch (controlMode) {
    case MODE_AUTO:
      return 0;

    case MODE_HAZARD:
      // Left and right yellow LEDs: physical bits 0,1,2 and 7,8,9.
      return ((now / 500UL) % 2UL)
        ? 0
        : 0x0387;

    case MODE_MANUAL:
      return manualPhysicalMask;

    case MODE_FLASH:
      return (((now - modeStartedAt) / 180UL) % 2UL)
        ? 0
        : 0x03FF;

    case MODE_CHASE: {
      uint8_t position =
        ((now - modeStartedAt) / 120UL) % 10UL;

      return 1U << UI_TO_PHYSICAL[position];
    }

    case MODE_OFF:
    default:
      return 0;
  }
}

uint16_t physicalToUiMask(uint16_t raw) {
  uint16_t ui = 0;

  for (int position = 0; position < 10; position++) {
    if (raw & (1U << UI_TO_PHYSICAL[position])) {
      ui |= 1U << position;
    }
  }

  return ui;
}

uint16_t activePhysicalMask(unsigned long now) {
#if ENABLE_CRASH_DETECTION
  if (crashDetected) {
    return 0x03FF;
  }
#endif

  if (controlMode != MODE_AUTO && controlMode != MODE_HAZARD) {
    return currentPhysicalMask(now);
  }

  if (controlMode == MODE_HAZARD) {
    // Mirror the same on/off phase renderAutoAndHazard() just drew.
    uint16_t raw = ((now / 500UL) % 2UL) ? 0 : 0x0387;
    if (currentBrightness > 1) raw |= 0x0078;
    return raw;
  }

  uint16_t raw = 0;

  // Brake LEDs are included only when the auto brake output is actually on.
  if (currentBrightness > 1) {
    raw |= 0x0078;
  }

  if (digitalRead(LEFT1)) {
    raw |= 1U << 0;
  }

  if (digitalRead(LEFT2)) {
    raw |= 1U << 1;
  }

  if (digitalRead(LEFT3)) {
    raw |= 1U << 2;
  }

  if (digitalRead(RIGHT1)) {
    raw |= 1U << 7;
  }

  if (digitalRead(RIGHT2)) {
    raw |= 1U << 8;
  }

  if (digitalRead(RIGHT3)) {
    raw |= 1U << 9;
  }

  return raw;
}

void writeManualPhysicalMask(uint16_t raw) {
  raw &= 0x03FF;

  digitalWrite(
    LEFT1,
    (raw & (1U << 0)) ? HIGH : LOW
  );

  digitalWrite(
    LEFT2,
    (raw & (1U << 1)) ? HIGH : LOW
  );

  digitalWrite(
    LEFT3,
    (raw & (1U << 2)) ? HIGH : LOW
  );

  digitalWrite(
    RIGHT1,
    (raw & (1U << 7)) ? HIGH : LOW
  );

  digitalWrite(
    RIGHT2,
    (raw & (1U << 8)) ? HIGH : LOW
  );

  digitalWrite(
    RIGHT3,
    (raw & (1U << 9)) ? HIGH : LOW
  );

  for (int i = 0; i < 4; i++) {
    int value =
      (raw & (1U << (i + 3)))
        ? redBrightness
        : 0;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(brakePins[i], value);
#else
    ledcWrite(i, value);
#endif
  }
}

void setEcoRadio(bool enabled) {
  ecoRadioMode = enabled;

  WiFi.setTxPower(
    enabled
      ? WIFI_POWER_2dBm
      : WIFI_POWER_19_5dBm
  );
}

const char *modeName() {
  switch (controlMode) {
    case MODE_AUTO:
      return "AUTO";

    case MODE_HAZARD:
      return "HAZARD";

    case MODE_MANUAL:
      return "MANUAL";

    case MODE_FLASH:
      return "FLASH";

    case MODE_CHASE:
      return "CHASE";

    case MODE_OFF:
    default:
      return "LIGHTS_OFF";
  }
}
