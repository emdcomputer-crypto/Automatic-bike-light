/*
  SMART BIKE - FIRMWARE v4.0 (Code_3, self-contained)
  Adapted for Seeed Studio XIAO ESP32-S3 (Mini S3) & Shared Brake Pins (3 & 4)
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>

// ===================== FEATURE TOGGLES =====================
#define ENABLE_HAZARD_BUTTON      0
#define ENABLE_BATTERY_MONITOR    0
#define ENABLE_AMBIENT_BRIGHTNESS 0
#define ENABLE_BLE_TELEMETRY      0

#define ENABLE_TILT_COMPENSATION  1
#define ENABLE_CRASH_DETECTION    1
#define ENABLE_ESS_FLASH          1
#define ENABLE_DRL                0
#define ENABLE_SERIAL_TUNING      1

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29578f
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292f
#endif

Adafruit_MPU6050 mpu;
Preferences prefs;

// ===================== LED PINS (XIAO ESP32-S3 D0-D8 Mapping) =====================
const int LEFT1 = D0, LEFT2 = D1, LEFT3 = D2;
const int BRAKE1 = D3, BRAKE2 = D4, BRAKE3 = D5; // BRAKE3 drives shared brake lights 3 & 4
const int RIGHT1 = D6, RIGHT2 = D7, RIGHT3 = D8;

const int brakePins[3] = { BRAKE1, BRAKE2, BRAKE3 };
const int leftPins[3]  = { LEFT3,  LEFT2,  LEFT1  };
const int rightPins[3] = { RIGHT1, RIGHT2, RIGHT3 };

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
uint16_t manualPhysicalMask = 0;
uint8_t redBrightness = 255;
unsigned long modeStartedAt = 0;
bool ecoRadioMode = false;

const unsigned long TEST_MODE_TIMEOUT_MS = 120000UL;

const uint8_t UI_TO_PHYSICAL[10] = {
  9, 8, 7,
  3, 4, 5, 6,
  2, 1, 0
};

// ===================== FORWARD DECLARATIONS =====================
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
#if ENABLE_SERIAL_TUNING
void handleSerialInput();
void processCommand(String line);
void printHelp();
void printStatus();
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
* { box-sizing: border-box; }
body { margin: 0; background: #070a10; color: #f5f7fb; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial, sans-serif; }
main { max-width: 530px; margin: auto; padding: 16px 12px 30px; }
.top { display: flex; align-items: center; justify-content: space-between; margin: 4px 4px 18px; }
.brand { font-size: 22px; font-weight: 800; letter-spacing: 2px; }
.brand small { display: block; font-size: 10px; letter-spacing: 1px; color: #91a0b9; margin-top: 4px; }
.connection { font-size: 11px; font-weight: bold; letter-spacing: 1px; border-radius: 20px; padding: 8px 10px; background: #1c3144; color: #8bd4ff; }
.panel { background: #121926; border: 1px solid #293750; border-radius: 16px; padding: 15px; margin: 12px 0; }
.panel.safe { border-color: #28765f; }
.panel.warn { border-color: #996920; }
.panel.alert { border-color: #a33a4e; }
.kicker { font-size: 11px; font-weight: 600; color: #9baac2; }
.mode { font-size: 27px; font-weight: bold; margin: 5px 0; }
.detail { min-height: 18px; color: #bac6d9; font-size: 13px; line-height: 1.35; }
.buttons { display: grid; grid-template-columns: repeat(2, 1fr); gap: 9px; margin-top: 14px; }
button { min-height: 46px; border: 1px solid #3b4c68; border-radius: 11px; background: #202c40; color: #f4f7ff; font-size: 14px; font-weight: bold; padding: 10px 8px; }
button.active { background: #1477b8; border-color: #67c4f6; }
button.hazard { background: #4d3518; border-color: #976a20; }
button.off { background: #4a2330; border-color: #914254; }
.titleline { display: flex; justify-content: space-between; align-items: end; gap: 10px; }
.titleline h2 { font-size: 17px; margin: 4px 0 0; }
.small { font-size: 11px; color: #9caac2; }
.mapnote { text-align: center; color: #aab7ca; font-size: 11px; margin: 14px 0 8px; }
.leds { display: grid; grid-template-columns: repeat(10, minmax(0, 1fr)); gap: 4px; }
.led { height: 49px; min-height: 49px; padding: 3px 0 0; background: #1d2636; border: 1px solid #40506b; border-radius: 11px; color: #c0ccde; font-size: 10px; font-weight: bold; display: flex; align-items: center; justify-content: center; }
button.led { cursor: pointer; }
.led.gap1 { margin-left: 7px; }
.led.gap2 { margin-left: 7px; }
.led.yellow.on { background: #a87508; border-color: #ffd052; color: #fff; box-shadow: 0 0 12px #c98b08; }
.led.red.on { background: #ae2943; border-color: #ff7890; color: #fff; box-shadow: 0 0 12px #d82e4e; }
.twobuttons { display: grid; grid-template-columns: 1fr 1fr; gap: 9px; margin-top: 13px; }
.slider { margin-top: 15px; }
.slider label { display: block; font-size: 13px; color: #c6d0e0; margin-bottom: 8px; }
.slider input { width: 100%; accent-color: #ee4f68; }
.metrics { display: grid; grid-template-columns: repeat(2, 1fr); gap: 9px; margin-top: 12px; }
.metric { background: #1a2333; border-radius: 11px; padding: 11px; }
.metric span { display: block; color: #9baac2; font-size: 10px; margin-bottom: 4px; }
.metric b { font-size: 20px; }
.metric i { font-style: normal; font-size: 11px; color: #b7c2d6; }
.flags { margin-top: 12px; color: #c6d1e2; font-size: 13px; line-height: 1.65; }
.good { color: #78dfab; font-weight: bold; }
.bad { color: #ff8b9b; font-weight: bold; }
.eco { display: flex; justify-content: space-between; align-items: center; gap: 12px; }
.eco b { font-size: 14px; }
.eco span { display: block; font-size: 12px; color: #a9b6c9; line-height: 1.35; margin-top: 4px; }
.eco button { min-width: 90px; min-height: 40px; }
.footer { text-align: center; color: #98a6bd; font-size: 11px; line-height: 1.6; margin: 14px 8px; }
</style>
</head>
<body>
<main>
<header class="top">
  <div class="brand">SMART BIKE<small>PHONE CONTROL</small></div>
  <div class="connection" id="connection">CONNECTING</div>
</header>
<section class="panel safe" id="modepanel">
  <div class="kicker">Active mode</div>
  <div class="mode" id="mode">LOADING</div>
  <div class="detail" id="detail">Checking controller status.</div>
  <div class="buttons">
    <button id="auto" onclick="setMode('auto')">RIDE AUTO</button>
    <button id="hazard" class="hazard" onclick="setMode('hazard')">HAZARD</button>
    <button id="manual" onclick="setMode('manual')">MANUAL TEST</button>
    <button id="flash" onclick="setMode('flash')">FLASH ALL</button>
    <button id="chase" onclick="setMode('chase')">CHASE</button>
    <button id="off" class="off" onclick="setMode('off')">LIGHTS OFF</button>
  </div>
</section>
<section class="panel">
  <div class="titleline">
    <div><div class="kicker">Live LED output</div><h2>What the bike is showing</h2></div>
    <div class="small">REAR VIEW</div>
  </div>
  <div class="mapnote">L3 L2 L1 | B1 B2 B3 B4 | R3 R2 R1</div>
  <div class="leds" id="preview"></div>
</section>
<section class="panel">
  <div class="titleline">
    <div><div class="kicker">Manual LED test</div><h2>Individual LEDs</h2></div>
    <div class="small">REAR VIEW</div>
  </div>
  <div class="mapnote">L3 L2 L1 | B1 B2 B3 B4 | R3 R2 R1</div>
  <div class="leds" id="leds"></div>
  <div class="twobuttons">
    <button onclick="setAll(1)">ALL ON</button>
    <button onclick="setAll(0)">ALL OFF</button>
    <button onclick="runSelfTest()">LED SELF TEST</button>
    <button onclick="runCalibration()">RECALIBRATE</button>
  </div>
  <div class="slider">
    <label for="brightness">RED LED BRIGHTNESS: <b id="brightvalue">255</b> / 255</label>
    <input id="brightness" type="range" min="0" max="255" value="255" oninput="setBrightness(this.value)">
  </div>
</section>
<section class="panel">
  <div class="titleline">
    <div><div class="kicker">Live sensor data</div><h2>Ride status</h2></div>
    <div class="small" id="movement">--</div>
  </div>
  <div class="metrics">
    <div class="metric"><span>DECELERATION</span><b id="decel">--</b><i>m/s2</i></div>
    <div class="metric"><span>PITCH</span><b id="pitch">--</b><i>deg</i></div>
    <div class="metric"><span>ROLL</span><b id="roll">--</b><i>deg</i></div>
    <div class="metric"><span>BRAKE</span><b id="brake">--</b></div>
  </div>
  <div class="flags" id="flags">Reading sensor state...</div>
</section>
<section class="panel eco">
  <div><b>LOW POWER RADIO</b><span>Reduces Wi-Fi range and radio power. It never dims automatic safety lights.</span></div>
  <button id="eco" onclick="toggleEco()">OFF</button>
</section>
<div class="footer">AUTO is the riding mode. Manual, Flash, Chase, and Lights Off return to AUTO after 2 minutes. Crash SOS always wins.<br>Controller uptime: <span id="uptime">--</span></div>
</main>
<script>
var uiMask = 0, previewMask = 0, eco = false;
var labels = ['L3', 'L2', 'L1', 'B1', 'B2', 'B3', 'B4', 'R3', 'R2', 'R1'];
var red = [false, false, false, true, true, true, true, false, false, false];
var box = document.getElementById('leds'), preview = document.getElementById('preview');
for (var i = 0; i < 10; i++) {
  var b = document.createElement('button');
  b.className = 'led ' + (red[i] ? 'red' : 'yellow') + (i === 3 ? ' gap1' : '') + (i === 7 ? ' gap2' : '');
  b.textContent = labels[i]; b.setAttribute('data-pos', i);
  b.onclick = function() { toggleLed(parseInt(this.getAttribute('data-pos'))); };
  box.appendChild(b);
  var p = document.createElement('div');
  p.className = 'led ' + (red[i] ? 'red' : 'yellow') + (i === 3 ? ' gap1' : '') + (i === 7 ? ' gap2' : '');
  p.textContent = labels[i]; preview.appendChild(p);
}
function get(path, done) {
  var x = new XMLHttpRequest(); x.open('GET', path, true);
  x.onreadystatechange = function() { if (x.readyState === 4 && done) done(x); };
  x.onerror = function() { document.getElementById('connection').textContent = 'OFFLINE'; };
  x.send();
}
function command(path) {
  get(path, function(x) { if (x.status === 200) setTimeout(poll, 80); });
}
function setMode(v) { command('/api/mode?m=' + encodeURIComponent(v)); }
function setAll(v) { command('/api/all?v=' + v); }
function toggleLed(pos) { var on = (uiMask & (1 << pos)) ? 0 : 1; command('/api/led?p=' + pos + '&v=' + on); }
var bTimer = null;
function setBrightness(v) {
  document.getElementById('brightvalue').textContent = v;
  clearTimeout(bTimer);
  bTimer = setTimeout(function() { command('/api/brightness?v=' + encodeURIComponent(v)); }, 80);
}
function toggleEco() { command('/api/eco?v=' + (eco ? 0 : 1)); }
function runSelfTest() { command('/api/selftest'); }
function runCalibration() { if (confirm('Keep bike still. Calibrate?')) command('/api/calibrate'); }
function paintGrid(kids, mask) {
  for (var i = 0; i < 10; i++) kids[i].classList.toggle('on', (mask & (1 << i)) !== 0);
}
function formatUptime(s) {
  var t = Math.max(0, Math.floor(s || 0)), h = Math.floor(t / 3600), m = Math.floor((t % 3600) / 60), sec = t % 60;
  if (h > 0) return h + 'h ' + m + 'm';
  if (m > 0) return m + 'm ' + sec + 's';
  return sec + 's';
}
function updateMode(data) {
  var mode = data.mode, crash = data.crash === 1;
  document.getElementById('mode').textContent = crash ? 'CRASH SOS' : mode.replace('_', ' ');
  var panel = document.getElementById('modepanel');
  panel.className = 'panel ' + (crash ? 'alert' : mode === 'AUTO' ? 'safe' : mode === 'HAZARD' ? 'alert' : 'warn');
  ['auto', 'hazard', 'manual', 'flash', 'chase', 'off'].forEach(function(id) {
    document.getElementById(id).classList.remove('active');
  });
  var active = mode === 'LIGHTS_OFF' ? 'off' : mode.toLowerCase();
  var el = document.getElementById(active); if (el) el.classList.add('active');
}
function poll() {
  get('/api/status?t=' + Date.now(), function(x) {
    if (x.status !== 200) { document.getElementById('connection').textContent = 'OFFLINE'; return; }
    try {
      var data = JSON.parse(x.responseText);
      uiMask = data.test; previewMask = data.preview; eco = data.eco === 1;
      paintGrid(box.children, uiMask); paintGrid(preview.children, previewMask);
      updateMode(data);
      document.getElementById('connection').textContent = 'ONLINE';
      document.getElementById('decel').textContent = Number(data.decel).toFixed(2);
      document.getElementById('pitch').textContent = Number(data.pitch).toFixed(1);
      document.getElementById('roll').textContent = Number(data.roll).toFixed(1);
      document.getElementById('brake').textContent = data.brake ? 'ON' : 'READY';
      document.getElementById('movement').textContent = data.moving ? 'MOVING' : 'STILL';
      document.getElementById('uptime').textContent = formatUptime(data.uptime);
      var state = [data.brake ? 'BRAKE ACTIVE' : 'BRAKE READY'];
      if (data.left) state.push('LEFT TURN');
      if (data.right) state.push('RIGHT TURN');
      if (data.hazard) state.push('HAZARD ACTIVE');
      if (data.crash) state.push('CRASH SOS');
      var flags = document.getElementById('flags');
      flags.textContent = state.join(' | '); flags.className = data.crash ? 'flags bad' : 'flags good';
      var ecoBtn = document.getElementById('eco'); ecoBtn.textContent = eco ? 'ON' : 'OFF'; ecoBtn.className = eco ? 'active' : '';
    } catch (e) {}
  });
}
poll(); setInterval(poll, 500);
</script>
</body>
</html>
)HTML";

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  for (int p : leftPins)  pinMode(p, OUTPUT);
  for (int p : rightPins) pinMode(p, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  for (int i = 0; i < 3; i++) ledcAttach(brakePins[i], PWM_FREQ, PWM_RES);
#else
  for (int i = 0; i < 3; i++) { ledcSetup(i, PWM_FREQ, PWM_RES); ledcAttachPin(brakePins[i], i); }
#endif

  allLightsOff();
  startupLEDTest();

  Wire.begin(); // Uses default I2C pins (D4/GPIO5 and D5/GPIO6) on XIAO ESP32-S3
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
  if (haveSavedCal) {
    pitchEstimate = pitchBaseline;
    rollEstimate  = rollBaseline;
    runCalibration(30, 0.7f);
  } else {
    pitchEstimate = 0; rollEstimate = 0;
    runCalibration(120, 0.0f);
    saveCalibration();
  }

  lastOrientationMicros = micros();
  bootMillis = millis();
  readyAnimation();

  controlMode = MODE_AUTO;
  modeStartedAt = millis();
  manualPhysicalMask = 0;
  hazardActive = false;

  startController();
  Serial.println("=== SMART BIKE READY (Mini S3 / Shared Brake 3 & 4) ===");
}

// ===================== MAIN LOOP =====================
void loop() {
  unsigned long now = millis();
  web.handleClient();
  now = millis();

  bool ridingMode = (controlMode == MODE_AUTO || controlMode == MODE_HAZARD);
  updateSensors(now, ridingMode);

  if ((controlMode == MODE_MANUAL || controlMode == MODE_FLASH ||
       controlMode == MODE_CHASE || controlMode == MODE_OFF) &&
      now - modeStartedAt >= TEST_MODE_TIMEOUT_MS) {
    selectMode(MODE_AUTO);
  }

  renderLights(now);

#if ENABLE_SERIAL_TUNING
  handleSerialInput();
#endif
}

// ===================== SENSOR ENGINE =====================
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

  float rawDiff = accRaw - filtAccel;
  if (fabsf(rawDiff) > cfg.spikeClamp) {
    accRaw = filtAccel + copysignf(cfg.spikeClamp, rawDiff);
  }

  filtAccel = filtAccel * (1 - cfg.accelFilter) + accRaw * cfg.accelFilter;
  filtGyroZ = filtGyroZ * (1 - cfg.gyroFilter)  + gyrRaw * cfg.gyroFilter;

  updateOrientation(a, g, dt);

  float pitchDelta = pitchEstimate - pitchBaseline;
  float gravityCompX = 0;
#if ENABLE_TILT_COMPENSATION
  gravityCompX = GRAVITY_MPS2 * sinf(pitchDelta);
#endif
  float accelFromRest = (filtAccel - accelBaseline) - gravityCompX;
  float turnRate = filtGyroZ - gyroZBias;
  lastTurnRate = turnRate;

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

  int target = 0;
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
}

// ===================== OUTPUT =====================
void renderLights(unsigned long now) {
#if ENABLE_CRASH_DETECTION
  if (crashDetected) { renderCrashBeacon(now); return; }
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

void renderAutoAndHazard(unsigned long now) {
  int outputBrightness = (int)currentBrightness;
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

// ===================== ORIENTATION & CALIBRATION =====================
void updateOrientation(const sensors_event_t &a, const sensors_event_t &g, float dt) {
  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az));
  float rollAcc  = atan2f(ay, az);
  pitchEstimate = cfg.compAlpha * (pitchEstimate + g.gyro.y * dt) + (1.0f - cfg.compAlpha) * pitchAcc;
  rollEstimate  = cfg.compAlpha * (rollEstimate  + g.gyro.x * dt) + (1.0f - cfg.compAlpha) * rollAcc;
}

void runCalibration(int samples, float blendExisting) {
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
  accelBaseline = accelBaseline * blendExisting + (sa / samples) * (1.0f - blendExisting);
  gyroZBias     = gyroZBias     * blendExisting + (sg / samples) * (1.0f - blendExisting);
  pitchBaseline = pitchBaseline * blendExisting + pitchEstimate * (1.0f - blendExisting);
  rollBaseline  = rollBaseline  * blendExisting + rollEstimate  * (1.0f - blendExisting);
  filtAccel = accelBaseline; filtGyroZ = gyroZBias; accelMean = accelBaseline;
}

void saveTunables() {
  prefs.begin("bikecfg", false); prefs.putBytes("cfg", &cfg, sizeof(cfg)); prefs.end();
}
void loadTunables() {
  prefs.begin("bikecfg", true);
  if (prefs.isKey("cfg") && prefs.getBytesLength("cfg") == sizeof(cfg)) prefs.getBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
}
void saveCalibration() {
  calData.accelBaseline = accelBaseline; calData.gyroZBias = gyroZBias;
  calData.pitchBaseline = pitchBaseline; calData.rollBaseline = rollBaseline;
  calData.valid = true;
  prefs.begin("bikecfg", false); prefs.putBytes("cal", &calData, sizeof(calData)); prefs.end();
}
bool loadCalibration() {
  bool ok = false;
  prefs.begin("bikecfg", true);
  if (prefs.isKey("cal") && prefs.getBytesLength("cal") == sizeof(calData)) {
    prefs.getBytes("cal", &calData, sizeof(calData));
    if (calData.valid) {
      accelBaseline = calData.accelBaseline; gyroZBias = calData.gyroZBias;
      pitchBaseline = calData.pitchBaseline; rollBaseline = calData.rollBaseline;
      ok = true;
    }
  }
  prefs.end();
  return ok;
}

// ===================== CRASH DETECTION =====================
#if ENABLE_CRASH_DETECTION
void updateCrashDetection(const sensors_event_t &a, unsigned long now) {
  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  float excessG = fabsf(mag - GRAVITY_MPS2) / GRAVITY_MPS2;
  if (!crashDetected && excessG > cfg.impactThresholdG) { impactFlag = true; impactAt = now; }
  bool fallen = fabsf(rollEstimate) > (cfg.fallenAngleDeg * DEG_TO_RAD);
  if (fallen) { if (fallenSince == 0) fallenSince = now; } else fallenSince = 0;
  bool fallenConfirmed = fallen && fallenSince != 0 && (now - fallenSince > (unsigned long)cfg.fallenConfirmMs);
  if (!crashDetected) {
    bool impactThenStill = impactFlag && (now - impactAt < 8000UL) && isStationary && stillSince != 0 && (now - stillSince > (unsigned long)cfg.crashStillMs);
    if (impactThenStill || fallenConfirmed) { crashDetected = true; crashClearMovingSince = 0; }
    if (impactFlag && now - impactAt > 8000UL) impactFlag = false;
  } else {
    bool activelyMoving = !isStationary && (accelStd > cfg.stillJitter * 2.0f || gyroMag > cfg.stillGyroMax * 3.0f);
    if (activelyMoving) { if (crashClearMovingSince == 0) crashClearMovingSince = now; } else crashClearMovingSince = 0;
    if (crashClearMovingSince != 0 && now - crashClearMovingSince > 1500UL) { crashDetected = false; impactFlag = false; }
  }
}

void renderCrashBeacon(unsigned long now) {
  const int PATTERN_SHORT = 150, PATTERN_LONG = 400, PATTERN_GAP = 150, WORD_GAP = 900;
  const int SEQ_LEN = 9;
  const int seq[SEQ_LEN] = { PATTERN_SHORT, PATTERN_SHORT, PATTERN_SHORT, PATTERN_LONG, PATTERN_LONG, PATTERN_LONG, PATTERN_SHORT, PATTERN_SHORT, PATTERN_SHORT };
  static int idx = 0; static unsigned long stepStart = 0; static bool on = false;
  if (stepStart == 0) { stepStart = now; on = true; allLightsOn(); }
  unsigned long elapsed = now - stepStart;
  if (on) {
    if (elapsed >= (unsigned long)seq[idx]) { on = false; stepStart = now; allLightsOff(); }
  } else {
    unsigned long gap = (idx == SEQ_LEN - 1) ? WORD_GAP : PATTERN_GAP;
    if (elapsed >= gap) { idx = (idx + 1) % SEQ_LEN; on = true; stepStart = now; allLightsOn(); }
  }
}
#endif

// ===================== SERIAL TUNING =====================
#if ENABLE_SERIAL_TUNING
void handleSerialInput() {
  static String serialBuffer = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) { processCommand(serialBuffer); serialBuffer = ""; }
    } else {
      serialBuffer += c; if (serialBuffer.length() > 64) serialBuffer = "";
    }
  }
}
void processCommand(String line) {
  line.trim(); line.toUpperCase();
  if (line == "STATUS") { printStatus(); return; }
  if (line == "SAVE")   { saveTunables(); Serial.println("OK SAVED"); return; }
  if (line == "LOAD")   { loadTunables(); Serial.println("OK LOADED"); return; }
}
void printStatus() { Serial.print("MODE:"); Serial.println(modeName()); }
#endif

// ===================== TURN ANIMATION & HELPERS =====================
void animateTurn(const int pins[3], int &step, unsigned long &animAt, float turnRateValue) {
  float mag = constrain(fabsf(turnRateValue), 1.0f, 3.0f);
  float f = (mag - 1.0f) / 2.0f;
  unsigned long speed = (unsigned long)cfg.turnAnimSlowMs - (unsigned long)(f * (cfg.turnAnimSlowMs - cfg.turnAnimFastMs));
  if (millis() - animAt < speed) return;
  animAt = millis();
  for (int i = 0; i < 3; i++) digitalWrite(pins[i], LOW);
  for (int i = 0; i <= step && i < 3; i++) digitalWrite(pins[i], HIGH);
  step++; if (step > 2) step = 0;
}

void startupLEDTest() {
  leftLights();  delay(300); leftOff();
  brakeLights(); delay(300); brakeOff();
  rightLights(); delay(300); rightOff();
  allLightsOn(); delay(250); allLightsOff(); delay(150);
}

void sensorError() {
  while (true) {
    leftLights(); rightLights(); delay(200);
    leftOff(); rightOff(); delay(200);
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
  for (int i = 0; i < 3; i++) ledcWrite(brakePins[i], b);
#else
  for (int i = 0; i < 3; i++) ledcWrite(i, b);
#endif
}

void allLightsOn()  { leftLights(); brakeLights(); rightLights(); }
void allLightsOff() { leftOff(); brakeOff(); rightOff(); }

// ===================== WEB SERVER CONTROLLER =====================
void startController() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(CONTROL_AP_NAME, CONTROL_AP_PASSWORD);
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
  web.begin();
}

void handleRoot() { web.send_P(200, "text/html", CONTROL_PAGE); }
void handleStatus() {
  char json[480];
  int crash = 0;
#if ENABLE_CRASH_DETECTION
  crash = crashDetected ? 1 : 0;
#endif
  unsigned long now = millis();
  uint16_t testMask = physicalToUiMask(manualPhysicalMask);
  uint16_t previewMask = physicalToUiMask(activePhysicalMask(now));
  snprintf(json, sizeof(json), "{\"mode\":\"%s\",\"mask\":%u,\"test\":%u,\"preview\":%u,\"decel\":%.2f,\"pitch\":%.1f,\"roll\":%.1f,\"brake\":%d,\"left\":%d,\"right\":%d,\"hazard\":%d,\"crash\":%d,\"moving\":%d,\"eco\":%d,\"uptime\":%lu}",
    modeName(), testMask, testMask, previewMask, deceleration, pitchEstimate * RAD_TO_DEG, rollEstimate * RAD_TO_DEG,
    brakeActive ? 1 : 0, leftTurning ? 1 : 0, rightTurning ? 1 : 0, hazardActive ? 1 : 0, crash, isStationary ? 0 : 1, ecoRadioMode ? 1 : 0, (unsigned long)((now - bootMillis) / 1000UL));
  web.send(200, "application/json", json);
}

void handleMode() {
  if (!web.hasArg("m")) { web.send(400, "text/plain", "missing mode"); return; }
  String m = web.arg("m"); m.toUpperCase();
  if      (m == "AUTO")   selectMode(MODE_AUTO);
  else if (m == "HAZARD") selectMode(MODE_HAZARD);
  else if (m == "MANUAL") selectMode(MODE_MANUAL);
  else if (m == "FLASH")  selectMode(MODE_FLASH);
  else if (m == "CHASE")  selectMode(MODE_CHASE);
  else if (m == "OFF")    selectMode(MODE_OFF);
  sendOk();
}

void handleLed() {
  if (!web.hasArg("p") || !web.hasArg("v")) { web.send(400, "text/plain", "missing LED value"); return; }
  int pos = web.arg("p").toInt(), on = web.arg("v").toInt();
  if (pos < 0 || pos > 9 || (on != 0 && on != 1)) { web.send(400, "text/plain", "invalid"); return; }
  uint16_t bit = 1U << UI_TO_PHYSICAL[pos];
  if (on) manualPhysicalMask |= bit; else manualPhysicalMask &= ~bit;
  selectMode(MODE_MANUAL); sendOk();
}

void handleAll() {
  int val = web.arg("v").toInt();
  manualPhysicalMask = val ? 0x03FF : 0;
  selectMode(MODE_MANUAL); sendOk();
}

void handleBrightness() {
  redBrightness = static_cast<uint8_t>(constrain(web.arg("v").toInt(), 0, 255));
  sendOk();
}

void handleEco() {
  ecoRadioMode = (web.arg("v").toInt() != 0);
  WiFi.setTxPower(ecoRadioMode ? WIFI_POWER_2dBm : WIFI_POWER_19_5dBm);
  sendOk();
}

void handleSelfTest() {
  web.send(200, "text/plain", "OK");
  selectMode(MODE_OFF); allLightsOff(); delay(100);
  startupLEDTest(); allLightsOff(); selectMode(MODE_AUTO);
}

void handleCalibration() {
  web.send(200, "text/plain", "OK");
  selectMode(MODE_OFF); allLightsOff();
  runCalibration(120, 0.0f); saveCalibration();
  selectMode(MODE_AUTO);
}

void handleLegacyCommand() {
  if (web.hasArg("value")) selectMode(MODE_AUTO);
  sendOk();
}

void sendOk() { web.send(200, "text/plain", "OK"); }

void selectMode(ControlMode mode) {
  controlMode = mode;
  modeStartedAt = millis();
  hazardActive = (mode == MODE_HAZARD);
#if ENABLE_CRASH_DETECTION
  crashDetected = false; impactFlag = false; fallenSince = 0; crashClearMovingSince = 0;
#endif
  allLightsOff();
}

uint16_t currentPhysicalMask(unsigned long now) {
  switch (controlMode) {
    case MODE_AUTO:   return 0;
    case MODE_HAZARD: return ((now / 500UL) % 2UL) ? 0 : 0x0387;
    case MODE_MANUAL: return manualPhysicalMask;
    case MODE_FLASH:  return (((now - modeStartedAt) / 180UL) % 2UL) ? 0 : 0x03FF;
    case MODE_CHASE:  return 1U << UI_TO_PHYSICAL[((now - modeStartedAt) / 120UL) % 10UL];
    case MODE_OFF:    default: return 0;
  }
}

uint16_t physicalToUiMask(uint16_t raw) {
  uint16_t ui = 0;
  for (int pos = 0; pos < 10; pos++) {
    if (raw & (1U << UI_TO_PHYSICAL[pos])) ui |= 1U << pos;
  }
  return ui;
}

uint16_t activePhysicalMask(unsigned long now) {
#if ENABLE_CRASH_DETECTION
  if (crashDetected) return 0x03FF;
#endif
  if (controlMode != MODE_AUTO && controlMode != MODE_HAZARD) return currentPhysicalMask(now);
  if (controlMode == MODE_HAZARD) {
    uint16_t raw = ((now / 500UL) % 2UL) ? 0 : 0x0387;
    if (currentBrightness > 1) raw |= 0x0078;
    return raw;
  }
  uint16_t raw = 0;
  if (currentBrightness > 1) raw |= 0x0078;
  if (digitalRead(LEFT1)) raw |= 1U << 0;
  if (digitalRead(LEFT2)) raw |= 1U << 1;
  if (digitalRead(LEFT3)) raw |= 1U << 2;
  if (digitalRead(RIGHT1)) raw |= 1U << 7;
  if (digitalRead(RIGHT2)) raw |= 1U << 8;
  if (digitalRead(RIGHT3)) raw |= 1U << 9;
  return raw;
}

void writeManualPhysicalMask(uint16_t raw) {
  raw &= 0x03FF;
  digitalWrite(LEFT1,  (raw & (1U << 0)) ? HIGH : LOW);
  digitalWrite(LEFT2,  (raw & (1U << 1)) ? HIGH : LOW);
  digitalWrite(LEFT3,  (raw & (1U << 2)) ? HIGH : LOW);
  digitalWrite(RIGHT1, (raw & (1U << 7)) ? HIGH : LOW);
  digitalWrite(RIGHT2, (raw & (1U << 8)) ? HIGH : LOW);
  digitalWrite(RIGHT3, (raw & (1U << 9)) ? HIGH : LOW);

  // Write to 3 brake outputs, mapping brake 3 & 4 (bits 5 & 6) to BRAKE3 (`brakePins[2]`)
  for (int i = 0; i < 3; i++) {
    bool on = (raw & (1U << (i + 3)));
    if (i == 2) {
      if ((raw & (1U << 5)) || (raw & (1U << 6))) on = true;
    }
    int value = on ? redBrightness : 0;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(brakePins[i], value);
#else
    ledcWrite(i, value);
#endif
  }
}

const char *modeName() {
  switch (controlMode) {
    case MODE_AUTO:   return "AUTO";
    case MODE_HAZARD: return "HAZARD";
    case MODE_MANUAL: return "MANUAL";
    case MODE_FLASH:  return "FLASH";
    case MODE_CHASE:  return "CHASE";
    case MODE_OFF:    default: return "LIGHTS_OFF";
  }
}
