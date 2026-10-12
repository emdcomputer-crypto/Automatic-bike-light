# Smart Bike Dynamic Lighting System

This is my custom-built smart bike lighting system powered by a Seeed Studio XIAO ESP32-S3 and an MPU6050 inertial sensor. I designed this project from scratch to provide professional-grade safety features on the road—including automatic deceleration-triggered brake lights, dynamic turn signals, crash detection, and a local Wi-Fi web server that lets me control and monitor everything directly from my phone without needing an internet connection or an external app.

---

## System Overview & Core Features

* **Tilt-Compensated Braking:** The system calculates true deceleration using a complementary filter, filtering out false triggers caused by riding up or down steep hills.
* **Brake Hysteresis:** Uses independent thresholds to turn the brake light on and off, preventing annoying flickering during gradual slowing.
* **Dynamic Turn Signals:** Smooth, progressive sliding animations across the side LEDs when turning.
* **Crash Detection (SOS):** Automatically monitors for severe impacts or vehicle rollovers, triggering a Morse code SOS beacon (`... --- ...`) in an emergency.
* **Local Wi-Fi Web Server:** The ESP32 hosts a secure local hotspot (`Luci di Emad`) serving an interactive dashboard for live telemetry and manual overrides.
* **No Enclosure Design:** The PCB is built as a clean, standalone unit designed to operate without a 3D-printed enclosure.

---

## Hardware & PCB Design

I designed a custom PCB in KiCad measuring 120x35 mm with a sleek, capsule-like shape featuring rounded edges and two mounting holes in opposite corners for secure mechanical fastening.

### Component Breakdown
* **Microcontroller:** Seeed Studio XIAO ESP32-S3 (chosen for its compact footprint and built-in Wi-Fi).
* **Inertial Sensor:** MPU6050 6-axis accelerometer and gyroscope.
* **LED Matrix (Rear View Layout):**
  * **Left Turn Indicator:** 3 yellow LEDs (`LEFT1`, `LEFT2`, `LEFT3`).
  * **Brake Lights:** 4 red LEDs (`BRAKE1`, `BRAKE2`, `BRAKE3`, `BRAKE4`). *Note: Due to pin constraints on the XIAO, brake LEDs 3 and 4 share a single control pin.*
  * **Right Turn Indicator:** 3 yellow LEDs (`RIGHT1`, `RIGHT2`, `RIGHT3`).
* **Power Management:** LiPo battery connected via a 2-pin header paired with a USB-C charging circuit.

### PCB Layout Strategy
To keep the front display clean and professional, I routed the entire board and placed the XIAO ESP32-S3, the MPU6050 header, and the battery connector flipped onto the **back layer**. This left plenty of space on the front layer exclusively for all 10 LEDs. I added custom icons on the silkscreen, poured a solid GND copper plane across the bottom layer for signal integrity, and completed the layout with 0 DRC errors.

---

## Startup Sequence & Behavioral Diagnostics

When I power on the system, it runs a strict diagnostic and initialization sequence before arming road safety features:

1. **LED Self-Test:** 
   * Left LEDs -> Brake LEDs -> Right LEDs light up briefly in sequence to verify component health and circuit continuity.
2. **Full Flash:** 
   * All LEDs flash simultaneously to confirm the hardware test passed successfully.
3. **Sensor Diagnostics:** 
   * 3 yellow flashes indicate whether the MPU6050 is detected. If it fails to initialize, normal operation halts for safety to prevent erroneous automated triggers.
4. **Inertial Calibration:** 
   * Alternating yellow and red lights indicate MPU6050 calibration. I keep the bike completely still during this step to calibrate accelerometer and gyro biases.
5. **Ready Animation:** 
   * A sweeping light sequence from left to right followed by one full flash indicates the system is ready, and automatic braking/turning are active.

---

## Firmware Architecture (v4.0)

I wrote a self-contained Arduino firmware (`Code_3.ino`) optimized specifically for the ESP32-S3.

### Key Firmware Highlights:
* **Single-Loop Execution:** The sensor engine (`updateSensors`) runs every cycle in all operational modes, ensuring live telemetry never freezes during bench testing or manual overrides.
* **Automatic Mode Timeouts:** All manual test modes automatically revert to `AUTO` mode after 2 minutes of inactivity so the bike is never left without active safety lighting.
* **Persistent Storage:** Uses the `Preferences` library to save calibration offsets and custom tuning parameters directly to flash memory.

### Phone Controller & Web Dashboard
Connecting to the bike's Wi-Fi hotspot (`Luci di Emad`) opens a mobile-optimized control dashboard with the following capabilities:
* **Live Telemetry Readouts:** Real-time data for deceleration, pitch, roll, and motion status (`MOVING` / `STILL`).
* **Mode Switching:** Instantly toggle between **RIDE AUTO**, **HAZARD**, **MANUAL TEST**, **FLASH ALL**, **CHASE**, and **LIGHTS OFF**.
* **Individual LED Control:** Interactive rear-view grid allowing manual toggling of individual LEDs for testing.
* **Brightness Adjustment:** Slider to dynamically control red LED output brightness.
* **Low Power Radio Toggle:** Reduces Wi-Fi TX power to save battery without affecting safety lighting performance.

---

## Getting Started & Requirements

### Required Arduino Libraries
Make sure you have the following libraries installed in your Arduino IDE environment:
* `Wire`
* `Adafruit_MPU6050`
* `Adafruit_Sensor`
* `Preferences`
* `WiFi`
* `WebServer`

### Flashing and Operation
1. Open `Code_3.ino` in the Arduino IDE.
2. Select your target board as **Seeed Studio XIAO ESP32-S3**.
3. Compile and upload the sketch to the board.
4. Open the Serial Monitor at `115200` baud to view boot logs or type `HELP` into the console to use the live serial tuning interface.
5. Connect your phone to the Wi-Fi network `Luci di Emad` (password: `damiri123`) and navigate to `http://192.168.4.1`.
