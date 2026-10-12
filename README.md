# Smart Bike Dynamic Lighting System

This is my custom built smart bike lighting system powered by a Seeed Studio XIAO ESP32-S3 and an MPU6050 inertial sensor. I designed this project from scratch to provide safety features on the road including automatic deceleration triggered brake lights, dynamic turn signals, crash detection, and a local Wi-Fi web server that lets me control and monitor everything directly from my phone without needing an internet connection or an external app.

---

## Core Features

* **Tilt-Compensated Braking:** The system calculates true deceleration using a complementary filter, filtering out false triggers caused by riding up or down steep hills.
* **Brake Hysteresis:** Uses independent thresholds to turn the brake light on and off, preventing annoying flickering during gradual slowing.
* **Turn Signal effect:** Smooth, progressive sliding animations across the side LEDs when turning.
* **Crash Detection (SOS):** Automatically monitors for severe impacts or vehicle rollovers, triggering a Morse code SOS beacon (`... --- ...`) in an emergency. (Random feature)
* **Local Wi-Fi Web Server:** The ESP32 hosts a local hotspot serving a dashboard for live data showing the lights state and the possibility of different modes with also a manual customization mode wich lets you customize wich leds stay on (The mode will always go back to "auto" after every 2 minutes, of course you can change that).

---

## Hardware & PCB Design

I designed a custom PCB in KiCad measuring 120x35 mm with rounded edges (2mm diametre) and two mounting holes in opposite corners for securing it in any custom 3d printed enclosure

### Component Breakdown
* **Microcontroller:** Seeed Studio XIAO ESP32-S3 (chosen for its size and built-in Wi-Fi).
* **Inertial Sensor:** MPU6050 6-axis accelerometer and gyroscope.
* **LED Matrix (Rear View Layout):**
  * **Left Turn Indicator:** 3 yellow LEDs (`LEFT1`, `LEFT2`, `LEFT3`).
  * **Brake Lights:** 4 red LEDs (`BRAKE1`, `BRAKE2`, `BRAKE3`, `BRAKE4`). *Note: Due to pin constraints on the XIAO, brake LEDs 3 and 4 share a single control pin in the original firware when testing it was made for the full size esp32 wich may lead to bugs in the firmware.*
  * **Right Turn Indicator:** 3 yellow LEDs (`RIGHT1`, `RIGHT2`, `RIGHT3`).
* **Power Management:** LiPo battery connected via a 2-pin header paired with a USB-C charging circuit is the best option.

### PCB Layout Strategy
I routed the entire board and placed the XIAO ESP32-S3, the MPU6050 header, and the battery connector flipped onto the **back layer**. This left plenty of space on the front layer exclusively for all 10 LEDs. I added custom icons on the silkscreen, poured a solid GND copper plane across the bottom layer for signal integrity, and completed the layout with 0 DRC errors.


<img width="1252" height="415" alt="image" src="https://github.com/user-attachments/assets/d2cb35b9-9ba0-4c48-8486-b81afa325a8f" />
<img width="1000" height="408" alt="image" src="https://github.com/user-attachments/assets/3c44a3fe-7ea8-4dcd-8a2d-43c42eeb6698" />
<img width="1206" height="424" alt="image" src="https://github.com/user-attachments/assets/5e8144e3-1f17-4765-979a-4aca28f5bc09" />



---

## Startup Sequence & Behavioral Diagnostics

When I power on the system, it has some simple and usefull sequences before starting:

1. **LED Self-Test:** 
   * Left LEDs -> Brake LEDs -> Right LEDs light up briefly in sequence so we can verify all leds work.
2. **Full Flash:** 
   * All LEDs flash simultaneously to confirm everything is fine.
3. **Sensor Diagnostics:** 
   * 3 yellow flashes indicate whether the MPU6050 is detected. If it fails to get detected it will keep flashing .
4. **initial Calibration:** 
   * Alternating yellow and red lights indicate MPU6050 calibration. I keep the bike completely still during this step to calibrate accelerometer and gyro biases.
5. **Ready Animation:** 
   * A sweeping light sequence from left to right followed by one full flash indicates the system is on everything should work.

---

## Firmware Architecture (v4.0)

Using AI i wrote a lot of firmwares went thru different bug fixes and all but this was the best most stable one wich still due to the lack of space in the pcb i had to go with a smaller and different microcontroller than the original one wich may make the firmware unstable or make some stuff not work either way feel free to take this firmware fix it, customize it or make one from scratch as i will provide the schematic too.

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
Make sure you have the following libraries installed in your Arduino IDE environment to be able to use the firmware
* `Wire`
* `Adafruit_MPU6050`
* `Adafruit_Sensor`
* `Preferences`
* `WiFi`
* `WebServer`

### Flashing 
1. Open `Code_3.ino` in the Arduino IDE.
2. Select your target board as **Seeed Studio XIAO ESP32-S3** or something similar.
3. Compile and upload the sketch to the board.
4. Open the Serial Monitor at `115200` baud to view boot logs or type `HELP` into the console to use the live serial tuning interface.
5. Connect your phone to the Wi-Fi network `SmartBike` (password: `Bike123`) and on your browser navigate to `http://192.168.4.1`.
