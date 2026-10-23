# Smart Bike Dynamic Lighting System

This is my custom built smart bike lighting system powered by a Seeed Studio XIAO ESP32-S3 and an MPU6050 inertial sensor. I designed this project from scratch to provide safety features on the road including automatic deceleration triggered brake lights, dynamic turn signals, crash detection, and a local Wi-Fi web server that lets me control and monitor everything directly from my phone without needing an internet connection or an external app.


---
# Everything you need

* schematic, pcb and others in `Hardware` 
* Firmware `Firmware` (wich i highly reccomend to make changes to)
* Fabrication files (gerber) `FabricationFiles` needed to produce the pcb 
---

## Core Features

* **Tilt-Compensated Braking:** The system calculates true deceleration using a sort of filter, filtering out false triggers caused by riding up or down hills.
* **Brake time:** Uses independent thresholds to turn the brake light on and off, preventing annoying flickering during gradual slowing.
* **Turn Signal effect:** Cool and progressive sliding animations across the side LEDs when turning wich i really like.
* **Crash Detection (SOS):** Automatically monitors for severe impacts or vehicle rollovers, triggering a Morse code SOS beacon (`... --- ...`) in an emergency. (Random feature honestly but still cool)
* **Local Wi-Fi Web Server:** The ESP32 hosts a local hotspot serving a dashboard for live data showing the lights state and the possibility of different modes with also a manual customization mode wich lets you customize wich leds stay on (The mode will always go back to "auto" after every 2 minutes, of course you can change that).

---

## Hardware & PCB Design

I designed a custom PCB in KiCad measuring 120x35 mm with rounded edges (2mm diametre) and two mounting holes in opposite corners for securing it in any custom 3d printed enclosure

### Component Breakdown
* **Microcontroller:** Seeed Studio XIAO ESP32-S3 (chosen for its size and built-in Wi-Fi).
* **Inertial Sensor:** MPU6050 6-axis accelerometer and gyroscope.
* **LEDs  (Rear View Layout):**
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

# Bill of Materials (BOM)

| Category / Role | Item | Qty | Price | Link |
| :--- | :--- | :--- | :--- | :--- |
| **Microcontroller** | Seeed Studio XIAO ESP32-S3 | 1 | $18.99 | [Amazon Link](https://www.amazon.it/dp/B0BYSB66S5?psc=1&ref_=cm_sw_r_cp_ud_ct_YRPRFCPGAK7MDH9JT337_3) |
| **Boost Converter** | Youmile DC-DC Step Up Boost Module (2V-24V to 5V-28V 2A) with 24AWG wire | 1 | $18.32 | [Amazon Link](https://www.amazon.it/dp/B0C5HXZ6HT?psc=1&ref_=cm_sw_r_cp_ud_ct_YRPRFCPGAK7MDH9JT337) |
| **Battery Charger** | Greluma Type-C USB-C 5V 1A 18650 LiPo Charger Module (with protection) | 1 | $8.93 | [Amazon Link](https://www.amazon.it/dp/B09WMVLWH5?psc=1&ref_=cm_sw_r_cp_ud_ct_YRPRFCPGAK7MDH9JT337_1) |
| **Battery** | EEMB 963450 3.7V 1800mAh LiPo Rechargeable Battery with JST | 1 | $15.03 | [Amazon Link](https://www.amazon.it/dp/B08ZCQXFX4?psc=1&ref_=cm_sw_r_cp_ud_ct_YRPRFCPGAK7MDH9JT337_4) |
| **Inertial Sensor** | ARCELI GY-521 MPU6050 6-Axis Accelerometer & Gyroscope | 1 | $7.81 | [Amazon Link](https://www.amazon.it/dp/B07BVXN2GP?psc=1&ref_=cm_sw_r_cp_ud_ct_YRPRFCPGAK7MDH9JT337_5) |
| **LEDs & Resistors** | Lyeteung 450 Pcs 3mm LED Diode Kit (Red, Yellow, Green, White) + Resistors | 1 kit | $11.16 | [Amazon Link](https://www.amazon.it/dp/B0BTYFQGJM?psc=1&ref_=cm_sw_r_cp_ud_ct_YRPRFCPGAK7MDH9JT337_2) |
| **PCB Fabrication** | JLCPCB Custom PCB Manufacturing (Gerber files in Hardware folder) | 1 | 19.46 | [JLCPCB](https://jlcpcb.com) |

### Total Estimated: **$80.24 USD** *(excluding PCB manufacturing)*
### Total Estimated: **$99.70 USD** *(including PCB manufacturing)*


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

## Firmware Architecture 

Using the help of AI i wrote a lot of firmwares (still learning c++ in school but can't write full firwmwares so i'm sorry i'm still working and watching tutorials) went thru different bug fixes and all but this was the best most stable one wich still due to the lack of space in the pcb i had to go with a smaller and different microcontroller than the original one (that's why i included the original firmware that was built on the full sized esp32 wrom) so the firmware might be unstable or make some stuff not work either way feel free to take this firmware fix it, customize it or make one from scratch as i will provide the schematic too.


### Phone Controller & Web Dashboard
Connecting to the bike's Wi-Fi hotspot (`SmartBike`) opens a website control dashboard with the following capabilities:
* **Live data:** data for deceleration, pitch, roll, and motion status (`MOVING` / `STILL`).
* **Mode Switching:** the ability to toggle between different preset modes **RIDE AUTO**, **HAZARD**, **MANUAL TEST**, **FLASH ALL**, **CHASE**, and **LIGHTS OFF**.
* **Individual LED Control:** a grid, allowing manual toggling of individual LEDs for testing or for customization.
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
