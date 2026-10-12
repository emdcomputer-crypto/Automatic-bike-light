I have included two firmware versions depending on which board you choose for your build:

Original  Firmware (Original.ino): The original, fully tested, and working firmware built for the full-size ESP32 WROOM board used during early prototyping. (wich i highly reccomend you use but fix and adapt to the esp32 s3)

New XIAO S3 Firmware (Untested.ino): The updated version built for the compact Seeed Studio XIAO ESP32-S3. Because of pin constraints on the smaller board, brake LEDs 3 and 4 share a single control pin . (Not tested might not work as intended)

Note: This new XIAO S3 firmware version was not physically tested due to the unavailability of the parts.

Requirements & Flashing
Required Libraries: Wire, Adafruit_MPU6050, Adafruit_Sensor, Preferences, WiFi, WebServer

How to Flash: Open your chosen file in the Arduino IDE, select your target board (Seeed Studio XIAO ESP32-S3 or standard ESP32), and upload. Connect your phone to the Wi-Fi network SmartBike (password: Bike123) and open [http://192.168.4.1](http://192.168.4.1) in your browser.
