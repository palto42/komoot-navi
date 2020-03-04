/*
 * BLE receiver for Komoot navi app
 * The Komoot BLE communication is described on https://github.com/komoot/BLEConnect
 *
 * This sketch was inspired by code from <Andreas Spiess>
 * https://github.com/SensorsIot/Bluetooth-BLE-on-Arduino-IDE/blob/master/Polar_Receiver/Polar_Receiver.ino
 * which is based on Neil Kolban's example file: https://github.com/nkolban/ESP32_BLE_Arduino

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
  https://github.com/komoot/BLEConnect
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
  DEALINGS IN THE SOFTWARE.
 */

#define DEBUG 0 // flag to turn on/off debugging

#include <Arduino.h>
#define Serial if(DEBUG)Serial

#include <string>

#include "BLEDevice.h"
#include "soc/rtc.h"

#include<U8g2lib.h>
#include<Wire.h>

#define Threshold 50 /* touch pin threshold 75, greater the value = more the sensitivity */

#define PIN_SCL 22
#define PIN_SDA 21

// U8g2 Contructor
// see https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// The hardware I2C allows pin remapping for some controller types. The optional
// pin numbers are listed after the reset pin: ..._HW_I2C([reset [, clock, data]]).
// Use U8X8_PIN_NONE if the reset input of the display is not connected.
#if DEBUG
// using small OLED on dev-platform
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ PIN_SCL, /* data=*/ PIN_SDA );   // 0.8" OLED
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ PIN_SCL, /* data=*/ PIN_SDA);   // ESP32 HW I2C with pin remapping
#else
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // 1.3" OLED
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ PIN_SCL, /* data=*/ PIN_SDA);   // remapping I2C pins
#endif

const int rotation = 1;  // display rotaion: 0=none, 1=180deg

// local library with navigation icons and extra symbols
#include <symbols.h>

// Komoot Connect service and characteristics
static BLEUUID serviceUUID("71C1E128-D92F-4FA8-A2B2-0F171DB3436C"); // navigationServiceUUID
static BLEUUID charUUID("503DD605-9BCB-4F6E-B235-270A57483026");    // navigationServiceNotifyCharacteristicUUI
static BLEUUID heartUUID("6D75DBF0-D763-4147-942A-D97B1BC700CF");   // navigationServiceHeartbeatWriteCharacteristicUUID

// static BLEAddress *pServerAddress;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;

boolean new_notify = false;
boolean new_street = false;
int heartbeat = 0;
int timeout = 0;
int scan_time = 0;
BLEScan* pBLEScan;

uint8_t old_data[20];
std::string value = "Start";

const int battPin = 35; // A2=2 A6=34
unsigned int raw=0;
float volt=0.0;
// ESP32 ADV is a bit non-linear
const float vScale1 = 225.0; // divider for higher voltage range
const float vScale2 = 245.0; // divider for lower voltage range

std::string street = "Start";
std::string street_old = "";

void callback(){
  //placeholder callback function for touch
  Serial.println ("Touch!");
}

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    timeout = 0;
    // pData 4 = direction, 5-8 = distance, 9.. street
    // check if direction has changed
    if (pData[4] != old_data[4]) {
      new_notify = true;
      new_street = true;
      memcpy(old_data,pData,20);
    } else { // check if street has changed
      for (int i = 9; i < 20; i++) {  // 4 = direction, 5-8 = distance, 9.. street
        if (pData[i] != old_data[i]) {
          new_notify = true;
          new_street = true;
          memcpy(old_data,pData,20);
          break;
        }
      }
    }
    if (! new_street) { // check if distance has changed
      for (int i = 5; i < 9; i++) {
        if (pData[i] != old_data[i]) {
          new_notify = true;
          memcpy(old_data,pData,20);
          break;
        }
      }
    }
    Serial.print ("*");  // just print a * for each received notification
}

std::string utf8_substr(const std::string& str, unsigned int start, unsigned int leng)
{
    if (leng==0) { return ""; }
    unsigned int c, i, ix, q, min=std::string::npos, max=std::string::npos;
    for (q=0, i=0, ix=str.length(); i < ix; i++, q++)
    {
        if (q==start){ min=i; }
        if (q<=start+leng || leng==std::string::npos){ max=i; }

        c = (unsigned char) str[i];
        if      (
                 //c>=0   &&
                 c<=127) i+=0;
        else if ((c & 0xE0) == 0xC0) i+=1;
        else if ((c & 0xF0) == 0xE0) i+=2;
        else if ((c & 0xF8) == 0xF0) i+=3;
        //else if (($c & 0xFC) == 0xF8) i+=4; // 111110bb //byte 5, unnecessary in 4 byte UTF-8
        //else if (($c & 0xFE) == 0xFC) i+=5; // 1111110b //byte 6, unnecessary in 4 byte UTF-8
        else return "";//invalid utf8
    }
    if (q<=start+leng || leng==std::string::npos){ max=i; }
    if (min==std::string::npos || max==std::string::npos) { return ""; }
    return str.substr(min,max);
}

void show_message(std::string header, int sym_num = 32, std::string distance="Komoot",
                  std::string info="Navigation", float volt=0) {
  // Show message and symbol (max size = 64 w * 48 h)
  const int max_header = 128 / 6;
  header = utf8_substr(header,0,max_header);
  int icon_width = std::max(52,symbols[sym_num].width);
  int x_offset = std::max(0,24 - symbols[sym_num].width / 2);
  int y_offset = 40 - symbols[sym_num].height / 2;
  Serial.print("Icon x-offset: ");
  Serial.print(x_offset);
  Serial.print(", y-offset: ");
  Serial.println(y_offset);
  // need to set font before getting the message width
  u8g2.setFont(u8g2_font_6x13_te);
  int header_pos_x = std::max(0,63 - u8g2.getUTF8Width(header.c_str()) / 2);
  int info_pos_x = icon_width + std::max(0,(127 - icon_width - u8g2.getUTF8Width(info.c_str())) / 2);
  Serial.print("Header length:");
  Serial.print(header.length());
  Serial.print("width: ");
  Serial.print(u8g2.getUTF8Width(distance.c_str()));
  Serial.print(", offset: ");
  Serial.println(header_pos_x);
  u8g2.setFont(u8g2_font_logisoso16_tr);  // width 10, heigth 16
  int dist_pos_x = icon_width + std::max(0,(127 - icon_width - u8g2.getUTF8Width(distance.c_str())) / 2);
  Serial.print("Offset info: ");
  Serial.print(info_pos_x);
  Serial.print(", distance: ");
  Serial.print(dist_pos_x);
  Serial.print(", dist width: ");
  Serial.println(u8g2.getUTF8Width(distance.c_str()));
  int max_dist = (128 - dist_pos_x) / 10;
  distance = utf8_substr(distance,0,max_dist);
  Serial.print("Dist max length: ");
  Serial.println(max_dist);
  int max_info = (128 - info_pos_x) / 6;
  info = utf8_substr(info,0,max_info);
  Serial.print("Info max length: ");
  Serial.println(max_info);
  // use full buffer mode
  u8g2.clearBuffer();
  u8g2.drawXBMP(x_offset,y_offset,
                symbols[sym_num].width, symbols[sym_num].height,
                symbols[sym_num].xmp_bitmap);
  u8g2.setFont(u8g2_font_logisoso16_tr);
  //dist_pos_x = 52;
  u8g2.setCursor(dist_pos_x, 42);
  u8g2.print(distance.c_str());
  u8g2.setFont(u8g2_font_6x13_te);
  //header_pos_x = 0;
  u8g2.setCursor(header_pos_x, 12);
  u8g2.print(header.c_str());
  // show extra info (e.g. current street)
  u8g2.setFont(u8g2_font_6x13_te);
  //info_pos_x = 52;
  u8g2.setCursor(info_pos_x, 58);
  u8g2.print(info.c_str());
  // show battery voltage
  // draw a line from X=52 to x=127 for full battery (75 px)
  // Vmin = 3.0 Vmax=4.2 (delta 1.2)
  int batt_length = int((volt - 3.0) / 1.2 * 75);
  batt_length = _min (batt_length,75);
  u8g2.drawHLine(126-batt_length,63,batt_length);
  u8g2.sendBuffer();  // full buffer mode
  Serial.print ("Show message: ");
  Serial.println (header.c_str());
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
  BLEClient* pClient  = BLEDevice::createClient(); // or use global pClient ?
  Serial.println(" - Create client");
  // Connect to the remote BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.print (" - Refernce to service obtained");
  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println (" - Refernce to characteristics obtained");
  delay(333); // 1s delay (reduced clock)
  if (pRemoteCharacteristic->canRead()) {
    Serial.println (" - can read");
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  } else {
    Serial.println (" - can NOT notify");
  }

  delay(333); // 1s delay (reduced clock)
  if (pRemoteCharacteristic->canNotify()) {
    Serial.println (" - can notify");
    pRemoteCharacteristic->registerForNotify(notifyCallback);
  } else {
    Serial.println (" - can NOT notify");
  }

  connected = true;
  // Display that BLE has been connected
  show_message("BLE connected",34);
  Serial.println ("Connected to desired service on BLE server");
  return true;
}

// Scan for BLE servers and find the first one that advertises the service we are looking for.
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    //Called for each advertising BLE server.
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.print("BLE Advertised Device found: ");
      Serial.println(advertisedDevice.toString().c_str());
      // We have found a device, let us now see if it contains the service we are looking for.
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
        BLEDevice::getScan()->stop();
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnect = true;
        doScan = true;
      } // Found our server
    } // onResult
}; // MyAdvertisedDeviceCallbacks

// Main program setup
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting Arduino BLE Client application...");

  // reduce clock speed to save power
  // supported values = 2M (no BLE), 80M, 120M, 240M
  rtc_cpu_freq_config_t freq_conf;
  rtc_clk_cpu_freq_to_config(RTC_CPU_FREQ_80M, &freq_conf);
  rtc_clk_cpu_freq_set_config(&freq_conf);

  touchAttachInterrupt(T3, callback, Threshold); // Touch 3 = GPIO 15
  esp_sleep_enable_touchpad_wakeup();

  pinMode(battPin, INPUT);
  raw  = analogRead(battPin);
  volt = raw / vScale1;
  Serial.print ("Battery = ");
  Serial.println (volt);

  u8g2.begin();
  u8g2.setFlipMode(rotation);
  // enable UTF8 support for the Arduino print() function
  u8g2.enableUTF8Print();
  u8g2.setFontDirection(0);

  if (volt < 3.2) { //sleep below 3.2 V
    show_message("Switching off now...",36,"LOW","battery",volt);
    delay(999);
    u8g2.setPowerSave(1);
    //esp_wifi_stop();
    esp_deep_sleep_start();
  }

  // Welcome screen
  show_message("©2018 Matthias Homann"); // developer
  delay(500);
  String v_str = "Battery: " + String(volt,1) + "V";
  show_message(v_str.c_str()); // battery status
  delay(500);

  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);

  // show BLS status "try to connect"
  show_message("BLE try to connect",33);

  uint32_t scan_time = millis();
  pBLEScan->start(30, false); // try 30s to find a device
  scan_time = millis() -scan_time;
  Serial.print("Scan time: ");
  Serial.println(scan_time/1000);
  if (scan_time > 29000) {
    // timeout
    show_message("No BLE, will turn off",35);
    delay(999); // 3s delay, 1000/3 for clock 80M=240M/3
    //ESP.restart();
    u8g2.setPowerSave(1);
    //esp_wifi_stop();
    esp_deep_sleep_start();
  }
} // End of setup.

// Main program loop
void loop() {
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    Serial.print("connectToServer: ");
    if (connectToServer()) {
      connected = true;
      Serial.println("done");
    } else {
      connected = false;
      Serial.println("failed");
    }
    doConnect = false;
  }
  // poweroff if timeout exceeded
  if (timeout > 30) {
     // show BLS status "disconnected"
      show_message("BLE disconnected",36);
      connected = false;
      delay(999); // 3s delay, 1000/3 for clock 80M=240M/3
      //ESP.restart();
      u8g2.setPowerSave(1);
      //esp_wifi_stop();
      esp_deep_sleep_start();
  }

  if (new_notify || heartbeat > 4) {  // refresh at least every 4 seconds
    new_notify = false;
    if (new_street || heartbeat > 4) {  // get full new street, at least every 4 seconds (to keep BLE connection)
      heartbeat = 0;
      // Read the latest value of the characteristic.
      value = pRemoteCharacteristic->readValue();
      if (new_street) {
        new_street = false;
        street_old = street;
        street = value.substr(9);
        Serial.print ("Street: ");
        Serial.println (street.c_str());
      }
    }
    // calculate the distance to next fork
    // ignore 4th Byte, assume distance is below 16777 km ;-)
    double dist = int(old_data[5])+int(old_data[6])*256+int(old_data[7])*65536;
    int digits = 0;
    std::string dist_unit = " m";
    if (dist > 1000) { // km unit
      dist = dist / 1000;
      dist_unit = " km";
      if (dist < 2000) { // show 100m steps
        digits = 1;
      }
    } else {  // m unit
      if (dist > 200) { // 50 m steps
        dist = int(dist / 50) * 50;  // round down
      } else // 10m steps
        dist = int(dist / 10) * 10;  // round down
    }
    dist_unit = String(dist,digits).c_str() + dist_unit;
    Serial.print ("Distance: ");
    Serial.println (dist);
    // std::string street = value.substr(9);
    // get battery voltage
    raw  = analogRead(battPin);
    volt = raw / vScale2;
    if (volt < 3.1) { //sleep below 3.1 V
      show_message("Switching off now...",36,"LOW","battery",volt);
      delay(999);
      u8g2.setPowerSave(1);
      //esp_wifi_stop();
      esp_deep_sleep_start();
    }

    show_message(street.c_str(),old_data[4],dist_unit,street_old.c_str(),volt);

    u8g2.setFont(u8g2_font_6x13_te);
    int text_offset = 64 - u8g2.getUTF8Width(street.c_str()) / 2;
    text_offset = std::max(0, text_offset);
    u8g2.firstPage();
  }
  timeout++;
  heartbeat++;
  delay(333); // 1s delay, 1000/3 for clock 80M=240M/3
} // End of loop
