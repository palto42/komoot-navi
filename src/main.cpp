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

 #define DEBUG true // flag to turn on/off debugging

#include <Arduino.h>
#define Serial if(DEBUG)Serial

#include <string>

#include "BLEDevice.h"
#include "soc/rtc.h"

#include<U8g2lib.h>
#include<Wire.h>

#define Threshold 75 /* touch pin threshold, greater the value = more the sensitivity */

// U8g2 Contructor
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // 0.8" OLED
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // 1.3" OLED
const int rotation = 1;  // display rotaion: 0=none, 1=180deg

#include <navi_sym_48.h>  // local library "symbols"
#include <ble_sym_48.h>

// Komoot Connect service and characteristics
static BLEUUID serviceUUID("71C1E128-D92F-4FA8-A2B2-0F171DB3436C"); // navigationServiceUUID
static BLEUUID charUUID("503DD605-9BCB-4F6E-B235-270A57483026");    // navigationServiceNotifyCharacteristicUUI
static BLEUUID heartUUID("6D75DBF0-D763-4147-942A-D97B1BC700CF");   // navigationServiceHeartbeatWriteCharacteristicUUID

static BLEAddress *pServerAddress;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static boolean doConnect = false;
static boolean connected = false;

boolean new_notify = false;
boolean new_street = false;
int heartbeat = 0;
int timeout = 0;
int scan_time = 0;
uint8_t old_data[20];
std::string value = "Start";

const int battPin = 35; // A2=2 A6=34
unsigned int raw=0;
float volt=0.0;
const float vScale1 = 225.0;
const float vScale2 = 245.0;

std::string street = "";
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
    for (int i = 9; i < 20; i++) {  // 4 = direction, 5-8 = distance, 9.. street
      if (pData[i] != old_data[i]) {
        new_notify = true;
        new_street = true;
        memcpy(old_data,pData,20);
        break;
      }
    }
    if (! new_street) {
      for (int i = 4; i < 9; i++) {  // 4 = direction, 5-8 = distance, 9.. street
        if (pData[i] != old_data[i]) {
          new_notify = true;
          memcpy(old_data,pData,20);
          break;
        }
      }
    }
}

void welcome(int message) {
  // Welcome screen
  u8g2.firstPage();
  do {
    u8g2.drawXBMP(0,0,48,48,navi_sym[0]);
    u8g2.setFont(u8g2_font_logisoso18_tr);
    //u8g2.setFont(u8g2_font_logisoso22_tr);
    u8g2.setCursor(50, 22);
    u8g2.print("Komoot");
    u8g2.setCursor(70, 44);
    u8g2.print("Navi");
    u8g2.setFont(u8g2_font_6x13_te);
    u8g2.setCursor(0, 62);
    if ( message == 0) {
      u8g2.print("©2018 Matthias Homann");
    } else {
      u8g2.print("Akku: ");
      u8g2.print(volt);
      u8g2.print(" V");
    }
  } while( u8g2.nextPage() );
  Serial.print ("Show messge #");
  Serial.println (message);
}

void ble_connect(int status) {
  // BLE connection status screen
  u8g2.firstPage();
  do {
    u8g2.drawXBMP(0,0,48,48,ble_sym[status]);  // 0=BLE, 1= connected
    // show battery voltage
    // draw a line from X=52 to x=127 for full battery (75 px)
    // Vmin = 3.0 Vmax=4.2 (delta 1.2)
    int batt_length = int((volt - 3.0) / 1.2 * 75);
    batt_length = _min (batt_length,75);
    u8g2.setFont(u8g2_font_logisoso18_tr);
    //u8g2.setFont(u8g2_font_logisoso22_tr);
    u8g2.setCursor(50, 22);
    u8g2.print("Komoot");
    u8g2.setCursor(70, 44);
    u8g2.print("Navi");
    u8g2.setFont(u8g2_font_6x13_te);
    u8g2.setCursor(0, 62);
    if ( status == 0 ) {
      u8g2.print("BLE try to connect");
    } else if (status == 1) {
      u8g2.print("BLE connected");
    } else { // == 2
      u8g2.print("BLE disconnected");
    }
  } while( u8g2.nextPage() );
  Serial.print ("Show BLE icon #");
  Serial.println (status);
}

bool connectToServer(BLEAddress pAddress) {
  BLEClient* pClient  = BLEDevice::createClient(); // or use global pClient ?
  // Connect to the remove BLE Server.
  pClient->connect(pAddress);
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    return false;
  }
  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    return false;
  }
  // Read the value of the characteristic.
  std::string value = pRemoteCharacteristic->readValue();
  pRemoteCharacteristic->registerForNotify(notifyCallback);
  // Display that BLE has been connected
  ble_connect(1);
  Serial.println ("Connected to desired service on BLE server");
}

// Scan for BLE servers and find the first one that advertises the service we are looking for.
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    //Called for each advertising BLE server.
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      // We have found a device, let us now see if it contains the service we are looking for.
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID)) {
        advertisedDevice.getScan()->stop();
        pServerAddress = new BLEAddress(advertisedDevice.getAddress());
        doConnect = true;
      } // Found our server
    } // onResult
}; // MyAdvertisedDeviceCallbacks

// Main program setup
void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

  // reducte clock speed to save power
  // supported values = 2M (no BLE), 80M, 120M, 240M
  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);

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

  // Welcome screen
  welcome(0); // developer
  delay(500);
  welcome(1); // battery status
  delay(500);

  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);

  // show BLS status "try to connect"
  ble_connect(0);

  pBLEScan->start(30); // try 30s to find a device

} // End of setup.

// Main program loop
void loop() {
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      connected = true;
    } else {
      connected = false;
    }
    doConnect = false;
  }
  // poweroff if no server found after scan, could not connect or timeout
  if (timeout > 30 or (not doConnect and not connected )) { // (not doConnect and not connected ) or
     // show BLS status "disconnected"
      ble_connect(2);
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
    String dist_unit = " m";
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
    Serial.print ("Distance: ");
    Serial.println (dist);
    Serial.println (dist_unit);
    // std::string street = value.substr(9);
    // get battery voltage
    raw  = analogRead(battPin);
    volt = raw / vScale2;
    if (volt < 3.1) { //sleep below 3.1 V
      u8g2.setPowerSave(1);
      //esp_wifi_stop();
      esp_deep_sleep_start();
    }
    u8g2.firstPage();
    do {
      u8g2.drawXBMP(0,0,48,48,navi_sym[old_data[4]]);
      // show battery voltage
      // draw a line from X=52 to x=127 for full battery (75 px)
      // Vmin = 3.0 Vmax=4.2 (delta 1.2)
      int batt_length = int((volt - 3.0) / 1.2 * 75);
      batt_length = _min (batt_length,75);
      u8g2.drawHLine(127-batt_length,0,batt_length);
      // show previous street name at top (or bottom?)
      u8g2.setFont(u8g2_font_6x13_te);
      u8g2.setCursor(52, 16);
      u8g2.print(street_old.c_str());
      // show distance
      u8g2.setFont(u8g2_font_logisoso18_tr);
      //u8g2.setFont(u8g2_font_logisoso22_tr);
      u8g2.setCursor(52, 44);
      u8g2.print(dist,digits);
      u8g2.print(dist_unit);
      // show street name
      u8g2.setFont(u8g2_font_6x13_te);
      u8g2.setCursor(0, 62);
      u8g2.print(street.c_str());
    } while( u8g2.nextPage() );
  }
  timeout++;
  heartbeat++;
  delay(333); // 1s delay, 1000/3 for clock 80M=240M/3
} // End of loop
