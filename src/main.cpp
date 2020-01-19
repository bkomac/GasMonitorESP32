#include <Arduino.h>
#include "driver/rtc_io.h"
#include <BLE2902.h>
#include <BLEDevice.h>
#include <WiFi.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <soc/rtc.h>

#include "driver/adc.h"
#include "esp_adc_cal.h"

#define INVERT_BUILTINLED false
#define BUILTINLED 5

void blink();
void blink(int times);
void blink(int times, int milisec);
void blink(int times, int himilisec, int lowmilisec);
String getMacAddress();
bool connectToServer();
bool sendMsg(float deltaVol);
void startBleClientScan();

// BLE
static BLEUUID serviceUUID("f61b2de1-7656-4d9a-a4f5-37c495071ae8");
static BLEUUID charUUID("c7756044-41f0-4695-acc3-86bd78f3d6a6");
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *myDevice;

int hallPin = 32;
int hallState = 1;

String appId = "xx";
String ver = "1.0.3";
RTC_DATA_ATTR long lastTime = millis();
long sendInterval = 1000 * 30;
RTC_DATA_ATTR bool pulseDetected = false;
float pulseVol = 0.01;

RTC_DATA_ATTR int numPulses = 0;
RTC_DATA_ATTR float deltaVol = 0;
uint32_t vcc = 0;

RTC_DATA_ATTR long sleepLastTime = millis();
long sleepInterval = 1000 * 10;

long heartBeatInterval = 1000 * 5;
long heartBeatLastTime = millis();

// deep sleep wake up function
void RTC_IRAM_ATTR esp_wake_deep_sleep(void)
{
  esp_default_wake_deep_sleep();
  Serial.println("Woke up from deep sleep!");
}

// BLE
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                           uint8_t *pData, size_t length, bool isNotify)
{
  Serial.println("Notify callback for characteristic \n");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
  Serial.print("data: ");
  Serial.println((char *)pData);
}

class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient *pclient)
  {
    Serial.println("\nBLE connected!");
    blink(3);
  }

  void onDisconnect(BLEClient *pclient)
  {
    connected = false;
    Serial.println("\nBLE disconnected!");
    blink(10, 10);
    //startBleClientScan();
  }
};

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  /**
    * Called for each advertising BLE server.
    */
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    Serial.print("\nBLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are
    // looking for.
    if (advertisedDevice.haveServiceUUID() &&
        advertisedDevice.isAdvertisingService(serviceUUID))
    {
      // if (advertisedDevice.getName() == "ESP32 BLE Gateway") {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
      Serial.print("\nBLE connecting to: ");
      Serial.println(advertisedDevice.toString().c_str());
      Serial.println(advertisedDevice.getServiceUUID().toString().c_str());
    }
    else
    {
      Serial.println("\nService not found!");
      doScan = true;
    }
  } // onResult
};  // MyAdvertisedDeviceCallbacks

void setup()
{
  //rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
  delay(1000);

  //WiFi.mode(WIFI_OFF);
  WiFi.disconnect();

  blink(3, 200);
  appId = getMacAddress(); //.substring(0,6);
  Serial.println("\nGas Monitor v." + String(ver) + " " + String(appId));

  pinMode(hallPin, INPUT);

  Serial.begin(115200);

  Serial.println("\nStarting Arduino BLE Client..." + String(appId));

  startBleClientScan();

  lastTime = millis() - sendInterval - 1;

  // vcc = adc1_get_raw(ADC1_CHANNEL_5);
  // Serial.println("**** Vcc=" + String(vcc));
}

void startBleClientScan()
{
  Serial.println("\nScaning ...");
  BLEDevice::init("ESP");
  BLEScan *pBLEScan = BLEDevice::getScan();
  delay(200);
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

// 30AEA4F8136C
void sleep()
{
  Serial.println("\nlight_sleep_enter dV=" + String(deltaVol) + " millis=" +
                 String(millis()));

  delay(1000);
  gpio_wakeup_enable(GPIO_NUM_32, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  // esp_sleep_enable_timer_wakeup(milliSec * 1000); // 10 seconds
  int ret = 0;
  esp_deep_sleep_start(); // esp_light_sleep_start();
  Serial.println("woke up: " + String(ret) + " millis: " + String(millis()));
  rtc_gpio_deinit(GPIO_NUM_32);
  // pulseDetected = true;
  deltaVol += pulseVol;
  numPulses++;
  delay(200);
  // startBleClientScan();
}

// the loop function runs over and over again forever
void loop()
{

  if (heartBeatLastTime + heartBeatInterval < millis())
  {
    blink(1, 15);
    heartBeatLastTime = millis();
  }

  if (sleepLastTime + sleepInterval < millis() && numPulses == 0)
  {
    // sleep();
    sleepLastTime = millis();
  }

  hallState = digitalRead(hallPin);

  if (doConnect == true)
  {
    if (connectToServer())
    {
      Serial.println("We are now connected to the BLE Server.");
      doConnect = false;
    }
    else
    {
      Serial.println(
          "We have failed to connect to the server; there is nothing "
          "more we will do.");
    }
    //doConnect = false;
  }

  if (hallState == 0)
  {
    if (!pulseDetected)
    {
      blink(2);
      deltaVol += pulseVol;
      numPulses++;
      pulseDetected = true;
      sleepLastTime = millis();
    }
    Serial.print("\nPulse detected: deltaVol=" + String(deltaVol));
  }
  else
  {
    Serial.print("." + String(hallState));
    pulseDetected = false;
  }

  if (lastTime + sendInterval < millis())
  {
    bool ret = sendMsg(deltaVol);
    delay(100);
    if (ret)
    {
      deltaVol = 0;
      numPulses = 0;
      lastTime = millis();
    }
    else
    {
      Serial.println("\nNot sending  " + String(deltaVol));
    }
  }
  delay(100);
}

bool sendMsg(float dVol)
{
  if (connected)
  {
    String newValue = appId + ";" + String(dVol);
    Serial.println("\nSending  " + newValue);

    pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
    blink(2, 300);
    return true;
  }
  else if (doScan)
  {
    Serial.println("\nNot connected to BLE. Scanning...");
    BLEDevice::getScan()->start(0);
    //startBleClientScan();
    return false;
  }
  else
  {
    Serial.println("\nNot connected to BLE.");
    //connectToServer();
    return false;
  }
}

bool connectToServer()
{
  Serial.print("\nForming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient *pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(myDevice); // if you pass BLEAdvertisedDevice instead of
                              // address, it will be recognized type of peer
                              // device address (public or private)

  Serial.println(" - Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr)
  {
    Serial.print("\nFailed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  // Obtain a reference to the characteristic in the service of the remote BLE
  // server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr)
  {
    Serial.println("\nFailed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  // Read the value of the characteristic.
  if (pRemoteCharacteristic->canRead())
  {
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("\nThe characteristic value was: ");
    Serial.println(value.c_str());
  }

  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  connected = true;
  return true;
}

String getMac()
{
  unsigned char mac[6];
  WiFi.macAddress(mac);
  String result;
  for (int i = 0; i < 6; ++i)
  {
    result += String(mac[i], 16);
  }
  return result;
}

String getMacAddress()
{
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  char baseMacChr[18] = {0};
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1],
          baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  return String(baseMacChr);
}

// blink
void blink() { blink(1, 30, 30); }

void blink(int times) { blink(times, 30, 30); }

void blink(int times, int milisec) { blink(times, milisec, milisec); }

void blink(int times, int himilisec, int lowmilisec)
{
  pinMode(BUILTINLED, OUTPUT);
  for (int i = 0; i < times; i++)
  {
    if (INVERT_BUILTINLED)
      digitalWrite(BUILTINLED, HIGH);
    else
      digitalWrite(BUILTINLED, LOW);
    delay(lowmilisec);
    if (INVERT_BUILTINLED)
      digitalWrite(BUILTINLED, LOW);
    else
      digitalWrite(BUILTINLED, HIGH);
    delay(himilisec);
  }
}
