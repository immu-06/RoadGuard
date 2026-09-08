#include <Wire.h>
#include <math.h>
#include <TinyGPSPlus.h>

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>


#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>



#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  -1
);


// =====================================================
// MPU6050
// =====================================================

#define MPU_ADDR 0x68


// =====================================================
// GPS
// =====================================================

#define GPS_RX 18
#define GPS_TX 19

HardwareSerial GPSserial(1);

TinyGPSPlus gps;


// =====================================================
// VEHICLE
// =====================================================

#define VEHICLE_TYPE "CAR"


// =====================================================
// CRASH THRESHOLDS
// =====================================================

#define IMPACT_THRESHOLD 2.5
#define GYRO_THRESHOLD 250.0

#define HIGH_IMPACT 4.0
#define HIGH_GYRO 400.0


// =====================================================
// MPU VARIABLES
// =====================================================

int16_t AcX;
int16_t AcY;
int16_t AcZ;

int16_t GyX;
int16_t GyY;
int16_t GyZ;

float ax = 0.0;
float ay = 0.0;
float az = 0.0;

float gx = 0.0;
float gy = 0.0;
float gz = 0.0;

float acceleration = 0.0;
float gyro = 0.0;


// =====================================================
// GPS VARIABLES
// =====================================================

double lastLatitude = 0.0;
double lastLongitude = 0.0;

bool gpsFixAvailable = false;

int satelliteCount = 0;


// =====================================================
// VEHICLE DATA
// =====================================================

float vehicleSpeed = 0.0;
float motionIntensity = 0.0;


// =====================================================
// CRASH-TIME SENSOR DATA
// =====================================================

float crashAccX = 0.0;
float crashAccY = 0.0;
float crashAccZ = 0.0;

float crashGyroX = 0.0;
float crashGyroY = 0.0;
float crashGyroZ = 0.0;

float crashSpeed = 0.0;
float crashMotionIntensity = 0.0;


// =====================================================
// BLE UUIDS
// KEEP THESE EXACTLY THE SAME
// =====================================================

#define ROADGUARD_SERVICE_UUID \
"4fafc201-1fb5-459e-8fcc-c5c9c331914b"

#define ROADGUARD_CHARACTERISTIC_UUID \
"beb5483e-36e1-4688-b7f5-ea07361b26a8"



// =====================================================
// ESP-NOW RELAY TO NODE B
// =====================================================
// Replace these six bytes with NODE B's STA MAC address.
// Example: 24:6F:28:AA:BB:CC
uint8_t NODE_B_MAC[] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};

bool espNowReady = false;

void sendCrashESPNow(const String &packet)
{
  if (!espNowReady)
  {
    Serial.println("ESP-NOW NOT READY - PACKET NOT RELAYED");
    return;
  }

  esp_err_t result = esp_now_send(
    NODE_B_MAC,
    (const uint8_t *)packet.c_str(),
    packet.length() + 1
  );

  if (result == ESP_OK)
  {
    Serial.println("ESP-NOW: CRASH PACKET SENT TO NODE B");
  }
  else
  {
    Serial.print("ESP-NOW SEND FAILED: ");
    Serial.println(result);
  }
}

void onEspNowSent(
  const uint8_t *mac_addr,
  esp_now_send_status_t status
)
{
  Serial.print("ESP-NOW DELIVERY: ");
  Serial.println(
    status == ESP_NOW_SEND_SUCCESS
      ? "SUCCESS"
      : "FAILED"
  );
}

void setupESPNow()
{
  WiFi.mode(WIFI_STA);

  Serial.print("NODE A MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  esp_now_register_send_cb(onEspNowSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(
    peerInfo.peer_addr,
    NODE_B_MAC,
    6
  );

  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("ESP-NOW PEER ADD FAILED");
    return;
  }

  espNowReady = true;

  Serial.println("ESP-NOW READY");
  Serial.print("NODE B MAC: ");
  for (int i = 0; i < 6; i++)
  {
    if (i > 0) Serial.print(":");
    if (NODE_B_MAC[i] < 16) Serial.print("0");
    Serial.print(NODE_B_MAC[i], HEX);
  }
  Serial.println();
}


// =====================================================
// BLE VARIABLES
// =====================================================

BLEServer *bleServer = nullptr;

BLEAdvertising *advertising = nullptr;

BLEService *crashService = nullptr;

BLECharacteristic *crashCharacteristic = nullptr;



// =====================================================
// CRASH STATE
// =====================================================

bool crashDetected = false;

int crashNumber = 0;

String lastCrashPacket = "";


// =====================================================
// OLED STATE
// =====================================================

unsigned long lastOLEDUpdate = 0;

unsigned long animationTimer = 0;

int animationFrame = 0;


// =====================================================
// BLE SERVER CALLBACKS
// =====================================================

class NodeAServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *server) override
  {
    Serial.println();
    Serial.println("================================");
    Serial.println("BLE CLIENT CONNECTED");
    Serial.println("================================");

    // If a crash packet was generated before the phone connected,
    // keep it in the characteristic and send it after connection.
    if (lastCrashPacket.length() > 0)
    {
      delay(300);

      crashCharacteristic->setValue(
        lastCrashPacket.c_str()
      );

      crashCharacteristic->notify();

      Serial.println("STORED CRASH PACKET FOUND");
      Serial.println("SENDING STORED CRASH PACKET");
      Serial.println(lastCrashPacket);
      Serial.println("STORED CRASH PACKET SENT");
      Serial.println("================================");
    }
  }

  void onDisconnect(BLEServer *server) override
  {
    Serial.println();
    Serial.println("================================");
    Serial.println("BLE CLIENT DISCONNECTED");
    Serial.println("RESTARTING BLE ADVERTISING");
    Serial.println("================================");

    delay(200);

    server->startAdvertising();

    Serial.println("BLE ADVERTISING RESTARTED");
  }
};


// =====================================================
// OLED BASIC
// =====================================================

void clearOLED()
{
  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );
}


// =====================================================
// CENTER TEXT
// =====================================================

void centerText(
  String text,
  int y,
  int size
)
{
  display.setTextSize(size);

  int16_t x1;
  int16_t y1;

  uint16_t w;
  uint16_t h;

  display.getTextBounds(
    text,
    0,
    y,
    &x1,
    &y1,
    &w,
    &h
  );

  int x =
    (SCREEN_WIDTH - w) / 2;

  display.setCursor(
    x,
    y
  );

  display.println(text);
}


void startupAnimation()
{
  for (int i = 0; i < 3; i++)
  {
    clearOLED();

    centerText(
      "ROADGUARD",
      8,
      2
    );

    centerText(
      "SAFETY NODE",
      32,
      1
    );

    display.drawRect(
      8,
      50,
      112,
      8,
      SSD1306_WHITE
    );

    display.fillRect(
      10,
      52,
      (i + 1) * 35,
      4,
      SSD1306_WHITE
    );

    display.display();

    delay(500);
  }

  clearOLED();

  centerText(
    "INITIALIZING",
    20,
    1
  );

  centerText(
    "SYSTEM",
    36,
    2
  );

  display.display();

  delay(700);
}


// =====================================================
// READY SCREEN
// =====================================================

void readyScreen()
{
  clearOLED();

  centerText(
    "ROADGUARD",
    4,
    2
  );

  display.drawLine(
    0,
    25,
    127,
    25,
    SSD1306_WHITE
  );

  centerText(
    "SYSTEM READY",
    31,
    1
  );

  centerText(
    "MONITORING",
    46,
    1
  );

  display.display();

  delay(1200);
}


// =====================================================
// MPU SCREEN
// =====================================================

void showMPUStatus(
  bool connected
)
{
  clearOLED();

  centerText(
    "SENSOR CHECK",
    2,
    1
  );

  display.drawLine(
    0,
    15,
    127,
    15,
    SSD1306_WHITE
  );

  display.setTextSize(1);

  display.setCursor(
    5,
    24
  );

  display.print(
    "MPU6050"
  );

  display.setCursor(
    80,
    24
  );

  if (connected)
    display.print("OK");
  else
    display.print("FAIL");

  display.setCursor(
    5,
    40
  );

  display.print(
    "I2C: 0x68"
  );

  display.display();

  delay(900);
}


// =====================================================
// GPS SEARCH SCREEN
// =====================================================

void showGPSSearch()
{
  clearOLED();

  centerText(
    "GPS SEARCH",
    2,
    1
  );

  display.drawLine(
    0,
    15,
    127,
    15,
    SSD1306_WHITE
  );

  display.setTextSize(1);

  display.setCursor(
    5,
    26
  );

  display.print(
    "SATELLITES: "
  );

  display.print(
    satelliteCount
  );

  display.setCursor(
    5,
    42
  );

  display.print(
    "STATUS: "
  );

  int dots =
    (millis() / 400) % 4;

  display.print(
    "WAIT"
  );

  for (int i = 0; i < dots; i++)
    display.print(".");

  display.display();
}


// =====================================================
// GPS FIX SCREEN
// =====================================================

void showGPSFix()
{
  clearOLED();

  centerText(
    "GPS LOCKED",
    1,
    1
  );

  display.drawLine(
    0,
    14,
    127,
    14,
    SSD1306_WHITE
  );

  display.setTextSize(1);

  display.setCursor(
    2,
    21
  );

  display.print(
    "LAT:"
  );

  display.println(
    lastLatitude,
    4
  );

  display.setCursor(
    2,
    35
  );

  display.print(
    "LNG:"
  );

  display.println(
    lastLongitude,
    4
  );

  display.setCursor(
    2,
    49
  );

  display.print(
    "SAT:"
  );

  display.print(
    satelliteCount
  );

  display.print(
    " GPS:OK"
  );

  display.display();

  delay(1200);
}


// =====================================================
// NORMAL MONITORING SCREEN
// =====================================================

void showMonitoring()
{
  clearOLED();

  centerText(
    "ROADGUARD",
    0,
    1
  );

  display.drawLine(
    0,
    12,
    127,
    12,
    SSD1306_WHITE
  );

  display.setTextSize(1);

  display.setCursor(
    2,
    17
  );

  display.print(
    "ACC "
  );

  display.print(
    acceleration,
    2
  );

  display.print(
    "G"
  );

  display.setCursor(
    67,
    17
  );

  display.print(
    "ROT "
  );

  display.print(
    gyro,
    0
  );

  display.setCursor(
    2,
    31
  );

  display.print(
    "GPS:"
  );

  if (gpsFixAvailable)
    display.print("FIX");
  else
    display.print("SEARCH");

  display.setCursor(
    67,
    31
  );

  display.print(
    "SAT:"
  );

  display.print(
    satelliteCount
  );

  display.setCursor(
    2,
    46
  );

  display.print(
    "SPD:"
  );

  display.print(
    vehicleSpeed,
    1
  );

  display.print(
    "km/h"
  );

  display.setCursor(
    2,
    58
  );

  display.print(
    "BLE+BT: READY"
  );

  display.display();
}


// =====================================================
// EMERGENCY ANIMATION
// =====================================================

void emergencyAnimation(
  String crashID,
  int severity
)
{
  static bool invert = false;

  invert = !invert;

  display.clearDisplay();

  if (invert)
  {
    display.fillRect(
      0,
      0,
      128,
      64,
      SSD1306_WHITE
    );

    display.setTextColor(
      SSD1306_BLACK
    );
  }
  else
  {
    display.setTextColor(
      SSD1306_WHITE
    );
  }

  centerText(
    "!! CRASH !!",
    0,
    2
  );

  display.setTextSize(1);

  display.setCursor(
    3,
    25
  );

  display.print(
    "ID: "
  );

  display.print(
    crashID
  );

  display.setCursor(
    3,
    38
  );

  display.print(
    "SEVERITY: "
  );

  display.print(
    severity
  );

  display.setCursor(
    3,
    51
  );

  display.print(
    "BLE+BT: ALERT"
  );

  display.display();
}


// =====================================================
// WRITE MPU REGISTER
// =====================================================

void writeMPU(
  byte reg,
  byte data
)
{
  Wire.beginTransmission(
    MPU_ADDR
  );

  Wire.write(reg);
  Wire.write(data);

  Wire.endTransmission();
}


// =====================================================
// READ MPU
// =====================================================

bool readMPU()
{
  Wire.beginTransmission(
    MPU_ADDR
  );

  Wire.write(0x3B);

  if (
    Wire.endTransmission(false) != 0
  )
  {
    return false;
  }

  Wire.requestFrom(
    MPU_ADDR,
    14,
    true
  );

  if (
    Wire.available() != 14
  )
  {
    return false;
  }

  AcX =
    Wire.read() << 8 |
    Wire.read();

  AcY =
    Wire.read() << 8 |
    Wire.read();

  AcZ =
    Wire.read() << 8 |
    Wire.read();

  // Temperature
  Wire.read();
  Wire.read();

  GyX =
    Wire.read() << 8 |
    Wire.read();

  GyY =
    Wire.read() << 8 |
    Wire.read();

  GyZ =
    Wire.read() << 8 |
    Wire.read();

  // ±8G
  ax =
    AcX / 4096.0;

  ay =
    AcY / 4096.0;

  az =
    AcZ / 4096.0;

  // ±500 deg/s
  gx =
    GyX / 65.5;

  gy =
    GyY / 65.5;

  gz =
    GyZ / 65.5;

  acceleration =
    sqrt(
      ax * ax +
      ay * ay +
      az * az
    );

  gyro =
    sqrt(
      gx * gx +
      gy * gy +
      gz * gz
    );

  return true;
}


// =====================================================
// MOTION INTENSITY
// =====================================================

float calculateMotionIntensity()
{
  return sqrt(
    acceleration * acceleration +
    (gyro / 100.0) *
    (gyro / 100.0)
  );
}


// =====================================================
// GPS
// =====================================================

void readGPS()
{
  while (
    GPSserial.available()
  )
  {
    char c =
      GPSserial.read();

    gps.encode(c);
  }

  if (
    gps.location.isValid()
  )
  {
    lastLatitude =
      gps.location.lat();

    lastLongitude =
      gps.location.lng();

    gpsFixAvailable = true;
  }

  if (
    gps.satellites.isValid()
  )
  {
    satelliteCount =
      gps.satellites.value();
  }

  if (
    gps.speed.isValid()
  )
  {
    vehicleSpeed =
      gps.speed.kmph();
  }
}


// =====================================================
// CRASH ID
// =====================================================

String generateCrashID()
{
  crashNumber++;

  String id =
    "C";

  if (
    crashNumber < 10
  )
  {
    id += "00";
  }
  else if (
    crashNumber < 100
  )
  {
    id += "0";
  }

  id +=
    String(
      crashNumber
    );

  return id;
}


// =====================================================
// SEVERITY
// =====================================================

int calculateSeverity(
  float impact,
  float rotation
)
{
  if (
    impact >= 5.0 ||
    rotation >= 450
  )
  {
    return 5;
  }

  if (
    impact >= 4.0 ||
    rotation >= 400
  )
  {
    return 4;
  }

  if (
    impact >= 3.2 ||
    rotation >= 325
  )
  {
    return 3;
  }

  return 2;
}


// =====================================================
// VEHICLE STOPPED
// =====================================================

bool vehicleStopped()
{
  int stableSamples = 0;

  unsigned long start =
    millis();

  while (
    millis() - start < 1000
  )
  {
    readGPS();

    if (!readMPU())
    {
      delay(20);
      continue;
    }

    if (
      acceleration > 0.75 &&
      acceleration < 1.30 &&
      gyro < 60
    )
    {
      stableSamples++;
    }

    delay(50);
  }

  return (
    stableSamples >= 12
  );
}


// =====================================================
// CRASH CONFIRMATION
// =====================================================

bool confirmCrash(
  float initialImpact,
  float initialGyro
)
{
  float peakImpact =
    initialImpact;

  float peakGyro =
    initialGyro;

  // Initial sample
  crashAccX = ax;
  crashAccY = ay;
  crashAccZ = az;

  crashGyroX = gx;
  crashGyroY = gy;
  crashGyroZ = gz;

  crashSpeed =
    vehicleSpeed;

  crashMotionIntensity =
    calculateMotionIntensity();

  for (
    int i = 0;
    i < 10;
    i++
  )
  {
    readGPS();

    if (!readMPU())
    {
      delay(20);
      continue;
    }

    if (
      acceleration >
      peakImpact
    )
    {
      peakImpact =
        acceleration;

      crashAccX = ax;
      crashAccY = ay;
      crashAccZ = az;

      crashSpeed =
        vehicleSpeed;

      crashMotionIntensity =
        calculateMotionIntensity();
    }

    if (
      gyro >
      peakGyro
    )
    {
      peakGyro =
        gyro;

      crashGyroX = gx;
      crashGyroY = gy;
      crashGyroZ = gz;

      crashSpeed =
        vehicleSpeed;

      crashMotionIntensity =
        calculateMotionIntensity();
    }

    delay(50);
  }

  bool stopped =
    vehicleStopped();

  bool strongImpact =
    peakImpact >=
    IMPACT_THRESHOLD;

  bool strongRotation =
    peakGyro >=
    GYRO_THRESHOLD;

  if (
    strongImpact &&
    strongRotation
  )
  {
    return true;
  }

  if (
    peakImpact >= HIGH_IMPACT &&
    stopped
  )
  {
    return true;
  }

  if (
    peakGyro >= HIGH_GYRO &&
    stopped
  )
  {
    return true;
  }

  return false;
}


// =====================================================
// COUNT BLE CLIENTS
//
// This is mainly diagnostic.
// The BLE library handles notification
// delivery to subscribed clients.
// =====================================================

int getBLEClientCount()
{
  if (bleServer == nullptr)
  {
    return 0;
  }

  return bleServer->getConnectedCount();
}


// =====================================================
// BLE CRASH DELIVERY
// =====================================================

void sendCrashBLE(
  String packet
)
{
  if (
    crashCharacteristic == nullptr
  )
  {
    Serial.println(
      "BLE CHARACTERISTIC NOT READY"
    );

    return;
  }

  int clients =
    getBLEClientCount();

  // Always store the latest packet in the characteristic.
  // This allows a client that connects later to read it.
  crashCharacteristic->setValue(
    packet.c_str()
  );

  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "BLE CRASH DELIVERY"
  );

  Serial.print(
    "CONNECTED CLIENTS: "
  );

  Serial.println(
    clients
  );

  Serial.print(
    "PACKET: "
  );

  Serial.println(
    packet
  );

  if (clients > 0)
  {
    crashCharacteristic->notify();

    Serial.println(
      "CRASH NOTIFICATION SENT"
    );
  }
  else
  {
    Serial.println(
      "NO BLE CLIENT CONNECTED"
    );

    Serial.println(
      "BLE PACKET STORED FOR NEXT READ"
    );

    // Packet is already stored in the characteristic above.
    // A client connecting later can read it.
  }

  Serial.println(
    "================================"
  );
}


// =====================================================
// SEND TO ALL AVAILABLE CONNECTION TYPES
// =====================================================

void broadcastCrash(
  String packet
)
{
  lastCrashPacket =
    packet;

  Serial.println();
  Serial.println(
    "################################"
  );

  Serial.println(
    "BLE + CLASSIC CRASH BROADCAST"
  );

  Serial.print(
    "PACKET: "
  );

  Serial.println(
    packet
  );

  // -------------------------------
  // BLE
  // -------------------------------

  sendCrashBLE(
    packet
  );

  Serial.println();
  Serial.println(
    "CRASH DELIVERY COMPLETE"
  );

  Serial.println(
    "################################"
  );
}


// =====================================================
// PROCESS CRASH
// =====================================================

void processCrash(
  float impact,
  float rotation
)
{
  if (
    crashDetected
  )
  {
    return;
  }

  clearOLED();

  centerText(
    "IMPACT!",
    12,
    2
  );

  centerText(
    "VERIFYING...",
    38,
    1
  );

  display.display();

  bool confirmed =
    confirmCrash(
      impact,
      rotation
    );

  if (!confirmed)
  {
    clearOLED();

    centerText(
      "FALSE ALARM",
      20,
      1
    );

    centerText(
      "MONITORING",
      38,
      1
    );

    display.display();

    delay(1200);

    return;
  }

  crashDetected =
    true;


  // ===================================================
  // GPS
  // ===================================================

  readGPS();

  double latitude =
    lastLatitude;

  double longitude =
    lastLongitude;


  // ===================================================
  // CRASH ID
  // ===================================================

  String crashID =
    generateCrashID();


  // ===================================================
  // SEVERITY
  // ===================================================

  int severity =
    calculateSeverity(
      impact,
      rotation
    );


  // ===================================================
  // BUILD CRASH PACKET
  // ===================================================

  String packet =
    "CR|";

  packet +=
    crashID;

  packet +=
    "|";

  packet +=
    String(
      latitude,
      5
    );

  packet +=
    "|";

  packet +=
    String(
      longitude,
      5
    );

  packet +=
    "|";

  packet +=
    String(
      severity
    );

  packet +=
    "|";

  packet +=
    VEHICLE_TYPE;


  // ===================================================
  // ACCELERATION
  // ===================================================

  packet += "|";
  packet += String(
    crashAccX,
    2
  );

  packet += "|";
  packet += String(
    crashAccY,
    2
  );

  packet += "|";
  packet += String(
    crashAccZ,
    2
  );


  // ===================================================
  // GYROSCOPE
  // ===================================================

  packet += "|";
  packet += String(
    crashGyroX,
    2
  );

  packet += "|";
  packet += String(
    crashGyroY,
    2
  );

  packet += "|";
  packet += String(
    crashGyroZ,
    2
  );


  // ===================================================
  // SPEED
  // ===================================================

  packet += "|";

  packet += String(
    crashSpeed,
    2
  );


  // ===================================================
  // MOTION INTENSITY
  // ===================================================

  packet += "|";

  packet += String(
    crashMotionIntensity,
    2
  );


  // ===================================================
  // SERIAL CRASH INFORMATION
  // ===================================================

  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "🚨🚨 ACCIDENT CONFIRMED 🚨🚨"
  );

  Serial.println(
    "================================"
  );

  Serial.print(
    "Crash ID: "
  );

  Serial.println(
    crashID
  );

  Serial.print(
    "Latitude: "
  );

  Serial.println(
    latitude,
    6
  );

  Serial.print(
    "Longitude: "
  );

  Serial.println(
    longitude,
    6
  );

  Serial.print(
    "Severity: "
  );

  Serial.println(
    severity
  );

  Serial.print(
    "AccX: "
  );

  Serial.println(
    crashAccX,
    2
  );

  Serial.print(
    "AccY: "
  );

  Serial.println(
    crashAccY,
    2
  );

  Serial.print(
    "AccZ: "
  );

  Serial.println(
    crashAccZ,
    2
  );

  Serial.print(
    "GyroX: "
  );

  Serial.println(
    crashGyroX,
    2
  );

  Serial.print(
    "GyroY: "
  );

  Serial.println(
    crashGyroY,
    2
  );

  Serial.print(
    "GyroZ: "
  );

  Serial.println(
    crashGyroZ,
    2
  );

  Serial.print(
    "Speed: "
  );

  Serial.print(
    crashSpeed,
    2
  );

  Serial.println(
    " km/h"
  );

  Serial.print(
    "MotionIntensity: "
  );

  Serial.println(
    crashMotionIntensity,
    2
  );

  Serial.print(
    "BLE PACKET: "
  );

  Serial.println(
    packet
  );


  // ===================================================
  // SEND CRASH
  // ===================================================

  // ---------------------------------------------------
  // DIRECT TO PHONE THROUGH NODE A BLE
  // ---------------------------------------------------
  broadcastCrash(
    packet
  );

  // ---------------------------------------------------
  // RELAY TO NODE B THROUGH ESP-NOW
  // ---------------------------------------------------
  sendCrashESPNow(packet);


  // ===================================================
  // EMERGENCY DISPLAY
  // ===================================================

  // Keep the emergency animation active for 10 seconds.
  // Then return to normal monitoring without rebooting.
  unsigned long emergencyStart = millis();

  while (millis() - emergencyStart < 10000)
  {
    readGPS();

    emergencyAnimation(
      crashID,
      severity
    );

    delay(400);
  }

  // ===================================================
  // RETURN TO MONITORING
  // ===================================================

  crashDetected = false;

  clearOLED();
  readyScreen();

  Serial.println();
  Serial.println("================================");
  Serial.println("EMERGENCY DISPLAY COMPLETE");
  Serial.println("RETURNING TO MONITORING");
  Serial.println("WAITING FOR NEXT CRASH...");
  Serial.println("================================");
}


// =====================================================
// START BLE
// =====================================================

void startBLE()
{
  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "STARTING BLE"
  );

  Serial.println(
    "================================"
  );


  BLEDevice::init(
    "ROADGUARD"
  );


  bleServer =
    BLEDevice::createServer();


  bleServer->setCallbacks(
    new NodeAServerCallbacks()
  );


  // ---------------------------------------------------
  // SERVICE
  // ---------------------------------------------------

  crashService =
    bleServer->createService(
      ROADGUARD_SERVICE_UUID
    );


  // ---------------------------------------------------
  // CHARACTERISTIC
  // ---------------------------------------------------

  crashCharacteristic =
    crashService->createCharacteristic(
      ROADGUARD_CHARACTERISTIC_UUID,

      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY
    );


  // ---------------------------------------------------
  // NOTIFICATION DESCRIPTOR
  // ---------------------------------------------------

  crashCharacteristic->addDescriptor(
    new BLE2902()
  );


  // ---------------------------------------------------
  // INITIAL VALUE
  // ---------------------------------------------------

  crashCharacteristic->setValue(
    "ROADGUARD_READY"
  );


  // ---------------------------------------------------
  // START SERVICE
  // ---------------------------------------------------

  crashService->start();


  // ---------------------------------------------------
  // ADVERTISING
  // ---------------------------------------------------

  advertising =
    BLEDevice::getAdvertising();


  advertising->addServiceUUID(
    ROADGUARD_SERVICE_UUID
  );


  advertising->setScanResponse(
    true
  );


  advertising->start();


  Serial.println(
    "BLE ADVERTISING STARTED"
  );

  Serial.println(
    "BLE DEVICE NAME: ROADGUARD"
  );

  Serial.print(
    "SERVICE UUID: "
  );

  Serial.println(
    ROADGUARD_SERVICE_UUID
  );

  Serial.print(
    "CHARACTERISTIC UUID: "
  );

  Serial.println(
    ROADGUARD_CHARACTERISTIC_UUID
  );
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(
    115200
  );

  delay(1000);

  // ===================================================
  // ESP-NOW
  // ===================================================
  setupESPNow();


  // ===================================================
  // STARTUP SERIAL
  // ===================================================

  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "ROADGUARD NODE A"
  );

  Serial.println(
    "CRASH DETECTION NODE"
  );

  Serial.println(
    "================================"
  );


  // ===================================================
  // START BLE FIRST
  // ===================================================

  startBLE();


    // ===================================================
  // I2C
  // ===================================================

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  delay(100);


  // ===================================================
  // OLED
  // ===================================================

  if (
    !display.begin(
      SSD1306_SWITCHCAPVCC,
      OLED_ADDR
    )
  )
  {
    Serial.println(
      "OLED NOT FOUND!"
    );

    while (true)
    {
      delay(1000);
    }
  }

  Serial.println(
    "OLED FOUND"
  );


  startupAnimation();


  // ===================================================
  // MPU6050 CHECK
  // ===================================================

  Serial.println(
    "Checking MPU6050..."
  );

  Wire.beginTransmission(
    MPU_ADDR
  );

  if (
    Wire.endTransmission() != 0
  )
  {
    clearOLED();

    centerText(
      "MPU ERROR",
      18,
      2
    );

    centerText(
      "CHECK WIRING",
      42,
      1
    );

    display.display();

    Serial.println(
      "MPU6050 NOT FOUND!"
    );

    while (true)
    {
      delay(1000);
    }
  }

  Serial.println(
    "MPU6050 FOUND"
  );

  showMPUStatus(
    true
  );


  // ===================================================
  // MPU CONFIGURATION
  // ===================================================

  // Wake up
  writeMPU(
    0x6B,
    0x00
  );

  // Accelerometer ±8G
  writeMPU(
    0x1C,
    0x10
  );

  // Gyroscope ±500 deg/s
  writeMPU(
    0x1B,
    0x08
  );


  // ===================================================
  // GPS
  // ===================================================

  GPSserial.begin(
    9600,
    SERIAL_8N1,
    GPS_RX,
    GPS_TX
  );

  Serial.println(
    "GPS UART STARTED"
  );


  // ===================================================
  // GPS SEARCH
  // ===================================================

  unsigned long gpsStart =
    millis();

  while (
    millis() - gpsStart < 5000
  )
  {
    readGPS();

    showGPSSearch();

    delay(250);
  }


  if (
    gpsFixAvailable
  )
  {
    showGPSFix();
  }


  // ===================================================
  // READY
  // ===================================================

  readyScreen();

  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "ROADGUARD VEHICLE NODE READY"
  );

  Serial.println(
    "================================"
  );

  Serial.println(
    "MPU6050 : ONLINE"
  );

  Serial.println(
    "GPS     : ONLINE"
  );

  Serial.println(
    "OLED    : ONLINE"
  );

  Serial.println(
    "BLE     : ONLINE"
  );

  Serial.println(
    "BT      : ONLINE"
  );

  Serial.println();

  Serial.println(
    "DEVICE NAME: ROADGUARD"
  );

  Serial.println(
    "WAITING FOR CRASH..."
  );

  Serial.println(
    "================================"
  );
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  // ===================================================
  // GPS CONTINUOUSLY
  // ===================================================

  readGPS();


  // ===================================================
  // CRASH ALREADY DETECTED
  // ===================================================

  if (
    crashDetected
  )
  {
    delay(1000);

    return;
  }


  // ===================================================
  // MPU
  // ===================================================

  if (
    !readMPU()
  )
  {
    Serial.println(
      "MPU READ ERROR"
    );

    delay(100);

    return;
  }


  // ===================================================
  // OLED
  // ===================================================

  if (
    millis() -
    lastOLEDUpdate >=
    250
  )
  {
    lastOLEDUpdate =
      millis();

    showMonitoring();
  }


  // ===================================================
  // SERIAL MONITOR
  // ===================================================

  static unsigned long lastPrint =
    0;

  if (
    millis() -
    lastPrint >=
    1000
  )
  {
    lastPrint =
      millis();

    motionIntensity =
      calculateMotionIntensity();

    Serial.print(
      "ACC="
    );

    Serial.print(
      acceleration,
      2
    );

    Serial.print(
      "G | ROT="
    );

    Serial.print(
      gyro,
      0
    );

    Serial.print(
      " | SPEED="
    );

    Serial.print(
      vehicleSpeed,
      1
    );

    Serial.print(
      " km/h | MOTION="
    );

    Serial.print(
      motionIntensity,
      2
    );

    Serial.print(
      " | GPS="
    );

    if (
      gpsFixAvailable
    )
    {
      Serial.print(
        "FIX "
      );

      Serial.print(
        lastLatitude,
        4
      );

      Serial.print(
        ","
      );

      Serial.print(
        lastLongitude,
        4
      );
    }
    else
    {
      Serial.print(
        "SEARCH"
      );
    }

    Serial.print(
      " | SAT="
    );

    Serial.print(
      satelliteCount
    );

    Serial.print(
      " | BLE CLIENTS="
    );

    Serial.print(
      getBLEClientCount()
    );

    Serial.println();
  }


  // ===================================================
  // CRASH DETECTION
  // ===================================================

  if (
    acceleration >=
    IMPACT_THRESHOLD ||

    gyro >=
    GYRO_THRESHOLD
  )
  {
    processCrash(
      acceleration,
      gyro
    );
  }


  delay(20);
}
