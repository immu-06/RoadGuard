#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =====================================================
// NODE B - ESP-NOW LISTENER + BLE GATEWAY
// =====================================================

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
// BLE UUIDS - SAME AS NODE A / FLUTTER
// =====================================================

#define ROADGUARD_SERVICE_UUID \
"4fafc201-1fb5-459e-8fcc-c5c9c331914b"

#define ROADGUARD_CHARACTERISTIC_UUID \
"beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer *bleServer = nullptr;
BLEAdvertising *advertising = nullptr;
BLEService *crashService = nullptr;
BLECharacteristic *crashCharacteristic = nullptr;

// =====================================================
// CRASH PACKET
// =====================================================

String lastCrashPacket = "";
String lastCrashId = "";

volatile bool packetReceived = false;
char receivedPacket[256] = {0};
size_t receivedLength = 0;

// =====================================================
// OLED
// =====================================================

void clearOLED()
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
}

void centerText(
  const String &text,
  int y,
  int size
)
{
  display.setTextSize(size);

  int16_t x1, y1;
  uint16_t w, h;

  display.getTextBounds(
    text,
    0,
    y,
    &x1,
    &y1,
    &w,
    &h
  );

  int x = (SCREEN_WIDTH - w) / 2;

  display.setCursor(x, y);
  display.println(text);
}

String extractCrashId(const String &packet)
{
  if (!packet.startsWith("CR|"))
    return "";

  int first = packet.indexOf('|');

  if (first < 0)
    return "";

  int second = packet.indexOf('|', first + 1);

  if (second < 0)
    return "";

  return packet.substring(
    first + 1,
    second
  );
}

String extractSeverity(const String &packet)
{
  // CR|ID|LAT|LNG|SEVERITY|VEHICLE|...
  int positions[5];
  int found = 0;

  for (int i = 0; i < packet.length() && found < 5; i++)
  {
    if (packet[i] == '|')
    {
      positions[found++] = i;
    }
  }

  if (found < 5)
    return "?";

  int start = positions[3] + 1;
  int end = positions[4];

  return packet.substring(start, end);
}

void showReady()
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
    "NODE B",
    31,
    1
  );

  centerText(
    "LISTENING",
    46,
    1
  );

  display.display();
}

void showCrash(
  const String &packet
)
{
  String crashId = extractCrashId(packet);
  String severity = extractSeverity(packet);

  clearOLED();

  centerText(
    "!! CRASH !!",
    0,
    2
  );

  display.setTextSize(1);

  display.setCursor(3, 27);
  display.print("ID: ");
  display.println(crashId);

  display.setCursor(3, 40);
  display.print("SEVERITY: ");
  display.println(severity);

  display.setCursor(3, 53);
  display.print("RELAY -> APP");

  display.display();
}

// =====================================================
// BLE SERVER CALLBACKS
// =====================================================

class NodeBServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *server) override
  {
    Serial.println("BLE APP CONNECTED");

    // If a crash arrived before the phone connected,
    // deliver the stored packet immediately.
    if (lastCrashPacket.length() > 0)
    {
      delay(200);

      crashCharacteristic->setValue(
        lastCrashPacket.c_str()
      );

      crashCharacteristic->notify();

      Serial.println(
        "STORED CRASH SENT TO APP"
      );
    }
  }

  void onDisconnect(BLEServer *server) override
  {
    Serial.println("BLE APP DISCONNECTED");

    delay(100);

    server->startAdvertising();

    Serial.println(
      "NODE B BLE ADVERTISING RESTARTED"
    );
  }
};

// =====================================================
// SEND NODE B CRASH TO PHONE
// =====================================================

void sendCrashToApp(
  const String &packet
)
{
  if (crashCharacteristic == nullptr)
    return;

  // Always store the latest packet.
  crashCharacteristic->setValue(
    packet.c_str()
  );

  if (bleServer->getConnectedCount() > 0)
  {
    crashCharacteristic->notify();

    Serial.println(
      "NODE B -> APP: CRASH NOTIFICATION SENT"
    );
  }
  else
  {
    Serial.println(
      "NODE B: NO APP CONNECTED - PACKET STORED"
    );
  }
}

// =====================================================
// ESP-NOW RECEIVE CALLBACK
// =====================================================

void onEspNowReceive(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
)
{
  if (data == nullptr || len <= 0)
    return;

  // Keep the packet small and safely terminated.
  size_t copyLength = len;

  if (copyLength >= sizeof(receivedPacket))
  {
    copyLength = sizeof(receivedPacket) - 1;
  }

  memcpy(
    receivedPacket,
    data,
    copyLength
  );

  receivedPacket[copyLength] = '\0';

  receivedLength = copyLength;
  packetReceived = true;
}

// =====================================================
// PROCESS RECEIVED PACKET
// =====================================================

void processReceivedPacket()
{
  if (!packetReceived)
    return;

  noInterrupts();
  packetReceived = false;

  String packet = String(receivedPacket);

  receivedPacket[0] = '\0';
  interrupts();

  packet.trim();

  Serial.println();
  Serial.println(
    "================================"
  );
  Serial.println(
    "ESP-NOW PACKET RECEIVED"
  );
  Serial.println(
    "================================"
  );
  Serial.println(packet);

  if (!packet.startsWith("CR|"))
  {
    Serial.println(
      "IGNORING NON-CRASH PACKET"
    );
    return;
  }

  String crashId = extractCrashId(packet);

  if (crashId.length() == 0)
  {
    Serial.println(
      "INVALID CRASH PACKET"
    );
    return;
  }

  // Ignore an exact repeated ESP-NOW packet.
  // This prevents accidental duplicate forwarding.
  if (crashId == lastCrashId)
  {
    Serial.print(
      "DUPLICATE NODE A CRASH IGNORED: "
    );
    Serial.println(crashId);
    return;
  }

  lastCrashId = crashId;
  lastCrashPacket = packet;

  Serial.print("CRASH ID: ");
  Serial.println(crashId);

  Serial.println(
    "DISPLAYING CRASH ON NODE B OLED"
  );

  showCrash(packet);

  // Give the OLED a moment to visibly show the event.
  delay(500);

  Serial.println(
    "TRANSMITTING CRASH TO FLUTTER APP"
  );

  sendCrashToApp(packet);

  Serial.println(
    "================================"
  );
}

// =====================================================
// ESP-NOW SETUP
// =====================================================

void setupESPNow()
{
  WiFi.mode(WIFI_STA);

  Serial.print(
    "NODE B MAC: "
  );
  Serial.println(
    WiFi.macAddress()
  );

  if (esp_now_init() != ESP_OK)
  {
    Serial.println(
      "ESP-NOW INIT FAILED"
    );
    return;
  }

  esp_now_register_recv_cb(
    onEspNowReceive
  );

  Serial.println(
    "ESP-NOW RECEIVER READY"
  );
}

// =====================================================
// BLE SETUP
// =====================================================

void setupBLE()
{
  BLEDevice::init(
    "ROADGUARD_B"
  );

  bleServer =
    BLEDevice::createServer();

  bleServer->setCallbacks(
    new NodeBServerCallbacks()
  );

  crashService =
    bleServer->createService(
      ROADGUARD_SERVICE_UUID
    );

  crashCharacteristic =
    crashService->createCharacteristic(
      ROADGUARD_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY
    );

  crashCharacteristic->addDescriptor(
    new BLE2902()
  );

  crashCharacteristic->setValue(
    "ROADGUARD_B_READY"
  );

  crashService->start();

  advertising =
    BLEDevice::getAdvertising();

  advertising->addServiceUUID(
    ROADGUARD_SERVICE_UUID
  );

  advertising->setScanResponse(true);

  advertising->start();

  Serial.println(
    "NODE B BLE ADVERTISING STARTED"
  );
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println(
    "================================"
  );
  Serial.println(
    "ROADGUARD NODE B"
  );
  Serial.println(
    "ESP-NOW RELAY + BLE GATEWAY"
  );
  Serial.println(
    "================================"
  );

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  if (!display.begin(
    SSD1306_SWITCHCAPVCC,
    OLED_ADDR
  ))
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

  showReady();

  setupESPNow();
  setupBLE();

  Serial.println();
  Serial.println(
    "================================"
  );
  Serial.println(
    "NODE B READY"
  );
  Serial.println(
    "ESP-NOW : LISTENING"
  );
  Serial.println(
    "BLE     : ADVERTISING"
  );
  Serial.println(
    "OLED    : ONLINE"
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
  processReceivedPacket();

  delay(10);
}
