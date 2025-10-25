//----------include----------
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <esp_wifi.h>
#include <HardwareSerial.h>
#include <PZEM004Tv30.h>
#include <WiFiManager.h>

//----------Object Instantiation------------
HardwareSerial PZEMSerial(2);
PZEM004Tv30 pzem(&PZEMSerial);
LiquidCrystal_I2C lcd(0x27, 16, 2);
//----------I/O PIN------------
int relay = 38;
int state_led;
uint8_t overload;
bool initRequested = false;
unsigned long previousMillis = 0;
const long interval = 5000;
#define RESET_BUTTON_PIN 0

// -------- NETPIE2020 --------
const char* mqttServer = "broker.netpie.io";
const int mqttPort = 1883;
const char* mqttClientID = "your Device ID";  // Device ID
const char* mqttUsername = "your Token";      // Token
const char* mqttPassword = "your Secret";      // Secret
WiFiClient espClient;
PubSubClient client(espClient);


void onceStartup(const String& key, const int& value) {
  //Serial.println(key + " = " + value);
  lcd.clear();
  if (key == "relay") {
    if (value == 1) {
      state_led = value;
      digitalWrite(relay, state_led);
    } else {
      state_led = value;
      digitalWrite(relay, state_led);
    }
  }
  if (key == "overload") {
    overload = value;
  }
}


// ฟังก์ชันอ่านค่าจาก Shadow (data{...})
void parseShadowData(const String& msg) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, msg);
  if (err) {
    Serial.print("JSON parse error: ");
    Serial.println(err.c_str());
    return;
  }
  if (!doc.containsKey("data")) {
    Serial.println("No 'data' object in shadow");
    return;
  }
  JsonObject data = doc["data"];
  for (JsonPair kv : data) {
    String key = kv.key().c_str();
    int value = kv.value().as<int>();  // เก็บเป็น int (รองรับเลข)
    Serial.println("[@Set Value]: " + key + " = " + String(value));
    onceStartup(key, value);
  }
}

void requestShadowOnce() {
  if (!initRequested && client.connected()) {
    Serial.println(F("Requesting Shadow..."));
    client.publish("@shadow/data/get", "{}");
    initRequested = true;
    lcd.setCursor(0, 0);
    lcd.print("WIFI: CONNECTED");
    delay(2000);
  }
}

void dosomething(const String& msg, const int state) {
  if (msg == "relay") {
    digitalWrite(relay, state);
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection>>>");
    if (client.connect(mqttClientID, mqttUsername, mqttPassword)) {
      Serial.println("Connected");
      client.subscribe("@msg/relay");
      client.subscribe("@msg/overload");
      client.subscribe("@msg/StateOverload");  // 1 is Overload ||| 0 is not Overload
      client.subscribe("@shadow/data/get/accepted");
      client.subscribe("@shadow/data/get/rejected");
      client.subscribe("@private/#");
      initRequested = false;
      requestShadowOnce();  // ขอค่า Shadow ทันทีหลังต่อสำเร็จ
      Serial.println("*****************");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("Try again in 5 seconds...");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  String msg;
  for (int i = 0; i < length; i++) {
    msg = msg + (char)payload[i];
  }
  Serial.println(msg);
  if (String(topic) == "@private/shadow/data/get/response") {
    parseShadowData(msg);
  }
  if (msg == "@shadow/data/get/accepted" || msg == "@private/shadow/data/get/accepted") {
    Serial.println("Accept");
  }
  if (msg == "@shadow/data/get/rejected" || msg == "@private/shadow/data/get/rejected") {
    Serial.println(F("Shadow GET rejected (check device/shadow keys)."));
  }
  // ---------------------Setoverload -----------------
  if (String(topic) == "@msg/overload") {
    overload = msg.toInt();
    client.publish("@shadow/data/update", ("{\"data\" : {\"overload\" : " + String(msg) + "}}").c_str());
  }
  // ---------------------Relay  -----------------
  if (String(topic) == "@msg/relay") {
    if (msg == "1") {
      client.publish("@shadow/data/update", "{\"data\" : {\"relay\" : 1}}");
      client.publish("@shadow/data/update", "{\"data\" : {\"StateOverload\" : 0}}");
      dosomething("relay", 1);
    } else if (msg == "0") {
      client.publish("@shadow/data/update", "{\"data\" : {\"relay\" : 0}}");
      dosomething("relay", 0);
    }
  }
}


void setup() {
  Serial.begin(115200);
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  PZEMSerial.begin(9600, SERIAL_8N1, 17, 18);
  pinMode(relay, OUTPUT);
  Serial.println("\nHELLO :)");
  Serial.println("*****************");
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  // --- WiFiManager Setup ---
  Serial.println("Press BOOT button now to reset WiFi settings.");
  delay(3000);  // Delay 3 seconds to give user time to press the button

  WiFiManager wm;

  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    Serial.println("Reset button was pressed! Resetting WiFi settings...");
    wm.resetSettings();
    Serial.println("WiFi settings erased. Please restart the device.");
    while (true)
      ;  // Halt execution
  }

  wm.setConfigPortalTimeout(180);  // Set config portal timeout to 3 minutes

  lcd.clear();
  lcd.print("Connecting Wifi..");

  if (!wm.autoConnect("ESP32-PZEM-Setup")) {
    Serial.println("Failed to connect and timeout occurred. Restarting...");
    lcd.clear();
    lcd.print("Connect Failed");
    delay(3000);
    ESP.restart();
  }

  // --- WiFi Connected ---
  Serial.println("");
  Serial.println("========================================");
  Serial.println("WiFi connected!");
  Serial.print("Connected SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("========================================");

  // Set sleep mode to none
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  // --- MQTT Setup ---
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  client.setBufferSize(512);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    float v = pzem.voltage();
    float i = pzem.current();
    float p = pzem.power();
    float e = pzem.energy();
    if (isnan(v) || isnan(i) || isnan(p) || isnan(e)) {
      Serial.println("Read fail (check wiring/baud)");
    } else {
        String payload = String("{\"data\":{") + "\"volt\":" + String(v, 2) + "," + "\"amp\":" + String(i, 3) + "," + "\"power\":" + String(p, 2) + "," + "\"e\":" + String(e, 3) + "}}";
        client.publish("@shadow/data/update", payload.c_str());
        lcd.setCursor(0, 0);
        lcd.print("Volt:" + String(v) + "  ");
        lcd.setCursor(0, 1);
        lcd.print("Amp:" + String(i) + "   ");
        if (i > overload && overload >= 1) {
          client.publish("@shadow/data/update", "{\"data\" : {\"relay\" : 0}}");
          client.publish("@shadow/data/update", "{\"data\" : {\"StateOverload\" : 1}}");
          digitalWrite(relay, 0);
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("OVERLOAD");
      }
    }
  }
}
