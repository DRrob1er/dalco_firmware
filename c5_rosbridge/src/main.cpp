// ✅ Rebuilt version of your original script with updated logic
// ✅ Supports: /gui_button_cur_room, correct LED rendering rules

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include "esp_eap_client.h"

using namespace websockets;

// ---------- USER CONFIG ----------
#define USE_ENTERPRISE false

// Normal WPA2-PSK
const char* WIFI_SSID     = "YES!Delft";
const char* WIFI_PASSWORD = "Z3=Be4-C2";

// WPA2-Enterprise
const char* EAP_SSID      = "EnterpriseSSID";
const char* EAP_IDENTITY  = "testuser";
const char* EAP_USERNAME  = "testuser";
const char* EAP_PASSWORD  = "s3cr3t";



// const char* ROS_HOST      = "172.16.30.9";
const char* ROS_HOST      = "172.16.30.15";
const uint16_t ROS_PORT   = 9090;
const char* ROS_PATH      = "/";

const char* TOPIC_button_eta      = "/gc_eta";
const char* TOPIC_button_que_nr   = "/gui_button_que_nr";
const char* TOPIC_button_cur_room = "/gui_button_cur_room_nr";
const char* TOPIC_req_room        = "/gui_emc_req_room";

#define LED_PIN   1
#define LED_COUNT 12
#define LED_TYPE  NEO_GRB
#define LED_BRIGHTNESS  30
const uint16_t ETA_FULL_SCALE_SEC = 60;

const uint8_t COLOR_ETA_R = 255, COLOR_ETA_G = 120, COLOR_ETA_B = 0;
const uint8_t COLOR_Q_R   = 255, COLOR_Q_G   = 0,   COLOR_Q_B   = 0;
const uint8_t COLOR_OFF_R = 0,   COLOR_OFF_G = 0,   COLOR_OFF_B = 0;

const int RING_ZERO_INDEX = 0;
const int RING_DIR        = +1;

volatile bool buttonPressed = false;
int room_nr = 2;
const int button_pin = 2;

WebsocketsClient ws;
bool wsConnected = false;
bool advertised  = false;
unsigned long lastPingMs = 0;

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, LED_TYPE + NEO_KHZ800);

volatile int16_t gui_eta_sec   = 0;
volatile int16_t gui_queue_cnt = 0;
volatile int16_t gui_cur_room  = -1;

unsigned long lastRenderMs = 0;
const uint16_t RENDER_PERIOD_MS = 50;

volatile unsigned long lastInterruptTime = 0;
const unsigned long debounceDelay = 1000;

unsigned long lastMsgMs = 0;
const unsigned long idleTimeout = 10000;

void sendJson(const JsonDocument& doc) {
  String out;
  serializeJson(doc, out);
  ws.send(out);
}

void IRAM_ATTR handleButtonInterrupt() {
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > debounceDelay) {
    buttonPressed = true;
    lastInterruptTime = interruptTime;
  }
}

void advertise_int16(const char* topic_name) {
  StaticJsonDocument<128> d;
  d["op"] = "advertise";
  d["topic"] = topic_name;
  d["type"] = "std_msgs/Int16";
  sendJson(d);
  advertised = true;
}

void subscribe_topic(const char* topic_name) {
  StaticJsonDocument<256> d;
  d["op"] = "subscribe";
  d["topic"] = topic_name;
  sendJson(d);
  Serial.print("Subscribed "); Serial.println(topic_name);
}

void publish_int_16(int data, const char* topic_name) {
  if (!advertised) return;
  StaticJsonDocument<96> d;
  d["op"] = "publish";
  d["topic"] = topic_name;
  JsonObject msg = d.createNestedObject("msg");
  msg["data"] = data;
  sendJson(d);
}

int physIndex(int logical) {
  int idx = (RING_ZERO_INDEX + (RING_DIR * logical)) % LED_COUNT;
  if (idx < 0) idx += LED_COUNT;
  return idx;
}

uint32_t cColor(uint8_t r, uint8_t g, uint8_t b) {
  return pixels.Color(r, g, b);
}

int etaLitCount(int16_t eta_sec) {
  if (eta_sec <= 0) return 0;
  if (eta_sec >= ETA_FULL_SCALE_SEC) return LED_COUNT;
  long num = (long)eta_sec * LED_COUNT;
  int lit = (int)(num / ETA_FULL_SCALE_SEC);
  return constrain(lit, 0, LED_COUNT);
}

void clearRing() {
  for (int i = 0; i < LED_COUNT; ++i)
    pixels.setPixelColor(i, cColor(COLOR_OFF_R, COLOR_OFF_G, COLOR_OFF_B));
}

void render() {
  if (millis() - lastRenderMs < RENDER_PERIOD_MS) return;
  lastRenderMs = millis();

  bool isCurrentRoom = (gui_cur_room == room_nr);

  int litEta = etaLitCount(gui_eta_sec);
  int litQueue = constrain(gui_queue_cnt, 0, LED_COUNT);

  clearRing();

  if (!isCurrentRoom && gui_queue_cnt == 0) {
    pixels.show();
    return;
  }

  for (int i = 0; i < LED_COUNT; ++i) {
    int p = physIndex(i);

    if (i < litQueue) {
      pixels.setPixelColor(p, cColor(COLOR_Q_R, COLOR_Q_G, COLOR_Q_B));
    } else if (isCurrentRoom) {
      if (gui_eta_sec == 0) {
        pixels.setPixelColor(p, cColor(COLOR_ETA_R, COLOR_ETA_G, COLOR_ETA_B));
      } else if (i < litEta) {
        pixels.setPixelColor(p, cColor(COLOR_ETA_R, COLOR_ETA_G, COLOR_ETA_B));
      }
    }
  }
  pixels.show();
}

void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);

  if (USE_ENTERPRISE) {
      Serial.printf("Connecting to WPA2-Enterprise SSID: %s\n", EAP_SSID);

      WiFi.disconnect(true, true);
      WiFi.mode(WIFI_STA);

      // Configure EAP identity / username / password
      esp_eap_client_set_identity((const uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
      esp_eap_client_set_username((const uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
      esp_eap_client_set_password((const uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));

      // Set EAP method: takes a single esp_eap_method_t, not an array
      esp_eap_client_set_eap_methods(ESP_EAP_TYPE_PEAP);

      // No more esp_eap_client_enable() in C5 Arduino core —
      // EAP config is applied automatically when WiFi.begin() is called.
      WiFi.begin(EAP_SSID);
  } else {
    Serial.printf("Connecting to WPA2-PSK SSID: %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  // Wait for connection
  const uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(300);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi OK. IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("WiFi connect failed.");
  }
}

bool connectRosbridge() {
  if (wsConnected) return true;
  Serial.printf("Connecting to rosbridge: %s:%u%s\n", ROS_HOST, ROS_PORT, ROS_PATH);

  ws.onMessage([&](WebsocketsMessage msg) {
    if (!msg.isText()) return;
    StaticJsonDocument<512> d;
    auto err = deserializeJson(d, msg.data());
    if (err) {
      Serial.print("JSON parse error: ");
      Serial.println(err.c_str());
      return;
    }
    const char* op = d["op"] | "";
    const char* topic = d["topic"] | "";
    if (strcmp(op, "publish") == 0) {
      lastMsgMs = millis();
      if (strcmp(topic, TOPIC_button_eta) == 0) {
        gui_eta_sec = d["msg"]["data"] | 0;
      } else if (strcmp(topic, TOPIC_button_que_nr) == 0) {
        gui_queue_cnt = d["msg"]["data"] | 0;
      } else if (strcmp(topic, TOPIC_button_cur_room) == 0) {
        gui_cur_room = d["msg"]["data"] | -1;
      }
    }
  });

  ws.onEvent([&](WebsocketsEvent ev, String) {
    switch (ev) {
      case WebsocketsEvent::ConnectionOpened:
        wsConnected = true;
        advertised = false;
        Serial.println("WebSocket connected.");
        advertise_int16(TOPIC_req_room);
        subscribe_topic(TOPIC_button_eta);
        subscribe_topic(TOPIC_button_que_nr);
        subscribe_topic(TOPIC_button_cur_room);
        break;
      case WebsocketsEvent::ConnectionClosed:
        wsConnected = false;
        advertised = false;
        Serial.println("WebSocket closed.");
        break;
      default:
        break;
    }
  });

  return ws.connect(ROS_HOST, ROS_PORT, ROS_PATH);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  pixels.begin();
  pixels.setBrightness(LED_BRIGHTNESS);
  clearRing();
  pixels.show();

  connectWifi();
  esp_sleep_enable_wifi_wakeup();
  connectRosbridge();

  pinMode(button_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button_pin), handleButtonInterrupt, FALLING);
}

void loop() {
  if (wsConnected) {
    ws.poll();
  } else {
    static unsigned long lastTry = 0;
    if (millis() - lastTry > 3000) {
      lastTry = millis();
      if (WiFi.status() != WL_CONNECTED) connectWifi();
      connectRosbridge();
    }
  }

  if (wsConnected && millis() - lastPingMs > 15000) {
    lastPingMs = millis();
    ws.ping();
  }

  if (wsConnected && buttonPressed) {
    publish_int_16(room_nr, TOPIC_req_room);
    buttonPressed = false;
  }

  if (wsConnected && (millis() - lastMsgMs > idleTimeout)) {
    esp_sleep_enable_wifi_wakeup();
    esp_light_sleep_start();
    lastMsgMs = millis();
  }

  render();
}
