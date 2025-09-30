
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

using namespace websockets;

const char* WIFI_SSID     = "YES!Delft";
const char* WIFI_PASSWORD = "Z3=Be4-C2";
const char* ROS_HOST      = "172.16.30.9";
const uint16_t ROS_PORT   = 9090;
const char* ROS_PATH      = "/";

const char* TOPIC_button_eta      = "/gc_eta";
const char* TOPIC_button_que_nr   = "/gui_button_que_nr";
const char* TOPIC_button_cur_room = "/gui_button_cur_room_nr";
const char* TOPIC_req_room        = "/gui_emc_req_room";

#define LED_PIN     4
#define LED_COUNT   12
#define LED_OFFSET  2
#define LED_TYPE    NEO_GRB
#define BRIGHTNESS  30

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_SDA 8
#define OLED_SCL 9

#define BUTTON_PIN 5           // Room scroll
#define SEND_BUTTON_PIN 2      // Send button
#define DEBOUNCE_MS 50
#define LONG_PRESS_MS 800
#define SCROLL_REPEAT_MS 200

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, LED_TYPE + NEO_KHZ800);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WebsocketsClient ws;

#define MAX_ROOMS 64
String room_labels[MAX_ROOMS];
int room_ids[MAX_ROOMS];
int room_index = 0;
int room_count = 0;

volatile bool shortPressDetected = false;
volatile bool sendPressDetected = false;
unsigned long buttonDownTime = 0;
bool buttonHeld = false;
unsigned long lastScrollTime = 0;
unsigned long lastRoomButtonMs = 0;
unsigned long lastSendButtonMs = 0;

volatile int16_t gui_eta_sec   = 0;
volatile int16_t gui_queue_cnt = 0;
volatile int16_t gui_cur_room  = -1;

bool wsConnected = false;
bool advertised = false;
unsigned long lastMsgMs = 0;
const unsigned long idleTimeout = 10000;

// ---------- Button ISRs ----------
void IRAM_ATTR handleRoomButton() {
  unsigned long now = millis();
  if (now - lastRoomButtonMs < DEBOUNCE_MS) return;
  lastRoomButtonMs = now;

  if (digitalRead(BUTTON_PIN) == LOW) { // button pressed
    buttonDownTime = now;
    buttonHeld = true;
  } else {
    unsigned long pressDuration = now - buttonDownTime;
    if (pressDuration < LONG_PRESS_MS) {
      shortPressDetected = true;
    }
  }
}

void IRAM_ATTR handleSendButton() {
  unsigned long now = millis();
  if (now - lastSendButtonMs < DEBOUNCE_MS) return;
  lastSendButtonMs = now;

  if (digitalRead(SEND_BUTTON_PIN) == LOW) {
    sendPressDetected = true;
  }
}

// ---------- ROS Helper ----------
void sendJson(const JsonDocument& doc) {
  String out;
  serializeJson(doc, out);
  ws.send(out);
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

// ---------- Room Selector ----------
void drawRoomLabel() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Room:");
  display.setCursor(0, 32);
  display.println(room_labels[room_index]);
  display.display();
}

void initRoomMapping() {
  for (int i = 0; i <= 32; ++i) {
    room_labels[i] = "NF-" + String(1100 + i);
    room_ids[i] = i;
  }
  room_count = 33;
  Serial.printf("Hardcoded %d rooms initialized\n", room_count);
}

// ---------- LED Renderer ----------
int physIndex(int logical) {
  return (LED_OFFSET + logical) % LED_COUNT;
}

uint32_t cColor(uint8_t r, uint8_t g, uint8_t b) {
  return pixels.Color(r, g, b);
}

int etaLitCount(int16_t eta_sec) {
  if (eta_sec <= 0) return 0;
  if (eta_sec >= 60) return LED_COUNT;
  return (eta_sec * LED_COUNT) / 60;
}

void clearRing() {
  for (int i = 0; i < LED_COUNT; ++i)
    pixels.setPixelColor(i, cColor(0, 0, 0));
}

void render() {
  int room_id = room_ids[room_index];
  int litEta = (gui_cur_room == room_id && gui_eta_sec > 0) ? etaLitCount(gui_eta_sec) : 0;
  int litQueue = constrain(gui_queue_cnt, 0, LED_COUNT);
  if (gui_cur_room == room_id && gui_eta_sec == 0) {
    litQueue = min(LED_COUNT, litQueue + 1);
  }
  clearRing();
  for (int i = 0; i < LED_COUNT; ++i) {
    int p = physIndex(i);
    if (i < litEta) {
      pixels.setPixelColor(p, cColor(255, 120, 0)); // orange ETA
    } else if (i < litQueue) {
      pixels.setPixelColor(p, cColor(255, 0, 0)); // red QUEUE
    }
  }
  pixels.show();
}

// ---------- Connections ----------
void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? "WiFi connected" : "WiFi failed");
}

bool connectRosbridge() {
  if (wsConnected) return true;
  ws.onMessage([&](WebsocketsMessage msg) {
    if (!msg.isText()) return;
    StaticJsonDocument<512> d;
    auto err = deserializeJson(d, msg.data());
    if (err) return;
    const char* topic = d["topic"] | "";
    if (strcmp(topic, TOPIC_button_eta) == 0) gui_eta_sec = d["msg"]["data"] | 0;
    else if (strcmp(topic, TOPIC_button_que_nr) == 0) gui_queue_cnt = d["msg"]["data"] | 0;
    else if (strcmp(topic, TOPIC_button_cur_room) == 0) gui_cur_room = d["msg"]["data"] | -1;
    lastMsgMs = millis();
  });
  ws.onEvent([&](WebsocketsEvent ev, String) {
    if (ev == WebsocketsEvent::ConnectionOpened) {
      wsConnected = true;
      advertise_int16(TOPIC_req_room);
      subscribe_topic(TOPIC_button_eta);
      subscribe_topic(TOPIC_button_que_nr);
      subscribe_topic(TOPIC_button_cur_room);
    } else if (ev == WebsocketsEvent::ConnectionClosed) {
      wsConnected = false;
    }
  });
  return ws.connect(ROS_HOST, ROS_PORT, ROS_PATH);
}

// ---------- Main ----------
void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(SEND_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleRoomButton, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SEND_BUTTON_PIN), handleSendButton, FALLING);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }

  initRoomMapping();
  drawRoomLabel();

  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);
  clearRing();
  pixels.show();

  connectWifi();
  connectRosbridge();
}

void loop() {
  // Short press
  if (shortPressDetected) {
    shortPressDetected = false;
    room_index = (room_index + 1) % room_count;
    drawRoomLabel();
  }

  // Long hold scroll
  if (buttonHeld && digitalRead(BUTTON_PIN) == LOW &&
      millis() - buttonDownTime > LONG_PRESS_MS &&
      millis() - lastScrollTime > SCROLL_REPEAT_MS) {
    room_index = (room_index + 1) % room_count;
    drawRoomLabel();
    lastScrollTime = millis();
  }

  // Release -> stop holding
  if (digitalRead(BUTTON_PIN) == HIGH) {
    buttonHeld = false;
  }

  // Send button
  if (sendPressDetected) {
    sendPressDetected = false;
    if (wsConnected && advertised) {
      publish_int_16(room_ids[room_index], TOPIC_req_room);
      Serial.printf("Published room_id: %d to %s\n", room_ids[room_index], TOPIC_req_room);
    }
  }

  if (wsConnected) {
    ws.poll();
  } else if (millis() - lastMsgMs > 3000) {
    connectRosbridge();
  }

  if (wsConnected && millis() - lastMsgMs > idleTimeout) {
    esp_sleep_enable_wifi_wakeup();
    esp_light_sleep_start();
    lastMsgMs = millis();
  }

  render();
}