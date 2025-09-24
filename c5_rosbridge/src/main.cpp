// const char* WIFI_SSID     = "YES!Delft";
// const char* WIFI_PASSWORD = "Z3=Be4-C2";
// static const char* TOPIC_ODOM   = "/odom_esp";     // nav_msgs/Odometry from robot
// static const char* TOPIC_CMDVEL = "/cmd_vel_esp";  // geometry_msgs/Twist we publish

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>            // v6.x
#include <ArduinoWebsockets.h>
#include <Adafruit_NeoPixel.h>      // install "Adafruit NeoPixel" library
#include <math.h>

using namespace websockets;

// ---------- USER CONFIG ----------
const char* WIFI_SSID     = "YES!Delft";
const char* WIFI_PASSWORD = "Z3=Be4-C2";

// rosbridge server (non-default port)
const char*   ROS_HOST  = "172.16.30.9";

// const char* WIFI_SSID     = "Dalco_5G_DFS";
// const char* WIFI_PASSWORD = "18873221";

// // rosbridge server (non-default port)
// const char*   ROS_HOST  = "192.168.0.171";
const uint16_t ROS_PORT = 9090;
const char*   ROS_PATH  = "/";

// Topics (std_msgs/Int16 for both inputs; Int16 for request)
// const char* TOPIC_button_eta    = "/gui_button_eta";     // std_msgs/Int16  (seconds to arrival)
const char* TOPIC_button_eta    = "/gc_eta";     // std_msgs/Int16  (seconds to arrival)
const char* TOPIC_button_que_nr = "/gui_button_que_nr";  // std_msgs/Int16  (#rooms in queue)
const char* TOPIC_req_room      = "/gui_emc_req_room";   // std_msgs/Int16  (outgoing on button)

// -------- LED RING CONFIG --------
// SK6812 rings exist as RGB and RGBW; pick the correct color order.
// For SK6812 RGBW: NEO_GRBW. For RGB-only: NEO_GRB.
#define LED_PIN   1             // <- set to your data pin
#define LED_COUNT 12            // <- change to 12 when you swap rings
#define LED_TYPE  NEO_GRB      // <- NEO_GRBW (SK6812 RGBW) or NEO_GRB (RGB)
#define LED_BRIGHTNESS  30      // 0..255
const bool ENABLE_GAMMA = true;
// How long a "full ring" represents (seconds). You said 24 LEDs = 120 s (5 s/LED).
// If ETA >= this value, ring is full; if ETA <= 0, ring is empty.
const uint16_t ETA_FULL_SCALE_SEC = 60;

// Colors (8-bit per channel). Works for both RGB and RGBW (W=0 if RGB).
// You can tune these in-place later.
const uint8_t COLOR_ETA_R = 255, COLOR_ETA_G = 120, COLOR_ETA_B = 0,   COLOR_ETA_W = 0;   // orange
const uint8_t COLOR_Q_R   = 255, COLOR_Q_G   = 0,   COLOR_Q_B   = 0,   COLOR_Q_W   = 0;   // red
const uint8_t COLOR_OFF_R = 0,   COLOR_OFF_G = 0,   COLOR_OFF_B = 0,   COLOR_OFF_W = 0;   // off
// ---------------------------------

// Optional: orientation (which LED is "first") and direction (CW/CCW).
// If your ring appears reversed or rotated, adjust these two.
const int RING_ZERO_INDEX = 0;     // 0..LED_COUNT-1
const int RING_DIR        = +1;    // +1 for CW, -1 for CCW

// ---------- BUTTON (unchanged) ----------
volatile bool buttonPressed = false; // set by ISR
int room_nr = 2;
const int button_pin = 2;

// ---------- WS/ROS ----------
WebsocketsClient ws;
bool wsConnected = false;
bool advertised  = false;

unsigned long lastPingMs    = 0;

// ---------- LED DRIVER ----------
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, LED_TYPE + NEO_KHZ800);

// Cached inputs
volatile int16_t gui_eta_sec   = 0;   // seconds to arrival
volatile int16_t gui_queue_cnt = 0;   // # rooms in queue

// For render throttling
unsigned long lastRenderMs = 0;
const uint16_t RENDER_PERIOD_MS = 50;   // 20 FPS max

volatile unsigned long lastInterruptTime = 0;  
const unsigned long debounceDelay = 1000; // milliseconds

unsigned long lastMsgMs = 0;   // timestamp of last incoming topic
const unsigned long idleTimeout = 10000; // 10s


// ---------- STATE MACHINE ----------
enum class LedState : uint8_t {
  UNKNOWN = 0,
  NO_QUEUE,
  QUEUE_PRESENT
};

LedState currentState = LedState::UNKNOWN;

// ---------- HELPERS ----------
static inline void sendJson(const JsonDocument& doc) {
  String out;
  serializeJson(doc, out);
  ws.send(out);
}

void IRAM_ATTR handleButtonInterrupt() {
  unsigned long interruptTime = millis();
  // If interrupts come faster than debounceDelay, ignore them
  if (interruptTime - lastInterruptTime > debounceDelay) {
    buttonPressed = true;
    lastInterruptTime = interruptTime;
  }
}

void advertise_int16(const char* topic_name) {
  StaticJsonDocument<128> d;
  d["op"]    = "advertise";
  d["topic"] = topic_name;
  d["type"]  = "std_msgs/Int16";
  sendJson(d);
  advertised = true;
}

void subscribe_topic(const char* topic_name) {
  StaticJsonDocument<256> d;
  d["op"]    = "subscribe";
  d["topic"] = topic_name;
  sendJson(d);
  Serial.print("Subscribed "); Serial.println(topic_name);
}

void publish_int_16(int data, const char* topic_name) {
  if (!advertised) return;  // ensure we advertised this topic first
  StaticJsonDocument<96> d;
  d["op"]    = "publish";
  d["topic"] = topic_name;
  JsonObject msg = d.createNestedObject("msg");
  msg["data"] = data;
  sendJson(d);
}

// Map logical index -> physical pixel (handles rotation & direction)
int physIndex(int logical) {
  int idx = (RING_ZERO_INDEX + (RING_DIR * logical)) % LED_COUNT;
  if (idx < 0) idx += LED_COUNT;
  return idx;
}

uint32_t cColor(uint8_t r, uint8_t g, uint8_t b, uint8_t w=0) {
#if (LED_TYPE == NEO_GRBW)
  return pixels.Color(r, g, b, w);
#else
  (void)w;
  return pixels.Color(r, g, b);
#endif
}

// Compute how many LEDs should be lit for ETA (full ring at ETA_FULL_SCALE_SEC, empty at 0)
// At 120 sec -> LED_COUNT; at 1 sec -> 0 LEDs (per your spec).
int etaLitCount(int16_t eta_sec) {
  if (eta_sec <= 0) return 0;
  if (eta_sec >= ETA_FULL_SCALE_SEC) return LED_COUNT;
  // Linear map, truncate toward zero to show “walking back”
  // Add + (ETA_FULL_SCALE_SEC-1) / ETA_FULL_SCALE_SEC if you prefer rounding.
  long num = (long)eta_sec * LED_COUNT;
  int lit = (int)(num / ETA_FULL_SCALE_SEC);
  return constrain(lit, 0, LED_COUNT);
}

void clearRing() {
  for (int i = 0; i < LED_COUNT; ++i) pixels.setPixelColor(i, cColor(COLOR_OFF_R, COLOR_OFF_G, COLOR_OFF_B, COLOR_OFF_W));
}

void renderNoQueue(int16_t eta_sec) {
  int lit = etaLitCount(eta_sec);
  for (int i = 0; i < LED_COUNT; ++i) {
    int p = physIndex(i);
    if (i < lit)
      pixels.setPixelColor(p, cColor(COLOR_ETA_R, COLOR_ETA_G, COLOR_ETA_B, COLOR_ETA_W));
    else
      pixels.setPixelColor(p, cColor(COLOR_OFF_R, COLOR_OFF_G, COLOR_OFF_B, COLOR_OFF_W));
  }
}

void renderQueuePresent(int16_t eta_sec, int16_t queue_cnt) {
  int litEta = etaLitCount(eta_sec);
  int q = constrain(queue_cnt, 0, LED_COUNT);

  for (int i = 0; i < LED_COUNT; ++i) {
    int p = physIndex(i);
    if (i < q) {
      // Front LEDs show queue in red, regardless of ETA fill
      pixels.setPixelColor(p, cColor(COLOR_Q_R, COLOR_Q_G, COLOR_Q_B, COLOR_Q_W));
    } else if (i < litEta) {
      pixels.setPixelColor(p, cColor(COLOR_ETA_R, COLOR_ETA_G, COLOR_ETA_B, COLOR_ETA_W));
    } else {
      pixels.setPixelColor(p, cColor(COLOR_OFF_R, COLOR_OFF_G, COLOR_OFF_B, COLOR_OFF_W));
    }
  }
}

void updateState() {
  LedState next =
      (gui_queue_cnt > 0) ? LedState::QUEUE_PRESENT : LedState::NO_QUEUE;
  if (next != currentState) {
    currentState = next;
  }
}

void render() {
  if (millis() - lastRenderMs < RENDER_PERIOD_MS) return;
  lastRenderMs = millis();

  updateState();

  switch (currentState) {
    case LedState::NO_QUEUE:
      renderNoQueue(gui_eta_sec);
      break;
    case LedState::QUEUE_PRESENT:
      renderQueuePresent(gui_eta_sec, gui_queue_cnt);
      break;
    default:
      clearRing();
      break;
  }
  pixels.show();
}

// ---------- ROSBRIDGE ----------
void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);  // po.er consumption
  // WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  

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

    StaticJsonDocument<512> d;   // sufficient for std_msgs/Int16
    auto err = deserializeJson(d, msg.data());
    if (err) {
      Serial.print("JSON parse error: ");
      Serial.println(err.c_str());
      return;
    }
    const char* op    = d["op"]    | "";
    const char* topic = d["topic"] | "";

    if (strcmp(op, "publish") == 0) {
      lastMsgMs = millis();   // reset timeout whenever a message comes
      // std_msgs/Int16 payload: {"op":"publish","topic": "...","msg":{"data": <int>}}
      if (strcmp(topic, TOPIC_button_eta) == 0) {
        JsonVariantConst v = d["msg"]["data"];
        if (!v.isNull()) {
          gui_eta_sec = v.as<int16_t>();
          // Clamp to safe range
          if (gui_eta_sec < 0) gui_eta_sec = 0;
        }
      } else if (strcmp(topic, TOPIC_button_que_nr) == 0) {
        JsonVariantConst v = d["msg"]["data"];
        if (!v.isNull()) {
          gui_queue_cnt = v.as<int16_t>();
          if (gui_queue_cnt < 0) gui_queue_cnt = 0;
        }
      }
    }
  });

  ws.onEvent([&](WebsocketsEvent ev, String){
    switch (ev) {
      case WebsocketsEvent::ConnectionOpened:
        wsConnected = true;
        advertised  = false;
        Serial.println("WebSocket connected.");
        advertise_int16(TOPIC_req_room);
        subscribe_topic(TOPIC_button_eta);
        subscribe_topic(TOPIC_button_que_nr);
        break;
      case WebsocketsEvent::ConnectionClosed:
        wsConnected = false;
        advertised  = false;
        Serial.println("WebSocket closed.");
        break;
      case WebsocketsEvent::GotPing:
        Serial.println("Ping <--");
        break;
      case WebsocketsEvent::GotPong:
        Serial.println("Pong -->");
        break;
    }
  });

  const bool ok = ws.connect(ROS_HOST, ROS_PORT, ROS_PATH);
  if (!ok) Serial.println("WebSocket connect failed.");
  return ok;
}

// ---------- ARDUINO ----------
void setup() {
  Serial.begin(115200);
  delay(200);

  // LED init
  pixels.begin();
  pixels.setBrightness(LED_BRIGHTNESS);
  clearRing();
  pixels.show();

  connectWifi();
  
  // Wake up when button pressed (GPIO)
  // esp_sleep_enable_ext0_wakeup((gpio_num_t)button_pin, 0); // 0 = falling edge

  // Wake up on WiFi traffic (so incoming packets resume CPU)
  esp_sleep_enable_wifi_wakeup();

  connectRosbridge();

  pinMode(button_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button_pin), handleButtonInterrupt, FALLING);
}



void loop() {
  // WS service
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

  // Keepalive ping
  if (wsConnected && millis() - lastPingMs > 15000) {
    lastPingMs = millis();
    ws.ping();
  }

  // User button publishes an Int16 on request topic (unchanged)
  if (wsConnected && buttonPressed) {
    publish_int_16(room_nr, TOPIC_req_room);
    buttonPressed = false;
  }

// Check for inactivity
if (wsConnected && (millis() - lastMsgMs > idleTimeout)) {
  Serial.println("No messages for 10s → entering light sleep");

  // Enable wakeup sources
  // esp_sleep_enable_timer_wakeup(5 * 1000000ULL);  // auto wake after 5s
  // esp_sleep_enable_ext0_wakeup((gpio_num_t)button_pin, 0); // wake on button
  esp_sleep_enable_wifi_wakeup(); // wake on WiFi traffic

  // Enter light sleep
  esp_light_sleep_start();

  // Execution resumes here after wakeup
  lastMsgMs = millis();  // reset timer
}

  // LED renderer (throttled)
  render();
}








// #include <Arduino.h>
// #include <WiFi.h>
// #include <ArduinoJson.h>            // v6.x (pin to 6.21.x in platformio.ini)
// #include <ArduinoWebsockets.h>

// using namespace websockets;

// // ---------- USER CONFIG ----------
// const char* WIFI_SSID     = "Henk";
// const char* WIFI_PASSWORD = "Henk";

// // rosbridge server (non-default port)
// const char*   ROS_HOST = "172.16.30.9";
// const uint16_t ROS_PORT = 9090;     // if you move rosbridge to 80/443, adjust this
// const char*   ROS_PATH = "/";       // change if your server expects e.g. "/rosbridge"

// // Topics
// const char* TOPIC_button_eta    = "/gui_button_eta";    // nav_msgs/Odometry
// const char* TOPIC_button_que_nr = "/gui_button_que_nr";    // nav_msgs/Odometry
// const char* TOPIC_req_room      = "/gui_emc_req_room"; // geometry_msgs/Twist
// // ---------------------------------

// volatile bool buttonPressed = false; // set by ISR
// int room_nr = 2;
// const int button_pin = 1;

// WebsocketsClient ws;
// bool wsConnected = false;
// bool advertised  = false;

// unsigned long lastPublishMs = 0;
// unsigned long lastPingMs    = 0;

// // Send any JsonDocument over WS
// static inline void sendJson(const JsonDocument& doc) {
//   String out;
//   serializeJson(doc, out);
//   ws.send(out);
// }

// // Setup
// void IRAM_ATTR handleButtonInterrupt() {
//   buttonPressed = true;
// }
// void subscribe_eta() {
//   StaticJsonDocument<256> d;
//   d["op"]    = "subscribe";
//   d["topic"] = TOPIC_button_eta;
//   sendJson(d);
//   Serial.println("Subscribed /gui_button_eta");
// }

// void subscribe_que_nr() {
//   StaticJsonDocument<256> d;
//   d["op"]    = "subscribe";
//   d["topic"] = TOPIC_button_que_nr;
//   sendJson(d);
//   Serial.println("Subscribed /gui_button_que_nr");
// }

// void advertise_int16(const char* topic_name) {
//   StaticJsonDocument<128> d;
//   d["op"]    = "advertise";
//   d["topic"] = topic_name;
//   d["type"]  = "std_msgs/Int16";
//   sendJson(d);
//   advertised = true;
// }

// // Loop
// void publish_int_16(int data, const char* topic_name) {
//   if (!advertised) return;  // make sure you've advertised this topic as std_msgs/Int16
//   StaticJsonDocument<96> d;
//   d["op"]    = "publish";
//   d["topic"] = topic_name;
//   JsonObject msg = d.createNestedObject("msg");
//   msg["data"] = data;
//   sendJson(d);
// }

// void handleOdomMessage(const JsonDocument& doc) {
//   // Expect: {"op":"publish","topic":"/odom","msg":{...}}
//   if (!doc.containsKey("msg")) return;

//   JsonVariantConst posx = doc["msg"]["pose"]["pose"]["position"]["x"];
//   JsonVariantConst posy = doc["msg"]["pose"]["pose"]["position"]["y"];
//   if (!posx.isNull() && !posy.isNull()) {
//     Serial.print("ODOM position -> x: ");
//     Serial.print(posx.as<float>(), 3);
//     Serial.print("  y: ");
//     Serial.println(posy.as<float>(), 3);
//   } else {
//     String tmp; serializeJson(doc["msg"], tmp);
//     Serial.printf("ODOM message size: %d bytes\n", tmp.length());
//   }
// }

// void connectWifi() {
//   if (WiFi.status() == WL_CONNECTED) return;

//   Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
//   WiFi.mode(WIFI_STA);
//   WiFi.setSleep(false);          // improves stability with some APs
//   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

//   const uint32_t t0 = millis();
//   while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
//     delay(300);
//     Serial.print('.');
//   }
//   Serial.println();

//   if (WiFi.status() == WL_CONNECTED) {
//     Serial.printf("WiFi OK. IP: %s\n", WiFi.localIP().toString().c_str());
//   } else {
//     Serial.println("WiFi connect failed.");
//   }
// }

// bool connectRosbridge() {
//   if (wsConnected) return true;

//   Serial.printf("Connecting to rosbridge: %s:%u%s\n", ROS_HOST, ROS_PORT, ROS_PATH);

//   // Assign handlers (replaces any previous ones automatically)
//   ws.onMessage([&](WebsocketsMessage msg) {
//     if (!msg.isText()) return;
//     StaticJsonDocument<2048> d;             // adjust if your messages are larger
//     auto err = deserializeJson(d, msg.data());
//     if (err) {
//       Serial.print("JSON parse error: ");
//       Serial.println(err.c_str());
//       return;
//     }
//     const char* op    = d["op"]    | "";
//     const char* topic = d["topic"] | "";
//     if (strcmp(op, "publish") == 0 && strcmp(topic, TOPIC_button_eta) == 0) {
//       handleOdomMessage(d);
//     }
//   });

//   ws.onEvent([&](WebsocketsEvent ev, String){
//     switch (ev) {
//       case WebsocketsEvent::ConnectionOpened:
//         wsConnected = true;
//         advertised  = false;
//         Serial.println("WebSocket connected.");
//         // (Re)advertise + subscribe on every connect
//         advertise_int16(TOPIC_req_room);
//         subscribe_eta();
//         break;
//       case WebsocketsEvent::ConnectionClosed:
//         wsConnected = false;
//         advertised  = false;
//         Serial.println("WebSocket closed.");
//         break;
//       case WebsocketsEvent::GotPing:
//         Serial.println("Ping <--");
//         break;
//       case WebsocketsEvent::GotPong:
//         Serial.println("Pong -->");
//         break;
//     }
//   });

//   // IMPORTANT:
//   // - Do NOT add "Host" or "Origin" headers here (avoids duplicates).
//   // - Use host/port/path overload; if your lib omits :port in Host, apply the 1-line patch.
//   const bool ok = ws.connect(ROS_HOST, ROS_PORT, ROS_PATH);
//   if (!ok) Serial.println("WebSocket connect failed.");
//   return ok;
// }

// // Execution
// void setup() {
//   Serial.begin(115200);
//   delay(200);
//   connectWifi();
//   connectRosbridge();
//   pinMode(button_pin,INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(button_pin), handleButtonInterrupt, FALLING);
// }

// void loop() {
//   if (wsConnected) {
//     ws.poll();  // process incoming frames
//   } else {
//     static unsigned long lastTry = 0;
//     if (millis() - lastTry > 3000) {
//       lastTry = millis();
//       if (WiFi.status() != WL_CONNECTED) connectWifi();
//       connectRosbridge();
//     }
//   }

//   // Keepalive ping to detect stale connections
//   if (wsConnected && millis() - lastPingMs > 15000) {
//     lastPingMs = millis();
//     ws.ping();
//   }

//   // Periodic demo publish
//   // if (wsConnected && advertised && millis() - lastPublishMs > 1000) {
//   //   lastPublishMs = millis();
//   //   publishCmdVel(0.05f, 0.0f);  // 5 cm/s forward, no rotation
//   //   Serial.println("Published /cmd_vel");
//   // }

//   if (wsConnected && buttonPressed) {
//     publish_int_16(room_nr, TOPIC_req_room);
//     buttonPressed = false; // reset so it won’t send again until next falling edge
//   }
  
// }
