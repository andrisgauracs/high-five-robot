// Arm control over Wi-Fi (and USB serial), with smooth moves (explicit LEDC, robot_config.h). firmware/arm_serial plus
// a TCP server, so the arm runs from the DC supply alone: on power-up it goes home, joins Wi-Fi as
// highfive-arm.local and waits for tools/highfive.py on the Mac.
// Commands, one per line (serial 115200, or TCP port 8766: one command per connection, like tools/arm_bridge.py):
//   M <pan> <shoulder> <elbow> <wrist> <twist>   move to these pulses (us); all joints arrive together
//   J <joint 0-4> <us>                            move one joint (a limp joint engages straight at that pulse)
//   L [joint]                                     go limp (no pulse): one joint, or all
//   H                                             go to the home pose (HOME_US); C = trimmed centre (JOINT_TRIM_US)
//   V <deg per s>                                 speed limit of the fastest joint (default 30, max 360: then the servos' own top speed rules)
//   ?                                             print the state
//   B                                             boot log: why the board restarted and how far it got, last 16 boots
// Replies "OK ..." when a move starts and "DONE ..." when it ends; L, ?, V reply "AT ...". A TCP client gets one line
// back: DONE once the move has finished (or ERR / AT straight away).
// The board's RGB LED (GPIO48) shows how far start-up got: red = starting, yellow = servos on, blue = joining Wi-Fi,
// green = online. Each boot's reset reason and furthest stage are kept in flash (command B), so a board that fails on
// the DC supply can be read out over USB afterwards.
// Wi-Fi details come from wifi_secrets.h, generated from the project .env by make_secrets.py (never committed).
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include "esp_system.h"
#include "robot_config.h"
#include "wifi_secrets.h"

const char *HOSTNAME = "highfive-arm";
const int TCP_PORT = 8766;

// pulse limits per joint: the pan has nothing to hit, so it gets the servo's full 500..2500 range
const int US_MIN[JOINT_COUNT] = {500, 700, 700, 700, 700}, US_MAX[JOINT_COUNT] = {2500, 2300, 2300, 2300, 2300};
float pos[JOINT_COUNT], start[JOINT_COUNT], goal[JOINT_COUNT];
bool on[JOINT_COUNT];
float degPerS = 30;
uint32_t t0 = 0, dur = 0;
bool moving = false;
String line, lastLine;

WiFiServer server(TCP_PORT);
WiFiClient client;             // the TCP client waiting for its reply
bool clientWaits = false;      // ... for a move to finish
bool mdnsUp = false;
uint32_t wifiStart = 0;        // when the current Wi-Fi attempt began (0 = connected)

const int STATUS_LED = 48;
enum Stage { STARTING = 1, SERVOS_ON, JOINING, ONLINE };
const int BOOTS = 16;
uint8_t boots[BOOTS][2];       // [reset reason, furthest stage], newest last
Preferences prefs;

void stage(Stage st) {
  static const uint8_t RGB[][3] = {{0, 0, 0}, {40, 0, 0}, {30, 20, 0}, {0, 0, 40}, {0, 40, 0}};
  rgbLedWrite(STATUS_LED, RGB[st][0], RGB[st][1], RGB[st][2]);
  if (st <= boots[BOOTS - 1][1]) return;
  boots[BOOTS - 1][1] = st;
  prefs.putBytes("log", boots, sizeof(boots));
}

void logBoot() {
  prefs.begin("boot");
  prefs.getBytes("log", boots, sizeof(boots));
  memmove(boots[0], boots[1], (BOOTS - 1) * 2);
  boots[BOOTS - 1][0] = esp_reset_reason();
  boots[BOOTS - 1][1] = 0;
  stage(STARTING);
}

void printBoots() {
  static const char *WHY[] = {"?", "power-on", "reset-pin", "software", "crash", "int-wdt", "task-wdt", "wdt",
                              "deepsleep", "brownout", "sdio", "usb", "jtag", "efuse", "pwr-glitch", "cpu-lockup"};
  static const char *HOW_FAR[] = {"-", "starting", "servos-on", "joining", "online"};
  String s = "BOOTS";
  for (int i = 0; i < BOOTS; i++)
    if (boots[i][0]) s += String(" ") + WHY[boots[i][0] < 16 ? boots[i][0] : 0] + ">" + HOW_FAR[boots[i][1] < 5 ? boots[i][1] : 0];
  emit(s);
}

void writeUs(int j, float us) {
  ledcWrite(JOINT_PIN[j], on[j] ? (uint32_t)(us / 20000.0f * 16384) : 0);
}

void emit(const String &s) {
  Serial.println(s);
  lastLine = s;
}

void print(const char *tag) {
  String s = tag;
  for (int j = 0; j < JOINT_COUNT; j++) s += " " + String((int)lroundf(pos[j]));
  s += " | on";
  for (int j = 0; j < JOINT_COUNT; j++) s += " " + String((int)on[j]);
  emit(s);
}

void moveTo(const float *target) {
  float most = 0;
  for (int j = 0; j < JOINT_COUNT; j++) {
    goal[j] = constrain(target[j], US_MIN[j], US_MAX[j]);
    if (!on[j]) { pos[j] = goal[j]; on[j] = true; }           // limp: engage directly (nothing to ramp from)
    start[j] = pos[j];
    most = max(most, fabsf(goal[j] - start[j]));
  }
  dur = (uint32_t)(most / US_PER_DEG / degPerS * 1000.0f);
  t0 = millis();
  moving = true;
  print("OK");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  logBoot();
  for (int j = 0; j < JOINT_COUNT; j++) {
    ledcAttachChannel(JOINT_PIN[j], 50, 14, JOINT_CHANNEL[j]);
    on[j] = HOME_US[j] > 0;
    pos[j] = on[j] ? HOME_US[j] : 1500 + JOINT_TRIM_US[j];
    writeUs(j, pos[j]);
    if (on[j]) delay(400);
  }
  print("READY");
  stage(SERVOS_ON);
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                // no power save: replies come straight away
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // joins in the background; the arm works over USB meanwhile
  wifiStart = millis();
  stage(JOINING);
  server.begin();
}

void handle(String s) {
  s.trim();
  if (!s.length()) return;
  char c = toupper(s[0]);
  float t[JOINT_COUNT];
  for (int j = 0; j < JOINT_COUNT; j++) t[j] = pos[j];
  if (c == 'M') {
    if (sscanf(s.c_str() + 1, "%f %f %f %f %f", &t[0], &t[1], &t[2], &t[3], &t[4]) == 5) moveTo(t);
    else emit("ERR need 5 pulses");
  } else if (c == 'J') {
    int j; float us;
    if (sscanf(s.c_str() + 1, "%d %f", &j, &us) == 2 && j >= 0 && j < JOINT_COUNT) {
      for (int k = 0; k < JOINT_COUNT; k++) if (k != j && !on[k]) t[k] = pos[k];
      t[j] = us;
      bool keep[JOINT_COUNT];
      for (int k = 0; k < JOINT_COUNT; k++) keep[k] = on[k];
      moveTo(t);
      for (int k = 0; k < JOINT_COUNT; k++) if (k != j && !keep[k]) { on[k] = false; writeUs(k, 0); }
    } else emit("ERR J <joint> <us>");
  } else if (c == 'L') {
    int j = -1;
    sscanf(s.c_str() + 1, "%d", &j);
    moving = false;
    for (int k = 0; k < JOINT_COUNT; k++) if (j < 0 || k == j) { on[k] = false; writeUs(k, 0); }
    print("AT");
  } else if (c == 'H' || c == 'C') {
    for (int j = 0; j < JOINT_COUNT; j++) t[j] = (c == 'H' && HOME_US[j] > 0) ? HOME_US[j] : 1500 + JOINT_TRIM_US[j];
    moveTo(t);
  } else if (c == 'V') {
    degPerS = constrain(s.substring(1).toFloat(), 5, 360);
    print("AT");
  } else if (c == '?') print("AT");
  else if (c == 'B') printBoots();
  else emit("ERR unknown command");
}

void reply(const String &s) {
  if (client && client.connected()) { client.print(s + "\n"); client.flush(); }
  client.stop();
  clientWaits = false;
}

void serveTcp() {
  if (!mdnsUp && WiFi.status() == WL_CONNECTED) {
    mdnsUp = MDNS.begin(HOSTNAME);
    if (mdnsUp) MDNS.addService("highfive-arm", "tcp", TCP_PORT);
    Serial.printf("WIFI %s as %s.local\n", WiFi.localIP().toString().c_str(), HOSTNAME);
    stage(ONLINE);
    wifiStart = 0;
  }
  if (WiFi.status() != WL_CONNECTED) {
    mdnsUp = false;                                      // announce again after a reconnect
    if (!wifiStart) { wifiStart = millis(); stage(JOINING); }
    else if (millis() - wifiStart > 20000) {             // stuck (the router refused us, say): start over
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      wifiStart = millis();
    }
  }
  if (clientWaits) return;                               // one command at a time
  WiFiClient c = server.accept();
  if (!c) return;
  client = c;
  client.setTimeout(2);
  String cmd = client.readStringUntil('\n');
  handle(cmd);
  if (lastLine.startsWith("OK")) clientWaits = true;     // a move started: answer when it is DONE
  else reply(lastLine);
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handle(line); line = ""; }
    else line += ch;
  }
  serveTcp();
  if (moving) {
    float k = dur ? min(1.0f, (millis() - t0) / (float)dur) : 1.0f;
    float e = k * k * (3 - 2 * k);                       // ease in and out
    for (int j = 0; j < JOINT_COUNT; j++) {
      pos[j] = start[j] + (goal[j] - start[j]) * e;
      writeUs(j, pos[j]);
    }
    if (k >= 1) {
      moving = false;
      print("DONE");
      if (clientWaits) reply(lastLine);
    }
  }
  delay(10);
}
