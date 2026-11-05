// Arm control over USB serial, with slow, smooth moves (explicit LEDC, robot_config.h).
// Serial 115200, one command per line:
//   M <pan> <shoulder> <elbow> <wrist> <twist>   move to these pulses (us); all joints arrive together
//   J <joint 0-4> <us>                            move one joint (a limp joint engages straight at that pulse)
//   L [joint]                                     go limp (no pulse): one joint, or all
//   H                                             go to the home pose (HOME_US); C = trimmed centre (JOINT_TRIM_US)
//   V <deg per s>                                 speed limit of the fastest joint (default 30, max 120)
//   ?                                             print the state
// Replies "OK ..." when a move starts and "DONE ..." when it ends; L, ?, V reply "AT ...".
// On boot each joint with a HOME_US goes there, one at a time; joints without one stay limp. A servo can't report
// its position, so the first pulse to a limp joint always makes it jump to that pulse.
#include "robot_config.h"

// pulse limits per joint: the pan has nothing to hit, so it gets the servo's full 500..2500 range
const int US_MIN[JOINT_COUNT] = {500, 700, 700, 700, 700}, US_MAX[JOINT_COUNT] = {2500, 2300, 2300, 2300, 2300};
float pos[JOINT_COUNT], start[JOINT_COUNT], goal[JOINT_COUNT];
bool on[JOINT_COUNT];
float degPerS = 30;
uint32_t t0 = 0, dur = 0;
bool moving = false;
String line;

void writeUs(int j, float us) {
  ledcWrite(JOINT_PIN[j], on[j] ? (uint32_t)(us / 20000.0f * 16384) : 0);
}

void print(const char *tag) {
  Serial.printf("%s", tag);
  for (int j = 0; j < JOINT_COUNT; j++) Serial.printf(" %d", (int)lroundf(pos[j]));
  Serial.print(" | on");
  for (int j = 0; j < JOINT_COUNT; j++) Serial.printf(" %d", on[j]);
  Serial.println();
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
  for (int j = 0; j < JOINT_COUNT; j++) {
    ledcAttachChannel(JOINT_PIN[j], 50, 14, JOINT_CHANNEL[j]);
    on[j] = HOME_US[j] > 0;
    pos[j] = on[j] ? HOME_US[j] : 1500 + JOINT_TRIM_US[j];
    writeUs(j, pos[j]);
    if (on[j]) delay(400);
  }
  print("READY");
}

void handle(String s) {
  s.trim();
  if (!s.length()) return;
  char c = toupper(s[0]);
  float t[JOINT_COUNT];
  for (int j = 0; j < JOINT_COUNT; j++) t[j] = pos[j];
  if (c == 'M') {
    if (sscanf(s.c_str() + 1, "%f %f %f %f %f", &t[0], &t[1], &t[2], &t[3], &t[4]) == 5) moveTo(t);
    else Serial.println("ERR need 5 pulses");
  } else if (c == 'J') {
    int j; float us;
    if (sscanf(s.c_str() + 1, "%d %f", &j, &us) == 2 && j >= 0 && j < JOINT_COUNT) {
      for (int k = 0; k < JOINT_COUNT; k++) if (k != j && !on[k]) t[k] = pos[k];
      t[j] = us;
      bool keep[JOINT_COUNT];
      for (int k = 0; k < JOINT_COUNT; k++) keep[k] = on[k];
      moveTo(t);
      for (int k = 0; k < JOINT_COUNT; k++) if (k != j && !keep[k]) { on[k] = false; writeUs(k, 0); }
    } else Serial.println("ERR J <joint> <us>");
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
    degPerS = constrain(s.substring(1).toFloat(), 5, 120);
    print("AT");
  } else if (c == '?') print("AT");
  else Serial.println("ERR unknown command");
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handle(line); line = ""; }
    else line += ch;
  }
  if (moving) {
    float k = dur ? min(1.0f, (millis() - t0) / (float)dur) : 1.0f;
    float e = k * k * (3 - 2 * k);                       // ease in and out
    for (int j = 0; j < JOINT_COUNT; j++) {
      pos[j] = start[j] + (goal[j] - start[j]) * e;
      writeUs(j, pos[j]);
    }
    if (k >= 1) { moving = false; print("DONE"); }
  }
  delay(10);
}
