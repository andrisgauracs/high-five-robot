// Joint calibration, all five joints: holds every joint at its trimmed centre (robot_config.h), explicit LEDC.
// Serial 115200 commands:  p / s / e / w / t  select pan / shoulder / elbow / wrist / twist
//                          + / -  nudge 10 us (~1 deg)    ] / [  nudge 2 us    c  selected joint back to its centre
//                          a number like 1520 (then Enter)  set the selected joint to that pulse
// Every change prints all five pulses and their offsets from 1500 (= the trim to put in robot_config.h).
#include "robot_config.h"

const char *NAMES[JOINT_COUNT] = {"pan", "shoulder", "elbow", "wrist", "twist"};
int usNow[JOINT_COUNT];
int sel = WRIST;
String line;

void writeUs(int j) {
  usNow[j] = constrain(usNow[j], 600, 2400);
  ledcWrite(JOINT_PIN[j], (uint32_t)((float)usNow[j] / 20000.0f * 16384));
}
void report() {
  Serial.printf("joint_cal5: selected %s |", NAMES[sel]);
  for (int j = 0; j < JOINT_COUNT; j++) Serial.printf(" %s %d (%+d)", NAMES[j], usNow[j], usNow[j] - 1500);
  Serial.println(" us");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  for (int j = 0; j < JOINT_COUNT; j++) {
    ledcAttachChannel(JOINT_PIN[j], 50, 14, JOINT_CHANNEL[j]);
    usNow[j] = 1500 + JOINT_TRIM_US[j];
    writeUs(j);
    delay(700);                                  // one at a time, so they don't all jump at once
  }
  report();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'p') { sel = PAN; report(); }
    else if (ch == 's') { sel = SHOULDER; report(); }
    else if (ch == 'e') { sel = ELBOW; report(); }
    else if (ch == 'w') { sel = WRIST; report(); }
    else if (ch == 't') { sel = TWIST; report(); }
    else if (ch == '+') { usNow[sel] += 10; writeUs(sel); report(); }
    else if (ch == '-') { usNow[sel] -= 10; writeUs(sel); report(); }
    else if (ch == ']') { usNow[sel] += 2; writeUs(sel); report(); }
    else if (ch == '[') { usNow[sel] -= 2; writeUs(sel); report(); }
    else if (ch == 'c') { usNow[sel] = 1500 + JOINT_TRIM_US[sel]; writeUs(sel); report(); }
    else if (isDigit(ch)) line += ch;
    else if ((ch == '\n' || ch == '\r') && line.length()) { usNow[sel] = line.toInt(); line = ""; writeUs(sel); report(); }
  }
}
