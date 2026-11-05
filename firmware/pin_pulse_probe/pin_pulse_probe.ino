// Pin pulse probe: sets up the five servo pins exactly like arm_serial (explicit LEDC channels), then sends pulses on
// ONE joint at a time while the others get duty 0 (limp), and samples every servo pin's actual level through the GPIO
// input register. Prints how often each pin was high: only the driven joint's pin should show ~7.5 % (1500 us / 20 ms).
// Tells a firmware/channel mix-up apart from a wiring mix-up without needing a meter.
#include "robot_config.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"

const char *NAMES[JOINT_COUNT] = {"base", "shoulder", "elbow", "wrist", "twist"};

int level(int pin) {
  return pin < 32 ? (REG_READ(GPIO_IN_REG) >> pin) & 1 : (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  for (int j = 0; j < JOINT_COUNT; j++) {
    ledcAttachChannel(JOINT_PIN[j], 50, 14, JOINT_CHANNEL[j]);
    ledcWrite(JOINT_PIN[j], 0);
    gpio_input_enable((gpio_num_t)JOINT_PIN[j]);           // read back what the pin really outputs
  }
}

void loop() {
  for (int d = 0; d < JOINT_COUNT; d++) {
    for (int j = 0; j < JOINT_COUNT; j++) ledcWrite(JOINT_PIN[j], j == d ? (uint32_t)(1500 / 20000.0f * 16384) : 0);
    delay(100);
    long high[JOINT_COUNT] = {0}, n = 0;
    uint32_t t = millis();
    while (millis() - t < 400) {                          // 20 PWM periods
      for (int j = 0; j < JOINT_COUNT; j++) high[j] += level(JOINT_PIN[j]);
      n++;
      delayMicroseconds(20);
    }
    Serial.printf("probe: pulses on %-8s (GPIO%2d) ->", NAMES[d], JOINT_PIN[d]);
    for (int j = 0; j < JOINT_COUNT; j++) Serial.printf("  GPIO%d %4.1f%%", JOINT_PIN[j], 100.0 * high[j] / n);
    Serial.println();
  }
  for (int j = 0; j < JOINT_COUNT; j++) ledcWrite(JOINT_PIN[j], 0);
  Serial.println("probe: done");
  while (true) delay(1000);
}
