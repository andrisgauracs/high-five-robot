// High-Five Robot: one place for pins, LEDC channels, trims and directions.
// Copy into a sketch folder (Arduino only picks up headers next to the .ino) or include by path.
// Servos are driven with explicit LEDC channels, not the ESP32Servo library (it reuses channel 0 for the
// third servo on this ESP32-S3 / core 3.x).
#pragma once

enum Joint { PAN = 0, SHOULDER = 1, ELBOW = 2, WRIST = 3, TWIST = 4, JOINT_COUNT = 5 };

const int JOINT_PIN[JOINT_COUNT]     = {4, 5, 6, 7, 15};
const int JOINT_CHANNEL[JOINT_COUNT] = {0, 1, 2, 3, 4};

// Pulse at which each joint sits at its calibrated reference, as an offset from 1500 us
// (pan/elbow measured 2026-10-07, all five confirmed by the user 2026-10-08 with firmware/joint_cal5)
//   pan:      1500 us = pointing straight at the camera
//   shoulder: 1500 us = user's reference position (accepted as is)
//   elbow:    1450 us = forearm straight
//   wrist:    1580 us = hand straight in line with the forearm
//   twist:    1500 us = palm flat, facing down
const int JOINT_TRIM_US[JOINT_COUNT] = {0, 0, -50, 80, 0};

// Which way a LOWER pulse moves the joint (observed): shoulder and elbow move up/back towards the
// electronics bay with a lower pulse, and forward/down towards the camera with a higher one.
// Wrist: a higher pulse moves the hand towards the camera side (seen while trimming, 2026-10-08).
// Pan and twist directions not yet recorded.

const float US_PER_DEG = 2000.0f / 180.0f;   // MG995 / MG90S: ~500..2500 us for ~180 deg

// Home (rest) pose the arm goes to on every boot, in us. Separate from the trims above, which stay the kinematic
// reference ("straight") positions. 0 = no home set for that joint: it stays limp (no pulse) until commanded.
// Set with tools/arm_jog.py ("Save as home").
const int HOME_US[JOINT_COUNT] = {0, 0, 0, 0, 0};
