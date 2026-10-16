// encoder_input.h
// KY-040 style rotary encoder with push button.
// Rotation is counted by the S3's pulse-counter hardware (ESP32Encoder library),
// so turns aren't missed while the video task is busy. encoderPoll() just turns
// the counts and button state into UiEvents.
//
// Install: arduino-cli lib install "ESP32Encoder"
//
// Wiring (KY-040 module): + to 3.3V (NOT 5V), GND to GND, CLK/DT/SW to the pins below.
// Do not use GPIO 19/20 (USB host data pins, used by your MIDI keyboard).

#pragma once

#include <Arduino.h>
#include <ESP32Encoder.h>
#include "ui_events.h"

// ---- Pins ----
constexpr int ENC_CLK = 21;
constexpr int ENC_DT  = 47;
constexpr int ENC_SW  = 48;   // PLACEHOLDER: change to a free GPIO your TFT doesn't use

// ---- Tuning ----
constexpr int      COUNTS_PER_DETENT = 4;     // full-quad counts per click; try 2 or 1 if it skips
constexpr bool     ENC_REVERSE       = true; // flip if clockwise moves the menu up
constexpr uint32_t DEBOUNCE_MS       = 25;
constexpr uint32_t LONG_PRESS_MS     = 600;

static ESP32Encoder encoder;
static int64_t      encLastCount = 0;
static int          encAccum = 0;

static bool     btnRaw = false;        // last raw reading (true = pressed)
static bool     btnStable = false;     // debounced state (true = pressed)
static uint32_t btnChangeAt = 0;
static uint32_t btnPressStart = 0;
static bool     btnLongFired = false;

void encoderBegin() {
  uiEventsInit();

  // If this line doesn't compile on your library version, use puType::UP.
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(ENC_CLK, ENC_DT);
  encoder.setFilter(1023);     // hardware glitch filter (contact bounce)
  encoder.clearCount();
  encLastCount = 0;

  pinMode(ENC_SW, INPUT_PULLUP);
}

// Call from loop().
void encoderPoll() {
  // ---- Rotation ----
  int64_t count = encoder.getCount();
  encAccum += (int)(count - encLastCount);
  encLastCount = count;

  while (encAccum >= COUNTS_PER_DETENT) {
    encAccum -= COUNTS_PER_DETENT;
    uiSend(ENC_REVERSE ? UI_UP : UI_DOWN);
  }
  while (encAccum <= -COUNTS_PER_DETENT) {
    encAccum += COUNTS_PER_DETENT;
    uiSend(ENC_REVERSE ? UI_DOWN : UI_UP);
  }

  // ---- Button (active low) ----
  uint32_t now = millis();
  bool raw = (digitalRead(ENC_SW) == LOW);

  if (raw != btnRaw) {
    btnRaw = raw;
    btnChangeAt = now;
  }

  if ((now - btnChangeAt) >= DEBOUNCE_MS && raw != btnStable) {
    btnStable = raw;
    if (btnStable) {
      btnPressStart = now;
      btnLongFired = false;
    } else if (!btnLongFired) {
      uiSend(UI_CLICK);          // released without a long press
    }
  }

  if (btnStable && !btnLongFired && (now - btnPressStart) >= LONG_PRESS_MS) {
    btnLongFired = true;
    uiSend(UI_LONG);
  }
}

/* ------------------------------------------------------------------
   Sketch changes
   ------------------------------------------------------------------

   Includes at the top of the sketch:

     TFT_eSPI tft = TFT_eSPI();
     #include "mjpeg_video.h"
     #include "encoder_input.h"
     ...your globals (SAMPLE_RATE, BLOCK_FRAMES, synthMux, txHandle...)
     #include "audio_duplex.h"

   setup(): call encoderBegin() before videoBegin():

     encoderBegin();
     ...
     if (sdOK) videoBegin("/video.mjpeg");

   loop():

     void loop() {
       processMidi();
       encoderPoll();

       if (Serial.available()) {
         switch (Serial.read()) {
           case 'r': mediaCmd = CMD_REC_START;   break;
           case 'l': mediaCmd = CMD_PLAY_START;  break;
           case 'v': mediaCmd = CMD_VIDEO_START; break;
           case 's': mediaCmd = CMD_STOP;        break;
           case 'p':
             passthroughEnabled = !passthroughEnabled;
             Serial.printf("Passthrough %s\n", passthroughEnabled ? "ON" : "OFF");
             break;
         }
       }

       delay(1);
     }
   ------------------------------------------------------------------ */
