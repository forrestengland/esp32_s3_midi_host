// midi_log.h
// Thread-safe MIDI event log. processMidi() (loop task) writes through
// logMidiMessage(); the media task reads a snapshot to draw the MIDI monitor.
//
// This REPLACES the logging code in your sketch. Delete these from the sketch:
//   MIDI_LOG_LINES, struct MidiLogEntry, midiLog[], midiLogCount,
//   midiDisplayDirty, noteName(), logMidiMessage(), drawMidiLog()
// (lastDisplayUpdate / DISPLAY_INTERVAL_MS are unused and can go too.)
// processMidi() stays as it is; it already calls logMidiMessage(...).
//
// Include order at the top of the sketch:
//   TFT_eSPI tft = TFT_eSPI();
//   #include "midi_log.h"
//   #include "mjpeg_video.h"

#pragma once

#include <Arduino.h>

constexpr int MIDI_LOG_LINES = 8;

struct MidiLogEntry {
  char     text[40];
  uint16_t color;
};

static MidiLogEntry    midiLogRing[MIDI_LOG_LINES];
static volatile uint32_t midiLogSeq = 0;   // total events logged since boot
static portMUX_TYPE    midiLogMux = portMUX_INITIALIZER_UNLOCKED;

static void midiLogPush(const char* text, uint16_t color) {
  MidiLogEntry e;
  strncpy(e.text, text, sizeof(e.text) - 1);
  e.text[sizeof(e.text) - 1] = 0;
  e.color = color;

  portENTER_CRITICAL(&midiLogMux);
  midiLogRing[midiLogSeq % MIDI_LOG_LINES] = e;
  midiLogSeq = midiLogSeq + 1;
  portEXIT_CRITICAL(&midiLogMux);
}

// Copies the newest entries into out[0..] (newest first) and returns the total
// event count. Valid entries: min(returned count, MIDI_LOG_LINES).
static uint32_t midiLogSnapshot(MidiLogEntry* out) {
  portENTER_CRITICAL(&midiLogMux);
  uint32_t seq = midiLogSeq;
  int count = (seq < (uint32_t)MIDI_LOG_LINES) ? (int)seq : MIDI_LOG_LINES;
  for (int i = 0; i < count; i++) {
    out[i] = midiLogRing[(seq - 1 - i) % MIDI_LOG_LINES];
  }
  portEXIT_CRITICAL(&midiLogMux);
  return seq;
}

static void noteNameInto(int note, char* out, size_t outSize) {
  static const char* names[] = {
    "C", "C#", "D", "D#", "E", "F",
    "F#", "G", "G#", "A", "A#", "B"
  };
  snprintf(out, outSize, "%s%d", names[note % 12], note / 12 - 1);
}

// Same signature the sketch already uses.
void logMidiMessage(const char* type, int note,
                    int velocity, int channel,
                    uint16_t color) {
  char name[8];
  char line[40];
  noteNameInto(note, name, sizeof(name));
  snprintf(line, sizeof(line), "%-7s CH%u %-4s V%03d",
           type, (unsigned)channel, name, velocity);
  midiLogPush(line, color);
}
