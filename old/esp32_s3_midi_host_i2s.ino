
#include <Arduino.h>
#include <math.h>

#include <ESP32_Host_MIDI.h>
#include <USBConnection.h>

#include "driver/i2s_std.h"

// -------- I2S wiring --------
constexpr int I2S_BCLK = 5;
constexpr int I2S_LRCK = 6;
constexpr int I2S_DOUT = 7;

// -------- Audio settings --------
constexpr int SAMPLE_RATE = 44100;
constexpr int BLOCK_FRAMES = 128;
// constexpr float TWO_PI = 6.28318530718f;
constexpr float MAX_AMPLITUDE = 0.25f;

// -------- MIDI --------
USBConnection usbHost;
static uint64_t lastEventIndex = 0;

int activeNote = -1;
float targetAmplitude = 0.0f;
float currentAmplitude = 0.0f;
float phase = 0.0f;
float phaseIncrement = 0.0f;

// -------- I2S driver --------
i2s_chan_handle_t txHandle = nullptr;

void setupAudio() {
  i2s_chan_config_t chanConfig =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

  ESP_ERROR_CHECK(
      i2s_new_channel(&chanConfig, &txHandle, nullptr)
  );

  i2s_std_config_t stdConfig = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),

      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
          I2S_DATA_BIT_WIDTH_16BIT,
          I2S_SLOT_MODE_STEREO
      ),

      .gpio_cfg = {
          .mclk = I2S_GPIO_UNUSED,
          .bclk = (gpio_num_t)I2S_BCLK,
          .ws = (gpio_num_t)I2S_LRCK,
          .dout = (gpio_num_t)I2S_DOUT,
          .din = I2S_GPIO_UNUSED,
          .invert_flags = {
              .mclk_inv = false,
              .bclk_inv = false,
              .ws_inv = false
          }
      }
  };

  ESP_ERROR_CHECK(
      i2s_channel_init_std_mode(txHandle, &stdConfig)
  );

  ESP_ERROR_CHECK(i2s_channel_enable(txHandle));

  Serial.println("I2S audio started.");
}

void setNoteOn(int note, int velocity) {
  if (velocity <= 0) {
    if (activeNote == note) {
      targetAmplitude = 0.0f;
    }
    return;
  }

  activeNote = note;

  // MIDI note 69 = A4 = 440 Hz
  float frequency = 440.0f *
      powf(2.0f, (note - 69) / 12.0f);

  phaseIncrement = TWO_PI * frequency / SAMPLE_RATE;

  targetAmplitude =
      MAX_AMPLITUDE * (velocity / 127.0f);

  Serial.printf(
      "Note ON: %d, frequency: %.2f Hz\n",
      note, frequency
  );
}

void setNoteOff(int note) {
  // Only release the currently sounding note
  if (activeNote == note) {
    targetAmplitude = 0.0f;
    activeNote = -1;

    Serial.printf("Note OFF: %d\n", note);
  }
}

void processMidi() {
  midiHandler.task();

  const auto& queue = midiHandler.getQueue();

  for (const auto& ev : queue) {
    // Each queued event should be processed only once
    if (ev.index <= lastEventIndex) {
      continue;
    }
    lastEventIndex = ev.index;

    uint8_t status = ev.statusCode & 0xF0;

    // Ignore pitch bend and control change
    if (status == 0xE0 || status == 0xB0) {
      continue;
    }

    // Ignore all events except Note On and Note Off
    if (status != 0x90 && status != 0x80) {
      continue;
    }

    int note = ev.noteNumber;
    int velocity = ev.velocity7;

    if (note < 0 || note > 127) {
      continue;
    }

    // Note On with velocity zero means Note Off
    if (status == 0x90 && velocity > 0) {
      setNoteOn(note, velocity);
    } else {
      setNoteOff(note);
    }
  }
}

void loop() {
  // Handle MIDI before producing each audio block
  processMidi();

  // Interleaved left/right 16-bit samples
  int16_t buffer[BLOCK_FRAMES * 2];

  // Approximately 5 ms attack/release ramp
  constexpr float RAMP_STEP =
      1.0f / (SAMPLE_RATE * 0.005f);

  for (int i = 0; i < BLOCK_FRAMES; i++) {
    if (currentAmplitude < targetAmplitude) {
      currentAmplitude += RAMP_STEP;
      if (currentAmplitude > targetAmplitude) {
        currentAmplitude = targetAmplitude;
      }
    } else if (currentAmplitude > targetAmplitude) {
      currentAmplitude -= RAMP_STEP;
      if (currentAmplitude < targetAmplitude) {
        currentAmplitude = targetAmplitude;
      }
    }

    float sample = 0.0f;

    if (currentAmplitude > 0.00001f) {
      sample = sinf(phase) * currentAmplitude;

      phase += phaseIncrement;
      if (phase >= TWO_PI) {
        phase -= TWO_PI;
      }
    }

    int16_t pcm =
        (int16_t)(sample * 32767.0f);

    buffer[i * 2] = pcm;       // Left
    buffer[i * 2 + 1] = pcm;   // Right
  }

  size_t bytesWritten = 0;

  esp_err_t result = i2s_channel_write(
      txHandle,
      buffer,
      sizeof(buffer),
      &bytesWritten,
      1000
  );

  if (result != ESP_OK) {
    Serial.printf("I2S write error: %d\n", result);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("USB MIDI Sine Synth");

  setupAudio();

  midiHandler.addTransport(&usbHost);
  usbHost.begin();
  midiHandler.begin();

  Serial.println("Ready. Press a key on your MIDI keyboard.");
}
