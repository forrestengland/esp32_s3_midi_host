#include <Arduino.h>
#include <math.h>

#include <ESP32_Host_MIDI.h>
#include <USBConnection.h>

#include "driver/i2s_std.h"

#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

constexpr int MIDI_LOG_LINES = 8;

struct MidiLogEntry {
  char text[40];
  uint16_t color;
};

MidiLogEntry midiLog[MIDI_LOG_LINES];
int midiLogCount = 0;
bool midiDisplayDirty = false;

uint32_t lastDisplayUpdate = 0;
constexpr uint32_t DISPLAY_INTERVAL_MS = 100;

// Protect shared audio parameters between MIDI and audio tasks
portMUX_TYPE synthMux = portMUX_INITIALIZER_UNLOCKED;

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
    setNoteOff(note);
    return;
  }

  float frequency = 440.0f *
      powf(2.0f, (note - 69) / 12.0f);

  float increment = TWO_PI * frequency / SAMPLE_RATE;
  float amplitude = MAX_AMPLITUDE * (velocity / 127.0f);

  portENTER_CRITICAL(&synthMux);
  activeNote = note;
  phaseIncrement = increment;
  targetAmplitude = amplitude;
  portEXIT_CRITICAL(&synthMux);

  Serial.printf("Note ON: %d, frequency: %.2f Hz\n",
                note, frequency);
}

void setNoteOff(int note) {
  bool released = false;

  portENTER_CRITICAL(&synthMux);
  if (activeNote == note) {
    targetAmplitude = 0.0f;
    activeNote = -1;
    released = true;
  }
  portEXIT_CRITICAL(&synthMux);

  if (released) {
    Serial.printf("Note OFF: %d\n", note);
  }
}

const char* noteName(int note) {
  static const char* names[] = {
    "C", "C#", "D", "D#", "E", "F",
    "F#", "G", "G#", "A", "A#", "B"
  };

  static char name[8];
  snprintf(name, sizeof(name), "%s%d",
           names[note % 12], note / 12 - 1);
  return name;
}

void logMidiMessage(const char* type, int note,
                    int velocity, int channel,
                    uint16_t color) {
  if (midiLogCount < MIDI_LOG_LINES) {
    midiLogCount++;
  }

  for (int i = midiLogCount - 1; i > 0; i--) {
    midiLog[i] = midiLog[i - 1];
  }

  snprintf(midiLog[0].text, sizeof(midiLog[0].text),
           "%-7s CH%u %-4s V%03d",
           type, channel, noteName(note), velocity);

  midiLog[0].color = color;
  midiDisplayDirty = true;
}

void drawMidiLog() {

  if (!midiDisplayDirty) {
    return;
  }

  midiDisplayDirty = false;

  // Clear only the message area, preserving the header.
  tft.fillRect(0, 42, 320, 198, TFT_BLACK);

  tft.setTextSize(1);

  for (int i = 0; i < midiLogCount; i++) {
    int y = 48 + i * 23;

    tft.setTextColor(midiLog[i].color, TFT_BLACK);
    tft.setCursor(10, y);
    tft.println(midiLog[i].text);
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
		//   if (status == 0x90 && velocity > 0) {
		//      setNoteOn(note, velocity);
		//    } else {
		//      setNoteOff(note);
		//    }
		int channel = (ev.statusCode & 0x0F) + 1;

		if (status == 0x90 && velocity > 0) {
			setNoteOn(note, velocity);

			logMidiMessage(
										 "NOTE ON", note, velocity, channel, TFT_GREEN);
		} else {
			setNoteOff(note);

			logMidiMessage(
										 "NOTE OFF", note, velocity, channel, TFT_YELLOW);
		}		
  }
}

void loop() {
  processMidi();

  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
    lastDisplayUpdate = millis();
    drawMidiLog();
  }
}

void audioTask(void* parameter) {
  int16_t buffer[BLOCK_FRAMES * 2];

  constexpr float RAMP_STEP =
      1.0f / (SAMPLE_RATE * 0.005f);

  for (;;) {
    // Take a consistent snapshot of MIDI-controlled parameters.
    float blockTargetAmplitude;
    float blockPhaseIncrement;

    portENTER_CRITICAL(&synthMux);
    blockTargetAmplitude = targetAmplitude;
    blockPhaseIncrement = phaseIncrement;
    portEXIT_CRITICAL(&synthMux);

    for (int i = 0; i < BLOCK_FRAMES; i++) {
      if (currentAmplitude < blockTargetAmplitude) {
        currentAmplitude += RAMP_STEP;
        if (currentAmplitude > blockTargetAmplitude)
          currentAmplitude = blockTargetAmplitude;
      } else if (currentAmplitude > blockTargetAmplitude) {
        currentAmplitude -= RAMP_STEP;
        if (currentAmplitude < blockTargetAmplitude)
          currentAmplitude = blockTargetAmplitude;
      }

      float sample = 0.0f;

      if (currentAmplitude > 0.00001f) {
        sample = sinf(phase) * currentAmplitude;

        phase += blockPhaseIncrement;
        if (phase >= TWO_PI)
          phase -= TWO_PI;
      }

      int16_t pcm = (int16_t)(sample * 32767.0f);

      buffer[i * 2] = pcm;
      buffer[i * 2 + 1] = pcm;
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
}

void setup() {
  Serial.begin(115200);

	tft.init();
	tft.setRotation(1);  // Landscape
	tft.fillScreen(TFT_BLACK);
	tft.setTextSize(2);
	tft.setTextColor(TFT_GREEN, TFT_BLACK);
	tft.setCursor(10, 10);
	tft.println("MIDI MONITOR");
	tft.drawFastHLine(10, 35, 300, TFT_DARKGREY);
	
  delay(500);

  Serial.println("USB MIDI Sine Synth");

  setupAudio();

	BaseType_t taskResult = xTaskCreatePinnedToCore(
																									audioTask,
																									"AudioTask",
																									4096,
																									nullptr,
																									3,
																									nullptr,
																									0                 // Core 0
																									);

	if (taskResult != pdPASS) {
		Serial.println("ERROR: Failed to create audio task!");
		while (true) {
			delay(1000);
		}
	}	

  midiHandler.addTransport(&usbHost);
  usbHost.begin();
  midiHandler.begin();

  Serial.println("Ready. Press a key on your MIDI keyboard.");
}
