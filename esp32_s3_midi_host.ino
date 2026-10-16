#include <Arduino.h>
#include <math.h>

#include <ESP32_Host_MIDI.h>
#include <USBConnection.h>

#include "driver/i2s_std.h"

#include <TFT_eSPI.h>

#include <SPI.h>
#include <SD.h>

constexpr int SD_CS = 10;
constexpr int SD_MISO = 9;

TFT_eSPI tft = TFT_eSPI();

#include "midi_log.h"
#include "mjpeg_video.h"
#include "encoder_input.h"

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

#include "audio_duplex.h"

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
	encoderPoll();

	if (Serial.available()) {
		switch (Serial.read()) {
		case 'r': mediaCmd = CMD_REC_START;  break;
		case 'l': mediaCmd = CMD_PLAY_START; break;
		case 'v': mediaCmd = CMD_VIDEO_START; break;			
		case 's': mediaCmd = CMD_STOP;       break;
		case 'p':
			passthroughEnabled = !passthroughEnabled;
			Serial.printf("Passthrough %s\n", passthroughEnabled ? "ON" : "OFF");
			break;
		}
	}

	delay(1);
}

bool drawBmp(const char *filename, int16_t x, int16_t y) {

	Serial.println("BMP: opening file");

    File bmp = SD.open(filename, FILE_READ);

    Serial.println("BMP: open returned");

    if (!bmp) {
        Serial.printf("Cannot open %s\n", filename);
        return false;
    }

    Serial.println("BMP: file opened");

    // BMP header
    if (bmp.read() != 'B' || bmp.read() != 'M') {
        Serial.println("Not a valid BMP file");
        bmp.close();
        return false;
    }

    bmp.seek(10);
    uint32_t pixelOffset;
    bmp.read((uint8_t *)&pixelOffset, 4);

    bmp.seek(18);
    int32_t width, height;
    bmp.read((uint8_t *)&width, 4);
    bmp.read((uint8_t *)&height, 4);

    uint16_t planes, depth;
    uint32_t compression;

    bmp.seek(26);
    bmp.read((uint8_t *)&planes, 2);
    bmp.read((uint8_t *)&depth, 2);
    bmp.read((uint8_t *)&compression, 4);

    if (width != 320 || abs(height) != 240 ||
        planes != 1 || depth != 24 || compression != 0) {
        Serial.printf(
            "Unsupported BMP: %ldx%ld, %u-bit, compression %lu\n",
            (long)width, (long)height, depth, (unsigned long)compression
        );
        bmp.close();
        return false;
    }

    // BMP rows are padded to a multiple of 4 bytes.
    uint32_t rowSize = (width * 3 + 3) & ~3;
    bool bottomUp = height > 0;
    uint8_t row[320 * 3 + 4];

    // Send each row as RGB565 pixels.
    uint16_t pixels[320];

		Serial.println("BMP: starting pixel loop");

    for (int rowIndex = 0; rowIndex < 240; rowIndex++) {
        int sourceRow = bottomUp ? 239 - rowIndex : rowIndex;

        if (rowIndex == 0) {
            Serial.println("BMP: seeking first row");
        }

        bmp.seek(pixelOffset + (uint32_t)sourceRow * rowSize);

        if (rowIndex == 0) {
            Serial.println("BMP: reading first row");
        }

        int bytesRead = bmp.read(row, rowSize);

        if (rowIndex == 0) {
            Serial.printf("BMP: first row read %d bytes\n", bytesRead);
        }

        if (bytesRead != (int)rowSize) {
            Serial.println("Error reading BMP pixels");
            bmp.close();
            return false;
        }

        for (int col = 0; col < 320; col++) {
            uint8_t b = row[col * 3 + 0];
            uint8_t g = row[col * 3 + 1];
            uint8_t r = row[col * 3 + 2];

            pixels[col] = tft.color565(r, g, b);
        }

        if (rowIndex == 0) {
            Serial.println("BMP: pushing first row to TFT");
        }

				tft.startWrite();
				tft.setAddrWindow(x, y + rowIndex, 320, 1);
				tft.pushColors(pixels, 320, true);
				tft.endWrite();
				//        tft.pushImage(x, y + rowIndex, 320, 1, pixels);

        if (rowIndex == 0) {
            Serial.println("BMP: first row displayed");
        }
    }

		Serial.println("BMP: finished pixel loop");

    bmp.close();
    Serial.printf("Displayed %s\n", filename);
    return true;
}


void setup() {

	Serial.begin(115200);

	pinMode(10, OUTPUT);
	digitalWrite(10, HIGH);

	tft.init();
	tft.setRotation(1);
	tft.fillScreen(TFT_BLACK);

	Serial.printf("TFT_MOSI=%d, TFT_SCLK=%d, TFT_MISO=%d\n",
              TFT_MOSI, TFT_SCLK, TFT_MISO);

	Serial.println("Initializing SD...");

	SPIClass& tftSPI = tft.getSPIinstance();

	Serial.println("Initializing SD...");

	bool sdOK = SD.begin(10, tftSPI, 20000000);
	Serial.printf("SD init: %s\n", sdOK ? "OK" : "FAILED");

	/*	Serial.println("Testing TFT...");
	tft.fillScreen(TFT_RED);
	Serial.println("TFT fill complete");

	File bmp = SD.open("/SUNSET.BMP", FILE_READ);

	if (bmp) {
		Serial.println("BMP opened successfully");
		bmp.close();
		drawBmp("/SUNSET.BMP", 0, 0);
	} else {
		Serial.println("BMP open failed");
		} */

	encoderBegin();

  delay(500);

  Serial.println("USB MIDI Sine Synth");

  setupAudio();

	passthroughEnabled = 1;

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

	setNoteOn(60, 127);

	if (sdOK) videoBegin("/video.mjpeg");
}
