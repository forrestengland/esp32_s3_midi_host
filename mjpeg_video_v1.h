// mjpeg_video.h
// Serial MJPEG playback: SD read -> JPEG decode -> TFT push, one after another.
// Assumes TFT and SD share one SPI bus (SUPPORT_TRANSACTIONS enabled),
// and SPI_FREQUENCY is 40000000 in your TFT_eSPI setup file.
//
// Requires the "TJpg_Decoder" library (Bodmer).
//
// IMPORTANT: once videoBegin() has been called, only the video task may touch
// the TFT or the SD card. Don't call tft.* or SD.* from loop() or elsewhere.

#pragma once

#include <Arduino.h>
#include <SD.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>

extern TFT_eSPI tft;

// ---- Settings ----
constexpr size_t   MJPEG_BUF_SIZE = 48 * 1024;  // must be larger than your biggest frame
constexpr uint32_t VIDEO_FPS      = 15;         // target rate; slower hardware just runs behind

static File     videoFile;
static uint8_t* mjpegBuf = nullptr;
static size_t   mjpegLen = 0;   // valid bytes currently in mjpegBuf

static uint32_t usRead = 0, usDecode = 0, usPush = 0;

// TJpg_Decoder calls this for each decoded block (usually 16x16 or 8x8 MCUs).
static bool tftOutput(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (y >= tft.height()) return false;  // stop decoding if we're off-screen
  uint32_t t = micros();
  tft.pushImage(x, y, w, h, bitmap);
  usPush += micros() - t;
  return true;
}

// Tops up the buffer with ONE big sequential read, then finds the end of the
// first JPEG in the buffer (FF D9). Inside JPEG entropy data, FF is always
// followed by 00 or an RSTn marker, so FF D9 is a reliable end-of-image.
static bool loadFrame(size_t& frameLen) {
  int n = videoFile.read(mjpegBuf + mjpegLen, MJPEG_BUF_SIZE - mjpegLen);
  if (n > 0) mjpegLen += n;

  // Need at least a start marker (FF D8) at the front of the buffer.
  if (mjpegLen < 4 || mjpegBuf[0] != 0xFF || mjpegBuf[1] != 0xD8) {
    return false;  // end of file (or corrupt data)
  }

  for (size_t i = 2; i + 1 < mjpegLen; i++) {
    if (mjpegBuf[i] == 0xFF && mjpegBuf[i + 1] == 0xD9) {
      frameLen = i + 2;
      return true;
    }
  }

  if (mjpegLen == MJPEG_BUF_SIZE) {
    Serial.println("Video: frame larger than MJPEG_BUF_SIZE, raise it or lower quality");
  }
  return false;
}

static void videoTask(void*) {
  const uint32_t intervalMs = 1000 / VIDEO_FPS;
  uint32_t nextFrameAt = millis();
  uint32_t frameCount = 0;
  uint32_t statStart = millis();

  for (;;) {
    size_t frameLen = 0;

    uint32_t t0 = micros();
    bool ok = loadFrame(frameLen);
    usRead += micros() - t0;

    if (!ok) {
      // End of file: loop the video.
      videoFile.seek(0);
      mjpegLen = 0;
      vTaskDelay(pdMS_TO_TICKS(1));  // avoids a tight spin on an empty/bad file
      continue;
    }

    // Decode + push to TFT (blocks until done). Serial: nothing else
    // touches the bus during this.
    t0 = micros();
    TJpgDec.drawJpg(0, 0, mjpegBuf, frameLen);
    usDecode += micros() - t0;

    // Drop the consumed frame, keep any bytes already read for the next one.
    memmove(mjpegBuf, mjpegBuf + frameLen, mjpegLen - frameLen);
    mjpegLen -= frameLen;

    // Frame pacing. If we're running behind, don't sleep and don't accumulate debt.
    nextFrameAt += intervalMs;
    int32_t wait = (int32_t)(nextFrameAt - millis());
    if (wait > 0) {
      vTaskDelay(pdMS_TO_TICKS(wait));
    } else if (wait < -(int32_t)intervalMs) {
      nextFrameAt = millis();
    }

    // Report real fps every 60 frames.
    if (++frameCount == 60) {
      uint32_t elapsed = millis() - statStart;
      Serial.printf("Video: %.1f fps | read %lu ms, decode %lu ms, push %lu ms per frame\n", 60000.0f / elapsed, usRead / 60000, (usDecode - usPush) / 60000, usPush / 60000);
      frameCount = 0;
      statStart = millis();
    }
  }
}

// Call from setup() AFTER tft.init() and SD.begin() have succeeded.
bool videoBegin(const char* path) {
  mjpegBuf = (uint8_t*)malloc(MJPEG_BUF_SIZE);
  if (!mjpegBuf) {
    Serial.println("Video: buffer allocation failed");
    return false;
  }

  videoFile = SD.open(path, FILE_READ);
  if (!videoFile) {
    Serial.printf("Video: cannot open %s\n", path);
    return false;
  }

  tft.setSwapBytes(true);        // TJpg_Decoder outputs byte-swapped RGB565 for TFT_eSPI
  TJpgDec.setJpgScale(1);        // 1, 2, 4 or 8: decode at reduced size if you want speed
  TJpgDec.setCallback(tftOutput);

  // Core 1, same priority as loop(). loop() handles MIDI, this handles video.
  BaseType_t ok = xTaskCreatePinnedToCore(videoTask, "Video", 8192, nullptr, 1, nullptr, 1);
  if (ok != pdPASS) {
    Serial.println("Video: failed to create task");
    return false;
  }
  return true;
}

/* ------------------------------------------------------------------
   Changes to your sketch
   ------------------------------------------------------------------

   1. At the top, after `TFT_eSPI tft = TFT_eSPI();`
        #include "mjpeg_video.h"

   2. In setup(), replace the SD init and the BMP test with:

        pinMode(SD_CS, OUTPUT);
        digitalWrite(SD_CS, HIGH);

        tft.init();
        tft.setRotation(1);
        tft.fillScreen(TFT_BLACK);

        SPIClass& tftSPI = tft.getSPIinstance();
        bool sdOK = SD.begin(SD_CS, tftSPI, 20000000);   // try 10000000 if unreliable
        Serial.printf("SD init: %s\n", sdOK ? "OK" : "FAILED");

      ...and at the END of setup(), after midiHandler.begin():

        if (sdOK) videoBegin("/video.mjpeg");

   3. loop() becomes:

        void loop() {
          processMidi();
          delay(1);        // lets the video task (same core, same priority) run
        }

   4. Don't call drawMidiLog() or drawBmp() while video is playing; they would
      fight the video task for the TFT/SD bus.

   5. Make the video file (baseline JPEG, which is ffmpeg's default for mjpeg):

        ffmpeg -i in.mp4 \
          -vf "fps=15,scale=320:240:force_original_aspect_ratio=decrease,pad=320:240:(ow-iw)/2:(oh-ih)/2" \
          -q:v 7 -an -f mjpeg video.mjpeg

      Copy it to the SD card root as video.mjpeg. Check average frame size:
      file size / frame count should be well under MJPEG_BUF_SIZE (15-20 KB is typical).
   ------------------------------------------------------------------ */
