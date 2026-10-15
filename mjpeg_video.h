// mjpeg_video.h  (v2)
// Serial MJPEG playback on a shared SPI bus: SD read -> JPEG decode -> TFT push.
// Uses JPEGDEC (bitbank2) decoding into a full-frame buffer, then ONE burst push.
//
// Install: arduino-cli lib install "JPEGDEC"
// No longer needed: TJpg_Decoder. Also REMOVE tft.setSwapBytes(true) from your
// sketch if you added it: JPEGDEC outputs byte-swapped pixels already.
//
// Once videoBegin() has been called, only the video task may touch the TFT or SD.

#pragma once

#include <Arduino.h>
#include <SD.h>
#include <TFT_eSPI.h>
#include <JPEGDEC.h>
#include "esp_heap_caps.h"

extern TFT_eSPI tft;

// ---- Settings ----
constexpr size_t   MJPEG_BUF_SIZE = 48 * 1024;  // larger than your biggest frame
constexpr uint32_t VIDEO_FPS      = 30;         // target; it just runs as fast as it can
constexpr int      VIDEO_W        = 320;
constexpr int      VIDEO_H        = 240;

static File      videoFile;
static JPEGDEC   jpeg;
static uint8_t*  mjpegBuf = nullptr;
static size_t    mjpegLen = 0;
static uint16_t* frameBuf = nullptr;   // VIDEO_W * VIDEO_H RGB565, big-endian

// JPEGDEC calls this with blocks of decoded pixels. Copy into the frame buffer.
static int jpegDraw(JPEGDRAW* d) {
  int w = d->iWidth;
  if (d->x + w > VIDEO_W) w = VIDEO_W - d->x;
  for (int row = 0; row < d->iHeight; row++) {
    int y = d->y + row;
    if (y >= VIDEO_H) break;
    memcpy(frameBuf + y * VIDEO_W + d->x,
           d->pPixels + row * d->iWidth,
           w * 2);
  }
  return 1;  // keep decoding
}

// Tops up the buffer with ONE big sequential read, then finds the end of the
// first JPEG in the buffer (FF D9).
static bool loadFrame(size_t& frameLen) {
  int n = videoFile.read(mjpegBuf + mjpegLen, MJPEG_BUF_SIZE - mjpegLen);
  if (n > 0) mjpegLen += n;

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
  uint32_t usRead = 0, usDecode = 0, usPush = 0;

  for (;;) {
    size_t frameLen = 0;

    uint32_t t = micros();
    bool ok = loadFrame(frameLen);
    usRead += micros() - t;

    if (!ok) {
      videoFile.seek(0);   // end of file: loop the video
      mjpegLen = 0;
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    // Decode into frameBuf (no TFT access during this).
    t = micros();
    if (jpeg.openRAM(mjpegBuf, frameLen, jpegDraw)) {
      jpeg.setPixelType(RGB565_BIG_ENDIAN);
      jpeg.decode(0, 0, 0);
      jpeg.close();
    }
    usDecode += micros() - t;

    // Push the whole frame in one burst.
    t = micros();
    tft.startWrite();
    tft.setAddrWindow(0, 0, VIDEO_W, VIDEO_H);
    tft.pushPixels(frameBuf, VIDEO_W * VIDEO_H);
    tft.endWrite();
    usPush += micros() - t;

    // Drop the consumed frame, keep bytes already read for the next one.
    memmove(mjpegBuf, mjpegBuf + frameLen, mjpegLen - frameLen);
    mjpegLen -= frameLen;

    // Frame pacing: sleep if ahead, never accumulate debt if behind.
    nextFrameAt += intervalMs;
    int32_t wait = (int32_t)(nextFrameAt - millis());
    if (wait > 0) {
      vTaskDelay(pdMS_TO_TICKS(wait));
    } else if (wait < -(int32_t)intervalMs) {
      nextFrameAt = millis();
    }

    if (++frameCount == 60) {
      uint32_t elapsed = millis() - statStart;
      Serial.printf("Video: %.1f fps | read %lu ms, decode %lu ms, push %lu ms per frame\n",
                    60000.0f / elapsed,
                    (unsigned long)(usRead / 60000),
                    (unsigned long)(usDecode / 60000),
                    (unsigned long)(usPush / 60000));
      usRead = usDecode = usPush = 0;
      frameCount = 0;
      statStart = millis();
    }
  }
}

// Call from setup() AFTER tft.init() and SD.begin() have succeeded.
bool videoBegin(const char* path) {
  mjpegBuf = (uint8_t*)malloc(MJPEG_BUF_SIZE);
  if (!mjpegBuf) {
    Serial.println("Video: JPEG buffer allocation failed");
    return false;
  }

  // Frame buffer: try internal RAM first (faster), fall back to PSRAM.
  const size_t fbBytes = VIDEO_W * VIDEO_H * 2;
  frameBuf = (uint16_t*)heap_caps_malloc(fbBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (frameBuf) {
    Serial.println("Video: frame buffer in internal RAM");
  } else {
    frameBuf = (uint16_t*)heap_caps_malloc(fbBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (frameBuf) Serial.println("Video: frame buffer in PSRAM");
  }
  if (!frameBuf) {
    Serial.println("Video: frame buffer allocation failed (no room, enable PSRAM?)");
    return false;
  }

  videoFile = SD.open(path, FILE_READ);
  if (!videoFile) {
    Serial.printf("Video: cannot open %s\n", path);
    return false;
  }

  tft.setSwapBytes(false);   // JPEGDEC already outputs big-endian RGB565

  BaseType_t ok = xTaskCreatePinnedToCore(videoTask, "Video", 10240, nullptr, 1, nullptr, 1);
  if (ok != pdPASS) {
    Serial.println("Video: failed to create task");
    return false;
  }
  return true;
}
