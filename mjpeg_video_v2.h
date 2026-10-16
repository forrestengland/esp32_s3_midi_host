// mjpeg_video.h  (v3: media task)
// One task owns the TFT and the SD card and runs in one of three states:
//   M_VIDEO  - MJPEG playback (default)
//   M_RECORD - writes ADC audio to /rec.raw (video stopped)
//   M_PLAY   - streams /rec.raw into the audio output (video stopped)
//
// Raw file format: signed 16-bit little-endian, stereo, 44100 Hz.
//
// Requires: JPEGDEC library, audio_shared.h.
// Only the media task may touch the TFT or SD after videoBegin().

#pragma once

#include <Arduino.h>
#include <SD.h>
#include <TFT_eSPI.h>
#include <JPEGDEC.h>
#include "esp_heap_caps.h"
#include "audio_shared.h"

extern TFT_eSPI tft;

// ---- Settings ----
constexpr size_t   MJPEG_BUF_SIZE = 48 * 1024;  // larger than your biggest frame
constexpr uint32_t VIDEO_FPS      = 30;         // target; it just runs as fast as it can
constexpr int      VIDEO_W        = 320;
constexpr int      VIDEO_H        = 240;

constexpr const char* REC_PATH       = "/rec.raw";
constexpr size_t      SB_SIZE        = 128 * 1024;  // ~740 ms of audio buffering (max ~150 KB)
constexpr size_t      IO_CHUNK       = 4096;        // SD read/write size
constexpr uint32_t    REC_BYTES_PER_SEC = 44100 * 2 * 2;
constexpr size_t      BLOCK_BYTES_MIN   = 128 * 2 * 2;   // one audio block (BLOCK_FRAMES * 2 ch * 2 bytes)

// ---- Commands (set from loop(), consumed by the media task) ----
enum MediaCmd : uint8_t { CMD_NONE = 0, CMD_REC_START, CMD_PLAY_START, CMD_STOP };
volatile MediaCmd mediaCmd = CMD_NONE;

enum MediaState : uint8_t { M_VIDEO, M_RECORD, M_PLAY };
static MediaState mediaState = M_VIDEO;

// ---- Video state ----
static File      videoFile;
static JPEGDEC   jpeg;
static uint8_t*  mjpegBuf = nullptr;
static size_t    mjpegLen = 0;
static uint16_t* frameBuf = nullptr;   // VIDEO_W * VIDEO_H RGB565, big-endian

static uint32_t nextFrameAt = 0;
static uint32_t frameCount = 0;
static uint32_t statStart = 0;
static uint32_t usRead = 0, usDecode = 0, usPush = 0;

// ---- Recording / playback state ----
static File      recFile;
static File      playFile;
static uint32_t  recBytes = 0;
static uint32_t  lastRecPrint = 0;
static bool      playEof = false;
static uint8_t   ioChunk[IO_CHUNK];

static StaticStreamBuffer_t sbStruct;
static uint8_t*             sbStorage = nullptr;

// ------------------------------------------------------------------
// Video
// ------------------------------------------------------------------

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
  return 1;
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

static void videoStep() {
  const uint32_t intervalMs = 1000 / VIDEO_FPS;
  size_t frameLen = 0;

  uint32_t t = micros();
  bool ok = loadFrame(frameLen);
  usRead += micros() - t;

  if (!ok) {
    videoFile.seek(0);   // end of file: loop the video
    mjpegLen = 0;
    vTaskDelay(pdMS_TO_TICKS(1));
    return;
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

static void resumeVideo() {
  mediaState = M_VIDEO;
  nextFrameAt = millis();
  statStart = millis();
  frameCount = 0;
  usRead = usDecode = usPush = 0;
}

// ------------------------------------------------------------------
// Stream buffer helpers
// ------------------------------------------------------------------

// The video frame buffer (150 KB) sits idle while recording or playing back,
// so it doubles as the stream buffer's storage: no allocation, no heap
// fragmentation. (A static stream buffer needs size + 1 bytes of storage.)
static bool sbCreate(size_t size, size_t triggerLevel) {
  if (size + 1 > (size_t)VIDEO_W * VIDEO_H * 2) {
    Serial.println("Audio buffer larger than the frame buffer, lower SB_SIZE");
    return false;
  }
  sbStorage = (uint8_t*)frameBuf;
  audioSB = xStreamBufferCreateStatic(size, triggerLevel, sbStorage, &sbStruct);
  if (!audioSB) {
    sbStorage = nullptr;
    return false;
  }
  return true;
}

// Call only after audioMode has been set to AUDIO_IDLE.
static void sbDestroy() {
  vTaskDelay(pdMS_TO_TICKS(20));   // let the audio task finish its current block
  StreamBufferHandle_t h = audioSB;
  audioSB = nullptr;
  if (h) vStreamBufferDelete(h);
  sbStorage = nullptr;   // storage is the video frame buffer, not freed here
}

static void showStatus(const char* text, uint16_t color) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.setTextSize(3);
  tft.setCursor(40, 105);
  tft.print(text);
}

// ------------------------------------------------------------------
// Recording
// ------------------------------------------------------------------

static void startRecording() {
  // Trigger level = IO_CHUNK so the writer wakes for full chunks, not tiny ones.
  if (!sbCreate(SB_SIZE, IO_CHUNK)) return;

  recFile = SD.open(REC_PATH, FILE_WRITE);   // truncates an existing file
  if (!recFile) {
    Serial.printf("Cannot open %s for writing\n", REC_PATH);
    sbDestroy();
    return;
  }

  showStatus("RECORDING", TFT_RED);

  recBytes = 0;
  sbOverruns = 0;
  lastRecPrint = millis();
  audioMode = AUDIO_RECORDING;
  mediaState = M_RECORD;
  Serial.println("Recording started (s to stop)");
}

static void stopRecording() {
  audioMode = AUDIO_IDLE;
  vTaskDelay(pdMS_TO_TICKS(20));   // no more producer after this

  // Write out whatever is still buffered.
  size_t n;
  while ((n = xStreamBufferReceive(audioSB, ioChunk, sizeof(ioChunk), 0)) > 0) {
    recFile.write(ioChunk, n);
    recBytes += n;
  }
  recFile.close();
  sbDestroy();

  Serial.printf("Recording stopped: %lu bytes (%.1f s), %lu dropped blocks\n",
                (unsigned long)recBytes,
                (float)recBytes / REC_BYTES_PER_SEC,
                (unsigned long)sbOverruns);
  if (sbOverruns) {
    Serial.println("Warning: the SD card couldn't keep up, so the recording has gaps.");
  }

  resumeVideo();
}

static void recordStep() {
  size_t n = xStreamBufferReceive(audioSB, ioChunk, sizeof(ioChunk), pdMS_TO_TICKS(50));
  if (n > 0) {
    size_t w = recFile.write(ioChunk, n);
    recBytes += w;
    if (w != n) {
      Serial.println("SD write failed (card full?), stopping");
      stopRecording();
      return;
    }
  }

  if (millis() - lastRecPrint >= 2000) {
    lastRecPrint = millis();
    Serial.printf("REC %.0f s, %lu dropped blocks\n",
                  (float)recBytes / REC_BYTES_PER_SEC,
                  (unsigned long)sbOverruns);
  }
}

// ------------------------------------------------------------------
// Playback
// ------------------------------------------------------------------

// Reads one chunk from the file into the stream buffer if there is room.
static void playFeed() {
  if (playEof) return;
  if (xStreamBufferSpacesAvailable(audioSB) < IO_CHUNK) return;

  int n = playFile.read(ioChunk, IO_CHUNK);
  if (n <= 0) {
    playEof = true;
    return;
  }
  n &= ~3;   // whole stereo frames only
  if (n > 0) xStreamBufferSend(audioSB, ioChunk, n, 0);
}

static void startPlayback() {
  playFile = SD.open(REC_PATH, FILE_READ);
  if (!playFile) {
    Serial.println("No recording found (record one with r)");
    return;
  }

  if (!sbCreate(SB_SIZE, 1)) {
    playFile.close();
    return;
  }

  showStatus("PLAYING", TFT_GREEN);

  // Prefill so playback starts with a full buffer.
  playEof = false;
  while (!playEof && xStreamBufferSpacesAvailable(audioSB) >= IO_CHUNK) {
    playFeed();
  }

  sbUnderruns = 0;
  audioMode = AUDIO_PLAYING;
  mediaState = M_PLAY;
  Serial.println("Playback started (s to stop)");
}

static void stopPlayback() {
  audioMode = AUDIO_IDLE;
  playFile.close();
  sbDestroy();

  Serial.printf("Playback stopped, %lu underrun blocks\n", (unsigned long)sbUnderruns);
  resumeVideo();
}

static void playStep() {
  playFeed();

  // Finished: file read completely and the audio task has drained the buffer.
  // (Less than one block left means it can never be consumed.)
  if (playEof && xStreamBufferBytesAvailable(audioSB) < BLOCK_BYTES_MIN) {
    Serial.println("Playback finished");
    stopPlayback();
    return;
  }

  vTaskDelay(pdMS_TO_TICKS(3));
}

// ------------------------------------------------------------------
// Task
// ------------------------------------------------------------------

static void handleCmd(MediaCmd cmd) {
  switch (cmd) {
    case CMD_REC_START:
      if (mediaState == M_VIDEO) startRecording();
      else Serial.println("Busy: press s to stop first");
      break;

    case CMD_PLAY_START:
      if (mediaState == M_VIDEO) startPlayback();
      else Serial.println("Busy: press s to stop first");
      break;

    case CMD_STOP:
      if (mediaState == M_RECORD) stopRecording();
      else if (mediaState == M_PLAY) stopPlayback();
      break;

    default:
      break;
  }
}

static void mediaTask(void*) {
  resumeVideo();

  for (;;) {
    MediaCmd cmd = mediaCmd;
    if (cmd != CMD_NONE) {
      mediaCmd = CMD_NONE;
      handleCmd(cmd);
    }

    switch (mediaState) {
      case M_VIDEO:  videoStep();  break;
      case M_RECORD: recordStep(); break;
      case M_PLAY:   playStep();   break;
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

  BaseType_t ok = xTaskCreatePinnedToCore(mediaTask, "Media", 10240, nullptr, 1, nullptr, 1);
  if (ok != pdPASS) {
    Serial.println("Video: failed to create task");
    return false;
  }

  Serial.println("Serial commands: r = record, s = stop, l = play, p = passthrough");
  return true;
}

/* ------------------------------------------------------------------
   Sketch changes
   ------------------------------------------------------------------

   Include order at the top of the sketch:

     TFT_eSPI tft = TFT_eSPI();
     #include "mjpeg_video.h"          // after tft
     ...your globals (SAMPLE_RATE, BLOCK_FRAMES, synthMux, txHandle...)
     #include "audio_duplex.h"         // after the globals

   loop():

     void loop() {
       processMidi();

       if (Serial.available()) {
         switch (Serial.read()) {
           case 'r': mediaCmd = CMD_REC_START;  break;
           case 'l': mediaCmd = CMD_PLAY_START; break;
           case 's': mediaCmd = CMD_STOP;       break;
           case 'p':
             passthroughEnabled = !passthroughEnabled;
             Serial.printf("Passthrough %s\n", passthroughEnabled ? "ON" : "OFF");
             break;
         }
       }

       delay(1);
     }

   Play a recording on a computer:
     ffplay -f s16le -ar 44100 -ac 2 rec.raw
   Convert to WAV:
     ffmpeg -f s16le -ar 44100 -ac 2 -i rec.raw rec.wav
   ------------------------------------------------------------------ */
