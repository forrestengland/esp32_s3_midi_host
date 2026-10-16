// mjpeg_video.h  (v5: media task with menu, MIDI monitor, synth toggle)
// One task owns the TFT and the SD card and runs in one of five states:
//   M_MENU   - menu on screen (default at boot)
//   M_VIDEO  - MJPEG playback
//   M_RECORD - writes ADC audio to /rec.raw
//   M_PLAY   - streams /rec.raw into the audio output
//   M_MIDI   - live MIDI monitor
//
// Encoder events (ui_events.h) drive the menu. Click or long press leaves
// any screen and returns to the menu.
//
// Raw file format: signed 16-bit little-endian, stereo, 44100 Hz.
//
// Requires: JPEGDEC library, audio_shared.h, ui_events.h, midi_log.h.
// Only the media task may touch the TFT or SD after videoBegin().

#pragma once

#include <Arduino.h>
#include <SD.h>
#include <TFT_eSPI.h>
#include <JPEGDEC.h>
#include "esp_heap_caps.h"
#include "audio_shared.h"
#include "ui_events.h"
#include "midi_log.h"

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

// ---- Serial/other commands (set from loop(), consumed by the media task) ----
enum MediaCmd : uint8_t { CMD_NONE = 0, CMD_REC_START, CMD_PLAY_START, CMD_VIDEO_START, CMD_STOP };
volatile MediaCmd mediaCmd = CMD_NONE;

enum MediaState : uint8_t { M_MENU, M_VIDEO, M_RECORD, M_PLAY, M_MIDI };
static MediaState mediaState = M_MENU;

// ---- Menu ----
enum MenuItem : uint8_t {
  MI_VIDEO = 0, MI_RECORD, MI_PLAY, MI_MIDI, MI_PASSTHROUGH, MI_SYNTH, MENU_COUNT
};
static const char* MENU_ITEMS[MENU_COUNT] = {
  "Play video", "Record", "Play recording", "MIDI monitor", "Passthrough", "Synth"
};
constexpr int MENU_TOP   = 40;
constexpr int MENU_ROW_H = 32;

static int  menuIndex = 0;
static bool menuDirty = false;
static bool drawnPass = false;
static bool drawnSynth = true;

// ---- MIDI monitor ----
constexpr int MONITOR_TOP   = 40;
constexpr int MONITOR_ROW_H = 24;
static uint32_t midiDrawnSeq = 0xFFFFFFFF;

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
static uint32_t  lastTimeDraw = 0;
static bool      playEof = false;
static uint8_t   ioChunk[IO_CHUNK];

static StaticStreamBuffer_t sbStruct;
static uint8_t*             sbStorage = nullptr;

// ------------------------------------------------------------------
// Screen helpers (media task only)
// ------------------------------------------------------------------

static void showStatus(const char* text, uint16_t color, const char* hint) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.setTextSize(3);
  tft.setCursor(40, 70);
  tft.print(text);

  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(40, 175);
  tft.print(hint);
}

// total < 0 means "no total".
static void drawTime(uint32_t sec, int32_t total) {
  char buf[28];
  if (total >= 0) {
    snprintf(buf, sizeof(buf), "%lu / %ld s    ", (unsigned long)sec, (long)total);
  } else {
    snprintf(buf, sizeof(buf), "%lu s    ", (unsigned long)sec);
  }
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(40, 125);
  tft.print(buf);
}

// ------------------------------------------------------------------
// Menu
// ------------------------------------------------------------------

static void drawMenuRows() {
  char label[28];
  tft.setTextSize(2);

  for (int i = 0; i < MENU_COUNT; i++) {
    int y = MENU_TOP + i * MENU_ROW_H;
    bool sel = (i == menuIndex);
    uint16_t bg = sel ? TFT_NAVY : TFT_BLACK;
    uint16_t fg = sel ? TFT_WHITE : TFT_LIGHTGREY;

    tft.fillRect(0, y, VIDEO_W, MENU_ROW_H - 3, bg);

    if (i == MI_PASSTHROUGH) {
      snprintf(label, sizeof(label), "Passthrough: %s", passthroughEnabled ? "ON" : "OFF");
    } else if (i == MI_SYNTH) {
      snprintf(label, sizeof(label), "Synth: %s", synthEnabled ? "ON" : "OFF");
    } else {
      snprintf(label, sizeof(label), "%s", MENU_ITEMS[i]);
    }

    tft.setTextColor(fg, bg);
    tft.setCursor(14, y + 7);
    tft.print(label);
  }

  menuDirty = false;
  drawnPass = passthroughEnabled;
  drawnSynth = synthEnabled;
}

static void enterMenu() {
  mediaState = M_MENU;

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(3);
  tft.setCursor(14, 8);
  tft.print("MENU");

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(100, 16);
  tft.print("turn: select  click: go  hold: back");

  drawMenuRows();
}

// ------------------------------------------------------------------
// MIDI monitor
// ------------------------------------------------------------------

static void drawMidiRows() {
  MidiLogEntry snap[MIDI_LOG_LINES];
  uint32_t seq = midiLogSnapshot(snap);
  int count = (seq < (uint32_t)MIDI_LOG_LINES) ? (int)seq : MIDI_LOG_LINES;

  // Total event counter, top right.
  char buf[16];
  snprintf(buf, sizeof(buf), "%lu   ", (unsigned long)seq);
  tft.setTextSize(2);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(230, 10);
  tft.print(buf);

  for (int i = 0; i < MIDI_LOG_LINES; i++) {
    int y = MONITOR_TOP + i * MONITOR_ROW_H;
    tft.fillRect(0, y, VIDEO_W, MONITOR_ROW_H, TFT_BLACK);
    if (i < count) {
      tft.setTextColor(snap[i].color, TFT_BLACK);
      tft.setCursor(10, y + 4);
      tft.print(snap[i].text);
    }
  }

  if (count == 0) {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(10, MONITOR_TOP + 4);
    tft.print("Waiting for MIDI...");
  }

  midiDrawnSeq = seq;
}

static void enterMidiMonitor() {
  mediaState = M_MIDI;

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(14, 10);
  tft.print("MIDI MONITOR");

  drawMidiRows();
}

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

static void startVideo() {
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

  showStatus("RECORDING", TFT_RED, "Click to stop");
  drawTime(0, -1);

  recBytes = 0;
  sbOverruns = 0;
  lastRecPrint = millis();
  lastTimeDraw = millis();
  audioMode = AUDIO_RECORDING;
  mediaState = M_RECORD;
  Serial.println("Recording started");
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

  enterMenu();
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

  uint32_t now = millis();

  if (now - lastTimeDraw >= 1000) {
    lastTimeDraw = now;
    drawTime(recBytes / REC_BYTES_PER_SEC, -1);
  }

  if (now - lastRecPrint >= 2000) {
    lastRecPrint = now;
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
    Serial.println("No recording found (record one first)");
    return;
  }

  if (!sbCreate(SB_SIZE, 1)) {
    playFile.close();
    return;
  }

  showStatus("PLAYING", TFT_GREEN, "Click to stop");
  drawTime(0, (int32_t)(playFile.size() / REC_BYTES_PER_SEC));

  // Prefill so playback starts with a full buffer.
  playEof = false;
  while (!playEof && xStreamBufferSpacesAvailable(audioSB) >= IO_CHUNK) {
    playFeed();
  }

  sbUnderruns = 0;
  lastTimeDraw = millis();
  audioMode = AUDIO_PLAYING;
  mediaState = M_PLAY;
  Serial.println("Playback started");
}

static void stopPlayback() {
  audioMode = AUDIO_IDLE;
  playFile.close();
  sbDestroy();

  Serial.printf("Playback stopped, %lu underrun blocks\n", (unsigned long)sbUnderruns);
  enterMenu();
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

  uint32_t now = millis();
  if (now - lastTimeDraw >= 1000) {
    lastTimeDraw = now;
    drawTime(playFile.position() / REC_BYTES_PER_SEC,
             (int32_t)(playFile.size() / REC_BYTES_PER_SEC));
  }

  vTaskDelay(pdMS_TO_TICKS(3));
}

// ------------------------------------------------------------------
// Input handling
// ------------------------------------------------------------------

static void menuSelect() {
  switch (menuIndex) {
    case MI_VIDEO:       startVideo();        break;
    case MI_RECORD:      startRecording();    break;
    case MI_PLAY:        startPlayback();     break;
    case MI_MIDI:        enterMidiMonitor();  break;
    case MI_PASSTHROUGH:
      passthroughEnabled = !passthroughEnabled;
      menuDirty = true;
      break;
    case MI_SYNTH:
      synthEnabled = !synthEnabled;
      menuDirty = true;
      break;
  }
}

static void handleUi(UiEvent ev) {
  switch (mediaState) {
    case M_MENU:
      if (ev == UI_UP) {
        menuIndex = (menuIndex + MENU_COUNT - 1) % MENU_COUNT;
        menuDirty = true;
      } else if (ev == UI_DOWN) {
        menuIndex = (menuIndex + 1) % MENU_COUNT;
        menuDirty = true;
      } else if (ev == UI_CLICK) {
        menuSelect();
      }
      break;

    case M_VIDEO:
    case M_MIDI:
      if (ev == UI_CLICK || ev == UI_LONG) enterMenu();
      break;

    case M_RECORD:
      if (ev == UI_CLICK || ev == UI_LONG) stopRecording();
      break;

    case M_PLAY:
      if (ev == UI_CLICK || ev == UI_LONG) stopPlayback();
      break;
  }
}

// Serial commands: usable in addition to the encoder.
static void handleCmd(MediaCmd cmd) {
  bool idle = (mediaState == M_MENU || mediaState == M_VIDEO || mediaState == M_MIDI);

  switch (cmd) {
    case CMD_REC_START:
      if (idle) startRecording(); else Serial.println("Busy: press s to stop first");
      break;

    case CMD_PLAY_START:
      if (idle) startPlayback(); else Serial.println("Busy: press s to stop first");
      break;

    case CMD_VIDEO_START:
      if (idle) startVideo(); else Serial.println("Busy: press s to stop first");
      break;

    case CMD_STOP:
      if (mediaState == M_RECORD) stopRecording();
      else if (mediaState == M_PLAY) stopPlayback();
      else if (mediaState == M_VIDEO || mediaState == M_MIDI) enterMenu();
      break;

    default:
      break;
  }
}

// ------------------------------------------------------------------
// Task
// ------------------------------------------------------------------

static void mediaTask(void*) {
  enterMenu();

  for (;;) {
    MediaCmd cmd = mediaCmd;
    if (cmd != CMD_NONE) {
      mediaCmd = CMD_NONE;
      handleCmd(cmd);
    }

    UiEvent ev;
    while (uiReceive(ev)) {
      handleUi(ev);
    }

    switch (mediaState) {
      case M_MENU:
        // Redraw once per batch of events (and if a flag changed via serial).
        if (menuDirty || drawnPass != passthroughEnabled || drawnSynth != synthEnabled) {
          drawMenuRows();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        break;
      case M_VIDEO:  videoStep();  break;
      case M_RECORD: recordStep(); break;
      case M_PLAY:   playStep();   break;
      case M_MIDI:
        if (midiLogSeq != midiDrawnSeq) drawMidiRows();
        vTaskDelay(pdMS_TO_TICKS(15));
        break;
    }
  }
}

// Call from setup() AFTER tft.init() and SD.begin() have succeeded.
bool videoBegin(const char* path) {
  uiEventsInit();

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

  Serial.println("Serial commands: r = record, l = play, v = video, s = stop/menu, p = passthrough");
  return true;
}
