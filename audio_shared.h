// audio_shared.h
// State shared between the audio task (core 0) and the media task (core 1).
// Included by both audio_duplex.h and mjpeg_video.h.

#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"

enum AudioMode : uint8_t {
  AUDIO_IDLE = 0,
  AUDIO_RECORDING,   // audio task copies ADC input blocks into audioSB
  AUDIO_PLAYING      // audio task mixes blocks from audioSB into the output
};

// Written by the media task, read by the audio task.
volatile AudioMode audioMode = AUDIO_IDLE;

// Created when recording/playback starts, deleted when it ends.
// The audio task only touches it while audioMode is not AUDIO_IDLE.
volatile StreamBufferHandle_t audioSB = nullptr;

volatile uint32_t sbOverruns  = 0;   // recording: blocks dropped (writer fell behind)
volatile uint32_t sbUnderruns = 0;   // playback: blocks missing (reader fell behind)
