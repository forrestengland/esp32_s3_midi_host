// audio_duplex.h  (v3)
// Full-duplex I2S: PCM5102 out (TX) + PCM1808 in (RX) on one shared clock.
// Adds: record tap (ADC input -> stream buffer) and playback mix
// (stream buffer -> output) controlled by audioMode from audio_shared.h.
//
// Usage:
//  1. DELETE your existing setupAudio() and audioTask() from the sketch
//     (and any debug min/max code you added).
//  2. Keep your globals (SAMPLE_RATE, BLOCK_FRAMES, synthMux, targetAmplitude,
//     currentAmplitude, phase, phaseIncrement, txHandle, I2S_BCLK/LRCK/DOUT).
//  3. Add  #include "audio_duplex.h"  right after those globals.
//  4. AudioTask stack should be 6144 in setup().

#pragma once

#include "driver/i2s_std.h"
#include "audio_shared.h"

// ---- New pins (change to free GPIOs on your board) ----
constexpr int I2S_DIN  = 2;    // PCM1808 OUT
constexpr int I2S_MCLK = 1;   // PCM1808 SCK

i2s_chan_handle_t rxHandle = nullptr;

// passthroughEnabled (the on/off flag) lives in audio_shared.h.

// 1.0 = input passes at full level. Lower it if the input plus synth clips.
constexpr float PASSTHROUGH_GAIN = 1.0f;

void setupAudio() {
  i2s_chan_config_t chanConfig =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

  // One block per DMA buffer keeps passthrough latency low (~12 ms).
  chanConfig.dma_desc_num  = 4;
  chanConfig.dma_frame_num = BLOCK_FRAMES;

  // Passing both handles creates a full-duplex pair on the same port.
  ESP_ERROR_CHECK(i2s_new_channel(&chanConfig, &txHandle, &rxHandle));

  i2s_std_config_t stdConfig = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),   // MCLK = 256 x fs

      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
          I2S_DATA_BIT_WIDTH_16BIT,
          I2S_SLOT_MODE_STEREO
      ),

      .gpio_cfg = {
          .mclk = (gpio_num_t)I2S_MCLK,
          .bclk = (gpio_num_t)I2S_BCLK,
          .ws   = (gpio_num_t)I2S_LRCK,
          .dout = (gpio_num_t)I2S_DOUT,
          .din  = (gpio_num_t)I2S_DIN,
          .invert_flags = {
              .mclk_inv = false,
              .bclk_inv = false,
              .ws_inv   = false
          }
      }
  };

  // 16-bit samples carried in 32-bit slots -> 64 BCLK per frame (PCM1808 needs this).
  // RX keeps the top 16 bits of the ADC's 24-bit sample.
  stdConfig.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;

  // REQUIRED: the default config sets the word-select width to the 16-bit data
  // width, so with 32-bit slots WS would be high for 16 of 64 clocks (uneven
  // channels). It must be 32 so each channel gets half the frame.
  stdConfig.slot_cfg.ws_width = 32;

  // Same config for both directions, as required for a shared clock.
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(txHandle, &stdConfig));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(rxHandle, &stdConfig));

  ESP_ERROR_CHECK(i2s_channel_enable(txHandle));
  ESP_ERROR_CHECK(i2s_channel_enable(rxHandle));

  Serial.println("I2S duplex audio started.");
}

void audioTask(void* parameter) {
  int16_t outBuf[BLOCK_FRAMES * 2];
  int16_t inBuf[BLOCK_FRAMES * 2];
  int16_t playBuf[BLOCK_FRAMES * 2];

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

    // Synth switched off from the menu: ramp to silence (no clicks).
    if (!synthEnabled) blockTargetAmplitude = 0.0f;

    // Always read the ADC so RX never overruns.
    size_t bytesRead = 0;
    esp_err_t rxResult = i2s_channel_read(
        rxHandle, inBuf, sizeof(inBuf), &bytesRead, 1000);
    bool inputValid = (rxResult == ESP_OK && bytesRead == sizeof(inBuf));
    bool passthrough = passthroughEnabled && inputValid;

    AudioMode mode = audioMode;
    StreamBufferHandle_t sb = audioSB;

    // ---- Recording tap: copy the raw ADC block (whole blocks only) ----
    if (mode == AUDIO_RECORDING && sb && inputValid) {
      if (xStreamBufferSpacesAvailable(sb) >= sizeof(inBuf)) {
        xStreamBufferSend(sb, inBuf, sizeof(inBuf), 0);
      } else {
        sbOverruns++;
      }
    }

    // ---- Playback: fetch one whole block if available ----
    bool havePlay = false;
    if (mode == AUDIO_PLAYING && sb) {
      if (xStreamBufferBytesAvailable(sb) >= sizeof(playBuf)) {
        xStreamBufferReceive(sb, playBuf, sizeof(playBuf), 0);
        havePlay = true;
      } else {
        sbUnderruns++;
      }
    }

    for (int i = 0; i < BLOCK_FRAMES; i++) {
      // ---- synth (unchanged) ----
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

      int32_t synthPcm = (int32_t)(sample * 32767.0f);

      // ---- mix: synth + live input (if enabled) + playback (if playing) ----
      int32_t left  = synthPcm;
      int32_t right = synthPcm;

      if (passthrough) {
        left  += (int32_t)(inBuf[i * 2]     * PASSTHROUGH_GAIN);
        right += (int32_t)(inBuf[i * 2 + 1] * PASSTHROUGH_GAIN);
      }

      if (havePlay) {
        left  += playBuf[i * 2];
        right += playBuf[i * 2 + 1];
      }

      if (left  >  32767) left  =  32767;
      if (left  < -32768) left  = -32768;
      if (right >  32767) right =  32767;
      if (right < -32768) right = -32768;

      outBuf[i * 2]     = (int16_t)left;
      outBuf[i * 2 + 1] = (int16_t)right;
    }

    size_t bytesWritten = 0;

    esp_err_t result = i2s_channel_write(
        txHandle,
        outBuf,
        sizeof(outBuf),
        &bytesWritten,
        1000
    );

    if (result != ESP_OK) {
      Serial.printf("I2S write error: %d\n", result);
    }
  }
}
