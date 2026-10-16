// ui_events.h
// Input events from the rotary encoder (produced in loop(), consumed by the
// media task, which owns the display).

#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

enum UiEvent : uint8_t {
  UI_UP = 1,    // encoder turned one detent counter-clockwise
  UI_DOWN,      // encoder turned one detent clockwise
  UI_CLICK,     // short press
  UI_LONG       // long press
};

static QueueHandle_t uiQueue = nullptr;

// Safe to call more than once.
static void uiEventsInit() {
  if (!uiQueue) uiQueue = xQueueCreate(32, sizeof(uint8_t));
}

static void uiSend(UiEvent ev) {
  if (!uiQueue) return;
  uint8_t e = (uint8_t)ev;
  xQueueSend(uiQueue, &e, 0);   // drop the event if the queue is full
}

static bool uiReceive(UiEvent& ev) {
  if (!uiQueue) return false;
  uint8_t e;
  if (xQueueReceive(uiQueue, &e, 0) == pdTRUE) {
    ev = (UiEvent)e;
    return true;
  }
  return false;
}
