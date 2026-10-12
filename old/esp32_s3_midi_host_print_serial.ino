
#include <ESP32_Host_MIDI.h>
#include <USBConnection.h>

USBConnection usbHost;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("ESP32-S3 USB MIDI Host");
  Serial.println("Waiting for MIDI keyboard...");

  midiHandler.addTransport(&usbHost);
  usbHost.begin();
  midiHandler.begin();
}


static uint64_t lastEventIndex = 0;

void loop() {
  midiHandler.task();

  const auto& queue = midiHandler.getQueue();

  for (const auto& ev : queue) {

    // Skip events we've already processed
    if (ev.index <= lastEventIndex) {
      continue;
    }
    lastEventIndex = ev.index;

    uint8_t status = ev.statusCode & 0xF0;

    // Ignore pitch bend and control change
    /* if (status == 0xE0 || status == 0xB0) {
      continue;
			} */

    // Only print note on/off events
		/*    if (status != 0x90 && status != 0x80) {
      continue;
			} */

    char noteName[8];

    Serial.printf(
      "%-12s %-4s channel=%d velocity=%d\n",
      MIDIHandler::statusName(ev.statusCode),
      MIDIHandler::noteWithOctave(
        ev.noteNumber, noteName, sizeof(noteName)
      ),
      ev.channel0 + 1,
      ev.velocity7
    );
  }
}
