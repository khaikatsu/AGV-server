#include <Arduino.h>
#include "core/AGVController.h"

namespace { AGVController agv; }

void setup() {
    Serial.begin(Config::SERIAL_BAUD);
    agv.begin();
}
void loop() {
    agv.update();
    yield();
}
