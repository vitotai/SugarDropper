#include <Arduino.h>
#include "SugarDoser.h"

ApeController sugarDaddy;

void setup() {
  // put your setup code here, to run once:
#if DEBUG_OUT
  Serial.begin(115200);
#endif
  sugarDaddy.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  sugarDaddy.loop();
}