#include <Arduino.h>
#include "FastLED.h"


#define NUM_LEDS      216
#define LED_TYPE   WS2813
#define COLOR_ORDER   RGB
#define DATA_PIN        21
//#define CLK_PIN       4
#define VOLTS          12
#define MAX_MA       4000


CRGBArray<NUM_LEDS> leds;

void setup() {
   FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
}

void loop() {
   for(int i = 0; i < NUM_LEDS; i++){
      leds[i] = CRGB::Purple;
   }
   FastLED.setBrightness(255);
   FastLED.show();
}