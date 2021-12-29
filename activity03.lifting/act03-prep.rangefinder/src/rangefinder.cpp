#include <Arduino.h>
#include <wpi-32u4-lib.h>
#include <Rangefinder.h>
#include <IRdecoder.h>
#include <ir_codes.h>

Rangefinder rangefinder(11, 4);

// Setup the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);

void setup() 
{
  // This will initialize the Serial as 115200 for prints
  Serial.begin(115200);

  // Call init() to set up the rangefinder
  rangefinder.init();

  //init the decoder
  decoder.init();
}

void loop() 
{
  int16_t keyPress = decoder.getKeyCode();

  if(keyPress == ENTER_SAVE){
    for(int i = 0; i < 50; i++){
      float distance = rangefinder.getDistance();

      // We add a short delay to make the output more legible -- you wouldn't want this in most code
      delay(50);

      Serial.print(millis());
      Serial.print('\t');
      Serial.print(distance); 
      Serial.print(" cm\n");
    }
  }

}
